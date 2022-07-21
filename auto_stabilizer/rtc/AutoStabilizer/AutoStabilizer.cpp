#include "AutoStabilizer.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/ValueTree>
#include <cnoid/EigenUtil>
#include "MathUtil.h"
#include "FootGuidedController.h"
#include "LegCoordsGenerator.h"
#include "FootStepGenerator.h"
#include <limits>

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* AutoStabilizer_spec[] = {
  "implementation_id", "AutoStabilizer",
  "type_name",         "AutoStabilizer",
  "description",       "AutoStabilizer component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

AutoStabilizer::Ports::Ports() :
  m_qRefIn_("qRef", m_qRef_),
  m_refTauIn_("refTauIn", m_refTau_),
  m_refBasePosIn_("refBasePosIn", m_refBasePos_),
  m_refBaseRpyIn_("refBaseRpyIn", m_refBaseRpy_),
  m_qActIn_("qAct", m_qAct_),
  m_dqActIn_("dqAct", m_dqAct_),
  m_actImuIn_("actImuIn", m_actImu_),

  m_qOut_("q", m_q_),
  m_genTauOut_("genTauOut", m_genTau_),
  m_genBasePoseOut_("genBasePoseOut", m_genBasePose_),
  m_genBaseTformOut_("genBaseTformOut", m_genBaseTform_),

  m_genBasePosOut_("genBasePosOut", m_genBasePos_),
  m_genBaseRpyOut_("genBaseRpyOut", m_genBaseRpy_),

  m_AutoStabilizerServicePort_("AutoStabilizerService") {
}

AutoStabilizer::AutoStabilizer(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  debugLevel_(0)
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t AutoStabilizer::onInitialize(){

  // add ports
  this->addInPort("qRef", this->ports_.m_qRefIn_);
  this->addInPort("refTauIn", this->ports_.m_refTauIn_);
  this->addInPort("refBasePosIn", this->ports_.m_refBasePosIn_);
  this->addInPort("refBaseRpyIn", this->ports_.m_refBaseRpyIn_);
  this->addInPort("qAct", this->ports_.m_qActIn_);
  this->addInPort("dqAct", this->ports_.m_dqActIn_);
  this->addInPort("actImuIn", this->ports_.m_actImuIn_);
  this->addOutPort("q", this->ports_.m_qOut_);
  this->addOutPort("genTauOut", this->ports_.m_genTauOut_);
  this->addOutPort("genBasePoseOut", this->ports_.m_genBasePoseOut_);
  this->addOutPort("genBaseTformOut", this->ports_.m_genBaseTformOut_);
  this->addOutPort("genBasePosOut", this->ports_.m_genBasePosOut_);
  this->addOutPort("genBaseRpyOut", this->ports_.m_genBaseRpyOut_);
  this->ports_.m_AutoStabilizerServicePort_.registerProvider("service0", "AutoStabilizerService", this->ports_.m_service0_);
  this->addPort(this->ports_.m_AutoStabilizerServicePort_);

  {
    // load dt
    std::string buf; this->getProperty("dt", buf);
    this->dt_ = std::stod(buf);
    if(this->dt_ <= 0.0){
      this->getProperty("exec_cxt.periodic.rate", buf);
      double rate = std::stod(buf);
      if(rate > 0.0){
        this->dt_ = 1.0/rate;
      }else{
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "dt is invalid" << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
    }
  }

  {
    // load robot model
    cnoid::BodyLoader bodyLoader;
    std::string fileName; this->getProperty("model", fileName);
    if (fileName.find("file://") == 0) fileName.erase(0, strlen("file://"));
    cnoid::BodyPtr robot = bodyLoader.load(fileName);
    if(!robot){
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
    this->refRobot_ = robot;
    this->refRobot_->calcForwardKinematics(); this->refRobot_->calcCenterOfMass();
    this->refRobotOrigin_ = robot;
    this->refRobotOrigin_->calcForwardKinematics(); this->refRobotOrigin_->calcCenterOfMass();
    this->actRobot_ = robot->clone();
    this->actRobot_->calcForwardKinematics(); this->actRobot_->calcCenterOfMass();
    this->actRobotOrigin_ = robot->clone();
    this->actRobotOrigin_->calcForwardKinematics(); this->actRobotOrigin_->calcCenterOfMass();
    this->genRobot_ = robot->clone();
    this->genRobot_->calcForwardKinematics(); this->genRobot_->calcCenterOfMass();
  }

  {
    // load end_effector
    std::string endEffectors; this->getProperty("end_effectors", endEffectors);
    std::stringstream ss_endEffectors(endEffectors);
    std::string buf;
    while(std::getline(ss_endEffectors, buf, ',')){
      std::string name;
      std::string parentLink;
      cnoid::Vector3 localp;
      cnoid::Vector3 localaxis;
      double localangle;

      //   name, parentLink, (not used), x, y, z, theta, ax, ay, az
      name = buf;
      if(!std::getline(ss_endEffectors, buf, ',')) break; parentLink = buf;
      if(!std::getline(ss_endEffectors, buf, ',')) break; // not used
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localangle = std::stod(buf);

      // check validity
      name.erase(std::remove(name.begin(), name.end(), ' '), name.end()); // remove whitespace
      parentLink.erase(std::remove(name.begin(), name.end(), ' '), name.end()); // remove whitespace
      if(!this->refRobot_->link(parentLink)){
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " link [" << parentLink << "]" << " is not found for " << name << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
      cnoid::Matrix3 localR;
      if(localaxis.norm() == 0) localR = cnoid::Matrix3::Identity();
      else localR = Eigen::AngleAxisd(localangle, localaxis.normalized()).toRotationMatrix();

      EndEffectorParam ikp;
      ikp.name = name;
      ikp.parentLink = parentLink;
      ikp.localT.translation() = localp;
      ikp.localT.linear() = localR;
      this->endEffectorParams_.push_back(ikp);
    }
  }

  {
    //add more information to EndEffectors

    // 0番目が右脚. 1番目が左脚. という仮定がある.
    if(this->endEffectorParams_.size() < NUM_LEGS || this->endEffectorParams_[RLEG].name != "rleg" || this->endEffectorParams_[LLEG].name != "lleg"){
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " this->endEffectorParams_.size() < 2 || this->endEffectorParams_[0].name != \"rleg\" || this->endEffectorParams_[1].name != \"lleg\" not holds" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }

    // 各EndEffectorsから親リンク側に遡っていき、最初に見つかったForceSensorをEndEffectorに対応付ける. 以後、ForceSensorの値を座標変換したものがEndEffectorが受けている力とみなされる. 見つからなければ受けている力は常に0とみなされる
    cnoid::DeviceList<cnoid::ForceSensor> forceSensors(this->actRobot_->devices());
    for (int i=0;i<this->endEffectorParams_.size();i++){
      cnoid::LinkPtr link = this->actRobot_->link(this->endEffectorParams_[i].parentLink);
      bool found = false;
      while (link != nullptr && found == false) {
        for (size_t j = 0; j < forceSensors.size(); j++) {
          if(forceSensors[j]->link() == link) {
            this->endEffectorParams_[i].forceSensor = forceSensors[j]->name();
            found = true;
            break;
          }
        }
      }
    }
  }

  {
    // generate LegParams
    cnoid::Position defautFootMidCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{cnoid::Position(this->refRobotOrigin_->link(this->endEffectorParams_[RLEG].parentLink)->T()*this->endEffectorParams_[RLEG].localT),cnoid::Position(this->refRobotOrigin_->link(this->endEffectorParams_[LLEG].parentLink)->T()*this->endEffectorParams_[LLEG].localT)},
                                                            std::vector<double>{1,1});
    for(int i=0; i<NUM_LEGS; i++){
      LegParam param;
      param.name = this->endEffectorParams_[i].name;
      cnoid::Position defaultPose = this->refRobotOrigin_->link(this->endEffectorParams_[i].parentLink)->T()*this->endEffectorParams_[i].localT;
      param.defaultTranslatePos = defautFootMidCoords.inverse() * defaultPose.translation();
      this->legParams_.push_back(param);
    }
  }

  {
    // generate JointParams
    for(int i=0;i<this->genRobot_->numJoints();i++){
      cnoid::LinkPtr joint = this->genRobot_->joint(i);
      JointParam param;
      param.name = joint->name();
      double climit = 0.0, gearRatio = 0.0, torqueConst = 0.0;
      joint->info()->read("climit",climit); joint->info()->read("gearRatio",gearRatio); joint->info()->read("torqueConst",torqueConst);
      param.maxTorque = climit * gearRatio * torqueConst;
      param.controllable = true;
      this->jointParams_.push_back(param);
    }

    // apply margin to jointlimit
    for(int i=0;i<this->genRobot_->numJoints();i++){
      cnoid::LinkPtr joint = this->genRobot_->joint(i);
      if(joint->q_upper() - joint->q_lower() > 0.002){
        joint->setJointRange(joint->q_lower()+0.001,joint->q_upper()-0.001);
      }
      // JointVelocityについて. 1.0だと安全.4.0は脚.10.0はlapid manipulation らしい. limitを小さくしすぎた状態で、速い指令を送ると、狭いlimitの中で高優先度タスクを頑張って満たそうとすることで、低優先度タスクを満たす余裕がなくエラーが大きくなってしまうことに注意.
      if(joint->dq_upper() - joint->dq_lower() > 0.02){
        joint->setJointVelocityRange(joint->dq_lower()+0.01,joint->dq_upper()-0.01);
      }
    }
  }

  {
    // add more ports (ロボットモデルやEndEffectorの情報を使って)

    // 各EndEffectorにつき、ref<name>WrenchInというInPortをつくる
    this->ports_.m_refWrenchIn_.resize(this->endEffectorParams_.size());
    this->ports_.m_refWrench_.resize(this->endEffectorParams_.size());
    for(int i=0;i<this->endEffectorParams_.size();i++){
      std::string name = "ref"+this->endEffectorParams_[i].name+"WrenchIn";
      this->ports_.m_refWrenchIn_[i] = std::make_unique<RTC::InPort<RTC::TimedDoubleSeq> >(name.c_str(), this->ports_.m_refWrench_[i]);
      this->addInPort(name.c_str(), *(this->ports_.m_refWrenchIn_[i]));
    }

    // 各ForceSensorにつき、act<name>InというInportをつくる
    cnoid::DeviceList<cnoid::ForceSensor> forceSensors(this->actRobot_->devices());
    this->ports_.m_actWrenchIn_.resize(forceSensors.size());
    this->ports_.m_actWrench_.resize(forceSensors.size());
    for(int i=0;i<forceSensors.size();i++){
      std::string name = "act"+forceSensors[i]->name()+"In";
      this->ports_.m_actWrenchIn_[i] = std::make_unique<RTC::InPort<RTC::TimedDoubleSeq> >(name.c_str(), this->ports_.m_actWrench_[i]);
      this->addInPort(name.c_str(), *(this->ports_.m_actWrenchIn_[i]));
    }
  }

  // initialize parameters
  this->loop_ = 0;



  // std::string jointLimitTableStr;
  // if(this->getProperties().hasKey("joint_limit_table")) jointLimitTableStr = std::string(this->getProperties()["joint_limit_table"]);
  // else jointLimitTableStr = std::string(this->m_pManager->getConfig()["joint_limit_table"]); // 引数 -o で与えたプロパティを捕捉
  // std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > jointLimitTables = joint_limit_table::readJointLimitTablesFromProperty (this->robot_com_, jointLimitTableStr);
  // std::cerr << "[" << this->m_profile.instance_name << "] joint_limit_table: " << jointLimitTableStr<<std::endl;
  // for(size_t i=0;i<jointLimitTables.size();i++){
  //   // apply margin
  //   for(size_t j=0;j<jointLimitTables[i]->lLimitTable().size();j++){
  //     if(jointLimitTables[i]->uLimitTable()[j] - jointLimitTables[i]->lLimitTable()[j] > 0.002){
  //       jointLimitTables[i]->uLimitTable()[j] -= 0.001;
  //       jointLimitTables[i]->lLimitTable()[j] += 0.001;
  //     }
  //   }
  //   this->jointLimitTablesMap_[jointLimitTables[i]->getSelfJoint()].push_back(jointLimitTables[i]);
  // }

  return RTC::RTC_OK;
}

// static function
bool AutoStabilizer::readInPortData(AutoStabilizer::Ports& ports, cnoid::BodyPtr refRobot, cnoid::BodyPtr actRobot, std::vector<AutoStabilizer::EndEffectorParam>& endEffectors){
  bool refRobot_changed = false;
  bool qRef_updated = false;
  if(ports.m_qRefIn_.isNew()){
    ports.m_qRefIn_.read();
    if(ports.m_qRef_.data.length() == refRobot->numJoints()){
      for(int i=0;i<ports.m_qRef_.data.length();i++){
        if(std::isfinite(ports.m_qRef_.data[i])) refRobot->joint(i)->q() = ports.m_qRef_.data[i];
      }
      refRobot_changed = true;
      qRef_updated = true;
    }
  }
  if(ports.m_refTauIn_.isNew()){
    ports.m_refTauIn_.read();
    if(ports.m_refTau_.data.length() == refRobot->numJoints()){
      for(int i=0;i<ports.m_refTau_.data.length();i++){
        if(std::isfinite(ports.m_refTau_.data[i])) refRobot->joint(i)->u() = ports.m_refTau_.data[i];
      }
    }
  }
  if(ports.m_refBasePosIn_.isNew()){
    ports.m_refBasePosIn_.read();
    if(std::isfinite(ports.m_refBasePos_.data.x) && std::isfinite(ports.m_refBasePos_.data.y) && std::isfinite(ports.m_refBasePos_.data.z)){
      refRobot->rootLink()->p()[0] = ports.m_refBasePos_.data.x;
      refRobot->rootLink()->p()[1] = ports.m_refBasePos_.data.y;
      refRobot->rootLink()->p()[2] = ports.m_refBasePos_.data.z;
    }
    refRobot_changed = true;
  }
  if(ports.m_refBaseRpyIn_.isNew()){
    ports.m_refBaseRpyIn_.read();
    if(std::isfinite(ports.m_refBaseRpy_.data.r) && std::isfinite(ports.m_refBaseRpy_.data.p) && std::isfinite(ports.m_refBaseRpy_.data.y)){
      refRobot->rootLink()->R() = cnoid::rotFromRpy(ports.m_refBaseRpy_.data.r, ports.m_refBaseRpy_.data.p, ports.m_refBaseRpy_.data.y);
    }
    refRobot_changed = true;
  }
  if(refRobot_changed){
    refRobot->calcForwardKinematics();
    refRobot->calcCenterOfMass();
  }

  for(int i=0;i<ports.m_refWrenchIn_.size();i++){
    if(ports.m_refWrenchIn_[i]->isNew()){
      ports.m_refWrenchIn_[i]->read();
      if(ports.m_refWrench_[i].data.length() == 6){
        for(int j=0;j<6;j++){
          if(std::isfinite(ports.m_refWrench_[i].data[j])) endEffectors[i].refWrenchOrigin[j] = ports.m_refWrench_[i].data[j];
        }
      }
    }
  }

  bool actRobot_changed = false;
  if(ports.m_qActIn_.isNew()){
    ports.m_qActIn_.read();
    if(ports.m_qAct_.data.length() == actRobot->numJoints()){
      for(int i=0;i<ports.m_qAct_.data.length();i++){
        if(std::isfinite(ports.m_qAct_.data[i])) actRobot->joint(i)->q() = ports.m_qAct_.data[i];
      }
      actRobot_changed = true;
    }
  }
  if(ports.m_dqActIn_.isNew()){
    ports.m_dqActIn_.read();
    if(ports.m_dqAct_.data.length() == actRobot->numJoints()){
      for(int i=0;i<ports.m_dqAct_.data.length();i++){
        if(std::isfinite(ports.m_dqAct_.data[i])) actRobot->joint(i)->dq() = ports.m_dqAct_.data[i];
      }
      actRobot_changed = true;
    }
  }
  if(ports.m_actImuIn_.isNew()){
    ports.m_actImuIn_.read();
    if(std::isfinite(ports.m_actImu_.data.r) && std::isfinite(ports.m_actImu_.data.p) && std::isfinite(ports.m_actImu_.data.y)){
      actRobot->calcForwardKinematics();
      cnoid::RateGyroSensorPtr imu = actRobot->findDevice<cnoid::RateGyroSensor>("gyrometer");
      cnoid::Matrix3 imuR = imu->link()->R() * imu->R_local();
      cnoid::Matrix3 actR = cnoid::rotFromRpy(ports.m_actImu_.data.r, ports.m_actImu_.data.p, ports.m_actImu_.data.y);
      actRobot->rootLink()->R() = Eigen::Matrix3d(Eigen::AngleAxisd(actR) * Eigen::AngleAxisd(imuR.transpose() * actRobot->rootLink()->R())); // 単純に3x3行列の空間でRを積算していると、だんだん数値誤差によって回転行列でなくなってしまう恐れがあるので念の為
    }
    actRobot_changed = true;
  }
  if(actRobot_changed){
    actRobot->calcForwardKinematics();
    actRobot->calcCenterOfMass();
  }

  cnoid::DeviceList<cnoid::ForceSensor> forceSensors(actRobot->devices());
  for(int i=0;i<ports.m_actWrenchIn_.size();i++){
    if(ports.m_actWrenchIn_[i]->isNew()){
      ports.m_actWrenchIn_[i]->read();
      if(ports.m_actWrench_[i].data.length() == 6){
        for(int j=0;j<6;j++){
          if(std::isfinite(ports.m_refWrench_[i].data[j])) forceSensors[i]->F()[j] = ports.m_actWrench_[i].data[j];
        }
      }
    }
  }


  return qRef_updated;
}

// static function
bool AutoStabilizer::copyRobotState(cnoid::BodyPtr inRobot, cnoid::BodyPtr outRobot) {
  outRobot->rootLink()->T() = inRobot->rootLink()->T();
  outRobot->rootLink()->v() = inRobot->rootLink()->v();
  outRobot->rootLink()->w() = inRobot->rootLink()->w();
  outRobot->rootLink()->dv() = inRobot->rootLink()->dv();
  outRobot->rootLink()->dw() = inRobot->rootLink()->dw();
  for(int i=0;i<outRobot->numJoints();i++){
    outRobot->joint(i)->q() = inRobot->joint(i)->q();
    outRobot->joint(i)->dq() = inRobot->joint(i)->dq();
    outRobot->joint(i)->ddq() = inRobot->joint(i)->ddq();
    outRobot->joint(i)->u() = inRobot->joint(i)->u();
  }
  outRobot->calcForwardKinematics();
  outRobot->calcCenterOfMass();
  return true;
}

// static function
cnoid::Position AutoStabilizer::calcRefFootMidCoords(const cnoid::BodyPtr robot, const std::vector<AutoStabilizer::LegParam>& legParams, const std::vector<AutoStabilizer::EndEffectorParam>& endEffectorParams){
  cnoid::Position rleg = robot->link(endEffectorParams[RLEG].parentLink)->T()*endEffectorParams[RLEG].localT;
  cnoid::Position lleg = robot->link(endEffectorParams[LLEG].parentLink)->T()*endEffectorParams[LLEG].localT;

  cnoid::Position bothmidcoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{rleg, lleg},
                                                          std::vector<double>{1.0, 1.0});
  cnoid::Position rlegmidcoords = rleg; rlegmidcoords.translation() -= rlegmidcoords.linear() * legParams[RLEG].defaultTranslatePos;
  cnoid::Position llegmidcoords = lleg; llegmidcoords.translation() -= llegmidcoords.linear() * legParams[LLEG].defaultTranslatePos;

  double bothweight = std::min(legParams[RLEG].refFootOriginWeight, legParams[LLEG].refFootOriginWeight);
  double rlegweight = legParams[RLEG].refFootOriginWeight - bothweight;
  double llegweight = legParams[LLEG].refFootOriginWeight - bothweight;
  return mathutil::calcMidCoords(std::vector<cnoid::Position>{bothmidcoords, rlegmidcoords, llegmidcoords},
                                 std::vector<double>{bothweight, rlegweight, llegweight});
}

// static function
void AutoStabilizer::moveCoords(cnoid::BodyPtr robot, const cnoid::Position& target, const cnoid::Position& at){
  // fix ’at’ coords on ’robot’ to ’target’
  cnoid::Position transform = target * at.inverse();
  robot->rootLink()->T() = transform * robot->rootLink()->T();
}

// static function
bool AutoStabilizer::calcReferenceParameters(const AutoStabilizer::ControlMode& mode, const cnoid::BodyPtr& refRobot, cnoid::BodyPtr& refRobotOrigin, cnoid::BodyPtr& genRobot, std::vector<AutoStabilizer::LegParam>& legParams, std::vector<AutoStabilizer::EndEffectorParam>& endEffectorParams, AutoStabilizer::FullbodyState& fullbodyState) {
  // FootOrigin座標系を用いてrefRobotをgenerate frameに投影しrefRobotOriginとする
  {
    if(mode.isABCInit()){ // startAutoBalancer直後の初回
      // generate frame中のfootMidCoordsの位置がreference frame中のrefRobotのfootMidCoordsの位置と同じで、generate frame中のfootMidCoordsの傾きが水平になるように、refRobotOriginの初期位置を決める
      refRobotOrigin->rootLink()->T() = refRobot->rootLink()->T();
      for(int i=0;i<refRobotOrigin->numJoints();i++){
        refRobotOrigin->joint(i)->q() = refRobot->joint(i)->q();
      }
      refRobotOrigin->calcForwardKinematics();
      cnoid::Position refFootMidCoords = AutoStabilizer::calcRefFootMidCoords(refRobotOrigin, legParams, endEffectorParams);
      fullbodyState.footMidCoords.reset(mathutil::orientCoordToAxis(refFootMidCoords, cnoid::Vector3::UnitZ())); // 初期化
      AutoStabilizer::moveCoords(refRobotOrigin, fullbodyState.footMidCoords.value(), refFootMidCoords);
      for(int i=0;i<refRobotOrigin->numJoints();i++){
        refRobotOrigin->joint(i)->q() = refRobot->joint(i)->q();
      }
      refRobotOrigin->calcForwardKinematics();
      for(int i=0; i<endEffectorParams.size(); i++){
        endEffectorParams[i].abcTargetPose = refRobotOrigin->link(endEffectorParams[i].parentLink)->T() * endEffectorParams[i].localT; // 初期化
      }

    } // if(mode.isABCInit())
  }

  {
    for(int i=0;i<refRobotOrigin->numJoints();i++){
      refRobotOrigin->joint(i)->q() = refRobot->joint(i)->q();
      refRobotOrigin->joint(i)->u() = refRobot->joint(i)->u();
    }
    refRobotOrigin->calcForwardKinematics();
    cnoid::Position refFootMidCoords = AutoStabilizer::calcRefFootMidCoords(refRobotOrigin, legParams, endEffectorParams);
    AutoStabilizer::moveCoords(refRobotOrigin, fullbodyState.footMidCoords.value(), refFootMidCoords); // 1周期前のfullbodyState.footMidCoordsを使っているが、fullbodyState.footMidCoordsは不連続に変化するものではないのでよい
    refRobotOrigin->calcForwardKinematics();
    refRobotOrigin->calcCenterOfMass();
  }

  {
    // 各エンドエフェクタのreferenceの位置・力を計算
    for(int i=0;i<endEffectorParams.size(); i++){
      endEffectorParams[i].refPose = refRobotOrigin->link(endEffectorParams[i].parentLink)->T() * endEffectorParams[i].localT;
      endEffectorParams[i].refWrench.head<3>() = fullbodyState.footMidCoords.value().linear() * endEffectorParams[i].refWrenchOrigin.head<3>();
      endEffectorParams[i].refWrench.tail<3>() = fullbodyState.footMidCoords.value().linear() * endEffectorParams[i].refWrenchOrigin.tail<3>();
    }
  }

  if(mode.isABCInit()){ // startAutoBalancer直後の初回
    // genRobotの姿勢を初期化する
    AutoStabilizer::copyRobotState(refRobotOrigin, genRobot);
    for(int i=0;i<endEffectorParams.size();i++){
      endEffectorParams[i].abcTargetPose = genRobot->link(endEffectorParams[i].parentLink)->T()*endEffectorParams[i].localT;
    }
  }
  return true;
}

// static function
bool AutoStabilizer::calcActualParameters(const AutoStabilizer::ControlMode& mode, const cnoid::BodyPtr& actRobot, cnoid::BodyPtr& actRobotOrigin, std::vector<AutoStabilizer::LegParam>& legParams, std::vector<AutoStabilizer::EndEffectorParam>& endEffectorParams, GaitParam& gaitParam, double dt) {

  {
    // FootOrigin座標系を用いてactRobotをgenerate frameに投影しactRobotOriginとする
    actRobotOrigin->rootLink()->T() = actRobot->rootLink()->T();
    for(int i=0;i<actRobotOrigin->numJoints();i++){
      actRobotOrigin->joint(i)->q() = actRobot->joint(i)->q();
      actRobotOrigin->joint(i)->dq() = actRobot->joint(i)->dq();
    }
    cnoid::DeviceList<cnoid::ForceSensor> actForceSensors(actRobot->devices());
    cnoid::DeviceList<cnoid::ForceSensor> actOriginForceSensors(actRobotOrigin->devices());
    for(int i=0;i<actForceSensors.size();i++){
      actOriginForceSensors[i]->F() = actForceSensors[i]->F();
    }
    double rlegweight = (legParams[RLEG].genContactState)? 1.0 : 0.0;
    double llegweight = (legParams[LLEG].genContactState)? 1.0 : 0.0;
    if(!legParams[RLEG].genContactState && !legParams[LLEG].genContactState) rlegweight = llegweight = 1.0;
    cnoid::Position actrleg = actRobotOrigin->link(endEffectorParams[RLEG].parentLink)->T()*endEffectorParams[RLEG].localT;
    cnoid::Position actlleg = actRobotOrigin->link(endEffectorParams[LLEG].parentLink)->T()*endEffectorParams[LLEG].localT;
    cnoid::Position actFootMidCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{actrleg, actlleg},
                                                               std::vector<double>{rlegweight, llegweight});
    cnoid::Position actFootOriginCoords = mathutil::orientCoordToAxis(actFootMidCoords, cnoid::Vector3::UnitZ());
    cnoid::Position genFootMidCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{endEffectorParams[RLEG].abcTargetPose, endEffectorParams[LLEG].abcTargetPose},
                                                               std::vector<double>{rlegweight, llegweight});  // 1周期前のabcTargetPoseを使っているが、abcTargetPoseは不連続に変化するものではないのでよい
    cnoid::Position genFootOriginCoords = mathutil::orientCoordToAxis(genFootMidCoords, cnoid::Vector3::UnitZ());
    AutoStabilizer::moveCoords(actRobotOrigin, genFootOriginCoords, actFootOriginCoords);
    actRobotOrigin->calcForwardKinematics();
    actRobotOrigin->calcCenterOfMass();
  }

  {
    // 各エンドエフェクタのactualの位置・力を計算
    for(int i=0;i<endEffectorParams.size(); i++){
      endEffectorParams[i].actPose = actRobotOrigin->link(endEffectorParams[i].parentLink)->T() * endEffectorParams[i].localT;
      if(endEffectorParams[i].forceSensor != ""){
        cnoid::ForceSensorPtr sensor = actRobotOrigin->findDevice<cnoid::ForceSensor>(endEffectorParams[i].forceSensor);
        cnoid::Vector6 senF = sensor->F();
        cnoid::Position senPose = sensor->link()->T() * sensor->T_local();
        cnoid::Position eefTosenPose = endEffectorParams[i].actPose.inverse() * senPose;
        cnoid::Vector6 eefF; // endeffector frame. endeffector origin.
        eefF.head<3>() = eefTosenPose.linear() * senF.head<3>();
        eefF.tail<3>() = eefTosenPose.linear() * senF.tail<3>() + eefTosenPose.translation().cross(eefF.head<3>());
        endEffectorParams[i].actWrench.head<3>() = endEffectorParams[i].actPose.linear() * eefF.head<3>();
        endEffectorParams[i].actWrench.tail<3>() = endEffectorParams[i].actPose.linear() * eefF.tail<3>();
      }
    }
  }

  if(mode.isABCInit()){ // startAutoBalancer直後の初回
    // actCogを初期化
    gaitParam.actCog = actRobotOrigin->centerOfMass();
    gaitParam.actCogVel.reset(cnoid::Vector3::Zero());
  }

  {
    // actCogを計算
    bool genContactState_changed = false;
    for(int i=0;i<NUM_LEGS;i++){
      if(legParams[i].genContactState != legParams[i].genContactStatePrev) genContactState_changed = true;
    }
    if(genContactState_changed){
      //座標系が飛んでいるので、gaitParam.actCogVel は前回の周期の値をそのままつかう
    }else{
      cnoid::Vector3 cogVel = (actRobotOrigin->centerOfMass() - gaitParam.actCog) / dt;
      gaitParam.actCogVel.passFilter(cogVel);
    }
    gaitParam.actCog = actRobotOrigin->centerOfMass();
  }

  return true;
}

// static function
bool AutoStabilizer::execAutoBalancer(const AutoStabilizer::ControlMode& mode, const cnoid::BodyPtr& refRobot, cnoid::BodyPtr& refRobotOrigin, const cnoid::BodyPtr& actRobot, cnoid::BodyPtr& actRobotOrigin, cnoid::BodyPtr& genRobot, std::vector<AutoStabilizer::LegParam>& legParams, std::vector<AutoStabilizer::EndEffectorParam>& endEffectorParams, AutoStabilizer::FullbodyState& fullbodyState, GaitParam& gaitParam, double dt, const std::vector<JointParam>& jointParams) {
  AutoStabilizer::calcReferenceParameters(mode, refRobot, refRobotOrigin, genRobot, legParams, endEffectorParams, fullbodyState);

  if(mode.isABCInit()){ // startAutoBalancer直後の初回
    {
      // footStepNodesListを初期化する
      cnoid::Position rlegCoords = genRobot->link(endEffectorParams[RLEG].parentLink)->T()*endEffectorParams[RLEG].localT;
      cnoid::Position llegCoords = genRobot->link(endEffectorParams[LLEG].parentLink)->T()*endEffectorParams[LLEG].localT;
      gaitParam.footstepNodesList.resize(1);
      gaitParam.footstepNodesList[0].dstCoords = {rlegCoords, llegCoords};
      gaitParam.footstepNodesList[0].supportTime = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
      gaitParam.footstepNodesList[0].remainTime = 0.0;
      gaitParam.footstepNodesList[0].stepHeight = {0.0,0.0};
      gaitParam.srcCoords = {rlegCoords, llegCoords};
      gaitParam.genCoords.clear();
      gaitParam.genCoords.emplace_back(rlegCoords, cnoid::Vector6::Zero(), cnoid::Vector6::Zero(), cpp_filters::HOFFARBIB);
      gaitParam.genCoords.emplace_back(llegCoords, cnoid::Vector6::Zero(), cnoid::Vector6::Zero(), cpp_filters::HOFFARBIB);
      cnoid::Vector3 zmp = 0.5 * (rlegCoords.translation() + rlegCoords.linear()*gaitParam.copOffset[RLEG]) + 0.5 * (llegCoords.translation() + llegCoords.linear()*gaitParam.copOffset[LLEG]);
      gaitParam.refZmpTraj.clear();
      gaitParam.refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(zmp,zmp,0.0));
    }

    for(int i=0;i<NUM_LEGS;i++){
      legParams[i].genContactState = gaitParam.isSupportPhase(i);
      legParams[i].genContactStatePrev = legParams[i].genContactState;
    }
  }

  AutoStabilizer::calcActualParameters(mode, actRobot, actRobotOrigin, legParams, endEffectorParams, gaitParam, dt);
  footstepgenerator::calcFootSteps(gaitParam, dt);
  legcoordsgenerator::calcNextCoords(gaitParam, dt, 9.80665, genRobot->mass());

  for(int i=0;i<NUM_LEGS;i++){
    legParams[i].genContactStatePrev = legParams[i].genContactState;
    legParams[i].genContactState = gaitParam.isSupportPhase(i);
  }

  for(int i=0;i<endEffectorParams.size();i++){
    if(i<NUM_LEGS) endEffectorParams[i].abcTargetPose = gaitParam.genCoords[i].value();
    else endEffectorParams[i].abcTargetPose = endEffectorParams[i].refPose;
  }

  return true;
}

// static function
bool AutoStabilizer::execStabilizer(std::vector<EndEffectorParam>& endEffectorParams) {
  // TODO
  for(int i=0;i<endEffectorParams.size();i++){
   endEffectorParams[i].stTargetPose = endEffectorParams[i].abcTargetPose;
  }

  return true;
}

// static function
bool AutoStabilizer::solveFullbodyIK(cnoid::BodyPtr& genRobot, const cnoid::BodyPtr& refRobotOrigin, std::vector<EndEffectorParam>& endEffectorParams, AutoStabilizer::FullbodyIKParam& fullbodyIKParam, double dt, const std::vector<JointParam>& jointParams, const GaitParam& gaitParam) {
  if(fullbodyIKParam.jlim_avoid_weight.size() != 6+genRobot->numJoints()) fullbodyIKParam.jlim_avoid_weight = cnoid::VectorX::Zero(6+genRobot->numJoints());
  cnoid::VectorX dq_weight_all = cnoid::VectorX::Zero(6+genRobot->numJoints());
  for(int i=0;i<6;i++) dq_weight_all[i] = 1.0;
  for(int i=0;i<jointParams.size();i++){
    if(jointParams[i].controllable) dq_weight_all[6+i] = 1.0;
  }

  std::vector<std::shared_ptr<IK::IKConstraint> > ikConstraint;

  // EEF
  for(int i=0;i<endEffectorParams.size();i++){
    endEffectorParams[i].ikPositionConstraint->A_link() = genRobot->link(endEffectorParams[i].parentLink);
    endEffectorParams[i].ikPositionConstraint->A_localpos() = endEffectorParams[i].localT;
    endEffectorParams[i].ikPositionConstraint->B_link() = nullptr;
    endEffectorParams[i].ikPositionConstraint->B_localpos() = endEffectorParams[i].stTargetPose;
    endEffectorParams[i].ikPositionConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt;
    endEffectorParams[i].ikPositionConstraint->precision() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
    if(i<NUM_LEGS) endEffectorParams[i].ikPositionConstraint->weight() << 9.0, 9.0, 9.0, 9.0, 9.0, 9.0;
    else endEffectorParams[i].ikPositionConstraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    endEffectorParams[i].ikPositionConstraint->eval_link() = nullptr;
    endEffectorParams[i].ikPositionConstraint->eval_localR() = endEffectorParams[i].ikPositionConstraint->B_localpos().linear();
    ikConstraint.push_back(endEffectorParams[i].ikPositionConstraint);
  }

  // COM
  {
    fullbodyIKParam.comConstraint->A_robot() = genRobot;
    fullbodyIKParam.comConstraint->A_localp() = cnoid::Vector3::Zero();
    fullbodyIKParam.comConstraint->B_robot() = nullptr;
    fullbodyIKParam.comConstraint->B_localp() = gaitParam.genCog;
    fullbodyIKParam.comConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt;
    fullbodyIKParam.comConstraint->precision() << 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
    fullbodyIKParam.comConstraint->weight() << 3.0, 3.0, 3.0;
    fullbodyIKParam.comConstraint->eval_R() = cnoid::Matrix3::Identity();
    ikConstraint.push_back(fullbodyIKParam.comConstraint);
  }

  // Angular Momentum
  {
    fullbodyIKParam.angularMomentumConstraint->robot() = genRobot;
    fullbodyIKParam.angularMomentumConstraint->targetAngularMomentum() = cnoid::Vector3::Zero(); // TODO
    fullbodyIKParam.angularMomentumConstraint->maxError() << 1.0*dt, 1.0*dt, 1.0*dt;
    fullbodyIKParam.angularMomentumConstraint->precision() << 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
    fullbodyIKParam.angularMomentumConstraint->weight() << 1e-4, 1e-4, 0.0; // TODO
    fullbodyIKParam.angularMomentumConstraint->dt() = dt;
    fullbodyIKParam.comConstraint->eval_R() = cnoid::Matrix3::Identity();
    ikConstraint.push_back(fullbodyIKParam.angularMomentumConstraint);
  }

  // root
  {
    fullbodyIKParam.rootPositionConstraint->A_link() = genRobot->rootLink();
    fullbodyIKParam.rootPositionConstraint->A_localpos() = cnoid::Position::Identity();
    fullbodyIKParam.rootPositionConstraint->B_link() = nullptr;
    fullbodyIKParam.rootPositionConstraint->B_localpos() = refRobotOrigin->rootLink()->T();
    fullbodyIKParam.rootPositionConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt;
    fullbodyIKParam.rootPositionConstraint->precision() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
    fullbodyIKParam.rootPositionConstraint->weight() << 0.0, 0.0, 0.0, 1e-1, 1e-1, 1e-1;
    fullbodyIKParam.rootPositionConstraint->eval_link() = nullptr;
    fullbodyIKParam.rootPositionConstraint->eval_localR() = cnoid::Matrix3::Identity();
    ikConstraint.push_back(fullbodyIKParam.rootPositionConstraint);
  }

  // reference angle
  {
    if(fullbodyIKParam.refJointAngleConstraint.size() != genRobot->numJoints()) { // 初回
      fullbodyIKParam.refJointAngleConstraint.clear();
      for(size_t i=0;i<genRobot->numJoints();i++) fullbodyIKParam.refJointAngleConstraint.push_back(std::make_shared<IK::JointAngleConstraint>());
    }
    for(size_t i=0;i<genRobot->numJoints();i++){
      if(!jointParams[i].controllable) continue;
      fullbodyIKParam.refJointAngleConstraint[i]->joint() = genRobot->joint(i);
      fullbodyIKParam.refJointAngleConstraint[i]->maxError() = 10.0 * dt; // 高優先度のmaxError以下にしないと優先度逆転するおそれ
      fullbodyIKParam.refJointAngleConstraint[i]->weight() = 1e-1; // 小さい値すぎると、qp終了判定のtoleranceによって無視されてしまう
      fullbodyIKParam.refJointAngleConstraint[i]->targetq() = refRobotOrigin->joint(i)->q();
      ikConstraint.push_back(fullbodyIKParam.refJointAngleConstraint[i]);
    }
  }

  for(int i=0;i<ikConstraint.size();i++) ikConstraint[i]->debuglevel() = 0; //debuglevel
  fik::solveFullbodyIKLoopFast(genRobot,
                               ikConstraint,
                               fullbodyIKParam.jlim_avoid_weight,
                               dq_weight_all,
                               1,//loop
                               1e-6,
                               0 //debug
                               );
  return true;
}

// static function
bool AutoStabilizer::writeOutPortData(AutoStabilizer::Ports& ports, cnoid::BodyPtr genRobot, const AutoStabilizer::ControlMode& mode, AutoStabilizer::OutputOffsetInterpolators& outputOffsetInterpolators, double dt){
  if(outputOffsetInterpolators.qInterpolator.size() == 0){ // 初回のみ
    for(int i=0;i<genRobot->numJoints();i++){
      outputOffsetInterpolators.qInterpolator.emplace_back(0.0,0.0,0.0,cpp_filters::HOFFARBIB);
      outputOffsetInterpolators.genTauInterpolator.emplace_back(0.0,0.0,0.0,cpp_filters::HOFFARBIB);
    }
  }

  {
    // q
    ports.m_q_.tm = ports.m_qRef_.tm;
    ports.m_q_.data.length(genRobot->numJoints());
    for(int i=0;i<genRobot->numJoints();i++){
      if(!mode.isSync()){
        ports.m_q_.data[i] = genRobot->joint(i)->q();
      }else{
        if(mode.isSyncInit()){
          outputOffsetInterpolators.qInterpolator[i].reset(ports.m_q_.data[i]-genRobot->joint(i)->q());
        }
        outputOffsetInterpolators.qInterpolator[i].setGoal(0.0,mode.remainTime());
        double x,v,a;
        outputOffsetInterpolators.qInterpolator[i].get(x,v,a,dt);
        ports.m_q_.data[i] = genRobot->joint(i)->q() + x;
      }
    }
    ports.m_qOut_.write();
  }

  {
    // tau
    ports.m_genTau_.tm = ports.m_qRef_.tm;
    ports.m_genTau_.data.length(genRobot->numJoints());
    for(int i=0;i<genRobot->numJoints();i++){
      if(!mode.isSync()){
        ports.m_genTau_.data[i] = genRobot->joint(i)->u();
      }else{
        if(mode.isSyncInit()){
          outputOffsetInterpolators.genTauInterpolator[i].reset(ports.m_genTau_.data[i]-genRobot->joint(i)->u());
        }
        outputOffsetInterpolators.genTauInterpolator[i].setGoal(0.0,mode.remainTime());
        double x,v,a;
        outputOffsetInterpolators.genTauInterpolator[i].get(x,v,a,dt);
        ports.m_genTau_.data[i] = genRobot->joint(i)->u() + x;
      }
    }
    ports.m_genTauOut_.write();
  }

  {
    // basePose
    cnoid::Position basePose = genRobot->rootLink()->T();
    if(mode.isSync()){
      if(mode.isSyncInit()){
        cnoid::Position prevPose;
        prevPose.translation()= cnoid::Vector3(ports.m_genBasePose_.data.position.x,ports.m_genBasePose_.data.position.x,ports.m_genBasePose_.data.position.x);
        prevPose.linear() = cnoid::rotFromRpy(ports.m_genBasePose_.data.orientation.r, ports.m_genBasePose_.data.orientation.p, ports.m_genBasePose_.data.orientation.y);
        outputOffsetInterpolators.genBasePoseInterpolator.reset(prevPose * basePose.inverse());
      }
      outputOffsetInterpolators.genBasePoseInterpolator.setGoal(cnoid::Position::Identity(),mode.remainTime());
      outputOffsetInterpolators.genBasePoseInterpolator.interpolate(dt);
      cnoid::Position offsetPose = outputOffsetInterpolators.genBasePoseInterpolator.value();
      basePose = offsetPose * basePose;
    }
    cnoid::Vector3 basePos = basePose.translation();
    cnoid::Matrix3 baseR = basePose.linear();
    cnoid::Vector3 baseRpy = cnoid::rpyFromRot(basePose.linear());

    ports.m_genBasePose_.tm = ports.m_qRef_.tm;
    ports.m_genBasePose_.data.position.x = basePos[0];
    ports.m_genBasePose_.data.position.y = basePos[1];
    ports.m_genBasePose_.data.position.z = basePos[2];
    ports.m_genBasePose_.data.orientation.r = baseRpy[0];
    ports.m_genBasePose_.data.orientation.p = baseRpy[1];
    ports.m_genBasePose_.data.orientation.y = baseRpy[2];
    ports.m_genBasePoseOut_.write();

    ports.m_genBaseTform_.tm = ports.m_qRef_.tm;
    ports.m_genBaseTform_.data.length(12);
    for(int i=0;i<3;i++){
      ports.m_genBaseTform_.data[i] = basePos[i];
    }
    for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        ports.m_genBaseTform_.data[3+i*3+j] = baseR(i,j);// row major
      }
    }
    ports.m_genBaseTformOut_.write();

    ports.m_genBasePos_.tm = ports.m_qRef_.tm;
    ports.m_genBasePos_.data.x = basePos[0];
    ports.m_genBasePos_.data.y = basePos[1];
    ports.m_genBasePos_.data.z = basePos[2];
    ports.m_genBasePosOut_.write();
    ports.m_genBaseRpy_.tm = ports.m_qRef_.tm;
    ports.m_genBaseRpy_.data.r = baseRpy[0];
    ports.m_genBaseRpy_.data.p = baseRpy[1];
    ports.m_genBaseRpy_.data.y = baseRpy[2];
    ports.m_genBaseRpyOut_.write();
  }

  return true;
}

RTC::ReturnCode_t AutoStabilizer::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);

  std::string instance_name = std::string(this->m_profile.instance_name);
  this->loop_++;

  if(!AutoStabilizer::readInPortData(this->ports_, this->refRobot_, this->actRobot_, this->endEffectorParams_)) return RTC::RTC_OK;  // qRef が届かなければ何もしない

  this->mode_.update(this->dt_);

  if(!this->mode_.isABCRunning()) {
    AutoStabilizer::copyRobotState(this->refRobot_, this->genRobot_);
  }else{
    AutoStabilizer::execAutoBalancer(this->mode_, this->refRobot_, this->refRobotOrigin_, this->actRobot_, this->actRobotOrigin_, this->genRobot_, this->legParams_, this->endEffectorParams_, this->fullbodyState_, this->gaitParam_, this->dt_, this->jointParams_);
    AutoStabilizer::execStabilizer(this->endEffectorParams_);
    AutoStabilizer::solveFullbodyIK(this->genRobot_, this->refRobotOrigin_, this->endEffectorParams_, this->fullbodyIKParam_, this->dt_, this->jointParams_, this->gaitParam_);
  }

  AutoStabilizer::writeOutPortData(this->ports_, this->genRobot_, this->mode_, this->outputOffsetInterpolators_, this->dt_);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t AutoStabilizer::onActivated(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  // 各種処理を初期化する TODO
  this->mode_.reset();
  return RTC::RTC_OK;
}
RTC::ReturnCode_t AutoStabilizer::onDeactivated(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t AutoStabilizer::onFinalize(){ return RTC::RTC_OK; }

bool AutoStabilizer::goPos(const double& x, const double& y, const double& th){
  std::lock_guard<std::mutex> guard(this->mutex_);
  return true;
}
bool AutoStabilizer::goVelocity(const double& vx, const double& vy, const double& vth){
  std::lock_guard<std::mutex> guard(this->mutex_);
  return true;
}
bool AutoStabilizer::goStop(){
  std::lock_guard<std::mutex> guard(this->mutex_);
  return true;
}
bool AutoStabilizer::jumpTo(const double& x, const double& y, const double& z, const double& ts, const double& tf){
  std::lock_guard<std::mutex> guard(this->mutex_);
  return true;
}
bool AutoStabilizer::setFootSteps(const OpenHRP::AutoStabilizerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx){
  std::lock_guard<std::mutex> guard(this->mutex_);
  return true;
}
bool AutoStabilizer::setFootStepsWithParam(const OpenHRP::AutoStabilizerService::FootstepsSequence& fss, const OpenHRP::AutoStabilizerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx){
  std::lock_guard<std::mutex> guard(this->mutex_);
  return true;
}
void AutoStabilizer::waitFootSteps(){
  return;
}

bool AutoStabilizer::releaseEmergencyStop(){
  std::lock_guard<std::mutex> guard(this->mutex_);
  return true;
}


bool AutoStabilizer::startAutoBalancer(){
  if(this->mode_.setNextTransition(ControlMode::START_ABC)){
    std::cerr << "[" << m_profile.instance_name << "] start auto balancer mode" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_ABC) usleep(1000);
    usleep(1000);
    return true;
  }else{
    return false;
  }
}
bool AutoStabilizer::stopAutoBalancer(){
  if(this->mode_.setNextTransition(ControlMode::STOP_ABC)){
    std::cerr << "[" << m_profile.instance_name << "] stop auto balancer mode" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_IDLE) usleep(1000);
    usleep(1000);
    return true;
  }else{
    return false;
  }
}
bool AutoStabilizer::startStabilizer(void){
  if(this->mode_.setNextTransition(ControlMode::START_ST)){
    std::cerr << "[" << m_profile.instance_name << "] start ST" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_ST) usleep(1000);
    usleep(1000);
    return true;
  }else{
    return false;
  }
}
bool AutoStabilizer::stopStabilizer(void){
  if(this->mode_.setNextTransition(ControlMode::STOP_ST)){
    std::cerr << "[" << m_profile.instance_name << "] stop ST" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_ABC) usleep(1000);
    usleep(1000);
    return true;
  }else{
    return false;
  }
}

bool AutoStabilizer::setGaitGeneratorParam(const OpenHRP::AutoStabilizerService::GaitGeneratorParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  return true;
}
bool AutoStabilizer::getGaitGeneratorParam(OpenHRP::AutoStabilizerService::GaitGeneratorParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  return true;
}
bool AutoStabilizer::setAutoBalancerParam(const OpenHRP::AutoStabilizerService::AutoBalancerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  this->mode_.abc_transition_time = i_param.transition_time;
  return true;
}
bool AutoStabilizer::getAutoBalancerParam(OpenHRP::AutoStabilizerService::AutoBalancerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  i_param.transition_time = this->mode_.abc_transition_time;
  // i_param.leg_names // refFootOriginWeightとautoControlRatioが必要
  return true;
}
void AutoStabilizer::setStabilizerParam(const OpenHRP::AutoStabilizerService::StabilizerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  this->mode_.st_transition_time = i_param.transition_time;
  return;
}
void AutoStabilizer::getStabilizerParam(OpenHRP::AutoStabilizerService::StabilizerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  i_param.transition_time = this->mode_.st_transition_time;
  return;
}

bool AutoStabilizer::getProperty(const std::string& key, std::string& ret) {
  if (this->getProperties().hasKey(key.c_str())) {
    ret = std::string(this->getProperties()[key.c_str()]);
  } else if (this->m_pManager->getConfig().hasKey(key.c_str())) { // 引数 -o で与えたプロパティを捕捉
    ret = std::string(this->m_pManager->getConfig()[key.c_str()]);
  } else {
    return false;
  }
  std::cerr << "[" << this->m_profile.instance_name << "] " << key << ": " << ret <<std::endl;
  return true;
}


extern "C"{
    void AutoStabilizerInit(RTC::Manager* manager) {
        RTC::Properties profile(AutoStabilizer_spec);
        manager->registerFactory(profile, RTC::Create<AutoStabilizer>, RTC::Delete<AutoStabilizer>);
    }
};
