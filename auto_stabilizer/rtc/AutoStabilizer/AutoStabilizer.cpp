#include "AutoStabilizer.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/ValueTree>
#include <cnoid/EigenUtil>
#include "MathUtil.h"
#include "CnoidBodyUtil.h"
#include <limits>

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
  m_selfCollisionIn_("selfCollisionIn", m_selfCollision_),
  m_steppableRegionIn_("steppableRegionIn", m_steppableRegion_),
  m_landingHeightIn_("landingHeightIn", m_landingHeight_),

  m_qOut_("q", m_q_),
  m_genTauOut_("genTauOut", m_genTau_),
  m_genBasePoseOut_("genBasePoseOut", m_genBasePose_),
  m_genBaseTformOut_("genBaseTformOut", m_genBaseTform_),
  m_genImuAccOut_("genImuAccOut", m_genImuAcc_),
  m_landingTargetOut_("landingTargetOut", m_landingTarget_),

  m_genBasePosOut_("genBasePosOut", m_genBasePos_),
  m_genBaseRpyOut_("genBaseRpyOut", m_genBaseRpy_),
  m_genCogOut_("genCogOut", m_genCog_),
  m_genDcmOut_("genDcmOut", m_genDcm_),
  m_genZmpOut_("genZmpOut", m_genZmp_),
  m_tgtZmpOut_("tgtZmpOut", m_tgtZmp_),
  m_actCogOut_("actCogOut", m_actCog_),
  m_actDcmOut_("actDcmOut", m_actDcm_),
  m_dstLandingPosOut_("dstLandingPosOut", m_dstLandingPos_),
  m_remainTimeOut_("remainTimeOut", m_remainTime_),
  m_genCoordsOut_("genCoordsOut", m_genCoords_),
  m_captureRegionOut_("captureRegionOut", m_captureRegion_),
  m_steppableRegionLogOut_("steppableRegionLogOut", m_steppableRegionLog_),
  m_steppableRegionNumLogOut_("steppableRegionNumLogOut", m_steppableRegionNumLog_),
  m_strideLimitationHullOut_("strideLimitationHullOut", m_strideLimitationHull_),
  m_cpViewerLogOut_("cpViewerLogOut", m_cpViewerLog_),

  m_AutoStabilizerServicePort_("AutoStabilizerService"),

  m_RobotHardwareServicePort_("RobotHardwareService"){
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
  this->addInPort("selfCollisionIn", this->ports_.m_selfCollisionIn_);
  this->addInPort("steppableRegionIn", this->ports_.m_steppableRegionIn_);
  this->addInPort("landingHeightIn", this->ports_.m_landingHeightIn_);
  this->addOutPort("q", this->ports_.m_qOut_);
  this->addOutPort("genTauOut", this->ports_.m_genTauOut_);
  this->addOutPort("genBasePoseOut", this->ports_.m_genBasePoseOut_);
  this->addOutPort("genBaseTformOut", this->ports_.m_genBaseTformOut_);
  this->addOutPort("genImuAccOut", this->ports_.m_genImuAccOut_);
  this->addOutPort("landingTargetOut", this->ports_.m_landingTargetOut_);
  this->addOutPort("genBasePosOut", this->ports_.m_genBasePosOut_);
  this->addOutPort("genBaseRpyOut", this->ports_.m_genBaseRpyOut_);
  this->addOutPort("genCogOut", this->ports_.m_genCogOut_);
  this->addOutPort("genDcmOut", this->ports_.m_genDcmOut_);
  this->addOutPort("genZmpOut", this->ports_.m_genZmpOut_);
  this->addOutPort("tgtZmpOut", this->ports_.m_tgtZmpOut_);
  this->addOutPort("actCogOut", this->ports_.m_actCogOut_);
  this->addOutPort("actDcmOut", this->ports_.m_actDcmOut_);
  this->addOutPort("dstLandingPosOut", this->ports_.m_dstLandingPosOut_);
  this->addOutPort("remainTimeOut", this->ports_.m_remainTimeOut_);
  this->addOutPort("genCoordsOut", this->ports_.m_genCoordsOut_);
  this->addOutPort("captureRegionOut", this->ports_.m_captureRegionOut_);
  this->addOutPort("steppableRegionLogOut", this->ports_.m_steppableRegionLogOut_);
  this->addOutPort("steppableRegionNumLogOut", this->ports_.m_steppableRegionNumLogOut_);
  this->addOutPort("strideLimitationHullOut", this->ports_.m_strideLimitationHullOut_);
  this->addOutPort("cpViewerLogOut", this->ports_.m_cpViewerLogOut_);
  this->ports_.m_AutoStabilizerServicePort_.registerProvider("service0", "AutoStabilizerService", this->ports_.m_service0_);
  this->addPort(this->ports_.m_AutoStabilizerServicePort_);
  this->ports_.m_RobotHardwareServicePort_.registerConsumer("service0", "RobotHardwareService", this->ports_.m_robotHardwareService0_);
  this->addPort(this->ports_.m_RobotHardwareServicePort_);
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
    if(!robot->rootLink()->isFreeJoint()){
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "rootLink is not FreeJoint [" << fileName << "]" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
    this->gaitParam_.init(robot);

    // generate JointParams
    for(int i=0;i<this->gaitParam_.genRobot->numJoints();i++){
      cnoid::LinkPtr joint = this->gaitParam_.genRobot->joint(i);
      double climit = 0.0, gearRatio = 0.0, torqueConst = 0.0;
      joint->info()->read("climit",climit); joint->info()->read("gearRatio",gearRatio); joint->info()->read("torqueConst",torqueConst);
      this->gaitParam_.maxTorque[i] = std::max(climit * gearRatio * torqueConst, 0.0);
    }
    std::string jointLimitTableStr; this->getProperty("joint_limit_table",jointLimitTableStr);
    std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > jointLimitTables = joint_limit_table::readJointLimitTablesFromProperty (this->gaitParam_.genRobot, jointLimitTableStr);
    for(size_t i=0;i<jointLimitTables.size();i++){
      // apply margin
      for(size_t j=0;j<jointLimitTables[i]->lLimitTable().size();j++){
        if(jointLimitTables[i]->uLimitTable()[j] - jointLimitTables[i]->lLimitTable()[j] > 0.002){
          jointLimitTables[i]->uLimitTable()[j] -= 0.001;
          jointLimitTables[i]->lLimitTable()[j] += 0.001;
        }
      }
      this->gaitParam_.jointLimitTables[jointLimitTables[i]->getSelfJoint()->jointId()].push_back(jointLimitTables[i]);
    }

    // apply margin to jointlimit
    for(int i=0;i<this->gaitParam_.genRobot->numJoints();i++){
      cnoid::LinkPtr joint = this->gaitParam_.genRobot->joint(i);
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
      parentLink.erase(std::remove(parentLink.begin(), parentLink.end(), ' '), parentLink.end()); // remove whitespace
      if(!this->gaitParam_.refRobotRaw->link(parentLink)){
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " link [" << parentLink << "]" << " is not found for " << name << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
      cnoid::Matrix3 localR;
      if(localaxis.norm() == 0) localR = cnoid::Matrix3::Identity();
      else localR = Eigen::AngleAxisd(localangle, localaxis.normalized()).toRotationMatrix();
      cnoid::Position localT;
      localT.translation() = localp;
      localT.linear() = localR;

      this->gaitParam_.push_backEE(name, parentLink, localT);
    }

    // 0番目が右脚. 1番目が左脚. という仮定がある.
    if(this->gaitParam_.eeName.size() < NUM_LEGS || this->gaitParam_.eeName[RLEG] != "rleg" || this->gaitParam_.eeName[LLEG] != "lleg"){
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " this->gaitParam_.eeName.size() < 2 || this->gaitParams.eeName[0] != \"rleg\" || this->gaitParam_.eeName[1] != \"lleg\" not holds" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
  }

  {
    // generate LegParams
    // init-poseのとき両脚が同一平面上で, Y軸方向に横に並んでいるという仮定がある
    cnoid::Position defautFootMidCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{cnoid::Position(this->gaitParam_.refRobot->link(this->gaitParam_.eeParentLink[RLEG])->T()*this->gaitParam_.eeLocalT[RLEG]),cnoid::Position(this->gaitParam_.refRobot->link(this->gaitParam_.eeParentLink[LLEG])->T()*this->gaitParam_.eeLocalT[LLEG])},
                                                            std::vector<double>{1,1});
    for(int i=0; i<NUM_LEGS; i++){
      cnoid::Position defaultPose = this->gaitParam_.refRobot->link(this->gaitParam_.eeParentLink[i])->T()*this->gaitParam_.eeLocalT[i];
      cnoid::Vector3 defaultTranslatePos = defautFootMidCoords.inverse() * defaultPose.translation();
      defaultTranslatePos[0] = 0.0;
      defaultTranslatePos[2] = 0.0;
      this->gaitParam_.defaultTranslatePos[i].reset(defaultTranslatePos);
    }
  }

  {
    // add more ports (ロボットモデルやEndEffectorの情報を使って)

    // 各EndEffectorにつき、ref<name>WrenchInというInPortをつくる
    this->ports_.m_refEEWrenchIn_.resize(this->gaitParam_.eeName.size());
    this->ports_.m_refEEWrench_.resize(this->gaitParam_.eeName.size());
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      std::string name = "ref"+this->gaitParam_.eeName[i]+"WrenchIn";
      this->ports_.m_refEEWrenchIn_[i] = std::make_unique<RTC::InPort<RTC::TimedDoubleSeq> >(name.c_str(), this->ports_.m_refEEWrench_[i]);
      this->addInPort(name.c_str(), *(this->ports_.m_refEEWrenchIn_[i]));
    }

    // 各ForceSensorにつき、act<name>InというInportをつくる
    cnoid::DeviceList<cnoid::ForceSensor> forceSensors(this->gaitParam_.actRobotRaw->devices());
    this->ports_.m_actWrenchIn_.resize(forceSensors.size());
    this->ports_.m_actWrench_.resize(forceSensors.size());
    for(int i=0;i<forceSensors.size();i++){
      std::string name = "act"+forceSensors[i]->name()+"In";
      this->ports_.m_actWrenchIn_[i] = std::make_unique<RTC::InPort<RTC::TimedDoubleSeq> >(name.c_str(), this->ports_.m_actWrench_[i]);
      this->addInPort(name.c_str(), *(this->ports_.m_actWrenchIn_[i]));
    }

    // 各EndEffectorにつき、ref<name>PoseInというInPortをつくる
    this->ports_.m_refEEPoseIn_.resize(this->gaitParam_.eeName.size());
    this->ports_.m_refEEPose_.resize(this->gaitParam_.eeName.size());
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      std::string name = "ref"+this->gaitParam_.eeName[i]+"PoseIn";
      this->ports_.m_refEEPoseIn_[i] = std::make_unique<RTC::InPort<RTC::TimedPose3D> >(name.c_str(), this->ports_.m_refEEPose_[i]);
      this->addInPort(name.c_str(), *(this->ports_.m_refEEPoseIn_[i]));
    }

    // 各EndEffectorにつき、act<name>PoseOutというOutPortをつくる
    this->ports_.m_actEEPoseOut_.resize(this->gaitParam_.eeName.size());
    this->ports_.m_actEEPose_.resize(this->gaitParam_.eeName.size());
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      std::string name = "act"+this->gaitParam_.eeName[i]+"PoseOut";
      this->ports_.m_actEEPoseOut_[i] = std::make_unique<RTC::OutPort<RTC::TimedPose3D> >(name.c_str(), this->ports_.m_actEEPose_[i]);
      this->addOutPort(name.c_str(), *(this->ports_.m_actEEPoseOut_[i]));
    }

    // 各EndEffectorにつき、tgt<name>WrenchOutというOutPortをつくる
    this->ports_.m_tgtEEWrenchOut_.resize(this->gaitParam_.eeName.size());
    this->ports_.m_tgtEEWrench_.resize(this->gaitParam_.eeName.size());
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      std::string name = "tgt"+this->gaitParam_.eeName[i]+"WrenchOut";
      this->ports_.m_tgtEEWrenchOut_[i] = std::make_unique<RTC::OutPort<RTC::TimedDoubleSeq> >(name.c_str(), this->ports_.m_tgtEEWrench_[i]);
      this->addOutPort(name.c_str(), *(this->ports_.m_tgtEEWrenchOut_[i]));
    }

    // 各EndEffectorにつき、act<name>WrenchOutというOutPortをつくる
    this->ports_.m_actEEWrenchOut_.resize(this->gaitParam_.eeName.size());
    this->ports_.m_actEEWrench_.resize(this->gaitParam_.eeName.size());
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      std::string name = "act"+this->gaitParam_.eeName[i]+"WrenchOut";
      this->ports_.m_actEEWrenchOut_[i] = std::make_unique<RTC::OutPort<RTC::TimedDoubleSeq> >(name.c_str(), this->ports_.m_actEEWrench_[i]);
      this->addOutPort(name.c_str(), *(this->ports_.m_actEEWrenchOut_[i]));
    }

  }

  {
    // init ActToGenFrameConverter
    this->actToGenFrameConverter_.eeForceSensor.resize(this->gaitParam_.eeName.size());
    cnoid::DeviceList<cnoid::ForceSensor> forceSensors(this->gaitParam_.refRobotRaw->devices());
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      // 各EndEffectorsから親リンク側に遡っていき、最初に見つかったForceSensorをEndEffectorに対応付ける. 以後、ForceSensorの値を座標変換したものがEndEffectorが受けている力とみなされる. 見つからなければ受けている力は常に0とみなされる
      std::string forceSensor = "";
      cnoid::LinkPtr link = this->gaitParam_.refRobotRaw->link(this->gaitParam_.eeParentLink[i]);
      bool found = false;
      while (link != nullptr && found == false) {
        for (size_t j = 0; j < forceSensors.size(); j++) {
          if(forceSensors[j]->link() == link) {
            forceSensor = forceSensors[j]->name();
            found = true;
            break;
          }
        }
      }
      this->actToGenFrameConverter_.eeForceSensor[i] = forceSensor;
    }
  }

  // init ImpedanceController
  for(int i=0;i<this->gaitParam_.eeName.size();i++) this->impedanceController_.push_backEE();

  // init Stabilizer
  this->stabilizer_.init(this->gaitParam_, this->gaitParam_.actRobotTqc);

  // init FullbodyIKSolver
  this->fullbodyIKSolver_.init(this->gaitParam_.genRobot, this->gaitParam_);

  // initialize parameters
  this->loop_ = 0;

  return RTC::RTC_OK;
}

// static function
bool AutoStabilizer::readInPortData(const double& dt, const GaitParam& gaitParam, const AutoStabilizer::ControlMode& mode, AutoStabilizer::Ports& ports, cnoid::BodyPtr refRobotRaw, cnoid::BodyPtr actRobotRaw, std::vector<cnoid::Vector6>& refEEWrenchOrigin, std::vector<cpp_filters::TwoPointInterpolatorSE3>& refEEPoseRaw, std::vector<GaitParam::Collision>& selfCollision, std::vector<std::vector<cnoid::Vector3> >& steppableRegion, std::vector<double>& steppableHeight, double& relLandingHeight, cnoid::Vector3& relLandingNormal){
  bool qRef_updated = false;
  if(ports.m_qRefIn_.isNew()){
    ports.m_qRefIn_.read();
    if(ports.m_qRef_.data.length() == refRobotRaw->numJoints()){
      for(int i=0;i<ports.m_qRef_.data.length();i++){
        if(std::isfinite(ports.m_qRef_.data[i])) refRobotRaw->joint(i)->q() = ports.m_qRef_.data[i];
        else std::cerr << "m_qRef is not finite!" << std::endl;
      }
      qRef_updated = true;
    }
  }
  if(ports.m_refTauIn_.isNew()){
    ports.m_refTauIn_.read();
    if(ports.m_refTau_.data.length() == refRobotRaw->numJoints()){
      for(int i=0;i<ports.m_refTau_.data.length();i++){
        if(std::isfinite(ports.m_refTau_.data[i])) refRobotRaw->joint(i)->u() = ports.m_refTau_.data[i];
        else std::cerr << "m_refTau is not finite!" << std::endl;
      }
    }
  }
  if(ports.m_refBasePosIn_.isNew()){
    ports.m_refBasePosIn_.read();
    if(std::isfinite(ports.m_refBasePos_.data.x) && std::isfinite(ports.m_refBasePos_.data.y) && std::isfinite(ports.m_refBasePos_.data.z)){
      refRobotRaw->rootLink()->p()[0] = ports.m_refBasePos_.data.x;
      refRobotRaw->rootLink()->p()[1] = ports.m_refBasePos_.data.y;
      refRobotRaw->rootLink()->p()[2] = ports.m_refBasePos_.data.z;
    } else {
      std::cerr << "m_refBasePos is not finite!" << std::endl;
    }
  }
  if(ports.m_refBaseRpyIn_.isNew()){
    ports.m_refBaseRpyIn_.read();
    if(std::isfinite(ports.m_refBaseRpy_.data.r) && std::isfinite(ports.m_refBaseRpy_.data.p) && std::isfinite(ports.m_refBaseRpy_.data.y)){
      refRobotRaw->rootLink()->R() = cnoid::rotFromRpy(ports.m_refBaseRpy_.data.r, ports.m_refBaseRpy_.data.p, ports.m_refBaseRpy_.data.y);
    } else {
      std::cerr << "m_refBaseRpy is not finite!" << std::endl;
    }
  }
  refRobotRaw->calcForwardKinematics();
  refRobotRaw->calcCenterOfMass();

  for(int i=0;i<ports.m_refEEWrenchIn_.size();i++){
    if(ports.m_refEEWrenchIn_[i]->isNew()){
      ports.m_refEEWrenchIn_[i]->read();
      if(ports.m_refEEWrench_[i].data.length() == 6){
        for(int j=0;j<6;j++){
          if(std::isfinite(ports.m_refEEWrench_[i].data[j])) refEEWrenchOrigin[i][j] = ports.m_refEEWrench_[i].data[j];
          else std::cerr << "m_refEEWrench is not finite!" << std::endl;
        }
      }
    }
  }

  for(int i=0;i<ports.m_refEEPoseIn_.size();i++){
    if(ports.m_refEEPoseIn_[i]->isNew()){
      ports.m_refEEPoseIn_[i]->read();
      if(std::isfinite(ports.m_refEEPose_[i].data.position.x) && std::isfinite(ports.m_refEEPose_[i].data.position.y) && std::isfinite(ports.m_refEEPose_[i].data.position.z) &&
         std::isfinite(ports.m_refEEPose_[i].data.orientation.r) && std::isfinite(ports.m_refEEPose_[i].data.orientation.p) && std::isfinite(ports.m_refEEPose_[i].data.orientation.y)){
        cnoid::Position pose;
        pose.translation()[0] = ports.m_refEEPose_[i].data.position.x;
        pose.translation()[1] = ports.m_refEEPose_[i].data.position.y;
        pose.translation()[2] = ports.m_refEEPose_[i].data.position.z;
        pose.linear() = cnoid::rotFromRpy(ports.m_refEEPose_[i].data.orientation.r, ports.m_refEEPose_[i].data.orientation.p, ports.m_refEEPose_[i].data.orientation.y);
        refEEPoseRaw[i].setGoal(pose, 0.3); // 0.3秒で補間
        ports.refEEPoseLastUpdateTime_ = ports.m_qRef_.tm;
      } else {
        std::cerr << "m_refEEPose is not finite!" << std::endl;
      }
    }
    refEEPoseRaw[i].interpolate(dt);
  }

  if(ports.m_qActIn_.isNew()){
    ports.m_qActIn_.read();
    if(ports.m_qAct_.data.length() == actRobotRaw->numJoints()){
      for(int i=0;i<ports.m_qAct_.data.length();i++){
        if(std::isfinite(ports.m_qAct_.data[i])) actRobotRaw->joint(i)->q() = ports.m_qAct_.data[i];
        else std::cerr << "m_qAct is not finite!" << std::endl;
      }
    }
  }
  if(ports.m_dqActIn_.isNew()){
    ports.m_dqActIn_.read();

    if(ports.m_dqAct_.data.length() == actRobotRaw->numJoints()){
      for(int i=0;i<ports.m_dqAct_.data.length();i++){
        if(std::isfinite(ports.m_dqAct_.data[i])) actRobotRaw->joint(i)->dq() = ports.m_dqAct_.data[i];
        else  std::cerr << "m_dqAct is not finite!" << std::endl;
      }
    }
  }
  if(ports.m_actImuIn_.isNew()){
    ports.m_actImuIn_.read();
    if(std::isfinite(ports.m_actImu_.data.r) && std::isfinite(ports.m_actImu_.data.p) && std::isfinite(ports.m_actImu_.data.y)){
      actRobotRaw->calcForwardKinematics();
      cnoid::RateGyroSensorPtr imu = actRobotRaw->findDevice<cnoid::RateGyroSensor>("gyrometer");
      cnoid::Matrix3 imuR = imu->link()->R() * imu->R_local();
      cnoid::Matrix3 actR = cnoid::rotFromRpy(ports.m_actImu_.data.r, ports.m_actImu_.data.p, ports.m_actImu_.data.y);
      actRobotRaw->rootLink()->R() = Eigen::Matrix3d(Eigen::AngleAxisd(actR) * Eigen::AngleAxisd(imuR.transpose() * actRobotRaw->rootLink()->R())); // 単純に3x3行列の空間でRを積算していると、だんだん数値誤差によって回転行列でなくなってしまう恐れがあるので念の為
    }else{
      std::cerr << "m_actImu is not finite!" << std::endl;
    }
  }
  actRobotRaw->calcForwardKinematics();
  actRobotRaw->calcCenterOfMass();

  cnoid::DeviceList<cnoid::ForceSensor> forceSensors(actRobotRaw->devices());
  for(int i=0;i<ports.m_actWrenchIn_.size();i++){
    if(ports.m_actWrenchIn_[i]->isNew()){
      ports.m_actWrenchIn_[i]->read();
      if(ports.m_actWrench_[i].data.length() == 6){
        for(int j=0;j<6;j++){
          if(std::isfinite(ports.m_actWrench_[i].data[j])) forceSensors[i]->F()[j] = ports.m_actWrench_[i].data[j];
          else std::cerr << "m_actWrench is not finite!" << std::endl;
        }
      }
    }
  }

  if(ports.m_selfCollisionIn_.isNew()) {
    ports.m_selfCollisionIn_.read();
    selfCollision.resize(ports.m_selfCollision_.data.length());
    for (int i=0; i<selfCollision.size(); i++){
      if(refRobotRaw->link(std::string(ports.m_selfCollision_.data[i].link1)) &&
         std::isfinite(ports.m_selfCollision_.data[i].point1.x) &&
         std::isfinite(ports.m_selfCollision_.data[i].point1.y) &&
         std::isfinite(ports.m_selfCollision_.data[i].point1.z) &&
         refRobotRaw->link(std::string(ports.m_selfCollision_.data[i].link2)) &&
         std::isfinite(ports.m_selfCollision_.data[i].point2.x) &&
         std::isfinite(ports.m_selfCollision_.data[i].point2.y) &&
         std::isfinite(ports.m_selfCollision_.data[i].point2.z) &&
         std::isfinite(ports.m_selfCollision_.data[i].direction21.x) &&
         std::isfinite(ports.m_selfCollision_.data[i].direction21.y) &&
         std::isfinite(ports.m_selfCollision_.data[i].direction21.z) &&
         std::isfinite(ports.m_selfCollision_.data[i].distance)){
        selfCollision[i].link1 = ports.m_selfCollision_.data[i].link1;
        selfCollision[i].point1[0] = ports.m_selfCollision_.data[i].point1.x;
        selfCollision[i].point1[1] = ports.m_selfCollision_.data[i].point1.y;
        selfCollision[i].point1[2] = ports.m_selfCollision_.data[i].point1.z;
        selfCollision[i].link2 = ports.m_selfCollision_.data[i].link2;
        selfCollision[i].point2[0] = ports.m_selfCollision_.data[i].point2.x;
        selfCollision[i].point2[1] = ports.m_selfCollision_.data[i].point2.y;
        selfCollision[i].point2[2] = ports.m_selfCollision_.data[i].point2.z;
        selfCollision[i].direction21[0] = ports.m_selfCollision_.data[i].direction21.x;
        selfCollision[i].direction21[1] = ports.m_selfCollision_.data[i].direction21.y;
        selfCollision[i].direction21[2] = ports.m_selfCollision_.data[i].direction21.z;
        selfCollision[i].distance = ports.m_selfCollision_.data[i].distance;
      }else{
        std::cerr << "m_selfCollision is not finite or has unknown link name!" << std::endl;
        selfCollision.resize(0);
        break;
      }
    }
  }

  if(ports.m_steppableRegionIn_.isNew()){
    ports.m_steppableRegionIn_.read();
    //steppableRegionを送るのは片足支持期のみ
    if (mode.isABCRunning() && // ABC起動中でないと現在支持脚という概念が無い
        ((gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG] && (ports.m_steppableRegion_.data.l_r == auto_stabilizer_msgs::RLEG)) ||
         (gaitParam.footstepNodesList[0].isSupportPhase[LLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && (ports.m_steppableRegion_.data.l_r == auto_stabilizer_msgs::LLEG))) //現在支持脚と計算時支持脚が同じ
        ){
      int swingLeg = gaitParam.footstepNodesList[0].isSupportPhase[RLEG] ? LLEG : RLEG;
      int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;
      cnoid::Position supportPose = gaitParam.genCoords[supportLeg].value(); // TODO. 支持脚のgenCoordsとdstCoordsが異なることは想定していない
      cnoid::Position supportPoseHorizontal = mathutil::orientCoordToAxis(supportPose, cnoid::Vector3::UnitZ());
      steppableRegion.resize(ports.m_steppableRegion_.data.region.length());
      steppableHeight.resize(ports.m_steppableRegion_.data.region.length());
      for (int i=0; i<steppableRegion.size(); i++){
        double heightSum = 0.0;
        std::vector<cnoid::Vector3> vertices;
        for (int j=0; j<ports.m_steppableRegion_.data.region[i].length()/3; j++){
          if(!std::isfinite(ports.m_steppableRegion_.data.region[i][3*j]) || !std::isfinite(ports.m_steppableRegion_.data.region[i][3*j+1]) || !std::isfinite(ports.m_steppableRegion_.data.region[i][3*j+2])){
            std::cerr << "m_steppableRegion is not finite!" << std::endl;
            vertices.clear();
            break;
          }
          cnoid::Vector3 p = supportPoseHorizontal * cnoid::Vector3(ports.m_steppableRegion_.data.region[i][3*j],ports.m_steppableRegion_.data.region[i][3*j+1],ports.m_steppableRegion_.data.region[i][3*j+2]);
          heightSum += p[2];
          p[2] = 0.0;
          vertices.push_back(p);
        }
        double heightAverage = (ports.m_steppableRegion_.data.region[i].length()/3>0) ? heightSum / (ports.m_steppableRegion_.data.region[i].length()/3) : 0;
        steppableRegion[i] = mathutil::calcConvexHull(vertices);
        steppableHeight[i] = heightAverage;
      }
      ports.steppableRegionLastUpdateTime_ = ports.m_qRef_.tm;
    }
  }else{ //ports.m_steppableRegionIn_.isNew()
    if(std::abs(((long long)ports.steppableRegionLastUpdateTime_.sec - (long long)ports.m_qRef_.tm.sec) + 1e-9 * ((long long)ports.steppableRegionLastUpdateTime_.nsec - (long long)ports.m_qRef_.tm.nsec)) > 2.0){ // 2秒間steppableRegionが届いていない.  RTC::Timeはunsigned long型なので、符号付きの型に変換してから引き算
      steppableRegion.clear();
      steppableHeight.clear();
    }
  }

  if(ports.m_landingHeightIn_.isNew()) {
    ports.m_landingHeightIn_.read();
    if(std::isfinite(ports.m_landingHeight_.data.x) && std::isfinite(ports.m_landingHeight_.data.y) && std::isfinite(ports.m_landingHeight_.data.z) && std::isfinite(ports.m_landingHeight_.data.nx) && std::isfinite(ports.m_landingHeight_.data.ny) && std::isfinite(ports.m_landingHeight_.data.nz)){
      cnoid::Vector3 normal = cnoid::Vector3(ports.m_landingHeight_.data.nx, ports.m_landingHeight_.data.ny, ports.m_landingHeight_.data.nz);
      if(normal.norm() > 1.0 - 1e-2 && normal.norm() < 1.0 + 1e-2){ // ノルムがほぼ1
        if(mode.isABCRunning()){ // ABC起動中でないと現在支持脚という概念が無い
          if(ports.m_landingHeight_.data.l_r == auto_stabilizer_msgs::RLEG && gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) { //現在支持脚と計算時支持脚が同じ
            cnoid::Position supportPoseHorizontal = mathutil::orientCoordToAxis(gaitParam.genCoords[RLEG].value(), cnoid::Vector3::UnitZ());
            relLandingHeight = supportPoseHorizontal.translation()[2] + ports.m_landingHeight_.data.z;
            relLandingNormal = supportPoseHorizontal.linear() * normal.normalized();
          }else if(ports.m_landingHeight_.data.l_r == auto_stabilizer_msgs::LLEG && gaitParam.footstepNodesList[0].isSupportPhase[LLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[RLEG]) { //現在支持脚と計算時支持脚が同じ
            cnoid::Position supportPoseHorizontal = mathutil::orientCoordToAxis(gaitParam.genCoords[LLEG].value(), cnoid::Vector3::UnitZ());
            relLandingHeight = supportPoseHorizontal.translation()[2] + ports.m_landingHeight_.data.z;
            relLandingNormal = supportPoseHorizontal.linear() * normal.normalized();
          }
        }
      }else{
        std::cerr << "m_landingHeight's norm != 1 !" << std::endl;
      }
    }else{
      std::cerr << "m_landingHeight is not finite!" << std::endl;
    }
  }

  return qRef_updated;
}

// static function
bool AutoStabilizer::execAutoStabilizer(const AutoStabilizer::ControlMode& mode, GaitParam& gaitParam, double dt, const FootStepGenerator& footStepGenerator, const LegCoordsGenerator& legCoordsGenerator, const RefToGenFrameConverter& refToGenFrameConverter, const ActToGenFrameConverter& actToGenFrameConverter, const ImpedanceController& impedanceController, const Stabilizer& stabilizer, const ExternalForceHandler& externalForceHandler, const FullbodyIKSolver& fullbodyIKSolver,const LegManualController& legManualController, const CmdVelGenerator& cmdVelGenerator) {
  if(mode.isSyncToABCInit()){ // startAutoBalancer直後の初回. gaitParamのリセット
    refToGenFrameConverter.initGenRobot(gaitParam,
                                        gaitParam.genRobot, gaitParam.footMidCoords, gaitParam.genCogVel, gaitParam.genCogAcc);
    externalForceHandler.initExternalForceHandlerOutput(gaitParam,
                                                        gaitParam.omega, gaitParam.l, gaitParam.sbpOffset, gaitParam.genCog);
    impedanceController.initImpedanceOutput(gaitParam,
                                            gaitParam.icEEOffset);
    footStepGenerator.initFootStepNodesList(gaitParam,
                                            gaitParam.footstepNodesList, gaitParam.srcCoords, gaitParam.dstCoordsOrg, gaitParam.remainTimeOrg, gaitParam.swingState, gaitParam.elapsedTime, gaitParam.prevSupportPhase);
    legCoordsGenerator.initLegCoords(gaitParam,
                                     gaitParam.refZmpTraj, gaitParam.genCoords);
    stabilizer.initStabilizerOutput(gaitParam,
                                    gaitParam.stOffsetRootRpy, gaitParam.stTargetZmp, gaitParam.stServoPGainPercentage, gaitParam.stServoDGainPercentage);
  }

  // FootOrigin座標系を用いてrefRobotRawをgenerate frameに投影しrefRobotとする
  refToGenFrameConverter.convertFrame(gaitParam, dt,
                                      gaitParam.refRobot, gaitParam.refEEPose, gaitParam.refEEWrench, gaitParam.refdz, gaitParam.footMidCoords);

  // FootOrigin座標系を用いてactRobotRawをgenerate frameに投影しactRobotとする
  actToGenFrameConverter.convertFrame(gaitParam, dt,
                                      gaitParam.actRobot, gaitParam.actEEPose, gaitParam.actEEWrench, gaitParam.actCogVel);

  // 目標外力に応じてオフセットを計算する
  externalForceHandler.handleExternalForce(gaitParam, mode.isSTRunning(), dt,
                                           gaitParam.omega, gaitParam.l, gaitParam.sbpOffset, gaitParam.actCog);

  // Impedance Controller
  impedanceController.calcImpedanceControl(dt, gaitParam,
                                           gaitParam.icEEOffset, gaitParam.icEETargetPose);

  // Manual Control Modeの足の現在位置をreferenceで上書きする
  legManualController.legManualControl(gaitParam, dt,
                                       gaitParam.genCoords, gaitParam.footstepNodesList, gaitParam.isManualControlMode);

  // CmdVelGenerator
  cmdVelGenerator.calcCmdVel(gaitParam,
                             gaitParam.cmdVel);

  // AutoBalancer
  footStepGenerator.procFootStepNodesList(gaitParam, dt, mode.isSTRunning(),
                                          gaitParam.footstepNodesList, gaitParam.srcCoords, gaitParam.dstCoordsOrg, gaitParam.remainTimeOrg, gaitParam.swingState, gaitParam.elapsedTime, gaitParam.prevSupportPhase, gaitParam.relLandingHeight);
  footStepGenerator.calcFootSteps(gaitParam, dt, mode.isSTRunning(),
                                  gaitParam.debugData, //for log
                                  gaitParam.footstepNodesList);
  legCoordsGenerator.calcLegCoords(gaitParam, dt, mode.isSTRunning(),
                                   gaitParam.refZmpTraj, gaitParam.genCoords, gaitParam.swingState);
  legCoordsGenerator.calcCOMCoords(gaitParam, dt,
                                   gaitParam.genCog, gaitParam.genCogVel, gaitParam.genCogAcc);
  for(int i=0;i<gaitParam.eeName.size();i++){
    if(i<NUM_LEGS) gaitParam.abcEETargetPose[i] = gaitParam.genCoords[i].value();
    else gaitParam.abcEETargetPose[i] = gaitParam.icEETargetPose[i];
  }

  // Stabilizer
  if(mode.isSyncToStopSTInit()){ // stopST直後の初回
    gaitParam.stOffsetRootRpy.setGoal(cnoid::Vector3::Zero(),mode.remainTime());
    for(int i=0;i<gaitParam.genRobot->numJoints();i++){
      if(gaitParam.stServoPGainPercentage[i].getGoal() != 100.0) gaitParam.stServoPGainPercentage[i].setGoal(100.0, mode.remainTime());
      if(gaitParam.stServoDGainPercentage[i].getGoal() != 100.0) gaitParam.stServoDGainPercentage[i].setGoal(100.0, mode.remainTime());
    }
  }
  stabilizer.execStabilizer(gaitParam, dt, mode.isSTRunning(),
                            gaitParam.actRobotTqc, gaitParam.stOffsetRootRpy, gaitParam.stTargetRootPose, gaitParam.stTargetZmp, gaitParam.stEETargetWrench, gaitParam.stServoPGainPercentage, gaitParam.stServoDGainPercentage);

  // FullbodyIKSolver
  fullbodyIKSolver.solveFullbodyIK(dt, gaitParam,// input
                                   gaitParam.genRobot); // output

  return true;
}

// static function
bool AutoStabilizer::writeOutPortData(AutoStabilizer::Ports& ports, const AutoStabilizer::ControlMode& mode, cpp_filters::TwoPointInterpolator<double>& idleToAbcTransitionInterpolator, double dt, const GaitParam& gaitParam){
  if(mode.isSyncToABC()){
    if(mode.isSyncToABCInit()){
      idleToAbcTransitionInterpolator.reset(0.0);
    }
    idleToAbcTransitionInterpolator.setGoal(1.0,mode.remainTime());
    idleToAbcTransitionInterpolator.interpolate(dt);
  }else if(mode.isSyncToIdle()){
    if(mode.isSyncToIdleInit()){
      idleToAbcTransitionInterpolator.reset(1.0);
    }
    idleToAbcTransitionInterpolator.setGoal(0.0,mode.remainTime());
    idleToAbcTransitionInterpolator.interpolate(dt);
  }

  {
    // q
    ports.m_q_.tm = ports.m_qRef_.tm;
    ports.m_q_.data.length(gaitParam.genRobot->numJoints());
    for(int i=0;i<gaitParam.genRobot->numJoints();i++){
      if(mode.now() == AutoStabilizer::ControlMode::MODE_IDLE || !gaitParam.jointControllable[i]){
        double value = gaitParam.refRobotRaw->joint(i)->q();
        if(std::isfinite(value)) ports.m_q_.data[i] = value;
        else std::cerr << "m_q is not finite!" << std::endl;
      }else if(mode.isSyncToABC() || mode.isSyncToIdle()){
        double ratio = idleToAbcTransitionInterpolator.value();
        double value = gaitParam.refRobotRaw->joint(i)->q() * (1.0 - ratio) + gaitParam.genRobot->joint(i)->q() * ratio;
        if(std::isfinite(value)) ports.m_q_.data[i] = value;
        else std::cerr << "m_q is not finite!" << std::endl;
      }else{
        double value = gaitParam.genRobot->joint(i)->q();
        if(std::isfinite(value)) ports.m_q_.data[i] = value;
        else std::cerr << "m_q is not finite!" << std::endl;
      }
    }
    ports.m_qOut_.write();
  }

  {
    // tau
    ports.m_genTau_.tm = ports.m_qRef_.tm;
    ports.m_genTau_.data.length(gaitParam.actRobotTqc->numJoints());
    for(int i=0;i<gaitParam.actRobotTqc->numJoints();i++){
      if(mode.now() == AutoStabilizer::ControlMode::MODE_IDLE || !gaitParam.jointControllable[i]){
        double value = gaitParam.refRobotRaw->joint(i)->u();
        if(std::isfinite(value)) ports.m_genTau_.data[i] = value;
        else std::cerr << "m_genTau is not finite!" << std::endl;
      }else if(mode.isSyncToABC() || mode.isSyncToIdle()){
        double ratio = idleToAbcTransitionInterpolator.value();
        double value = gaitParam.refRobotRaw->joint(i)->u() * (1.0 - ratio) + gaitParam.actRobotTqc->joint(i)->u() * ratio;
        if(std::isfinite(value)) ports.m_genTau_.data[i] = value;
        else std::cerr << "m_genTau is not finite!" << std::endl;
      }else{
        double value = gaitParam.actRobotTqc->joint(i)->u();
        if(std::isfinite(value)) ports.m_genTau_.data[i] = value;
        else std::cerr << "m_genTau is not finite!" << std::endl;
      }
    }
    ports.m_genTauOut_.write();
  }

  {
    // basePose
    cnoid::Position basePose;
    if(mode.now() == AutoStabilizer::ControlMode::MODE_IDLE){
      basePose = gaitParam.refRobotRaw->rootLink()->T();
    }else if(mode.isSyncToABC() || mode.isSyncToIdle()){
      double ratio = idleToAbcTransitionInterpolator.value();
      basePose = mathutil::calcMidCoords(std::vector<cnoid::Position>{gaitParam.refRobotRaw->rootLink()->T(), gaitParam.genRobot->rootLink()->T()},
                                         std::vector<double>{1.0 - ratio, ratio});
    }else{
      basePose = gaitParam.genRobot->rootLink()->T();
    }
    cnoid::Vector3 basePos = basePose.translation();
    cnoid::Matrix3 baseR = basePose.linear();
    cnoid::Vector3 baseRpy = cnoid::rpyFromRot(basePose.linear());

    if(std::isfinite(basePos[0]) && std::isfinite(basePos[1]) && std::isfinite(basePos[2]) &&
       std::isfinite(baseRpy[0]) && std::isfinite(baseRpy[1]) && std::isfinite(baseRpy[2])){

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
    }else{
      std::cerr << "m_genBasePose is not finite!" << std::endl;
    }
  }

  // acc ref
  {
    cnoid::Vector3 genImuAcc = cnoid::Vector3::Zero(); // imu frame
    if(mode.isABCRunning()){
      cnoid::RateGyroSensorPtr imu = gaitParam.genRobot->findDevice<cnoid::RateGyroSensor>("gyrometer"); // genrobot imu
      cnoid::Matrix3 imuR = imu->link()->R() * imu->R_local(); // generate frame
      genImuAcc/*imu frame*/ = imuR.transpose() * gaitParam.genCogAcc/*generate frame*/; // 本当は重心の加速ではなく、関節の加速等を考慮したimuセンサの加速を直接与えたいが、関節角度ベースのinverse-kinematicsを使う以上モデルのimuセンサの位置の加速が不連続なものになることは避けられないので、重心の加速を用いている. この出力の主な用途は歩行時の姿勢推定のため、重心の加速が考慮できればだいたい十分.
    }
    if(std::isfinite(genImuAcc[0]) && std::isfinite(genImuAcc[1]) && std::isfinite(genImuAcc[2])){
      ports.m_genImuAcc_.tm = ports.m_qRef_.tm;
      ports.m_genImuAcc_.data.ax = genImuAcc[0];
      ports.m_genImuAcc_.data.ay = genImuAcc[1];
      ports.m_genImuAcc_.data.az = genImuAcc[2];
      ports.m_genImuAccOut_.write();
    }else{
      std::cerr << "m_genImuAcc is not finite!" << std::endl;
    }
  }

  // Gains
  if(!CORBA::is_nil(ports.m_robotHardwareService0_._ptr()) && //コンシューマにプロバイダのオブジェクト参照がセットされていない(接続されていない)状態
     !ports.m_robotHardwareService0_->_non_existent()){ //プロバイダのオブジェクト参照は割り当てられているが、相手のオブジェクトが非活性化 (RTC は Inactive 状態) になっている状態
    for(int i=0;i<gaitParam.genRobot->numJoints();i++){
      if(mode.now() == AutoStabilizer::ControlMode::MODE_IDLE || !gaitParam.jointControllable[i]){
        // pass
      }else if(mode.isSyncToABC()){
        // pass
      }else if(mode.isSyncToIdle()){
        // pass
      }else{
        // Stabilizerが動いている間にonDeactivated()->onActivated()が呼ばれると、ゲインがもとに戻らない. onDeactivated()->onActivated()が呼ばれるのはサーボオン直前で、通常、サーボオン時にゲインを指令するので、問題ない.
        if(gaitParam.stServoPGainPercentage[i].remain_time() > 0.0 && gaitParam.stServoPGainPercentage[i].current_time() <= dt) { // 補間が始まった初回
          if(std::isfinite(gaitParam.stServoPGainPercentage[i].getGoal()) && std::isfinite(gaitParam.stServoPGainPercentage[i].goal_time())){
            ports.m_robotHardwareService0_->setServoPGainPercentageWithTime(gaitParam.actRobotTqc->joint(i)->name().c_str(),gaitParam.stServoPGainPercentage[i].getGoal(),gaitParam.stServoPGainPercentage[i].goal_time());
          }else{
            std::cerr << "setServoPGainPercentageWithTime is not finite!" << std::endl;
          }
        }
        if(gaitParam.stServoDGainPercentage[i].remain_time() > 0.0 && gaitParam.stServoDGainPercentage[i].current_time() <= dt) { // 補間が始まった初回
          if(std::isfinite(gaitParam.stServoDGainPercentage[i].getGoal()) && std::isfinite(gaitParam.stServoDGainPercentage[i].goal_time())){
            ports.m_robotHardwareService0_->setServoDGainPercentageWithTime(gaitParam.actRobotTqc->joint(i)->name().c_str(),gaitParam.stServoDGainPercentage[i].getGoal(),gaitParam.stServoDGainPercentage[i].goal_time());
          }else{
            std::cerr << "setServoDGainPercentageWithTime is not finite!" << std::endl;
          }
        }
      }
    }
  }

  //landngTarget
  if(mode.isABCRunning() && // ABC起動中でないと支持脚という概念が無い
     gaitParam.footstepNodesList.size() >= 2 &&
     ((gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) ||
      (!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG])) && // 今が片足支持
     (gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[1].isSupportPhase[LLEG]) // 次が両足支持
     ) {
    int supportLeg = gaitParam.footstepNodesList[0].isSupportPhase[RLEG] ? RLEG : LLEG;
    int swingLeg = gaitParam.footstepNodesList[0].isSupportPhase[RLEG] ? LLEG : RLEG;
    ports.m_landingTarget_.tm = ports.m_qRef_.tm;
    cnoid::Position supportPoseHorizontal = mathutil::orientCoordToAxis(gaitParam.genCoords[supportLeg].value(), cnoid::Vector3::UnitZ());
    ports.m_landingTarget_.data.x = (supportPoseHorizontal.inverse() * gaitParam.footstepNodesList[0].dstCoords[swingLeg].translation())[0];
    ports.m_landingTarget_.data.y = (supportPoseHorizontal.inverse() * gaitParam.footstepNodesList[0].dstCoords[swingLeg].translation())[1];
    ports.m_landingTarget_.data.z = (supportPoseHorizontal.inverse() * gaitParam.footstepNodesList[0].dstCoords[swingLeg].translation())[2];
    ports.m_landingTarget_.data.l_r = gaitParam.footstepNodesList[0].isSupportPhase[RLEG] ? auto_stabilizer_msgs::RLEG : auto_stabilizer_msgs::LLEG;
    ports.m_landingTargetOut_.write();
  }

  // actEEPose actEEWrench (for wholebodymasterslave)
  if(mode.isABCRunning()){
    for(int i=0;i<gaitParam.eeName.size();i++){
      ports.m_actEEPose_[i].tm = ports.m_qRef_.tm;
      ports.m_actEEPose_[i].data.position.x = gaitParam.actEEPose[i].translation()[0];
      ports.m_actEEPose_[i].data.position.y = gaitParam.actEEPose[i].translation()[1];
      ports.m_actEEPose_[i].data.position.z = gaitParam.actEEPose[i].translation()[2];
      cnoid::Vector3 rpy = cnoid::rpyFromRot(gaitParam.actEEPose[i].linear());
      ports.m_actEEPose_[i].data.orientation.r = rpy[0];
      ports.m_actEEPose_[i].data.orientation.p = rpy[1];
      ports.m_actEEPose_[i].data.orientation.y = rpy[2];
      ports.m_actEEPoseOut_[i]->write();
    }
    for(int i=0;i<gaitParam.eeName.size();i++){
      ports.m_actEEWrench_[i].tm = ports.m_qRef_.tm;
      ports.m_actEEWrench_[i].data.length(6);
      for(int j=0;j<6;j++) ports.m_actEEWrench_[i].data[j] = gaitParam.actEEWrench[i][j];
      ports.m_actEEWrenchOut_[i]->write();
    }
  }

  // only for logger. (IDLE時の出力や、モード遷移時の連続性はてきとうで良い)
  if(mode.isABCRunning()){
    ports.m_genCog_.tm = ports.m_qRef_.tm;
    ports.m_genCog_.data.x = gaitParam.genCog[0];
    ports.m_genCog_.data.y = gaitParam.genCog[1];
    ports.m_genCog_.data.z = gaitParam.genCog[2];
    ports.m_genCogOut_.write();
    cnoid::Vector3 genDcm = gaitParam.genCog + gaitParam.genCogVel / gaitParam.omega;
    ports.m_genDcm_.tm = ports.m_qRef_.tm;
    ports.m_genDcm_.data.x = genDcm[0];
    ports.m_genDcm_.data.y = genDcm[1];
    ports.m_genDcm_.data.z = genDcm[2];
    ports.m_genDcmOut_.write();
    ports.m_genZmp_.tm = ports.m_qRef_.tm;
    ports.m_genZmp_.data.x = gaitParam.refZmpTraj[0].getStart()[0];
    ports.m_genZmp_.data.y = gaitParam.refZmpTraj[0].getStart()[1];
    ports.m_genZmp_.data.z = gaitParam.refZmpTraj[0].getStart()[2];
    ports.m_genZmpOut_.write();
    ports.m_tgtZmp_.tm = ports.m_qRef_.tm;
    ports.m_tgtZmp_.data.x = gaitParam.stTargetZmp[0];
    ports.m_tgtZmp_.data.y = gaitParam.stTargetZmp[1];
    ports.m_tgtZmp_.data.z = gaitParam.stTargetZmp[2];
    ports.m_tgtZmpOut_.write();
    ports.m_actCog_.tm = ports.m_qRef_.tm;
    ports.m_actCog_.data.x = gaitParam.actCog[0];
    ports.m_actCog_.data.y = gaitParam.actCog[1];
    ports.m_actCog_.data.z = gaitParam.actCog[2];
    ports.m_actCogOut_.write();
    cnoid::Vector3 actDcm = gaitParam.actCog + gaitParam.actCogVel.value() / gaitParam.omega;
    ports.m_actDcm_.tm = ports.m_qRef_.tm;
    ports.m_actDcm_.data.x = actDcm[0];
    ports.m_actDcm_.data.y = actDcm[1];
    ports.m_actDcm_.data.z = actDcm[2];
    ports.m_actDcmOut_.write();
    ports.m_dstLandingPos_.tm = ports.m_qRef_.tm;
    ports.m_dstLandingPos_.data.length(6);
    ports.m_dstLandingPos_.data[0] = gaitParam.footstepNodesList[0].dstCoords[RLEG].translation()[0];
    ports.m_dstLandingPos_.data[1] = gaitParam.footstepNodesList[0].dstCoords[RLEG].translation()[1];
    ports.m_dstLandingPos_.data[2] = gaitParam.footstepNodesList[0].dstCoords[RLEG].translation()[2];
    ports.m_dstLandingPos_.data[3] = gaitParam.footstepNodesList[0].dstCoords[LLEG].translation()[0];
    ports.m_dstLandingPos_.data[4] = gaitParam.footstepNodesList[0].dstCoords[LLEG].translation()[1];
    ports.m_dstLandingPos_.data[5] = gaitParam.footstepNodesList[0].dstCoords[LLEG].translation()[2];
    ports.m_dstLandingPosOut_.write();
    ports.m_remainTime_.tm = ports.m_qRef_.tm;
    ports.m_remainTime_.data.length(1);
    ports.m_remainTime_.data[0] = gaitParam.footstepNodesList[0].remainTime;
    ports.m_remainTimeOut_.write();
    ports.m_genCoords_.tm = ports.m_qRef_.tm;
    ports.m_genCoords_.data.length(12);
    for (int i=0; i<3; i++) {
      ports.m_genCoords_.data[0+i] = gaitParam.genCoords[RLEG].value().translation()[i];
      ports.m_genCoords_.data[3+i] = gaitParam.genCoords[LLEG].value().translation()[i];
      ports.m_genCoords_.data[6+i] = gaitParam.genCoords[RLEG].getGoal().translation()[i];
      ports.m_genCoords_.data[9+i] = gaitParam.genCoords[LLEG].getGoal().translation()[i];
    }
    ports.m_genCoordsOut_.write();
    {
      ports.m_captureRegion_.tm = ports.m_qRef_.tm;
      int sum = 0;
      for (int i=0; i<gaitParam.debugData.capturableHulls.size(); i++) sum+=gaitParam.debugData.capturableHulls[i].size();
      ports.m_captureRegion_.data.length(sum*2);
      int index = 0;
      for (int i=0; i<gaitParam.debugData.capturableHulls.size(); i++) {
        for (int j=0; j<gaitParam.debugData.capturableHulls[i].size(); j++) {
          ports.m_captureRegion_.data[index+0] = gaitParam.debugData.capturableHulls[i][j][0];
          ports.m_captureRegion_.data[index+1] = gaitParam.debugData.capturableHulls[i][j][1];
          index+=2;
        }
      }
      ports.m_captureRegionOut_.write();
    }
    {
      ports.m_steppableRegionLog_.tm = ports.m_qRef_.tm;
      ports.m_steppableRegionNumLog_.tm = ports.m_qRef_.tm;
      int sum = 0;
      for (int i=0; i<gaitParam.steppableRegion.size(); i++) sum+=gaitParam.steppableRegion[i].size();
      ports.m_steppableRegionLog_.data.length(sum*2);
      ports.m_steppableRegionNumLog_.data.length(gaitParam.steppableRegion.size());
      int index=0;
      for (int i=0; i<gaitParam.steppableRegion.size(); i++) {
        for (int j=0; j<gaitParam.steppableRegion[i].size(); j++) {
          ports.m_steppableRegionLog_.data[index+0] = gaitParam.steppableRegion[i][j][0];
          ports.m_steppableRegionLog_.data[index+1] = gaitParam.steppableRegion[i][j][1];
          index+=2;
        }
        ports.m_steppableRegionNumLog_.data[i] = gaitParam.steppableRegion[i].size();
      }
      ports.m_steppableRegionLogOut_.write();
      ports.m_steppableRegionNumLogOut_.write();
    }
    ports.m_strideLimitationHull_.tm = ports.m_qRef_.tm;
    ports.m_strideLimitationHull_.data.length(gaitParam.debugData.strideLimitationHull.size()*2);
    for (int i=0; i<gaitParam.debugData.strideLimitationHull.size(); i++) {
      ports.m_strideLimitationHull_.data[i*2+0] = gaitParam.debugData.strideLimitationHull[i][0];
      ports.m_strideLimitationHull_.data[i*2+1] = gaitParam.debugData.strideLimitationHull[i][1];
    }
    ports.m_strideLimitationHullOut_.write();
    ports.m_cpViewerLog_.tm = ports.m_qRef_.tm;
    ports.m_cpViewerLog_.data.length(gaitParam.debugData.cpViewerLog.size());
    for (int i=0; i<gaitParam.debugData.cpViewerLog.size(); i++) {
      ports.m_cpViewerLog_.data[i] = gaitParam.debugData.cpViewerLog[i];
    }
    ports.m_cpViewerLogOut_.write();
    for(int i=0;i<gaitParam.eeName.size();i++){
      ports.m_tgtEEWrench_[i].tm = ports.m_qRef_.tm;
      ports.m_tgtEEWrench_[i].data.length(6);
      for(int j=0;j<6;j++) ports.m_tgtEEWrench_[i].data[j] = gaitParam.stEETargetWrench[i][j];
      ports.m_tgtEEWrenchOut_[i]->write();
    }
  }

  return true;
}

RTC::ReturnCode_t AutoStabilizer::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);

  std::string instance_name = std::string(this->m_profile.instance_name);
  this->loop_++;

  if(!AutoStabilizer::readInPortData(this->dt_, this->gaitParam_, this->mode_, this->ports_, this->gaitParam_.refRobotRaw, this->gaitParam_.actRobotRaw, this->gaitParam_.refEEWrenchOrigin, this->gaitParam_.refEEPoseRaw, this->gaitParam_.selfCollision, this->gaitParam_.steppableRegion, this->gaitParam_.steppableHeight, this->gaitParam_.relLandingHeight, this->gaitParam_.relLandingNormal)) return RTC::RTC_OK;  // qRef が届かなければ何もしない

  this->mode_.update(this->dt_);
  this->gaitParam_.update(this->dt_);
  this->refToGenFrameConverter_.update(this->dt_);
  this->fullbodyIKSolver_.update(this->dt_);

  if(this->mode_.isABCRunning()) {
    if(this->mode_.isSyncToABCInit()){ // startAutoBalancer直後の初回. 内部パラメータのリセット
      this->gaitParam_.reset();
      this->refToGenFrameConverter_.reset();
      this->actToGenFrameConverter_.reset();
      this->externalForceHandler_.reset();
      this->footStepGenerator_.reset();
      this->impedanceController_.reset();
      this->fullbodyIKSolver_.reset();
    }
    AutoStabilizer::execAutoStabilizer(this->mode_, this->gaitParam_, this->dt_, this->footStepGenerator_, this->legCoordsGenerator_, this->refToGenFrameConverter_, this->actToGenFrameConverter_, this->impedanceController_, this->stabilizer_,this->externalForceHandler_, this->fullbodyIKSolver_, this->legManualController_, this->cmdVelGenerator_);
  }

  AutoStabilizer::writeOutPortData(this->ports_, this->mode_, this->idleToAbcTransitionInterpolator_, this->dt_, this->gaitParam_);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t AutoStabilizer::onActivated(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  this->mode_.reset();
  this->idleToAbcTransitionInterpolator_.reset(0.0);
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
  if(this->mode_.isABCRunning()){
    if(std::isfinite(x) && std::isfinite(y) && std::isfinite(th)){
      return this->footStepGenerator_.goPos(this->gaitParam_, x, y, th,
                                            this->gaitParam_.footstepNodesList);
    }else{
      std::cerr << "goPos is not finite!" << std::endl;
      return false;
    }
  }else{
    return false;
  }
}
bool AutoStabilizer::goVelocity(const double& vx, const double& vy, const double& vth){
  std::lock_guard<std::mutex> guard(this->mutex_);
  if(this->mode_.isABCRunning()){
    if(std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vth)){
      this->cmdVelGenerator_.refCmdVel[0] = vx;
      this->cmdVelGenerator_.refCmdVel[1] = vy;
      this->cmdVelGenerator_.refCmdVel[2] = vth / 180.0 * M_PI;
      this->footStepGenerator_.isGoVelocityMode = true;
      return true;
    }else{
      std::cerr << "goVelocity is not finite!" << std::endl;
      return false;
    }
  }else{
    return false;
  }
}
bool AutoStabilizer::goStop(){
  std::lock_guard<std::mutex> guard(this->mutex_);
  if(this->mode_.isABCRunning() && this->footStepGenerator_.isGoVelocityMode){ // this->footStepGenerator_.isGoVelocityMode時のみ行う. goStopが呼ばれて、staticになる前にgoStopが再度呼ばれることが繰り返されると、止まらないので
    this->cmdVelGenerator_.refCmdVel.setZero();
    this->footStepGenerator_.isGoVelocityMode = false;
    this->footStepGenerator_.goStop(this->gaitParam_,
                                    this->gaitParam_.footstepNodesList);
    return true;
  }else{
    return false;
  }
}
bool AutoStabilizer::jumpTo(const double& x, const double& y, const double& z, const double& ts, const double& tf){
  std::lock_guard<std::mutex> guard(this->mutex_);
  return true;
}

bool AutoStabilizer::setFootSteps(const OpenHRP::AutoStabilizerService::FootstepSequence& fs){
  OpenHRP::AutoStabilizerService::StepParamSequence sps;
  sps.length(fs.length());
  for(int i=0;i<fs.length();i++){
    sps[i].step_height = this->footStepGenerator_.defaultStepHeight;
    sps[i].step_time = this->footStepGenerator_.defaultStepTime;
    sps[i].swing_end = false;
  }
  return this->setFootStepsWithParam(fs, sps); // この中でmutexをとるので、setFootSteps関数ではmutexはとらない
}

bool AutoStabilizer::setFootStepsWithParam(const OpenHRP::AutoStabilizerService::FootstepSequence& fs, const OpenHRP::AutoStabilizerService::StepParamSequence& sps){
  std::lock_guard<std::mutex> guard(this->mutex_);
  if(this->mode_.isABCRunning()){
    std::vector<FootStepGenerator::StepNode> footsteps;
    if(fs.length() != sps.length()){
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] fs.length() != sps.length()" << "\x1b[39m" << std::endl;
      return false;
    }
    for(int i=0;i<fs.length();i++){
      FootStepGenerator::StepNode stepNode;
      if(std::string(fs[i].leg) == "rleg") stepNode.l_r = RLEG;
      else if(std::string(fs[i].leg) == "lleg") stepNode.l_r = LLEG;
      else {
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] leg name [" << fs[i].leg << "] is invalid" << "\x1b[39m" << std::endl;
        return false;
      }
      if(!std::isfinite(fs[i].pos[0]) || !std::isfinite(fs[i].pos[1]) || !std::isfinite(fs[i].pos[2]) ||
         !std::isfinite(fs[i].rot[0]) || !std::isfinite(fs[i].rot[1]) || !std::isfinite(fs[i].rot[2]) || !std::isfinite(fs[i].rot[3]) ||
         !std::isfinite(sps[i].step_height) || !std::isfinite(sps[i].step_time)){
        std::cerr << "setFootStepsWithParam is not finite!" << std::endl;
        return false;
      }
      stepNode.coords.translation() = cnoid::Vector3(fs[i].pos[0],fs[i].pos[1],fs[i].pos[2]);
      stepNode.coords.linear() = Eigen::Quaterniond(fs[i].rot[0],fs[i].rot[1],fs[i].rot[2],fs[i].rot[3]).toRotationMatrix();
      stepNode.stepHeight = sps[i].step_height;
      stepNode.stepTime = sps[i].step_time;
      stepNode.swingEnd = sps[i].swing_end;
      footsteps.push_back(stepNode);
    }
    return this->footStepGenerator_.setFootSteps(this->gaitParam_, footsteps, // input
                                                 this->gaitParam_.footstepNodesList); // output
  }else{
    return false;
  }
}
void AutoStabilizer::waitFootSteps(){
  while (this->mode_.isABCRunning() && !this->gaitParam_.isStatic()) usleep(1000);
  usleep(1000);
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
    std::cerr << "[" << this->m_profile.instance_name << "] auto balancer is already started" << std::endl;
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
    std::cerr << "[" << this->m_profile.instance_name << "] auto balancer is already stopped or stabilizer is running" << std::endl;
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
    std::cerr << "[" << this->m_profile.instance_name << "] Please start AutoBalancer" << std::endl;
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
    std::cerr << "[" << this->m_profile.instance_name << "] Please start AutoBalancer" << std::endl;
    return false;
  }
}

bool AutoStabilizer::startImpedanceController(const std::string& i_name){
  std::lock_guard<std::mutex> guard(this->mutex_);
  if(this->mode_.isABCRunning()){
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      if(this->gaitParam_.eeName[i] != i_name) continue;
      if(this->impedanceController_.isImpedanceMode[i]) {
        std::cerr << "[" << this->m_profile.instance_name << "] Impedance control [" << i_name << "] is already started" << std::endl;
        return false;
      }
      std::cerr << "[" << this->m_profile.instance_name << "] Start impedance control [" << i_name << "]" << std::endl;
      this->impedanceController_.isImpedanceMode[i] = true;
      return true;
    }
    std::cerr << "[" << this->m_profile.instance_name << "] Could not found impedance controller param [" << i_name << "]" << std::endl;
    return false;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] Please start AutoBalancer" << std::endl;
    return false;
  }
}

bool AutoStabilizer::stopImpedanceController(const std::string& i_name){
  std::lock_guard<std::mutex> guard(this->mutex_);
  if(this->mode_.isABCRunning()){
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      if(this->gaitParam_.eeName[i] != i_name) continue;
      if(!this->impedanceController_.isImpedanceMode[i]) {
        std::cerr << "[" << this->m_profile.instance_name << "] Impedance control [" << i_name << "] is already stopped" << std::endl;
        return false;
      }
      std::cerr << "[" << this->m_profile.instance_name << "] Stop impedance control [" << i_name << "]" << std::endl;
      this->impedanceController_.isImpedanceMode[i] = false;
      this->gaitParam_.icEEOffset[i].setGoal(cnoid::Vector6::Zero(), 2.0);
      return true;
    }
    std::cerr << "[" << this->m_profile.instance_name << "] Could not found impedance controller param [" << i_name << "]" << std::endl;
    return false;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] Please start AutoBalancer" << std::endl;
    return false;
  }
}
bool AutoStabilizer::startWholeBodyMasterSlave(void){
  std::lock_guard<std::mutex> guard(this->mutex_);
  if(this->mode_.isABCRunning()){
    if(this->refToGenFrameConverter_.solveFKMode.getGoal() == 0.0){
      std::cerr << "[" << this->m_profile.instance_name << "] WholeBodyMasterSlave is already started" << std::endl;
      return false;
    }
    if(std::abs(((long long)this->ports_.refEEPoseLastUpdateTime_.sec - (long long)this->ports_.m_qRef_.tm.sec) + 1e-9 * ((long long)this->ports_.refEEPoseLastUpdateTime_.nsec - (long long)this->ports_.m_qRef_.tm.nsec)) > 1.0) { // 最新のm_refEEPose_が1秒以上前. master sideが立ち上がっていないので、姿勢の急変を引き起こし危険. RTC::Timeはunsigned long型なので、符号付きの型に変換してから引き算
      std::cerr << "[" << this->m_profile.instance_name << "] Please start master side" << std::endl;
      return false;
    }
    this->refToGenFrameConverter_.solveFKMode.setGoal(0.0, 5.0); // 5秒で遷移
    std::cerr << "[" << this->m_profile.instance_name << "] Start WholeBodyMasterSlave" << std::endl;
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] Please start AutoBalancer" << std::endl;
    return false;
  }
}
bool AutoStabilizer::stopWholeBodyMasterSlave(void){
  std::lock_guard<std::mutex> guard(this->mutex_);
  if(this->mode_.isABCRunning()){
    if(this->refToGenFrameConverter_.solveFKMode.getGoal() == 1.0){
      std::cerr << "[" << this->m_profile.instance_name << "] WholeBodyMasterSlave is already stopped" << std::endl;
      return false;
    }
    this->refToGenFrameConverter_.solveFKMode.setGoal(1.0, 5.0); // 5秒で遷移
    std::cerr << "[" << this->m_profile.instance_name << "] Stop WholeBodyMasterSlave" << std::endl;
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] Please start AutoBalancer" << std::endl;
    return false;
  }
}

bool AutoStabilizer::setAutoStabilizerParam(const OpenHRP::AutoStabilizerService::AutoStabilizerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);

  // ignore i_param.ee_name
  if(this->mode_.now() == ControlMode::MODE_IDLE){
    for(int i=0;i<this->gaitParam_.jointControllable.size();i++) this->gaitParam_.jointControllable[i] = false;
    for(int i=0;i<i_param.controllable_joints.length();i++){
      cnoid::LinkPtr joint = this->gaitParam_.genRobot->link(std::string(i_param.controllable_joints[i]));
      if(joint) this->gaitParam_.jointControllable[joint->jointId()] = true;
    }
  }
  this->mode_.abc_start_transition_time = std::max(i_param.abc_start_transition_time, 0.01);
  this->mode_.abc_stop_transition_time = std::max(i_param.abc_stop_transition_time, 0.01);
  this->mode_.st_start_transition_time = std::max(i_param.st_start_transition_time, 0.01);
  this->mode_.st_stop_transition_time = std::max(i_param.st_stop_transition_time, 0.01);

  if(i_param.default_zmp_offsets.length() == NUM_LEGS){
    for(int i=0;i<NUM_LEGS; i++) {
      if(i_param.default_zmp_offsets[i].length() == 2){
        cnoid::Vector3 copOffset = cnoid::Vector3::Zero();
        for(int j=0;j<2;j++) copOffset[j] = i_param.default_zmp_offsets[i][j];
        if(copOffset != this->gaitParam_.copOffset[i].getGoal()) {
          if(this->mode_.isABCRunning()) this->gaitParam_.copOffset[i].setGoal(copOffset, 2.0); // 2.0[s]で補間
          else this->gaitParam_.copOffset[i].reset(copOffset);
        }
      }
    }
  }
  if(i_param.leg_hull.length() == NUM_LEGS){
    for(int i=0;i<NUM_LEGS;i++){
      std::vector<cnoid::Vector3> vertices;
      for(int j=0;j<i_param.leg_hull[i].length();j++) vertices.emplace_back(i_param.leg_hull[i][j][0],i_param.leg_hull[i][j][1],0.0);
      vertices = mathutil::calcConvexHull(vertices);
      if(vertices.size() > 0) this->gaitParam_.legHull[i] = vertices;
    }
  }
  if(i_param.leg_default_translate_pos.length() == NUM_LEGS){
    for(int i=0;i<NUM_LEGS; i++) {
      cnoid::Vector3 defaultTranslatePos = cnoid::Vector3::Zero();
      defaultTranslatePos[1] = i_param.leg_default_translate_pos[i];
      if(defaultTranslatePos != this->gaitParam_.defaultTranslatePos[i].getGoal()){
        if(this->mode_.isABCRunning()) this->gaitParam_.defaultTranslatePos[i].setGoal(defaultTranslatePos, 2.0); // 2.0[s]で補間
        this->gaitParam_.defaultTranslatePos[i].reset(defaultTranslatePos);
      }
    }
  }
  if(i_param.is_manual_control_mode.length() == NUM_LEGS && (!i_param.is_manual_control_mode[RLEG] || !i_param.is_manual_control_mode[LLEG])){
    for(int i=0;i<NUM_LEGS; i++) {
      if(this->mode_.isABCRunning()) {
        if(i_param.is_manual_control_mode[i] != (this->gaitParam_.isManualControlMode[i].getGoal() == 1.0)){
          if(i_param.is_manual_control_mode[i]){
            if(this->gaitParam_.isStatic() && !this->gaitParam_.footstepNodesList[0].isSupportPhase[i]) {
              this->gaitParam_.isManualControlMode[i].setGoal(1.0, 2.0); // 2.0[s]で遷移
            }
          }else{
            this->gaitParam_.isManualControlMode[i].setGoal(0.0, 2.0); // 2.0[s]で遷移
          }
        }
      }else{
        this->gaitParam_.isManualControlMode[i].reset(i_param.is_manual_control_mode[i] ? 1.0 : 0.0);
      }
    }
  }

  if((this->refToGenFrameConverter_.handFixMode.getGoal() == 1.0) != i_param.is_hand_fix_mode) {
    if(this->mode_.isABCRunning()) this->refToGenFrameConverter_.handFixMode.setGoal(i_param.is_hand_fix_mode ? 1.0 : 0.0, 1.0); // 1.0[s]で補間
    else this->refToGenFrameConverter_.handFixMode.reset(i_param.is_hand_fix_mode ? 1.0 : 0.0);
  }
  if(i_param.reference_frame.length() == NUM_LEGS && (i_param.reference_frame[RLEG] || i_param.reference_frame[LLEG])){
    for(int i=0;i<NUM_LEGS;i++) {
      if(this->mode_.isABCRunning()) this->refToGenFrameConverter_.refFootOriginWeight[i].setGoal(i_param.reference_frame[i] ? 1.0 : 0.0, 1.0); // 1.0[s]で補間
      else this->refToGenFrameConverter_.refFootOriginWeight[i].reset(i_param.reference_frame[i] ? 1.0 : 0.0);
    }
  }

  if(i_param.rpy_offset.length() == 3){
    for(int i=0;i<3;i++) {
      this->actToGenFrameConverter_.rpyOffset[i] = i_param.rpy_offset[i];
    }
  }

  this->externalForceHandler_.useDisturbanceCompensation = i_param.use_disturbance_compensation;
  this->externalForceHandler_.disturbanceCompensationTimeConst = std::max(i_param.disturbance_compensation_time_const, 0.01);
  this->externalForceHandler_.disturbanceCompensationStepNum = std::max(i_param.disturbance_compensation_step_num, 1);
  this->externalForceHandler_.disturbanceCompensationLimit = std::max(i_param.disturbance_compensation_limit, 0.0);

  if(i_param.impedance_M_p.length() == this->gaitParam_.eeName.size() &&
     i_param.impedance_D_p.length() == this->gaitParam_.eeName.size() &&
     i_param.impedance_K_p.length() == this->gaitParam_.eeName.size() &&
     i_param.impedance_M_r.length() == this->gaitParam_.eeName.size() &&
     i_param.impedance_D_r.length() == this->gaitParam_.eeName.size() &&
     i_param.impedance_K_r.length() == this->gaitParam_.eeName.size() &&
     i_param.impedance_force_gain.length() == this->gaitParam_.eeName.size() &&
     i_param.impedance_moment_gain.length() == this->gaitParam_.eeName.size() &&
     i_param.impedance_pos_compensation_limit.length() == this->gaitParam_.eeName.size() &&
     i_param.impedance_rot_compensation_limit.length() == this->gaitParam_.eeName.size()){
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      if(i_param.impedance_M_p[i].length() == 3 &&
         i_param.impedance_D_p[i].length() == 3 &&
         i_param.impedance_K_p[i].length() == 3 &&
         i_param.impedance_M_r[i].length() == 3 &&
         i_param.impedance_D_r[i].length() == 3 &&
         i_param.impedance_K_r[i].length() == 3 &&
         i_param.impedance_force_gain[i].length() == 3 &&
         i_param.impedance_moment_gain[i].length() == 3 &&
         i_param.impedance_pos_compensation_limit[i].length() == 3 &&
         i_param.impedance_rot_compensation_limit[i].length() == 3){
        for(int j=0;j<3;j++){
          this->impedanceController_.M[i][j] = std::max(i_param.impedance_M_p[i][j], 0.0);
          this->impedanceController_.D[i][j] = std::max(i_param.impedance_D_p[i][j], 0.0);
          this->impedanceController_.K[i][j] = std::max(i_param.impedance_K_p[i][j], 0.0);
          this->impedanceController_.M[i][3+j] = std::max(i_param.impedance_M_r[i][j], 0.0);
          this->impedanceController_.D[i][3+j] = std::max(i_param.impedance_D_r[i][j], 0.0);
          this->impedanceController_.K[i][3+j] = std::max(i_param.impedance_K_r[i][j], 0.0);
          this->impedanceController_.wrenchGain[i][j] = std::max(i_param.impedance_force_gain[i][j], 0.0);
          this->impedanceController_.wrenchGain[i][3+j] = std::max(i_param.impedance_moment_gain[i][j], 0.0);
          if(!this->impedanceController_.isImpedanceMode[i]){
            this->impedanceController_.compensationLimit[i][j] = std::max(i_param.impedance_pos_compensation_limit[i][j], 0.0);
            this->impedanceController_.compensationLimit[i][3+j] = std::max(i_param.impedance_rot_compensation_limit[i][j], 0.0);
          }
        }
      }
    }
  }

  this->cmdVelGenerator_.isGraspLessManipMode = i_param.graspless_manip_mode;
  {
    std::vector<double> grasplessManipArm;
    for(int i=0;i<i_param.graspless_manip_arm.length();i++){
      for(int j=0;j<this->gaitParam_.eeName.size();j++){
        if(this->gaitParam_.eeName[j] == std::string(i_param.graspless_manip_arm[i])){
          grasplessManipArm.push_back(j);
          break;
        }
      }
    }
    if(grasplessManipArm.size() <= 2) this->cmdVelGenerator_.graspLessManipArm = grasplessManipArm;
  }
  if(i_param.graspless_manip_time_const.length() == 3){
    for(int i=0;i<3;i++){
      this->cmdVelGenerator_.graspLessManipTimeConst[i] = std::max(i_param.graspless_manip_time_const[i], 0.01);
    }
  }

  this->footStepGenerator_.legCollisionMargin = std::max(i_param.leg_collision_margin, 0.0);
  this->footStepGenerator_.defaultStepTime = std::max(i_param.default_step_time, 0.01);
  if(i_param.default_stride_limitation_max_theta.length() == NUM_LEGS){
    for(int i=0;i<NUM_LEGS;i++){
      this->footStepGenerator_.defaultStrideLimitationMaxTheta[i] = std::max(i_param.default_stride_limitation_max_theta[i], 0.0);
    }
  }
  if(i_param.default_stride_limitation_min_theta.length() == NUM_LEGS){
    for(int i=0;i<NUM_LEGS;i++){
      this->footStepGenerator_.defaultStrideLimitationMinTheta[i] = std::min(i_param.default_stride_limitation_min_theta[i], 0.0);
    }
  }
  if(i_param.default_stride_limitation.length() == NUM_LEGS){
    for(int i=0;i<NUM_LEGS;i++){
      std::vector<cnoid::Vector3> vertices;
      for(int j=0;j<i_param.default_stride_limitation[i].length();j++) vertices.emplace_back(i_param.default_stride_limitation[i][j][0],i_param.default_stride_limitation[i][j][1],0.0);
      vertices = mathutil::calcConvexHull(vertices);
      if(vertices.size() > 0) this->footStepGenerator_.defaultStrideLimitationHull[i] = vertices;
    }
  }
  this->footStepGenerator_.defaultDoubleSupportRatio = std::min(std::max(i_param.default_double_support_ratio, 0.01), 0.99);
  this->footStepGenerator_.defaultStepHeight = std::max(i_param.default_step_height, 0.0);
  this->footStepGenerator_.goVelocityStepNum = std::max(i_param.go_velocity_step_num, 1);
  this->footStepGenerator_.isModifyFootSteps = i_param.modify_footsteps;
  this->footStepGenerator_.overwritableRemainTime = std::max(i_param.overwritable_remain_time, std::max(i_param.swing_trajectory_delay_time_offset, 0.0));
  this->footStepGenerator_.overwritableMinTime = std::max(i_param.overwritable_min_time, 0.01);
  this->footStepGenerator_.overwritableMinStepTime = std::max(i_param.overwritable_min_step_time, 0.01);
  this->footStepGenerator_.overwritableMaxStepTime = std::max(i_param.overwritable_max_step_time, this->footStepGenerator_.overwritableMinStepTime);
  this->footStepGenerator_.overwritableMaxSwingVelocity = std::max(i_param.overwritable_max_swing_velocity, 0.0);
  if(i_param.safe_leg_hull.length() == NUM_LEGS){
    for(int i=0;i<NUM_LEGS;i++){
      std::vector<cnoid::Vector3> vertices;
      for(int j=0;j<i_param.safe_leg_hull[i].length();j++) vertices.emplace_back(i_param.safe_leg_hull[i][j][0],i_param.safe_leg_hull[i][j][1],0.0);
      vertices = mathutil::calcConvexHull(vertices);
      if(vertices.size() > 0) this->footStepGenerator_.safeLegHull[i] = vertices;
    }
  }
  if(i_param.overwritable_stride_limitation_max_theta.length() == NUM_LEGS){
    for(int i=0;i<NUM_LEGS;i++){
      this->footStepGenerator_.overwritableStrideLimitationMaxTheta[i] = std::max(i_param.overwritable_stride_limitation_max_theta[i], 0.0);
    }
  }
  if(i_param.overwritable_stride_limitation_min_theta.length() == NUM_LEGS){
    for(int i=0;i<NUM_LEGS;i++){
      this->footStepGenerator_.overwritableStrideLimitationMinTheta[i] = std::min(i_param.overwritable_stride_limitation_min_theta[i], 0.0);
    }
  }
  if(!this->mode_.isABCRunning() || this->gaitParam_.isStatic()){
    if(i_param.overwritable_stride_limitation.length() == NUM_LEGS){
      for(int i=0;i<NUM_LEGS;i++){
        std::vector<cnoid::Vector3> vertices;
        for(int j=0;j<i_param.overwritable_stride_limitation[i].length();j++) vertices.emplace_back(i_param.overwritable_stride_limitation[i][j][0],i_param.overwritable_stride_limitation[i][j][1],0.0);
        vertices = mathutil::calcConvexHull(vertices);
        if(vertices.size() > 0) this->footStepGenerator_.overwritableStrideLimitationHull[i] = vertices;
      }
    }
  }
  this->footStepGenerator_.overwritableMaxLandingHeight = i_param.overwritable_max_landing_height;
  this->footStepGenerator_.overwritableMinLandingHeight = std::min(i_param.overwritable_min_landing_height, this->footStepGenerator_.overwritableMaxLandingHeight);
  this->footStepGenerator_.overwritableMaxGenGroundZVelocity = std::max(i_param.overwritable_max_gen_ground_z_velocity, 0.01);
  this->footStepGenerator_.overwritableMaxSrcGroundZVelocity = std::max(i_param.overwritable_max_src_ground_z_velocity, 0.01);
  this->footStepGenerator_.contactDetectionThreshold = i_param.contact_detection_threshold;
  this->footStepGenerator_.contactModificationThreshold = std::max(i_param.contact_modification_threshold, 0.0);
  this->footStepGenerator_.isEmergencyStepMode = i_param.is_emergency_step_mode;
  this->footStepGenerator_.isStableGoStopMode = i_param.is_stable_go_stop_mode;
  this->footStepGenerator_.emergencyStepNum = std::max(i_param.emergency_step_num, 1);
  this->footStepGenerator_.emergencyStepCpCheckMargin = std::max(i_param.emergency_step_cp_check_margin, 0.0);
  this->footStepGenerator_.touchVel = std::max(i_param.swing_trajectory_touch_vel, 0.001);
  if(!this->mode_.isABCRunning() || this->gaitParam_.isStatic()) this->footStepGenerator_.goalOffset = std::min(i_param.goal_offset, 0.0);

  this->legCoordsGenerator_.delayTimeOffset = std::max(i_param.swing_trajectory_delay_time_offset, 0.0);
  this->legCoordsGenerator_.finalDistanceWeight = std::max(i_param.swing_trajectory_final_distance_weight, 0.01);
  this->legCoordsGenerator_.previewStepNum = std::max(i_param.preview_step_num, 2);
  this->legCoordsGenerator_.footGuidedBalanceTime = std::max(i_param.footguided_balance_time, 0.01);

  if(i_param.eefm_body_attitude_control_gain.length() == 2 &&
     i_param.eefm_body_attitude_control_time_const.length() == 2 &&
     i_param.eefm_body_attitude_control_compensation_limit.length() == 2){
    for(int i=0;i<2;i++) {
      this->stabilizer_.bodyAttitudeControlGain[i] = std::max(i_param.eefm_body_attitude_control_gain[i], 0.0);
      this->stabilizer_.bodyAttitudeControlTimeConst[i] = std::max(i_param.eefm_body_attitude_control_time_const[i], 0.01);
      if(!this->mode_.isSTRunning()) this->stabilizer_.bodyAttitudeControlCompensationLimit[i] = std::max(i_param.eefm_body_attitude_control_compensation_limit[i], 0.0);
    }
  }

  this->stabilizer_.swing2LandingTransitionTime = std::max(i_param.swing2landing_transition_time, 0.01);
  this->stabilizer_.landing2SupportTransitionTime = std::max(i_param.landing2support_transition_time, 0.01);
  this->stabilizer_.support2SwingTransitionTime = std::max(i_param.support2swing_transition_time, 0.01);
  if(i_param.support_pgain.length() == NUM_LEGS &&
     i_param.support_dgain.length() == NUM_LEGS &&
     i_param.landing_pgain.length() == NUM_LEGS &&
     i_param.landing_dgain.length() == NUM_LEGS &&
     i_param.swing_pgain.length() == NUM_LEGS &&
     i_param.swing_dgain.length() == NUM_LEGS){
    for(int i=0;i<NUM_LEGS;i++){
      if(i_param.support_pgain[i].length() == this->stabilizer_.supportPgain[i].size() &&
         i_param.support_dgain[i].length() == this->stabilizer_.supportPgain[i].size() &&
         i_param.landing_pgain[i].length() == this->stabilizer_.supportPgain[i].size() &&
         i_param.landing_dgain[i].length() == this->stabilizer_.supportPgain[i].size() &&
         i_param.swing_pgain[i].length() == this->stabilizer_.supportPgain[i].size() &&
         i_param.swing_dgain[i].length() == this->stabilizer_.supportPgain[i].size()){
        for(int j=0;j<this->stabilizer_.supportPgain[i].size();j++){
          this->stabilizer_.supportPgain[i][j] = std::min(std::max(i_param.support_pgain[i][j], 0.0), 100.0);
          this->stabilizer_.supportDgain[i][j] = std::min(std::max(i_param.support_dgain[i][j], 0.0), 100.0);
          this->stabilizer_.landingPgain[i][j] = std::min(std::max(i_param.landing_pgain[i][j], 0.0), 100.0);
          this->stabilizer_.landingDgain[i][j] = std::min(std::max(i_param.landing_dgain[i][j], 0.0), 100.0);
          this->stabilizer_.swingPgain[i][j] = std::min(std::max(i_param.swing_pgain[i][j], 0.0), 100.0);
          this->stabilizer_.swingDgain[i][j] = std::min(std::max(i_param.swing_dgain[i][j], 0.0), 100.0);
        }
      }
    }
  }

  if(i_param.dq_weight.length() == this->fullbodyIKSolver_.dqWeight.size()){
    for(int i=0;i<this->fullbodyIKSolver_.dqWeight.size();i++){
      double value = std::max(0.01, i_param.dq_weight[i]);
      if(value != this->fullbodyIKSolver_.dqWeight[i].getGoal()) this->fullbodyIKSolver_.dqWeight[i].setGoal(value, 2.0); // 2秒で遷移
    }
  }

  return true;
}
bool AutoStabilizer::getAutoStabilizerParam(OpenHRP::AutoStabilizerService::AutoStabilizerParam& i_param) {
  std::lock_guard<std::mutex> guard(this->mutex_);

  i_param.ee_name.length(this->gaitParam_.eeName.size());
  for(int i=0;i<this->gaitParam_.eeName.size();i++) i_param.ee_name[i] = this->gaitParam_.eeName[i].c_str();
  std::vector<std::string> controllable_joints;
  for(int i=0;i<this->gaitParam_.jointControllable.size();i++) if(this->gaitParam_.jointControllable[i]) controllable_joints.push_back(this->gaitParam_.genRobot->joint(i)->name());
  i_param.controllable_joints.length(controllable_joints.size());
  for(int i=0;i<controllable_joints.size();i++) i_param.controllable_joints[i] = controllable_joints[i].c_str();
  i_param.abc_start_transition_time = this->mode_.abc_start_transition_time;
  i_param.abc_stop_transition_time = this->mode_.abc_stop_transition_time;
  i_param.st_start_transition_time = this->mode_.st_start_transition_time;
  i_param.st_stop_transition_time = this->mode_.st_stop_transition_time;

  i_param.default_zmp_offsets.length(NUM_LEGS);
  for(int i=0;i<NUM_LEGS; i++) {
    i_param.default_zmp_offsets[i].length(2);
    for(int j=0;j<2;j++) i_param.default_zmp_offsets[i][j] = this->gaitParam_.copOffset[i].value()[j];
  }
  i_param.leg_hull.length(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++){
    i_param.leg_hull[i].length(this->gaitParam_.legHull[i].size());
    for(int j=0;j<this->gaitParam_.legHull[i].size(); j++) {
      i_param.leg_hull[i][j].length(2);
      for(int k=0;k<2;k++) i_param.leg_hull[i][j][k] = this->gaitParam_.legHull[i][j][k];
    }
  }
  i_param.leg_default_translate_pos.length(NUM_LEGS);
  for(int i=0;i<NUM_LEGS; i++) {
    i_param.leg_default_translate_pos[i] = this->gaitParam_.defaultTranslatePos[i].value()[1];
  }
  i_param.is_manual_control_mode.length(NUM_LEGS);
  for(int i=0;i<NUM_LEGS; i++) {
    i_param.is_manual_control_mode[i] = (this->gaitParam_.isManualControlMode[i].getGoal() == 1.0);
  }

  i_param.is_hand_fix_mode = (this->refToGenFrameConverter_.handFixMode.getGoal() == 1.0);
  i_param.reference_frame.length(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++) {
    i_param.reference_frame[i] = (this->refToGenFrameConverter_.refFootOriginWeight[i].getGoal() == 1.0);
  }

  i_param.rpy_offset.length(3);
  for(int i=0;i<3;i++) {
    i_param.rpy_offset[i] = this->actToGenFrameConverter_.rpyOffset[i];
  }

  i_param.use_disturbance_compensation = this->externalForceHandler_.useDisturbanceCompensation;
  i_param.disturbance_compensation_time_const = this->externalForceHandler_.disturbanceCompensationTimeConst;
  i_param.disturbance_compensation_step_num = this->externalForceHandler_.disturbanceCompensationStepNum;
  i_param.disturbance_compensation_limit = this->externalForceHandler_.disturbanceCompensationLimit;

  i_param.impedance_M_p.length(this->gaitParam_.eeName.size());
  i_param.impedance_D_p.length(this->gaitParam_.eeName.size());
  i_param.impedance_K_p.length(this->gaitParam_.eeName.size());
  i_param.impedance_M_r.length(this->gaitParam_.eeName.size());
  i_param.impedance_D_r.length(this->gaitParam_.eeName.size());
  i_param.impedance_K_r.length(this->gaitParam_.eeName.size());
  i_param.impedance_force_gain.length(this->gaitParam_.eeName.size());
  i_param.impedance_moment_gain.length(this->gaitParam_.eeName.size());
  i_param.impedance_pos_compensation_limit.length(this->gaitParam_.eeName.size());
  i_param.impedance_rot_compensation_limit.length(this->gaitParam_.eeName.size());
  for(int i=0;i<this->gaitParam_.eeName.size();i++){
    i_param.impedance_M_p[i].length(3);
    i_param.impedance_D_p[i].length(3);
    i_param.impedance_K_p[i].length(3);
    i_param.impedance_M_r[i].length(3);
    i_param.impedance_D_r[i].length(3);
    i_param.impedance_K_r[i].length(3);
    i_param.impedance_force_gain[i].length(3);
    i_param.impedance_moment_gain[i].length(3);
    i_param.impedance_pos_compensation_limit[i].length(3);
    i_param.impedance_rot_compensation_limit[i].length(3);
    for(int j=0;j<3;j++){
      i_param.impedance_M_p[i][j] = this->impedanceController_.M[i][j];
      i_param.impedance_D_p[i][j] = this->impedanceController_.D[i][j];
      i_param.impedance_K_p[i][j] = this->impedanceController_.K[i][j];
      i_param.impedance_M_r[i][j] = this->impedanceController_.M[i][3+j];
      i_param.impedance_D_r[i][j] = this->impedanceController_.D[i][3+j];
      i_param.impedance_K_r[i][j] = this->impedanceController_.K[i][3+j];
      i_param.impedance_force_gain[i][j] = this->impedanceController_.wrenchGain[i][j];
      i_param.impedance_moment_gain[i][j] = this->impedanceController_.wrenchGain[i][3+j];
      i_param.impedance_pos_compensation_limit[i][j] = this->impedanceController_.compensationLimit[i][j];
      i_param.impedance_rot_compensation_limit[i][j] = this->impedanceController_.compensationLimit[i][3+j];
    }
  }

  i_param.graspless_manip_mode = this->cmdVelGenerator_.isGraspLessManipMode;
  i_param.graspless_manip_arm.length(this->cmdVelGenerator_.graspLessManipArm.size());
  for(int i=0;i<this->cmdVelGenerator_.graspLessManipArm.size();i++){
    i_param.graspless_manip_arm[i] = this->gaitParam_.eeName[this->cmdVelGenerator_.graspLessManipArm[i]].c_str();
  }
  i_param.graspless_manip_time_const.length(3);
  for(int i=0;i<3;i++){
    i_param.graspless_manip_time_const[i] = this->cmdVelGenerator_.graspLessManipTimeConst[i];
  }

  i_param.leg_collision_margin = this->footStepGenerator_.legCollisionMargin;
  i_param.default_step_time = this->footStepGenerator_.defaultStepTime;
  i_param.default_stride_limitation_max_theta.length(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++){
    i_param.default_stride_limitation_max_theta[i] = this->footStepGenerator_.defaultStrideLimitationMaxTheta[i];
  }
  i_param.default_stride_limitation_min_theta.length(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++){
    i_param.default_stride_limitation_min_theta[i] = this->footStepGenerator_.defaultStrideLimitationMinTheta[i];
  }
  i_param.default_stride_limitation.length(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++){
    i_param.default_stride_limitation[i].length(this->footStepGenerator_.defaultStrideLimitationHull[i].size());
    for(int j=0;j<this->footStepGenerator_.defaultStrideLimitationHull[i].size(); j++) {
      i_param.default_stride_limitation[i][j].length(2);
      for(int k=0;k<2;k++) i_param.default_stride_limitation[i][j][k] = this->footStepGenerator_.defaultStrideLimitationHull[i][j][k];
    }
  }
  i_param.default_double_support_ratio = this->footStepGenerator_.defaultDoubleSupportRatio;
  i_param.default_step_height = this->footStepGenerator_.defaultStepHeight;
  i_param.go_velocity_step_num = this->footStepGenerator_.goVelocityStepNum;
  i_param.modify_footsteps = this->footStepGenerator_.isModifyFootSteps;
  i_param.overwritable_remain_time = this->footStepGenerator_.overwritableRemainTime;
  i_param.overwritable_min_time = this->footStepGenerator_.overwritableMinTime;
  i_param.overwritable_min_step_time = this->footStepGenerator_.overwritableMinStepTime;
  i_param.overwritable_max_step_time = this->footStepGenerator_.overwritableMaxStepTime;
  i_param.overwritable_max_swing_velocity = this->footStepGenerator_.overwritableMaxSwingVelocity;
  i_param.safe_leg_hull.length(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++){
    i_param.safe_leg_hull[i].length(this->footStepGenerator_.safeLegHull[i].size());
    for(int j=0;j<this->footStepGenerator_.safeLegHull[i].size(); j++) {
      i_param.safe_leg_hull[i][j].length(2);
      for(int k=0;k<2;k++) i_param.safe_leg_hull[i][j][k] = this->footStepGenerator_.safeLegHull[i][j][k];
    }
  }
  i_param.overwritable_stride_limitation_max_theta.length(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++){
    i_param.overwritable_stride_limitation_max_theta[i] = this->footStepGenerator_.overwritableStrideLimitationMaxTheta[i];
  }
  i_param.overwritable_stride_limitation_min_theta.length(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++){
    i_param.overwritable_stride_limitation_min_theta[i] = this->footStepGenerator_.overwritableStrideLimitationMinTheta[i];
  }
  i_param.overwritable_stride_limitation.length(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++){
    i_param.overwritable_stride_limitation[i].length(this->footStepGenerator_.overwritableStrideLimitationHull[i].size());
    for(int j=0;j<this->footStepGenerator_.overwritableStrideLimitationHull[i].size(); j++) {
      i_param.overwritable_stride_limitation[i][j].length(2);
      for(int k=0;k<2;k++) i_param.overwritable_stride_limitation[i][j][k] = this->footStepGenerator_.overwritableStrideLimitationHull[i][j][k];
    }
  }
  i_param.overwritable_max_landing_height = this->footStepGenerator_.overwritableMaxLandingHeight;
  i_param.overwritable_min_landing_height = this->footStepGenerator_.overwritableMinLandingHeight;
  i_param.overwritable_max_gen_ground_z_velocity = this->footStepGenerator_.overwritableMaxGenGroundZVelocity;
  i_param.overwritable_max_src_ground_z_velocity = this->footStepGenerator_.overwritableMaxSrcGroundZVelocity;
  i_param.contact_detection_threshold = this->footStepGenerator_.contactDetectionThreshold;
  i_param.contact_modification_threshold = this->footStepGenerator_.contactModificationThreshold;
  i_param.is_emergency_step_mode = this->footStepGenerator_.isEmergencyStepMode;
  i_param.is_stable_go_stop_mode = this->footStepGenerator_.isStableGoStopMode;
  i_param.emergency_step_num = this->footStepGenerator_.emergencyStepNum;
  i_param.emergency_step_cp_check_margin = this->footStepGenerator_.emergencyStepCpCheckMargin;
  i_param.goal_offset = this->footStepGenerator_.goalOffset;
  i_param.swing_trajectory_touch_vel = this->footStepGenerator_.touchVel;

  i_param.swing_trajectory_delay_time_offset = this->legCoordsGenerator_.delayTimeOffset;
  i_param.swing_trajectory_final_distance_weight = this->legCoordsGenerator_.finalDistanceWeight;
  i_param.preview_step_num = this->legCoordsGenerator_.previewStepNum;
  i_param.footguided_balance_time = this->legCoordsGenerator_.footGuidedBalanceTime;

  i_param.eefm_body_attitude_control_gain.length(2);
  i_param.eefm_body_attitude_control_time_const.length(2);
  i_param.eefm_body_attitude_control_compensation_limit.length(2);
  for(int i=0;i<2;i++) {
    i_param.eefm_body_attitude_control_gain[i] = this->stabilizer_.bodyAttitudeControlGain[i];
    i_param.eefm_body_attitude_control_time_const[i] = this->stabilizer_.bodyAttitudeControlTimeConst[i];
    i_param.eefm_body_attitude_control_compensation_limit[i] = this->stabilizer_.bodyAttitudeControlCompensationLimit[i];
  }
  i_param.swing2landing_transition_time = this->stabilizer_.swing2LandingTransitionTime;
  i_param.landing2support_transition_time = this->stabilizer_.landing2SupportTransitionTime;
  i_param.support2swing_transition_time = this->stabilizer_.support2SwingTransitionTime;
  i_param.support_pgain.length(NUM_LEGS);
  i_param.support_dgain.length(NUM_LEGS);
  i_param.landing_pgain.length(NUM_LEGS);
  i_param.landing_dgain.length(NUM_LEGS);
  i_param.swing_pgain.length(NUM_LEGS);
  i_param.swing_dgain.length(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++){
    i_param.support_pgain[i].length(this->stabilizer_.supportPgain[i].size());
    i_param.support_dgain[i].length(this->stabilizer_.supportPgain[i].size());
    i_param.landing_pgain[i].length(this->stabilizer_.supportPgain[i].size());
    i_param.landing_dgain[i].length(this->stabilizer_.supportPgain[i].size());
    i_param.swing_pgain[i].length(this->stabilizer_.supportPgain[i].size());
    i_param.swing_dgain[i].length(this->stabilizer_.supportPgain[i].size());
    for(int j=0;j<this->stabilizer_.supportPgain[i].size();j++){
      i_param.support_pgain[i][j] = this->stabilizer_.supportPgain[i][j];
      i_param.support_dgain[i][j] = this->stabilizer_.supportDgain[i][j];
      i_param.landing_pgain[i][j] = this->stabilizer_.landingPgain[i][j];
      i_param.landing_dgain[i][j] = this->stabilizer_.landingDgain[i][j];
      i_param.swing_pgain[i][j] = this->stabilizer_.swingPgain[i][j];
      i_param.swing_dgain[i][j] = this->stabilizer_.swingDgain[i][j];
    }
  }

  i_param.dq_weight.length(this->fullbodyIKSolver_.dqWeight.size());
  for(int i=0;i<this->fullbodyIKSolver_.dqWeight.size();i++){
    i_param.dq_weight[i] = this->fullbodyIKSolver_.dqWeight[i].getGoal();
  }

  return true;
}

bool AutoStabilizer::getFootStepState(OpenHRP::AutoStabilizerService::FootStepState& i_param) {
  std::lock_guard<std::mutex> guard(this->mutex_);

  i_param.leg_coords.length(NUM_LEGS);
  i_param.support_leg.length(NUM_LEGS);
  i_param.leg_src_coords.length(NUM_LEGS);
  i_param.leg_dst_coords.length(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++){
    i_param.leg_coords[i].leg = this->gaitParam_.eeName[i].c_str();
    AutoStabilizer::copyEigenCoords2FootStep(this->gaitParam_.genCoords[i].value(), i_param.leg_coords[i]);
    i_param.support_leg[i] = this->gaitParam_.footstepNodesList[0].isSupportPhase[i];
    i_param.leg_src_coords[i].leg = this->gaitParam_.eeName[i].c_str();
    AutoStabilizer::copyEigenCoords2FootStep(this->gaitParam_.srcCoords[i], i_param.leg_src_coords[i]);
    i_param.leg_dst_coords[i].leg = this->gaitParam_.eeName[i].c_str();
    AutoStabilizer::copyEigenCoords2FootStep(this->gaitParam_.footstepNodesList[0].dstCoords[i], i_param.leg_dst_coords[i]);
  }
  // 現在支持脚、または現在遊脚で次支持脚になる脚の、dstCoordsの中間. 水平
  std::vector<double> weights(NUM_LEGS, 0.0);
  for(int i=0;i<NUM_LEGS; i++){
    if(this->gaitParam_.footstepNodesList[0].isSupportPhase[i] ||
       (this->gaitParam_.footstepNodesList.size() > 1 && this->gaitParam_.footstepNodesList[1].isSupportPhase[i]))
      weights[i] = 1.0;
  }
  if(weights[RLEG] == 0.0 && weights[LLEG] == 0.0) {
    weights[RLEG] = 1.0; weights[LLEG] = 1.0;
  }
  if(weights[RLEG] == 1.0 && weights[LLEG] == 1.0) i_param.dst_foot_midcoords.leg = "both";
  else if(weights[RLEG] == 1.0) i_param.dst_foot_midcoords.leg = "rleg";
  else if(weights[LLEG] == 1.0) i_param.dst_foot_midcoords.leg = "lleg";
  AutoStabilizer::copyEigenCoords2FootStep(mathutil::orientCoordToAxis(mathutil::calcMidCoords(this->gaitParam_.footstepNodesList[0].dstCoords, weights), cnoid::Vector3::UnitZ()), i_param.dst_foot_midcoords);
  i_param.is_manual_control_mode.length(NUM_LEGS);
  for(int i=0;i<NUM_LEGS; i++) {
    i_param.is_manual_control_mode[i] = (this->gaitParam_.isManualControlMode[i].getGoal() == 1.0);
  }
  i_param.joint_angle.length(this->gaitParam_.genRobot->numJoints());
  for(int i=0;i<this->gaitParam_.genRobot->numJoints();i++){
    i_param.joint_angle[i] = this->gaitParam_.genRobot->joint(i)->q();
  }
  return true;
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

// static function
void AutoStabilizer::copyEigenCoords2FootStep(const cnoid::Position& in_fs, OpenHRP::AutoStabilizerService::Footstep& out_fs){
  out_fs.pos.length(3);
  for(int j=0;j<3;j++) out_fs.pos[j] = in_fs.translation()[j];
  out_fs.rot.length(4);
  Eigen::Quaterniond quat(in_fs.linear());
  out_fs.rot[0] = quat.w(); out_fs.rot[1] = quat.x(); out_fs.rot[2] = quat.y(); out_fs.rot[3] = quat.z();
}

extern "C"{
    void AutoStabilizerInit(RTC::Manager* manager) {
        RTC::Properties profile(AutoStabilizer_spec);
        manager->registerFactory(profile, RTC::Create<AutoStabilizer>, RTC::Delete<AutoStabilizer>);
    }
};
