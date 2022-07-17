#include "AutoStabilizer.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/EigenUtil>

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
  m_refBasePosIn_("refBasePosIn", m_refBasePos_),
  m_refBaseRpyIn_("refBaseRpyIn", m_refBaseRpy_),
  m_qActIn_("qAct", m_qAct_),
  m_dqActIn_("dqAct", m_dqAct_),
  m_actImuIn_("actImuIn", m_actImu_),

  m_qOut_("q", m_q_),
  m_genTauOut_("genTau", m_genTau_),
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
  this->addInPort("refBasePosIn", this->ports_.m_refBasePosIn_);
  this->addInPort("refBaseRpyIn", this->ports_.m_refBaseRpyIn_);
  this->addInPort("qAct", this->ports_.m_qActIn_);
  this->addInPort("dqAct", this->ports_.m_dqActIn_);
  this->addInPort("actImuIn", this->ports_.m_actImuIn_);
  this->addOutPort("q", this->ports_.m_qOut_);
  this->addOutPort("genTau", this->ports_.m_genTauOut_);
  this->addOutPort("genBasePoseOut", this->ports_.m_genBasePoseOut_);
  this->addOutPort("genBaseTformOut", this->ports_.m_genBaseTformOut_);
  this->addOutPort("genBasePosOut", this->ports_.m_genBasePosOut_);
  this->addOutPort("genBaseRpyOut", this->ports_.m_genBaseRpyOut_);
  this->ports_.m_AutoStabilizerServicePort_.registerProvider("service0", "AutoStabilizerService", this->ports_.m_service0_);
  this->addPort(this->ports_.m_AutoStabilizerServicePort_);

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
    this->actRobot_ = robot->clone();
    this->actRobot_->calcForwardKinematics(); this->actRobot_->calcCenterOfMass();
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

      EndEffector ikp;
      ikp.name = name;
      ikp.parentLink = parentLink;
      ikp.localT.translation() = localp;
      ikp.localT.linear() = localR;
      this->endEffectors_.push_back(ikp);
    }
  }

  {
    //add more information to EndEffectors

    // 0番目が右脚. 1番目が左脚. という仮定がある.
    if(this->endEffectors_.size() < 2 || this->endEffectors_[0].name != "rleg" || this->endEffectors_[1].name != "lleg"){
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " this->endEffectors_.size() < 2 || this->endEffectors_[0].name != \"rleg\" || this->endEffectors_[1].name != \"lleg\" not holds" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }

    // 各EndEffectorsから親リンク側に遡っていき、最初に見つかったForceSensorをEndEffectorに対応付ける. 以後、ForceSensorの値を座標変換したものがEndEffectorが受けている力とみなされる. 見つからなければ受けている力は常に0とみなされる
    cnoid::DeviceList<cnoid::ForceSensor> forceSensors(this->actRobot_->devices());
    for (int i=0;i<this->endEffectors_.size();i++){
      cnoid::LinkPtr link = this->actRobot_->link(this->endEffectors_[i].parentLink);
      bool found = false;
      while (link != nullptr && found == false) {
        for (size_t j = 0; j < forceSensors.size(); j++) {
          if(forceSensors[j]->link() == link) {
            this->endEffectors_[i].forceSensor = forceSensors[j]->name();
            found = true;
            break;
          }
        }
      }
    }

  }

  {
    // add more ports (ロボットモデルやEndEffectorの情報を使って)

    // 各EndEffectorにつき、ref<name>WrenchInというInPortをつくる
    this->ports_.m_refWrenchIn_.resize(this->endEffectors_.size());
    this->ports_.m_refWrench_.resize(this->endEffectors_.size());
    for(int i=0;i<this->endEffectors_.size();i++){
      std::string name = "ref"+this->endEffectors_[i].name+"WrenchIn";
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



  // if(!this->robot_com_->rootLink()->isFixedJoint()) this->useJoints_.push_back(this->robot_com_->rootLink());
  // for(int i=0;i<this->robot_com_->numJoints();i++) this->useJoints_.push_back(this->robot_com_->joint(i));

  // this->outputOffsetInterpolators_.rootpInterpolator_= std::make_shared<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB);
  // this->outputOffsetInterpolators_.rootRInterpolator_= std::make_shared<cpp_filters::TwoPointInterpolatorSO3 >(cnoid::Matrix3::Identity(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB);
  // for(int i=0;i<this->robot_com_->numJoints();i++) this->outputOffsetInterpolators_.jointInterpolatorMap_[this->robot_com_->joint(i)] = std::make_shared<cpp_filters::TwoPointInterpolator<double> >(0.0,0.0,0.0,cpp_filters::HOFFARBIB);

  // for(size_t i=0;i<this->robot_com_->numJoints();i++){
  //   // apply margin
  //   if(this->refRobot_->joint(i)->q_upper() - this->refRobot_->joint(i)->q_lower() > 0.002){
  //     this->robot_com_->joint(i)->setJointRange(this->refRobot_->joint(i)->q_lower()+0.001,this->refRobot_->joint(i)->q_upper()-0.001);
  //   }
  //   // apply margin
  //   // 1.0だと安全.4.0は脚.10.0はlapid manipulation らしい. limitを小さくしすぎた状態で、速い指令を送ると、狭いlimitの中で高優先度タスクを頑張って満たそうとすることで、低優先度タスクを満たす余裕がなくエラーが大きくなってしまうことに注意.
  //   if(this->refRobot_->joint(i)->dq_upper() - this->refRobot_->joint(i)->dq_lower() > 0.02){
  //     this->robot_com_->joint(i)->setJointVelocityRange(std::max(this->refRobot_->joint(i)->dq_lower()+0.01, -this->jointVelocityLimit_),
  //                                                         std::min(this->refRobot_->joint(i)->dq_upper()-0.01, this->jointVelocityLimit_));
  //   }
  // }
  // this->robot_com_->rootLink()->setJointVelocityRange(std::max(this->refRobot_->rootLink()->dq_lower()+0.01, -this->jointVelocityLimit_),
  //                                                       std::min(this->refRobot_->rootLink()->dq_upper()-0.01, this->jointVelocityLimit_));

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
bool AutoStabilizer::readInPortData(AutoStabilizer::Ports& ports, cnoid::BodyPtr refRobot, cnoid::BodyPtr actRobot, std::vector<AutoStabilizer::EndEffector>& endEffectors){
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
          if(std::isfinite(ports.m_refWrench_[i].data[j])) endEffectors[i].refWrench[j] = ports.m_refWrench_[i].data[j];
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
      actRobot->rootLink()->R() = Eigen::Matrix3d(Eigen::AngleAxisd(actR) * Eigen::AngleAxisd(imuR.transpose() * actRobot->rootLink()->R())); // 単純に3x3行列の空間でRを積算していると、だんだん数値誤差によってユニタリ行列でなくなってしまう恐れがあるので念の為
    }
    actRobot_changed = true;
  }
  if(actRobot_changed){
    actRobot->calcForwardKinematics();
    actRobot->calcCenterOfMass();
  }

  return qRef_updated;
}

// static function
bool AutoStabilizer::writeOutPortData(AutoStabilizer::Ports& ports, cnoid::BodyPtr genRobot){
  ports.m_q_.tm = ports.m_qRef_.tm;
  ports.m_q_.data.length(genRobot->numJoints());
  for(int i=0;i<genRobot->numJoints();i++){
    ports.m_q_.data[i] = genRobot->joint(i)->q();
  }
  ports.m_qOut_.write();

  ports.m_genTau_.tm = ports.m_qRef_.tm;
  ports.m_genTau_.data.length(genRobot->numJoints());
  for(int i=0;i<genRobot->numJoints();i++){
    ports.m_genTau_.data[i] = genRobot->joint(i)->u();
  }
  ports.m_genTauOut_.write();

  cnoid::Vector3 baseRpy = cnoid::rpyFromRot(genRobot->rootLink()->R());
  ports.m_genBasePose_.tm = ports.m_qRef_.tm;
  ports.m_genBasePose_.data.position.x = genRobot->rootLink()->p()[0];
  ports.m_genBasePose_.data.position.y = genRobot->rootLink()->p()[1];
  ports.m_genBasePose_.data.position.z = genRobot->rootLink()->p()[2];
  ports.m_genBasePose_.data.orientation.r = baseRpy[0];
  ports.m_genBasePose_.data.orientation.p = baseRpy[1];
  ports.m_genBasePose_.data.orientation.y = baseRpy[2];
  ports.m_genBasePoseOut_.write();

  ports.m_genBaseTform_.tm = ports.m_qRef_.tm;
  for(int i=0;i<3;i++){
    ports.m_genBaseTform_.data[i] = genRobot->rootLink()->p()[i];
  }
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      ports.m_genBaseTform_.data[3+i*3+j] = genRobot->rootLink()->R()(i,j);// row major
    }
  }
  ports.m_genBaseTformOut_.write();

  ports.m_genBasePos_.tm = ports.m_qRef_.tm;
  ports.m_genBasePos_.data.x = genRobot->rootLink()->p()[0];
  ports.m_genBasePos_.data.y = genRobot->rootLink()->p()[1];
  ports.m_genBasePos_.data.z = genRobot->rootLink()->p()[2];
  ports.m_genBasePosOut_.write();
  ports.m_genBaseRpy_.tm = ports.m_qRef_.tm;
  ports.m_genBaseRpy_.data.r = baseRpy[0];
  ports.m_genBaseRpy_.data.p = baseRpy[1];
  ports.m_genBaseRpy_.data.y = baseRpy[2];
  ports.m_genBaseRpyOut_.write();

  return true;
}

RTC::ReturnCode_t AutoStabilizer::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);

  std::string instance_name = std::string(this->m_profile.instance_name);
  double dt = 1.0 / this->get_context(ec_id)->get_rate();
  this->loop_++;

  if(AutoStabilizer::readInPortData(this->ports_, this->refRobot_, this->actRobot_, this->endEffectors_)){ // if qRef is updated
    this->mode_.update(dt);

    this->genRobot_->rootLink()->T() = this->refRobot_->rootLink()->T();
    for(int i=0;i<this->genRobot_->numJoints();i++){
      this->genRobot_->joint(i)->q() = this->refRobot_->joint(i)->q();
      this->genRobot_->joint(i)->u() = 0;
    }

    AutoStabilizer::writeOutPortData(this->ports_, this->genRobot_);
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t AutoStabilizer::onActivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  // 各種処理を初期化する TODO
  this->mode_.reset();
  return RTC::RTC_OK;
}
RTC::ReturnCode_t AutoStabilizer::onDeactivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t AutoStabilizer::onFinalize(){ return RTC::RTC_OK; }

bool AutoStabilizer::goPos(const double& x, const double& y, const double& th){
  return true;
}
bool AutoStabilizer::goVelocity(const double& vx, const double& vy, const double& vth){
  return true;
}
bool AutoStabilizer::goStop(){
  return true;
}
bool AutoStabilizer::jumpTo(const double& x, const double& y, const double& z, const double& ts, const double& tf){
  return true;
}
bool AutoStabilizer::emergencyStop (){
  return true;
}
bool AutoStabilizer::setFootSteps(const OpenHRP::AutoStabilizerService::FootstepSequence& fs, CORBA::Long overwrite_fs_idx){
  return true;
}
bool AutoStabilizer::setFootSteps(const OpenHRP::AutoStabilizerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx){
  return true;
}
bool AutoStabilizer::setFootStepsWithParam(const OpenHRP::AutoStabilizerService::FootstepSequence& fs, const OpenHRP::AutoStabilizerService::StepParamSequence& sps, CORBA::Long overwrite_fs_idx){
  return true;
}
bool AutoStabilizer::setFootStepsWithParam(const OpenHRP::AutoStabilizerService::FootstepsSequence& fss, const OpenHRP::AutoStabilizerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx){
  return true;
}
void AutoStabilizer::waitFootSteps(){
  return;
}

bool AutoStabilizer::releaseEmergencyStop(){
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
  return true;
}
bool AutoStabilizer::getGaitGeneratorParam(OpenHRP::AutoStabilizerService::GaitGeneratorParam& i_param){
  return true;
}
bool AutoStabilizer::setAutoBalancerParam(const OpenHRP::AutoStabilizerService::AutoBalancerParam& i_param){
  this->mode_.abc_transition_time = i_param.transition_time;
  return true;
}
bool AutoStabilizer::getAutoBalancerParam(OpenHRP::AutoStabilizerService::AutoBalancerParam& i_param){
  i_param.transition_time = this->mode_.abc_transition_time;
  return true;
}
void AutoStabilizer::setStabilizerParam(const OpenHRP::AutoStabilizerService::StabilizerParam& i_param){
  this->mode_.st_transition_time = i_param.transition_time;
  return;
}
void AutoStabilizer::getStabilizerParam(OpenHRP::AutoStabilizerService::StabilizerParam& i_param){
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
