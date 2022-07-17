#include "AutoStabilizer.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/AccelerationSensor>
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
  this->addOutPort("q", this->ports_.m_qOut_);
  this->addOutPort("genBasePosOut", this->ports_.m_genBasePosOut_);
  this->addOutPort("genBaseRpyOut", this->ports_.m_genBaseRpyOut_);
  this->addOutPort("genBasePoseOut", this->ports_.m_genBasePoseOut_);
  this->addOutPort("genBaseTformOut", this->ports_.m_genBaseTformOut_);
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
    this->robot_ref_ = robot;
    this->robot_act_ = robot->clone();
    this->robot_gen_ = robot->clone();
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
      if(!std::getline(ss_endEffectors, buf, ',')) break;
      parentLink = buf;
      if(!std::getline(ss_endEffectors, buf, ',')) break; // not used
      if(!std::getline(ss_endEffectors, buf, ',')) break;
      localp[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break;
      localp[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break;
      localp[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break;
      localaxis[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break;
      localaxis[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break;
      localaxis[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break;
      localangle = std::stod(buf);

      // check validity
      name.erase (std::remove(name.begin(), name.end(), ' '), name.end()); // remove whitespace
      parentLink.erase (std::remove(name.begin(), name.end(), ' '), name.end()); // remove whitespace
      if(!this->robot_ref_->link(parentLink)){
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
    cnoid::DeviceList<cnoid::ForceSensor> forceSensors(this->robot_act_->devices());
    for (int i=0;i<this->endEffectors_.size();i++){
      cnoid::LinkPtr link = this->robot_act_->link(this->endEffectors_[i].parentLink);
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
    cnoid::DeviceList<cnoid::ForceSensor> forceSensors(this->robot_act_->devices());
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
  //   if(this->robot_ref_->joint(i)->q_upper() - this->robot_ref_->joint(i)->q_lower() > 0.002){
  //     this->robot_com_->joint(i)->setJointRange(this->robot_ref_->joint(i)->q_lower()+0.001,this->robot_ref_->joint(i)->q_upper()-0.001);
  //   }
  //   // apply margin
  //   // 1.0だと安全.4.0は脚.10.0はlapid manipulation らしい. limitを小さくしすぎた状態で、速い指令を送ると、狭いlimitの中で高優先度タスクを頑張って満たそうとすることで、低優先度タスクを満たす余裕がなくエラーが大きくなってしまうことに注意.
  //   if(this->robot_ref_->joint(i)->dq_upper() - this->robot_ref_->joint(i)->dq_lower() > 0.02){
  //     this->robot_com_->joint(i)->setJointVelocityRange(std::max(this->robot_ref_->joint(i)->dq_lower()+0.01, -this->jointVelocityLimit_),
  //                                                         std::min(this->robot_ref_->joint(i)->dq_upper()-0.01, this->jointVelocityLimit_));
  //   }
  // }
  // this->robot_com_->rootLink()->setJointVelocityRange(std::max(this->robot_ref_->rootLink()->dq_lower()+0.01, -this->jointVelocityLimit_),
  //                                                       std::min(this->robot_ref_->rootLink()->dq_upper()-0.01, this->jointVelocityLimit_));

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

RTC::ReturnCode_t AutoStabilizer::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);

  std::string instance_name = std::string(this->m_profile.instance_name);
  double dt = 1.0 / this->get_context(ec_id)->get_rate();
  this->loop_++;

  if (this->ports_.m_qRefIn_.isNew()){
    this->ports_.m_qRefIn_.read();
    this->ports_.m_q_.tm = this->ports_.m_qRef_.tm;
    this->ports_.m_q_.data.length(this->ports_.m_qRef_.data.length());
    for(int i=0;i<this->ports_.m_q_.data.length();i++){
      this->ports_.m_q_.data[i] = this->ports_.m_qRef_.data[i];
    }
    this->ports_.m_qOut_.write();
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t AutoStabilizer::onActivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  // 各種処理を初期化する TODO
  return RTC::RTC_OK;
}
RTC::ReturnCode_t AutoStabilizer::onDeactivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  // 各種処理を初期化する TODO
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
bool AutoStabilizer::startAutoBalancer(const ::OpenHRP::AutoStabilizerService::StrSequence& limbs){
  return true;
}
bool AutoStabilizer::stopAutoBalancer(){
  return true;
}
bool AutoStabilizer::setGaitGeneratorParam(const OpenHRP::AutoStabilizerService::GaitGeneratorParam& i_param){
  return true;
}
bool AutoStabilizer::getGaitGeneratorParam(OpenHRP::AutoStabilizerService::GaitGeneratorParam& i_param){
  return true;
}
bool AutoStabilizer::setAutoBalancerParam(const OpenHRP::AutoStabilizerService::AutoBalancerParam& i_param){
  return true;
}
bool AutoStabilizer::getAutoBalancerParam(OpenHRP::AutoStabilizerService::AutoBalancerParam& i_param){
  return true;
}

bool AutoStabilizer::releaseEmergencyStop(){
  return true;
}

void AutoStabilizer::getStabilizerParam(OpenHRP::AutoStabilizerService::StabilizerParam& i_param){
  return;
}
void AutoStabilizer::setStabilizerParam(const OpenHRP::AutoStabilizerService::StabilizerParam& i_param){
  return;
}
void AutoStabilizer::startStabilizer(void){
  return;
}
void AutoStabilizer::stopStabilizer(void){
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
