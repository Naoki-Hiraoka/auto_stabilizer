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

  addInPort("qRefIn", this->ports_.m_qRefIn_);// from sh
  addInPort("basePosRefIn", this->ports_.m_basePosRefIn_);
  addInPort("baseRpyRefIn", this->ports_.m_baseRpyRefIn_);
  addOutPort("qComOut", this->ports_.m_qComOut_);
  addOutPort("basePosComOut", this->ports_.m_basePosComOut_);
  addOutPort("baseRpyComOut", this->ports_.m_baseRpyComOut_);
  addOutPort("basePoseComOut", this->ports_.m_basePoseComOut_);
  addOutPort("baseTformComOut", this->ports_.m_baseTformComOut_);
  this->ports_.m_AutoStabilizerServicePort_.registerProvider("service0", "AutoStabilizerService", this->ports_.m_service0_);
  addPort(this->ports_.m_AutoStabilizerServicePort_);

  this->loop_ = 0;

  cnoid::BodyLoader bodyLoader;
  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
  cnoid::BodyPtr robot = bodyLoader.load(fileName);
  if(!robot){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }
  this->m_robot_ref_ = robot;
  this->m_robot_com_ = robot->clone();

  if(!this->m_robot_com_->rootLink()->isFixedJoint()) this->useJoints_.push_back(this->m_robot_com_->rootLink());
  for(int i=0;i<this->m_robot_com_->numJoints();i++) this->useJoints_.push_back(this->m_robot_com_->joint(i));

  // this->outputOffsetInterpolators_.rootpInterpolator_= std::make_shared<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB);
  // this->outputOffsetInterpolators_.rootRInterpolator_= std::make_shared<cpp_filters::TwoPointInterpolatorSO3 >(cnoid::Matrix3::Identity(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB);
  // for(int i=0;i<this->m_robot_com_->numJoints();i++) this->outputOffsetInterpolators_.jointInterpolatorMap_[this->m_robot_com_->joint(i)] = std::make_shared<cpp_filters::TwoPointInterpolator<double> >(0.0,0.0,0.0,cpp_filters::HOFFARBIB);

  // for(size_t i=0;i<this->m_robot_com_->numJoints();i++){
  //   // apply margin
  //   if(this->m_robot_ref_->joint(i)->q_upper() - this->m_robot_ref_->joint(i)->q_lower() > 0.002){
  //     this->m_robot_com_->joint(i)->setJointRange(this->m_robot_ref_->joint(i)->q_lower()+0.001,this->m_robot_ref_->joint(i)->q_upper()-0.001);
  //   }
  //   // apply margin
  //   // 1.0だと安全.4.0は脚.10.0はlapid manipulation らしい. limitを小さくしすぎた状態で、速い指令を送ると、狭いlimitの中で高優先度タスクを頑張って満たそうとすることで、低優先度タスクを満たす余裕がなくエラーが大きくなってしまうことに注意.
  //   if(this->m_robot_ref_->joint(i)->dq_upper() - this->m_robot_ref_->joint(i)->dq_lower() > 0.02){
  //     this->m_robot_com_->joint(i)->setJointVelocityRange(std::max(this->m_robot_ref_->joint(i)->dq_lower()+0.01, -this->jointVelocityLimit_),
  //                                                         std::min(this->m_robot_ref_->joint(i)->dq_upper()-0.01, this->jointVelocityLimit_));
  //   }
  // }
  // this->m_robot_com_->rootLink()->setJointVelocityRange(std::max(this->m_robot_ref_->rootLink()->dq_lower()+0.01, -this->jointVelocityLimit_),
  //                                                       std::min(this->m_robot_ref_->rootLink()->dq_upper()-0.01, this->jointVelocityLimit_));

  // std::string jointLimitTableStr;
  // if(this->getProperties().hasKey("joint_limit_table")) jointLimitTableStr = std::string(this->getProperties()["joint_limit_table"]);
  // else jointLimitTableStr = std::string(this->m_pManager->getConfig()["joint_limit_table"]); // 引数 -o で与えたプロパティを捕捉
  // std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > jointLimitTables = joint_limit_table::readJointLimitTablesFromProperty (this->m_robot_com_, jointLimitTableStr);
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
  return RTC::RTC_OK;
}

RTC::ReturnCode_t AutoStabilizer::onActivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
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

extern "C"{
    void AutoStabilizerInit(RTC::Manager* manager) {
        RTC::Properties profile(AutoStabilizer_spec);
        manager->registerFactory(profile, RTC::Create<AutoStabilizer>, RTC::Delete<AutoStabilizer>);
    }
};
