#ifndef AutoStabilizer_H
#define AutoStabilizer_H

#include <memory>
#include <map>
#include <time.h>
#include <mutex>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/CorbaNaming.h>

#include <cnoid/Body>

#include <cpp_filters/TwoPointInterpolator.h>
#include <cpp_filters/IIRFilter.h>
#include <joint_limit_table/JointLimitTable.h>

#include <collision_checker_msgs/idl/Collision.hh>
#include <primitive_motion_level_msgs/idl/PrimitiveState.hh>
#include <primitive_motion_level_tools/PrimitiveState.h>
#include "AutoStabilizerService_impl.h"
#include "PositionController.h"
#include "Collision.h"

class AutoStabilizer : public RTC::DataFlowComponentBase{
public:
  class Ports {
  public:
    Ports() :
      m_qRefIn_("qRefIn", m_qRef_),// from sh
      m_basePosRefIn_("basePosRefIn", m_basePosRef_),// from sh
      m_baseRpyRefIn_("baseRpyRefIn", m_baseRpyRef_),// from sh
      m_primitiveCommandRefIn_("primitiveCommandRefIn", m_primitiveCommandRef_),
      m_selfCollisionComIn_("selfCollisionComIn", m_selfCollisionCom_),
      m_envCollisionComIn_("envCollisionComIn", m_envCollisionCom_),

      m_qComOut_("qComOut", m_qCom_),
      m_basePosComOut_("basePosComOut", m_basePosCom_),
      m_baseRpyComOut_("baseRpyComOut", m_baseRpyCom_),
      m_basePoseComOut_("basePoseComOut", m_basePoseCom_),
      m_baseTformComOut_("baseTformComOut", m_baseTformCom_),
      m_primitiveCommandComOut_("primitiveCommandComOut", m_primitiveCommandCom_),

      m_AutoStabilizerServicePort_("AutoStabilizerService") {
    }

    RTC::TimedDoubleSeq m_qRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn_;
    RTC::TimedPoint3D m_basePosRef_;
    RTC::InPort<RTC::TimedPoint3D> m_basePosRefIn_;
    RTC::TimedOrientation3D m_baseRpyRef_;
    RTC::InPort<RTC::TimedOrientation3D> m_baseRpyRefIn_;
    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveCommandRef_;
    RTC::InPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveCommandRefIn_;
    collision_checker_msgs::TimedCollisionSeq m_selfCollisionCom_;
    RTC::InPort <collision_checker_msgs::TimedCollisionSeq> m_selfCollisionComIn_;
    collision_checker_msgs::TimedCollisionSeq m_envCollisionCom_;
    RTC::InPort <collision_checker_msgs::TimedCollisionSeq> m_envCollisionComIn_;

    RTC::TimedDoubleSeq m_qCom_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_qComOut_;
    RTC::TimedPoint3D m_basePosCom_;
    RTC::OutPort<RTC::TimedPoint3D> m_basePosComOut_;
    RTC::TimedOrientation3D m_baseRpyCom_;
    RTC::OutPort<RTC::TimedOrientation3D> m_baseRpyComOut_;
    RTC::TimedPose3D m_basePoseCom_;
    RTC::OutPort<RTC::TimedPose3D> m_basePoseComOut_;
    RTC::TimedDoubleSeq m_baseTformCom_; // for HrpsysSeqStateROSBridge
    RTC::OutPort<RTC::TimedDoubleSeq> m_baseTformComOut_; // for HrpsysSeqStateROSBridge
    primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveCommandCom_;
    RTC::OutPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveCommandComOut_;

    AutoStabilizerService_impl m_service0_;
    RTC::CorbaPort m_AutoStabilizerServicePort_;
  };

  class ControlMode{
  public:
    enum mode_enum{ MODE_IDLE, MODE_SYNC_TO_CONTROL, MODE_CONTROL, MODE_SYNC_TO_IDLE};
  private:
    mode_enum current, previous, next;
  public:
    ControlMode(){ current = previous = next = MODE_IDLE;}
    ~ControlMode(){}
    bool setNextMode(const mode_enum _request){
      switch(_request){
      case MODE_SYNC_TO_CONTROL:
        if(current == MODE_IDLE){ next = MODE_SYNC_TO_CONTROL; return true; }else{ return false; }
      case MODE_CONTROL:
        if(current == MODE_SYNC_TO_CONTROL){ next = MODE_CONTROL; return true; }else{ return false; }
      case MODE_SYNC_TO_IDLE:
        if(current == MODE_CONTROL){ next = MODE_SYNC_TO_IDLE; return true; }else{ return false; }
      case MODE_IDLE:
        if(current == MODE_SYNC_TO_IDLE ){ next = MODE_IDLE; return true; }else{ return false; }
      default:
        return false;
      }
    }
    void update(){ previous = current; current = next; }
    mode_enum now(){ return current; }
    mode_enum pre(){ return previous; }
    bool isRunning(){ return (current==MODE_SYNC_TO_CONTROL) || (current==MODE_CONTROL) || (current==MODE_SYNC_TO_IDLE) ;}
    bool isInitialize(){ return (previous==MODE_IDLE) && (current==MODE_SYNC_TO_CONTROL) ;}
  };

  class OutputOffsetInterpolators {
  public:
    std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> > rootpInterpolator_;
    std::shared_ptr<cpp_filters::TwoPointInterpolatorSO3> rootRInterpolator_;
    std::unordered_map<cnoid::LinkPtr, std::shared_ptr<cpp_filters::TwoPointInterpolator<double> > > jointInterpolatorMap_; // controlで上書きしない関節について、refereceに加えるoffset
  };

public:
  AutoStabilizer(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
  bool startControl();
  bool stopControl();
  bool setParams(const whole_body_position_controller::AutoStabilizerService::AutoStabilizerParam& i_param);
  bool getParams(whole_body_position_controller::AutoStabilizerService::AutoStabilizerParam& i_param);

protected:
  std::mutex mutex_;

  unsigned int debugLevel_;
  unsigned int loop_;

  Ports ports_;
  ControlMode mode_;

  std::vector<cnoid::LinkPtr> useJoints_; // controlで上書きする関節(root含む)のリスト
  OutputOffsetInterpolators outputOffsetInterpolators_;

  cnoid::BodyPtr m_robot_ref_; // reference (q, basepos and baserpy only)
  cnoid::BodyPtr m_robot_com_; // command<

  // params
  double jointVelocityLimit_ = 4.0;


  // 0. robotの設定
  std::unordered_map<cnoid::LinkPtr, std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > > jointLimitTablesMap_;

  // 1. portから受け取ったprimitive motion level 指令など
  primitive_motion_level_tools::PrimitiveStates primitiveStates_;
  std::vector<std::shared_ptr<WholeBodyPosition::Collision> > selfCollisions_;
  std::vector<std::shared_ptr<WholeBodyPosition::Collision> > envCollisions_;

  // 2. primitiveCommandMap_を受け取り、m_robot_comを計算する
  WholeBodyPosition::PositionController positionController_;

  // static functions
  static void calcReferenceRobot(const std::string& instance_name, AutoStabilizer::Ports& port, cnoid::BodyPtr& robot);
  static void getPrimitiveCommand(const std::string& instance_name, AutoStabilizer::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStates& primitiveStates);
  static void getCollision(const std::string& instance_name, AutoStabilizer::Ports& port, std::vector<std::shared_ptr<WholeBodyPosition::Collision> >& selfCollisions, std::vector<std::shared_ptr<WholeBodyPosition::Collision> >& envCollisions);
  static void processModeTransition(const std::string& instance_name, AutoStabilizer::ControlMode& mode, const double dt, const cnoid::BodyPtr& robot_ref, const cnoid::BodyPtr& robot_com, AutoStabilizer::OutputOffsetInterpolators& outputOffsetInterpolators);
  static void preProcessForControl(const std::string& instance_name, WholeBodyPosition::PositionController& positionController);
  static void passThrough(const std::string& instance_name, const cnoid::BodyPtr& robot_ref, cnoid::BodyPtr& robot_com, AutoStabilizer::OutputOffsetInterpolators& outputOffsetInterpolators, double dt, const std::vector<cnoid::LinkPtr>& useJoints = std::vector<cnoid::LinkPtr>());
  static void calcOutputPorts(const std::string& instance_name, AutoStabilizer::Ports& port, const cnoid::BodyPtr& robot_com, const primitive_motion_level_tools::PrimitiveStates& primitiveStates);
};


extern "C"
{
  void AutoStabilizerInit(RTC::Manager* manager);
};

#endif // AutoStabilizer_H
