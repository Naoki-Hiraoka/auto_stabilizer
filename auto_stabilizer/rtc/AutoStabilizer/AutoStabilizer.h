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

// #include <cpp_filters/TwoPointInterpolator.h>
// #include <cpp_filters/IIRFilter.h>
// #include <joint_limit_table/JointLimitTable.h>

#include "AutoStabilizerService_impl.h"

class AutoStabilizer : public RTC::DataFlowComponentBase{
public:
  class Ports {
  public:
    Ports() :
      m_qRefIn_("qRefIn", m_qRef_),// from sh
      m_basePosRefIn_("basePosRefIn", m_basePosRef_),// from sh
      m_baseRpyRefIn_("baseRpyRefIn", m_baseRpyRef_),// from sh

      m_qComOut_("qComOut", m_qCom_),
      m_basePosComOut_("basePosComOut", m_basePosCom_),
      m_baseRpyComOut_("baseRpyComOut", m_baseRpyCom_),
      m_basePoseComOut_("basePoseComOut", m_basePoseCom_),
      m_baseTformComOut_("baseTformComOut", m_baseTformCom_),

      m_AutoStabilizerServicePort_("AutoStabilizerService") {
    }

    RTC::TimedDoubleSeq m_qRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn_;
    RTC::TimedPoint3D m_basePosRef_;
    RTC::InPort<RTC::TimedPoint3D> m_basePosRefIn_;
    RTC::TimedOrientation3D m_baseRpyRef_;
    RTC::InPort<RTC::TimedOrientation3D> m_baseRpyRefIn_;

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

  // class OutputOffsetInterpolators {
  // public:
  //   std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> > rootpInterpolator_;
  //   std::shared_ptr<cpp_filters::TwoPointInterpolatorSO3> rootRInterpolator_;
  //   std::unordered_map<cnoid::LinkPtr, std::shared_ptr<cpp_filters::TwoPointInterpolator<double> > > jointInterpolatorMap_; // controlで上書きしない関節について、refereceに加えるoffset
  // };

public:
  AutoStabilizer(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool goPos(const double& x, const double& y, const double& th);
  bool goVelocity(const double& vx, const double& vy, const double& vth);
  bool goStop();
  bool jumpTo(const double& x, const double& y, const double& z, const double& ts, const double& tf);
  bool emergencyStop ();
  bool setFootSteps(const OpenHRP::AutoStabilizerService::FootstepSequence& fs, CORBA::Long overwrite_fs_idx);
  bool setFootSteps(const OpenHRP::AutoStabilizerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx);
  bool setFootStepsWithParam(const OpenHRP::AutoStabilizerService::FootstepSequence& fs, const OpenHRP::AutoStabilizerService::StepParamSequence& sps, CORBA::Long overwrite_fs_idx);
  bool setFootStepsWithParam(const OpenHRP::AutoStabilizerService::FootstepsSequence& fss, const OpenHRP::AutoStabilizerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx);
  void waitFootSteps();
  bool startAutoBalancer(const ::OpenHRP::AutoStabilizerService::StrSequence& limbs);
  bool stopAutoBalancer();
  bool setGaitGeneratorParam(const OpenHRP::AutoStabilizerService::GaitGeneratorParam& i_param);
  bool getGaitGeneratorParam(OpenHRP::AutoStabilizerService::GaitGeneratorParam& i_param);
  bool setAutoBalancerParam(const OpenHRP::AutoStabilizerService::AutoBalancerParam& i_param);
  bool getAutoBalancerParam(OpenHRP::AutoStabilizerService::AutoBalancerParam& i_param);
  bool releaseEmergencyStop();
  void getStabilizerParam(OpenHRP::AutoStabilizerService::StabilizerParam& i_param);
  void setStabilizerParam(const OpenHRP::AutoStabilizerService::StabilizerParam& i_param);
  void startStabilizer(void);
  void stopStabilizer(void);

protected:
  std::mutex mutex_;

  unsigned int debugLevel_;
  unsigned long loop_;

  Ports ports_;
  ControlMode mode_;

  std::vector<cnoid::LinkPtr> useJoints_; // controlで上書きする関節(root含む)のリスト
  //OutputOffsetInterpolators outputOffsetInterpolators_;

  cnoid::BodyPtr m_robot_ref_; // reference (q, basepos and baserpy only)
  cnoid::BodyPtr m_robot_com_; // command<

  // params

};


extern "C"
{
  void AutoStabilizerInit(RTC::Manager* manager);
};

#endif // AutoStabilizer_H
