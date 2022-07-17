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
  bool getProperty(const std::string& key, std::string& ret);

protected:
  std::mutex mutex_;

  unsigned int debugLevel_;
  unsigned long loop_;

  class Ports {
  public:
    Ports();

    RTC::TimedDoubleSeq m_qRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn_;
    RTC::TimedPoint3D m_refBasePos_; // Reference World frame
    RTC::InPort<RTC::TimedPoint3D> m_refBasePosIn_;
    RTC::TimedOrientation3D m_refBaseRpy_; // Reference World frame
    RTC::InPort<RTC::TimedOrientation3D> m_refBaseRpyIn_;
    std::vector<RTC::TimedDoubleSeq> m_refWrench_; // Reference FootOrigin frame. EndEffector origin. 要素数及び順番はendEffectors_と同じ
    std::vector<std::unique_ptr<RTC::InPort<RTC::TimedDoubleSeq> > > m_refWrenchIn_;
    RTC::TimedDoubleSeq m_qAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_;
    RTC::TimedDoubleSeq m_dqAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_dqActIn_;
    RTC::TimedOrientation3D m_actImu_; // Actual Imu World Frame. robotのgyrometerという名のRateGyroSensorの傾きを表す
    RTC::InPort<RTC::TimedOrientation3D> m_actImuIn_;
    std::vector<RTC::TimedDoubleSeq> m_actWrench_; // Actual ForceSensor frame. ForceSensor origin. 要素数及び順番はrobot->forceSensorsと同じ
    std::vector<std::unique_ptr<RTC::InPort<RTC::TimedDoubleSeq> > > m_actWrenchIn_;

    RTC::TimedDoubleSeq m_q_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_qOut_;
    RTC::TimedDoubleSeq m_genTau_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_genTauOut_;
    RTC::TimedPose3D m_genBasePose_; // Generate World frame
    RTC::OutPort<RTC::TimedPose3D> m_genBasePoseOut_;
    RTC::TimedDoubleSeq m_genBaseTform_;  // Generate World frame
    RTC::OutPort<RTC::TimedDoubleSeq> m_genBaseTformOut_; // for HrpsysSeqStateROSBridge

    AutoStabilizerService_impl m_service0_;
    RTC::CorbaPort m_AutoStabilizerServicePort_;

    // only for log
    RTC::TimedPoint3D m_genBasePos_; // Generate World frame
    RTC::OutPort<RTC::TimedPoint3D> m_genBasePosOut_; // for log
    RTC::TimedOrientation3D m_genBaseRpy_; // Generate World frame
    RTC::OutPort<RTC::TimedOrientation3D> m_genBaseRpyOut_; // for log

  };
  Ports ports_;

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
  ControlMode mode_;

  cnoid::BodyPtr refRobot_; // actual
  cnoid::BodyPtr actRobot_; // actual
  cnoid::BodyPtr genRobot_; // output

  class EndEffector {
  public:
    // constant
    std::string name;
    std::string parentLink;
    cnoid::Position localT; // Parent Link Frame
    std::string forceSensor;

    // from reference port
    cnoid::Vector6 refWrench; // FootOrigin frame. EndEffector origin.
  };
  std::vector<EndEffector> endEffectors_;

  // params
  std::vector<cnoid::LinkPtr> useJoints_; // controlで上書きする関節(root含む)のリスト

  //OutputOffsetInterpolators outputOffsetInterpolators_;

protected:
  // utility functions
  static bool readInPortData(Ports& ports, cnoid::BodyPtr refRobot, cnoid::BodyPtr actRobot, std::vector<EndEffector>& endEffectors);
  static bool writeOutPortData(Ports& ports, cnoid::BodyPtr genRobot);
};


extern "C"
{
  void AutoStabilizerInit(RTC::Manager* manager);
};

#endif // AutoStabilizer_H
