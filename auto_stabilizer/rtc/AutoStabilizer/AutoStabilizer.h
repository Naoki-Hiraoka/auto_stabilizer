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

// #include <cpp_filters/IIRFilter.h>
// #include <joint_limit_table/JointLimitTable.h>

#include "AutoStabilizerService_impl.h"
#include "hrpsys/idl/RobotHardwareService.hh"
#include "GaitParam.h"
#include "RefToGenFrameConverter.h"
#include "ActToGenFrameConverter.h"
#include "LegManualController.h"
#include "FootStepGenerator.h"
#include "LegCoordsGenerator.h"
#include "ImpedanceController.h"
#include "Stabilizer.h"
#include "ExternalForceHandler.h"
#include "FullbodyIKSolver.h"
#include "CmdVelGenerator.h"

class AutoStabilizer : public RTC::DataFlowComponentBase{
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
  bool setFootSteps(const OpenHRP::AutoStabilizerService::FootstepSequence& fs);
  bool setFootStepsWithParam(const OpenHRP::AutoStabilizerService::FootstepSequence& fs, const OpenHRP::AutoStabilizerService::StepParamSequence& sps);
  void waitFootSteps();
  bool startAutoBalancer();
  bool stopAutoBalancer();
  bool setAutoStabilizerParam(const OpenHRP::AutoStabilizerService::AutoStabilizerParam& i_param);
  bool getAutoStabilizerParam(OpenHRP::AutoStabilizerService::AutoStabilizerParam& i_param);
  bool getFootStepState(OpenHRP::AutoStabilizerService::FootStepState& i_param);
  bool releaseEmergencyStop();
  bool startStabilizer(void);
  bool stopStabilizer(void);
  bool startImpedanceController(const std::string& i_name);
  bool stopImpedanceController(const std::string& i_name);

protected:
  std::mutex mutex_;

  unsigned int debugLevel_;
  unsigned long long loop_;
  double dt_;

  class Ports {
  public:
    Ports();

    RTC::TimedDoubleSeq m_qRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn_;
    RTC::TimedDoubleSeq m_refTau_;
    RTC::InPort<RTC::TimedDoubleSeq> m_refTauIn_;
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

    RTC::CorbaConsumer<OpenHRP::RobotHardwareService> m_robotHardwareService0_;
    RTC::CorbaPort m_RobotHardwareServicePort_;

    // only for log
    RTC::TimedPoint3D m_genBasePos_; // Generate World frame
    RTC::OutPort<RTC::TimedPoint3D> m_genBasePosOut_; // for log
    RTC::TimedOrientation3D m_genBaseRpy_; // Generate World frame
    RTC::OutPort<RTC::TimedOrientation3D> m_genBaseRpyOut_; // for log

  };
  Ports ports_;

  class ControlMode{
  public:
    /*
      MODE_IDLE -> startAutoBalancer() -> MODE_SYNC_TO_ABC -> MODE_ABC -> startStabilizer() -> MODE_SYNC_TO_ST -> MODE_ST -> stopStabilizer() -> MODE_SYNC_TO_STOPST -> MODE_ABC -> stopAutoBalancer() -> MODE_SYNC_TO_IDLE -> MODE_IDLE
      MODE_SYNC_TO*の時間はtransition_timeの時間をかけて遷移するが、少なくとも1周期はMODE_SYNC_TO*を経由する.
      MODE_SYNC_TO*では、基本的に次のMODEと同じ処理が行われるが、出力時に前回のMODEの出力から補間するような軌道に加工されることで出力の連続性を確保する
      補間している途中で別のmodeに切り替わることは無いので、そこは安心してプログラムを書いてよい(例外はonActivated). 同様に、remainTimeが突然減ったり増えたりすることもない
     */
    enum Mode_enum{ MODE_IDLE, MODE_SYNC_TO_ABC, MODE_ABC, MODE_SYNC_TO_ST, MODE_ST, MODE_SYNC_TO_STOPST, MODE_SYNC_TO_IDLE};
    enum Transition_enum{ START_ABC, STOP_ABC, START_ST, STOP_ST};
    double abc_start_transition_time, abc_stop_transition_time, st_start_transition_time, st_stop_transition_time;
  private:
    Mode_enum current, previous, next;
    double remain_time;
  public:
    ControlMode(){ reset(); abc_start_transition_time = 2.0; abc_stop_transition_time = 2.0; st_start_transition_time = 0.5; st_stop_transition_time = 2.0;}
    void reset(){ current = previous = next = MODE_IDLE; remain_time = 0;}
    bool setNextTransition(const Transition_enum request){
      switch(request){
      case START_ABC:
        if(current == MODE_IDLE){ next = MODE_SYNC_TO_ABC; return true; }else{ return false; }
      case STOP_ABC:
        if(current == MODE_ABC){ next = MODE_SYNC_TO_IDLE; return true; }else{ return false; }
      case START_ST:
        if(current == MODE_ABC){ next = MODE_SYNC_TO_ST; return true; }else{ return false; }
      case STOP_ST:
        if(current == MODE_ST){ next = MODE_SYNC_TO_STOPST; return true; }else{ return false; }
      default:
        return false;
      }
    }
    void update(double dt){
      if(current != next) {
        previous = current; current = next;
        switch(current){
        case MODE_SYNC_TO_ABC:
          remain_time = abc_start_transition_time; break;
        case MODE_SYNC_TO_IDLE:
          remain_time = abc_stop_transition_time; break;
        case MODE_SYNC_TO_ST:
          remain_time = st_start_transition_time; break;
        case MODE_SYNC_TO_STOPST:
          remain_time = st_stop_transition_time; break;
        default:
          break;
        }
      }else{
        previous = current;
        remain_time -= dt;
        if(remain_time <= 0.0){
          remain_time = 0.0;
          switch(current){
          case MODE_SYNC_TO_ABC:
            current = next = MODE_ABC; break;
          case MODE_SYNC_TO_IDLE:
            current = next = MODE_IDLE; break;
          case MODE_SYNC_TO_ST:
            current = next = MODE_ST; break;
          case MODE_SYNC_TO_STOPST:
            current = next = MODE_ABC; break;
          default:
            break;
          }
        }
      }
    }
    double remainTime() const{ return remain_time;}
    Mode_enum now() const{ return current; }
    Mode_enum pre() const{ return previous; }
    bool isABCRunning() const{ return (current==MODE_SYNC_TO_ABC) || (current==MODE_ABC) || (current==MODE_SYNC_TO_ST) || (current==MODE_ST) || (current==MODE_SYNC_TO_STOPST) ;}
    bool isSyncToABC() const{ return current==MODE_SYNC_TO_ABC;}
    bool isSyncToABCInit() const{ return (current != previous) && isSyncToABC();}
    bool isSyncToIdle() const{ return current==MODE_SYNC_TO_IDLE;}
    bool isSyncToIdleInit() const{ return (current != previous) && isSyncToIdle();}
    bool isSyncToStopST() const{ return current == MODE_SYNC_TO_STOPST;}
    bool isSyncToStopSTInit() const{ return (current != previous) && isSyncToStopST();}
    bool isSTRunning() const{ return (current==MODE_SYNC_TO_ST) || (current==MODE_ST) ;}
  };
  ControlMode mode_;
  cpp_filters::TwoPointInterpolator<double> idleToAbcTransitionInterpolator_ = cpp_filters::TwoPointInterpolator<double>(0.0, 0.0, 0.0, cpp_filters::LINEAR);

  GaitParam gaitParam_;

  RefToGenFrameConverter refToGenFrameConverter_;
  ActToGenFrameConverter actToGenFrameConverter_;
  ExternalForceHandler externalForceHandler_;
  ImpedanceController impedanceController_;
  LegManualController legManualController_;
  CmdVelGenerator cmdVelGenerator_;
  FootStepGenerator footStepGenerator_;
  LegCoordsGenerator legCoordsGenerator_;
  Stabilizer stabilizer_;
  FullbodyIKSolver fullbodyIKSolver_;

protected:
  // utility functions
  bool getProperty(const std::string& key, std::string& ret);
  static void copyEigenCoords2FootStep(const cnoid::Position& in_fs, OpenHRP::AutoStabilizerService::Footstep& out_fs);

  static bool readInPortData(AutoStabilizer::Ports& ports, cnoid::BodyPtr refRobotRaw, cnoid::BodyPtr actRobotRaw, std::vector<cnoid::Vector6>& refEEWrenchOrigin);
  static bool execAutoStabilizer(const AutoStabilizer::ControlMode& mode, GaitParam& gaitParam, double dt, const FootStepGenerator& footStepGenerator, const LegCoordsGenerator& legCoordsGenerator, const RefToGenFrameConverter& refToGenFrameConverter, const ActToGenFrameConverter& actToGenFrameConverter, const ImpedanceController& impedanceController, const Stabilizer& stabilizer, const ExternalForceHandler& externalForceHandler, const FullbodyIKSolver& fullbodyIKSolver, const LegManualController& legManualController, const CmdVelGenerator& cmdVelGenerator);
  static bool writeOutPortData(AutoStabilizer::Ports& ports, const AutoStabilizer::ControlMode& mode, cpp_filters::TwoPointInterpolator<double>& idleToAbcTransitionInterpolator, double dt, const GaitParam& gaitParam);
};


extern "C"
{
  void AutoStabilizerInit(RTC::Manager* manager);
};

#endif // AutoStabilizer_H
