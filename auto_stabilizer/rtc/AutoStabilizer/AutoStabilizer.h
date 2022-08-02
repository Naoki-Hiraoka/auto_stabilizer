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
#include <fullbody_inverse_kinematics_solver/FullbodyInverseKinematicsSolverFast.h>
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint/COMConstraint.h>
#include <ik_constraint/JointAngleConstraint.h>
#include <ik_constraint/AngularMomentumConstraint.h>

// #include <cpp_filters/IIRFilter.h>
// #include <joint_limit_table/JointLimitTable.h>

#include "AutoStabilizerService_impl.h"
#include "EndEffectorParam.h"
#include "GaitParam.h"
#include "RefToGenFrameConverter.h"
#include "FootStepGenerator.h"
#include "LegCoordsGenerator.h"

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
  bool setFootSteps(const OpenHRP::AutoStabilizerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx);
  bool setFootStepsWithParam(const OpenHRP::AutoStabilizerService::FootstepsSequence& fss, const OpenHRP::AutoStabilizerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx);
  void waitFootSteps();
  bool startAutoBalancer();
  bool stopAutoBalancer();
  bool setGaitGeneratorParam(const OpenHRP::AutoStabilizerService::GaitGeneratorParam& i_param);
  bool getGaitGeneratorParam(OpenHRP::AutoStabilizerService::GaitGeneratorParam& i_param);
  bool setAutoBalancerParam(const OpenHRP::AutoStabilizerService::AutoBalancerParam& i_param);
  bool getAutoBalancerParam(OpenHRP::AutoStabilizerService::AutoBalancerParam& i_param);
  bool releaseEmergencyStop();
  void getStabilizerParam(OpenHRP::AutoStabilizerService::StabilizerParam& i_param);
  void setStabilizerParam(const OpenHRP::AutoStabilizerService::StabilizerParam& i_param);
  bool startStabilizer(void);
  bool stopStabilizer(void);

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
      MODE_SYNC_TO*の時間はstartの方はすぐに遷移し、stopの方はtransition_timeの時間をかけて遷移するが、少なくとも1周期はMODE_SYNC_TO*を経由する.
      MODE_SYNC_TO*では、基本的に次のMODEと同じ処理が行われるが、出力時に前回のMODEの出力から補間するような軌道に加工されることで出力の連続性を確保する
      補間している途中で別のmodeに切り替わることは無いので、そこは安心してプログラムを書いてよい(例外はonActivated)
     */
    enum Mode_enum{ MODE_IDLE, MODE_SYNC_TO_ABC, MODE_ABC, MODE_SYNC_TO_ST, MODE_ST, MODE_SYNC_TO_STOPST, MODE_SYNC_TO_IDLE};
    enum Transition_enum{ START_ABC, STOP_ABC, START_ST, STOP_ST};
    double abc_start_transition_time, abc_stop_transition_time, st_start_transition_time, st_stop_transition_time; // 一般に、現在の姿勢が関節角度上下限外といった状況でなければ、start_transition_timeはほぼゼロでよい
  private:
    Mode_enum current, previous, next;
    double remain_time;
  public:
    ControlMode(){ reset(); abc_start_transition_time = 0.5; abc_stop_transition_time = 2.0; st_start_transition_time = 0.5; st_stop_transition_time = 2.0;}
    void reset(){ current = previous = next = MODE_IDLE; remain_time = 0;}
    bool setNextTransition(const Transition_enum request){
      switch(request){
      case START_ABC:
        if(current == MODE_IDLE){ next = MODE_SYNC_TO_ABC; remain_time = abc_start_transition_time; return true; }else{ return false; }
      case STOP_ABC:
        if(current == MODE_ABC){ next = MODE_SYNC_TO_IDLE; remain_time = abc_stop_transition_time; return true; }else{ return false; }
      case START_ST:
        if(current == MODE_ABC){ next = MODE_SYNC_TO_ST; remain_time = st_start_transition_time; return true; }else{ return false; }
      case STOP_ST:
        if(current == MODE_ST){ next = MODE_SYNC_TO_STOPST; remain_time = st_stop_transition_time; return true; }else{ return false; }
      default:
        return false;
      }
    }
    void update(double dt){
      if(current != next) {
        previous = current; current = next;
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
    bool isABCInit() const{ return (current == MODE_SYNC_TO_ABC) && (previous == MODE_IDLE);}
    bool isABCRunning() const{ return (current==MODE_SYNC_TO_ABC) || (current==MODE_ABC) || (current==MODE_SYNC_TO_ST) || (current==MODE_ST) || (current==MODE_SYNC_TO_STOPST) ;}
    bool isSTInit() const{ return (current == MODE_SYNC_TO_ST) && (previous == MODE_ABC);}
    bool isSTRunning() const{ return (current==MODE_SYNC_TO_ST) || (current==MODE_ST) ;}
    bool isSyncInit() const{ return (current != previous) && isSync();}
    bool isSync() const{ return ((current==MODE_SYNC_TO_ABC) || (current==MODE_SYNC_TO_ST) || (current==MODE_SYNC_TO_STOPST) || (current==MODE_SYNC_TO_IDLE));}
  };
  ControlMode mode_;

  cnoid::BodyPtr refRobot_; // reference. reference world frame
  cnoid::BodyPtr refRobotOrigin_; // reference. generate frame
  cnoid::BodyPtr actRobot_; // actual. actual imu world frame
  cnoid::BodyPtr actRobotOrigin_; // actual. generate frame
  cnoid::BodyPtr genRobot_; // output

  class JointParam {
  public:
    // constant
    std::string name = ""; // 関節名
    double maxTorque = 0.0; // モデルのclimit * gearRatio * torqueConstより計算

    // params
    bool controllable = true; // falseの場合、qやtauはrefの値をそのまま出力する. その関節は存在しないかのように扱う. このパラメータはMODE_IDLEのときにしか変更されないので、そこは安心してプログラムを書いて良い
  };
  std::vector<JointParam> jointParams_; // 要素数robot->numJoints(). jointIdの順.

  EndEffectorParam endEffectorParams_;
  GaitParam gaitParam_;

  RefToGenFrameConverter refToGenFrameConverter_;
  FootStepGenerator footStepGenerator_;
  LegCoordsGenerator legCoordsGenerator_;

protected:
  // utility functions
  bool getProperty(const std::string& key, std::string& ret);

  static bool copyRobotState(cnoid::BodyPtr inRobot, cnoid::BodyPtr outRobot);
  static void moveCoords(cnoid::BodyPtr robot, const cnoid::Position& target, const cnoid::Position& at);

  static bool readInPortData(AutoStabilizer::Ports& ports, cnoid::BodyPtr refRobot, cnoid::BodyPtr actRobot, EndEffectorParam& endEffectorParams);
  static bool calcActualParameters(const AutoStabilizer::ControlMode& mode, const cnoid::BodyPtr& actRobot, cnoid::BodyPtr& actRobotOrigin, EndEffectorParam& endEffectorParams, GaitParam& gaitParam, double dt);
  static bool execAutoBalancer(const AutoStabilizer::ControlMode& mode, const cnoid::BodyPtr& refRobot, cnoid::BodyPtr& refRobotOrigin, const cnoid::BodyPtr& actRobot, cnoid::BodyPtr& actRobotOrigin, cnoid::BodyPtr& genRobot, EndEffectorParam& endEffectorParams, GaitParam& gaitParam, double dt, const std::vector<JointParam>& jointParams, const FootStepGenerator& footStepGenerator, const LegCoordsGenerator& legCoordsGenerator, const RefToGenFrameConverter& refToGenFrameConverter);
  class FullbodyIKParam {
  public:
    cnoid::VectorX jlim_avoid_weight;
    std::vector<std::shared_ptr<IK::JointAngleConstraint> > refJointAngleConstraint;
    std::shared_ptr<IK::PositionConstraint> rootPositionConstraint = std::make_shared<IK::PositionConstraint>();
    std::shared_ptr<IK::COMConstraint> comConstraint = std::make_shared<IK::COMConstraint>();
    std::shared_ptr<IK::AngularMomentumConstraint> angularMomentumConstraint = std::make_shared<IK::AngularMomentumConstraint>();
  };
  FullbodyIKParam fullbodyIKParam_;
  static bool solveFullbodyIK(cnoid::BodyPtr& genRobot, const cnoid::BodyPtr& refRobotOrigin, EndEffectorParam& endEffectorParams, AutoStabilizer::FullbodyIKParam& fullbodyIKParam, double dt, const std::vector<JointParam>& jointParams, const GaitParam& gaitParam);
  class OutputOffsetInterpolators {
  public:
    cpp_filters::TwoPointInterpolatorSE3 genBasePoseInterpolator = cpp_filters::TwoPointInterpolatorSE3(cnoid::Position::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB);
    std::vector<cpp_filters::TwoPointInterpolator<double> > qInterpolator; // 要素数はrobot->numJoints(). jointIdの順
    std::vector<cpp_filters::TwoPointInterpolator<double> > genTauInterpolator; // 要素数はrobot->numJoints(). jointIdの順
  };
  OutputOffsetInterpolators outputOffsetInterpolators_; // refereceに加えるoffset
  static bool writeOutPortData(AutoStabilizer::Ports& ports, cnoid::BodyPtr genRobot, const AutoStabilizer::ControlMode& mode, AutoStabilizer::OutputOffsetInterpolators& outputOffsetInterpolators, double dt);
};


extern "C"
{
  void AutoStabilizerInit(RTC::Manager* manager);
};

#endif // AutoStabilizer_H
