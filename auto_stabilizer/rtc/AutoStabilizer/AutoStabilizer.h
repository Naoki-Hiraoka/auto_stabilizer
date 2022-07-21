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
#include "GaitParam.h"

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
    double abc_transition_time, st_transition_time;
  private:
    Mode_enum current, previous, next;
    double remain_time;
  public:
    ControlMode(){ reset(); abc_transition_time = 2.0; st_transition_time = 2.0;}
    void reset(){ current = previous = next = MODE_IDLE; remain_time = 0;}
    bool setNextTransition(const Transition_enum request){
      switch(request){
      case START_ABC:
        if(current == MODE_IDLE){ next = MODE_SYNC_TO_ABC; remain_time = 0.0; return true; }else{ return false; }
      case STOP_ABC:
        if(current == MODE_ABC){ next = MODE_SYNC_TO_IDLE; remain_time = abc_transition_time; return true; }else{ return false; }
      case START_ST:
        if(current == MODE_ABC){ next = MODE_SYNC_TO_ST; remain_time = 0.0; return true; }else{ return false; }
      case STOP_ST:
        if(current == MODE_ST){ next = MODE_SYNC_TO_STOPST; remain_time = st_transition_time; return true; }else{ return false; }
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

  class EndEffectorParam {
  public:
    // constant
    std::string name = ""; // 右脚はrleg. 左脚はllegという名前である必要がある
    std::string parentLink = ""; // 必ずrobot->link(parentLink)がnullptrではないことを約束する. そのため、毎回robot->link(parentLink)がnullptrかをチェックしなくても良い
    cnoid::Position localT = cnoid::Position::Identity(); // Parent Link Frame
    std::string forceSensor = ""; // actualのForceSensorの値を座標変換したものがEndEffectorが受けている力とみなされる. forceSensorが""ならば受けている力は常に0とみなされる. forceSensorが""で無いならばrobot->findDevice<cnoid::ForceSensor>(endEffectorParams[i].forceSensor)がnullptrでは無いことを約束するので、毎回nullptrかをチェックしなくても良い

    // from reference port
    cnoid::Vector6 refWrenchOrigin = cnoid::Vector6::Zero(); // FootOrigin frame. EndEffector origin. ロボットが受ける力

    // AutoStabilizerの内部で計算される
    cnoid::Position refPose = cnoid::Position::Identity(); // generate frame
    cnoid::Vector6 refWrench = cnoid::Vector6::Zero(); // generate frame. EndEffector origin. ロボットが受ける力
    cnoid::Position actPose = cnoid::Position::Identity(); // generate frame
    cnoid::Vector6 actWrench = cnoid::Vector6::Zero(); // generate frame. EndEffector origin. ロボットが受ける力
    cnoid::Position abcTargetPose = cnoid::Position::Identity(); // generate frame. abcで計算された目標位置姿勢
    cnoid::Position stTargetPose = cnoid::Position::Identity(); // generate frame. stで計算された目標位置姿勢
    std::shared_ptr<IK::PositionConstraint> ikPositionConstraint = std::make_shared<IK::PositionConstraint>();
  };
  std::vector<EndEffectorParam> endEffectorParams_; // 要素数2以上. 0: rleg. 1: lleg. 要素数や順序は変化しない

  class LegParam {
  public:
    // constant
    std::string name = ""; // 右脚はrleg. 左脚はllegという名前である必要がある

    // AutoStabilizerの内部で計算される
    bool genContactState = true; // 全てのLegParamのgenContactStateがfalseになることがあるので注意
    bool genContactStatePrev = true;
  //   enum Mode_enum{ MODE_GG, MODE_SYNC_TO_REF, MODE_REF, MODE_SYNC_TO_GG}; // MODE_GG: GaitGeneratorの生成する遊脚軌道に従う. MODE_REF: referenceの目標位置に従う
  //   enum Transition_enum{ START_REF, STOP_REF};
  //   double ref_transition_time;
  // private:
  //   Mode_enum current, previous, next;
  //   double remain_time;
  // public:

    // param
    double refFootOriginWeight = 1.0; // Reference座標系のfootOriginを計算するときに用いるweight. このfootOriginからの相対位置で、GaitGeneratorに管理されていないEndEffectorのReference位置が解釈される. interpolatorによって連続的に変化する. 全てのLegParamのrefFootOriginWeightが0になることはない
    cpp_filters::TwoPointInterpolator<double> refFootOriginWeight_interpolator = cpp_filters::TwoPointInterpolator<double>(1.0,0.0,0.0,cpp_filters::HOFFARBIB);
    cnoid::Vector3 defaultTranslatePos = cnoid::Vector3::Zero();
  };
  std::vector<LegParam> legParams_; // 要素数2. 0: rleg. 1: lleg.

  class FullbodyState {
  public:
    // AutoStabilizerの内部で計算される
    cpp_filters::TwoPointInterpolatorSE3 footMidCoords = cpp_filters::TwoPointInterpolatorSE3(cnoid::Position::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB); // generate frame. 実際の両足位置の中間とは異なり、地面の高さのまま移動し、stride typeによるのXY方向の速さの変化によらずに一定の速度で動く. reference frameとgenerate frameの対応付けはこの座標系を用いて行う. interpolatorによって連続的に変化する
  };
  FullbodyState fullbodyState_;

  class JointParam {
  public:
    // constant
    std::string name = ""; // 関節名
    double maxTorque = 0.0; // モデルのclimit * gearRatio * torqueConstより計算

    // params
    bool controllable = true; // falseの場合、qやtauはrefの値をそのまま出力する. その関節は存在しないかのように扱う. このパラメータはMODE_IDLEのときにしか変更されないので、そこは安心してプログラムを書いて良い
  };
  std::vector<JointParam> jointParams_; // 要素数robot->numJoints(). jointIdの順.

  GaitParam gaitParam_;

protected:
  // utility functions
  bool getProperty(const std::string& key, std::string& ret);

  static bool copyRobotState(cnoid::BodyPtr inRobot, cnoid::BodyPtr outRobot);
  static void moveCoords(cnoid::BodyPtr robot, const cnoid::Position& target, const cnoid::Position& at);

  static bool readInPortData(AutoStabilizer::Ports& ports, cnoid::BodyPtr refRobot, cnoid::BodyPtr actRobot, std::vector<AutoStabilizer::EndEffectorParam>& endEffectorParams);
  static cnoid::Position calcRefFootMidCoords(const cnoid::BodyPtr refRobotOrigin, const std::vector<AutoStabilizer::LegParam>& legParams, const std::vector<AutoStabilizer::EndEffectorParam>& endEffectorParams);
  static bool calcReferenceParameters(const AutoStabilizer::ControlMode& mode, const cnoid::BodyPtr& refRobot, cnoid::BodyPtr& refRobotOrigin, cnoid::BodyPtr& genRobot, std::vector<AutoStabilizer::LegParam>& legParams, std::vector<AutoStabilizer::EndEffectorParam>& endEffectorParams, AutoStabilizer::FullbodyState& fullbodyState);
  static bool calcActualParameters(const AutoStabilizer::ControlMode& mode, const cnoid::BodyPtr& actRobot, cnoid::BodyPtr& actRobotOrigin, std::vector<AutoStabilizer::LegParam>& legParams, std::vector<AutoStabilizer::EndEffectorParam>& endEffectorParams, GaitParam& gaitParam, double dt);
  static bool execAutoBalancer(const AutoStabilizer::ControlMode& mode, const cnoid::BodyPtr& refRobot, cnoid::BodyPtr& refRobotOrigin, const cnoid::BodyPtr& actRobot, cnoid::BodyPtr& actRobotOrigin, cnoid::BodyPtr& genRobot, std::vector<AutoStabilizer::LegParam>& legParams, std::vector<AutoStabilizer::EndEffectorParam>& endEffectorParams, AutoStabilizer::FullbodyState& fullbodyState, GaitParam& gaitParam, double dt);
  static bool execStabilizer();
  class FullbodyIKParam {
  public:
    cnoid::VectorX jlim_avoid_weight_old;
    std::unordered_map<cnoid::LinkPtr,std::shared_ptr<IK::JointAngleConstraint> > refJointAngleConstraint;
    std::shared_ptr<IK::PositionConstraint> rootPositionConstraint;
    std::shared_ptr<IK::COMConstraint> comConstraint;
    std::shared_ptr<IK::AngularMomentumConstraint> angularMomentumConstraint;
  };
  FullbodyIKParam fullbodyIKParam_;
  static bool solveFullbodyIK(cnoid::BodyPtr& genRobot, const cnoid::BodyPtr& refRobot, std::vector<AutoStabilizer::EndEffectorParam>& endEffectorParams, AutoStabilizer::FullbodyIKParam& fullbodyIKParam, const std::vector<cnoid::LinkPtr>& controllableJoints, double dt);
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
