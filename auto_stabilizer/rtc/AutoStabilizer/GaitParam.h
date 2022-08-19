#ifndef GAITPARAM_H
#define GAITPARAM_H

#include <sys/time.h>
#include <cnoid/EigenTypes>
#include <vector>
#include <cpp_filters/TwoPointInterpolator.h>
#include <cpp_filters/FirstOrderLowPassFilter.h>
#include <joint_limit_table/JointLimitTable.h>
#include "FootGuidedController.h"

enum leg_enum{RLEG=0, LLEG=1, NUM_LEGS=2};

class GaitParam {
public:
  // constant
  std::vector<std::string> eeName; // constant. 要素数2以上. 0番目がrleg, 1番目がllegという名前である必要がある
  std::vector<std::string> eeParentLink; // constant. 要素数と順序はeeNameと同じ. 必ずrobot->link(parentLink)がnullptrではないことを約束する. そのため、毎回robot->link(parentLink)がnullptrかをチェックしなくても良い
  std::vector<cnoid::Position> eeLocalT; // constant. 要素数と順序はeeNameと同じ. Parent Link Frame

  std::vector<double> maxTorque; // constant. 要素数と順序はnumJoints()と同じ. 単位は[Nm]. 0以上
  std::vector<std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > > jointLimitTables; // constant. 要素数と順序はnumJoints()と同じ. for genRobot.

  const double g = 9.80665; // 重力加速度
public:
  // AutoStabilizerの中で計算更新される.

  // refToGenFrameConverter
  std::vector<cnoid::Position> refEEPose; // 要素数と順序はeeNameと同じ.generate frame
  std::vector<cnoid::Vector6> refEEWrench; // 要素数と順序はeeNameと同じ.generate frame. EndEffector origin. ロボットが受ける力
  double refdz = 1.0; // generate frame. 支持脚からのCogの目標高さ. 0より大きい
  cpp_filters::TwoPointInterpolatorSE3 footMidCoords = cpp_filters::TwoPointInterpolatorSE3(cnoid::Position::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB); // generate frame. Z軸は鉛直. 支持脚の位置姿勢(Z軸は鉛直)にdefaultTranslatePosを適用したものの間をつなぐ. interpolatorによって連続的に変化する. reference frameとgenerate frameの対応付けに用いられる

  // actToGenFrameConverter
  cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3> actCogVel = cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3>(4.0, cnoid::Vector3::Zero());  // generate frame.  現在のCOM速度. cutoff=4.0Hzは今の歩行時間と比べて遅すぎる気もするが、実際のところ問題なさそう?
  std::vector<cnoid::Position> actEEPose; // 要素数と順序はeeNameと同じ.generate frame
  std::vector<cnoid::Vector6> actEEWrench; // 要素数と順序はeeNameと同じ.generate frame. EndEffector origin. ロボットが受ける力

  // ExternalForceHandler
  double omega = std::sqrt(g / refdz); // DCMの計算に用いる. 0より大きい
  cnoid::Vector3 l = cnoid::Vector3(0, 0, refdz); // generate frame. FootGuidedControlで外力を計算するときの、ZMP-重心の相対位置に対するオフセット. また、CMPの計算時にDCMに対するオフセット(CMP + l = DCM). 連続的に変化する.
  cnoid::Vector3 sbpOffset = cnoid::Vector3::Zero(); // generate frame. 外力考慮重心と重心のオフセット. genCog = genRobot->centerOfMass() - sbpOffset. actCog = actRobot->centerOfMass() - sbpOffset.
  cnoid::Vector3 actCog; // generate frame. 現在のCOMにsbpOffsetを施したもの actCog = actRobot->centerOfMass() - sbpOffset

  // ImpedanceController
  std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> > icEEOffset; // 要素数と順序はeeNameと同じ.generate frame. endEffector origin. icで計算されるオフセット
  std::vector<cnoid::Position> icEETargetPose; // 要素数と順序はeeNameと同じ.generate frame. icで計算された目標位置姿勢

  // CmdVelGenerator
  cnoid::Vector3 cmdVel = cnoid::Vector3::Zero(); // X[m/s] Y[m/s] theta[rad/s]. Z軸はgenerate frame鉛直. support leg frame. 不連続に変化する

  // FootStepGenerator
  class FootStepNodes {
  public:
    /*
      各足につきそれぞれ、remainTime 後に dstCoordsに動く.

      footstepNodesList[0]のisSupportPhaseは、変更されない
      footstepNodesList[0]のdstCoordsを変更する場合には、footstepNodesList[0]のremainTimeの小ささに応じて変更量を小さくする. remainTimeがほぼゼロなら変更量もほぼゼロ.
      footstepNodesList[0]は、!footstepNodesList[0].isSupportPhase && footstepNodesList[1].isSupportPhaseの足がある場合に、突然footstepNodesList[1]に遷移する場合がある(footStepGeneratorのearlyTouchDown)
      それ以外には、footstepNodesList[0]のremainTimeが突然0になることはない
      footstepNodesList[1]のisSupportPhaseは、footstepNodesListのサイズが1である場合を除いて変更されない.
      両足支持期の次のstepのisSupportPhaseを変えたり、後ろに新たにfootstepNodesを追加する場合、必ず両足支持期のremainTimeをそれなりに長い時間にする(片足に重心やrefZmpを移す補間の時間のため)
    */
    std::vector<cnoid::Position> dstCoords = std::vector<cnoid::Position>(NUM_LEGS,cnoid::Position::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame.
    std::vector<bool> isSupportPhase = std::vector<bool>(NUM_LEGS, true); // 要素数2. rleg: 0. lleg: 1. footstepNodesListの末尾の要素が両方falseであることは無い
    double remainTime = 0.0;

    // 遊脚軌道用パラメータ
    std::vector<std::vector<double> > stepHeight = std::vector<std::vector<double> >(NUM_LEGS,std::vector<double>(2,0)); // 要素数2. rleg: 0. lleg: 1. swing期には、srcCoordsの高さ+[0]とdstCoordsの高さ+[1]の高い方に上げるような軌道を生成する

    // 遊脚軌道用状態変数. (footstepNodesList[0]の値のみ意味がある. GaitParamのメンバ変数にしても良いが、footstepNodesListが次の要素に進んだときに毎回初期化するのが面倒だったので、FootStepNodesのメンバ変数にしている)
    enum SwingState_enum{LIFT_PHASE, SWING_PHASE, DOWN_PHASE};
    std::vector<SwingState_enum> swingState = std::vector<SwingState_enum>(NUM_LEGS,LIFT_PHASE); // 要素数2. rleg: 0. lleg: 0. footstepNodesList[1:]は全てLIFT_PHASEにしておく. footstepNodesList[0]は, isSupportPhase = falseの脚は、LIFT_PHASE->SWING_PHASE->DOWN_PHASEと遷移する. 一度DOWN_PHASEになったら別のPHASEになることはない. DOWN_PHASEのときはfootstepNodesList[0]のdstCoordsはgenCoordsよりも高い位置に変更されることはない
    double elapsedTime = 0.0; // このindexが始まってからの経過時間を積算したもの
  };
  std::vector<FootStepNodes> footstepNodesList = std::vector<FootStepNodes>(1); // 要素数1以上. 0番目が現在の状態. 末尾の要素以降は、末尾の状態がずっと続くとして扱われる.
  std::vector<cnoid::Position> srcCoords = std::vector<cnoid::Position>(NUM_LEGS,cnoid::Position::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame. 現在のfootstep開始時の位置
  std::vector<cnoid::Position> dstCoordsOrg = std::vector<cnoid::Position>(NUM_LEGS,cnoid::Position::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame. 現在のfootstep開始時のdstCoords
  std::vector<bool> prevSupportPhase = std::vector<bool>{true, true}; // 要素数2. rleg: 0. lleg: 1. 一つ前の周期でSupportPhaseだったかどうか

  // LegCoordsGenerator
  std::vector<cpp_filters::TwoPointInterpolatorSE3> genCoords = std::vector<cpp_filters::TwoPointInterpolatorSE3>(NUM_LEGS, cpp_filters::TwoPointInterpolatorSE3(cnoid::Position::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB)); // 要素数2. rleg: 0. lleg: 1. generate frame. 現在の位置
  std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> > refZmpTraj = {footguidedcontroller::LinearTrajectory<cnoid::Vector3>(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),0.0)}; // 要素数1以上. generate frame. footstepNodesListを単純に線形補間して計算される現在の目標zmp軌道

  cnoid::Vector3 genCog; // generate frame. abcで計算された目標COM
  cnoid::Vector3 genCogVel;  // generate frame.  abcで計算された目標COM速度
  std::vector<cnoid::Position> abcEETargetPose; // 要素数と順序はeeNameと同じ.generate frame. abcで計算された目標位置姿勢

  // Stabilizer
  cpp_filters::TwoPointInterpolator<cnoid::Vector3> stOffsetRootRpy = cpp_filters::TwoPointInterpolator<cnoid::Vector3>(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::LINEAR);; // gaitParam.footMidCoords座標系. stで計算された目標位置姿勢オフセット
  cnoid::Position stTargetRootPose = cnoid::Position::Identity(); // generate frame
  std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> > stEEOffsetDampingControl; // 要素数と順序はeeNameと同じ.generate frame. endEffector origin. stで計算されるオフセット(Damping Control)
  std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> > stEEOffsetSwingEEModification; // 要素数と順序はeeNameと同じ.generate frame. endEffector origin. stで計算されるオフセット(SwingEEModification)
  std::vector<cnoid::Position> stEETargetPose; // 要素数と順序はeeNameと同じ.generate frame. stで計算された目標位置姿勢
  cnoid::Vector3 stTargetZmp; // generate frame. stで計算された目標ZMP
  std::vector<cpp_filters::TwoPointInterpolator<double> > stServoPGainPercentage; // 要素数と順序はrobot->numJoints()と同じ. 0~100. 現状, setGoal(*,dt)以下の時間でgoal指定するとwriteOutPortDataが破綻する
  std::vector<cpp_filters::TwoPointInterpolator<double> > stServoDGainPercentage; // 要素数と順序はrobot->numJoints()と同じ. 0~100. 現状, setGoal(*,dt)以下の時間でgoal指定するとwriteOutPortDataが破綻する

public:
  // param
  std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> > copOffset = std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >{cpp_filters::TwoPointInterpolator<cnoid::Vector3>(cnoid::Vector3(0.0,0.02,0.0),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB),cpp_filters::TwoPointInterpolator<cnoid::Vector3>(cnoid::Vector3(0.0,-0.02,0.0),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB)}; // 要素数2. rleg: 0. lleg: 1. leg frame. 足裏COPの目標位置. 幾何的な位置はcopOffset.value()無しで考えるが、目標COPを考えるときはcopOffset.value()を考慮する. クロスできたりジャンプできたりする脚でないと左右方向(外側向き)の着地位置修正は難しいので、その方向に転びそうになることが極力ないように内側にcopをオフセットさせておくとよい. 滑らかに変化する
  std::vector<std::vector<cnoid::Vector3> > legHull = std::vector<std::vector<cnoid::Vector3> >(2, std::vector<cnoid::Vector3>{cnoid::Vector3(0.115,0.065,0.0),cnoid::Vector3(-0.105,0.065,0.0),cnoid::Vector3(-0.105,-0.065,0.0),cnoid::Vector3(0.115,-0.065,0.0)}); // 要素数2. rleg: 0. lleg: 1. leg frame.  凸形状で,上から見て半時計回り. Z成分はあったほうが計算上扱いやすいからありにしているが、0でなければならない
  std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> > defaultTranslatePos = std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >(2,cpp_filters::TwoPointInterpolator<cnoid::Vector3>(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB)); // goPos, goVelocity, その場足踏みをするときの右脚と左脚の中心からの相対位置. あるいは、reference frameとgenerate frameの対応付けに用いられる. (Z軸は鉛直). Z成分はあったほうが計算上扱いやすいからありにしているが、0でなければならない. RefToGenFrameConverterが「左右」という概念を使うので、X成分も0でなければならない. 滑らかに変化する
  std::vector<cpp_filters::TwoPointInterpolator<double> > isManualControlMode = std::vector<cpp_filters::TwoPointInterpolator<double> >(2, cpp_filters::TwoPointInterpolator<double>(0.0, 0.0, 0.0, cpp_filters::LINEAR)); // 要素数2. 0: rleg. 1: lleg. 0~1. 連続的に変化する. 1ならicEETargetPoseに従い、refEEWrenchに応じて重心をオフセットする. 0ならImpedanceControlをせず、refEEWrenchを無視し、DampingControlやSwingEEModificationを行わない. 静止状態で無い場合や、支持脚の場合は、勝手に0になる. 両足が同時に1になることはない. 1にするなら、RefToGenFrameConverter.refFootOriginWeightを0にしたほうが良い. 1の状態でStartAutoStabilizerすると、遊脚で始まる

  std::vector<bool> jointControllable; // 要素数と順序はnumJoints()と同じ. falseの場合、qやtauはrefの値をそのまま出力する(writeOutputPort時にref値で上書き). IKでは動かさない(ref値をそのまま). トルク計算では目標トルクを通常通り計算する. このパラメータはMODE_IDLEのときにしか変更されない

public:
  // from reference port
  std::vector<cnoid::Vector6> refEEWrenchOrigin; // 要素数と順序はeeNameと同じ.FootOrigin frame. EndEffector origin. ロボットが受ける力

public:
  bool isStatic() const{ // 現在static状態かどうか
    return this->footstepNodesList.size() == 1 && this->footstepNodesList[0].remainTime == 0.0;
  }

public:
  // startAutoStabilizer時に呼ばれる
  void reset(){
    for(int i=0;i<NUM_LEGS;i++){
      copOffset[i].reset(copOffset[i].getGoal());
      defaultTranslatePos[i].reset(defaultTranslatePos[i].getGoal());
      isManualControlMode[i].reset(isManualControlMode[i].getGoal());
    }
  }

  // 内部の補間器をdtだけ進める
  void update(double dt){
    for(int i=0;i<NUM_LEGS;i++){
      copOffset[i].interpolate(dt);
      defaultTranslatePos[i].interpolate(dt);
    }
  }

  void init(const cnoid::BodyPtr& genRobot){
    stServoPGainPercentage.resize(genRobot->numJoints(), cpp_filters::TwoPointInterpolator<double>(100.0, 0.0, 0.0, cpp_filters::LINEAR));
    stServoDGainPercentage.resize(genRobot->numJoints(), cpp_filters::TwoPointInterpolator<double>(100.0, 0.0, 0.0, cpp_filters::LINEAR));
  }

  void push_backEE(const std::string& name_, const std::string& parentLink_, const cnoid::Position& localT_){
    eeName.push_back(name_);
    eeParentLink.push_back(parentLink_);
    eeLocalT.push_back(localT_);
    refEEWrenchOrigin.push_back(cnoid::Vector6::Zero());
    refEEPose.push_back(cnoid::Position::Identity());
    refEEWrench.push_back(cnoid::Vector6::Zero());
    actEEPose.push_back(cnoid::Position::Identity());
    actEEWrench.push_back(cnoid::Vector6::Zero());
    icEEOffset.push_back(cpp_filters::TwoPointInterpolator<cnoid::Vector6>(cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(), cpp_filters::HOFFARBIB));
    icEETargetPose.push_back(cnoid::Position::Identity());
    abcEETargetPose.push_back(cnoid::Position::Identity());
    stEEOffsetDampingControl.push_back(cpp_filters::TwoPointInterpolator<cnoid::Vector6>(cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(), cpp_filters::HOFFARBIB));
    stEEOffsetSwingEEModification.push_back(cpp_filters::TwoPointInterpolator<cnoid::Vector6>(cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(), cpp_filters::HOFFARBIB));
    stEETargetPose.push_back(cnoid::Position::Identity());
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // for debug
  mutable struct timeval prevTime;
  void resetTime() const { gettimeofday(&prevTime, NULL);}
  void printTime(const std::string& message="") const {
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    std::cerr << message << (currentTime.tv_sec - prevTime.tv_sec) + (currentTime.tv_usec - prevTime.tv_usec) * 1e-6 << std::endl;
  }
};

inline std::ostream &operator<<(std::ostream &os, const GaitParam& gaitParam) {
  os << "current" << std::endl;
  os << " RLEG: " << std::endl;
  os << "  pos: " << (gaitParam.genCoords[RLEG].value().translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
  os << "  rot: " << (gaitParam.genCoords[RLEG].value().linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
  os << " LLEG: " << std::endl;
  os << "  pos: " << (gaitParam.genCoords[LLEG].value().translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
  os << "  rot: " << (gaitParam.genCoords[LLEG].value().linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
  for(int i=0;i<gaitParam.footstepNodesList.size();i++){
    os << "footstep" << i << std::endl;
    os << " RLEG: " << std::endl;
    os << "  pos: " << (gaitParam.footstepNodesList[i].dstCoords[RLEG].translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
    os << "  rot: " << (gaitParam.footstepNodesList[i].dstCoords[RLEG].linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
    os << "  height = " << gaitParam.footstepNodesList[i].stepHeight[RLEG][0] << " " << gaitParam.footstepNodesList[i].stepHeight[RLEG][1] << std::endl;
    os << " LLEG: " << std::endl;
    os << "  pos: " << (gaitParam.footstepNodesList[i].dstCoords[LLEG].translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
    os << "  rot: " << (gaitParam.footstepNodesList[i].dstCoords[LLEG].linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
    os << "  height = " << gaitParam.footstepNodesList[i].stepHeight[LLEG][0] << " " << gaitParam.footstepNodesList[i].stepHeight[LLEG][1] << std::endl;
    os << " time = " << gaitParam.footstepNodesList[i].remainTime << "[s]" << std::endl;;
  }
  return os;
};

inline std::ostream &operator<<(std::ostream &os, const std::vector<GaitParam::FootStepNodes>& footstepNodesList) {
  for(int i=0;i<footstepNodesList.size();i++){
    os << "footstep" << i << std::endl;
    os << " RLEG: " << std::endl;
    os << "  pos: " << (footstepNodesList[i].dstCoords[RLEG].translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
    os << "  rot: " << (footstepNodesList[i].dstCoords[RLEG].linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
    os << "  height = " << footstepNodesList[i].stepHeight[RLEG][0] << " " << footstepNodesList[i].stepHeight[RLEG][1] << std::endl;
    os << " LLEG: " << std::endl;
    os << "  pos: " << (footstepNodesList[i].dstCoords[LLEG].translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
    os << "  rot: " << (footstepNodesList[i].dstCoords[LLEG].linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
    os << "  height = " << footstepNodesList[i].stepHeight[LLEG][0] << " " << footstepNodesList[i].stepHeight[LLEG][1] << std::endl;
    os << " time = " << footstepNodesList[i].remainTime << "[s]" << std::endl;;
  }
  return os;
};


#endif
