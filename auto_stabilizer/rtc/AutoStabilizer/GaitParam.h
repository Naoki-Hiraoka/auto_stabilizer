#ifndef GAITPARAM_H
#define GAITPARAM_H

#include <cnoid/EigenTypes>
#include <vector>
#include <cpp_filters/TwoPointInterpolator.h>
#include <cpp_filters/FirstOrderLowPassFilter.h>
#include <ik_constraint/PositionConstraint.h>
#include "FootGuidedController.h"

enum leg_enum{RLEG=0, LLEG=1, NUM_LEGS=2};

class GaitParam {
public:
  // constant
  std::vector<std::string> eeName; // constant. 要素数2以上. 0番目がrleg, 1番目がllegという名前である必要がある
  std::vector<std::string> eeParentLink; // constant. 要素数と順序はeeNameと同じ. 必ずrobot->link(parentLink)がnullptrではないことを約束する. そのため、毎回robot->link(parentLink)がnullptrかをチェックしなくても良い
  std::vector<cnoid::Position> eeLocalT; // constant. 要素数と順序はeeNameと同じ. Parent Link Frame

public:
  // AutoStabilizerの中で計算更新される.

  std::vector<cnoid::Position> refEEPose; // 要素数と順序はeeNameと同じ.generate frame
  std::vector<cnoid::Vector6> refEEWrench; // 要素数と順序はeeNameと同じ.generate frame. EndEffector origin. ロボットが受ける力
  double refdz = 1.0; // generate frame. 支持脚からのCogの目標高さ. 0より大きい

  cnoid::Vector3 actCog; // generate frame.  現在のCOM
  cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3> actCogVel = cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3>(4.0, cnoid::Vector3::Zero());  // generate frame.  現在のCOM速度. cutoff=4.0Hzは今の歩行時間と比べて遅すぎる気もするが、実際のところ問題なさそう?
  std::vector<cnoid::Position> actEEPose; // 要素数と順序はeeNameと同じ.generate frame
  std::vector<cnoid::Vector6> actEEWrench; // 要素数と順序はeeNameと同じ.generate frame. EndEffector origin. ロボットが受ける力

  std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> > icEEOffset; // 要素数と順序はeeNameと同じ.generate frame. endEffector origin. icで計算されるオフセット
  std::vector<cnoid::Position> icEETargetPose; // 要素数と順序はeeNameと同じ.generate frame. icで計算された目標位置姿勢

  class FootStepNodes {
  public:
    std::vector<cnoid::Position> dstCoords = std::vector<cnoid::Position>(NUM_LEGS,cnoid::Position::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame. 終了時の位置
    std::vector<double> supportTime = std::vector<double>(NUM_LEGS,std::numeric_limits<double>::max()); // 要素数2. rleg: 0. lleg: 1. remainTimeがこの値以下なら、support期. ずっとsupport期の場合はinfinity(remainTimeが後から長くなることがあるので), ずっとswing期の場合はマイナスにするとよい. swing期の間に、curCoordsからdstCoordsに移動しきるようにswing軌道が生成される. support期のときは、curCoordsからdstCoordsまで直線的に補間がなされる. footstepNodesList[0]については、一度remainTime<=supportTimeが成り立つと、再びremainTime>supportTimeとなるようにremainTimeまたはsupportTimeが変更されることは無い. footstepNodesList[0]については、supportTimeとremainTimeの大小関係が変化することは時間経過によるもの以外はない. 両足のsupportTimeがともにマイナスであることはない(終了時は必ずsupport期の足が一つ以上ある)
    double remainTime = 1.0; // step time. 必ず0より大きい. footstepNodesListの末尾の要素のみ、0であることがありえる

    // 遊脚軌道用パラメータ
    std::vector<std::vector<double> > stepHeight = std::vector<std::vector<double> >(NUM_LEGS,std::vector<double>(2,0)); // 要素数2. rleg: 0. lleg: 1. swing期には、srcCoordsの高さ+[0]とdstCoordsの高さ+[1]の高い方に上げるような軌道を生成する
  };
  std::vector<FootStepNodes> footstepNodesList = std::vector<FootStepNodes>(1); // 要素数1以上. 0番目が現在の状態. 末尾の要素以降は、末尾の状態がずっと続くとして扱われる.
  std::vector<cnoid::Position> srcCoords = std::vector<cnoid::Position>(NUM_LEGS,cnoid::Position::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame. footstepNodesList[0]開始時の位置を保持する. 基本的にはfootstepNodesList[-1]のdstCoordsと同じ
  std::vector<cpp_filters::TwoPointInterpolatorSE3> genCoords = std::vector<cpp_filters::TwoPointInterpolatorSE3>(NUM_LEGS, cpp_filters::TwoPointInterpolatorSE3(cnoid::Position::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB)); // 要素数2. rleg: 0. lleg: 1. generate frame. 現在の位置
  std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> > refZmpTraj = {footguidedcontroller::LinearTrajectory<cnoid::Vector3>(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),0.0)}; // 要素数1以上. generate frame. footstepNodesListを単純に線形補間して計算される現在の目標zmp軌道
  cpp_filters::TwoPointInterpolatorSE3 footMidCoords = cpp_filters::TwoPointInterpolatorSE3(cnoid::Position::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB); // generate frame. Z軸は鉛直. footstepNodesListの各要素終了時の支持脚の位置姿勢(Z軸は鉛直)にdefaultTranslatePosを適用したものの間をつなぐ. interpolatorによって連続的に変化する. reference frameとgenerate frameの対応付けに用いられる
  std::vector<bool> prevSupportPhase = std::vector<bool>{true, true}; // 要素数2. rleg: 0. lleg: 1. 一つ前の周期でSupportPhaseだったかどうか

  cnoid::Vector3 genCog; // generate frame. abcで計算された目標COM
  cnoid::Vector3 genCogVel;  // generate frame.  abcで計算された目標COM速度
  std::vector<cnoid::Position> abcEETargetPose; // 要素数と順序はeeNameと同じ.generate frame. abcで計算された目標位置姿勢

  cpp_filters::TwoPointInterpolator<cnoid::Vector3> stOffsetRootRpy = cpp_filters::TwoPointInterpolator<cnoid::Vector3>(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB);; // gaitParam.footMidCoords座標系. stで計算された目標位置姿勢オフセット
  cnoid::Position stTargetRootPose = cnoid::Position::Identity(); // generate frame
  std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> > stEEOffset; // 要素数と順序はeeNameと同じ.generate frame. endEffector origin. stで計算されるオフセット
  std::vector<cnoid::Position> stEETargetPose; // 要素数と順序はeeNameと同じ.generate frame. stで計算された目標位置姿勢

  std::vector<std::shared_ptr<IK::PositionConstraint> > ikEEPositionConstraint; // 要素数と順序はeeNameと同じ.

public:
  // param
  std::vector<cnoid::Vector3> copOffset = std::vector<cnoid::Vector3>{cnoid::Vector3::Zero(),cnoid::Vector3::Zero()}; // 要素数2. rleg: 0. lleg: 1. leg frame. 足裏COPの目標位置. 幾何的な位置はcopOffset無しで考えるが、目標COPを考えるときはcopOffsetを考慮する
  std::vector<std::vector<cnoid::Vector3> > legHull = std::vector<std::vector<cnoid::Vector3> >(2, std::vector<cnoid::Vector3>{cnoid::Vector3(0.115,0.065,0.0),cnoid::Vector3(-0.115,0.065,0.0),cnoid::Vector3(-0.115,-0.065,0.0),cnoid::Vector3(0.115,-0.065,0.0)}); // 要素数2. rleg: 0. lleg: 1. leg frame.  凸形状で,上から見て半時計回り. Z成分はあったほうが計算上扱いやすいからありにしているが、0でなければならない
  std::vector<cnoid::Vector3> defaultTranslatePos = std::vector<cnoid::Vector3>(2,cnoid::Vector3::Zero()); // goPos, goVelocity, その場足踏みをするときの右脚と左脚の中心からの相対位置. あるいは、reference frameとgenerate frameの対応付けに用いられる. (Z軸は鉛直). Z成分はあったほうが計算上扱いやすいからありにしているが、0でなければならない

  std::vector<bool> isLegAutoControlMode = std::vector<bool>{true,true}; // 要素数2. rleg: 0. lleg: 1. 脚軌道生成器が自動で位置姿勢を生成するか(true)、reference軌道を使うか(false) TODO

public:
  // from reference port
  std::vector<cnoid::Vector6> refEEWrenchOrigin; // 要素数と順序はeeNameと同じ.FootOrigin frame. EndEffector origin. ロボットが受ける力

public:
  bool isSupportPhase(int leg) const{ // 今がSupportPhaseかどうか
    return isSupportPhaseStart(this->footstepNodesList[0], leg);
  }
  static bool isSupportPhase(const FootStepNodes& footstepNodes, int leg, double remainTime){ // footStepNodesのlegは、remainTimeのときにSupportPhaseかどうか
    assert(0<=leg && leg<NUM_LEGS); assert(0<=remainTime && remainTime <= footstepNodes.remainTime);
    return remainTime <= footstepNodes.supportTime[leg];
  }
  static bool isSupportPhaseEnd(const FootStepNodes& footstepNodes, int leg){ // footStepNodesのlegは、終了時にSupportPhaseかどうか
    assert(0<=leg && leg<NUM_LEGS);
    return isSupportPhase(footstepNodes, leg, 0);
  }
  static bool isSupportPhaseStart(const FootStepNodes& footstepNodes, int leg){ // footStepNodesのlegは、開始時にSupportPhaseかどうか
    assert(0<=leg && leg<NUM_LEGS);
    return isSupportPhase(footstepNodes, leg, footstepNodes.remainTime);
  }
  bool isStatic() const{ // 現在static状態かどうか
    this->footstepNodesList.size() == 1 && this->footstepNodesList[0].remainTime == 0.0;
  }

public:
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
    stEEOffset.push_back(cpp_filters::TwoPointInterpolator<cnoid::Vector6>(cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(), cpp_filters::HOFFARBIB));
    stEETargetPose.push_back(cnoid::Position::Identity());
    ikEEPositionConstraint.push_back(std::make_shared<IK::PositionConstraint>());
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


#endif
