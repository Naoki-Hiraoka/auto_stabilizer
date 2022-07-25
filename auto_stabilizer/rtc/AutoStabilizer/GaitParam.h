#ifndef GAITPARAM_H
#define GAITPARAM_H

#include <cnoid/EigenTypes>
#include <vector>
#include <cpp_filters/TwoPointInterpolator.h>
#include <cpp_filters/FirstOrderLowPassFilter.h>
#include "FootGuidedController.h"

enum leg_enum{RLEG=0, LLEG=1, NUM_LEGS=2};

class GaitParam {
public:
  class FootStepNodes {
  public:
    std::vector<cnoid::Position> dstCoords = std::vector<cnoid::Position>(NUM_LEGS,cnoid::Position::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame. 終了時の位置
    std::vector<double> supportTime = std::vector<double>(NUM_LEGS,std::numeric_limits<double>::max()); // 要素数2. rleg: 0. lleg: 1. remainTimeがこの値以下なら、support期. ずっとsupport期の場合はinfinity(remainTimeが後から長くなることがあるので), ずっとswing期の場合はマイナスにするとよい. swing期の間に、curCoordsからdstCoordsに移動しきるようにswing軌道が生成される. support期のときは、curCoordsからdstCoordsまで直線的に補間がなされる. footstepNodesList[0]については、一度remainTime<=supportTimeが成り立つと、再びremainTime>supportTimeとなるようにremainTimeまたはsupportTimeが変更されることは無い. 両足のsupportTimeがともにマイナスであることはない(終了時は必ずsupport期の足が一つ以上ある)
    double remainTime = 1.0; // step time. 必ず0より大きい. footstepNodesListの末尾の要素のみ、0であることがありえる

    // 遊脚軌道用パラメータ
    std::vector<std::vector<double> > stepHeight = std::vector<std::vector<double> >(NUM_LEGS,std::vector<double>(2,0)); // 要素数2. rleg: 0. lleg: 1. swing期には、srcCoordsの高さ+[0]とdstCoordsの高さ+[1]の高い方に上げるような軌道を生成する
  };
  std::vector<FootStepNodes> footstepNodesList = std::vector<FootStepNodes>(1); // 要素数1以上. 0番目が現在の状態. 末尾の要素以降は、末尾の状態がずっと続くとして扱われる.
  std::vector<cnoid::Position> srcCoords = std::vector<cnoid::Position>(NUM_LEGS,cnoid::Position::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame. footstepNodesList[0]開始時の位置を保持する. 基本的にはfootstepNodesList[-1]のdstCoordsと同じ
  std::vector<cpp_filters::TwoPointInterpolatorSE3> genCoords = std::vector<cpp_filters::TwoPointInterpolatorSE3>(NUM_LEGS, cpp_filters::TwoPointInterpolatorSE3(cnoid::Position::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB)); // 要素数2. rleg: 0. lleg: 1. generate frame. 現在の位置
  std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> > refZmpTraj = {footguidedcontroller::LinearTrajectory<cnoid::Vector3>(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),0.0)}; // 要素数1以上. generate frame. footstepNodesListを単純に線形補間して計算される現在の目標zmp軌道
  cpp_filters::TwoPointInterpolatorSE3 footMidCoords = cpp_filters::TwoPointInterpolatorSE3(cnoid::Position::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB); // generate frame. Z軸は鉛直. footstepNodesListの各要素終了時の支持脚の位置姿勢(Z軸は鉛直)にdefaultTranslatePosを適用したものの間をつなぐ. interpolatorによって連続的に変化する. reference frameとgenerate frameの対応付けに用いられる

  // param
  std::vector<cnoid::Vector3> copOffset = std::vector<cnoid::Vector3>{cnoid::Vector3::Zero(),cnoid::Vector3::Zero()}; // 要素数2. rleg: 0. lleg: 1. leg frame. 足裏COPの目標位置. 幾何的な位置はcopOffset無しで考えるが、目標COPを考えるときはcopOffsetを考慮する
  std::vector<std::vector<cnoid::Vector2> > legPolygon; // 要素数2. rleg: 0. lleg: 1. leg frame.
  std::vector<cnoid::Vector3> defaultTranslatePos = std::vector<cnoid::Vector3>(2,cnoid::Vector3::Zero()); // goPos, goVelocity, その場足踏みをするときの右脚と左脚の中心からの相対位置. あるいは、reference frameとgenerate frameの対応付けに用いられる. (Z軸は鉛直).
  double dz = 1.0; // generate frame. 支持脚からのCogの目標高さ. 0より大きい

  cnoid::Vector3 actCog; // generate frame.  現在のCOM
  cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3> actCogVel = cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3>(4.0, cnoid::Vector3::Zero());  // generate frame.  現在のCOM速度
  cnoid::Vector3 genCog; // generate frame.  現在のCOM
  cnoid::Vector3 genCogVel;  // generate frame.  現在のCOM速度

public:
  bool isSupportPhase(int leg){ // 今がSupportPhaseかどうか
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
};

std::ostream &operator<<(std::ostream &os, const GaitParam& gaitParam);

#endif
