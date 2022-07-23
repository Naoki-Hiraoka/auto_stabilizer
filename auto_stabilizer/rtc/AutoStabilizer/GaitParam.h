#ifndef GAITPARAM_H
#define GAITPARAM_H

#include <cnoid/EigenTypes>
#include <vector>
#include <cpp_filters/TwoPointInterpolator.h>
#include <cpp_filters/FirstOrderLowPassFilter.h>
#include "FootGuidedController.h"

class GaitParam {
public:
  class FootStepNodes {
  public:
    std::vector<cnoid::Position> dstCoords = std::vector<cnoid::Position>(2,cnoid::Position::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame. 終了時の位置
    std::vector<double> supportTime = std::vector<double>(2,std::numeric_limits<double>::max()); // 要素数2. rleg: 0. lleg: 1. remainTimeがこの値以下なら、support期. ずっとsupport期の場合はinfinity(remainTimeが後から長くなることがあるので), ずっとswing期の場合はマイナスにするとよい. swing期の間に、curCoordsからdstCoordsに移動しきるようにswing軌道が生成される. support期のときは、curCoordsからdstCoordsまで直線的に補間がなされる. footstepNodesList[0]については、一度remainTime<=supportTimeが成り立つと、再びremainTime>supportTimeとなるようにremainTimeまたはsupportTimeが変更されることは無い. 両足のsupportTimeがともにマイナスであることはない(終了時は必ずsupport期の足が一つ以上ある)
    double remainTime = 1.0; // step time. 必ず0より大きい. footstepNodesListの末尾の要素のみ、0であることがありえる

    // 遊脚軌道用パラメータ
    std::vector<std::vector<double> > stepHeight = std::vector<std::vector<double> >(2,std::vector<double>(2,0)); // 要素数2. rleg: 0. lleg: 1. swing期には、srcCoordsの高さ+[0]とdstCoordsの高さ+[1]の高い方に上げるような軌道を生成する
  };
  std::vector<FootStepNodes> footstepNodesList = std::vector<FootStepNodes>(1); // 要素数1以上. 0番目が現在の状態. 末尾の要素以降は、末尾の状態がずっと続くとして扱われる.
  std::vector<cnoid::Position> srcCoords = std::vector<cnoid::Position>(2,cnoid::Position::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame. footstepNodesList[0]開始時の位置を保持する. 基本的にはfootstepNodesList[-1]のdstCoordsと同じ
  std::vector<cpp_filters::TwoPointInterpolatorSE3> genCoords; // 要素数2. rleg: 0. lleg: 1. generate frame. 現在の位置
  std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> > refZmpTraj; // 要素数1以上. generate frame. footstepNodesListを単純に線形補間して計算される現在の目標zmp軌道
  bool isSupportPhase(int leg) const{ // 今がSupportPhaseかどうか
    return isSupportPhaseStart(leg, 0);
  }
  bool isSupportPhase(int leg, int footstepNodesindex, double remainTime) const{ // footStepNodesList[footstepNodesindex]のlegは、remainTimeのときにSupportPhaseかどうか
    assert(0<=leg && leg<NUM_LEGS); assert(0<=footstepNodesindex && footstepNodesindex<footstepNodesList.size()); assert(0<=remainTime && remainTime <= footstepNodesList[footstepNodesindex].remainTime);
    return remainTime <= footstepNodesList[footstepNodesindex].supportTime[leg];
  }
  bool isSupportPhaseEnd(int leg, int footstepNodesindex) const{ // footStepNodesList[footstepNodesindex]のlegは、終了時にSupportPhaseかどうか
    assert(0<=leg && leg<NUM_LEGS); assert(0<=footstepNodesindex && footstepNodesindex<footstepNodesList.size());
    return isSupportPhase(leg, footstepNodesindex, 0);
  }
  bool isSupportPhaseStart(int leg, int footstepNodesindex) const{ // footStepNodesList[footstepNodesindex]のlegは、開始時にSupportPhaseかどうか
    assert(0<=leg && leg<NUM_LEGS); assert(0<=footstepNodesindex && footstepNodesindex<footstepNodesList.size());
    return isSupportPhase(leg, footstepNodesindex, footstepNodesList[footstepNodesindex].remainTime);
  }

  // param
  std::vector<cnoid::Vector3> copOffset = std::vector<cnoid::Vector3>{cnoid::Vector3::Zero(),cnoid::Vector3::Zero()}; // 要素数2. rleg: 0. lleg: 1. leg frame. 足裏COPの目標位置. 幾何的な位置はcopOffset無しで考えるが、目標COPを考えるときはcopOffsetを考慮する
  std::vector<std::vector<cnoid::Vector2> > legPolygon; // 要素数2. rleg: 0. lleg: 1. leg frame.
  double dz = 1.0; // generate frame. 支持脚からのCogの目標高さ. 0より大きい

  cnoid::Vector3 actCog; // generate frame.  現在のCOM
  cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3> actCogVel = cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3>(4.0, cnoid::Vector3::Zero());  // generate frame.  現在のCOM速度
  cnoid::Vector3 genCog; // generate frame.  現在のCOM
  cnoid::Vector3 genCogVel;  // generate frame.  現在のCOM速度

  // 遊脚軌道用パラメータ
  double delayTimeOffset = 0.2; // 0以上. swing期は、remainTime - supportTime - delayTimeOffset後にdstCoordsに到達するようなrectangle軌道を生成し、その軌道にdelayTimeOffset遅れで滑らかに追従するような軌道を生成する
  double touchVel = 0.5; // 0より大きい. 足を下ろすときの速さ
  //cnoid::Vector3 goal_off; // TODO

  // 足配置決定用パラメータ
  std::vector<std::vector<cnoid::Vector2> > steppable_region; // 要素数任意. generate frame. endCoordsが存在できる領域
  std::vector<double> steppable_height; // 要素数はsteppable_regionと同じ. generate frame. 各polygonごとのおおよその値. そのpolygonに届くかどうかの判定と、
  double relLandingHeight; // generate frame. 現在の遊脚のfootstepNodesList[0]のdstCoordsのZ
  cnoid::Vector3 relLandingNormal; // generate frame. 現在の遊脚のfootstepNodesList[0]のdstCoordsのZ軸の方向
};

enum leg_enum{RLEG=0, LLEG=1, NUM_LEGS=2};

#endif
