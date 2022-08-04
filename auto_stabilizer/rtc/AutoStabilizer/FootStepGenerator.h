#ifndef FOOTSTEPGENERATOR_H
#define FOOTSTEPGENERATOR_H

#include "GaitParam.h"


class FootStepGenerator{
public:
  // FootStepGeneratorでしか使わないパラメータ
  double defaultStepTime = 0.8; // [s]
  double defaultDoubleSupportTime = 0.12; // [s]. defaultStepTime未満である必要がある.
  double defaultStepHeight = 0.07; // [s].
  unsigned int goVelocityStepNum = 3; // 1以上
  bool modifyFootSteps = false; // 着地位置修正を行うかどうか
  bool isEmergencyStepMode = false; // footstepNodesList[0]が末尾の要素でかつ現在のdstCoordsのままだとバランスが取れないなら、footstepNodesList[1]に両脚が横に並ぶ位置に一歩歩くnodeが末尾に入る. (modifyFootSteps=trueのときのみ有効)

  // FootStepGeneratorでしか使わないパラメータ. startAutoBalancer時に初期化が必要
  bool isGoVelocityMode = false; // 進行方向に向けてfootStepNodesList[1] ~ footStepNodesList[goVelocityStepNum]の要素をfootstepNodesList[0]から機械的に計算してどんどん位置修正&末尾appendしていく.
  cnoid::Vector3 cmdVel = cnoid::Vector3::Zero(); // X[m/s] Y[m/s] theta[rad/s]. Z軸はgenerate frame鉛直

  std::vector<std::vector<cnoid::Vector2> > steppable_region; // 要素数任意. generate frame. endCoordsが存在できる領域
  std::vector<double> steppable_height; // 要素数はsteppable_regionと同じ. generate frame. 各polygonごとのおおよその値. そのpolygonに届くかどうかの判定と、
  double relLandingHeight; // generate frame. 現在の遊脚のfootstepNodesList[0]のdstCoordsのZ
  cnoid::Vector3 relLandingNormal; // generate frame. 現在の遊脚のfootstepNodesList[0]のdstCoordsのZ軸の方向

  void reset(){
    isGoVelocityMode = false;
    cmdVel = cnoid::Vector3::Zero();
  }

  /*
    footstepNodesList[1]開始時のsupport/swingの状態を上書きによって変更する場合は、footstepNodesList[0]の終了時の状態が両脚支持でかつその期間の時間がdefaultDoubleSupportTimeよりも短いなら延長する

    外部からsetFootStepsやgoPosをされるfootstepNodesList[1]~がその値で上書きされる. ただしfootstepNodesList[0]終了時にswing状態の足をfootstepNodesList[1]開始時にsupportにする必要がある場合は、footstepNodesList[0]の直後に両脚が横に並ぶ位置に一歩歩いてその足を下ろすnodeが挿入される.

    外部からgoStopをされると、footstepNodesListの末尾に両脚が横に並ぶ位置に2歩歩くnodeが入る. 外部からgoVelocityModeをfalseにすること

    goVelocityModeなら、進行方向に向けてfootStepNodesList[1] ~ footStepNodesList[goVelocityStepNum]の要素を機械的に計算してどんどん末尾appendしていく. cmdVelに応じてきまる

    跳躍中でなければ、
      emergencyStepModeなら、footstepNodesList[0]が末尾の要素でかつ現在のdstCoordsのままだとバランスが取れないなら、footstepNodesList[1]に両脚が横に並ぶ位置に一歩歩くnodeが末尾に入る. (modifyFootStepsと併用せよ)
      modifyFootStepsなら、footstepNodesList[0]の現在のdstCoordsのままだとバランスが取れないか、今swing期でfootstepNodesList[0]終了時に着地する予定の要素のdstCoordsが着地可能領域上にないなら、footstepNodesList[0]の、今swing期でfootstepNodesList[0]終了時に着地する予定の要素を修正する. また、それ以降一回でもswingする要素の位置を平行移動する
  */
public:
  class StepNode {
  public:
    int l_r; // 0: RLEG. 1: LLEG
    cnoid::Position coords;
    double stepHeight, stepTime;
  public:
    StepNode () : l_r(RLEG), coords(cnoid::Position::Identity()), stepHeight(), stepTime(){};
    StepNode (const int _l_r, const cnoid::Position& _coords, const double _stepHeight, const double _stepTime)
      : l_r(_l_r), coords(_coords), stepHeight(_stepHeight), stepTime(_stepTime) {};
    friend std::ostream &operator<<(std::ostream &os, const StepNode &sn) {
      os << "footstep" << std::endl;
      os << "  name = [" << ((sn.l_r==LLEG)?std::string("lleg"):
                             std::string("rleg")) << "]" << std::endl;
      os << "  pos =";
      os << (sn.coords.translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
      os << "  rot =";
      os << (sn.coords.linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
      os << "  step_height = " << sn.stepHeight << "[m], step_time = " << sn.stepTime << "[s]";
      return os;
    };
  };
  /*
    footstepNodesList[1]開始時のsupport/swingの状態を上書きによって変更する場合は、footstepNodesList[0]の終了時の状態が両脚支持でかつその期間の時間がdefaultDoubleSupportTimeよりも短いなら延長する

    外部からsetFootStepsやgoPosをされると呼ばれる
    footstepNodesList[1]~がその値で上書きされる. ただしfootstepNodesList[0]終了時にswing状態の足をfootstepNodesList[1]開始時にsupportにする必要がある場合は、footstepNodesList[0]の直後に両脚が横に並ぶ位置に一歩歩いてその足を下ろすnodeが挿入される.
    footstepsの0番目の要素は、実際には歩かず、基準座標としてのみ使われる. footstepNodesList[0].dstCoordsのZ軸を鉛直に直した座標系と、footstepsの0番目の要素のZ軸を鉛直に直した座標系が一致するように座標変換する.
   */
  bool setFootSteps(const GaitParam& gaitParam, const std::vector<StepNode>& footsteps,
                    std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const;

  // footstepNodesListの末尾に両脚が横に並ぶ位置に2歩歩くnodeが入る. 外部からgoVelocityModeをfalseにすること
  bool goStop(const GaitParam& gaitParam,
                    std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const;

  /*
    footstepNodesList[1]開始時のsupport/swingの状態を上書きによって変更する場合は、footstepNodesList[0]の終了時の状態が両脚支持でかつその期間の時間がdefaultDoubleSupportTimeよりも短いなら延長する

    goVelocityModeなら、進行方向に向けてfootStepNodesList[1] ~ footStepNodesList[goVelocityStepNum]の要素を機械的に計算してどんどん末尾appendしていく. cmdVelに応じてきまる

    跳躍中でなければ、
    emergencyStepModeなら、footstepNodesList[0]が末尾の要素でかつ現在のdstCoordsのままだとバランスが取れないなら、footstepNodesList[1]に両脚が横に並ぶ位置に一歩歩くnodeが末尾に入る. (modifyFootStepsと併用せよ)
    modifyFootStepsなら、footstepNodesList[0]の現在のdstCoordsのままだとバランスが取れないか、今swing期でfootstepNodesList[0]終了時に着地する予定の要素のdstCoordsが着地可能領域上にないなら、footstepNodesList[0]の、今swing期でfootstepNodesList[0]終了時に着地する予定の要素を修正する. また、それ以降一回でもswingする要素の位置を平行移動する

  */
  bool calcFootSteps(const GaitParam& gaitParam, const double& dt,
                     std::vector<GaitParam::FootStepNodes>& o_footstepNodesList, std::vector<cnoid::Position>& o_srcCoords) const;


protected:
  // footstepNodesの次の一歩を作る. 両脚が地面についた状態で終わる. RLEGとLLEGどちらをswingすべきかも決める
  GaitParam::FootStepNodes calcDefaultNextStep(const GaitParam::FootStepNodes& footstepNodes, const std::vector<cnoid::Vector3>& defaultTranslatePos, const cnoid::Position& offset = cnoid::Position::Identity()) const;
  // footstepNodesの次の一歩を作る. 両脚が地面についた状態で終わる
  GaitParam::FootStepNodes calcDefaultNextStep(const int& swingLeg, const GaitParam::FootStepNodes& footstepNodes, const std::vector<cnoid::Vector3>& defaultTranslatePos, const cnoid::Position& offset = cnoid::Position::Identity()) const;

  // footstepNodesListの終了時の状態が両脚支持でかつその期間の時間がdefaultDoubleSupportTimeよりも短いなら延長する
  void extendDoubleSupportTime(GaitParam::FootStepNodes& footstepNodes) const;
};

#endif
