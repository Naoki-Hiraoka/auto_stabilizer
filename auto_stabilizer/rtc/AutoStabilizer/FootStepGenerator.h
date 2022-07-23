#ifndef FOOTSTEPGENERATOR_H
#define FOOTSTEPGENERATOR_H

#include "GaitParam.h"


class FootStepGenerator{
public:
  // FootStepGeneratorでしか使わないパラメータ
  double defaultStepTime = 0.8; // [s]
  double defaultDoubleSupportTime = 0.12; // [s]. defaultStepTime未満である必要がある.
  double defaultStepHeight = 0.07; // [s].
  std::vector<cnoid::Vector3> defaultTranslatePos = std::vector<cnoid::Vector3>(2,cnoid::Vector3::Zero()); // goPos, goVelocity, その場足踏みをするときの右脚と左脚の中心からの相対位置(Z軸は鉛直)


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
  bool setFootSteps(const GaitParam& gaitParam, const std::vector<StepNode>& footsteps, std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const;

  /*
    footstepNodesList[1]開始時のsupport/swingの状態を上書きによって変更する場合は、footstepNodesList[0]の終了時の状態が両脚支持でかつその期間の時間がdefaultDoubleSupportTimeよりも短いなら延長する

    外部からgoStopをされると、footstepNodesList[N]以降の要素は削除され、代わりに両脚が横に並ぶ位置に一歩歩くnodeが末尾に入る
    跳躍中でなければ、
    emergencyStepModeなら、footstepNodesList[0]が末尾の要素でかつ現在のdstCoordsのままだとバランスが取れないなら、footstepNodesList[1]に両脚が横に並ぶ位置に一歩歩くnodeが末尾に入る. (modifyFootStepsと併用せよ)
    modifyFootStepsなら、footstepNodesList[0]の現在のdstCoordsのままだとバランスが取れないか、今swing期でfootstepNodesList[0]終了時に着地する予定の要素のdstCoordsが着地可能領域上にないなら、footstepNodesList[0]の、今swing期でfootstepNodesList[0]終了時に着地する予定の要素を修正する. また、それ以降一回でもswingする要素の位置を平行移動する
    goVelocityModeなら、進行方向に向けてfootStepNodesList[1] ~ footStepNodesList[5]の要素をfootstepNodesList[0]から機械的に計算してどんどん位置修正&末尾appendしていく.

  */
  bool calcFootSteps(const GaitParam& gaitParam, const double& dt, std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const;

};

#endif
