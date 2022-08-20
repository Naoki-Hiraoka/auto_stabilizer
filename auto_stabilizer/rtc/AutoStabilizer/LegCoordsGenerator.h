#ifndef LEGCOORDSGENERATOR_H
#define LEGCOORDSGENERATOR_H

#include "GaitParam.h"

class LegCoordsGenerator{
public:
  // LegCoordsGeneratorでしか使わないパラメータ

  double delayTimeOffset = 0.2; // 0以上. swing期は、remainTime - supportTime - delayTimeOffset後にdstCoordsに到達するようなrectangle軌道を生成し、その軌道にdelayTimeOffset遅れで滑らかに追従するような軌道を生成する
  double touchVel = 0.5; // 0より大きい. 足を下ろすときの速さ(この値より速く下ろす) [m/s]
  double finalDistanceWeight = 3.0; // 0より大きい. swing期のDOWN_PHASEの速さを、他のPHASEとくらべ何倍遅くするか. 着地時の衝撃をやわらげる
  double goalOffset = -0.05; // [m]. 遊脚軌道生成時に、次に着地する場合、generate frameで鉛直方向に, 目標着地位置に対して加えるオフセット. 0以下. 歩行中は急激に変更されない

  int previewStepNum = 2; // 2以上. 着地位置修正アルゴリズムが1 step capturabilityに基づくものであるため、今の一歩より先のstepを予見して重心軌道を生成してしまうと破綻する. そのため、2がよい.
  double footGuidedBalanceTime = 0.4; // [s]. refZmpTrajの終端時間. 0より大きい. (1.0[s]だと大きすぎて, 両足で立っているときに傾いたまま戻ってこなかったり、少しずつ傾いていくことがある)
public:
  void initLegCoords(const GaitParam& gaitParam,
                     std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> >& o_refZmpTraj, std::vector<cpp_filters::TwoPointInterpolatorSE3>& o_genCoords) const;

  void calcLegCoords(const GaitParam& gaitParam, double dt, bool useActStates,
                     std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> >& o_refZmpTraj, std::vector<cpp_filters::TwoPointInterpolatorSE3>& o_genCoords, std::vector<GaitParam::SwingState_enum>& o_swingState) const;

  void calcCOMCoords(const GaitParam& gaitParam, double dt,
                     cnoid::Vector3& o_genNextCog, cnoid::Vector3& o_genNextCogVel) const;

};

#endif
