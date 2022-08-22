#ifndef LEGCOORDSGENERATOR_H
#define LEGCOORDSGENERATOR_H

#include "GaitParam.h"

class LegCoordsGenerator{
public:
  // LegCoordsGeneratorでしか使わないパラメータ

  double delayTimeOffset = 0.2; // 0以上. swing期は、remainTime - supportTime - delayTimeOffset後にdstCoordsに到達するようなrectangle軌道を生成し、その軌道にdelayTimeOffset遅れで滑らかに追従するような軌道を生成する
  double touchVel = 0.5; // 0より大きい. 足を下ろすときの速さ(この値より速く下ろす) [m/s]
  double finalDistanceWeight = 1.0; // 0より大きい. swing期のDOWN_PHASEの速さを、他のPHASEとくらべ何倍遅くするか. 大きくすると着地が遅くなって着地衝撃をやわらげる? 大きくするとswingの速度が上がるため、遊脚の力センサにノイズが乗って着地誤検知しやすい
  double goalOffset = -0.05; // [m]. 遊脚軌道生成時に、次に着地する場合、generate frameで鉛直方向に, 目標着地位置に対して加えるオフセット. 0以下. 歩行中は急激に変更されない

  int previewStepNum = 4; // 2以上. 着地位置修正アルゴリズムが1 step capturabilityに基づくものであるため、previewStepNum = 2にして今の一歩だけを予見して重心軌道を生成した方がいいように思えるが、実際には4以上でないと目標軌道に追従するために必要なZmp入力が大きく、refZmpTrajの値を大きく外れてしまう.
  double footGuidedBalanceTime = 0.4; // [s]. refZmpTrajの終端時間. 0より大きい. (1.0[s]だと大きすぎて, 両足で立っているときに傾いたままなかなか戻ってこなかったり、少しずつ傾いていくことがある)
public:
  void initLegCoords(const GaitParam& gaitParam,
                     std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> >& o_refZmpTraj, std::vector<cpp_filters::TwoPointInterpolatorSE3>& o_genCoords) const;

  void calcLegCoords(const GaitParam& gaitParam, double dt, bool useActStates,
                     std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> >& o_refZmpTraj, std::vector<cpp_filters::TwoPointInterpolatorSE3>& o_genCoords, std::vector<GaitParam::SwingState_enum>& o_swingState) const;

  void calcCOMCoords(const GaitParam& gaitParam, double dt,
                     cnoid::Vector3& o_genNextCog, cnoid::Vector3& o_genNextCogVel) const;

};

#endif
