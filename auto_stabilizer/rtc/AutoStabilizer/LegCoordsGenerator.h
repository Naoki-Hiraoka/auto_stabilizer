#ifndef LEGCOORDSGENERATOR_H
#define LEGCOORDSGENERATOR_H

#include "GaitParam.h"

class LegCoordsGenerator{
public:
  // LegCoordsGeneratorでしか使わないパラメータ

  double delayTimeOffset = 0.2; // 0以上. 単位[s]. swing期は、(remainTime - delayTimeOffset)後にdstCoordsに到達するようなrectangle軌道を生成し、その軌道にdelayTimeOffset遅れで滑らかに追従するような軌道を生成する
  double touchVel = 0.3; // 0より大きい. 単位[m/s]. この速さで足を下ろした場合に着地までに要する時間をremainTimeが下回るまで、足下げを始めない. 0.5だと実機では少し速すぎるか?
  double finalDistanceWeight = 1.0; // 0より大きい. swing期のDOWN_PHASEの速さを、他のPHASEとくらべ何倍遅くするか. 大きくすると足をまっすぐ下ろせる利点がある. 大きくするとswingの速度が上がるため、遊脚の力センサにノイズが乗って着地誤検知しやすいという欠点がある.
  double goalOffset = -0.02; // [m]. 遊脚軌道生成時に、次に着地する場合、generate frameで鉛直方向に, 目標着地位置に対して加えるオフセット. 0以下. 歩行中は急激に変更されない. 遅づきに対応するためのもの. 位置制御だと着地の衝撃が大きいのでデフォルトを0にしているが、トルク制御時や、高低差がある地形や、衝撃を気にする必要がないシミュレーションでは-0.05等にした方がよい.

  int previewStepNum = 4; // 2以上. ZMPと重心軌道を生成する際に予見するfootStepNodesListのサイズ. 着地位置修正アルゴリズムが1 step capturabilityに基づくものであるため、previewStepNum = 2にして今の一歩だけを予見して重心軌道を生成した方がいいように思えるが、実際には4以上でないと目標軌道に追従するために必要なZmp入力が大きく、refZmpTrajの値を大きく外れてしまう.
  double footGuidedBalanceTime = 0.6; // [s]. refZmpTrajの終端時間. 0より大きい. (1.0[s]だと大きすぎて, 両足で立っているときに傾いたままなかなか戻ってこなかったり、停止時に重心がなかなか中央に戻らずemergency stepが無限誘発したり、少しずつ傾いていくことがある). (0.4だと静止時に衝撃が加わると上下方向に左右交互に振動することがある)
public:
  void initLegCoords(const GaitParam& gaitParam,
                     std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> >& o_refZmpTraj, std::vector<cpp_filters::TwoPointInterpolatorSE3>& o_genCoords) const;

  void calcLegCoords(const GaitParam& gaitParam, double dt, bool useActStates,
                     std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> >& o_refZmpTraj, std::vector<cpp_filters::TwoPointInterpolatorSE3>& o_genCoords, std::vector<GaitParam::SwingState_enum>& o_swingState) const;

  void calcCOMCoords(const GaitParam& gaitParam, double dt,
                     cnoid::Vector3& o_genNextCog, cnoid::Vector3& o_genNextCogVel) const;

};

#endif
