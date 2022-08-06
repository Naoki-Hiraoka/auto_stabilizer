#ifndef LEGCOORDSGENERATOR_H
#define LEGCOORDSGENERATOR_H

#include "GaitParam.h"

class LegCoordsGenerator{
public:
  // LegCoordsGeneratorでしか使わないパラメータ

  double delayTimeOffset = 0.2; // 0以上. swing期は、remainTime - supportTime - delayTimeOffset後にdstCoordsに到達するようなrectangle軌道を生成し、その軌道にdelayTimeOffset遅れで滑らかに追従するような軌道を生成する
  double touchVel = 0.3; // 0より大きい. 足を下ろすときの速さ(この値より速く下ろす) [m/s]
  //cnoid::Vector3 goal_off; // TODO


public:
  void initLegCoords(const GaitParam& gaitParam,
                     std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> >& o_refZmpTraj, std::vector<cpp_filters::TwoPointInterpolatorSE3>& o_genCoords) const;

  void calcLegCoords(const GaitParam& gaitParam, double dt,
                     std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> >& o_refZmpTraj, std::vector<cpp_filters::TwoPointInterpolatorSE3>& o_genCoords, cpp_filters::TwoPointInterpolatorSE3& o_footMidCoords, std::vector<GaitParam::FootStepNodes::SwingState_enum>& o_swingState) const;
  // swingLegGainControl TODO

  void calcCOMCoords(const GaitParam& gaitParam, double dt, double mass,
                     cnoid::Vector3& o_genNextCog, cnoid::Vector3& o_genNextCogVel) const;

};

#endif
