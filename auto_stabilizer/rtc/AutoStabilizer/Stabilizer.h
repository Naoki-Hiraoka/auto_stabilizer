#ifndef Stabilizer_H
#define Stabilizer_H

#include "GaitParam.h"
#include "EndEffectorParam.h"

class Stabilizer{
public:
  // Stabilizerでしか使わないパラメータ


public:
  // start Stabilizer時に呼ばれる
  void reset();

  // 内部の補間器を進める
  void update(double dt);


  bool execStabilizer(const GaitParam& gaitParam, double dt, double g, double mass,
                      cnoid::Vector3& o_genNextCog, cnoid::Vector3& o_genNextCogVel) const;

};

#endif
