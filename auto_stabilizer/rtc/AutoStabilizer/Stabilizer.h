#ifndef Stabilizer_H
#define Stabilizer_H

#include "GaitParam.h"
#include "EndEffectorParam.h"

class Stabilizer{
public:
  // Stabilizerでしか使わないパラメータ
  std::vector<double> bodyAttitudeControlGain=std::vector<double>{0.5, 0.5}; // 要素数2. gaitParam.footMidCoords座標系X軸, Y軸. 単位は[/s]
  std::vector<double> bodyAttitudeControlTimeConst=std::vector<double>{1000, 1000}; // 要素数2. gaitParam.footMidCoords座標系X軸, Y軸. 単位は[s]
  std::vector<double> rootRotCompensationLimit=std::vector<double>{1.0,1.0}; //要素数2. gaitParam.footMidCoords座標系X軸, Y軸. 単位は[rad]

protected:
  // 内部で変更されるステータス. reset()時にresetされる

public:
  // start Stabilizer時に呼ばれる
  void reset(){
  }

  // 内部の補間器を進める
  void update(double dt){
  }


  bool execStabilizer(const cnoid::BodyPtr refRobotOrigin, const cnoid::BodyPtr actRobotOrigin, const cnoid::BodyPtr genRobot, const GaitParam& gaitParam, const EndEffectorParam& endEffectorParam, double dt, double g, double mass,
                      cnoid::BodyPtr& genRobotTqc, cnoid::Vector3& o_dRootRpy) const;

protected:
  bool moveBasePosRotForBodyRPYControl(const cnoid::BodyPtr refRobotOrigin, const cnoid::BodyPtr actRobotOrigin, double dt, const GaitParam& gaitParam,
                                       cnoid::Vector3& o_dRootRp) const;
  bool calcZMP(const GaitParam& gaitParam, double dt, double g, double mass,
               cnoid::Vector3& o_tgtZmp/*generate座標系*/, cnoid::Vector3& o_tgtForce/*generate座標系*/) const;
  bool calcWrench(const GaitParam& gaitParam, const EndEffectorParam& endEffectorParam, const cnoid::Vector3& tgtZmp, const cnoid::Vector3& tgtForce,
                  std::vector<cnoid::Vector6>& o_tgtWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/) const;

  bool calcTorque(const cnoid::BodyPtr actRobotOrigin, double dt, const EndEffectorParam& endEffectorParam, const std::vector<cnoid::Vector6>& tgtWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/,
                  cnoid::BodyPtr& genRobotTqc) const;


};

#endif
