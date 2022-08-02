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

  std::vector<cnoid::Vector6> dampingCompensationLimit; // 要素数EndEffectors. generate frame. endEffector origin
  std::vector<cnoid::Vector6> dampingGain; // 要素数2. [rleg. lleg].  EndEffector frame(actual). endEffector origin
  std::vector<cnoid::Vector6> dampingTimeConst; // 要素数2. [rleg. lleg]. EndEffector frame(actual). endEffector origin
  std::vector<cnoid::Vector6> dampingWrenchErrorLimit; // 要素数2. [rleg. lleg]. EndEffector frame(actual). endEffector origin

protected:
  // 内部で変更されるステータス. reset()時にresetされる
  std::vector<cnoid::Vector6> offsetPrev; // 要素数EndEffectors. generate frame. endEffector origin
  std::vector<cnoid::Vector6> doffsetPrev; // 要素数EndEffectors. generate frame. endEffector origin
public:
  void push_back(){
    cnoid::Vector6 defaultDampingCompensationLimit; defaultDampingCompensationLimit << 0.08, 0.08, 0.08, 0.05, 0.05, 0.05;
    this->dampingCompensationLimit.push_back(defaultDampingCompensationLimit);
    cnoid::Vector6 defaultDampingGain; defaultDampingGain << 33600, 33600, 7839, 60, 60, 1e5;
    this->dampingGain.push_back(defaultDampingGain);
    cnoid::Vector6 defaultDampingTimeConst; defaultDampingTimeConst << 3.0/1.1, 3.0/1.1/ 1.5/1.1, 1.5/1.1, 1.5/1.1, 1.5/1.1;
    this->dampingTimeConst.push_back(defaultDampingTimeConst);
    cnoid::Vector6 defaultDampingWrenchErrorLimit; defaultDampingWrenchErrorLimit << 200, 200, 200, 15, 15, 15;
    this->dampingWrenchErrorLimit.push_back(defaultDampingWrenchErrorLimit);

    this->offsetPrev.push_back(cnoid::Vector6::Zero());
    this->doffsetPrev.push_back(cnoid::Vector6::Zero());

  }

  // start Stabilizer時に呼ばれる
  void reset(){
    for(int i=0;i<this->offsetPrev.size();i++) this->offsetPrev[i].setZero();
    for(int i=0;i<this->doffsetPrev.size();i++) this->doffsetPrev[i].setZero();
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
  bool calcDampingControl(double dt, const EndEffectorParam& endEffectorParam, const std::vector<cnoid::Vector6>& tgtWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/,
                          std::vector<cnoid::Position>& o_tgtPose /*generate frame*/) const;
};

#endif
