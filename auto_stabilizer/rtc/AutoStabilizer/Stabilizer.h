#ifndef Stabilizer_H
#define Stabilizer_H

#include "GaitParam.h"
#include <prioritized_qp/PrioritizedQPSolver.h>

class Stabilizer{
public:
  // Stabilizerでしか使わないパラメータ
  std::vector<double> bodyAttitudeControlGain=std::vector<double>{0.5, 0.5}; // 要素数2. gaitParam.footMidCoords座標系X軸, Y軸. 単位は[/s]
  std::vector<double> bodyAttitudeControlTimeConst=std::vector<double>{1000, 1000}; // 要素数2. gaitParam.footMidCoords座標系X軸, Y軸. 単位は[s]
  std::vector<double> rootRotCompensationLimit=std::vector<double>{0.7,0.7}; //要素数2. gaitParam.footMidCoords座標系X軸, Y軸. 単位は[rad]

  std::vector<cnoid::Vector6> dampingCompensationLimit = std::vector<cnoid::Vector6>(NUM_LEGS); // 要素数2. [rleg, lleg]. generate frame. endEffector origin
  std::vector<cnoid::Vector6> dampingGain = std::vector<cnoid::Vector6>(NUM_LEGS); // 要素数2. [rleg. lleg].  EndEffector frame(offset+abcTargetPose). endEffector origin
  std::vector<cnoid::Vector6> dampingTimeConst = std::vector<cnoid::Vector6>(NUM_LEGS); // 要素数2. [rleg. lleg]. EndEffector frame(offset+abcTargetPose). endEffector origin
  std::vector<cnoid::Vector6> dampingWrenchErrorLimit = std::vector<cnoid::Vector6>(NUM_LEGS); // 要素数2. [rleg. lleg]. EndEffector frame(offset+abcTargetPose). endEffector origin

  Stabilizer(){
    for(int i=0;i<NUM_LEGS;i++){
      dampingCompensationLimit[i] << 0.08, 0.08, 0.08, 0.05, 0.05, 0.05;
      dampingGain[i] << 33600, 33600, 7839, 60, 60, 1e5;
      dampingTimeConst[i] << 3.0/1.1, 3.0/1.1, 1.5/1.1, 1.5/1.1, 1.5/1.1, 1.5/1.1;
      dampingWrenchErrorLimit[i] << 200, 200, 200, 15, 15, 15;
    }
  }
protected:
  // 計算高速化のためのキャッシュ. クリアしなくても別に副作用はない.
  mutable std::shared_ptr<prioritized_qp::Task> constraintTask_ = std::make_shared<prioritized_qp::Task>();
  mutable std::shared_ptr<prioritized_qp::Task> tgtZmpTask_ = std::make_shared<prioritized_qp::Task>();;
  mutable std::shared_ptr<prioritized_qp::Task> copTask_ = std::make_shared<prioritized_qp::Task>();;
public:
  void initStabilizerOutput(const GaitParam& gaitParam,
                            cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy, std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffset /*generate frame, endeffector origin*/) const;

  bool execStabilizer(const cnoid::BodyPtr refRobot, const cnoid::BodyPtr actRobot, const cnoid::BodyPtr genRobot, const GaitParam& gaitParam, double dt, double mass,
                      cnoid::BodyPtr& actRobotTqc, cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy, std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffset /*generate frame, endeffector origin*/) const;

protected:
  bool moveBasePosRotForBodyRPYControl(const cnoid::BodyPtr refRobot, const cnoid::BodyPtr actRobot, double dt, const GaitParam& gaitParam,
                                       cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy) const;
  bool calcZMP(const GaitParam& gaitParam, double dt, double mass,
               cnoid::Vector3& o_tgtZmp/*generate座標系*/, cnoid::Vector3& o_tgtForce/*generate座標系*/) const;
  bool calcWrench(const GaitParam& gaitParam, const cnoid::Vector3& tgtZmp/*generate座標系*/, const cnoid::Vector3& tgtForce/*generate座標系. ロボットが受ける力*/,
                  std::vector<cnoid::Vector6>& o_tgtEEWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/) const;
  bool calcTorque(const cnoid::BodyPtr actRobot, double dt, const GaitParam& gaitParam, const std::vector<cnoid::Vector6>& tgtEEWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/,
                  cnoid::BodyPtr& actRobotTqc) const;
  bool calcDampingControl(double dt, const GaitParam& gaitParam, const std::vector<cnoid::Vector6>& tgtEEWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/,
                          std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffset /*generate frame, endeffector origin*/) const;
};

#endif
