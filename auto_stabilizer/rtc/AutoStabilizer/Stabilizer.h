#ifndef Stabilizer_H
#define Stabilizer_H

#include "GaitParam.h"
#include <prioritized_qp/PrioritizedQPSolver.h>

class Stabilizer{
public:
  // Stabilizerでしか使わないパラメータ
  std::vector<double> bodyAttitudeControlGain=std::vector<double>{0.5, 0.5}; // 要素数2. gaitParam.footMidCoords座標系X軸, Y軸. 単位は[/s]. 0以上
  std::vector<double> bodyAttitudeControlTimeConst=std::vector<double>{1000, 1000}; // 要素数2. gaitParam.footMidCoords座標系X軸, Y軸. 単位は[s]. 0より大きい
  std::vector<double> bodyAttitudeControlCompensationLimit=std::vector<double>{0.7,0.7}; //要素数2. gaitParam.footMidCoords座標系X軸, Y軸. 単位は[rad]. STが動いている間は変更されない. 0以上

  std::vector<cnoid::Vector6> dampingCompensationLimit = std::vector<cnoid::Vector6>(NUM_LEGS); // 要素数2. [rleg, lleg]. generate frame. endEffector origin. STが動いている間は変更されない. 0以上
  std::vector<cnoid::Vector6> dampingGain = std::vector<cnoid::Vector6>(NUM_LEGS); // 要素数2. [rleg. lleg].  EndEffector frame(offset+abcTargetPose). endEffector origin. 0より大きい
  std::vector<cnoid::Vector6> dampingTimeConst = std::vector<cnoid::Vector6>(NUM_LEGS); // 要素数2. [rleg. lleg]. EndEffector frame(offset+abcTargetPose). endEffector origin. 0より大きい

  std::vector<cnoid::Vector6> springCompensationLimit = std::vector<cnoid::Vector6>(NUM_LEGS); // 要素数2. [rleg, lleg]. generate frame. endEffector origin. STが動いている間は変更されない. 0以上
  std::vector<cnoid::Vector6> springCompensationVelocityLimit = std::vector<cnoid::Vector6>(NUM_LEGS); // 要素数2. [rleg, lleg]. generate frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> springGain = std::vector<cnoid::Vector6>(NUM_LEGS); // 要素数2. [rleg. lleg].  EndEffector frame(offset+abcTargetPose). endEffector origin. 0より大きい
  std::vector<cnoid::Vector6> springTimeConst = std::vector<cnoid::Vector6>(NUM_LEGS); // 要素数2. [rleg. lleg]. EndEffector frame(offset+abcTargetPose). endEffector origin. 単位は[s]. 0より大きい

  Stabilizer(){
    for(int i=0;i<NUM_LEGS;i++){
      dampingCompensationLimit[i] << 0.08, 0.08, 0.08, 0.523599, 0.523599, 0.523599; // 0.523599rad = 30deg
      dampingGain[i] << 33600, 33600, 7839, 60, 60, 1e5;
      dampingTimeConst[i] << 3.0/1.1, 3.0/1.1, 1.5/1.1, 1.5/1.1, 1.5/1.1, 1.5/1.1;

      springCompensationLimit[i] << 0.05, 0.05, 0.05, 0.523599, 0.523599, 0.523599; // 0.523599rad = 30deg
      springCompensationVelocityLimit[i] << 0.05, 0.05, 0.05, 0.349066, 0.349066, 0.349066; // 0.349066rad/2 = 20deg/s
      springGain[i] << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0;
      springTimeConst[i] << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    }
  }
protected:
  // 計算高速化のためのキャッシュ. クリアしなくても別に副作用はない.
  mutable std::shared_ptr<prioritized_qp::Task> constraintTask_ = std::make_shared<prioritized_qp::Task>();
  mutable std::shared_ptr<prioritized_qp::Task> tgtZmpTask_ = std::make_shared<prioritized_qp::Task>();;
  mutable std::shared_ptr<prioritized_qp::Task> copTask_ = std::make_shared<prioritized_qp::Task>();;
public:
  void initStabilizerOutput(const GaitParam& gaitParam,
                            cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy, std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffsetDampingControl /*generate frame, endeffector origin*/, std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffsetSwingEEModification /*generate frame, endeffector origin*/, cnoid::Vector3& o_stTargetZmp) const;

  bool execStabilizer(const cnoid::BodyPtr refRobot, const cnoid::BodyPtr actRobot, const cnoid::BodyPtr genRobot, const GaitParam& gaitParam, double dt, double mass,
                      cnoid::BodyPtr& actRobotTqc, cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy, std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffsetDampingControl /*generate frame, endeffector origin*/, std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffsetSwingEEModification /*generate frame, endeffector origin*/, cnoid::Vector3& o_stTargetZmp) const;

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
                          std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffsetDampingControl /*generate frame, endeffector origin*/) const;
  bool calcSwingEEModification(double dt, const GaitParam& gaitParam,
                               std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffsetSwingEEModification /*generate frame, endeffector origin*/) const;
};

#endif
