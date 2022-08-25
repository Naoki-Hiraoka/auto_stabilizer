#ifndef Stabilizer_H
#define Stabilizer_H

#include "GaitParam.h"
#include <prioritized_qp/PrioritizedQPSolver.h>
#include <cnoid/JointPath>

class Stabilizer{
public:
  // Stabilizerでしか使わないパラメータ
  std::vector<double> bodyAttitudeControlGain=std::vector<double>{0.5, 0.5}; // 要素数2. gaitParam.footMidCoords座標系X軸, Y軸. 単位は[/s]. 0以上
  std::vector<double> bodyAttitudeControlTimeConst=std::vector<double>{1000, 1000}; // 要素数2. gaitParam.footMidCoords座標系X軸, Y軸. 単位は[s]. 0より大きい
  std::vector<double> bodyAttitudeControlCompensationLimit=std::vector<double>{0.7,0.7}; //要素数2. gaitParam.footMidCoords座標系X軸, Y軸. 単位は[rad]. STが動いている間は変更されない. 0以上

  std::vector<cnoid::Vector6> dampingCompensationLimit = std::vector<cnoid::Vector6>(NUM_LEGS); // 要素数2. [rleg, lleg]. generate frame. endEffector origin. STが動いている間は変更されない. 0以上
  std::vector<cnoid::Vector6> dampingGain = std::vector<cnoid::Vector6>(NUM_LEGS); // 要素数2. [rleg. lleg].  EndEffector frame(offset+abcTargetPose). endEffector origin. 0より大きい
  std::vector<cnoid::Vector6> dampingTimeConst = std::vector<cnoid::Vector6>(NUM_LEGS); // 要素数2. [rleg. lleg]. EndEffector frame(offset+abcTargetPose). endEffector origin. 0より大きい

  bool isTorqueControlMode = false; // falseなら位置制御のみ. この値はisSTRunning時には変更されない. trueにするなら事前にself.rh_svc.setServoTorqueGainPercentage("all",100)を呼んでおくこと.
  std::vector<std::vector<double> > supportPgain = std::vector<std::vector<double> >(2); // 要素数2. [rleg, lleg]. rootLinkから各endeffectorまでの各関節のゲイン. 0~100
  std::vector<std::vector<double> > supportDgain = std::vector<std::vector<double> >(2); // 要素数2. [rleg, lleg]. rootLinkから各endeffectorまでの各関節のゲイン. 0~100
  std::vector<std::vector<double> > landingPgain = std::vector<std::vector<double> >(2); // 要素数2. [rleg, lleg]. rootLinkから各endeffectorまでの各関節のゲイン. 0~100
  std::vector<std::vector<double> > landingDgain = std::vector<std::vector<double> >(2); // 要素数2. [rleg, lleg]. rootLinkから各endeffectorまでの各関節のゲイン. 0~100
  std::vector<std::vector<double> > swingPgain = std::vector<std::vector<double> >(2); // 要素数2. [rleg, lleg]. rootLinkから各endeffectorまでの各関節のゲイン. 0~100
  std::vector<std::vector<double> > swingDgain = std::vector<std::vector<double> >(2); // 要素数2. [rleg, lleg]. rootLinkから各endeffectorまでの各関節のゲイン. 0~100
  double swing2LandingTransitionTime = 0.05; // [s]. 0より大きい
  double landing2SupportTransitionTime = 0.1; // [s]. 0より大きい
  double support2SwingTransitionTime = 0.1; // [s]. 0より大きい
  Stabilizer(){
    for(int i=0;i<NUM_LEGS;i++){
      dampingCompensationLimit[i] << 0.08, 0.08, 0.08, 0.523599, 0.523599, 0.523599; // 0.523599rad = 30deg
      dampingGain[i] << 33600, 33600, 9000, 60, 60, 1e5;
      dampingTimeConst[i] << 3.0/1.1, 3.0/1.1, 1.5/1.1, 1.5/1.1, 1.5/1.1, 1.5/1.1;
    }
  }
  void init(const GaitParam& gaitParam, cnoid::BodyPtr& actRobotTqc){
    for(int i=0;i<NUM_LEGS;i++){
      cnoid::JointPath jointPath(actRobotTqc->rootLink(), actRobotTqc->link(gaitParam.eeParentLink[i]));
      if(jointPath.numJoints() == 6){
        supportPgain[i] = {5,10,10,5,0.1,0.1};
        supportDgain[i] = {10,20,20,10,10,10};
        landingPgain[i] = {5,1,1,1,0.1,0.1};
        landingDgain[i] = {10,10,10,10,5,5};
        swingPgain[i] = {5,30,20,10,5,5};
        swingDgain[i] = {10,30,20,20,10,10};
      }else{
        supportPgain[i].resize(jointPath.numJoints(), 100.0);
        supportDgain[i].resize(jointPath.numJoints(), 100.0);
        landingPgain[i].resize(jointPath.numJoints(), 100.0);
        landingDgain[i].resize(jointPath.numJoints(), 100.0);
        swingPgain[i].resize(jointPath.numJoints(), 100.0);
        swingDgain[i].resize(jointPath.numJoints(), 100.0);
      }
    }
  }
protected:
  // 計算高速化のためのキャッシュ. クリアしなくても別に副作用はない.
  mutable std::shared_ptr<prioritized_qp::Task> constraintTask_ = std::make_shared<prioritized_qp::Task>();
  mutable std::shared_ptr<prioritized_qp::Task> tgtZmpTask_ = std::make_shared<prioritized_qp::Task>();;
  mutable std::shared_ptr<prioritized_qp::Task> copTask_ = std::make_shared<prioritized_qp::Task>();;
public:
  void initStabilizerOutput(const GaitParam& gaitParam,
                            cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy, std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffset /*generate frame, endeffector origin*/, cnoid::Vector3& o_stTargetZmp, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPGainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDGainPercentage) const;

  bool execStabilizer(const GaitParam& gaitParam, double dt, bool useActState,
                      cnoid::BodyPtr& actRobotTqc, cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy, cnoid::Position& o_stTargetRootPose, std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffset /*generate frame, endeffector origin*/, std::vector<cnoid::Position>& o_stEETargetPose, cnoid::Vector3& o_stTargetZmp, std::vector<cnoid::Vector6>& o_stEETargetWrench, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPgainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDgainPercentage) const;

protected:
  bool moveBasePosRotForBodyRPYControl(double dt, const GaitParam& gaitParam, bool useActState,
                                       cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy, cnoid::Position& o_stTargetRootPose) const;
  bool calcZMP(const GaitParam& gaitParam, double dt, bool useActState,
               cnoid::Vector3& o_tgtZmp/*generate座標系*/, cnoid::Vector3& o_tgtForce/*generate座標系*/) const;
  bool calcWrench(const GaitParam& gaitParam, const cnoid::Vector3& tgtZmp/*generate座標系*/, const cnoid::Vector3& tgtForce/*generate座標系. ロボットが受ける力*/, bool useActState,
                  std::vector<cnoid::Vector6>& o_tgtEEWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/) const;
  bool calcTorque(double dt, const GaitParam& gaitParam, const std::vector<cnoid::Vector6>& tgtEEWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/,
                  cnoid::BodyPtr& actRobotTqc, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPGainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDGainPercentage) const;
  bool calcDampingControl(double dt, const GaitParam& gaitParam, const std::vector<cnoid::Vector6>& tgtEEWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/, bool useActState,
                          std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffset /*generate frame, endeffector origin*/, std::vector<cnoid::Position>& o_stEETargetPose) const;
};

#endif
