#ifndef Stabilizer_H
#define Stabilizer_H

#include "GaitParam.h"
#include <prioritized_qp_osqp/prioritized_qp_osqp.h>
#include <cnoid/JointPath>

#include <aik_constraint/PositionConstraint.h>
#include <aik_constraint/COMConstraint.h>
#include <aik_constraint/JointAngleConstraint.h>
#include <aik_constraint/AngularMomentumConstraint.h>
#include <aik_constraint_joint_limit_table/JointLimitMinMaxTableConstraint.h>
#include <aik_constraint/ClientCollisionConstraint.h>
#include <prioritized_acc_inverse_kinematics_solver/PrioritizedAccInverseKinematicsSolver.h>

class Stabilizer{
public:
  // Stabilizerでしか使わないパラメータ
  std::vector<double> bodyAttitudeControlGain=std::vector<double>{0.5, 0.5}; // 要素数2. gaitParam.footMidCoords座標系X軸, Y軸. 単位は[/s]. 0以上
  std::vector<double> bodyAttitudeControlTimeConst=std::vector<double>{1000, 1000}; // 要素数2. gaitParam.footMidCoords座標系X軸, Y軸. 単位は[s]. 0より大きい
  std::vector<double> bodyAttitudeControlCompensationLimit=std::vector<double>{0.7,0.7}; //要素数2. gaitParam.footMidCoords座標系X軸, Y軸. 単位は[rad]. STが動いている間は変更されない. 0以上

  std::vector<std::vector<double> > supportPgain = std::vector<std::vector<double> >(2); // 要素数2. [rleg, lleg]. rootLinkから各endeffectorまでの各関節のゲイン. 0~100
  std::vector<std::vector<double> > supportDgain = std::vector<std::vector<double> >(2); // 要素数2. [rleg, lleg]. rootLinkから各endeffectorまでの各関節のゲイン. 0~100
  std::vector<std::vector<double> > landingPgain = std::vector<std::vector<double> >(2); // 要素数2. [rleg, lleg]. rootLinkから各endeffectorまでの各関節のゲイン. 0~100
  std::vector<std::vector<double> > landingDgain = std::vector<std::vector<double> >(2); // 要素数2. [rleg, lleg]. rootLinkから各endeffectorまでの各関節のゲイン. 0~100
  std::vector<std::vector<double> > swingPgain = std::vector<std::vector<double> >(2); // 要素数2. [rleg, lleg]. rootLinkから各endeffectorまでの各関節のゲイン. 0~100
  std::vector<std::vector<double> > swingDgain = std::vector<std::vector<double> >(2); // 要素数2. [rleg, lleg]. rootLinkから各endeffectorまでの各関節のゲイン. 0~100
  double swing2LandingTransitionTime = 0.05; // [s]. 0より大きい
  double landing2SupportTransitionTime = 0.1; // [s]. 0より大きい
  double support2SwingTransitionTime = 0.2; // [s]. 0より大きい

  std::vector<cnoid::Vector6> ee_K; // 要素数EndEffectors. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_D; // 要素数EndEffectors. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_swing_K; // 要素数NUM_LEGS. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_swing_D; // 要素数NUM_LEGS. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_landing_K; // 要素数NUM_LEGS. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_landing_D; // 要素数NUM_LEGS. EndEffector frame. endEffector origin. 0以上
  cnoid::Vector3 root_K = cnoid::Vector3(100,100,100); // rootlink frame
  cnoid::Vector3 root_D = cnoid::Vector3(40,40,40); // rootlink frame

  std::vector<cpp_filters::TwoPointInterpolator<double> > dqWeight; // 要素数と順序はrobot->numJoints()と同じ. 0より大きい. 各関節の速度に対するダンピング項の比. default 1. 動かしたくない関節は大きくする. 全く動かしたくないなら、controllable_jointsを使うこと

  void init(const GaitParam& gaitParam, cnoid::BodyPtr& actRobotTqc){
    for(int i=0;i<NUM_LEGS;i++){
      cnoid::JointPath jointPath(actRobotTqc->rootLink(), actRobotTqc->link(gaitParam.eeParentLink[i]));
      if(jointPath.numJoints() == 6){
        // supportPgain[i] = {5,10,10,5,0.1,0.1};
        // supportDgain[i] = {10,20,20,10,10,10};
        // landingPgain[i] = {5,10,10,5,0.1,0.1};
        // landingDgain[i] = {10,20,20,10,10,10};
        // swingPgain[i] = {5,10,10,5,0.1,0.1};
        // swingDgain[i] = {10,20,20,10,10,10};
        // 下はもとのauto_stabilizerの値. ゲインが低すぎて、go-velocity 0 0 0のときに前に進んでいってしまう
        supportPgain[i] = {5,15,10,5,0.2,0.2};
        supportDgain[i] = {20,40,40,20,5,5};
        landingPgain[i] = {5,15,1,1,0.2,0.2};
        landingDgain[i] = {20,20,20,20,5,5};
        swingPgain[i] = {5,30,20,10,5,5};
        swingDgain[i] = {20,60,40,40,60,60};
      }else{
        supportPgain[i].resize(jointPath.numJoints(), 100.0);
        supportDgain[i].resize(jointPath.numJoints(), 100.0);
        landingPgain[i].resize(jointPath.numJoints(), 100.0);
        landingDgain[i].resize(jointPath.numJoints(), 100.0);
        swingPgain[i].resize(jointPath.numJoints(), 100.0);
        swingDgain[i].resize(jointPath.numJoints(), 100.0);
      }
    }

    for(int i=0;i<gaitParam.eeName.size();i++){
      ee_K.push_back((cnoid::Vector6() << 50, 50, 50, 20, 20, 20).finished());
      ee_D.push_back((cnoid::Vector6() << 10, 10, 10, 10, 10, 10).finished());
    }
    for(int i=0;i<NUM_LEGS;i++){
      ee_swing_K.push_back((cnoid::Vector6() << 200, 200, 200, 100, 100, 100).finished());
      ee_swing_D.push_back((cnoid::Vector6() << 30, 30, 30, 20, 20, 20).finished());
      ee_landing_K.push_back((cnoid::Vector6() << 200, 200, 20, 100, 100, 100).finished());
      ee_landing_D.push_back((cnoid::Vector6() << 30, 30, 5, 20, 20, 20).finished());
    }

    dqWeight.resize(actRobotTqc->numJoints(), cpp_filters::TwoPointInterpolator<double>(1.0, 0.0, 0.0, cpp_filters::HOFFARBIB));

    ikEEPositionConstraint.clear();
    for(int i=0;i<gaitParam.eeName.size();i++) ikEEPositionConstraint.push_back(std::make_shared<aik_constraint::PositionConstraint>());
    refJointAngleConstraint.clear();
    for(int i=0;i<actRobotTqc->numJoints();i++) refJointAngleConstraint.push_back(std::make_shared<aik_constraint::JointAngleConstraint>());
    jointLimitConstraint.clear();
    for(int i=0;i<actRobotTqc->numJoints();i++) jointLimitConstraint.push_back(std::make_shared<aik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint>());
    // selfCollisionConstraint.clear();
    // for(int i=0;i<gaitParam.selfCollision.size();i++) selfCollisionConstraint.push_back(std::make_shared<aik_constraint::ClientCollisionConstraint>());
  }
protected:
  // 計算高速化のためのキャッシュ. クリアしなくても別に副作用はない.
  mutable std::shared_ptr<prioritized_qp_osqp::Task> constraintTask_ = std::make_shared<prioritized_qp_osqp::Task>();
  mutable std::shared_ptr<prioritized_qp_osqp::Task> tgtForceTask_ = std::make_shared<prioritized_qp_osqp::Task>();
  mutable std::shared_ptr<prioritized_qp_osqp::Task> tgtTorqueTask_ = std::make_shared<prioritized_qp_osqp::Task>();
  mutable std::shared_ptr<prioritized_qp_osqp::Task> normTask_ = std::make_shared<prioritized_qp_osqp::Task>();

  // FullbodyIKSolverでのみ使うキャッシュ
  // 内部にヤコビアンの情報をキャッシュするが、クリアしなくても副作用はあまりない
  mutable std::vector<std::shared_ptr<aik_constraint::PositionConstraint> > ikEEPositionConstraint; // 要素数と順序はeeNameと同じ.
  mutable std::vector<std::shared_ptr<aik_constraint::JointAngleConstraint> > refJointAngleConstraint; // 要素数と順序はrobot->numJoints()と同じ
  mutable std::shared_ptr<aik_constraint::PositionConstraint> rootPositionConstraint = std::make_shared<aik_constraint::PositionConstraint>();
  mutable std::shared_ptr<aik_constraint::COMConstraint> comConstraint = std::make_shared<aik_constraint::COMConstraint>();
  mutable std::shared_ptr<aik_constraint::AngularMomentumConstraint> angularMomentumConstraint = std::make_shared<aik_constraint::AngularMomentumConstraint>();
  mutable std::vector<std::shared_ptr<aik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint> > jointLimitConstraint;
  mutable std::vector<std::shared_ptr<aik_constraint::ClientCollisionConstraint> > selfCollisionConstraint;
  mutable std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
public:
  void initStabilizerOutput(const GaitParam& gaitParam,
                            cnoid::Vector3& o_stTargetZmp, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPGainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDGainPercentage) const;

  bool execStabilizer(const GaitParam& gaitParam, double dt, bool useActState,
                      cnoid::BodyPtr& actRobotTqc, cnoid::Vector3& o_stTargetZmp, std::vector<cnoid::Vector6>& o_stEETargetWrench, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPgainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDgainPercentage) const;

protected:
  bool calcZMP(const GaitParam& gaitParam, double dt, bool useActState,
               cnoid::Vector3& o_tgtZmp/*generate座標系*/, cnoid::Vector3& o_tgtCogAcc/*generate座標系*/, cnoid::Vector3& o_tgtCogForce/*generate座標系*/) const;
  bool calcResolvedAccelerationControl(const GaitParam& gaitParam, double dt, cnoid::Vector3& tgtCogAcc/*generate座標系*/, bool useActState,
                                       cnoid::BodyPtr& actRobotTqc) const;
  bool calcWrench(const GaitParam& gaitParam, const cnoid::Vector3& tgtZmp/*generate座標系*/, const cnoid::Vector3& tgtForce/*generate座標系. ロボットが受ける力*/, bool useActState,
                  std::vector<cnoid::Vector6>& o_tgtEEWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/, cnoid::BodyPtr& actRobotTqc) const;
};

#endif
