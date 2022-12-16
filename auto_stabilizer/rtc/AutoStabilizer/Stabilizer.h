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
  std::vector<cnoid::Vector6> ee_K; // 要素数EndEffectors. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_D; // 要素数EndEffectors. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_swing_K; // 要素数NUM_LEGS. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_swing_D; // 要素数NUM_LEGS. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_landing_K; // 要素数NUM_LEGS. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_landing_D; // 要素数NUM_LEGS. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_support_D; // 要素数NUM_LEGS. EndEffector frame. endEffector origin. 0以上
  cnoid::Vector3 root_K = cnoid::Vector3(100,100,100); // rootlink frame.  0以上
  cnoid::Vector3 root_D = cnoid::Vector3(25,25,25); // rootlink frame 0以上
  double joint_K = 1.0; // 0以上
  double joint_D = 1.0; // 0以上

  std::vector<cpp_filters::TwoPointInterpolator<double> > dqWeight; // 要素数と順序はrobot->numJoints()と同じ. 0より大きい. 各関節の速度に対するダンピング項の比. default 1. 動かしたくない関節は大きくする. 全く動かしたくないなら、controllable_jointsを使うこと

  void init(const GaitParam& gaitParam, cnoid::BodyPtr& actRobotTqc){
    for(int i=0;i<gaitParam.eeName.size();i++){
      ee_K.push_back((cnoid::Vector6() << 50, 50, 50, 20, 20, 20).finished());
      ee_D.push_back((cnoid::Vector6() << 10, 10, 10, 10, 10, 10).finished());
    }
    for(int i=0;i<NUM_LEGS;i++){
      ee_swing_K.push_back((cnoid::Vector6() << 200, 200, 200, 100, 100, 100).finished());
      ee_swing_D.push_back((cnoid::Vector6() << 30, 30, 30, 20, 20, 20).finished());
      ee_landing_K.push_back((cnoid::Vector6() << 200, 200, 20, 100, 100, 100).finished());
      ee_landing_D.push_back((cnoid::Vector6() << 30, 30, 5, 20, 20, 20).finished());
      ee_support_D.push_back((cnoid::Vector6() << 30, 30, 50, 20, 20, 20).finished());
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
               cnoid::Vector3& o_tgtZmp/*generate座標系*/, cnoid::Vector3& o_tgtCogAcc/*generate座標系*/) const;
  bool calcResolvedAccelerationControl(const GaitParam& gaitParam, double dt, cnoid::Vector3& tgtCogAcc/*generate座標系*/, bool useActState,
                                       cnoid::BodyPtr& actRobotTqc) const;
  bool calcWrench(const GaitParam& gaitParam, const cnoid::Vector3& tgtZmp/*generate座標系*/, bool useActState,
                  std::vector<cnoid::Vector6>& o_tgtEEWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/, cnoid::BodyPtr& actRobotTqc) const;
};

#endif
