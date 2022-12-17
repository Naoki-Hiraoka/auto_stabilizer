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

#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint/COMConstraint.h>
#include <ik_constraint/JointAngleConstraint.h>
#include <ik_constraint/AngularMomentumConstraint.h>
#include <ik_constraint_joint_limit_table/JointLimitMinMaxTableConstraint.h>
#include <ik_constraint/JointVelocityConstraint.h>
#include <ik_constraint/ClientCollisionConstraint.h>
#include <prioritized_inverse_kinematics_solver/PrioritizedInverseKinematicsSolver.h>

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
  std::vector<cpp_filters::TwoPointInterpolator<double> > aikdqWeight; // 要素数と順序はrobot->numJoints()と同じ. 0より大きい. 各関節の速度に対するダンピング項の比. default 1. 動かしたくない関節は大きくする. 全く動かしたくないなら、controllable_jointsを使うこと. resolved acceleration control用

  std::vector<cpp_filters::TwoPointInterpolator<double> > ikdqWeight; // 要素数と順序はrobot->numJoints()と同じ. 0より大きい. 各関節の変位に対する重みの比. default 1. 動かしたくない関節は大きくする. 全く動かしたくないなら、controllable_jointsを使うこと. fullbody ik用

  void init(const GaitParam& gaitParam, cnoid::BodyPtr& actRobotTqc, cnoid::BodyPtr& genRobot){
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

    ikdqWeight.resize(genRobot->numJoints(), cpp_filters::TwoPointInterpolator<double>(1.0, 0.0, 0.0, cpp_filters::HOFFARBIB));

    aikdqWeight.resize(actRobotTqc->numJoints(), cpp_filters::TwoPointInterpolator<double>(1.0, 0.0, 0.0, cpp_filters::HOFFARBIB));
    aikEEPositionConstraint.clear();
    for(int i=0;i<gaitParam.eeName.size();i++) aikEEPositionConstraint.push_back(std::make_shared<aik_constraint::PositionConstraint>());
    aikRefJointAngleConstraint.clear();
    for(int i=0;i<actRobotTqc->numJoints();i++) aikRefJointAngleConstraint.push_back(std::make_shared<aik_constraint::JointAngleConstraint>());
    aikJointLimitConstraint.clear();
    for(int i=0;i<actRobotTqc->numJoints();i++) aikJointLimitConstraint.push_back(std::make_shared<aik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint>());
    // selfCollisionConstraint.clear();
    // for(int i=0;i<gaitParam.selfCollision.size();i++) selfCollisionConstraint.push_back(std::make_shared<aik_constraint::ClientCollisionConstraint>());

    ikEEPositionConstraint.clear();
    for(int i=0;i<gaitParam.eeName.size();i++) ikEEPositionConstraint.push_back(std::make_shared<IK::PositionConstraint>());
    ikRefJointAngleConstraint.clear();
    for(int i=0;i<genRobot->numJoints();i++) ikRefJointAngleConstraint.push_back(std::make_shared<IK::JointAngleConstraint>());
    ikJointVelocityConstraint.clear();
    for(int i=0;i<genRobot->numJoints();i++) ikJointVelocityConstraint.push_back(std::make_shared<IK::JointVelocityConstraint>());
    ikJointLimitConstraint.clear();
    for(int i=0;i<genRobot->numJoints();i++) ikJointLimitConstraint.push_back(std::make_shared<ik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint>());
    ikSelfCollisionConstraint.clear();
    for(int i=0;i<gaitParam.selfCollision.size();i++) ikSelfCollisionConstraint.push_back(std::make_shared<IK::ClientCollisionConstraint>());

    commandJointAngleFilter.resize(genRobot->numJoints(), cpp_filters::TwoPointInterpolator<double>(0.0, 0.0, 0.0, cpp_filters::HOFFARBIB));
  }

  // startAutoBalancer時に一回呼ばれる
  void reset(){
    for(int i=0;i<ikdqWeight.size();i++) ikdqWeight[i].reset(ikdqWeight[i].getGoal());
    for(int i=0;i<aikdqWeight.size();i++) aikdqWeight[i].reset(aikdqWeight[i].getGoal());
  }

  // 毎周期呼ばれる
  void update(double dt){
    for(int i=0;i<ikdqWeight.size();i++) ikdqWeight[i].interpolate(dt);
    for(int i=0;i<aikdqWeight.size();i++) aikdqWeight[i].interpolate(dt);
  }

protected:
  // 計算高速化のためのキャッシュ. クリアしなくても別に副作用はない.
  mutable std::shared_ptr<prioritized_qp_osqp::Task> constraintTask_ = std::make_shared<prioritized_qp_osqp::Task>();
  mutable std::shared_ptr<prioritized_qp_osqp::Task> tgtForceTask_ = std::make_shared<prioritized_qp_osqp::Task>();
  mutable std::shared_ptr<prioritized_qp_osqp::Task> tgtTorqueTask_ = std::make_shared<prioritized_qp_osqp::Task>();
  mutable std::shared_ptr<prioritized_qp_osqp::Task> normTask_ = std::make_shared<prioritized_qp_osqp::Task>();

  // 内部にヤコビアンの情報をキャッシュするが、クリアしなくても副作用はあまりない
  // ik
  mutable std::vector<std::shared_ptr<IK::PositionConstraint> > ikEEPositionConstraint; // 要素数と順序はeeNameと同じ.
  mutable std::vector<std::shared_ptr<IK::JointAngleConstraint> > ikRefJointAngleConstraint; // 要素数と順序はrobot->numJoints()と同じ
  mutable std::shared_ptr<IK::PositionConstraint> ikRootPositionConstraint = std::make_shared<IK::PositionConstraint>();
  mutable std::shared_ptr<IK::COMConstraint> ikComConstraint = std::make_shared<IK::COMConstraint>();
  mutable std::shared_ptr<IK::AngularMomentumConstraint> ikAngularMomentumConstraint = std::make_shared<IK::AngularMomentumConstraint>();
  mutable std::vector<std::shared_ptr<ik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint> > ikJointLimitConstraint;
  mutable std::vector<std::shared_ptr<IK::JointVelocityConstraint> > ikJointVelocityConstraint;
  mutable std::vector<std::shared_ptr<IK::ClientCollisionConstraint> > ikSelfCollisionConstraint;
  mutable std::vector<std::shared_ptr<prioritized_qp_base::Task> > ikTasks;
  // aik
  mutable std::vector<std::shared_ptr<aik_constraint::PositionConstraint> > aikEEPositionConstraint; // 要素数と順序はeeNameと同じ.
  mutable std::vector<std::shared_ptr<aik_constraint::JointAngleConstraint> > aikRefJointAngleConstraint; // 要素数と順序はrobot->numJoints()と同じ
  mutable std::shared_ptr<aik_constraint::PositionConstraint> aikRootPositionConstraint = std::make_shared<aik_constraint::PositionConstraint>();
  mutable std::shared_ptr<aik_constraint::COMConstraint> aikComConstraint = std::make_shared<aik_constraint::COMConstraint>();
  mutable std::shared_ptr<aik_constraint::AngularMomentumConstraint> aikAngularMomentumConstraint = std::make_shared<aik_constraint::AngularMomentumConstraint>();
  mutable std::vector<std::shared_ptr<aik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint> > aikJointLimitConstraint;
  mutable std::vector<std::shared_ptr<aik_constraint::ClientCollisionConstraint> > aikSelfCollisionConstraint;
  mutable std::vector<std::shared_ptr<prioritized_qp_base::Task> > aikTasks;

  // mode_ST時に、指令関節角度をactual角度に緩やかに従わせるためのフィルター. stがオフのときは今の指令関節角度で毎回リセットされるので、クリアしなくても副作用はあまりない
  mutable cpp_filters::TwoPointInterpolatorSE3 commandRootPoseFilter = cpp_filters::TwoPointInterpolatorSE3(cnoid::Position::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB);
  mutable std::vector<cpp_filters::TwoPointInterpolator<double> > commandJointAngleFilter;
public:
  bool execStabilizer(const GaitParam& gaitParam, double dt, bool useActState,
                      GaitParam::DebugData& debugData, //for Log
                      cnoid::BodyPtr& actRobotTqc, cnoid::BodyPtr& genRobot, cnoid::Vector3& o_genNextCog, cnoid::Vector3& o_genNextCogVel, cnoid::Vector3& o_genNextCogAcc) const;

protected:
  bool calcCogAcc(const GaitParam& gaitParam, double dt, bool useActState,
                  GaitParam::DebugData& debugData, //for Log
                  cnoid::Vector3& o_tgtCogAcc/*generate座標系*/, cnoid::Vector3& o_genNextCog, cnoid::Vector3& o_genNextCogVel, cnoid::Vector3& o_genNextCogAcc, cnoid::Vector3& o_genNextForce) const;
  bool calcZmp(const GaitParam& gaitParam, const cnoid::Vector3& cog, const cnoid::Vector3& DCM, const std::vector<cnoid::Position>& EEPose, const bool& useSoftLimit,
               cnoid::Vector3& o_zmp) const;
  bool calcResolvedAccelerationControl(const GaitParam& gaitParam, double dt, const cnoid::Vector3& tgtCogAcc/*generate座標系*/, const cnoid::Vector3& genNextCog, bool useActState,
                                       cnoid::BodyPtr& actRobotTqc, cnoid::BodyPtr& o_genRobot) const;
  bool calcWrench(const GaitParam& gaitParam, const cnoid::Vector3& genNextForce, bool useActState,
                  std::vector<cnoid::Vector6>& o_tgtEEWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/, cnoid::BodyPtr& actRobotTqc) const;
};

#endif
