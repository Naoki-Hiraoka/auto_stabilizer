#ifndef FULLBODYIKSOLVER_H
#define FULLBODYIKSOLVER_H

#include "GaitParam.h"
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint/COMConstraint.h>
#include <ik_constraint/JointAngleConstraint.h>
#include <ik_constraint/AngularMomentumConstraint.h>
#include <ik_constraint_joint_limit_table/JointLimitMinMaxTableConstraint.h>
#include <ik_constraint/JointVelocityConstraint.h>
#include <ik_constraint/ClientCollisionConstraint.h>
#include <prioritized_inverse_kinematics_solver/PrioritizedInverseKinematicsSolver.h>

class FullbodyIKSolver{
public:
  // FullbodyIKSolverでのみ使うパラメータ
  // 内部にヤコビアンの情報をキャッシュするが、クリアしなくても副作用はあまりない
  mutable std::vector<std::shared_ptr<IK::PositionConstraint> > ikEEPositionConstraint; // 要素数と順序はeeNameと同じ.
  mutable std::vector<std::shared_ptr<IK::JointAngleConstraint> > refJointAngleConstraint; // 要素数と順序はrobot->numJoints()と同じ
  mutable std::shared_ptr<IK::PositionConstraint> rootPositionConstraint = std::make_shared<IK::PositionConstraint>();
  mutable std::shared_ptr<IK::COMConstraint> comConstraint = std::make_shared<IK::COMConstraint>();
  mutable std::shared_ptr<IK::AngularMomentumConstraint> angularMomentumConstraint = std::make_shared<IK::AngularMomentumConstraint>();
  mutable std::vector<std::shared_ptr<ik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint> > jointLimitConstraint;
  mutable std::vector<std::shared_ptr<IK::JointVelocityConstraint> > jointVelocityConstraint;
  mutable std::vector<std::shared_ptr<IK::ClientCollisionConstraint> > selfCollisionConstraint;
protected:
  // クリアしなくても副作用はあまりない
  mutable cnoid::VectorX jlim_avoid_weight;
  mutable std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
public:
  void init(const cnoid::BodyPtr& genRobot, const GaitParam& gaitParam){
    ikEEPositionConstraint.clear();
    for(int i=0;i<gaitParam.eeName.size();i++) ikEEPositionConstraint.push_back(std::make_shared<IK::PositionConstraint>());
    refJointAngleConstraint.clear();
    for(int i=0;i<genRobot->numJoints();i++) refJointAngleConstraint.push_back(std::make_shared<IK::JointAngleConstraint>());
    jointVelocityConstraint.clear();
    for(int i=0;i<genRobot->numJoints();i++) jointVelocityConstraint.push_back(std::make_shared<IK::JointVelocityConstraint>());
    jointLimitConstraint.clear();
    for(int i=0;i<genRobot->numJoints();i++) jointLimitConstraint.push_back(std::make_shared<ik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint>());
    selfCollisionConstraint.clear();
    for(int i=0;i<gaitParam.selfCollision.size();i++) selfCollisionConstraint.push_back(std::make_shared<IK::ClientCollisionConstraint>());
  }

  bool solveFullbodyIK(double dt, const GaitParam& gaitParam,
                       cnoid::BodyPtr& genRobot) const;
};

#endif
