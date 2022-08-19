#ifndef FULLBODYIKSOLVER_H
#define FULLBODYIKSOLVER_H

#include "GaitParam.h"
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint/COMConstraint.h>
#include <ik_constraint/JointAngleConstraint.h>
#include <ik_constraint/AngularMomentumConstraint.h>

class FullbodyIKSolver{
public:
  // FullbodyIKSolverでのみ使うパラメータ
  // 内部にヤコビアンの情報をキャッシュするが、クリアしなくても副作用はあまりない
  mutable std::vector<std::shared_ptr<IK::PositionConstraint> > ikEEPositionConstraint; // 要素数と順序はeeNameと同じ.
  mutable std::vector<std::shared_ptr<IK::JointAngleConstraint> > refJointAngleConstraint; // 要素数と順序はrobot->numJoints()と同じ
  mutable std::shared_ptr<IK::PositionConstraint> rootPositionConstraint = std::make_shared<IK::PositionConstraint>();
  mutable std::shared_ptr<IK::COMConstraint> comConstraint = std::make_shared<IK::COMConstraint>();
  mutable std::shared_ptr<IK::AngularMomentumConstraint> angularMomentumConstraint = std::make_shared<IK::AngularMomentumConstraint>();
protected:
  // クリアしなくても副作用はあまりない
  mutable cnoid::VectorX jlim_avoid_weight;
public:
  void init(const cnoid::BodyPtr& genRobot, const GaitParam& gaitParam){
    ikEEPositionConstraint.clear();
    for(int i=0;i<gaitParam.eeName.size();i++) ikEEPositionConstraint.push_back(std::make_shared<IK::PositionConstraint>());
    refJointAngleConstraint.clear();
    for(int i=0;i<genRobot->numJoints();i++) refJointAngleConstraint.push_back(std::make_shared<IK::JointAngleConstraint>());
  }

  bool solveFullbodyIK(double dt, const GaitParam& gaitParam,
                       cnoid::BodyPtr& genRobot) const;
};

#endif
