#include "FullbodyIKSolver.h"
#include <prioritized_inverse_kinematics_solver/PrioritizedInverseKinematicsSolver.h>

bool FullbodyIKSolver::solveFullbodyIK(double dt, const GaitParam& gaitParam,
                                       cnoid::BodyPtr& genRobot) const{
  // !jointControllableの関節は指令値をそのまま入れる
  for(size_t i=0;i<genRobot->numJoints();i++){
    if(!gaitParam.jointControllable[i]) genRobot->joint(i)->q() = gaitParam.refRobot->joint(i)->q();
  }

  // jointControllableの関節のみ、探索変数にする
  std::vector<cnoid::LinkPtr> variables; variables.reserve(1+genRobot->numJoints());
  std::vector<double> dqWeight; dqWeight.reserve(6+genRobot->numJoints());
  variables.push_back(genRobot->rootLink());
  for(int i=0;i<6;i++) dqWeight.push_back(1.0);
  for(size_t i=0;i<genRobot->numJoints();i++){
    if(gaitParam.jointControllable[i]) {
      variables.push_back(genRobot->joint(i));
      dqWeight.push_back(this->dqWeight[i].value());
    }
  }

  std::vector<std::shared_ptr<IK::IKConstraint> > ikConstraint0;

  // joint velocity
  for(size_t i=0;i<genRobot->numJoints();i++){
    if(!gaitParam.jointControllable[i]) continue;
    this->jointVelocityConstraint[i]->joint() = genRobot->joint(i);
    this->jointVelocityConstraint[i]->dt() = dt;
    this->jointVelocityConstraint[i]->maxError() = 1.0 * dt;
    this->jointVelocityConstraint[i]->weight() = 1.0;
    ikConstraint0.push_back(this->jointVelocityConstraint[i]);
  }

  // joint angle
  for(size_t i=0;i<genRobot->numJoints();i++){
    if(!gaitParam.jointControllable[i]) continue;
    this->jointLimitConstraint[i]->joint() = genRobot->joint(i);
    this->jointLimitConstraint[i]->jointLimitTables() = gaitParam.jointLimitTables[i];
    this->jointLimitConstraint[i]->maxError() = 1.0 * dt;
    this->jointLimitConstraint[i]->weight() = 1.0;
    ikConstraint0.push_back(this->jointLimitConstraint[i]);
  }

  std::vector<std::shared_ptr<IK::IKConstraint> > ikConstraint1;
  this->selfCollisionConstraint.resize(gaitParam.selfCollision.size());
  for(size_t i=0;i<this->selfCollisionConstraint.size();i++){
    if(!this->selfCollisionConstraint[i]) this->selfCollisionConstraint[i] = std::make_shared<IK::ClientCollisionConstraint>();
    this->selfCollisionConstraint[i]->A_link() = genRobot->link(gaitParam.selfCollision[i].link1);
    this->selfCollisionConstraint[i]->B_link() = genRobot->link(gaitParam.selfCollision[i].link2);
    this->selfCollisionConstraint[i]->tolerance() = 0.01;
    this->selfCollisionConstraint[i]->maxError() = 10.0*dt;
    this->selfCollisionConstraint[i]->weight() = 1.0;
    this->selfCollisionConstraint[i]->velocityDamper() = 0.1 / dt;
    this->selfCollisionConstraint[i]->A_localp() = gaitParam.selfCollision[i].point1;
    this->selfCollisionConstraint[i]->B_localp() = gaitParam.selfCollision[i].point2;
    this->selfCollisionConstraint[i]->direction() = gaitParam.selfCollision[i].direction21;

    // 全自己干渉情報を与えると計算コストが膨大になるため、距離が近いもののみ与える
    if(gaitParam.selfCollision[i].distance < 0.05){
      ikConstraint1.push_back(this->selfCollisionConstraint[i]);
    }
  }

  std::vector<std::shared_ptr<IK::IKConstraint> > ikConstraint2;

  // EEF
  for(int i=0;i<gaitParam.eeName.size();i++){
    this->ikEEPositionConstraint[i]->A_link() = genRobot->link(gaitParam.eeParentLink[i]);
    this->ikEEPositionConstraint[i]->A_localpos() = gaitParam.eeLocalT[i];
    this->ikEEPositionConstraint[i]->B_link() = nullptr;
    this->ikEEPositionConstraint[i]->B_localpos() = gaitParam.abcEETargetPose[i];
    this->ikEEPositionConstraint[i]->maxError() << 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt;
    this->ikEEPositionConstraint[i]->precision() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
    if(i<NUM_LEGS) this->ikEEPositionConstraint[i]->weight() << 3.0, 3.0, 3.0, 3.0, 3.0, 3.0;
    else this->ikEEPositionConstraint[i]->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    this->ikEEPositionConstraint[i]->eval_link() = nullptr;
    this->ikEEPositionConstraint[i]->eval_localR() = this->ikEEPositionConstraint[i]->B_localpos().linear();
    ikConstraint2.push_back(this->ikEEPositionConstraint[i]);
  }

  // COM
  {
    this->comConstraint->A_robot() = genRobot;
    this->comConstraint->A_localp() = cnoid::Vector3::Zero();
    this->comConstraint->B_robot() = nullptr;
    this->comConstraint->B_localp() = gaitParam.genCog + gaitParam.sbpOffset;
    this->comConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt;
    this->comConstraint->precision() << 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
    this->comConstraint->weight() << 10.0, 10.0, 1.0;
    this->comConstraint->eval_R() = cnoid::Matrix3::Identity();
    ikConstraint2.push_back(this->comConstraint);
  }

  // Angular Momentum
  {
    this->angularMomentumConstraint->robot() = genRobot;
    this->angularMomentumConstraint->targetAngularMomentum() = cnoid::Vector3::Zero(); // TODO
    this->angularMomentumConstraint->maxError() << 1.0*dt, 1.0*dt, 1.0*dt;
    this->angularMomentumConstraint->precision() << 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
    this->angularMomentumConstraint->weight() << 1e-4, 1e-4, 0.0; // TODO
    this->angularMomentumConstraint->dt() = dt;
    this->comConstraint->eval_R() = cnoid::Matrix3::Identity();
    ikConstraint2.push_back(this->angularMomentumConstraint);
  }

  // root
  {
    this->rootPositionConstraint->A_link() = genRobot->rootLink();
    this->rootPositionConstraint->A_localpos() = cnoid::Position::Identity();
    this->rootPositionConstraint->B_link() = nullptr;
    this->rootPositionConstraint->B_localpos() = gaitParam.stTargetRootPose;
    this->rootPositionConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt;
    this->rootPositionConstraint->precision() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
    this->rootPositionConstraint->weight() << 0.0, 0.0, 0.0, 3.0, 3.0, 3.0; // 角運動量を利用するときは重みを小さく. 通常時、胴の質量・イナーシャやマスパラ誤差の大きさや、胴を大きく動かすための出力不足などによって、二足動歩行では胴の傾きの自由度を使わない方がよい
    //this->rootPositionConstraint->weight() << 0.0, 0.0, 0.0, 3e-1, 3e-1, 3e-1;
    this->rootPositionConstraint->eval_link() = nullptr;
    this->rootPositionConstraint->eval_localR() = cnoid::Matrix3::Identity();
    ikConstraint2.push_back(this->rootPositionConstraint);
  }

  // reference angle
  {
    for(size_t i=0;i<genRobot->numJoints();i++){
      if(!gaitParam.jointControllable[i]) continue;
      this->refJointAngleConstraint[i]->joint() = genRobot->joint(i);
      this->refJointAngleConstraint[i]->maxError() = 10.0 * dt; // 高優先度のmaxError以下にしないと優先度逆転するおそれ
      this->refJointAngleConstraint[i]->weight() = 1e-1; // 小さい値すぎると、qp終了判定のtoleranceによって無視されてしまう
      this->refJointAngleConstraint[i]->targetq() = gaitParam.refRobot->joint(i)->q();
      this->refJointAngleConstraint[i]->precision() = 0.0; // 強制的にIKをmax loopまで回す
      ikConstraint2.push_back(this->refJointAngleConstraint[i]);
    }
  }

  // 特異点近傍で振動するようなことは起こりにくいが、歩行動作中の一瞬だけIKがときにくい姿勢があってすぐに解ける姿勢に戻るといった場合に、その一瞬の間だけIKを解くために頑張って姿勢が大きく変化するので、危険.
  //  この現象を防ぐには、未来の情報を含んだIKを作るか、歩行動作中にIKが解きづらい姿勢を経由しないように着地位置等をリミットするか. 後者を採用
  //  歩行動作ではないゆっくりとした動作であれば、この現象が発生しても問題ない

  std::vector<std::vector<std::shared_ptr<IK::IKConstraint> > > constraints{ikConstraint0,ikConstraint1,ikConstraint2};
  for(size_t i=0;i<constraints.size();i++){
    for(size_t j=0;j<constraints[i].size();j++){
      constraints[i][j]->debuglevel() = 0;//debuglevel
    }
  }
  prioritized_inverse_kinematics_solver::IKParam param;
  param.maxIteration = 1;
  param.dqWeight = dqWeight;
  param.wn = 1e-6;
  param.we = 1e2; // 1e0だとやや不安定. 1e3だと大きすぎる
  param.debugLevel = 0;
  param.dt = dt;
  prioritized_inverse_kinematics_solver::solveIKLoop(variables,
                                                     constraints,
                                                     this->tasks,
                                                     param
                                                     );


  // 念の為limit check
  for(int i=0;i<gaitParam.refRobot->numJoints();i++){
    if(!gaitParam.jointControllable[i]) continue;
    cnoid::LinkPtr joint = genRobot->joint(i);
    double u = gaitParam.refRobot->joint(i)->q_upper();
    double l = gaitParam.refRobot->joint(i)->q_lower();
    for(int j=0;j<gaitParam.jointLimitTables[i].size();j++){
      u = std::min(u,gaitParam.jointLimitTables[i][j]->getUlimit());
      l = std::max(l,gaitParam.jointLimitTables[i][j]->getLlimit());
    }
    joint->q() = std::min(u, std::max(l, joint->q()));
  }

  return true;
}
