#include "FullbodyIKSolver.h"
#include <fullbody_inverse_kinematics_solver/FullbodyInverseKinematicsSolverFast.h>

bool FullbodyIKSolver::solveFullbodyIK(double dt, const GaitParam& gaitParam,
                                       cnoid::BodyPtr& genRobot) const{
  // !jointControllableの関節は指令値をそのまま入れる
  for(size_t i=0;i<genRobot->numJoints();i++){
    if(!gaitParam.jointControllable[i]) genRobot->joint(i)->q() = gaitParam.refRobot->joint(i)->q();
  }

  if(this->jlim_avoid_weight.size() != 6+genRobot->numJoints()) this->jlim_avoid_weight = cnoid::VectorX::Zero(6+genRobot->numJoints());
  cnoid::VectorX dq_weight_all = cnoid::VectorX::Zero(6+genRobot->numJoints());
  for(int i=0;i<6;i++) dq_weight_all[i] = 1.0;
  for(int i=0;i<genRobot->numJoints();i++){
    if(gaitParam.jointControllable[i]) dq_weight_all[6+i] = 1.0;
  }

  std::vector<std::shared_ptr<IK::IKConstraint> > ikConstraint;

  // EEF
  for(int i=0;i<gaitParam.eeName.size();i++){
    this->ikEEPositionConstraint[i]->A_link() = genRobot->link(gaitParam.eeParentLink[i]);
    this->ikEEPositionConstraint[i]->A_localpos() = gaitParam.eeLocalT[i];
    this->ikEEPositionConstraint[i]->B_link() = nullptr;
    this->ikEEPositionConstraint[i]->B_localpos() = gaitParam.stEETargetPose[i];
    this->ikEEPositionConstraint[i]->maxError() << 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt;
    this->ikEEPositionConstraint[i]->precision() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
    if(i<NUM_LEGS) this->ikEEPositionConstraint[i]->weight() << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
    else this->ikEEPositionConstraint[i]->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    this->ikEEPositionConstraint[i]->eval_link() = nullptr;
    this->ikEEPositionConstraint[i]->eval_localR() = this->ikEEPositionConstraint[i]->B_localpos().linear();
    ikConstraint.push_back(this->ikEEPositionConstraint[i]);
  }

  // COM
  {
    this->comConstraint->A_robot() = genRobot;
    this->comConstraint->A_localp() = cnoid::Vector3::Zero();
    this->comConstraint->B_robot() = nullptr;
    this->comConstraint->B_localp() = gaitParam.genCog + gaitParam.sbpOffset;
    this->comConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt;
    this->comConstraint->precision() << 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
    this->comConstraint->weight() << 3.0, 3.0, 1.0;
    this->comConstraint->eval_R() = cnoid::Matrix3::Identity();
    ikConstraint.push_back(this->comConstraint);
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
    ikConstraint.push_back(this->angularMomentumConstraint);
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
    ikConstraint.push_back(this->rootPositionConstraint);
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
      ikConstraint.push_back(this->refJointAngleConstraint[i]);
    }
  }

  // 特異点近傍で振動するようなことは起こりにくいが、歩行動作中の一瞬だけIKがときにくい姿勢があってすぐに解ける姿勢に戻るといった場合に、その一瞬の間だけIKを解くために頑張って姿勢が大きく変化するので、危険.
  //  この現象を防ぐには、未来の情報を含んだIKを作るか、歩行動作中にIKが解きづらい姿勢を経由しないように着地位置等をリミットするか. 後者を採用
  //  歩行動作ではないゆっくりとした動作であれば、この現象が発生しても問題ない

  for(int i=0;i<ikConstraint.size();i++) ikConstraint[i]->debuglevel() = 0; //debuglevel
  fik::solveFullbodyIKLoopFast(genRobot,
                               ikConstraint,
                               this->jlim_avoid_weight,
                               dq_weight_all,
                               1,//loop
                               1e-6, // wn
                               0, //debug
                               dt,
                               1e2 // we. 1e0だとやや不安定. 1e3だと大きすぎる
                               );

  // limit joint angle by joint limit table
  //   本当はjoint->q_upper()とjoint->q_lower()をjoint limit tableを用いて更新して、fullbodyIKの中で自動的に考慮して解いて欲しい. 解いた後にlimitすると、各周期における各タスクの精度が若干不正確になる. (複数周期を通して見れば目標に収束するのでそこまで問題ではないが).
  //   joint->q_upper()とjoint->q_lower()をjoint limit tableを用いて更新しない理由:
  //     fullbodyIKは不等式制約が扱えないので、前回の周期の開始時よりも今回の開始時の方がlimitに近ければ、その関節のdq_weightを大きくして動かしにくくする、というアプローチをとっている.
  //     これは、各タスクが滑らかに変化していることを前提にしている。そうでなく、limitからの距離が近づいたり離れたりを頻繁にくりかえすような状況では、重みが不連続に変動するのでロボットが振動的になってしまう.
  //     ここまでは、各タスクが滑らかに変化するように作ればよく、実用上のほとんどのケースでは各タスクは滑らかに変化するので、問題ない.
  //     ところが、joint_limit_tableを用いてjoint->q_upper()とjoint->q_lower()を更新すると、limit側が勝手に動くので、タスクに関係なく、limitからの距離が近づいたり離れたりする. すると、ロボットが振動的になってしまうことがある.
  //     不等式制約が扱えるQPベースのIKなら、この問題は発生しない.
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
