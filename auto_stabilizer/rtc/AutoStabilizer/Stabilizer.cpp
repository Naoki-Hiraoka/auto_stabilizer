#include "Stabilizer.h"
#include "MathUtil.h"
#include <cnoid/Jacobian>
#include <cnoid/EigenUtil>
#include <cnoid/src/Body/InverseDynamics.h>

void Stabilizer::initStabilizerOutput(const GaitParam& gaitParam,
                                      cnoid::Vector3& o_stTargetZmp, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPGainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDGainPercentage) const{

  o_stTargetZmp = gaitParam.refZmpTraj[0].getStart();
  for(int i=0;i<o_stServoPGainPercentage.size();i++){
    o_stServoPGainPercentage[i].reset(100.0);
    o_stServoDGainPercentage[i].reset(100.0);
  }
}

bool Stabilizer::execStabilizer(const GaitParam& gaitParam, double dt, bool useActState,
                                cnoid::BodyPtr& actRobotTqc, cnoid::Vector3& o_stTargetZmp, std::vector<cnoid::Vector6>& o_stEETargetWrench, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPGainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDGainPercentage) const{
  // - 現在のactual重心位置から、目標重心加速を計算
  // - 目標重心加速を満たすように全身の加速度を計算
  // - 全身の加速度と重力に釣り合うように目標足裏反力を計算. 関節トルクも求まる

  /*
    FootGuided Controlから、次の周期の目標の重心の加速と、角運動量の変化(0)が出てくる. これを全身の加速度に分解するときのタスク優先度について、以下の循環優先度関係がある. この矛盾は、恐らく全身モデル予測制御を使わない限り無くならない.
    * 重心の目標加速(特にXY) > 腕などのエンドエフェクタの目標位置
        これは自明. 特に動歩行
    * 腕などのエンドエフェクタの目標位置 > 角運動量の目標変化
        角運動量の変化を厳密に守ると、まともにエンドエフェクタが動かない
    * 角運動量の目標変化 > 重心の目標加速
        重心位置はFootGuidedControlやFootStepModificationで制御できるが、角運動量は制御する手段が乏しく、発散しやすいため
    今は、次のような方法によって、とりあえず循環を回避している
    - acceleration based ik で、重心の目標加速(特にXY) > 腕などのエンドエフェクタの目標位置 > 角運動量の目標変化 の優先度で解く
    - wrench distributionで、acceleration based ikの結果の角運動量の変化 > acceleration based ikの結果の重心の加速 の優先度で解く.
  */

  // masterのhrpsysではSwingEEModification(位置制御)を行っていたが、地面についているときに、time_constでもとの位置に戻るまでの間、足と重心の相対位置が着地位置タイミング修正計算で用いるものとずれることによる着地位置修正パフォーマンスの低下のデメリットの方が大きいので、削除した
  // masterのhrpsysやもとのauto_stabilizerでは位置制御+DampingControlをサポートしていたが、位置制御+DampingControlは実機での目標接触力への追従に遅れがある. FootGuidedControlでは、目標ZMPの位相を進めたり、ZMPの追従遅れを考慮した目標ZMP計算を行ったりをしていないので、遅れに弱い. そのため、位置制御+DampingControlは削除し、所謂TorqueControlのみをサポートしている.


  // 現在のactual重心位置から、目標重心Accを計算
  cnoid::Vector3 tgtCogAcc; // generate frame
  this->calcZMP(gaitParam, dt, useActState, // input
                o_stTargetZmp, tgtCogAcc); // output

  // actRobotTqcのq,dqにactualの値を入れ、ddqに今回の目標値を求めて入れる
  this->calcResolvedAccelerationControl(gaitParam, dt, tgtCogAcc, useActState,
                                        actRobotTqc);

  // actRobotTqcの自重と加速に要する力と、manipulati0on arm/legのrefForceに釣り合うように、目標支持脚反力を計算. actRobotTqcのuを求める
  this->calcWrench(gaitParam, o_stTargetZmp, useActState,// input
                   o_stEETargetWrench, actRobotTqc); // output

  if(useActState){
    for(int i=0;i<actRobotTqc->numJoints();i++){
      if(o_stServoPGainPercentage[i].getGoal() != 0.0) o_stServoPGainPercentage[i].setGoal(0.0, 1.0);
      if(o_stServoDGainPercentage[i].getGoal() != 0.0) o_stServoDGainPercentage[i].setGoal(0.0, 1.0);
      o_stServoPGainPercentage[i].interpolate(dt);
      o_stServoDGainPercentage[i].interpolate(dt);
    }
  }else{
    for(int i=0;i<actRobotTqc->numJoints();i++) actRobotTqc->joint(i)->u() = 0.0;
    for(int i=0;i<actRobotTqc->numJoints();i++){
      o_stServoPGainPercentage[i].interpolate(dt);
      o_stServoDGainPercentage[i].interpolate(dt);
    }
  }

  return true;
}

bool Stabilizer::calcZMP(const GaitParam& gaitParam, double dt, bool useActState,
                         cnoid::Vector3& o_tgtZmp, cnoid::Vector3& o_tgtCogAcc) const{
  cnoid::Vector3 cog = useActState ? gaitParam.actCog : gaitParam.genCog;
  cnoid::Vector3 cogVel = useActState ? gaitParam.actCogVel.value() : gaitParam.genCogVel;
  cnoid::Vector3 DCM = cog + cogVel / gaitParam.omega;
  const std::vector<cnoid::Position>& EEPose = useActState ? gaitParam.actEEPose : gaitParam.abcEETargetPose;

  cnoid::Vector3 tgtZmp;
  if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] || gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    tgtZmp = footguidedcontroller::calcFootGuidedControl(gaitParam.omega,gaitParam.l,DCM,gaitParam.refZmpTraj);
    if(tgtZmp[2] >= gaitParam.actCog[2]) tgtZmp = gaitParam.actCog - cnoid::Vector3(gaitParam.l[0],gaitParam.l[1], 0.0); // 下向きの力は受けられないので
    else{
      // truncate zmp inside polygon. actual robotの関節角度を用いて計算する
      std::vector<cnoid::Vector3> vertices; // generate frame. 支持点の集合
      for(int i=0;i<NUM_LEGS;i++){
        if(!gaitParam.footstepNodesList[0].isSupportPhase[i]) continue;
        for(int j=0;j<gaitParam.legHull[i].size();j++){
          cnoid::Vector3 p = EEPose[i]*gaitParam.legHull[i][j];
          if(p[2] > gaitParam.actCog[2] - 1e-2) p[2] = gaitParam.actCog[2] - 1e-2; // 重心よりも支持点が高いと射影が破綻するので 
          vertices.push_back(p);
        }
      }
      tgtZmp = mathutil::calcInsidePointOfPolygon3D(tgtZmp,vertices,gaitParam.actCog - cnoid::Vector3(gaitParam.l[0],gaitParam.l[1], 0.0));
      // TODO. 角運動量オフセット.
    }
  }else{ // 跳躍期
    tgtZmp = cog - cnoid::Vector3(gaitParam.l[0],gaitParam.l[1], 0.0);
  }
  cnoid::Vector3 tgtCog,tgtCogVel,tgtCogAcc,tgtForce;
  footguidedcontroller::updateState(gaitParam.omega,gaitParam.l,cog,cogVel,tgtZmp,gaitParam.genRobot->mass(),dt,
                                      tgtCog, tgtCogVel, tgtCogAcc, tgtForce);

  o_tgtZmp = tgtZmp;
  o_tgtCogAcc = tgtCogAcc;
  return true;
}

bool Stabilizer::calcResolvedAccelerationControl(const GaitParam& gaitParam, double dt, cnoid::Vector3& tgtCogAcc/*generate座標系*/, bool useActState,
                                                 cnoid::BodyPtr& actRobotTqc) const{
  if(!useActState) return true; // TODO

  {
    // setup actRobotTqc
    actRobotTqc->rootLink()->T() = gaitParam.actRobot->rootLink()->T();
    actRobotTqc->rootLink()->v() = gaitParam.actRobot->rootLink()->v();
    actRobotTqc->rootLink()->w() = gaitParam.actRobot->rootLink()->w();
    actRobotTqc->rootLink()->dv().setZero();
    actRobotTqc->rootLink()->dw().setZero();
    for(int i=0;i<actRobotTqc->numJoints();i++){
      actRobotTqc->joint(i)->q() = gaitParam.actRobot->joint(i)->q();
      actRobotTqc->joint(i)->dq() = gaitParam.actRobot->joint(i)->dq();
      actRobotTqc->joint(i)->ddq() = 0.0;
    }
    for(int l=0;l<actRobotTqc->numLinks();l++) actRobotTqc->link(l)->F_ext().setZero();
    actRobotTqc->calcForwardKinematics(true,true);
    actRobotTqc->calcCenterOfMass();
    cnoid::Vector6 F_o = cnoid::calcInverseDynamics(actRobotTqc->rootLink()); // generate frame. generate frame origin
    actRobotTqc->rootLink()->F_ext().head<3>() = F_o.head<3>(); // generate frame. rootLink origin
    actRobotTqc->rootLink()->F_ext().tail<3>() = F_o.tail<3>() + (-actRobotTqc->rootLink()->p()).cross(F_o.head<3>()); // generate frame. rootLink origin
  }

  // jointControllableの関節のみ、探索変数にする
  std::vector<cnoid::LinkPtr> variables; variables.reserve(1+actRobotTqc->numJoints());
  variables.push_back(actRobotTqc->rootLink());
  for(size_t i=0;i<actRobotTqc->numJoints();i++){
    if(gaitParam.jointControllable[i]) {
      variables.push_back(actRobotTqc->joint(i));
    }
  }

  std::vector<std::shared_ptr<aik_constraint::IKConstraint> > ikConstraint0;

  // joint limit
  for(size_t i=0;i<actRobotTqc->numJoints();i++){
    if(!gaitParam.jointControllable[i]) continue;
    this->jointLimitConstraint[i]->joint() = actRobotTqc->joint(i);
    this->jointLimitConstraint[i]->jointLimitTables() = gaitParam.jointLimitTablesTqc[i];
    this->jointLimitConstraint[i]->pgain() = 400;
    this->jointLimitConstraint[i]->dgain() = 100;
    this->jointLimitConstraint[i]->maxAccByVelError() = 20.0;
    this->jointLimitConstraint[i]->weight() = 0.1;
    ikConstraint0.push_back(this->jointLimitConstraint[i]);
  }

  std::vector<std::shared_ptr<aik_constraint::IKConstraint> > ikConstraint1;
  // self Collision TODO

  std::vector<std::shared_ptr<aik_constraint::IKConstraint> > ikConstraint2;
  std::vector<std::shared_ptr<aik_constraint::IKConstraint> > ikConstraint3;

  for(int i=0;i<gaitParam.eeName.size();i++){
    this->ikEEPositionConstraint[i]->A_link() = actRobotTqc->link(gaitParam.eeParentLink[i]);
    this->ikEEPositionConstraint[i]->A_localpos() = gaitParam.eeLocalT[i];
    this->ikEEPositionConstraint[i]->B_link() = nullptr;
    this->ikEEPositionConstraint[i]->eval_link() = actRobotTqc->link(gaitParam.eeParentLink[i]); // local 座標系でerrorやgainを評価
    this->ikEEPositionConstraint[i]->eval_localR() = gaitParam.eeLocalT[i].linear(); // local 座標系でerrorやgainを評価

    if(i < NUM_LEGS &&
       (gaitParam.footstepNodesList[0].isSupportPhase[i] || // 支持脚
        gaitParam.footstepNodesList[0].stopCurrentPosition[i])) { // 早付き
      // 加速させない
      this->ikEEPositionConstraint[i]->pgain().setZero();
      this->ikEEPositionConstraint[i]->B_localvel().setZero();
      this->ikEEPositionConstraint[i]->dgain() = this->ee_support_D[i]; // 着地の瞬間には脚は速度をもっているので、適切に吸収してやらないと外乱として重心に加わってしまう
      this->ikEEPositionConstraint[i]->maxAccByVelError() = 20.0 * cnoid::Vector6::Ones();
      this->ikEEPositionConstraint[i]->ref_acc().setZero();
      this->ikEEPositionConstraint[i]->weight() = 3.0 * cnoid::Vector6::Ones();
      ikConstraint2.push_back(this->ikEEPositionConstraint[i]);
    }else if(i < NUM_LEGS &&
             gaitParam.isManualControlMode[i].getGoal() == 0.0){ // 遊脚
      if(gaitParam.swingState[i] == GaitParam::DOWN_PHASE){
        this->ikEEPositionConstraint[i]->pgain() = this->ee_landing_K[i];
        this->ikEEPositionConstraint[i]->dgain() = this->ee_landing_D[i];
      }else{
        this->ikEEPositionConstraint[i]->pgain() = this->ee_swing_K[i];
        this->ikEEPositionConstraint[i]->dgain() = this->ee_swing_D[i];
      }
      this->ikEEPositionConstraint[i]->B_localpos() = gaitParam.abcEETargetPose[i];
      this->ikEEPositionConstraint[i]->B_localvel() = gaitParam.abcEETargetVel[i];
      this->ikEEPositionConstraint[i]->ref_acc() = gaitParam.abcEETargetAcc[i];
      this->ikEEPositionConstraint[i]->weight() = 1.0 * cnoid::Vector6::Ones();
      ikConstraint2.push_back(this->ikEEPositionConstraint[i]);
    }else{ // maniulation arm/leg
      this->ikEEPositionConstraint[i]->pgain() = this->ee_K[i];
      this->ikEEPositionConstraint[i]->dgain() = this->ee_D[i];
      this->ikEEPositionConstraint[i]->B_localpos() = gaitParam.abcEETargetPose[i];
      this->ikEEPositionConstraint[i]->B_localvel() = gaitParam.abcEETargetVel[i];
      this->ikEEPositionConstraint[i]->ref_acc() = gaitParam.abcEETargetAcc[i];
      this->ikEEPositionConstraint[i]->weight() = 0.3 * cnoid::Vector6::Ones();
      ikConstraint3.push_back(this->ikEEPositionConstraint[i]); // low prioritiy
    }
  }

  {
    // task: COM to target
    this->comConstraint->A_robot() = actRobotTqc;
    this->comConstraint->pgain().setZero(); // footguidedで計算された加速をそのまま使う
    this->comConstraint->dgain().setZero(); // footguidedで計算された加速をそのまま使う
    this->comConstraint->ref_acc() = tgtCogAcc; // footguidedで計算された加速をそのまま使う
    this->comConstraint->weight() << 3.0, 3.0, 0.3; // 0.1, wn=1e-4だと、wnに負けて不正確になる
    ikConstraint2.push_back(this->comConstraint);
  }
  {
    // root
    this->rootPositionConstraint->A_link() = actRobotTqc->rootLink();
    this->rootPositionConstraint->B_link() = nullptr;
    this->rootPositionConstraint->pgain().head<3>().setZero(); // 傾きのみ
    this->rootPositionConstraint->pgain().tail<3>() = this->root_K;
    this->rootPositionConstraint->dgain().head<3>().setZero(); // 傾きのみ
    this->rootPositionConstraint->dgain().tail<3>() = this->root_D;
    this->rootPositionConstraint->B_localpos() = gaitParam.refRobot->rootLink()->T();
    this->rootPositionConstraint->B_localvel().tail<3>() = gaitParam.refRobot->rootLink()->w();
    this->rootPositionConstraint->ref_acc().tail<3>() = gaitParam.refRobot->rootLink()->dw();
    this->rootPositionConstraint->eval_link() = actRobotTqc->rootLink(); // local 座標系でerrorやgainを評価
    this->rootPositionConstraint->weight().head<3>().setZero(); // 傾きのみ
    this->rootPositionConstraint->weight().tail<3>() = 0.3 * cnoid::Vector3::Ones();
    ikConstraint2.push_back(this->rootPositionConstraint);
  }

  std::vector<std::shared_ptr<aik_constraint::IKConstraint> > ikConstraint4;
  {
    // task: angular momentum to zero
    this->angularMomentumConstraint->robot() = actRobotTqc;
    this->angularMomentumConstraint->weight() << 0.1, 0.1, 0.1; // yaw旋回歩行するときのために、Zは小さく. rollとpitchも、joint angle taskの方を優先したほうがよい
    ikConstraint4.push_back(this->angularMomentumConstraint);
  }
  {
    // task: joint angle to target
    for(int i=0;i<actRobotTqc->numJoints();i++){
      if(!gaitParam.jointControllable[i]) continue;
      this->refJointAngleConstraint[i]->joint() = actRobotTqc->joint(i);
      this->refJointAngleConstraint[i]->targetq() = gaitParam.refRobot->joint(i)->q();
      this->refJointAngleConstraint[i]->targetdq() = gaitParam.refRobot->joint(i)->dq();
      this->refJointAngleConstraint[i]->ref_acc() = gaitParam.refRobot->joint(i)->ddq();
      this->refJointAngleConstraint[i]->pgain() = this->joint_K * std::pow(this->dqWeight[i].value(), 2);
      this->refJointAngleConstraint[i]->maxAccByPosError() = 3.0;
      this->refJointAngleConstraint[i]->dgain() = this->joint_D * this->dqWeight[i].value();
      this->refJointAngleConstraint[i]->maxAccByVelError() = 10.0;
      this->refJointAngleConstraint[i]->weight() = 0.3 * this->dqWeight[i].value();
      ikConstraint4.push_back(this->refJointAngleConstraint[i]);
    }
  }

  int debugLevel = 0; // 0 or 1
  std::vector<std::vector<std::shared_ptr<aik_constraint::IKConstraint> > > constraints{ikConstraint0,ikConstraint1,ikConstraint2,ikConstraint3,ikConstraint4};
  for(size_t i=0;i<constraints.size();i++){
    for(size_t j=0;j<constraints[i].size();j++){
      constraints[i][j]->debugLevel() = debugLevel;
    }
  }

  prioritized_acc_inverse_kinematics_solver::IKParam param;
  param.debugLevel = debugLevel;
  param.wn = 1e-4;
  param.we = 1e-6;
  bool solved = prioritized_acc_inverse_kinematics_solver::solveAIK(variables,
                                                                    constraints,
                                                                    tasks,
                                                                    param);
  return true;
}

bool Stabilizer::calcWrench(const GaitParam& gaitParam, const cnoid::Vector3& tgtZmp/*generate座標系*/, bool useActState,
                            std::vector<cnoid::Vector6>& o_tgtEEWrench, cnoid::BodyPtr& actRobotTqc) const{
  if(!useActState) return false;

  // actRobotTqcの重力と加速につりあうトルクを求める
  for(int l=0;l<actRobotTqc->numLinks();l++) actRobotTqc->link(l)->F_ext().setZero();
  actRobotTqc->rootLink()->dv() += cnoid::Vector3(0.0,0.0,gaitParam.g);
  actRobotTqc->calcForwardKinematics(true, true);
  actRobotTqc->calcCenterOfMass();
  cnoid::Vector6 F_o = cnoid::calcInverseDynamics(actRobotTqc->rootLink()); // ルートリンクが受ける外力の和. generate frame. generate frame origin.

  // manipulation arm/legの目標反力
  std::vector<cnoid::Vector6> tgtEEManipWrench(gaitParam.eeName.size(), cnoid::Vector6::Zero()); /* 要素数EndEffector数. generate frame. EndEffector origin*/
  for(int i = 0;i<gaitParam.eeName.size();i++){
    if(i < NUM_LEGS && gaitParam.isManualControlMode[i].getGoal() == 0.0) tgtEEManipWrench[i].setZero(); // 支持脚 or 遊脚. 上位からの目標反力は使わない
    else tgtEEManipWrench[i] = gaitParam.refEEWrench[i]; // manipulation arm/leg. 上位からの目標反力を使う
  }

  cnoid::Vector6 tgtSupWrench_o = F_o; // ルートリンクが支持脚から受ける必要がある外力. generate frame. generate frame origin.
  for(int i = 0;i<gaitParam.eeName.size();i++){
    tgtSupWrench_o.head<3>() -= tgtEEManipWrench[i].head<3>();
    tgtSupWrench_o.tail<3>() -= tgtEEManipWrench[i].tail<3>();
    tgtSupWrench_o.tail<3>() -= gaitParam.actEEPose[i].translation().cross(tgtEEManipWrench[i].tail<3>());
  }

  cnoid::Vector6 tgtSupWrench; // ルートリンクが支持脚から受ける必要がある外力. generate frame. cog origin.
  tgtSupWrench.head<3>() = tgtSupWrench_o.head<3>();
  tgtSupWrench.tail<3>() = tgtSupWrench_o.tail<3>();
  tgtSupWrench.tail<3>() += (- actRobotTqc->centerOfMass()).cross(tgtSupWrench_o.head<3>());

  // トルク制御の目標反力
  std::vector<cnoid::Vector6> tgtEEWrench = tgtEEManipWrench; /* 要素数EndEffector数. generate frame. EndEffector origin*/

  std::vector<int> supportEE;
  if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    supportEE = {RLEG};
  }else if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    supportEE = {LLEG};
  }else if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    // 滞空期
  }else{
    supportEE = {RLEG, LLEG};
  }


  if(supportEE.size()>0){
    /*
      legは、legから受けるwrenchの和がtgtSupWrenchを満たすように.
      各EEFのwrenchを、EEF+copOffset frame/originの6軸表現で考える.

      階層QPのタスクは次の通り
      1. 接触力制約
      2. 和がtgtSupWrench (rot).
      3. 和がtgtSupWrench (trans)
      4. ノルムの2乗和の最小化 (fzは大きくて良い.)

      rotがtransより下の優先度になることにより、擬似的なhip strategyが実現されるようにも思える. しかし、重心位置はFootGuidedControlやmodifyFootStepsで制御できるが、身体の回転を制御する手段は乏しいので、回転しすぎて破綻しないようにrotを優先して満たしたほうが良い
    */

    const int dim = 6 * supportEE.size();
    {
      // 1. 接触力制約
      // 0 <  0  0  1  0  0  0 < 1e10
      // 0 <  1  0 mt  0  0  0 < 1e10
      // 0 < -1  0 mt  0  0  0 < 1e10
      // 0 <  0  1 mt  0  0  0 < 1e10
      // 0 <  0 -1 mt  0  0  0 < 1e10
      // 0 <  0  0  d r1 r2  0 < 1e10 ;; x legHull.size()
      // 0 <  0  0 mr  0  0  1 < 1e10
      // 0 <  0  0 mr  0  0 -1 < 1e10

      this->constraintTask_->A() = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
      this->constraintTask_->b() = Eigen::VectorXd::Zero(0);
      this->constraintTask_->wa() = cnoid::VectorX::Ones(0);

      int constraintDim = 0;
      for(int i=0;i<supportEE.size();i++) constraintDim += 7+gaitParam.legHull[supportEE[i]].size();
      this->constraintTask_->C() = Eigen::SparseMatrix<double,Eigen::RowMajor>(constraintDim,dim);
      this->constraintTask_->dl() = Eigen::VectorXd::Zero(constraintDim);
      this->constraintTask_->du() = 1e10 * Eigen::VectorXd::Ones(constraintDim);
      this->constraintTask_->wc() = cnoid::VectorX::Ones(constraintDim);
      for(int i=0, idx=0;i<supportEE.size();i++){
        int leg = supportEE[i];
        this->constraintTask_->C().insert(idx,i*6+2) = 1.0; this->constraintTask_->dl()[idx] = 50.0; idx++;
        this->constraintTask_->C().insert(idx,i*6+0) = 1.0; this->constraintTask_->C().insert(idx,i*6+2) = gaitParam.muTrans[leg]; idx++;
        this->constraintTask_->C().insert(idx,i*6+0) = -1.0; this->constraintTask_->C().insert(idx,i*6+2) = gaitParam.muTrans[leg]; idx++;
        this->constraintTask_->C().insert(idx,i*6+1) = 1.0; this->constraintTask_->C().insert(idx,i*6+2) = gaitParam.muTrans[leg]; idx++;
        this->constraintTask_->C().insert(idx,i*6+1) = -1.0; this->constraintTask_->C().insert(idx,i*6+2) = gaitParam.muTrans[leg]; idx++;
        for(int j=0;j<gaitParam.legHull[leg].size();j++){
          cnoid::Vector3 v1 = gaitParam.legHull[leg][j] - gaitParam.copOffset[leg].value(); // EEF+copOffset frame/origin
          cnoid::Vector3 v2 = gaitParam.legHull[leg][(j+1<gaitParam.legHull[leg].size())?j+1:0] - gaitParam.copOffset[leg].value(); // EEF+copOffset frame/origin
          if(v1.head<2>() == v2.head<2>()) continue;
          cnoid::Vector3 r = cnoid::Vector3(v2[1]-v1[1],v1[0]-v2[0],0).normalized();
          double d = r.dot(v1);
          this->constraintTask_->C().insert(idx,i*6+2) = d; this->constraintTask_->C().insert(idx,i*6+3) = -r[1]; this->constraintTask_->C().insert(idx,i*6+4) = r[0]; idx++;
        }
        this->constraintTask_->C().insert(idx,i*6+5) = 1.0; this->constraintTask_->C().insert(idx,2) = gaitParam.muRot[leg]; idx++;
        this->constraintTask_->C().insert(idx,i*6+5) = -1.0; this->constraintTask_->C().insert(idx,2) = gaitParam.muRot[leg]; idx++;
      }

      this->constraintTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->constraintTask_->toSolve() = false;
      this->constraintTask_->settings().verbose = 0;
    }

    {
      // 2. 和がtgtSupWrench (rot)
      Eigen::SparseMatrix<double,Eigen::ColMajor> A_colMajor(3,dim); // insertする順番がcolMajorなので
      for(int i=0;i<supportEE.size();i++){
        int leg = supportEE[i];
        cnoid::Position eePose = gaitParam.actEEPose[leg]; eePose.translation() += eePose.linear() * gaitParam.copOffset[leg].value();
        cnoid::Matrix3 eeR = eePose.linear();
        cnoid::Matrix3 eepCross = mathutil::cross(eePose.translation() - actRobotTqc->centerOfMass()) * eeR;
        for(int j=0;j<3;j++) {
          for(int k=0;k<3;k++) A_colMajor.insert(k,i*6+j) = eepCross(k,j);
        }
        for(int j=0;j<3;j++) {
          for(int k=0;k<3;k++) A_colMajor.insert(k,i*6+3+j) = eeR(k,j);
        }
      }
      this->tgtTorqueTask_->A() = A_colMajor;
      this->tgtTorqueTask_->b() = tgtSupWrench.tail<3>();
      this->tgtTorqueTask_->wa() = cnoid::VectorX::Ones(3);

      this->tgtTorqueTask_->C() = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
      this->tgtTorqueTask_->dl() = Eigen::VectorXd::Zero(0);
      this->tgtTorqueTask_->du() = Eigen::VectorXd::Ones(0);
      this->tgtTorqueTask_->wc() = cnoid::VectorX::Ones(0);

      this->tgtTorqueTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->tgtTorqueTask_->toSolve() = true;
      this->tgtTorqueTask_->settings().check_termination = 5; // default 25. 高速化
      this->tgtTorqueTask_->settings().verbose = 0;
    }

    {
      // 3. 和がtgtSupWrench (trans)
      Eigen::SparseMatrix<double,Eigen::ColMajor> A_colMajor(3,dim); // insertする順番がcolMajorなので
      for(int i=0;i<supportEE.size();i++){
        int leg = supportEE[i];
        cnoid::Matrix3 eeR = gaitParam.actEEPose[leg].linear();
        for(int j=0;j<3;j++) {
          for(int k=0;k<3;k++) A_colMajor.insert(k,i*6+j) = eeR(k,j);
        }
      }
      this->tgtForceTask_->A() = A_colMajor;
      this->tgtForceTask_->b() = tgtSupWrench.head<3>();
      this->tgtForceTask_->wa() = cnoid::VectorX::Ones(3);

      this->tgtForceTask_->C() = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
      this->tgtForceTask_->dl() = Eigen::VectorXd::Zero(0);
      this->tgtForceTask_->du() = Eigen::VectorXd::Ones(0);
      this->tgtForceTask_->wc() = cnoid::VectorX::Ones(0);

      this->tgtForceTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->tgtForceTask_->toSolve() = true;
      this->tgtForceTask_->settings().check_termination = 5; // default 25. 高速化
      this->tgtForceTask_->settings().verbose = 0;
    }

    {
      // 4. ノルムの2乗和の最小化
      this->normTask_->A() = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
      this->normTask_->b() = Eigen::VectorXd::Zero(0);
      this->normTask_->wa() = cnoid::VectorX::Ones(0);

      this->normTask_->C() = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
      this->normTask_->dl() = Eigen::VectorXd::Zero(0);
      this->normTask_->du() = Eigen::VectorXd::Ones(0);
      this->normTask_->wc() = cnoid::VectorX::Ones(0);

      this->normTask_->w() = cnoid::VectorX::Ones(dim);
      for(int i=0;i<supportEE.size();i++){
        int leg = supportEE[i];
        this->normTask_->w()[i*6+0] = std::pow(1e2, 2.0);
        this->normTask_->w()[i*6+1] = std::pow(1e2, 2.0);
        this->normTask_->w()[i*6+2] = std::pow(1e0, 2.0);
        this->normTask_->w()[i*6+3] = std::pow(1e2, 2.0);
        this->normTask_->w()[i*6+4] = std::pow(1e2, 2.0);
        this->normTask_->w()[i*6+5] = std::pow(1e3, 2.0);
      }

      this->normTask_->toSolve() = true;
      this->normTask_->settings().check_termination = 5; // default 25. 高速化
      this->normTask_->settings().verbose = 0;
    }

    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks{this->constraintTask_,this->tgtTorqueTask_,this->tgtForceTask_,this->normTask_};
    cnoid::VectorX result; // EEF+copOffset frame/origin
    if(prioritized_qp_base::solve(tasks,
                                   result,
                                   0 // debuglevel
                                   )){
      for(int i=0;i<supportEE.size();i++){
        int leg = supportEE[i];
        cnoid::Vector6 w = result.segment<6>(i*6); // EEF+copOffset frame/origin
        tgtEEWrench[leg].head<3>() += gaitParam.actEEPose[leg].linear() * w.head<3>();
        tgtEEWrench[leg].tail<3>() += gaitParam.actEEPose[leg].linear() * w.tail<3>();
        tgtEEWrench[leg].tail<3>() += (gaitParam.actEEPose[leg].linear() * gaitParam.copOffset[leg].value()).cross(gaitParam.actEEPose[leg].linear() * w.head<3>());
      }
    }
  }

  // エンドエフェクタ力を関節トルクに変換
  for(int i=0;i<gaitParam.eeName.size();i++){
    cnoid::JointPath jointPath(actRobotTqc->rootLink(), actRobotTqc->link(gaitParam.eeParentLink[i]));
    cnoid::MatrixXd J = cnoid::MatrixXd::Zero(6,jointPath.numJoints()); // generate frame. endeffector origin
    cnoid::setJacobian<0x3f,0,0,true>(jointPath,actRobotTqc->link(gaitParam.eeParentLink[i]),gaitParam.eeLocalT[i].translation(), // input
                                      J); // output
    cnoid::VectorX tau = - J.transpose() * tgtEEWrench[i];
    for(int j=0;j<jointPath.numJoints();j++){
      jointPath.joint(j)->u() += tau[j];
    }
  }

  o_tgtEEWrench = tgtEEWrench;
  return true;
}
