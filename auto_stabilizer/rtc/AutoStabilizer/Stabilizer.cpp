#include "Stabilizer.h"
#include "MathUtil.h"
#include <cnoid/Jacobian>
#include <cnoid/EigenUtil>
#include <cnoid/src/Body/InverseDynamics.h>

bool Stabilizer::execStabilizer(const GaitParam& gaitParam, double dt, bool useActState,
                                GaitParam::DebugData& debugData, //for Log
                                cnoid::BodyPtr& actRobotTqc, cnoid::BodyPtr& genRobot, cnoid::Vector3& o_genNextCog, cnoid::Vector3& o_genNextCogVel, cnoid::Vector3& o_genNextCogAcc) const{
  /* useActStateがtrueなら、
       - actRobotTqcはフィードバックによって次の周期の目標加速度・目標トルクが入る
       - genRobotはactRobotの値に緩やかに従う
       - 位置制御ゲインを0へ.
     useActStateがfalseなら、
       - actRobotTqcは0が入る
       - genRobotは逆運動学によってフィードフォワードに動く
       - 位置制御ゲインを100へ.
   */

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
    - FootGuidedControlで、支持領域リミットによって 角運動量の変化(=0) > FootGuidedControlの結果の重心の加速 の優先度でリミット.
    - acceleration based ik で、FootGuidedControlの結果の重心の目標加速(特にXY) > 腕などのエンドエフェクタの目標位置 > FootGuidedControlの結果の角運動量の目標変化(=0) の優先度で解く
    - wrench distributionで、acceleration based ikの結果の角運動量の変化 > acceleration based ikの結果の重心の加速 の優先度で解く.
  */

  // masterのhrpsysではSwingEEModification(位置制御)を行っていたが、地面についているときに、time_constでもとの位置に戻るまでの間、足と重心の相対位置が着地位置タイミング修正計算で用いるものとずれることによる着地位置修正パフォーマンスの低下のデメリットの方が大きいので、削除した
  // masterのhrpsysやもとのauto_stabilizerでは位置制御+DampingControlをサポートしていたが、位置制御+DampingControlは実機での目標接触力への追従に遅れがある. FootGuidedControlでは、目標ZMPの位相を進めたり、ZMPの追従遅れを考慮した目標ZMP計算を行ったりをしていないので、遅れに弱い. そのため、位置制御+DampingControlは削除し、所謂TorqueControlのみをサポートしている.


  // useActState==true: genCogから目標重心Accを計算し、genCogを進める. actCogから目標重心Accを計算する.
  // useActState==false: genCogから目標重心Accを計算し、genCogを進める.
  cnoid::Vector3 tgtActCogAcc; // generate frame
  cnoid::Vector3 genNextCog,genNextCogVel,genNextCogAcc,genNextForce;
  this->calcCogAcc(gaitParam, dt, useActState, // input
                   debugData, // debug
                   tgtActCogAcc,genNextCog,genNextCogVel,genNextCogAcc,genNextForce); // output

  // actRobotTqcのq,dqにactualの値を入れ、
  // useActState==true: actRobotTqcのddqに今回の目標値を求めて入れる. genRobotのqはactRobotの値に緩やかに従う
  // useActState==false: actRobotTqcのddqはゼロ. genRobotのqはフィードフォワードにIKを解く.
  this->calcResolvedAccelerationControl(gaitParam, dt, tgtActCogAcc, genNextCog, useActState,
                                        actRobotTqc, genRobot);

  // useActState==true: actRobotTqcの自重と加速に要する力と、manipulation arm/legのrefForceに釣り合うように、目標支持脚反力を計算. actRobotTqcのuを求める.
  // useActState==false: genRobotの自重と加速に要する力と、manipulation arm/legのrefForceに釣り合うように、目標支持脚反力を計算. actRobotTqcのuは0.
  this->calcWrench(gaitParam, genNextForce, useActState,// input
                   debugData.stEETargetWrench, actRobotTqc); // output

  o_genNextCog = genNextCog;
  o_genNextCogVel = genNextCogVel;
  o_genNextCogAcc = genNextCogAcc;

  return true;
}

bool Stabilizer::calcCogAcc(const GaitParam& gaitParam, double dt, bool useActState,
                            GaitParam::DebugData& debugData, //for Log
                            cnoid::Vector3& o_tgtActCogAcc, cnoid::Vector3& o_genNextCog, cnoid::Vector3& o_genNextCogVel, cnoid::Vector3& o_genNextCogAcc, cnoid::Vector3& o_genNextForce) const{

  // genCog (mode_st中も計算する. refToGenFrameConverterで使うので)
  cnoid::Vector3 genZmp;
  this->calcZmp(gaitParam, gaitParam.genCog, gaitParam.genCog + gaitParam.genCogVel / gaitParam.omega, gaitParam.abcEETargetPose, true,
                genZmp);
  cnoid::Vector3 genNextCog,genNextCogVel,genNextCogAcc,genNextForce;
  footguidedcontroller::updateState(gaitParam.omega,gaitParam.l,gaitParam.genCog,gaitParam.genCogVel,genZmp,gaitParam.genRobot->mass(),dt,
                                    genNextCog, genNextCogVel, genNextCogAcc, genNextForce);

  // actCog
  cnoid::Vector3 tgtActZmp;
  this->calcZmp(gaitParam, gaitParam.actCog, gaitParam.actCog + gaitParam.actCogVel.value() / gaitParam.omega, gaitParam.actEEPose, false,
                tgtActZmp);
  cnoid::Vector3 tgtActCog,tgtActCogVel,tgtActCogAcc,tgtActForce;
  footguidedcontroller::updateState(gaitParam.omega,gaitParam.l,gaitParam.actCog,gaitParam.actCogVel.value(),tgtActZmp,gaitParam.genRobot->mass(),dt,
                                    tgtActCog, tgtActCogVel, tgtActCogAcc, tgtActForce);

  o_tgtActCogAcc = tgtActCogAcc;
  o_genNextCog = genNextCog;
  o_genNextCogVel = genNextCogVel;
  o_genNextCogAcc = genNextCogAcc;
  o_genNextForce = genNextForce;

  debugData.stTargetZmp = tgtActZmp;
  return true;
}

bool Stabilizer::calcZmp(const GaitParam& gaitParam, const cnoid::Vector3& cog, const cnoid::Vector3& DCM, const std::vector<cnoid::Position>& EEPose, const bool& useSoftLimit,
                         cnoid::Vector3& o_zmp) const{
  cnoid::Vector3 tgtZmp;
  if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] || gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    tgtZmp = footguidedcontroller::calcFootGuidedControl(gaitParam.omega,gaitParam.l,DCM,gaitParam.refZmpTraj);
    if(tgtZmp[2] >= cog[2]) tgtZmp = cog - cnoid::Vector3(gaitParam.l[0],gaitParam.l[1], 0.0); // 下向きの力は受けられないので
    else{
      cnoid::Vector3 tgtZmpOrg = tgtZmp;
      // truncate zmp inside polygon. actual robotの関節角度を用いて計算する
      std::vector<cnoid::Vector3> vertices; // generate frame. 支持点の集合
      for(int i=0;i<NUM_LEGS;i++){
        if(!gaitParam.footstepNodesList[0].isSupportPhase[i]) continue;
        for(int j=0;j<gaitParam.legHull[i].size();j++){
          cnoid::Vector3 p = EEPose[i]*gaitParam.legHull[i][j];
          if(p[2] > cog[2] - 1e-2) p[2] = cog[2] - 1e-2; // 重心よりも支持点が高いと射影が破綻するので
          vertices.push_back(p);
        }
      }
      tgtZmp = mathutil::calcInsidePointOfPolygon3D(tgtZmp,vertices,cog - cnoid::Vector3(gaitParam.l[0],gaitParam.l[1], 0.0));

      if(useSoftLimit){
        //zmpがpolygon外に出たとしてもcogを進行方向に少しでもいいから動かす. そうしないとcogが無限遠に発散する恐れあり.
        for(int i=0;i<2;i++){
          if((tgtZmpOrg[i]-cog[i]+gaitParam.l[i]) > 0.001){
            if((tgtZmp[i]-cog[i]+gaitParam.l[i]) < 0.001) tgtZmp[i] = cog[i] - gaitParam.l[i] + 0.001;
          }else if((tgtZmpOrg[i]-cog[i]+gaitParam.l[i]) < -0.001){
            if((tgtZmp[i]-cog[i]+gaitParam.l[i]) > -0.001) tgtZmp[i] = cog[i] - gaitParam.l[i] - 0.001;
          }
        }
      }
      // TODO. 角運動量オフセット.
    }
  }else{ // 跳躍期
    tgtZmp = cog - cnoid::Vector3(gaitParam.l[0],gaitParam.l[1], 0.0);
  }
  o_zmp = tgtZmp;
  return true;
}

bool Stabilizer::calcResolvedAccelerationControl(const GaitParam& gaitParam, double dt, const cnoid::Vector3& tgtCogAcc/*generate座標系*/, const cnoid::Vector3& genNextCog, bool useActState,
                                                 cnoid::BodyPtr& actRobotTqc, cnoid::BodyPtr& genRobot) const{
  // actRobotTqcのq,dqにactualの値を入れる
  {
    actRobotTqc->rootLink()->T() = gaitParam.actRobot->rootLink()->T();
    actRobotTqc->rootLink()->v() = gaitParam.actRobot->rootLink()->v();
    actRobotTqc->rootLink()->w() = gaitParam.actRobot->rootLink()->w();
    actRobotTqc->rootLink()->dv().setZero();
    actRobotTqc->rootLink()->dw().setZero();
    for(int i=0;i<actRobotTqc->numJoints();i++){
      actRobotTqc->joint(i)->q() = gaitParam.actRobot->joint(i)->q();
      actRobotTqc->joint(i)->dq() = gaitParam.actRobot->joint(i)->dq();
      actRobotTqc->joint(i)->ddq() = 0.0;
      actRobotTqc->joint(i)->u() = 0.0;
    }
    for(int l=0;l<actRobotTqc->numLinks();l++) actRobotTqc->link(l)->F_ext().setZero();
    actRobotTqc->calcForwardKinematics(true,true);
    actRobotTqc->calcCenterOfMass();
  }

  if(useActState){ // useActState==true: actRobotTqcのddqに今回の目標値を求めて入れる. genRobotのqはactRobotの値に緩やかに従う
    /*
      actRobotTqc
    */

    {
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
      this->aikJointLimitConstraint[i]->joint() = actRobotTqc->joint(i);
      this->aikJointLimitConstraint[i]->jointLimitTables() = gaitParam.jointLimitTablesTqc[i];
      this->aikJointLimitConstraint[i]->pgain() = 400;
      this->aikJointLimitConstraint[i]->dgain() = 100;
      this->aikJointLimitConstraint[i]->maxAccByVelError() = 20.0;
      this->aikJointLimitConstraint[i]->weight() = 0.1;
      ikConstraint0.push_back(this->aikJointLimitConstraint[i]);
    }

    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > ikConstraint1;
    // self Collision TODO

    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > ikConstraint2;
    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > ikConstraint3;

    for(int i=0;i<gaitParam.eeName.size();i++){
      this->aikEEPositionConstraint[i]->A_link() = actRobotTqc->link(gaitParam.eeParentLink[i]);
      this->aikEEPositionConstraint[i]->A_localpos() = gaitParam.eeLocalT[i];
      this->aikEEPositionConstraint[i]->B_link() = nullptr;
      this->aikEEPositionConstraint[i]->eval_link() = actRobotTqc->link(gaitParam.eeParentLink[i]); // local 座標系でerrorやgainを評価
      this->aikEEPositionConstraint[i]->eval_localR() = gaitParam.eeLocalT[i].linear(); // local 座標系でerrorやgainを評価

      if(i < NUM_LEGS &&
         (gaitParam.footstepNodesList[0].isSupportPhase[i] || // 支持脚
          gaitParam.footstepNodesList[0].stopCurrentPosition[i])) { // 早付き
        // 加速させない
        this->aikEEPositionConstraint[i]->pgain().setZero();
        this->aikEEPositionConstraint[i]->B_localvel().setZero();
        this->aikEEPositionConstraint[i]->dgain() = this->ee_support_D[i];
        this->aikEEPositionConstraint[i]->ref_acc().setZero();
        this->aikEEPositionConstraint[i]->weight() = 3.0 * cnoid::Vector6::Ones();
        ikConstraint2.push_back(this->aikEEPositionConstraint[i]);
      }else if(i < NUM_LEGS &&
               gaitParam.isManualControlMode[i].getGoal() == 0.0){ // 遊脚
        if(gaitParam.swingState[i] == GaitParam::DOWN_PHASE){
          this->aikEEPositionConstraint[i]->pgain() = this->ee_landing_K[i];
          this->aikEEPositionConstraint[i]->dgain() = this->ee_landing_D[i];
        }else{
          this->aikEEPositionConstraint[i]->pgain() = this->ee_swing_K[i];
          this->aikEEPositionConstraint[i]->dgain() = this->ee_swing_D[i];
        }
        this->aikEEPositionConstraint[i]->B_localpos() = gaitParam.abcEETargetPose[i];
        this->aikEEPositionConstraint[i]->B_localvel() = gaitParam.abcEETargetVel[i];
        this->aikEEPositionConstraint[i]->ref_acc() = gaitParam.abcEETargetAcc[i];
        this->aikEEPositionConstraint[i]->weight() = 1.0 * cnoid::Vector6::Ones();
        ikConstraint2.push_back(this->aikEEPositionConstraint[i]);
      }else{ // maniulation arm/leg
        this->aikEEPositionConstraint[i]->pgain() = this->ee_K[i];
        this->aikEEPositionConstraint[i]->dgain() = this->ee_D[i];
        this->aikEEPositionConstraint[i]->B_localpos() = gaitParam.abcEETargetPose[i];
        this->aikEEPositionConstraint[i]->B_localvel() = gaitParam.abcEETargetVel[i];
        this->aikEEPositionConstraint[i]->ref_acc() = gaitParam.abcEETargetAcc[i];
        this->aikEEPositionConstraint[i]->weight() = 0.3 * cnoid::Vector6::Ones();
        ikConstraint3.push_back(this->aikEEPositionConstraint[i]); // low prioritiy
      }
    }

    {
      // task: COM to target
      this->aikComConstraint->A_robot() = actRobotTqc;
      this->aikComConstraint->pgain().setZero(); // footguidedで計算された加速をそのまま使う
      this->aikComConstraint->dgain().setZero(); // footguidedで計算された加速をそのまま使う
      this->aikComConstraint->ref_acc() = tgtCogAcc; // footguidedで計算された加速をそのまま使う
      this->aikComConstraint->weight() << 3.0, 3.0, 0.3; // 0.1, wn=1e-4だと、wnに負けて不正確になる
      ikConstraint2.push_back(this->aikComConstraint);
    }
    {
      // root
      this->aikRootPositionConstraint->A_link() = actRobotTqc->rootLink();
      this->aikRootPositionConstraint->B_link() = nullptr;
      this->aikRootPositionConstraint->pgain().head<3>().setZero(); // 傾きのみ
      this->aikRootPositionConstraint->pgain().tail<3>() = this->root_K;
      this->aikRootPositionConstraint->dgain().head<3>().setZero(); // 傾きのみ
      this->aikRootPositionConstraint->dgain().tail<3>() = this->root_D;
      this->aikRootPositionConstraint->B_localpos() = gaitParam.refRobot->rootLink()->T();
      this->aikRootPositionConstraint->B_localvel().tail<3>() = gaitParam.refRobot->rootLink()->w();
      this->aikRootPositionConstraint->ref_acc().tail<3>() = gaitParam.refRobot->rootLink()->dw();
      this->aikRootPositionConstraint->eval_link() = actRobotTqc->rootLink(); // local 座標系でerrorやgainを評価
      this->aikRootPositionConstraint->weight().head<3>().setZero(); // 傾きのみ
      this->aikRootPositionConstraint->weight().tail<3>() = 0.3 * cnoid::Vector3::Ones();
      ikConstraint2.push_back(this->aikRootPositionConstraint);
    }

    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > ikConstraint4;
    {
      // task: angular momentum to zero
      this->aikAngularMomentumConstraint->robot() = actRobotTqc;
      this->aikAngularMomentumConstraint->weight() << 0.1, 0.1, 0.1; // yaw旋回歩行するときのために、Zは小さく. rollとpitchも、joint angle taskの方を優先したほうがよい
      ikConstraint4.push_back(this->aikAngularMomentumConstraint);
    }
    {
      // task: joint angle to target
      for(int i=0;i<actRobotTqc->numJoints();i++){
        if(!gaitParam.jointControllable[i]) continue;
        this->aikRefJointAngleConstraint[i]->joint() = actRobotTqc->joint(i);
        this->aikRefJointAngleConstraint[i]->targetq() = gaitParam.refRobot->joint(i)->q();
        this->aikRefJointAngleConstraint[i]->targetdq() = gaitParam.refRobot->joint(i)->dq();
        this->aikRefJointAngleConstraint[i]->ref_acc() = gaitParam.refRobot->joint(i)->ddq();
        this->aikRefJointAngleConstraint[i]->pgain() = this->joint_K * std::pow(this->aikdqWeight[i].value(), 2);
        this->aikRefJointAngleConstraint[i]->maxAccByPosError() = 3.0;
        this->aikRefJointAngleConstraint[i]->dgain() = this->joint_D * this->aikdqWeight[i].value();
        this->aikRefJointAngleConstraint[i]->maxAccByVelError() = 10.0;
        this->aikRefJointAngleConstraint[i]->weight() = 0.3 * this->aikdqWeight[i].value();
        ikConstraint4.push_back(this->aikRefJointAngleConstraint[i]);
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
                                                                      aikTasks,
                                                                      param);

    /*
      genRobot
    */
    this->commandRootPoseFilter.setGoal(actRobotTqc->rootLink()->T(), 0.3); // 0.3秒で補間
    this->commandRootPoseFilter.interpolate(dt);
    genRobot->rootLink()->T() = this->commandRootPoseFilter.value();
    for(int i=0;i<genRobot->numJoints();i++){
      if(gaitParam.jointControllable[i]){
        this->commandJointAngleFilter[i].setGoal(actRobotTqc->joint(i)->q(), 0.3); // 0.3秒で補間
        this->commandJointAngleFilter[i].interpolate(dt);

        // limit check
        cnoid::LinkPtr joint = genRobot->joint(i);
        double u = gaitParam.genRobot->joint(i)->q_upper();
        double l = gaitParam.genRobot->joint(i)->q_lower();
        for(int j=0;j<gaitParam.jointLimitTables[i].size();j++){
          u = std::min(u,gaitParam.jointLimitTables[i][j]->getUlimit());
          l = std::max(l,gaitParam.jointLimitTables[i][j]->getLlimit());
        }
        joint->q() = std::min(u, std::max(l, this->commandJointAngleFilter[i].value()));
      }else{
        // !jointControllableの関節は指令値をそのまま入れる
        genRobot->joint(i)->q() = gaitParam.refRobot->joint(i)->q();
      }
    }
    genRobot->calcForwardKinematics();



  }else{ // useActState==false: actRobotTqcのddqはゼロ. genRobotのqはフィードフォワードにIKを解く.
    /*
      actRobotTqc
     */
    // 何もしない(ddqとuはゼロのまま)

    /*
      genRobot
    */
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
        dqWeight.push_back(this->ikdqWeight[i].value());
      }
    }

    std::vector<std::shared_ptr<IK::IKConstraint> > ikConstraint0;

    // joint velocity
    for(size_t i=0;i<genRobot->numJoints();i++){
      if(!gaitParam.jointControllable[i]) continue;
      this->ikJointVelocityConstraint[i]->joint() = genRobot->joint(i);
      this->ikJointVelocityConstraint[i]->dt() = dt;
      this->ikJointVelocityConstraint[i]->maxError() = 1.0 * dt;
      this->ikJointVelocityConstraint[i]->weight() = 1.0;
      ikConstraint0.push_back(this->ikJointVelocityConstraint[i]);
    }

    // joint angle
    for(size_t i=0;i<genRobot->numJoints();i++){
      if(!gaitParam.jointControllable[i]) continue;
      this->ikJointLimitConstraint[i]->joint() = genRobot->joint(i);
      this->ikJointLimitConstraint[i]->jointLimitTables() = gaitParam.jointLimitTables[i];
      this->ikJointLimitConstraint[i]->maxError() = 1.0 * dt;
      this->ikJointLimitConstraint[i]->weight() = 1.0;
      ikConstraint0.push_back(this->ikJointLimitConstraint[i]);
    }

    std::vector<std::shared_ptr<IK::IKConstraint> > ikConstraint1;
    this->ikSelfCollisionConstraint.resize(gaitParam.selfCollision.size());
    for(size_t i=0;i<this->ikSelfCollisionConstraint.size();i++){
      if(!this->ikSelfCollisionConstraint[i]) this->ikSelfCollisionConstraint[i] = std::make_shared<IK::ClientCollisionConstraint>();
      this->ikSelfCollisionConstraint[i]->A_link() = genRobot->link(gaitParam.selfCollision[i].link1);
      this->ikSelfCollisionConstraint[i]->B_link() = genRobot->link(gaitParam.selfCollision[i].link2);
      this->ikSelfCollisionConstraint[i]->tolerance() = 0.01;
      this->ikSelfCollisionConstraint[i]->maxError() = 10.0*dt;
      this->ikSelfCollisionConstraint[i]->weight() = 1.0;
      this->ikSelfCollisionConstraint[i]->velocityDamper() = 0.1 / dt;
      this->ikSelfCollisionConstraint[i]->A_localp() = gaitParam.selfCollision[i].point1;
      this->ikSelfCollisionConstraint[i]->B_localp() = gaitParam.selfCollision[i].point2;
      this->ikSelfCollisionConstraint[i]->direction() = gaitParam.selfCollision[i].direction21;

      // 全自己干渉情報を与えると計算コストが膨大になるため、距離が近いもののみ与える
      if(gaitParam.selfCollision[i].distance < 0.05){
        ikConstraint1.push_back(this->ikSelfCollisionConstraint[i]);
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
      this->ikComConstraint->A_robot() = genRobot;
      this->ikComConstraint->A_localp() = cnoid::Vector3::Zero();
      this->ikComConstraint->B_robot() = nullptr;
      this->ikComConstraint->B_localp() = genNextCog;
      this->ikComConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt;
      this->ikComConstraint->precision() << 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
      this->ikComConstraint->weight() << 10.0, 10.0, 1.0;
      this->ikComConstraint->eval_R() = cnoid::Matrix3::Identity();
      ikConstraint2.push_back(this->ikComConstraint);
    }

    // Angular Momentum
    {
      this->ikAngularMomentumConstraint->robot() = genRobot;
      this->ikAngularMomentumConstraint->targetAngularMomentum() = cnoid::Vector3::Zero(); // TODO
      this->ikAngularMomentumConstraint->maxError() << 1.0*dt, 1.0*dt, 1.0*dt;
      this->ikAngularMomentumConstraint->precision() << 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
      this->ikAngularMomentumConstraint->weight() << 1e-4, 1e-4, 0.0; // TODO
      this->ikAngularMomentumConstraint->dt() = dt;
      this->ikAngularMomentumConstraint->eval_R() = cnoid::Matrix3::Identity();
      ikConstraint2.push_back(this->ikAngularMomentumConstraint);
    }

    // root
    {
      this->ikRootPositionConstraint->A_link() = genRobot->rootLink();
      this->ikRootPositionConstraint->A_localpos() = cnoid::Position::Identity();
      this->ikRootPositionConstraint->B_link() = nullptr;
      this->ikRootPositionConstraint->B_localpos() = gaitParam.refRobot->rootLink()->T();
      this->ikRootPositionConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt;
      this->ikRootPositionConstraint->precision() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // 強制的にIKをmax loopまで回す
      this->ikRootPositionConstraint->weight() << 0.0, 0.0, 0.0, 3.0, 3.0, 3.0; // 角運動量を利用するときは重みを小さく. 通常時、胴の質量・イナーシャやマスパラ誤差の大きさや、胴を大きく動かすための出力不足などによって、二足動歩行では胴の傾きの自由度を使わない方がよい
      //this->ikRootPositionConstraint->weight() << 0.0, 0.0, 0.0, 3e-1, 3e-1, 3e-1;
      this->ikRootPositionConstraint->eval_link() = nullptr;
      this->ikRootPositionConstraint->eval_localR() = cnoid::Matrix3::Identity();
      ikConstraint2.push_back(this->ikRootPositionConstraint);
    }

    // reference angle
    {
      for(size_t i=0;i<genRobot->numJoints();i++){
        if(!gaitParam.jointControllable[i]) continue;
        this->ikRefJointAngleConstraint[i]->joint() = genRobot->joint(i);
        this->ikRefJointAngleConstraint[i]->maxError() = 10.0 * dt; // 高優先度のmaxError以下にしないと優先度逆転するおそれ
        this->ikRefJointAngleConstraint[i]->weight() = 1e-1; // 小さい値すぎると、qp終了判定のtoleranceによって無視されてしまう
        this->ikRefJointAngleConstraint[i]->targetq() = gaitParam.refRobot->joint(i)->q();
        this->ikRefJointAngleConstraint[i]->precision() = 0.0; // 強制的にIKをmax loopまで回す
        ikConstraint2.push_back(this->ikRefJointAngleConstraint[i]);
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
                                                       this->ikTasks,
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

    this->commandRootPoseFilter.reset(genRobot->rootLink()->T());
    for(int i=0;i<genRobot->numJoints();i++){
      this->commandJointAngleFilter[i].reset(genRobot->joint(i)->q());
    }

  }
  return true;
}

bool Stabilizer::calcWrench(const GaitParam& gaitParam, const cnoid::Vector3& genNextForce, bool useActState,
                            std::vector<cnoid::Vector6>& o_tgtEEWrench, cnoid::BodyPtr& actRobotTqc) const{

  cnoid::Vector6 tgtSupWrench = cnoid::Vector6::Zero(); // ルートリンクが支持脚から受ける必要がある外力. generate frame. cog origin.
  // manipulation arm/legの目標反力
  std::vector<cnoid::Vector6> tgtEEManipWrench(gaitParam.eeName.size(), cnoid::Vector6::Zero()); /* 要素数EndEffector数. generate frame. EndEffector origin*/
  for(int i = 0;i<gaitParam.eeName.size();i++){
    if(i < NUM_LEGS && gaitParam.isManualControlMode[i].getGoal() == 0.0) tgtEEManipWrench[i].setZero(); // 支持脚 or 遊脚. 上位からの目標反力は使わない
    else tgtEEManipWrench[i] = gaitParam.refEEWrench[i]; // manipulation arm/leg. 上位からの目標反力を使う
  }
  if(useActState) {
    // actRobotTqcの重力と加速につりあうトルクを求める
    for(int l=0;l<actRobotTqc->numLinks();l++) actRobotTqc->link(l)->F_ext().setZero();
    actRobotTqc->rootLink()->dv() += cnoid::Vector3(0.0,0.0,gaitParam.g);
    actRobotTqc->calcForwardKinematics(true, true);
    actRobotTqc->calcCenterOfMass();
    cnoid::Vector6 F_o = cnoid::calcInverseDynamics(actRobotTqc->rootLink()); // ルートリンクが受ける外力の和. generate frame. generate frame origin.

    cnoid::Vector6 tgtSupWrench_o = F_o; // ルートリンクが支持脚から受ける必要がある外力. generate frame. generate frame origin.
    for(int i = 0;i<gaitParam.eeName.size();i++){
      tgtSupWrench_o.head<3>() -= tgtEEManipWrench[i].head<3>();
      tgtSupWrench_o.tail<3>() -= tgtEEManipWrench[i].tail<3>();
      tgtSupWrench_o.tail<3>() -= gaitParam.actEEPose[i].translation().cross(tgtEEManipWrench[i].tail<3>());
    }

    tgtSupWrench.head<3>() = tgtSupWrench_o.head<3>();
    tgtSupWrench.tail<3>() = tgtSupWrench_o.tail<3>();
    tgtSupWrench.tail<3>() += (- actRobotTqc->centerOfMass()).cross(tgtSupWrench_o.head<3>());

  }else{ // useActState==false
    tgtSupWrench.head<3>() = genNextForce;
    for(int i = 0;i<gaitParam.eeName.size();i++){
      tgtSupWrench.head<3>() -= tgtEEManipWrench[i].head<3>();
      tgtSupWrench.tail<3>() -= tgtEEManipWrench[i].tail<3>();
      tgtSupWrench.tail<3>() -= (gaitParam.abcEETargetPose[i].translation()-gaitParam.genCog).cross(tgtEEManipWrench[i].tail<3>());
    }
  }

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
        cnoid::Position eePose = useActState?gaitParam.actEEPose[leg]:gaitParam.abcEETargetPose[i]; eePose.translation() += eePose.linear() * gaitParam.copOffset[leg].value();
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
        cnoid::Matrix3 eeR = useActState?gaitParam.actEEPose[leg].linear():gaitParam.abcEETargetPose[i].linear();
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
