#include "Stabilizer.h"
#include "MathUtil.h"
#include <cnoid/Jacobian>
#include <cnoid/EigenUtil>
#include <cnoid/src/Body/InverseDynamics.h>

void Stabilizer::initStabilizerOutput(const GaitParam& gaitParam,
                                      cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy, cnoid::Vector3& o_stTargetZmp, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPGainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDGainPercentage) const{

  o_stOffsetRootRpy.reset(cnoid::Vector3::Zero());
  o_stTargetZmp = gaitParam.refZmpTraj[0].getStart();
  for(int i=0;i<o_stServoPGainPercentage.size();i++){
    o_stServoPGainPercentage[i].reset(100.0);
    o_stServoDGainPercentage[i].reset(100.0);
  }
}

bool Stabilizer::execStabilizer(const GaitParam& gaitParam, double dt, bool useActState,
                                cnoid::BodyPtr& actRobotTqc, cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy, cnoid::Position& o_stTargetRootPose, cnoid::Vector3& o_stTargetZmp, std::vector<cnoid::Vector6>& o_stEETargetWrench, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPGainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDGainPercentage) const{
  // - root attitude control
  // - 現在のactual重心位置から、目標ZMPを計算
  // - 目標ZMPを満たすように目標足裏反力を計算
  // - 目標足裏反力を満たすようにDamping Control.
  // - 目標反力を満たすように重力補償+仮想仕事の原理

  // masterのhrpsysではSwingEEModificationを行っていたが、地面についているときに、time_constでもとの位置に戻るまでの間、足と重心の相対位置が着地位置タイミング修正計算で用いるものとずれることによる着地位置修正パフォーマンスの低下のデメリットの方が大きいので、削除した
  // masterのhrpsysやもとのauto_stabilizerでは位置制御+DampingControlをサポートしていたが、位置制御+DampingControlは実機での目標接触力への追従に遅れがある. FootGuidedControlでは、目標ZMPの位相を進めたり、ZMPの追従遅れを考慮した目標ZMP計算を行ったりをしていないので、遅れに弱い. そのため、位置制御+DampingControlは削除し、所謂TorqueControlのみをサポートしている.

  // root attitude control
  this->moveBasePosRotForBodyRPYControl(dt, gaitParam, useActState,// input
                                        o_stOffsetRootRpy, o_stTargetRootPose); // output

  // 現在のactual重心位置から、目標ZMPを計算
  cnoid::Vector3 tgtForce; // generate frame
  this->calcZMP(gaitParam, dt, useActState, // input
                o_stTargetZmp, tgtForce); // output

  // 目標ZMPを満たすように目標EndEffector反力を計算
  this->calcWrench(gaitParam, o_stTargetZmp, tgtForce, useActState,// input
                   o_stEETargetWrench); // output

  if(useActState){
    // 目標反力を満たすように重力補償+仮想仕事の原理
    this->calcTorque(dt, gaitParam, o_stEETargetWrench, // input
                     actRobotTqc, o_stServoPGainPercentage, o_stServoDGainPercentage); // output
  }else{
    for(int i=0;i<actRobotTqc->numJoints();i++) actRobotTqc->joint(i)->u() = 0.0;
    for(int i=0;i<actRobotTqc->numJoints();i++){
      o_stServoPGainPercentage[i].interpolate(dt);
      o_stServoDGainPercentage[i].interpolate(dt);
    }
  }

  return true;
}

bool Stabilizer::moveBasePosRotForBodyRPYControl(double dt, const GaitParam& gaitParam, bool useActState,
                                                 cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy, cnoid::Position& o_stTargetRootPose) const{

  // stOffsetRootRpyを計算
  if(!useActState){
      o_stOffsetRootRpy.interpolate(dt);
  }else{
    cnoid::Vector3 stOffsetRootRpy = o_stOffsetRootRpy.value(); // gaitParam.footMidCoords frame

    cnoid::Matrix3 rootRErrorGenerateFrame = gaitParam.refRobot->rootLink()->R() * gaitParam.actRobot->rootLink()->R().transpose(); // generate frame
    cnoid::Matrix3 rootRError = gaitParam.footMidCoords.value().linear().transpose() * rootRErrorGenerateFrame/*generate frame*/ * gaitParam.footMidCoords.value().linear(); // gaitParam.footMidCoords frame
    cnoid::Vector3 rootRpyError = cnoid::rpyFromRot(rootRError); // gaitParam.footMidCoords frame

    for (size_t i = 0; i < 2; i++) {
      stOffsetRootRpy[i] += (this->bodyAttitudeControlGain[i] * rootRpyError[i] - 1.0/this->bodyAttitudeControlTimeConst[i] * stOffsetRootRpy[i]) * dt;
      stOffsetRootRpy[i] = mathutil::clamp(stOffsetRootRpy[i], this->bodyAttitudeControlCompensationLimit[i]);
    }
    stOffsetRootRpy[2] = 0.0;

    o_stOffsetRootRpy.reset(stOffsetRootRpy);
  }

  // stTargetRootPoseを計算
  o_stTargetRootPose.translation() = gaitParam.refRobot->rootLink()->p();
  o_stTargetRootPose.linear() /*generate frame*/= gaitParam.footMidCoords.value().linear() * cnoid::rotFromRpy(o_stOffsetRootRpy.value()/*gaitParam.footMidCoords frame*/) * gaitParam.footMidCoords.value().linear().transpose() * gaitParam.refRobot->rootLink()->R()/*generate frame*/;
  return true;
}

bool Stabilizer::calcZMP(const GaitParam& gaitParam, double dt, bool useActState,
                         cnoid::Vector3& o_tgtZmp, cnoid::Vector3& o_tgtForce) const{
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
  cnoid::Vector3 tgtCog,tgtCogVel,tgtForce;
  footguidedcontroller::updateState(gaitParam.omega,gaitParam.l,cog,cogVel,tgtZmp,gaitParam.genRobot->mass(),dt,
                                      tgtCog, tgtCogVel, tgtForce);

  o_tgtZmp = tgtZmp;
  o_tgtForce = tgtForce;
  return true;
}

bool Stabilizer::calcWrench(const GaitParam& gaitParam, const cnoid::Vector3& tgtZmp/*generate座標系*/, const cnoid::Vector3& tgtForce/*generate座標系 ロボットが受ける力*/, bool useActState,
                            std::vector<cnoid::Vector6>& o_tgtEEWrench) const{
  std::vector<cnoid::Vector6> tgtEEWrench(gaitParam.eeName.size(), cnoid::Vector6::Zero()); /* 要素数EndEffector数. generate frame. EndEffector origin*/

  for(int i = 0;i<gaitParam.eeName.size();i++){
    tgtEEWrench[i] = gaitParam.refEEWrench[i];
  }

  /*
    legは、legから受けるwrenchの和がtgtZmp, tgtForceを満たすように.
    非Support期のlegには分配せずゼロを入れる. 全てのlegが非Support期なら分配計算すら行わない
    actual robotの関節角度を用いて計算する
    各EEFのwrenchを、polygonの各頂点からのSPAN表現で考える.
    各頂点のfx, fy, fzの向きは、合力の向きと同じで、ノルムだけを変数とする. 合力がtgtForce, ZMPがtgtZmpになるように、ノルムの値を求める.
      - nzが反映できない、力の向きの冗長性を利用できない、摩擦係数を考慮できない、といった欠点がある. 二次元動歩行なので、まずは物理的・数学的厳密性や冗長性の利用よりもシンプルさ、ロバストさを優先する. そのあたりをこだわりたいなら三次元多点接触でやる.
      - 動歩行の途中の一歩で偶然actualの足が90度以上倒れた姿勢で地面につくことがあるので、そうなったときにても破綻しないことが重要.
    最後に、FACE表現に変換する.

    階層QPのタスクは次の通り
    変数: SPAN表現のノルム. 0~1
    1. ノルム>0. 合力がtgtForce.
    2. ZMPがtgtZmp
    3. 各脚の各頂点のノルムの重心がCOPOffsetと一致 (fzの値でスケールされてしまうので、alphaを用いて左右をそろえる)
    4. ノルムの2乗和の最小化 (3の中で微小な重みで一緒にやる)
  */
  // 計算時間は、tgtZmpが支持領域内に無いと遅くなるなので、事前に支持領域内に入るように修正しておくこと
  const std::vector<cnoid::Position>& EEPose = useActState ? gaitParam.actEEPose : gaitParam.abcEETargetPose;

  if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    if(gaitParam.isManualControlMode[LLEG].getGoal() == 0.0) tgtEEWrench[LLEG].setZero(); // Manual Control ModeであればrefEEWrenchをそのまま使う
    tgtEEWrench[RLEG].head<3>() = tgtForce;
    tgtEEWrench[RLEG].tail<3>() = (tgtZmp - EEPose[RLEG].translation()).cross(tgtForce);
  }else if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    if(gaitParam.isManualControlMode[RLEG].getGoal() == 0.0) tgtEEWrench[RLEG].setZero(); // Manual Control ModeであればrefEEWrenchをそのまま使う
    tgtEEWrench[LLEG].head<3>() = tgtForce;
    tgtEEWrench[LLEG].tail<3>() = (tgtZmp - EEPose[LLEG].translation()).cross(tgtForce);
  }else if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    if(gaitParam.isManualControlMode[RLEG].getGoal() == 0.0) tgtEEWrench[RLEG].setZero(); // Manual Control ModeであればrefEEWrenchをそのまま使う
    if(gaitParam.isManualControlMode[LLEG].getGoal() == 0.0) tgtEEWrench[LLEG].setZero(); // Manual Control ModeであればrefEEWrenchをそのまま使う
  }else if(tgtForce.norm() == 0){
    if(gaitParam.isManualControlMode[RLEG].getGoal() == 0.0) tgtEEWrench[RLEG].setZero(); // Manual Control ModeであればrefEEWrenchをそのまま使う
    if(gaitParam.isManualControlMode[LLEG].getGoal() == 0.0) tgtEEWrench[LLEG].setZero(); // Manual Control ModeであればrefEEWrenchをそのまま使う
  }else{
    int dim = gaitParam.legHull[RLEG].size() + gaitParam.legHull[LLEG].size();
    {
      // 1. ノルム>0. 合力がtgtForce.

      // 合力がtgtForce. (合計が1)
      this->constraintTask_->A() = Eigen::SparseMatrix<double,Eigen::RowMajor>(1,dim);
      for(int i=0;i<dim;i++) this->constraintTask_->A().insert(0,i) = 1.0;
      this->constraintTask_->b() = Eigen::VectorXd::Ones(1);
      this->constraintTask_->wa() = cnoid::VectorX::Ones(1);

      // 各値が0~1
      this->constraintTask_->C() = Eigen::SparseMatrix<double,Eigen::RowMajor>(dim,dim);
      for(int i=0;i<dim;i++) this->constraintTask_->C().insert(i,i) = 1.0;
      this->constraintTask_->dl() = Eigen::VectorXd::Zero(dim);
      this->constraintTask_->du() = Eigen::VectorXd::Ones(dim);
      this->constraintTask_->wc() = cnoid::VectorX::Ones(dim);

      this->constraintTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->constraintTask_->toSolve() = false;
      this->constraintTask_->solver().settings()->setVerbosity(0);
    }
    {
      // 2. ZMPがtgtZmp
      // tgtZmpまわりのトルクの和を求めて、tgtForceの向きの単位ベクトルとの外積が0なら良い
      //this->tgtZmpTask_->A() = Eigen::SparseMatrix<double,Eigen::RowMajor>(3,dim);
      cnoid::Vector3 tgtForceDir = tgtForce.normalized();
      int idx = 0;
      Eigen::SparseMatrix<double,Eigen::ColMajor> A_ColMajor(3,dim); // insert()する順序がColMajorなので、RowMajorのAに直接insertすると計算効率が著しく悪い(ミリ秒単位で時間がかかる).
      for(int i=0;i<NUM_LEGS;i++){
        for(int j=0;j<gaitParam.legHull[i].size();j++){
          cnoid::Vector3 pos = EEPose[i] * gaitParam.legHull[i][j];
          cnoid::Vector3 a = tgtForceDir.cross( (pos - tgtZmp).cross(tgtForce));
          for(int k=0;k<3;k++) A_ColMajor.insert(k,idx) = a[k];
          idx ++;
        }
      }
      this->tgtZmpTask_->A() = A_ColMajor;
      this->tgtZmpTask_->b() = Eigen::VectorXd::Zero(3);
      this->tgtZmpTask_->wa() = cnoid::VectorX::Ones(3);

      this->tgtZmpTask_->C() = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
      this->tgtZmpTask_->dl() = Eigen::VectorXd::Zero(0);
      this->tgtZmpTask_->du() = Eigen::VectorXd::Ones(0);
      this->tgtZmpTask_->wc() = cnoid::VectorX::Ones(0);

      this->tgtZmpTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->tgtZmpTask_->toSolve() = false; // 常にtgtZmpが支持領域内にあるなら解く必要がないので高速化のためfalseにする. ない場合があるならtrueにする. calcWrenchでtgtZmpをtruncateしているのでfalseでよい
      this->tgtZmpTask_->solver().settings()->setVerbosity(0);
    }
    {
      // 3. 各脚の各頂点のノルムの重心がCOPOffsetと一致 (fzの値でスケールされてしまうので、alphaを用いて左右をそろえる)

      // 各EndEffectorとtgtZmpの距離を用いてalphaを求める
      std::vector<double> alpha(NUM_LEGS);
      {
        cnoid::Vector3 rleg2leg = EEPose[LLEG].translation() - EEPose[RLEG].translation();
        rleg2leg[2] = 0.0;
        if(rleg2leg.norm() == 0.0){
          alpha[RLEG] = alpha[LLEG] = 0.5;
        }else{
          cnoid::Vector3 rleg2legDir = rleg2leg.normalized();
          double rleg2llegDistance = rleg2leg.norm();
          double rleg2tgtZmpRatio = rleg2legDir.dot(tgtZmp - EEPose[RLEG].translation()) / rleg2llegDistance;
          alpha[RLEG] = mathutil::clamp(1.0 - rleg2tgtZmpRatio, 0.05, 1.0-0.05);
          alpha[LLEG] = mathutil::clamp(rleg2tgtZmpRatio, 0.05, 1.0-0.05);
        }
      }
      Eigen::SparseMatrix<double,Eigen::ColMajor> A_ColMajor(3*NUM_LEGS,dim); // insert()する順序がColMajorなので、RowMajorのAに直接insertすると計算効率が著しく悪い(ミリ秒単位で時間がかかる).
      int idx = 0;
      for(int i=0;i<NUM_LEGS;i++) {
        cnoid::Vector3 cop = EEPose[i].translation() + EEPose[i].linear() * gaitParam.copOffset[i].value();
        for(int j=0;j<gaitParam.legHull[i].size();j++){
          cnoid::Vector3 pos = EEPose[i].translation() + EEPose[i].linear() * gaitParam.legHull[i][j];
          cnoid::Vector3 a = (pos - cop) / alpha[i];
          for(int k=0;k<3;k++) A_ColMajor.insert(i*3+k,idx) = a[k];
          idx ++;
        }
      }
      this->copTask_->A() = A_ColMajor;
      this->copTask_->b() = Eigen::VectorXd::Zero(3*NUM_LEGS);
      this->copTask_->wa() = cnoid::VectorX::Ones(3*NUM_LEGS);

      this->copTask_->C() = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
      this->copTask_->dl() = Eigen::VectorXd::Zero(0);
      this->copTask_->du() = Eigen::VectorXd::Ones(0);
      this->copTask_->wc() = cnoid::VectorX::Ones(0);

      this->copTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->copTask_->toSolve() = true;
      // this->copTask_->options().setToReliable();
      // this->copTask_->options().printLevel = qpOASES::PL_NONE; // PL_HIGH or PL_NONE
      this->copTask_->solver().settings()->setCheckTermination(5); // default 25. 高速化
      this->copTask_->solver().settings()->setVerbosity(0);
    }

    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks{this->constraintTask_,this->tgtZmpTask_,this->copTask_};
    cnoid::VectorX result;
    if(!prioritized_qp_base::solve(tasks,
                                   result,
                                   0 // debuglevel
                                   )){
      // QP fail. 目標力を何も入れないよりはマシなので適当に入れる.
      tgtEEWrench[RLEG].head<3>() = tgtForce / 2;
      tgtEEWrench[LLEG].head<3>() = tgtForce / 2;
    }else{
      int idx = 0;
      for(int i=0;i<NUM_LEGS;i++){
	tgtEEWrench[i].setZero();
        for(int j=0;j<gaitParam.legHull[i].size();j++){
          tgtEEWrench[i].head<3>() += tgtForce * result[idx];
          tgtEEWrench[i].tail<3>() += (EEPose[i].linear() * gaitParam.legHull[i][j]).cross(tgtForce * result[idx]);
          idx ++;
        }
      }
    }
  }

  o_tgtEEWrench = tgtEEWrench;
  return true;
}

bool Stabilizer::calcTorque(double dt, const GaitParam& gaitParam, const std::vector<cnoid::Vector6>& tgtEEWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/,
                            cnoid::BodyPtr& actRobotTqc, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPGainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDGainPercentage) const{
  // 速度・加速度を考慮しない重力補償
  actRobotTqc->rootLink()->T() = gaitParam.actRobot->rootLink()->T();
  actRobotTqc->rootLink()->v() = cnoid::Vector3::Zero();
  actRobotTqc->rootLink()->w() = cnoid::Vector3::Zero();
  actRobotTqc->rootLink()->dv() = cnoid::Vector3(0.0,0.0,gaitParam.g);
  actRobotTqc->rootLink()->dw() = cnoid::Vector3::Zero();
  for(int i=0;i<actRobotTqc->numJoints();i++){
    actRobotTqc->joint(i)->q() = gaitParam.actRobot->joint(i)->q();
    actRobotTqc->joint(i)->dq() = 0.0;
    actRobotTqc->joint(i)->ddq() = 0.0;
  }
  actRobotTqc->calcForwardKinematics(true, true);
  cnoid::calcInverseDynamics(actRobotTqc->rootLink()); // actRobotTqc->joint()->u()に書き込まれる

  // tgtEEWrench
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

  // Gain
  for(int i=0;i<NUM_LEGS;i++){
    cnoid::JointPath jointPath(actRobotTqc->rootLink(), actRobotTqc->link(gaitParam.eeParentLink[i]));
    if(gaitParam.isManualControlMode[i].getGoal() == 0.0) { // Manual Control off
      if(gaitParam.footstepNodesList[0].isSupportPhase[i]){
        double transitionTime = std::max(this->landing2SupportTransitionTime, dt*2); // 現状, setGoal(*,dt)以下の時間でgoal指定するとwriteOutPortDataが破綻するのでテンポラリ
        for(int j=0;j<jointPath.numJoints();j++){
          if(o_stServoPGainPercentage[jointPath.joint(j)->jointId()].getGoal() != this->supportPgain[i][j]) o_stServoPGainPercentage[jointPath.joint(j)->jointId()].setGoal(this->supportPgain[i][j], transitionTime);
          if(o_stServoDGainPercentage[jointPath.joint(j)->jointId()].getGoal() != this->supportDgain[i][j]) o_stServoDGainPercentage[jointPath.joint(j)->jointId()].setGoal(this->supportDgain[i][j], transitionTime);
        }
      }else if(gaitParam.swingState[i] == GaitParam::DOWN_PHASE) {
        double transitionTime = std::max(this->swing2LandingTransitionTime, dt*2); // 現状, setGoal(*,dt)以下の時間でgoal指定するとwriteOutPortDataが破綻するのでテンポラリ
        for(int j=0;j<jointPath.numJoints();j++){
          if(o_stServoPGainPercentage[jointPath.joint(j)->jointId()].getGoal() != this->landingPgain[i][j]) o_stServoPGainPercentage[jointPath.joint(j)->jointId()].setGoal(this->landingPgain[i][j], transitionTime);
          if(o_stServoDGainPercentage[jointPath.joint(j)->jointId()].getGoal() != this->landingDgain[i][j]) o_stServoDGainPercentage[jointPath.joint(j)->jointId()].setGoal(this->landingDgain[i][j], transitionTime);
        }
      }else{
        double transitionTime = std::max(this->support2SwingTransitionTime, dt*2); // 現状, setGoal(*,dt)以下の時間でgoal指定するとwriteOutPortDataが破綻するのでテンポラリ
        for(int j=0;j<jointPath.numJoints();j++){
          if(o_stServoPGainPercentage[jointPath.joint(j)->jointId()].getGoal() != this->swingPgain[i][j]) o_stServoPGainPercentage[jointPath.joint(j)->jointId()].setGoal(this->swingPgain[i][j], transitionTime);
          if(o_stServoDGainPercentage[jointPath.joint(j)->jointId()].getGoal() != this->swingDgain[i][j]) o_stServoDGainPercentage[jointPath.joint(j)->jointId()].setGoal(this->swingDgain[i][j], transitionTime);
        }
      }
    }else{ // Manual Control on
      double transitionTime = std::max(gaitParam.isManualControlMode[i].remain_time(), dt*2); // 現状, setGoal(*,dt)以下の時間でgoal指定するとwriteOutPortDataが破綻するのでテンポラリ
      for(int j=0;j<jointPath.numJoints();j++){
        if(o_stServoPGainPercentage[jointPath.joint(j)->jointId()].getGoal() != 100.0) o_stServoPGainPercentage[jointPath.joint(j)->jointId()].setGoal(100.0, transitionTime);
        if(o_stServoDGainPercentage[jointPath.joint(j)->jointId()].getGoal() != 100.0) o_stServoDGainPercentage[jointPath.joint(j)->jointId()].setGoal(100.0, transitionTime);
      }
    }
  }
  for(int i=0;i<gaitParam.genRobot->numJoints();i++){
    o_stServoPGainPercentage[i].interpolate(dt);
    o_stServoDGainPercentage[i].interpolate(dt);
  }

  return true;
}

