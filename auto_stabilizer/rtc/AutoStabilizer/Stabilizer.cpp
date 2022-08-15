#include "Stabilizer.h"
#include "MathUtil.h"
#include <cnoid/JointPath>
#include <cnoid/Jacobian>
#include <cnoid/EigenUtil>

void Stabilizer::initStabilizerOutput(const GaitParam& gaitParam,
                                      cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy, std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffsetDampingControl /*generate frame, endeffector origin*/, std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffsetSwingEEModification /*generate frame, endeffector origin*/, cnoid::Vector3& o_stTargetZmp) const{

  for(int i=0;i<gaitParam.eeName.size();i++){
    o_stEEOffsetDampingControl[i].reset(cnoid::Vector6::Zero());
    o_stEEOffsetSwingEEModification[i].reset(cnoid::Vector6::Zero());
  }
  o_stOffsetRootRpy.reset(cnoid::Vector3::Zero());
  o_stTargetZmp = gaitParam.refZmpTraj[0].getStart();
}

bool Stabilizer::execStabilizer(const cnoid::BodyPtr refRobot, const cnoid::BodyPtr actRobot, const cnoid::BodyPtr genRobot, const GaitParam& gaitParam, double dt, double mass,
                                cnoid::BodyPtr& actRobotTqc, cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy, std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffsetDampingControl /*generate frame, endeffector origin*/, std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffsetSwingEEModification /*generate frame, endeffector origin*/, cnoid::Vector3& o_stTargetZmp) const{
  // - root attitude control
  // - 現在のactual重心位置から、目標ZMPを計算
  // - 目標ZMPを満たすように目標足裏反力を計算
  // - 目標反力を満たすように重力補償+仮想仕事の原理
  // - 目標足裏反力を満たすようにDamping Control.

  // root attitude control
  this->moveBasePosRotForBodyRPYControl(refRobot, actRobot, dt, gaitParam, // input
                                        o_stOffsetRootRpy); // output

  // 現在のactual重心位置から、目標ZMPを計算
  cnoid::Vector3 tgtForce; // generate frame
  this->calcZMP(gaitParam, dt, mass, // input
                o_stTargetZmp, tgtForce); // output

  // 目標ZMPを満たすように目標EndEffector反力を計算
  std::vector<cnoid::Vector6> tgtEEWrench; // 要素数EndEffector数. generate frame. EndEffector origin
  this->calcWrench(gaitParam, o_stTargetZmp, tgtForce, // input
                   tgtEEWrench); // output

  // 目標反力を満たすように重力補償+仮想仕事の原理
  this->calcTorque(actRobot, dt, gaitParam, tgtEEWrench, // input
                   actRobotTqc); // output

  // 目標足裏反力を満たすようにDamping Control
  this->calcDampingControl(dt, gaitParam, tgtEEWrench, // input
                           o_stEEOffsetDampingControl); // output

  // 目標遊脚位置を満たすように、SwingEEModification
  this->calcSwingEEModification(dt, gaitParam, //input
                                o_stEEOffsetSwingEEModification); //output

  return true;
}

bool Stabilizer::moveBasePosRotForBodyRPYControl(const cnoid::BodyPtr refRobot, const cnoid::BodyPtr actRobot, double dt, const GaitParam& gaitParam,
                                                 cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy) const{
  cnoid::Vector3 stOffsetRootRpy = gaitParam.stOffsetRootRpy.value(); // gaitParam.footMidCoords frame

  cnoid::Matrix3 rootRErrorGenerateFrame = refRobot->rootLink()->R() * actRobot->rootLink()->R().transpose(); // generate frame
  cnoid::Matrix3 rootRError = gaitParam.footMidCoords.value().linear().transpose() * rootRErrorGenerateFrame/*generate frame*/ * gaitParam.footMidCoords.value().linear(); // gaitParam.footMidCoords frame
  cnoid::Vector3 rootRpyError = cnoid::rpyFromRot(rootRError); // gaitParam.footMidCoords frame

  for (size_t i = 0; i < 2; i++) {
    stOffsetRootRpy[i] += (this->bodyAttitudeControlGain[i] * rootRpyError[i] - 1.0/this->bodyAttitudeControlTimeConst[i] * stOffsetRootRpy[i]) * dt;
    stOffsetRootRpy[i] = mathutil::clamp(stOffsetRootRpy[i], this->bodyAttitudeControlCompensationLimit[i]);
  }
  stOffsetRootRpy[2] = 0.0;

  o_stOffsetRootRpy.reset(stOffsetRootRpy);
  return true;
}

bool Stabilizer::calcZMP(const GaitParam& gaitParam, double dt, double mass,
                         cnoid::Vector3& o_tgtZmp, cnoid::Vector3& o_tgtForce) const{
  cnoid::Vector3 tgtZmp;
  if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] || gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    cnoid::Vector3 actDCM = gaitParam.actCog + gaitParam.actCogVel.value() / gaitParam.omega;
    tgtZmp = footguidedcontroller::calcFootGuidedControl(gaitParam.omega,gaitParam.l,actDCM,gaitParam.refZmpTraj);
    if(tgtZmp[2] >= gaitParam.actCog[2]) tgtZmp = gaitParam.actCog - cnoid::Vector3(gaitParam.l[0],gaitParam.l[1], 0.0); // 下向きの力は受けられないので
    else{
      // truncate zmp inside polygon. actual robotの関節角度を用いて計算する
      std::vector<cnoid::Vector3> vertices; // generate frame. 支持点の集合
      for(int i=0;i<NUM_LEGS;i++){
        if(!gaitParam.footstepNodesList[0].isSupportPhase[i]) continue;
        for(int j=0;j<gaitParam.legHull[i].size();j++){
          vertices.push_back(gaitParam.actEEPose[i]*gaitParam.legHull[i][j]);
        }
      }
      tgtZmp = mathutil::calcInsidePointOfPolygon3D(tgtZmp,vertices,gaitParam.actCog - cnoid::Vector3(gaitParam.l[0],gaitParam.l[1], 0.0));
      // TODO. 角運動量オフセット.
    }
  }else{ // 跳躍期
    tgtZmp = gaitParam.actCog - cnoid::Vector3(gaitParam.l[0],gaitParam.l[1], 0.0);
  }
  cnoid::Vector3 tgtCog,tgtCogVel,tgtForce;
  footguidedcontroller::updateState(gaitParam.omega,gaitParam.l,gaitParam.actCog,gaitParam.actCogVel.value(),tgtZmp,mass,dt,
                                    tgtCog, tgtCogVel, tgtForce);

  o_tgtZmp = tgtZmp;
  o_tgtForce = tgtForce;
  return true;
}

bool Stabilizer::calcWrench(const GaitParam& gaitParam, const cnoid::Vector3& tgtZmp/*generate座標系*/, const cnoid::Vector3& tgtForce/*generate座標系 ロボットが受ける力*/,
                            std::vector<cnoid::Vector6>& o_tgtEEWrench) const{
  std::vector<cnoid::Vector6> tgtEEWrench(gaitParam.eeName.size(), cnoid::Vector6::Zero()); /* 要素数EndEffector数. generate frame. EndEffector origin*/

  gaitParam.resetTime();

  // leg以外はref値をそのまま
  for(int i = NUM_LEGS;i<gaitParam.eeName.size();i++){
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

  if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    tgtEEWrench[LLEG].setZero();
    tgtEEWrench[RLEG].head<3>() = tgtForce;
    tgtEEWrench[RLEG].tail<3>() = (tgtZmp - gaitParam.actEEPose[RLEG].translation()).cross(tgtForce);
  }else if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    tgtEEWrench[RLEG].setZero();
    tgtEEWrench[LLEG].head<3>() = tgtForce;
    tgtEEWrench[LLEG].tail<3>() = (tgtZmp - gaitParam.actEEPose[LLEG].translation()).cross(tgtForce);
  }else if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    tgtEEWrench[RLEG].setZero();
    tgtEEWrench[LLEG].setZero();
  }else if(tgtForce.norm() == 0){
    tgtEEWrench[RLEG].setZero();
    tgtEEWrench[LLEG].setZero();
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
      //this->constraintTask_->solver().settings()->setVerbosity(0);
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
          cnoid::Vector3 pos = gaitParam.actEEPose[i] * gaitParam.legHull[i][j];
          cnoid::Vector3 a = tgtForceDir.cross( (pos - tgtZmp).cross(tgtForce));
          for(int k=0;k<3;k++) A_ColMajor.insert(k,idx) = a[k];
          idx ++;
        }
      }
      this->tgtZmpTask_->A() = A_ColMajor;
      gaitParam.printTime();
      this->tgtZmpTask_->b() = Eigen::VectorXd::Zero(3);
      this->tgtZmpTask_->wa() = cnoid::VectorX::Ones(3);

      this->tgtZmpTask_->C() = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
      this->tgtZmpTask_->dl() = Eigen::VectorXd::Zero(0);
      this->tgtZmpTask_->du() = Eigen::VectorXd::Ones(0);
      this->tgtZmpTask_->wc() = cnoid::VectorX::Ones(0);

      this->tgtZmpTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->tgtZmpTask_->toSolve() = false; // 常にtgtZmpが支持領域内にあるなら解く必要がないので高速化のためfalseにする. ない場合があるならtrueにする. calcWrenchでtgtZmpをtruncateしているのでfalseでよい
      //this->tgtZmpTask_->solver().settings()->setVerbosity(0);
    }
    gaitParam.printTime();
    {
      // 3. 各脚の各頂点のノルムの重心がCOPOffsetと一致 (fzの値でスケールされてしまうので、alphaを用いて左右をそろえる)

      // 各EndEffectorとtgtZmpの距離を用いてalphaを求める
      std::vector<double> alpha(NUM_LEGS);
      {
        cnoid::Vector3 rleg2leg = gaitParam.actEEPose[LLEG].translation() - gaitParam.actEEPose[RLEG].translation();
        rleg2leg[2] = 0.0;
        if(rleg2leg.norm() == 0.0){
          alpha[RLEG] = alpha[LLEG] = 0.5;
        }else{
          cnoid::Vector3 rleg2legDir = rleg2leg.normalized();
          double rleg2llegDistance = rleg2leg.norm();
          double rleg2tgtZmpRatio = rleg2legDir.dot(tgtZmp - gaitParam.actEEPose[RLEG].translation()) / rleg2llegDistance;
          alpha[RLEG] = mathutil::clamp(1.0 - rleg2tgtZmpRatio, 0.05, 1.0-0.05);
          alpha[LLEG] = mathutil::clamp(rleg2tgtZmpRatio, 0.05, 1.0-0.05);
        }
      }
      Eigen::SparseMatrix<double,Eigen::ColMajor> A_ColMajor(3*NUM_LEGS,dim); // insert()する順序がColMajorなので、RowMajorのAに直接insertすると計算効率が著しく悪い(ミリ秒単位で時間がかかる).
      int idx = 0;
      for(int i=0;i<NUM_LEGS;i++) {
        cnoid::Vector3 cop = gaitParam.actEEPose[i].translation() + gaitParam.actEEPose[i].linear() * gaitParam.copOffset[i].value();
        for(int j=0;j<gaitParam.legHull[i].size();j++){
          cnoid::Vector3 pos = gaitParam.actEEPose[i].translation() + gaitParam.actEEPose[i].linear() * gaitParam.legHull[i][j];
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
      this->copTask_->options().setToReliable();
      this->copTask_->options().printLevel = qpOASES::PL_NONE; // PL_HIGH or PL_NONE
      //this->copTask_->solver().settings()->setCheckTermination(5); // default 25. 高速化
      //this->copTask_->solver().settings()->setVerbosity(0);
    }

    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks{this->constraintTask_,this->tgtZmpTask_,this->copTask_};
    cnoid::VectorX result;
    gaitParam.printTime();
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
        for(int j=0;j<gaitParam.legHull[i].size();j++){
          tgtEEWrench[i].head<3>() += tgtForce * result[idx];
          tgtEEWrench[i].tail<3>() += (gaitParam.actEEPose[i].linear() * gaitParam.legHull[i][j]).cross(tgtForce * result[idx]);
          idx ++;
        }
      }
    }
    gaitParam.printTime();
  }

  o_tgtEEWrench = tgtEEWrench;
  return true;
}

bool Stabilizer::calcTorque(const cnoid::BodyPtr actRobot, double dt, const GaitParam& gaitParam, const std::vector<cnoid::Vector6>& tgtEEWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/,
                            cnoid::BodyPtr& actRobotTqc) const{
  // 速度・加速度を考慮しない重力補償
  actRobotTqc->rootLink()->T() = actRobot->rootLink()->T();
  actRobotTqc->rootLink()->v() = cnoid::Vector3::Zero();
  actRobotTqc->rootLink()->w() = cnoid::Vector3::Zero();
  actRobotTqc->rootLink()->dv() = cnoid::Vector3::Zero();
  actRobotTqc->rootLink()->dw() = cnoid::Vector3::Zero();
  for(int i=0;i<actRobotTqc->numJoints();i++){
    actRobotTqc->joint(i)->q() = actRobot->joint(i)->q();
    actRobotTqc->joint(i)->dq() = 0.0;
    actRobotTqc->joint(i)->ddq() = 0.0;
  }
  actRobotTqc->calcForwardKinematics(true, true); // actRobotTqc->joint()->u()に書き込まれる

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

  return true;
}

bool Stabilizer::calcDampingControl(double dt, const GaitParam& gaitParam, const std::vector<cnoid::Vector6>& tgtEEWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/,
                                    std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffsetDampingControl /*generate frame, endeffector origin*/) const{

  std::vector<cnoid::Vector6> wrenchError(NUM_LEGS); // generate frame. endEffector origin.
  for(int i=0;i<NUM_LEGS;i++){
    wrenchError[i] = gaitParam.actEEWrench[i] - tgtEEWrench[i];
  }
  // force difference control
  if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) wrenchError[RLEG][2] = 0.0;
  else if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) wrenchError[LLEG][2] = 0.0;
  else if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) {
    double averageFzError = (wrenchError[RLEG][2] + wrenchError[LLEG][2]) / 2.0;
    wrenchError[RLEG][2] -= averageFzError;
    wrenchError[LLEG][2] -= averageFzError;
  }

  for(int i=0;i<NUM_LEGS;i++){

    cnoid::Vector6 offsetPrev; // generate frame. endEffector origin
    cnoid::Vector6 dOffsetPrev; // generate frame. endEffector origin
    gaitParam.stEEOffsetDampingControl[i].value(offsetPrev, dOffsetPrev);

    cnoid::Matrix3 eeR = cnoid::AngleAxisd(offsetPrev.tail<3>().norm(),(offsetPrev.tail<3>().norm()>0)?offsetPrev.tail<3>().normalized() : cnoid::Vector3::UnitX()) * gaitParam.abcEETargetPose[i].linear();

    cnoid::Vector6 wrenchErrorLocal; //endEffector frame. endeffector origin
    wrenchErrorLocal.head<3>() = eeR.transpose() * wrenchError[i].head<3>();
    wrenchErrorLocal.tail<3>() = eeR.transpose() * wrenchError[i].tail<3>();

    cnoid::Vector6 offsetPrevLocal; //endEffector frame. endeffector origin
    offsetPrevLocal.head<3>() = eeR.transpose() * offsetPrev.head<3>();
    offsetPrevLocal.tail<3>() = eeR.transpose() * offsetPrev.tail<3>();

    cnoid::Vector6 dOffsetPrevLocal; //endEffector frame. endeffector origin
    dOffsetPrevLocal.head<3>() = eeR.transpose() * dOffsetPrev.head<3>();
    dOffsetPrevLocal.tail<3>() = eeR.transpose() * dOffsetPrev.tail<3>();

    cnoid::Vector6 dOffsetLocal; //endEffector frame. endeffector origin
    for(size_t j=0;j<6;j++){
      if(this->dampingGain[i][j] == 0.0 || this->dampingTimeConst[i][j] == 0.0){
        dOffsetLocal[j] = 0.0;
        continue;
      }

      dOffsetLocal[j] = (wrenchErrorLocal[j] / this->dampingGain[i][j] - offsetPrevLocal[j] / this->dampingTimeConst[i][j]) * dt;
    }

    cnoid::Vector6 dOffset; //generate frame. endeffector origin
    dOffset.head<3>() = eeR * dOffsetLocal.head<3>();
    dOffset.tail<3>() = eeR * dOffsetLocal.tail<3>();

    cnoid::Vector6 offset = offsetPrev + dOffset;
    offset = mathutil::clampMatrix<cnoid::Vector6>(offset, this->dampingCompensationLimit[i]);
    o_stEEOffsetDampingControl[i].reset(offset, dOffset/dt);
  }


  return true;
}

bool Stabilizer::calcSwingEEModification(double dt, const GaitParam& gaitParam,
                                         std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stEEOffsetSwingEEModification /*generate frame, endeffector origin*/) const{
  for(int i=0;i<NUM_LEGS;i++){
    cnoid::Vector6 offset = o_stEEOffsetSwingEEModification[i].value();
    offset -= offset.cwiseQuotient(this->springTimeConst[i]) * dt;
    if(!gaitParam.footstepNodesList[0].isSupportPhase[i]){
      cnoid::Vector6 diff; // generage frame. endeffeector origin
      diff.head<3>() = gaitParam.abcEETargetPose[i].translation() - gaitParam.actEEPose[i].translation();
      Eigen::AngleAxisd diffR = Eigen::AngleAxisd(gaitParam.abcEETargetPose[i].linear() * gaitParam.actEEPose[i].linear().inverse());
      diff.tail<3>() = diffR.angle() * diffR.axis();
      cnoid::Vector6 dp = this->springGain[i].cwiseProduct(diff);
      dp = mathutil::clampMatrix(dp, this->springCompensationVelocityLimit[i]);
      offset += dp * dt;
      offset = mathutil::clampMatrix(offset, this->springCompensationLimit[i]);
      o_stEEOffsetSwingEEModification[i].reset(offset, dp);
    }
  }
  return true;
}
