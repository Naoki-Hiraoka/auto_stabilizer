#include "Stabilizer.h"
#include "MathUtil.h"
#include <cnoid/JointPath>
#include <cnoid/Jacobian>

bool Stabilizer::execStabilizer(const cnoid::BodyPtr refRobotOrigin, const cnoid::BodyPtr actRobotOrigin, const cnoid::BodyPtr genRobot, const GaitParam& gaitParam, const EndEffectorParam& endEffectorParam, double dt, double g, double mass,
                                cnoid::BodyPtr& actRobotTqc, cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy, std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stOffset /*generate frame, endeffector origin*/) const{
  // - root attitude control
  // - 現在のactual重心位置から、目標ZMPを計算
  // - 目標ZMPを満たすように目標足裏反力を計算
  // - 目標反力を満たすように重力補償+仮想仕事の原理
  // - 目標足裏反力を満たすようにDamping Control.

  // root attitude control
  this->moveBasePosRotForBodyRPYControl(refRobotOrigin, actRobotOrigin, dt, gaitParam, // input
                                        o_stOffsetRootRpy); // output

  // 現在のactual重心位置から、目標ZMPを計算
  cnoid::Vector3 tgtZmp; // generate frame
  cnoid::Vector3 tgtForce; // generate frame
  this->calcZMP(gaitParam, dt, g, mass, // input
                tgtZmp, tgtForce); // output

  // 目標ZMPを満たすように目標EndEffector反力を計算
  std::vector<cnoid::Vector6> tgtWrench; // 要素数EndEffector数. generate frame. EndEffector origin
  this->calcWrench(gaitParam, endEffectorParam, tgtZmp, tgtForce, // input
                   tgtWrench); // output

  // 目標反力を満たすように重力補償+仮想仕事の原理
  this->calcTorque(actRobotOrigin, dt, endEffectorParam, tgtWrench, // input
                   actRobotTqc); // output

  // 目標足裏反力を満たすようにDamping Control
  this->calcDampingControl(dt, gaitParam, endEffectorParam, tgtWrench, // input
                           o_stOffset); // output

  return true;
}

bool Stabilizer::moveBasePosRotForBodyRPYControl(const cnoid::BodyPtr refRobotOrigin, const cnoid::BodyPtr actRobotOrigin, double dt, const GaitParam& gaitParam,
                                                 cpp_filters::TwoPointInterpolator<cnoid::Vector3>& o_stOffsetRootRpy) const{
  cnoid::Vector3 stOffsetRootRpy = gaitParam.stOffsetRootRpy.value(); // gaitParam.footMidCoords座標系

  cnoid::Matrix3 rootRError = gaitParam.footMidCoords.value().linear().transpose() * (refRobotOrigin->rootLink()->R() * actRobotOrigin->rootLink()->R().transpose()); // gaitParam.footMidCoords座標系
  cnoid::Vector3 rootRpyError = cnoid::rpyFromRot(rootRError); // gaitParam.footMidCoords座標系

  for (size_t i = 0; i < 2; i++) {
    stOffsetRootRpy[i] += (this->bodyAttitudeControlGain[i] * rootRpyError[i] - 1.0/this->bodyAttitudeControlTimeConst[i] * stOffsetRootRpy[i]) * dt;
    stOffsetRootRpy[i] = mathutil::clamp(stOffsetRootRpy[i], this->rootRotCompensationLimit[i]);
  }
  stOffsetRootRpy[2] = 0.0;

  o_stOffsetRootRpy.reset(stOffsetRootRpy);
  return true;
}

bool Stabilizer::calcZMP(const GaitParam& gaitParam, double dt, double g, double mass,
                         cnoid::Vector3& o_tgtZmp, cnoid::Vector3& o_tgtForce) const{
  double w = std::sqrt(g/gaitParam.dz); // TODO refforceZ
  cnoid::Vector3 l = cnoid::Vector3::Zero();
  l[2] = gaitParam.dz;
  cnoid::Vector3 actDCM = gaitParam.actCog + gaitParam.actCogVel.value() / w;
  cnoid::Vector3 tgtZmp = footguidedcontroller::calcFootGuidedControl(w,l,actDCM,gaitParam.refZmpTraj);
  // check zmp in polygon TODO. COMより低く. 角運動量オフセット
  cnoid::Vector3 tgtCog,tgtCogVel,tgtForce;
  footguidedcontroller::updateState(w,l,gaitParam.actCog,gaitParam.actCogVel.value(),tgtZmp,mass,dt,
                                    tgtCog, tgtCogVel, tgtForce);

  o_tgtZmp = tgtZmp;
  o_tgtForce = tgtForce;
  return true;
}

bool Stabilizer::calcWrench(const GaitParam& gaitParam, const EndEffectorParam& endEffectorParam, const cnoid::Vector3& tgtZmp, const cnoid::Vector3& tgtForce,
                            std::vector<cnoid::Vector6>& o_tgtWrench) const{
  std::vector<cnoid::Vector6> tgtWrench(endEffectorParam.name.size(), cnoid::Vector6::Zero()); /* 要素数EndEffector数. generate frame. EndEffector origin*/

  // leg以外はref値をそのまま
  for(int i = NUM_LEGS;i<endEffectorParam.name.size();i++){
    tgtWrench[i] = endEffectorParam.refWrench[i];
  }

  /*
    legは、legから受けるwrenchの和がtgtZmp, tgtForceを満たすように.
    非Support期のlegには分配せずゼロを入れる. 全てのlegが非Support期なら分配計算すら行わない
    各EEFのwrenchを、polygonの各頂点からのSPAN表現で考える.
    各頂点のfx, fy, fzの向きは、合力の向きと同じで、ノルムだけを変数とする. 合力がtgtForce, ZMPがtgtZmpになるように、ノルムの値を求める.
      - nzが反映できない、力の向きの冗長性を利用できない、摩擦係数を考慮できない、といった欠点がある. 二次元動歩行なので、まずは物理的・数学的厳密性や冗長性の利用よりもシンプルさ、ロバストさを優先する. そのあたりをこだわりたいなら三次元多点接触でやる.
      - 動歩行の途中の一歩で偶然actualの足が90度以上倒れた姿勢で地面につくことがあるので、そうなったときにても破綻しないことが重要.
    最後に、FACE表現に変換する.

    階層QPのタスクは次の通り
    変数: SPAN表現のノルム. 各脚のCOP(XY)
    1. ノルム>0. 合力がtgtForce. 各脚のCOPが各頂点のノルムの重心と一致
    2. ZMPがtgtZmp
    3. 各脚のCOPがCOPOffsetと一致
    4. ノルムの2乗和の最小化
  */

  // TODO

  o_tgtWrench = tgtWrench;
  return true;
}

bool Stabilizer::calcTorque(const cnoid::BodyPtr actRobotOrigin, double dt, const EndEffectorParam& endEffectorParam, const std::vector<cnoid::Vector6>& tgtWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/,
                            cnoid::BodyPtr& actRobotTqc) const{
  // 速度・加速度を考慮しない重力補償
  actRobotTqc->rootLink()->T() = actRobotOrigin->rootLink()->T();
  actRobotTqc->rootLink()->v() = cnoid::Vector3::Zero();
  actRobotTqc->rootLink()->w() = cnoid::Vector3::Zero();
  actRobotTqc->rootLink()->dv() = cnoid::Vector3::Zero();
  actRobotTqc->rootLink()->dw() = cnoid::Vector3::Zero();
  for(int i=0;i<actRobotTqc->numJoints();i++){
    actRobotTqc->joint(i)->q() = actRobotOrigin->joint(i)->q();
    actRobotTqc->joint(i)->dq() = 0.0;
    actRobotTqc->joint(i)->ddq() = 0.0;
  }
  actRobotTqc->calcForwardKinematics(true, true); // actRobotTqc->joint()->u()に書き込まれる

  // tgtWrench
  for(int i=0;i<endEffectorParam.name.size();i++){
    cnoid::JointPath jointPath(actRobotTqc->rootLink(), actRobotTqc->link(endEffectorParam.parentLink[i]));
    cnoid::MatrixXd J = cnoid::MatrixXd::Zero(6,jointPath.numJoints()); // generate frame. endeffector origin
    cnoid::setJacobian<0x3f,0,0,true>(jointPath,actRobotTqc->link(endEffectorParam.parentLink[i]),endEffectorParam.localT[i].translation(), // input
                                      J); // output
    cnoid::VectorX tau = - J.transpose() * tgtWrench[i];
    for(int j=0;j<jointPath.numJoints();j++){
      jointPath.joint(j)->u() += tau[j];
    }
  }

  return true;
}

bool Stabilizer::calcDampingControl(double dt, const GaitParam& gaitParam, const EndEffectorParam& endEffectorParam, const std::vector<cnoid::Vector6>& tgtWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/,
                                    std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_stOffset /*generate frame, endeffector origin*/) const{

  std::vector<cnoid::Vector6> wrenchError(NUM_LEGS); // generate frame. endEffector origin.
  for(int i=0;i<NUM_LEGS;i++){
    wrenchError[i] = endEffectorParam.actWrench[i] - tgtWrench[i];
  }
  // force difference control
  if(gaitParam.isSupportPhase(RLEG) && !gaitParam.isSupportPhase(LLEG)) wrenchError[RLEG][2] = 0.0;
  else if(!gaitParam.isSupportPhase(RLEG) && gaitParam.isSupportPhase(LLEG)) wrenchError[LLEG][2] = 0.0;
  else if(gaitParam.isSupportPhase(RLEG) && gaitParam.isSupportPhase(LLEG)) {
    double averageFzError = (wrenchError[RLEG][2] + wrenchError[LLEG][2]) / 2.0;
    wrenchError[RLEG][2] -= averageFzError;
    wrenchError[LLEG][2] -= averageFzError;
  }

  for(int i=0;i<NUM_LEGS;i++){

    cnoid::Vector6 offsetPrev; // generate frame. endEffector origin
    cnoid::Vector6 dOffsetPrev; // generate frame. endEffector origin
    endEffectorParam.stOffset[i].value(offsetPrev, dOffsetPrev);

    cnoid::Matrix3 eeR = cnoid::AngleAxisd(offsetPrev.tail<3>().norm(),(offsetPrev.tail<3>().norm()>0)?offsetPrev.tail<3>().normalized() : cnoid::Vector3::UnitX()) * endEffectorParam.abcTargetPose[i].linear();

    cnoid::Vector6 wrenchErrorLocal; //endEffector frame. endeffector origin
    wrenchErrorLocal.head<3>() = eeR.transpose() * wrenchError[i].head<3>();
    wrenchErrorLocal.tail<3>() = eeR.transpose() * wrenchError[i].tail<3>();
    wrenchErrorLocal = mathutil::clampMatrix<cnoid::Vector6>(wrenchErrorLocal, this->dampingWrenchErrorLimit[i]);

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
    o_stOffset[i].reset(offset, dOffset/dt);
  }


  return true;
}
