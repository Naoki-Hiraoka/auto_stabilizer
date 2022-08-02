#include "Stabilizer.h"
#include "MathUtil.h"
#include <cnoid/JointPath>
#include <cnoid/Jacobian>

bool Stabilizer::execStabilizer(const cnoid::BodyPtr refRobotOrigin, const cnoid::BodyPtr actRobotOrigin, const cnoid::BodyPtr genRobot, const GaitParam& gaitParam, const EndEffectorParam& endEffectorParam, double dt, double g, double mass,
                                cnoid::BodyPtr& genRobotTqc, cnoid::Vector3& o_dRootRpy) const{
  // - root attitude control
  // - 現在のactual重心位置から、目標ZMPを計算
  // - 目標ZMPを満たすように目標足裏反力を計算
  // - 目標足裏反力を満たすようにDamping Control. 目標手先反力を満たすようにImpedance Control. 目標反力を満たすように重力補償+仮想仕事の原理

  // root attitude control
  cnoid::Vector3 dRootRpy; // gaitParam.footMidCoords座標系.
  this->moveBasePosRotForBodyRPYControl(refRobotOrigin, actRobotOrigin, dt, gaitParam, // input
                                        dRootRpy); // output

  // 現在のactual重心位置から、目標ZMPを計算
  cnoid::Vector3 tgtZmp; // generate frame
  cnoid::Vector3 tgtForce; // generate frame
  this->calcZMP(gaitParam, dt, g, mass, // input
                tgtZmp, tgtForce); // output

  // 目標ZMPを満たすように目標EndEffector反力を計算
  std::vector<cnoid::Vector6> tgtWrench; // 要素数EndEffector数. generate frame. EndEffector origin
  this->calcWrench(gaitParam, endEffectorParam, tgtZmp, tgtForce, // input
                   tgtWrench); // output

  this->calcTorque(actRobotOrigin, dt, endEffectorParam, tgtWrench, // input
                   genRobotTqc); // output

  o_dRootRpy = dRootRpy;
  return true;
}

bool Stabilizer::moveBasePosRotForBodyRPYControl(const cnoid::BodyPtr refRobotOrigin, const cnoid::BodyPtr actRobotOrigin, double dt, const GaitParam& gaitParam,
                                                 cnoid::Vector3& o_dRootRpy) const{
  cnoid::Vector3 dRootRpy = gaitParam.dRootRpy; // gaitParam.footMidCoords座標系

  cnoid::Matrix3 rootRError = gaitParam.footMidCoords.value().linear().transpose() * (refRobotOrigin->rootLink()->R() * actRobotOrigin->rootLink()->R().transpose()); // gaitParam.footMidCoords座標系
  cnoid::Vector3 rootRpyError = cnoid::rpyFromRot(rootRError); // gaitParam.footMidCoords座標系

  for (size_t i = 0; i < 2; i++) {
    dRootRpy[i] += (this->bodyAttitudeControlGain[i] * rootRpyError[i] - 1.0/this->bodyAttitudeControlTimeConst[i] * dRootRpy[i]) * dt;
    dRootRpy[i] = mathutil::clamp(dRootRpy[i], this->rootRotCompensationLimit[i]);
  }
  dRootRpy[2] = 0.0;

  o_dRootRpy = dRootRpy;
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
    1. ノルム>0
    2. 合力がtgtForce, ZMPがtgtZmp
    3. 各Legについて、各頂点のノルムの重心がCOPOffsetと一致
    4. ノルムの2乗和の最小化
  */

  // TODO

  o_tgtWrench = tgtWrench;
  return true;
}

bool Stabilizer::calcTorque(const cnoid::BodyPtr actRobotOrigin, double dt, const EndEffectorParam& endEffectorParam, const std::vector<cnoid::Vector6>& tgtWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/,
                            cnoid::BodyPtr& genRobotTqc) const{
  // 速度・加速度を考慮しない重力補償
  genRobotTqc->rootLink()->T() = actRobotOrigin->rootLink()->T();
  genRobotTqc->rootLink()->v() = cnoid::Vector3::Zero();
  genRobotTqc->rootLink()->w() = cnoid::Vector3::Zero();
  genRobotTqc->rootLink()->dv() = cnoid::Vector3::Zero();
  genRobotTqc->rootLink()->dw() = cnoid::Vector3::Zero();
  for(int i=0;i<genRobotTqc->numJoints();i++){
    genRobotTqc->joint(i)->q() = actRobotOrigin->joint(i)->q();
    genRobotTqc->joint(i)->dq() = 0.0;
    genRobotTqc->joint(i)->ddq() = 0.0;
  }
  genRobotTqc->calcForwardKinematics(true, true); // genRobotTqc->joint()->u()に書き込まれる

  // tgtWrench
  for(int i=0;i<endEffectorParam.name.size();i++){
    cnoid::JointPath jointPath(genRobotTqc->rootLink(), genRobotTqc->link(endEffectorParam.parentLink[i]));
    cnoid::MatrixXd J = cnoid::MatrixXd::Zero(6,jointPath.numJoints()); // generate frame. endeffector origin
    cnoid::setJacobian<0x3f,0,0,true>(jointPath,genRobotTqc->link(endEffectorParam.parentLink[i]),endEffectorParam.localT[i].translation(), // input
                                      J); // output
    cnoid::VectorX tau = - J.transpose() * tgtWrench[i];
    for(int j=0;j<jointPath.numJoints();j++){
      jointPath.joint(j)->u() += tau[j];
    }
  }

  return true;
}

bool Stabilizer::calcDampingControl(double dt, const EndEffectorParam& endEffectorParam, const std::vector<cnoid::Vector6>& tgtWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/,
                                    std::vector<cnoid::Position>& o_tgtPose /*generate frame*/) const{
  std::vector<cnoid::Position> tgtPose = endEffectorParam.abcTargetPose; /*generate frame*/

  

  return true;
}
