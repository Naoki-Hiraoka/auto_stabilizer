#include "ExternalForceHandler.h"
#include "MathUtil.h"

bool ExternalForceHandler::initExternalForceHandlerOutput(const GaitParam& gaitParam,
                                                          double& o_omega, cnoid::Vector3& o_l, cnoid::Vector3& o_sbpOffset) const{
  double dz = (gaitParam.footMidCoords.value().inverse() * gaitParam.genCog)[2];
  if(dz <= 0.0){ // 倒立振子近似が成り立たないので破綻する
    o_omega = 1.0;
    o_l = cnoid::Vector3(0.0,0.0,1.0);
    o_sbpOffset.setZero();
    return false;
  }

  o_omega = std::sqrt(gaitParam.g / dz);
  o_l = cnoid::Vector3(0.0,0.0,dz);
  o_sbpOffset.setZero();
  return true;
}

bool ExternalForceHandler::handleExternalForce(const GaitParam& gaitParam, double mass, const cnoid::BodyPtr& actRobot, bool useActState, double dt,
                                               double& o_omega, cnoid::Vector3& o_l, cnoid::Vector3& o_sbpOffset, cnoid::Vector3& o_actCog) const{
  /*
    COG - lの位置まわりのトルクのXY成分が、脚以外の目標反力と重心に加わる重力の合計がゼロになればよいとする(厳密な力の釣り合いを考えるなら、全く正確ではないが...)
    脚以外のエンドエフェクタは、RefToGenFrameConverterで、COG - lの位置からの相対位置が変化しないように動く.(HandFixModeは考えない).
    よって、脚以外の目標反力による今のCOG - lの位置まわりのトルクを計算して、ゼロになっていないぶんだけ、COGを動かしてやれば良い.

    lは連続的に変化することが求められている(特にRefToGenFrameConverter). refEEPoseやrefEEWrenchが不連続に変化すると不連続に変化してしまうので注意. startAutoBalancer直後の初回は、refEEWrenchが非ゼロの場合に不連続に変化することは避けられない. startAutoBalancerのtransition_timeで補間してごまかす.
   */

  cnoid::Vector6 sumRefExternalWrench = cnoid::Vector6::Zero(); // generate frame. generate frame原点origin.
  for(int i=NUM_LEGS; i<gaitParam.eeName.size();i++){ // 脚以外の目標反力を足す
    sumRefExternalWrench.head<3>() += gaitParam.refEEWrench[i].head<3>()/*generate frame. endeffector origin*/;
    sumRefExternalWrench.tail<3>() += gaitParam.refEEWrench[i].tail<3>()/*generate frame. endeffector origin*/;
    cnoid::Vector3 trans = gaitParam.refEEPose[i].translation() - (gaitParam.genCog - gaitParam.l);
    sumRefExternalWrench.tail<3>() += trans.cross(gaitParam.refEEWrench[i].head<3>()/*generate frame. endeffector origin*/);
  }

  if(sumRefExternalWrench[2] >= mass * gaitParam.g) { // 倒立振子近似が成り立たないので計算が破綻する. 前回の値をそのまま使う
    o_omega = gaitParam.omega;
    o_l = gaitParam.l;
    return false;
  }

  double omega = std::sqrt((gaitParam.g-sumRefExternalWrench[2]/mass) / gaitParam.refdz);
  cnoid::Vector3 l = cnoid::Vector3::Zero();
  cnoid::Vector3 sbpOffset = cnoid::Vector3::Zero();
  l[2] = gaitParam.refdz;

  l[0] = (-sumRefExternalWrench[4] / (mass * gaitParam.g));
  l[1] = (sumRefExternalWrench[3] / (mass * gaitParam.g));

  cnoid::Vector3 actCP = actRobot->centerOfMass() + gaitParam.actCogVel.value() / omega; // generate frame. ここではsbpOffsetやlは考えない, 生の重心位置を用いる
  cnoid::Vector3 actCPVel;
  if(this->isInitial) actCPVel = cnoid::Vector3::Zero();
  else actCPVel = (actCP - this->actCPPrev) / dt;
  this->actCPPrev = actCP;
  cnoid::Vector3 tmpOffset = cnoid::Vector3::Zero();
  tmpOffset.head<2>() = (actCP - actCPVel / omega - gaitParam.stTargetZmp).head<2>();
  this->disturbance = (this->disturbance * this->disturbanceTime + tmpOffset * dt) / (this->disturbanceTime + dt);
  this->disturbanceTime += dt;
  if(this->disturbanceTime > 0.0 &&
     ((gaitParam.isStatic() && this->disturbanceTime >= this->disturbanceCompensationStaticTime) ||
      (gaitParam.prevSupportPhase[RLEG] != gaitParam.footstepNodesList[0].isSupportPhase[RLEG] || gaitParam.prevSupportPhase[LLEG] != gaitParam.footstepNodesList[0].isSupportPhase[LLEG]))){
    this->disturbanceQueue.emplace_back(this->disturbance, this->disturbanceTime);
    this->disturbance = cnoid::Vector3::Zero();
    this->disturbanceTime = 0.0;
    while(this->disturbanceQueue.size() > this->disturbanceCompensationStepNum) this->disturbanceQueue.pop_front();
  }

  if(this->useDisturbanceCompensation){
    cnoid::Vector3 average = cnoid::Vector3::Zero();
    double tm = 0;
    for(std::list<std::pair<cnoid::Vector3, double> >::iterator it = this->disturbanceQueue.begin(); it != this->disturbanceQueue.end(); it++){
      average += it->first * it->second;
      tm += it->second;
    }
    average /= tm;

    cnoid::Vector3 offset = offsetPrev;
    for(int i=0;i<2;i++){
      double timeConst = this->dcTimeConst;
      if(std::abs(offset[i]) > std::abs(average[i])) timeConst *= 0.1;
      offset[i] += (average[i] - offset[i]) * dt / timeConst;
    }

    sbpOffset += offset;
    offsetPrev = offset;

    std::cerr << offset.transpose() << std::endl;
  }

  o_omega = omega;
  o_l = l;
  o_sbpOffset = sbpOffset;
  o_actCog = actRobot->centerOfMass() - sbpOffset;
  this->isInitial = false;
  return true;
}
