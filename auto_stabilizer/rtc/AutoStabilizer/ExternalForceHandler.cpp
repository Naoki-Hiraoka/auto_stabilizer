#include "ExternalForceHandler.h"
#include "MathUtil.h"

bool ExternalForceHandler::initExternalForceHandlerOutput(const GaitParam& gaitParam, const cnoid::BodyPtr& genRobot,
                                                          double& o_omega, cnoid::Vector3& o_l, cnoid::Vector3& o_sbpOffset, cnoid::Vector3& o_genCog) const{
  double dz = genRobot->centerOfMass()[2] - gaitParam.footMidCoords.value().translation()[2];

  /*
    genCog - lの位置まわりのトルクのXY成分が、脚以外の目標反力とgenRobot->centerOfMass()に加わる重力の合計がゼロになるように、genCogを決める. そうすると、genCogにsbpOffsetを足したものがgenRobot->centerOfMass()になるため、出力が連続的になる
   */
  cnoid::Vector6 feedForwardExternalWrench = cnoid::Vector6::Zero(); // generate frame. generate frame原点(Z座標はfootMidCoords)origin.
  cnoid::Vector3 origin = cnoid::Vector3(0,0,gaitParam.footMidCoords.value().translation()[2]); // generate frame
  feedForwardExternalWrench.head<3>() += - gaitParam.g * genRobot->mass() * cnoid::Vector3::UnitZ();
  feedForwardExternalWrench.tail<3>() += (genRobot->centerOfMass() - origin).cross(- gaitParam.g * genRobot->mass() * cnoid::Vector3::UnitZ());
  for(int i=NUM_LEGS; i<gaitParam.eeName.size();i++){ // 脚以外の目標反力を足す
    cnoid::Position eePose = genRobot->link(gaitParam.eeParentLink[i])->T() * gaitParam.eeLocalT[i]; // generate frame
    cnoid::Vector6 eeWrench; /*generate frame. endeffector origin*/
    eeWrench.head<3>() = gaitParam.footMidCoords.value().linear() * gaitParam.refEEWrenchOrigin[i].head<3>();
    eeWrench.tail<3>() = gaitParam.footMidCoords.value().linear() * gaitParam.refEEWrenchOrigin[i].tail<3>();
    feedForwardExternalWrench.head<3>() += eeWrench.head<3>();
    feedForwardExternalWrench.tail<3>() += eeWrench.tail<3>();
    feedForwardExternalWrench.tail<3>() += (eePose.translation() - origin).cross(eeWrench.head<3>());
  }

  if(dz <= 0.0 || feedForwardExternalWrench[2] >= 0.0) { // 倒立振子近似が成り立たないので計算が破綻する.
    o_omega = 1.0;
    o_l = cnoid::Vector3(0.0,0.0,1.0);
    o_sbpOffset.setZero();
    o_genCog = genRobot->centerOfMass();
    return false;
  }

  cnoid::Vector3 genCog = genRobot->centerOfMass();
  genCog[0] = (-feedForwardExternalWrench[4] / feedForwardExternalWrench[2]);
  genCog[1] = (feedForwardExternalWrench[3] / feedForwardExternalWrench[2]);

  o_omega = std::sqrt((-feedForwardExternalWrench[2]/genRobot->mass()) / dz);
  o_l = cnoid::Vector3(0.0,0.0,dz);
  o_sbpOffset = genRobot->centerOfMass() - genCog;
  o_genCog = genCog;
  return true;
}

bool ExternalForceHandler::handleExternalForce(const GaitParam& gaitParam, double mass, const cnoid::BodyPtr& actRobot, bool useActState, double dt,
                                               double& o_omega, cnoid::Vector3& o_l, cnoid::Vector3& o_sbpOffset, cnoid::Vector3& o_actCog) const{
  /*
    genCog - lの位置まわりのトルクのXY成分が、脚以外の目標反力と重心に加わる重力の合計がゼロになればよいとする(厳密な力の釣り合いを考えるなら、全く正確ではないが...)
    脚以外のエンドエフェクタは、RefToGenFrameConverterで、genCog - lの位置からの相対位置が変化しないように動く.(HandFixModeは考えない).
    よって、脚以外の目標反力による今のgenCog - lの位置まわりのトルクを計算して、ゼロになっていないぶんだけ、COGのoffsetを動かしてやれば良い.

    lやsbpOffsetは連続的に変化することが求められている. refEEPoseやrefEEWrenchが不連続に変化すると不連続に変化してしまうので注意. startAutoBalancer直後の初回は、refEEWrenchが非ゼロの場合に不連続に変化することは避けられない. startAutoBalancerのtransition_timeで補間してごまかす.
   */

  cnoid::Vector6 feedForwardExternalWrench = cnoid::Vector6::Zero(); // generate frame. genCog - lの位置origin.
  for(int i=NUM_LEGS; i<gaitParam.eeName.size();i++){ // 脚以外の目標反力を足す
    feedForwardExternalWrench.head<3>() += gaitParam.refEEWrench[i].head<3>()/*generate frame. endeffector origin*/;
    feedForwardExternalWrench.tail<3>() += gaitParam.refEEWrench[i].tail<3>()/*generate frame. endeffector origin*/;
    cnoid::Vector3 trans = gaitParam.refEEPose[i].translation() - (gaitParam.genCog - gaitParam.l);
    feedForwardExternalWrench.tail<3>() += trans.cross(gaitParam.refEEWrench[i].head<3>()/*generate frame. endeffector origin*/);
  }

  if(feedForwardExternalWrench[2] >= mass * gaitParam.g) { // 倒立振子近似が成り立たないので計算が破綻する. 前回の値をそのまま使う
    return false;
  }

  double omega = std::sqrt((gaitParam.g-feedForwardExternalWrench[2]/mass) / gaitParam.refdz);
  cnoid::Vector3 l = cnoid::Vector3::Zero();
  cnoid::Vector3 sbpOffset = cnoid::Vector3::Zero();
  l[2] = gaitParam.refdz;

  // フィードフォワード外乱補償
  sbpOffset[0] = (-feedForwardExternalWrench[4] / (mass * gaitParam.g));
  sbpOffset[1] = (feedForwardExternalWrench[3] / (mass * gaitParam.g));

  // 長期的外乱補償
  {
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
       ((gaitParam.isStatic() && this->disturbanceTime >= this->disturbanceCompensationStaticTime) || // 歩いていないときに、この時間で区切ってステップとする
        (gaitParam.prevSupportPhase[RLEG] != gaitParam.footstepNodesList[0].isSupportPhase[RLEG] || gaitParam.prevSupportPhase[LLEG] != gaitParam.footstepNodesList[0].isSupportPhase[LLEG]))){ // footStepNodesListの変わり目
      this->disturbanceQueue.emplace_back(this->disturbance, this->disturbanceTime);
      this->disturbance = cnoid::Vector3::Zero();
      this->disturbanceTime = 0.0;
      while(this->disturbanceQueue.size() > this->disturbanceCompensationStepNum) this->disturbanceQueue.pop_front();
    }

    cnoid::Vector3 targetOffset = cnoid::Vector3::Zero();
    if(this->useDisturbanceCompensation && useActState){
      double tm = 0;
      for(std::list<std::pair<cnoid::Vector3, double> >::iterator it = this->disturbanceQueue.begin(); it != this->disturbanceQueue.end(); it++){
        targetOffset += it->first * it->second;
        tm += it->second;
      }
      targetOffset /= tm;
      targetOffset -= sbpOffset; // フィードフォワード外乱補償では足りない分のみを扱う
    }

    cnoid::Vector3 offset = this->offsetPrev;
    for(int i=0;i<2;i++){
      double timeConst = this->disturbanceCompensationTimeConst;
      if(std::abs(offset[i]) > std::abs(targetOffset[i])) timeConst *= 0.1;
      offset[i] += (targetOffset[i] - offset[i]) * dt / timeConst;
      offset[i] = mathutil::clamp(offset[i], this->disturbanceCompensationLimit);
    }
    offset[2] = 0.0;
    sbpOffset += offset; // フィードフォワード外乱補償に足す
    this->offsetPrev = offset;
  }

  o_omega = omega;
  o_l = l;
  o_sbpOffset = sbpOffset;
  o_actCog = actRobot->centerOfMass() - sbpOffset;
  this->isInitial = false;
  return true;
}
