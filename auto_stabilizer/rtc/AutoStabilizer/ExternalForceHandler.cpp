#include "ExternalForceHandler.h"
#include "MathUtil.h"

bool ExternalForceHandler::initExternalForceHandlerOutput(const GaitParam& gaitParam,
                                                          double& o_omega, cnoid::Vector3& o_l, cnoid::Vector3& o_sbpOffset, cnoid::Vector3& o_genCog) const{
  double dz = gaitParam.genRobot->centerOfMass()[2] - gaitParam.footMidCoords.value().translation()[2];

  /*
    genCog - lの位置まわりのトルクのXY成分が、脚以外の目標反力とgaitParam.genRobot->centerOfMass()に加わる重力の合計がゼロになるように、genCogを決める. そうすると、genCogにsbpOffsetを足したものがgaitParam.genRobot->centerOfMass()になるため、出力が連続的になる
   */
  cnoid::Vector6 feedForwardExternalWrench = cnoid::Vector6::Zero(); // generate frame. generate frame原点(Z座標はfootMidCoords)origin.
  cnoid::Vector3 origin = cnoid::Vector3(0,0,gaitParam.footMidCoords.value().translation()[2]); // generate frame
  feedForwardExternalWrench.head<3>() += - gaitParam.g * gaitParam.genRobot->mass() * cnoid::Vector3::UnitZ();
  feedForwardExternalWrench.tail<3>() += (gaitParam.genRobot->centerOfMass() - origin).cross(- gaitParam.g * gaitParam.genRobot->mass() * cnoid::Vector3::UnitZ());
  for(int i=0; i<gaitParam.eeName.size();i++){
    double ratio = 1.0;
    if(i < NUM_LEGS){ // 脚は、isManualControlModeの場合のみrefEEWrenchに応じて重心をオフセットする
      ratio = gaitParam.isManualControlMode[i].value();
    }
    cnoid::Position eePose = gaitParam.genRobot->link(gaitParam.eeParentLink[i])->T() * gaitParam.eeLocalT[i]; // generate frame
    cnoid::Vector6 eeWrench; /*generate frame. endeffector origin*/
    eeWrench.head<3>() = gaitParam.footMidCoords.value().linear() * gaitParam.refEEWrenchOrigin[i].head<3>();
    eeWrench.tail<3>() = gaitParam.footMidCoords.value().linear() * gaitParam.refEEWrenchOrigin[i].tail<3>();
    feedForwardExternalWrench.head<3>() += ratio * eeWrench.head<3>();
    feedForwardExternalWrench.tail<3>() += ratio * eeWrench.tail<3>();
    feedForwardExternalWrench.tail<3>() += (eePose.translation() - origin).cross(ratio * eeWrench.head<3>());
  }

  feedForwardExternalWrench[2] = std::min(feedForwardExternalWrench[2], (0.5 - 1.0) * gaitParam.genRobot->mass() * gaitParam.g); // 鉛直上向きの外力が自重と比べて大きいと倒立振子近似が成り立たないので計算が破綻する. てきとうに自重の0.5倍まででリミットする

  cnoid::Vector3 genCog = gaitParam.genRobot->centerOfMass();
  genCog[0] = (-feedForwardExternalWrench[4] / feedForwardExternalWrench[2]);
  genCog[1] = (feedForwardExternalWrench[3] / feedForwardExternalWrench[2]);

  double dz_limited = std::max(dz, 0.1); // 倒立振子近似が成り立たないので計算が破綻する. omegaが小さいと計算が不安定になる. 10cm以上とする
  o_omega = std::sqrt((-feedForwardExternalWrench[2]/gaitParam.genRobot->mass()) / dz_limited);
  o_l = cnoid::Vector3(0.0,0.0,dz);
  o_sbpOffset = gaitParam.genRobot->centerOfMass() - genCog;
  o_genCog = genCog;
  return true;
}

bool ExternalForceHandler::handleExternalForce(const GaitParam& gaitParam, bool useActState, double dt,
                                               double& o_omega, cnoid::Vector3& o_l, cnoid::Vector3& o_sbpOffset, cnoid::Vector3& o_actCog) const{

  double omega;
  cnoid::Vector3 l;
  cnoid::Vector3 feedForwardSbpOffset;
  this->handleFeedForwardExternalForce(gaitParam,
                                       omega, l, feedForwardSbpOffset);
  cnoid::Vector3 feedBackSbpOffset;
  this->handleFeedBackExternalForce(gaitParam, useActState, dt, omega, feedForwardSbpOffset,
                                    feedBackSbpOffset);

  o_omega = omega;
  o_l = l;
  o_sbpOffset = feedForwardSbpOffset + feedBackSbpOffset;
  o_actCog = gaitParam.actRobot->centerOfMass() - o_sbpOffset;
  this->isInitial = false;
  return true;
}

bool ExternalForceHandler::handleFeedForwardExternalForce(const GaitParam& gaitParam,
                                                          double& o_omega, cnoid::Vector3& o_l, cnoid::Vector3& o_feedForwardSbpOffset) const{
  /*
    genCog - lの位置まわりのトルクのXY成分が、gaitParam.refEEPoseに加わる脚以外の目標反力とgenRobot->centerOfMass()に加わる重力の合計がゼロになればよいとする(厳密な力の釣り合いを考えるなら、いろいろツッコミどころがあるが...)
    脚以外の目標反力による今のgenCog - lの位置まわりのトルクを計算して、ゼロになっていないぶんだけ、COGのoffsetを動かしてやれば良い.
    脚以外のエンドエフェクタのrefEEPoseは、RefToGenFrameConverterが、HandFixModeで無い場合genCog - lの位置からの相対位置が変化しないように動かすので、歩行中でもoffsetの値は変化しない. 一方HandFixModeの場合,歩行中にgenCog - lの位置からの相対位置が変化するので、offsetの値が変動し、加速度が発生し歩行に影響を与える恐れがある. が、masterのhrpsysもそうなっていたので、実用上問題ないらしい.

    lやsbpOffsetは連続的に変化することが求められている. refEEPoseやrefEEWrenchが不連続に変化すると不連続に変化してしまうので注意.
   */

  cnoid::Vector6 feedForwardExternalWrench = cnoid::Vector6::Zero(); // generate frame. genCog - lの位置origin.
  for(int i=0; i<gaitParam.eeName.size();i++){ // 脚以外の目標反力を足す
    double ratio = 1.0;
    if(i < NUM_LEGS){ // 脚は、isManualControlModeの場合のみrefEEWrenchに応じて重心をオフセットする
      ratio = gaitParam.isManualControlMode[i].value();
    }
    feedForwardExternalWrench.head<3>() += ratio * gaitParam.refEEWrench[i].head<3>()/*generate frame. endeffector origin*/;
    feedForwardExternalWrench.tail<3>() += ratio * gaitParam.refEEWrench[i].tail<3>()/*generate frame. endeffector origin*/;
    cnoid::Vector3 trans = gaitParam.refEEPose[i].translation() - (gaitParam.genCog - gaitParam.l);
    feedForwardExternalWrench.tail<3>() += trans.cross(ratio * gaitParam.refEEWrench[i].head<3>()/*generate frame. endeffector origin*/);
  }

  feedForwardExternalWrench[2] = std::min(feedForwardExternalWrench[2], 0.5 * gaitParam.genRobot->mass() * gaitParam.g); // 鉛直上向きの外力が自重と比べて大きいと倒立振子近似が成り立たないので計算が破綻する. てきとうに自重の0.5倍まででリミットする

  double dz_limited = std::max(gaitParam.refdz, 0.1); // 倒立振子近似が成り立たないので計算が破綻する. omegaが小さいと計算が不安定になる. 10cm以上とする
  double omega = std::sqrt((gaitParam.g-feedForwardExternalWrench[2]/gaitParam.genRobot->mass()) / dz_limited);
  cnoid::Vector3 l = cnoid::Vector3::Zero();
  cnoid::Vector3 feedForwardSbpOffset = cnoid::Vector3::Zero();
  l[2] = gaitParam.refdz;

  feedForwardSbpOffset[0] = (-feedForwardExternalWrench[4] / (gaitParam.genRobot->mass() * gaitParam.g));
  feedForwardSbpOffset[1] = (feedForwardExternalWrench[3] / (gaitParam.genRobot->mass() * gaitParam.g));

  o_omega = omega;
  o_l = l;
  o_feedForwardSbpOffset = feedForwardSbpOffset;

  return true;
}
bool ExternalForceHandler::handleFeedBackExternalForce(const GaitParam& gaitParam, bool useActState, double dt, double omega, const cnoid::Vector3& feedForwardSbpOffset,
                                                       cnoid::Vector3& o_feedBackSbpOffset) const{

  cnoid::Vector3 actCP = gaitParam.actRobot->centerOfMass() + gaitParam.actCogVel.value() / omega; // generate frame. ここではsbpOffsetやlは考えない, 生の重心位置を用いる
  cnoid::Vector3 actCPVel;
  if(this->isInitial) actCPVel = cnoid::Vector3::Zero();
  else actCPVel = (actCP - this->actCPPrev) / dt;
  this->actCPPrev = actCP;

  cnoid::Vector3 targetOffset = cnoid::Vector3::Zero();
  double timeConst = this->disturbanceCompensationTimeConst;
  if(this->useDisturbanceCompensation && useActState){
    if(!gaitParam.isStatic()) {// 非静止状態. (着地の衝撃が大きかったり、右脚と左脚とで誤差ののり方が反対向きになったりするので、一歩ごとに積算する) (逆に静止状態時に、適当に1[s]などで区切って一歩分の誤差として積算すると、BangBang的な挙動をしてしまう)
      cnoid::Vector3 tmpOffset = cnoid::Vector3::Zero(); // 今の外乱の大きさ
      tmpOffset.head<2>() = (actCP - actCPVel / omega - gaitParam.stTargetZmp).head<2>();
      this->disturbance = (this->disturbance * this->disturbanceTime + tmpOffset * dt) / (this->disturbanceTime + dt);
      this->disturbanceTime += dt;
      if(this->disturbanceTime > 0.0 &&
         (gaitParam.prevSupportPhase[RLEG] != gaitParam.footstepNodesList[0].isSupportPhase[RLEG] || gaitParam.prevSupportPhase[LLEG] != gaitParam.footstepNodesList[0].isSupportPhase[LLEG])){ // footStepNodesListの変わり目
        this->disturbanceQueue.emplace_back(this->disturbance, this->disturbanceTime);
        this->disturbance = cnoid::Vector3::Zero();
        this->disturbanceTime = 0.0;
        while(this->disturbanceQueue.size() > this->disturbanceCompensationStepNum) this->disturbanceQueue.pop_front();
      }
    } else { // 静止状態
      cnoid::Vector3 tmpOffset = cnoid::Vector3::Zero(); // 安易に現在の誤差を積分すると不安定になるので、現状は何もしない
      disturbanceQueue = {std::pair<cnoid::Vector3, double>{tmpOffset,1.0}};
      disturbance = cnoid::Vector3::Zero();
      disturbanceTime = 0.0;
    }

    double tm = 0;
    for(std::list<std::pair<cnoid::Vector3, double> >::iterator it = this->disturbanceQueue.begin(); it != this->disturbanceQueue.end(); it++){
      targetOffset += it->first * it->second;
      tm += it->second;
    }
    targetOffset /= tm;
    targetOffset -= feedForwardSbpOffset; // フィードフォワード外乱補償では足りない分のみを扱う

  }else{ // if(this->useDisturbanceCompensation && useActState)
    disturbanceQueue = {std::pair<cnoid::Vector3, double>{cnoid::Vector3::Zero(),1.0}};
    disturbance = cnoid::Vector3::Zero();
    disturbanceTime = 0.0;
  }

  cnoid::Vector3 feedBackSbpOffset = this->feedBackSbpOffsetPrev;
  for(int i=0;i<2;i++){
    double timeConst = this->disturbanceCompensationTimeConst;
    if(std::abs(feedBackSbpOffset[i]) > std::abs(targetOffset[i])) timeConst *= 0.1; // 減る方向には速く
    feedBackSbpOffset[i] += (targetOffset[i] - feedBackSbpOffset[i]) * dt / timeConst;
    feedBackSbpOffset[i] = mathutil::clamp(feedBackSbpOffset[i], this->disturbanceCompensationLimit);
  }
  feedBackSbpOffset[2] = 0.0;
  this->feedBackSbpOffsetPrev = feedBackSbpOffset;

  o_feedBackSbpOffset = feedBackSbpOffset;
  return true;
}
