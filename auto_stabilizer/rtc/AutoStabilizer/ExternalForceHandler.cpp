#include "ExternalForceHandler.h"

bool ExternalForceHandler::initExternalForceHandlerOutput(const GaitParam& gaitParam,
                                                          double& o_omega, cnoid::Vector3& o_l) const{
  double dz = (gaitParam.footMidCoords.value().inverse() * gaitParam.genCog)[2];
  if(dz <= 0.0){ // 倒立振子近似が成り立たないので破綻する
    o_omega = 1.0;
    o_l = cnoid::Vector3(0.0,0.0,1.0);
    return false;
  }

  o_omega = std::sqrt(gaitParam.g / dz);
  o_l = cnoid::Vector3(0.0,0.0,dz);
  return true;
}

bool ExternalForceHandler::handleExternalForce(const GaitParam& gaitParam, double mass,
                                               double& o_omega, cnoid::Vector3& o_l) const{
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
  cnoid::Vector3 l;
  l[0] = (-sumRefExternalWrench[4] / (mass * gaitParam.g));
  l[1] = (sumRefExternalWrench[3] / (mass * gaitParam.g));
  l[2] = gaitParam.refdz;

  o_omega = omega;
  o_l = l;
  return true;
}
