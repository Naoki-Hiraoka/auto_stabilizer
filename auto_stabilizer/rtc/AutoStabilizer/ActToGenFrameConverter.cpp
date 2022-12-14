#include "ActToGenFrameConverter.h"
#include "CnoidBodyUtil.h"
#include "MathUtil.h"
#include <cnoid/ForceSensor>
#include <cnoid/EigenUtil>

void ActToGenFrameConverter::initOutput(const GaitParam& gaitParam, // input
                                        cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3>& o_actCogVel, cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6>& o_actRootVel) const {
  o_actCogVel.reset(cnoid::Vector3::Zero());
  o_actRootVel.reset(cnoid::Vector6::Zero());
  return;
}

bool ActToGenFrameConverter::convertFrame(const GaitParam& gaitParam, double dt,// input
                                          cnoid::BodyPtr& actRobot, std::vector<cnoid::Position>& o_actEEPose, std::vector<cnoid::Vector6>& o_actEEWrench, cnoid::Vector3& o_actCog, cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3>& o_actCogVel, cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6>& o_actRootVel) const {

  cnoid::Vector3 actCogPrev = actRobot->centerOfMass();
  cnoid::Position actRootPrev = actRobot->rootLink()->T();

  {
    // FootOrigin座標系を用いてactRobotRawをgenerate frameに投影しactRobotとする
    cnoidbodyutil::copyRobotState(gaitParam.actRobotRaw, actRobot);
    actRobot->rootLink()->R() = (actRobot->rootLink()->R() * cnoid::rotFromRpy(this->rpyOffset)).eval(); // rpyOffsetを適用
    actRobot->calcForwardKinematics();
    cnoid::DeviceList<cnoid::ForceSensor> actForceSensors(gaitParam.actRobotRaw->devices());
    cnoid::DeviceList<cnoid::ForceSensor> actOriginForceSensors(actRobot->devices());
    for(int i=0;i<actForceSensors.size();i++){
      actOriginForceSensors[i]->F() = actForceSensors[i]->F();
    }
    double rlegweight = gaitParam.footstepNodesList[0].isSupportPhase[RLEG]? 1.0 : 0.0;
    double llegweight = gaitParam.footstepNodesList[0].isSupportPhase[LLEG]? 1.0 : 0.0;
    if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) rlegweight = llegweight = 1.0;
    cnoid::Position actrleg = actRobot->link(gaitParam.eeParentLink[RLEG])->T()*gaitParam.eeLocalT[RLEG];
    cnoid::Position actlleg = actRobot->link(gaitParam.eeParentLink[LLEG])->T()*gaitParam.eeLocalT[LLEG];
    cnoid::Position actFootMidCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{actrleg, actlleg},
                                                               std::vector<double>{rlegweight, llegweight});
    cnoid::Position actFootOriginCoords = mathutil::orientCoordToAxis(actFootMidCoords, cnoid::Vector3::UnitZ());
    cnoid::Position genFootMidCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{gaitParam.abcEETargetPose[RLEG], gaitParam.abcEETargetPose[LLEG]},
                                                               std::vector<double>{rlegweight, llegweight});  // 1周期前のabcTargetPoseを使っているが、abcTargetPoseは不連続に変化するものではないのでよい
    cnoid::Position genFootOriginCoords = mathutil::orientCoordToAxis(genFootMidCoords, cnoid::Vector3::UnitZ());
    cnoidbodyutil::moveCoords(actRobot, genFootOriginCoords, actFootOriginCoords);
    actRobot->calcForwardKinematics();
    actRobot->calcCenterOfMass();
  }

  std::vector<cnoid::Position> actEEPose(gaitParam.eeName.size(), cnoid::Position::Identity());
  std::vector<cnoid::Vector6> actEEWrench(gaitParam.eeName.size(), cnoid::Vector6::Zero());
  {
    // 各エンドエフェクタのactualの位置・力を計算
    for(int i=0;i<gaitParam.eeName.size(); i++){
      actEEPose[i] = actRobot->link(gaitParam.eeParentLink[i])->T() * gaitParam.eeLocalT[i];
      if(this->eeForceSensor[i] != ""){
        cnoid::ForceSensorPtr sensor = actRobot->findDevice<cnoid::ForceSensor>(this->eeForceSensor[i]);
        cnoid::Vector6 senF = sensor->F();
        cnoid::Position senPose = sensor->link()->T() * sensor->T_local();
        cnoid::Position eefTosenPose = gaitParam.actEEPose[i].inverse() * senPose;
        cnoid::Vector6 eefF; // endeffector frame. endeffector origin.
        eefF.head<3>() = eefTosenPose.linear() * senF.head<3>();
        eefF.tail<3>() = eefTosenPose.linear() * senF.tail<3>() + eefTosenPose.translation().cross(eefF.head<3>());
        actEEWrench[i].head<3>() = gaitParam.actEEPose[i].linear() * eefF.head<3>();
        actEEWrench[i].tail<3>() = gaitParam.actEEPose[i].linear() * eefF.tail<3>();
      }
    }
  }

  cnoid::Vector3 actCogVel;
  cnoid::Vector6 actRootVel;
  {
    bool genContactState_changed = false;
    for(int i=0;i<NUM_LEGS;i++){
      if(gaitParam.footstepNodesList[0].isSupportPhase[i] != gaitParam.prevSupportPhase[i]) genContactState_changed = true;
    }
    // actCogを計算
    if(this->isInitial || genContactState_changed){
      //座標系が飛んでいるので、gaitParam.actCogVel は前回の周期の値をそのままつかう
      actCogVel = gaitParam.actCogVel.value();
      actRootVel = gaitParam.actRootVel.value();
    }else{
      actCogVel = (actRobot->centerOfMass() - actCogPrev) / dt;
      actRootVel.head<3>() = (actRobot->rootLink()->translation() - actRootPrev.translation()) / dt;
      Eigen::AngleAxisd actRootdR(actRobot->rootLink()->R() * actRootPrev.linear().transpose()); // generate frame. rootlink origin.
      actRootVel.tail<3>() = actRootdR.angle() / dt * actRootdR.axis();
    }
  }

  o_actRootVel.passFilter(actRootVel, dt);
  actRobot->rootLink()->v() = o_actRootVel.value().head<3>();
  actRobot->rootLink()->w() = o_actRootVel.value().tail<3>();
  actRobot->calcForwardKinematics(true);

  o_actEEPose = actEEPose;
  o_actEEWrench = actEEWrench;
  o_actCog = actRobot->centerOfMass();
  o_actCogVel.passFilter(actCogVel, dt);

  this->isInitial = false;
  return true;
}
