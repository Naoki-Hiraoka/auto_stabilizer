#include "ActToGenFrameConverter.h"
#include "CnoidBodyUtil.h"
#include "MathUtil.h"
#include <cnoid/ForceSensor>

bool ActToGenFrameConverter::convertFrame(const cnoid::BodyPtr& actRobotRaw, const EndEffectorParam& endEffectorParams, const GaitParam& gaitParam, double dt,// input
                                          cnoid::BodyPtr& actRobot, std::vector<cnoid::Position>& o_actPose, std::vector<cnoid::Vector6>& o_actWrench, cnoid::Vector3& o_actCog, cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3>& o_actCogVel) const {

  {
    // FootOrigin座標系を用いてactRobotRawをgenerate frameに投影しactRobotとする
    cnoidbodyutil::copyRobotState(actRobotRaw, actRobot);
    cnoid::DeviceList<cnoid::ForceSensor> actForceSensors(actRobotRaw->devices());
    cnoid::DeviceList<cnoid::ForceSensor> actOriginForceSensors(actRobot->devices());
    for(int i=0;i<actForceSensors.size();i++){
      actOriginForceSensors[i]->F() = actForceSensors[i]->F();
    }
    double rlegweight = gaitParam.isSupportPhase(RLEG)? 1.0 : 0.0;
    double llegweight = gaitParam.isSupportPhase(LLEG)? 1.0 : 0.0;
    if(!gaitParam.isSupportPhase(RLEG) && !gaitParam.isSupportPhase(LLEG)) rlegweight = llegweight = 1.0;
    cnoid::Position actrleg = actRobot->link(endEffectorParams.parentLink[RLEG])->T()*endEffectorParams.localT[RLEG];
    cnoid::Position actlleg = actRobot->link(endEffectorParams.parentLink[LLEG])->T()*endEffectorParams.localT[LLEG];
    cnoid::Position actFootMidCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{actrleg, actlleg},
                                                               std::vector<double>{rlegweight, llegweight});
    cnoid::Position actFootOriginCoords = mathutil::orientCoordToAxis(actFootMidCoords, cnoid::Vector3::UnitZ());
    cnoid::Position genFootMidCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{endEffectorParams.abcTargetPose[RLEG], endEffectorParams.abcTargetPose[LLEG]},
                                                               std::vector<double>{rlegweight, llegweight});  // 1周期前のabcTargetPoseを使っているが、abcTargetPoseは不連続に変化するものではないのでよい
    cnoid::Position genFootOriginCoords = mathutil::orientCoordToAxis(genFootMidCoords, cnoid::Vector3::UnitZ());
    cnoidbodyutil::moveCoords(actRobot, genFootOriginCoords, actFootOriginCoords);
    actRobot->calcForwardKinematics();
    actRobot->calcCenterOfMass();
  }

  std::vector<cnoid::Position> actPose(endEffectorParams.name.size(), cnoid::Position::Identity());
  std::vector<cnoid::Vector6> actWrench(endEffectorParams.name.size(), cnoid::Vector6::Zero());
  {
    // 各エンドエフェクタのactualの位置・力を計算
    for(int i=0;i<endEffectorParams.name.size(); i++){
      actPose[i] = actRobot->link(endEffectorParams.parentLink[i])->T() * endEffectorParams.localT[i];
      if(endEffectorParams.forceSensor[i] != ""){
        cnoid::ForceSensorPtr sensor = actRobot->findDevice<cnoid::ForceSensor>(endEffectorParams.forceSensor[i]);
        cnoid::Vector6 senF = sensor->F();
        cnoid::Position senPose = sensor->link()->T() * sensor->T_local();
        cnoid::Position eefTosenPose = endEffectorParams.actPose[i].inverse() * senPose;
        cnoid::Vector6 eefF; // endeffector frame. endeffector origin.
        eefF.head<3>() = eefTosenPose.linear() * senF.head<3>();
        eefF.tail<3>() = eefTosenPose.linear() * senF.tail<3>() + eefTosenPose.translation().cross(eefF.head<3>());
        actWrench[i].head<3>() = endEffectorParams.actPose[i].linear() * eefF.head<3>();
        actWrench[i].tail<3>() = endEffectorParams.actPose[i].linear() * eefF.tail<3>();
      }
    }
  }

  cnoid::Vector3 actCog = actRobot->centerOfMass();
  cnoid::Vector3 actCogVel;
  {
    // actCogを計算
    bool genContactState_changed = false;
    for(int i=0;i<NUM_LEGS;i++){
      if(gaitParam.isSupportPhase(i) != gaitParam.prevSupportPhase[i]) genContactState_changed = true;
    }
    if(genContactState_changed){
      //座標系が飛んでいるので、gaitParam.actCogVel は前回の周期の値をそのままつかう
      actCogVel = gaitParam.actCogVel.value();
    }else{
      actCogVel = (actCog - gaitParam.actCog) / dt;
    }
  }

  o_actPose = actPose;
  o_actWrench = actWrench;
  o_actCog = actCog;
  if(this->isInitial){
    o_actCogVel.reset(cnoid::Vector3::Zero());
  } else {
    o_actCogVel.passFilter(actCogVel, dt);
  }

  this->isInitial = false;

  return true;
}
