#include "LegManualController.h"
#include "MathUtil.h"

bool LegManualController::legManualControl(const GaitParam& gaitParam, double dt,
                                           std::vector<cpp_filters::TwoPointInterpolatorSE3>& o_genCoords, std::vector<GaitParam::FootStepNodes>& o_footstepNodesList, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_isManualControlMode) const{
  for(int i=0;i<NUM_LEGS;i++){
    if(!gaitParam.isStatic() || gaitParam.footstepNodesList[0].isSupportPhase[i]){ // 静止状態で無い場合や、支持脚の場合は、勝手にManualControlはオフになる
      if(o_isManualControlMode[i].getGoal() != 0.0) o_isManualControlMode[i].setGoal(0.0, 2.0); // 2.0[s]で遷移
    }else{
      if(o_isManualControlMode[i].getGoal() == 1.0){ // Manual Control on
        cnoid::Position nextCoords;
        if(o_isManualControlMode[i].isEmpty()){
          nextCoords = gaitParam.icEETargetPose[i];
        }else{
          nextCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{gaitParam.icEETargetPose[i], gaitParam.genCoords[i].value()},
                                               std::vector<double>{o_isManualControlMode[i].value(), 1.0 - o_isManualControlMode[i].value()});
        }
        o_genCoords[i].reset(nextCoords);
        o_footstepNodesList[0].dstCoords[i] = nextCoords;
      }
    }
  }

  return true;
}
