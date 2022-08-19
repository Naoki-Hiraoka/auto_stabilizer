#include "CmdVelGenerator.h"
#include "MathUtil.h"

bool CmdVelGenerator::calcCmdVel(const GaitParam& gaitParam,
                                 cnoid::Vector3& o_cmdVel) const{

  cnoid::Vector3 cmdVel = this->refCmdVel;

  if(this->isGraspLessManipMode) {
    cnoid::Vector3 graspLessCmdVel = cnoid::Vector3::Zero();
    this->calcVelFromHandError(gaitParam,
                               graspLessCmdVel);
    cmdVel += graspLessCmdVel;
  }

  o_cmdVel = cmdVel;
  return true;
}


void CmdVelGenerator::calcVelFromHandError(const GaitParam& gaitParam,
                                           cnoid::Vector3& o_graspLessCmdVel) const{
  cnoid::Vector3 graspLessCmdVel = cnoid::Vector3::Zero(); // footPos frame. footPos origin

  cnoid::Position footPos; // generate frame. 次の一歩の支持脚のCoords-defaultTranslatePos. Z軸は鉛直. Z=0.0
  if(gaitParam.footstepNodesList.size() > 1 &&
     (gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG])){ // 現在両足支持期.
    if(!gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[1].isSupportPhase[LLEG]){ // 次右脚をswing
      footPos = mathutil::orientCoordToAxis(gaitParam.footstepNodesList[0].dstCoords[LLEG], cnoid::Vector3::UnitZ());
      footPos.translation() -= footPos.linear() * gaitParam.defaultTranslatePos[LLEG].value();
    }else if(gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[1].isSupportPhase[LLEG]){ // 次左脚をswing
      footPos = mathutil::orientCoordToAxis(gaitParam.footstepNodesList[0].dstCoords[RLEG], cnoid::Vector3::UnitZ());
      footPos.translation() -= footPos.linear() * gaitParam.defaultTranslatePos[RLEG].value();
    }else{
      o_graspLessCmdVel = graspLessCmdVel;
      return;
    }
  }else if(gaitParam.footstepNodesList.size() > 2 &&
           (gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[1].isSupportPhase[LLEG])){ // 次両足支持期
    if(!gaitParam.footstepNodesList[2].isSupportPhase[RLEG] && gaitParam.footstepNodesList[2].isSupportPhase[LLEG]){ // 次の次右脚をswing
      footPos = mathutil::orientCoordToAxis(gaitParam.footstepNodesList[1].dstCoords[LLEG], cnoid::Vector3::UnitZ());
      footPos.translation() -= footPos.linear() * gaitParam.defaultTranslatePos[LLEG].value();
    }else if(gaitParam.footstepNodesList[2].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[2].isSupportPhase[LLEG]){ // 次の次左脚をswing
      footPos = mathutil::orientCoordToAxis(gaitParam.footstepNodesList[1].dstCoords[RLEG], cnoid::Vector3::UnitZ());
      footPos.translation() -= footPos.linear() * gaitParam.defaultTranslatePos[RLEG].value();
    }else{
      o_graspLessCmdVel = graspLessCmdVel;
      return;
    }
  }else{
    o_graspLessCmdVel = graspLessCmdVel;
    return;
  }
  footPos.translation()[2] = 0.0;

  cnoid::Vector3 handPos; // generate frame. handPos[2]=0.0
  cnoid::Vector3 dp; // generate frame. dp[2] = 0.0
  cnoid::Vector3 dr; // generate frame. handPos origin. dr[0] = dr[1] = 0.0
  if(this->graspLessManipArm.size() == 1){
    handPos = gaitParam.icEETargetPose[this->graspLessManipArm[0]].translation();
    dp = gaitParam.icEEOffset[this->graspLessManipArm[0]].value().head<3>();
    dp[2] = 0.0;
    dr = gaitParam.icEEOffset[this->graspLessManipArm[0]].value().tail<3>();
    dr[0] = 0.0; dr[1] = 0.0;
  }else if(this->graspLessManipArm.size() == 2){
    handPos = 0.5 * (gaitParam.icEETargetPose[this->graspLessManipArm[0]].translation() + gaitParam.icEETargetPose[this->graspLessManipArm[1]].translation());
    dp = 0.5 * (gaitParam.icEEOffset[this->graspLessManipArm[0]].value().head<3>() + gaitParam.icEEOffset[this->graspLessManipArm[1]].value().head<3>());
    cnoid::Vector3 ref1to0Diff = gaitParam.refEEPose[this->graspLessManipArm[0]].translation() - gaitParam.refEEPose[this->graspLessManipArm[1]].translation();
    cnoid::Vector3 ic1to0Diff = gaitParam.icEETargetPose[this->graspLessManipArm[0]].translation() - gaitParam.icEETargetPose[this->graspLessManipArm[1]].translation();
    ref1to0Diff[2] = 0.0;
    ic1to0Diff[2] = 0.0;
    if(ref1to0Diff.norm() > 0.0 && ic1to0Diff.norm() > 0.0){
      dr[2] = std::atan2(ref1to0Diff.cross(ic1to0Diff)[2], ref1to0Diff.dot(ic1to0Diff));
    }else{
      dr.setZero();
    }
  }else{
    o_graspLessCmdVel = graspLessCmdVel;
    return;
  }

  graspLessCmdVel.head<2>() =
    ((footPos.linear().transpose() * dp).head<2>()
     + dr.cross(footPos.translation() - handPos).head<2>())
    .cwiseQuotient(this->graspLessManipTimeConst.head<2>());
  graspLessCmdVel[2] = dr[2] / this->graspLessManipTimeConst[2];

  o_graspLessCmdVel = graspLessCmdVel;
  return;
}
