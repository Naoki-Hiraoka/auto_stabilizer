#include "ImpedanceController.h"
#include "MathUtil.h"

void ImpedanceController::initImpedanceOutput(const GaitParam& gaitParam,
                                              std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_icEEOffset /*generate frame, endeffector origin*/) const{
  for(int i=0;i<gaitParam.eeName.size();i++){
    o_icEEOffset[i].reset(cnoid::Vector6::Zero());
  }
}

bool ImpedanceController::calcImpedanceControl(double dt, const GaitParam& gaitParam,
                                               std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& o_icEEOffset /*generate frame, endeffector origin*/, std::vector<cnoid::Position>& o_icEETargetPose /*generate frame*/) const{

  // icEEOffsetを計算
  for(int i=0;i<gaitParam.eeName.size();i++){
    if(!this->isImpedanceMode[i]){
      o_icEEOffset[i].interpolate(dt);
    }else{
      double ratio = 1.0;
      if(i < NUM_LEGS){ // 脚は、isManualControlModeに完全になっている場合のみImpedanceControlを行う.
        if(!(gaitParam.isManualControlMode[i].isEmpty() && gaitParam.isManualControlMode[i].getGoal() == 0.0)){
          ratio = 0.0;
        }
      }

      cnoid::Vector6 offsetPrev; // generate frame. endEffector origin
      cnoid::Vector6 dOffsetPrev; // generate frame. endEffector origin
      gaitParam.icEEOffset[i].value(offsetPrev, dOffsetPrev);

      cnoid::Matrix3 eeR = cnoid::AngleAxisd(offsetPrev.tail<3>().norm(),(offsetPrev.tail<3>().norm()>0)?offsetPrev.tail<3>().normalized() : cnoid::Vector3::UnitX()) * gaitParam.refEEPose[i].linear();

      cnoid::Vector6 refWrenchLocal; //endEffector frame. endeffector origin
      refWrenchLocal.head<3>() = eeR.transpose() * gaitParam.refEEWrench[i].head<3>();
      refWrenchLocal.tail<3>() = eeR.transpose() * gaitParam.refEEWrench[i].tail<3>();

      cnoid::Vector6 actWrenchLocal; //endEffector frame. endeffector origin
      actWrenchLocal.head<3>() = eeR.transpose() * gaitParam.actEEWrench[i].head<3>();
      actWrenchLocal.tail<3>() = eeR.transpose() * gaitParam.actEEWrench[i].tail<3>();

      cnoid::Vector6 offsetPrevLocal; //endEffector frame. endeffector origin
      offsetPrevLocal.head<3>() = eeR.transpose() * offsetPrev.head<3>();
      offsetPrevLocal.tail<3>() = eeR.transpose() * offsetPrev.tail<3>();

      cnoid::Vector6 dOffsetPrevLocal; //endEffector frame. endeffector origin
      dOffsetPrevLocal.head<3>() = eeR.transpose() * dOffsetPrev.head<3>();
      dOffsetPrevLocal.tail<3>() = eeR.transpose() * dOffsetPrev.tail<3>();

      cnoid::Vector6 dOffsetLocal; //endEffector frame. endeffector origin
      for(size_t j=0;j<6;j++){
        if(this->M[i][j] == 0.0 && this->D[i][j] == 0.0 && this->K[i][j]==0.0){
          dOffsetLocal[j] = 0.0;
          continue;
        }

        dOffsetLocal[j] =
          ((actWrenchLocal[j] - refWrenchLocal[j]) * this->wrenchGain[i][j] * ratio * dt * dt
           - this->K[i][j] * offsetPrevLocal[j] * dt * dt
           + this->M[i][j] * dOffsetPrevLocal[j]*dt)
          / (this->M[i][j] + this->D[i][j] * dt + this->K[i][j] * dt * dt);
      }

      cnoid::Vector6 dOffset; //generate frame. endeffector origin
      dOffset.head<3>() = eeR * dOffsetLocal.head<3>();
      dOffset.tail<3>() = eeR * dOffsetLocal.tail<3>();

      cnoid::Vector6 offset;
      offset.head<3>() = offsetPrev.head<3>() + dOffset.head<3>();
      Eigen::AngleAxisd offsetRPrev(offsetPrev.tail<3>().norm(), (offsetPrev.tail<3>().norm()==0)? cnoid::Vector3::UnitX() : offsetPrev.tail<3>().normalized());
      Eigen::AngleAxisd dOffsetR(dOffset.tail<3>().norm(), (dOffset.tail<3>().norm()==0)? cnoid::Vector3::UnitX() : dOffset.tail<3>().normalized());
      Eigen::AngleAxisd offsetR = Eigen::AngleAxisd(dOffsetR * offsetRPrev);
      offset.tail<3>() = offsetR.axis() * offsetR.angle();
      offset = mathutil::clampMatrix<cnoid::Vector6>(offset, this->compensationLimit[i]);
      o_icEEOffset[i].reset(offset, dOffset/dt);
    }
  }

  // refEEPoseとicEEOffsetから、icEETargetPoseを計算
  for(int i=0;i<gaitParam.eeName.size();i++){
    cnoid::Vector6 icOffset = o_icEEOffset[i].value();
    o_icEETargetPose[i].translation() = icOffset.head<3>() + gaitParam.refEEPose[i].translation();
    o_icEETargetPose[i].linear() = cnoid::AngleAxisd(icOffset.tail<3>().norm(),(icOffset.tail<3>().norm()>0)?icOffset.tail<3>().normalized() : cnoid::Vector3::UnitX()) * gaitParam.refEEPose[i].linear();
  }

  return true;
}
