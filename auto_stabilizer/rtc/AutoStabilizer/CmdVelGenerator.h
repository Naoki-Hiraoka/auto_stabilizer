#ifndef CMDVELGENERATOR_H
#define  CMDVELGENERATOR_H

#include "GaitParam.h"

class CmdVelGenerator{
public:
  // CmdVelGeneratorだけが使うparameter
  cnoid::Vector3 refCmdVel = cnoid::Vector3::Zero(); // X[m/s] Y[m/s] theta[rad/s]. Z軸はgenerate frame鉛直. support leg frame. 不連続に変化する
  bool isGraspLessManipMode = true; // ImpedanceControllerの変位の方向に歩くかどうか
  std::vector<double> graspLessManipArm; // graspLessManipの対象とするendeffectorのindex. gaitParam.eeNamesのindexに対応. 要素数は0~2.
  cnoid::Vector3 graspLessManipTimeConst = cnoid::Vector3(1.0,1.0,1.0); //単位[s]. 0より大きい
public:
  // startAutoBalancer時に呼ばれる
  void reset(){
    refCmdVel = cnoid::Vector3::Zero();
  }

  bool calcCmdVel(const GaitParam& gaitParam,
                  cnoid::Vector3& o_cmdVel) const;

protected:
  // graspLessManipArmのImpedanceControllerの修正量のぶんだけ、次の一歩のcmdVelを修正する. 修正量の旋回中心がgraspLessManipArmの位置になるようにする.
  void calcVelFromHandError(const GaitParam& gaitParam,
                            cnoid::Vector3& o_graspLessCmdVel) const;
};

#endif
