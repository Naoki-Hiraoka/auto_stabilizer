#ifndef IMPEDANCE_CONTROLLER_H
#define IMPEDANCE_CONTROLLER_H

#include "EndEffectorParam.h"

class ImpedanceController {
public:
  // Impedance Controllerでしか使わないパラメータ
  std::vector<cnoid::Vector6> CompensationLimit; // 要素数EndEffectors. generate frame. endEffector origin
  std::vector<cnoid::Vector6> M; // 要素数EndEffectors. EndEffector frame(icTargetPose). endEffector origin
  std::vector<cnoid::Vector6> D; // 要素数EndEffectors. EndEffector frame(icTargetPose). endEffector origin
  std::vector<cnoid::Vector6> K; // 要素数EndEffectors. EndEffector frame(icTargetPose). endEffector origin
  std::vector<cnoid::Vector6> wrenchGain; // 要素数EndEffectors. EndEffector frame(icTargetPose). endEffector origin

  // reset()時にresetされる
  std::vector<bool> isImpedanceMode; // 要素数EndEffectors. Impedance Controlをするかどうか
public:
  void push_back(){
    cnoid::Vector6 defaultCompensationLimit; defaultCompensationLimit << 0.08, 0.08, 0.08, 0.05, 0.05, 0.05;
    this->CompensationLimit.push_back(defaultCompensationLimit);
    cnoid::Vector6 defaultM; defaultM << 10, 10, 10, 5, 5, 5;
    this->M.push_back(defaultM);
    cnoid::Vector6 defaultD; defaultD << 600, 600, 600, 200, 200, 200;
    this->D.push_back(defaultD);
    cnoid::Vector6 defaultK; defaultK << 400, 400, 400, 200, 200, 200;
    this->K.push_back(defaultK);

    this->wrenchGain.push_back(cnoid::Vector6::Ones());

    this->isImpedanceMode.push_back(false);
  }

  // start AutoBalancerで呼ばれる
  bool reset(){
    for(int i=0;i<this->isImpedanceMode.size();i++) this->isImpedanceMode[i] = false;
  }

  bool calcImpedanceControl(double dt, const EndEffectorParam& endEffectorParam,
                            std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> >& icOffset /*generate frame, endeffector origin*/) const;
};

#endif
