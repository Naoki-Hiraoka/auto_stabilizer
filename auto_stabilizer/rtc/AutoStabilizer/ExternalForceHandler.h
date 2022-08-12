#ifndef EXTERNALFORCEHANDER_H
#define EXTERNALFORCEHANDER_H

#include "GaitParam.h"

class ExternalForceHandler{
public:
  // ExternalForceHandler

public:
  bool initExternalForceHandlerOutput(const GaitParam& gaitParam,
                                      double& o_omega, cnoid::Vector3& o_l) const;

  bool handleExternalForce(const GaitParam& gaitParam, double mass,
                           double& o_omega, cnoid::Vector3& o_l) const;
};

#endif
