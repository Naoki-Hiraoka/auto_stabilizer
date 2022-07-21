#ifndef FOOTSTEPGENERATOR_H
#define FOOTSTEPGENERATOR_H

#include "GaitParam.h"

namespace footstepgenerator{
  void calcFootSteps(GaitParam& gaitParam, const double& dt);
};

#endif
