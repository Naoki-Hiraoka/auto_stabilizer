#ifndef LEGCOORDSGENERATOR_H
#define LEGCOORDSGENERATOR_H

#include "GaitParam.h"

namespace legcoordsgenerator{

  void calcLegCoords(GaitParam& gaitParam, double dt);

  void calcCOMZMPCoords(const GaitParam& gaitParam, double dt, double g, double mass, cnoid::Vector3& genNextCog, cnoid::Vector3& genNextCogVel);

  bool calcNextCoords(GaitParam& gaitParam, double dt, double g, double mass);
}

#endif
