#include "Stabilizer.h"

void Stabilizer::reset() {
}

void Stabilizer::update(double dt){
}

bool Stabilizer::execStabilizer(const GaitParam& gaitParam, double dt, double g, double mass,
                                cnoid::Vector3& o_genNextCog, cnoid::Vector3& o_genNextCogVel) const{
  return true;
}
