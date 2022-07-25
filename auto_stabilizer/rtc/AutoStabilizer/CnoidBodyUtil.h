#ifndef CNOIDBODYUTIL_H
#define CNOIDBODYUTIL_H

#include <cnoid/Body>

namespace cnoidbodyutil {
  inline void moveCoords(cnoid::BodyPtr robot, const cnoid::Position& target, const cnoid::Position& at){
    // fix ’at’ coords on ’robot’ to ’target’
    cnoid::Position transform = target * at.inverse();
    robot->rootLink()->T() = transform * robot->rootLink()->T();
  }
};

#endif
