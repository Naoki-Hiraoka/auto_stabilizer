#ifndef CNOIDBODYUTIL_H
#define CNOIDBODYUTIL_H

#include <cnoid/Body>

namespace cnoidbodyutil {
  inline void moveCoords(cnoid::BodyPtr robot, const cnoid::Position& target, const cnoid::Position& at){
    // fix ’at’ coords on ’robot’ to ’target’
    cnoid::Position transform = target * at.inverse();
    robot->rootLink()->T() = transform * robot->rootLink()->T();
  }
  inline void copyRobotState(cnoid::BodyPtr inRobot, cnoid::BodyPtr outRobot) {
    outRobot->rootLink()->T() = inRobot->rootLink()->T();
    outRobot->rootLink()->v() = inRobot->rootLink()->v();
    outRobot->rootLink()->w() = inRobot->rootLink()->w();
    outRobot->rootLink()->dv() = inRobot->rootLink()->dv();
    outRobot->rootLink()->dw() = inRobot->rootLink()->dw();
    for(int i=0;i<outRobot->numJoints();i++){
      outRobot->joint(i)->q() = inRobot->joint(i)->q();
      outRobot->joint(i)->dq() = inRobot->joint(i)->dq();
      outRobot->joint(i)->ddq() = inRobot->joint(i)->ddq();
      outRobot->joint(i)->u() = inRobot->joint(i)->u();
    }
    outRobot->calcForwardKinematics();
    outRobot->calcCenterOfMass();
  }

};

#endif
