#ifndef ENDEFFECTORPARAM_H
#define ENDEFFECTORPARAM_H

#include <cnoid/EigenTypes>
#include <ik_constraint/PositionConstraint.h>

class EndEffectorParam {
public:
  // 各vectorのサイズは同じ. サイズや順序は一定不変. サイズは2以上で、0番目がrleg, 1番目がlleg

  // constant
  std::vector<std::string> name; // 右脚はrleg. 左脚はllegという名前である必要がある
  std::vector<std::string> parentLink; // 必ずrobot->link(parentLink)がnullptrではないことを約束する. そのため、毎回robot->link(parentLink)がnullptrかをチェックしなくても良い
  std::vector<cnoid::Position> localT; // Parent Link Frame
  std::vector<std::string> forceSensor; // actualのForceSensorの値を座標変換したものがEndEffectorが受けている力とみなされる. forceSensorが""ならば受けている力は常に0とみなされる. forceSensorが""で無いならばrobot->findDevice<cnoid::ForceSensor>(endEffectorParams[i].forceSensor)がnullptrでは無いことを約束するので、毎回nullptrかをチェックしなくても良い

  // from reference port
  std::vector<cnoid::Vector6> refWrenchOrigin; // FootOrigin frame. EndEffector origin. ロボットが受ける力

  // AutoStabilizerの内部で計算される
  std::vector<cnoid::Position> refPose; // generate frame
  std::vector<cnoid::Vector6> refWrench; // generate frame. EndEffector origin. ロボットが受ける力
  std::vector<cnoid::Position> actPose; // generate frame
  std::vector<cnoid::Vector6> actWrench; // generate frame. EndEffector origin. ロボットが受ける力
  std::vector<cnoid::Position> abcTargetPose; // generate frame. abcで計算された目標位置姿勢
  std::vector<cnoid::Position> stTargetPose; // generate frame. stで計算された目標位置姿勢
  std::vector<std::shared_ptr<IK::PositionConstraint> > ikPositionConstraint;

public:
  void push_back(const std::string& name_, const std::string& parentLink_, const cnoid::Position& localT_, const std::string& forceSensor_){
    name.push_back(name_);
    parentLink.push_back(parentLink_);
    localT.push_back(localT_);
    forceSensor.push_back(forceSensor_);
    refWrenchOrigin.push_back(cnoid::Vector6::Zero());
    refPose.push_back(cnoid::Position::Identity());
    refWrench.push_back(cnoid::Vector6::Zero());
    actPose.push_back(cnoid::Position::Identity());
    actWrench.push_back(cnoid::Vector6::Zero());
    abcTargetPose.push_back(cnoid::Position::Identity());
    stTargetPose.push_back(cnoid::Position::Identity());
    ikPositionConstraint.push_back(std::make_shared<IK::PositionConstraint>());
  }
};

#endif
