#ifndef REFTOGENFRAMECONVERTER_H
#define REFTOGENFRAMECONVERTER_H

#include "GaitParam.h"
#include "EndEffectorParam.h"
#include <cnoid/Body>

class RefToGenFrameConverter {
public:
  // RefToGenFrameConverterだけでつかうパラメータ

  bool isHandFixMode = false; // falseならHandは重心の動きに合わせて左右に揺れる. trueなら揺れない
  std::vector<cpp_filters::TwoPointInterpolator<double> > refFootOriginWeight = std::vector<cpp_filters::TwoPointInterpolator<double> >(NUM_LEGS,cpp_filters::TwoPointInterpolator<double>(1.0,0.0,0.0,cpp_filters::HOFFARBIB)); // 要素数2. 0: rleg. 1: lleg. Reference座標系のfootOriginを計算するときに用いるweight. このfootOriginからの相対位置で、GaitGeneratorに管理されていないEndEffectorのReference位置が解釈される. interpolatorによって連続的に変化する. 全てのLegのrefFootOriginWeightが同時に0になることはない

public:
  /*
    次の2つの座標系が一致するようにreference frameとgenerate frameを対応付ける
    - refRobotの、refFootOriginWeightとdefaultTranslatePosとcopOffsetに基づいて求めた足裏中間座標 (イメージとしては静止状態の目標ZMP位置にdefaultTranslatePosを作用させたもの)
    - 位置はgenRobotの重心位置. 姿勢はfootMidCoords. (ただしHandFixModeなら、位置のfootMidCoords座標系Y座標成分はfootMidCoordsの位置.)
   */

  // startAutoBalancer直後の初回に呼ぶ必要がある.
  // generate frame中のfootMidCoordsの位置がreference frame中のrefRobotのfootMidCoordsの位置と同じで、generate frame中のfootMidCoordsの傾きが水平になるように、genRobotの初期位置を決め、その姿勢でgenRobotを初期化する
  bool initGenRobot(const cnoid::BodyPtr& refRobot, const EndEffectorParam& endEffectorParams, const GaitParam& gaitParam, // input
                    cnoid::BodyPtr& genRobot, cpp_filters::TwoPointInterpolatorSE3& o_footMidCoords, cnoid::Vector3& o_genCog, cnoid::Vector3& o_genCogVel) const; // output

  // reference frameで表現されたrefRobotをgenerate frameに投影しrefRobotOriginとし、各種referencec値をgenerate frameに変換する
  bool convertFrame(const cnoid::BodyPtr& refRobot, const EndEffectorParam& endEffectorParams, const GaitParam& gaitParam, // input
                    cnoid::BodyPtr& refRobotOrigin, std::vector<cnoid::Position>& o_refPose, std::vector<cnoid::Vector6>& o_refWrench, double& o_dz) const; // output
protected:
  // refFootOriginWeightとdefaultTranslatePosとcopOffset に基づいて両足中間座標を求める
  cnoid::Position calcRefFootMidCoords(const cnoid::BodyPtr& robot, const EndEffectorParam& endEffectorParams, const GaitParam& gaitParam) const;
};

#endif
