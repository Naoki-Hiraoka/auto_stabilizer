#ifndef REFTOGENFRAMECONVERTER_H
#define REFTOGENFRAMECONVERTER_H

#include "GaitParam.h"
#include <cnoid/Body>

class RefToGenFrameConverter {
public:
  // RefToGenFrameConverterだけでつかうパラメータ
  cpp_filters::TwoPointInterpolator<double> handFixMode = cpp_filters::TwoPointInterpolator<double>(0.0,0.0,0.0,cpp_filters::HOFFARBIB); // 0~1. 0ならHandは重心の動きに合わせて左右に揺れる. 1なら揺れない. 滑らかに変化する. 「左右」という概念を使うので、defaultTranslatePosはX成分は0である必要がある.
  std::vector<cpp_filters::TwoPointInterpolator<double> > refFootOriginWeight = std::vector<cpp_filters::TwoPointInterpolator<double> >(NUM_LEGS,cpp_filters::TwoPointInterpolator<double>(1.0,0.0,0.0,cpp_filters::HOFFARBIB)); // 要素数2. 0: rleg. 1: lleg. 0~1. Reference座標系のfootOriginを計算するときに用いるweight. このfootOriginからの相対位置で、GaitGeneratorに管理されていないEndEffectorのReference位置が解釈される. interpolatorによって連続的に変化する. 全てのLegのrefFootOriginWeightが同時に0になることはない.
  cpp_filters::TwoPointInterpolator<double> solveFKMode = cpp_filters::TwoPointInterpolator<double>(1.0,0.0,0.0,cpp_filters::HOFFARBIB); // 0~1. 1ならエンドエフェクタ位置姿勢をrefRobotRawのFKから求める. 0なら、refEEPoseRawから求める. startAutoBalancerした直後は必ず1

public:
  // startAutoBalancer時に呼ばれる
  void reset() {
    handFixMode.reset(handFixMode.getGoal());
    for(int i=0;i<refFootOriginWeight.size();i++) refFootOriginWeight[i].reset(refFootOriginWeight[i].getGoal());
    solveFKMode.reset(1.0);
  }
  // 内部の補間器をdtだけ進める
  void update(double dt){
    handFixMode.interpolate(dt);
    for(int i=0;i<refFootOriginWeight.size();i++) refFootOriginWeight[i].interpolate(dt);
    solveFKMode.interpolate(dt);
  }
public:
  /*
    次の2つの座標系が一致するようにreference frameとgenerate frameを対応付ける
    - refRobotRawの、refFootOriginWeightとdefaultTranslatePosとcopOffset.value()に基づいて求めた足裏中間座標 (イメージとしては静止状態の目標ZMP位置にdefaultTranslatePosを作用させたもの)
    - 位置XYはgenRobotの重心位置. 位置ZはgenRobotの重心位置 - dz. 姿勢はfootMidCoords. (ただしHandFixModeなら、位置のfootMidCoords座標系Y成分はfootMidCoordsの位置.)
      - handControlWeight = 0なら、位置も姿勢もfootMidCoords
   */

  // startAutoBalancer直後の初回に呼ぶ必要がある.
  // generate frame中のfootMidCoordsの位置がreference frame中のrefRobotRawのfootMidCoordsの位置と同じで、generate frame中のfootMidCoordsの傾きが水平になるように、genRobotの初期位置を決め、その姿勢でgenRobotを初期化する
  bool initGenRobot(const GaitParam& gaitParam, // input
                    cnoid::BodyPtr& genRobot, cpp_filters::TwoPointInterpolatorSE3& o_footMidCoords, cnoid::Vector3& o_genCogVel, cnoid::Vector3& o_genCogAcc) const; // output

  // reference frameで表現されたrefRobotRawをgenerate frameに投影しrefRobotとし、各種referencec値をgenerate frameに変換する
  bool convertFrame(const GaitParam& gaitParam, double dt,// input
                    cnoid::BodyPtr& refRobot, std::vector<cnoid::Position>& o_refEEPose, std::vector<cnoid::Vector6>& o_refEEWrench, double& o_refdz, cpp_filters::TwoPointInterpolatorSE3& o_footMidCoords) const; // output
protected:
  // 現在のFootStepNodesListから、genRobotのfootMidCoordsを求める (gaitParam.footMidCoords)
  void calcFootMidCoords(const GaitParam& gaitParam, double dt, cpp_filters::TwoPointInterpolatorSE3& footMidCoords) const;
  // refRobotRawをrefRobotに変換する.
  void convertRefRobotRaw(const GaitParam& gaitParam, const cnoid::Position& genFootMidCoords, cnoid::BodyPtr& refRobot, std::vector<cnoid::Position>& refEEPoseFK, double& refdz) const;
  // refEEPoseRawを変換する.
  void convertRefEEPoseRaw(const GaitParam& gaitParam, const cnoid::Position& genFootMidCoords, std::vector<cnoid::Position>& refEEPoseWithOutFK) const;

  // refFootOriginWeightとdefaultTranslatePosとcopOffset.value() に基づいて両足中間座標を求める
  cnoid::Position calcRefFootMidCoords(const cnoid::Position& rleg_, const cnoid::Position& lleg_, const GaitParam& gaitParam) const;
};

#endif
