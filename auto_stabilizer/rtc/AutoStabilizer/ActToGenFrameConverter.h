#ifndef ACTTOGENFRAMECONVERTER_H
#define ACTTOGENFRAMECONVERTER_H

#include "GaitParam.h"
#include <cnoid/Body>

class ActToGenFrameConverter {
public:
  // ActToGenFrameConverterだけでつかうパラメータ

protected:
  // 内部で変更されるパラメータ. tartAutoBalancer時にリセットされる
  mutable bool isInitial = true;

public:
  // startAutoBalancer時に呼ばれる
  void reset() {
    isInitial = true;
  }

public:
  /*
    支持脚のfootOrigin座標系が一致するように、actual frametとgenerate frameを対応付ける
   */

  // actual frameで表現されたactRobotRawをgenerate frameに投影しactRobotとし、各種actual値をgenerate frameに変換する
  bool convertFrame(const cnoid::BodyPtr& actRobotRaw, const GaitParam& gaitParam, double dt, // input
                    cnoid::BodyPtr& actRobot, std::vector<cnoid::Position>& o_actPose, std::vector<cnoid::Vector6>& o_actWrench, cnoid::Vector3& o_actCog, cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3>& o_actCogVel) const; // output
};

#endif
