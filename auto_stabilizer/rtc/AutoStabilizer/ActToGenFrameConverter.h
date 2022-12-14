#ifndef ACTTOGENFRAMECONVERTER_H
#define ACTTOGENFRAMECONVERTER_H

#include "GaitParam.h"
#include <cnoid/Body>

class ActToGenFrameConverter {
public:
  // ActToGenFrameConverterだけでつかうパラメータ
  cnoid::Vector3 rpyOffset = cnoid::Vector3::Zero(); // [roll, pitch, yaw]. rootLink Frame. Actual robotのrootLinkの姿勢に加えるオフセット. IMUの取り付け位置のオフセットを考慮するためのものではない(それはモデルファイルを変えれば良い). 全身のキャリブのずれなど次第に出てくるなにかしらのずれをごますためのもの. 本来このようなパラメータは必要ないのが望ましいが、実用上は確かに必要.

public:
  // constant
  std::vector<std::string> eeForceSensor; // constant. 要素数と順序はGaitParam.eeNameと同じ. actualのForceSensorの値を座標変換したものがEndEffectorが受けている力とみなされる. eeForceSensorが""ならば受けている力は常に0とみなされる. eeForceSensorが""で無いならばrobot->findDevice<cnoid::ForceSensor>(eeForceSensor)がnullptrでは無いことを約束するので、毎回nullptrかをチェックしなくても良い

protected:
  // 内部で変更されるパラメータ. startAutoBalancer時にリセットされる
  mutable bool isInitial = true;

public:
  // startAutoBalancer時に呼ばれる
  void reset() {
    isInitial = true;
  }

public:
  // startAutoBalancer時に呼ばれる. convertFrame()内で前回の周期の値を参照することがある変数について、0で初期化する
  void initOutput(const GaitParam& gaitParam, // input
                  cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3>& o_actCogVel, cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6>& o_actRootVel) const; // output
  /*
    支持脚のfootOrigin座標系が一致するように、actual frametとgenerate frameを対応付ける
   */

  // actual frameで表現されたactRobotRawをgenerate frameに投影しactRobotとし、各種actual値をgenerate frameに変換する
  bool convertFrame(const GaitParam& gaitParam, double dt, // input
                    cnoid::BodyPtr& actRobot, std::vector<cnoid::Position>& o_actEEPose, std::vector<cnoid::Vector6>& o_actEEWrench, cnoid::Vector3& o_actCog, cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3>& o_actCogVel, cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6>& o_actRootVel) const; // output
};

#endif
