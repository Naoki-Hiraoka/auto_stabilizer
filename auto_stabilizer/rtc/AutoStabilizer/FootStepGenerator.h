#ifndef FOOTSTEPGENERATOR_H
#define FOOTSTEPGENERATOR_H

#include "GaitParam.h"


class FootStepGenerator{
public:
  // FootStepGeneratorでしか使わないパラメータ
  double legCollisionMargin = 0.02; // [m]. 左右の足のlegHullがこの距離以上離れるようにする. 0以上.
  double defaultStepTime = 0.8; // [s]. goPosやgoVelocityのときに自動生成されるfootstepの一歩の時間. 0より大きい
  std::vector<double> defaultStrideLimitationMaxTheta = std::vector<double>{0.261799,0.785398}; // 要素数2. rleg: 0. lleg: 1. leg frame. [rad]. goPosやgoVelocityのときに自動生成されるfootstepの上限. 支持脚相対. default 15,45[deg]. 0以上. 足裏同士の干渉は自動で回避できるが、膝同士の干渉はIK以外では回避できないので、内股方向には小さくすること.
  std::vector<double> defaultStrideLimitationMinTheta = std::vector<double>{-0.785398,-0.261799}; // 要素数2. rleg: 0. lleg: 1. leg frame. [rad]. goPosやgoVelocityのときに自動生成されるfootstepの下限. 支持脚相対. default 15,45[deg]. 0以下. 足裏同士の干渉は自動で回避できるが、膝同士の干渉はIK以外では回避できないので、内股方向には小さくすること.
  std::vector<std::vector<cnoid::Vector3> > defaultStrideLimitationHull = std::vector<std::vector<cnoid::Vector3> >{std::vector<cnoid::Vector3>{cnoid::Vector3(0.15,-0.18,0),cnoid::Vector3(-0.15,-0.18,0),cnoid::Vector3(-0.15,-0.35,0),cnoid::Vector3(0.15,-0.35,0)},std::vector<cnoid::Vector3>{cnoid::Vector3(0.15,0.35,0),cnoid::Vector3(-0.15,0.35,0),cnoid::Vector3(-0.15,0.18,0),cnoid::Vector3(0.15,0.18,0)}}; // 要素数2. 0: rleg用, 1: lleg用. 単位[m]. goPosやgoVelocityのときに自動生成されるfootstepの、遊脚のエンドエフェクタの着地位置の範囲の凸包. 反対の脚のEndEffector frame(Z軸は鉛直)で表現した着地可能領域(自己干渉やIKの考慮が含まれる). あったほうが扱いやすいのでZ成分があるが、Z成分は0でないといけない. 凸形状で,上から見て半時計回り. thetaの影響はlegHullとlegCollisionMarginを用いて別で評価されるので、defaultStrideLimitationHullでは考慮しなくて良い. 左右方向にsteppable regionをまたぐ場合は、これのY成分が大きくないと後ろ足がまたげない
  double defaultDoubleSupportRatio = 0.15; // defaultStepTimeのうちの、両足支持期の時間の割合. 0より大きく1未満
  double defaultStepHeight = 0.05; // goPosやgoVelocityのときに自動生成されるfootstepの足上げ高さ[m]. 0以上
  unsigned int goVelocityStepNum = 6; // goVelocity中にfootStepNodesListの将来の何ステップぶんを常に生成するか. 1以上
  bool isModifyFootSteps = true; // 着地位置時間修正を行うかどうか
  double overwritableRemainTime = 0.2; // 0以上. 単位[s]. 次indexまでの残り時間がこの値を下回っている場合、着地位置時間修正を行わない. DOWN_PHASEのときにfootstepNodesList[0]のdstCoordsはgenCoordsよりも高い位置に変更されることがないようにするために、LegCoordsGeneratorのdelayTimeOffset以上の値にせよ.
  double overwritableMinTime = 0.3; // 0より大きい. 単位[s]. 次indexまでの残り時間がこの値を下回るようには着地時間修正を行わない. もともと下回っている場合には、その値を下回るようには着地時間修正を行わない. これが無いと脚を空中から下ろす時間が足りなくて急激に動く
  double overwritableMinStepTime = 0.6; // 0より大きい. 単位[s]. 現index開始時からの経過時間がこの値を下回るようには着地時間修正を行わない. もともと下回っている場合には、その値を下回るようには着地時間修正を行わない. これが無いと脚を地面から上げて下ろす時間が足りなくて急激に動く
  double overwritableMaxStepTime = 2.0; // overwritableMinStepTimeより大きい. 単位[s]. 現index開始時からの経過時間がこの値を上回るようには着地時間修正を行わない. もともと上回っている場合には、その値を上回るようには着地時間修正を行わない. これが無いと、DCMが片足のcopとほぼ一致しているときに、ずっと脚を浮かせたまま止まってしまう.
  double overwritableMaxSwingVelocity = 1.0; // 0より大きい. 単位[m/s]. 今の遊脚の位置のXYから着地位置のXYまで移動するための速度がこの値を上回るようには着地位置時間修正を行わない
  std::vector<std::vector<cnoid::Vector3> > safeLegHull = std::vector<std::vector<cnoid::Vector3> >(2, std::vector<cnoid::Vector3>{cnoid::Vector3(0.075,0.055,0.0),cnoid::Vector3(-0.075,0.055,0.0),cnoid::Vector3(-0.075,-0.055,0.0),cnoid::Vector3(0.075,-0.055,0.0)}); // 要素数2. rleg: 0. lleg: 1. leg frame. 単位[m]. 凸形状で,上から見て半時計回り. Z成分はあったほうが計算上扱いやすいからありにしているが、0でなければならない. 大きさはgaitParam.legHull以下
  std::vector<double>  overwritableStrideLimitationMaxTheta = std::vector<double>{0.261799, 1.5708}; // 要素数2. rleg: 0. lleg: 1. leg frame. [rad]. 着地位置修正時に自動生成されるfootstepの上限. 支持脚相対. default 15,90[deg]. 0以上. 現状ではsetFootStepsのリミットとしてしか使われていない. 足裏同士の干渉は自動で回避できるが、膝同士の干渉はIK以外では回避できないので、内股方向には小さくすること.
  std::vector<double>  overwritableStrideLimitationMinTheta = std::vector<double>{-1.5708, -0.261799}; // 要素数2. rleg: 0. lleg: 1. leg frame. [rad]. 着地位置修正時に自動生成されるfootstepの下限. 支持脚相対. default 15,90[deg]. 0以下. 現状ではsetFootStepsのリミットとしてしか使われていない. 足裏同士の干渉は自動で回避できるが、膝同士の干渉はIK以外では回避できないので、内股方向には小さくすること.
  std::vector<std::vector<cnoid::Vector3> > overwritableStrideLimitationHull = std::vector<std::vector<cnoid::Vector3> >{std::vector<cnoid::Vector3>{cnoid::Vector3(0.35,-0.15,0),cnoid::Vector3(-0.35,-0.15,0),cnoid::Vector3(-0.35,-0.350,0),cnoid::Vector3(-0.20,-0.45,0),cnoid::Vector3(0.20,-0.45,0),cnoid::Vector3(0.35,-0.350,0)},std::vector<cnoid::Vector3>{cnoid::Vector3(0.35,0.350,0),cnoid::Vector3(0.20,0.45,0),cnoid::Vector3(-0.20,0.45,0),cnoid::Vector3(-0.35,0.350,0),cnoid::Vector3(-0.35,0.15,0),cnoid::Vector3(0.35,0.15,0)}}; // 要素数2. 0: rleg用, 1: lleg用. 着地位置修正時に自動生成されるfootstepの上下限の凸包. 反対の脚のEndEffector frame(Z軸は鉛直)で表現した着地可能領域(自己干渉やIKの考慮が含まれる). あったほうが扱いやすいのでZ成分があるが、Z成分は0でないといけない. 凸形状で,上から見て半時計回り. thetaの影響はlegHullとlegCollisionMarginを用いて別で評価されるので、defaultStrideLimitationHullでは考慮しなくて良い. 斜め方向の角を削るなどして、IKが解けるようにせよ. 歩行中は急激に変更されない
  double overwritableMaxLandingHeight = 0.25; // [m]. 反対の脚のEndEffector frame(Z軸は鉛直)で表現した着地高さの上限(自己干渉やIKの考慮が含まれる).
  double overwritableMinLandingHeight = -0.25; // [m]. 反対の脚のEndEffector frame(Z軸は鉛直)で表現した着地高さの下限(自己干渉やIKの考慮が含まれる). overwritableMaxLandingHeight以下.
  double overwritableMaxGenGroundZVelocity = 0.5; // [m/s]. genCoords(のgoal)からdstCoordsまでの高さの差がこの速度を超えないように、remainTimeを長くする. 0より大きい. touchVelと同じくらい?
  double overwritableMaxSrcGroundZVelocity = 0.14; // [m/s]. srcCoordsからdstCoordsまでの高さの差がこの速度を超えないように、一歩の時間(現index開始時からの経過時間)を長くする. 0より大きい. 0.2m登るときに一歩あたり1.4sくらい?
  double contactDetectionThreshold = 25.0; // [N]. generate frameで遊脚が着地時に鉛直方向にこの大きさ以上の力を受けたら接地とみなして、EarlyTouchDown処理を行う. 実機では25N程度がよい?
  double contactModificationThreshold = 0.02; // [m]. 早づき・遅づき時に、ずれの大きさがこの値以上の場合に、以降の足の位置をそのぶんだけずらす. 位置制御指令関節角度と実機の角度の差の関係で平らな地面でも常に早づきするため、地面の高さを誤って認識することがないよう、普段はずらさない方が性能が良い. 一方で、ずれが大きい場合には、そのぶんだけずらしたほうが性能が良い. 0以上.
  double goalOffset = -0.02; // [m]. 遊脚軌道生成時に、次に着地する場合、generate frameで鉛直方向に, 目標着地位置に対して加えるオフセット. 0以下. 歩行中は急激に変更されない. 遅づきに対応するためのもの. 位置制御だと着地の衝撃が大きいので0がよいが、トルク制御時や、高低差がある地形や、衝撃を気にする必要がないシミュレーションでは-0.05等にした方がよい.
  double touchVel = 0.3; // 0より大きい. 単位[m/s]. この速さで足を下ろした場合に着地までに要する時間をremainTimeが下回るまで、足下げを始めない. 0.5だと実機では少し速すぎるか?
  bool isEmergencyStepMode = true; // 現在静止状態で、CapturePointがsafeLegHullの外にあるまたは目標ZMPからemergencyStepCpCheckMargin以上離れているなら、footstepNodesListがemergencyStepNumのサイズになるまで歩くnodeが末尾に入る. (modifyFootSteps=trueのときのみ有効)
  double emergencyStepCpCheckMargin = 0.05; // [m]. EmergencyStepの閾値. 0以上
  bool isStableGoStopMode = true; // 現在非静止状態で、着地位置修正を行ったなら、footstepNodesListがemergencyStepNumのサイズになるまで歩くnodeが末尾に入る. (modifyFootSteps=trueのときのみ有効)
  unsigned int emergencyStepNum = 6; // 1以上

  // FootStepGeneratorでしか使わないパラメータ. startAutoBalancer時に初期化が必要
  bool isGoVelocityMode = false; // 進行方向に向けてfootStepNodesList[1] ~ footStepNodesList[goVelocityStepNum]の要素をfootstepNodesList[0]から機械的に計算してどんどん位置修正&末尾appendしていく.

protected:
  mutable std::vector<cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6> > actLegWrenchFilter = std::vector<cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6> >(2, cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6>(50.0, cnoid::Vector6::Zero()));  // 要素数2. rleg: 0. lleg: 1. generate frame. endeffector origin. cutoff 50hz. contactDecisionThresholdを用いた接触判定に用いる
public:
  // startAutoBalancer時に呼ばれる
  void reset(){
    isGoVelocityMode = false;
    for(int i=0;i<NUM_LEGS;i++){
      actLegWrenchFilter[i].reset(cnoid::Vector6::Zero());
    }
  }

  /*
    footstepNodesList[1]開始時のsupport/swingの状態を上書きによって変更する場合は、footstepNodesList[0]の終了時の状態が両脚支持でかつその期間の時間がdefaultDoubleSupportTimeよりも短いなら延長する

    外部からsetFootStepsやgoPosをされるfootstepNodesList[1]~がその値で上書きされる. ただしfootstepNodesList[0]終了時にswing状態の足をfootstepNodesList[1]開始時にsupportにする必要がある場合は、footstepNodesList[0]の直後に両脚が横に並ぶ位置に一歩歩いてその足を下ろすnodeが挿入される.

    外部からgoStopをされると、footstepNodesListの末尾に両脚が横に並ぶ位置に2歩歩くnodeが入る. 外部からgoVelocityModeをfalseにすること

    goVelocityModeなら、進行方向に向けてfootStepNodesList[1] ~ footStepNodesList[goVelocityStepNum]の要素を機械的に計算してどんどん末尾appendしていく. cmdVelに応じてきまる

    跳躍中でなければ、
      emergencyStepModeなら、footstepNodesList[0]が末尾の要素でかつ現在のdstCoordsのままだとバランスが取れないなら、footstepNodesList[1]に両脚が横に並ぶ位置に一歩歩くnodeが末尾に入る. (modifyFootStepsと併用せよ)
      modifyFootStepsなら、footstepNodesList[0]の現在のdstCoordsのままだとバランスが取れないか、今swing期でfootstepNodesList[0]終了時に着地する予定の要素のdstCoordsが着地可能領域上にないなら、footstepNodesList[0]の、今swing期でfootstepNodesList[0]終了時に着地する予定の要素を修正する. また、それ以降一回でもswingする要素の位置を平行移動する
  */
public:
  // startAutoBalancer直後の初回で呼ばれる
  bool initFootStepNodesList(const GaitParam& gaitParam,
                             std::vector<GaitParam::FootStepNodes>& o_footstepNodesList, std::vector<cnoid::Position>& o_srcCoords, std::vector<cnoid::Position>& o_dstCoordsOrg, double& o_remainTimeOrg, std::vector<GaitParam::SwingState_enum>& o_swingState, double& o_elapsedTime, std::vector<bool>& o_prevSupportPhase) const;

  class StepNode {
  public:
    int l_r; // 0: RLEG. 1: LLEG
    cnoid::Position coords;
    double stepHeight, stepTime; // stepHeightは0以上でなければならない. stepTimeは0より大きくなければならない
    bool swingEnd;
  public:
    StepNode () : l_r(RLEG), coords(cnoid::Position::Identity()), stepHeight(), stepTime(), swingEnd(false){};
    StepNode (const int _l_r, const cnoid::Position& _coords, const double _stepHeight, const double _stepTime, const bool _swingEnd)
      : l_r(_l_r), coords(_coords), stepHeight(_stepHeight), stepTime(_stepTime), swingEnd(_swingEnd) {};
    friend std::ostream &operator<<(std::ostream &os, const StepNode &sn) {
      os << "footstep" << std::endl;
      os << "  name = [" << ((sn.l_r==LLEG)?std::string("lleg"):
                             std::string("rleg")) << "]" << std::endl;
      os << "  pos =";
      os << (sn.coords.translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
      os << "  rot =";
      os << (sn.coords.linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
      os << "  step_height = " << sn.stepHeight << "[m], step_time = " << sn.stepTime << "[s], swing_end = " << sn.swingEnd;
      return os;
    };
  };
  /*
    外部からsetFootStepsやgoPosをされると呼ばれる. 静止中でないと無効
    footstepNodesList[1]~がその値で上書きされる. ただしfootstepNodesList[0]終了時にswing状態の足をfootstepNodesList[1]開始時にsupportにする必要がある場合は、footstepNodesList[0]の直後に両脚が横に並ぶ位置に一歩歩いてその足を下ろすnodeが挿入される.
    footstepsの0番目の要素は、実際には歩かず、基準座標としてのみ使われる. footstepNodesList[0].dstCoordsのZ軸を鉛直に直した座標系と、footstepsの0番目の要素のZ軸を鉛直に直した座標系が一致するように座標変換する.
   */
  bool setFootSteps(const GaitParam& gaitParam, const std::vector<StepNode>& footsteps,
                    std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const;
  bool goPos(const GaitParam& gaitParam, double x, double y, double th,
             std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const;

  // footstepNodesListの末尾に両脚が横に並ぶ位置に2歩歩くnodeが入る. 外部からgoVelocityModeをfalseにすること
  bool goStop(const GaitParam& gaitParam,
                    std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const;

  // FootStepNodesListをdtすすめる
  bool procFootStepNodesList(const GaitParam& gaitParam, const double& dt, bool useActState,
                             std::vector<GaitParam::FootStepNodes>& o_footstepNodesList, std::vector<cnoid::Position>& o_srcCoords, std::vector<cnoid::Position>& o_dstCoordsOrg, double& o_remainTimeOrg, std::vector<GaitParam::SwingState_enum>& o_swingState, double& o_elapsedTime, std::vector<bool>& o_prevSupportPhase, double& relLandingHeight) const;

  /*
    footstepNodesList[1]開始時のsupport/swingの状態を上書きによって変更する場合は、footstepNodesList[0]の終了時の状態が両脚支持でかつその期間の時間がdefaultDoubleSupportTimeよりも短いなら延長する

    goVelocityModeなら、進行方向に向けてfootStepNodesList[1] ~ footStepNodesList[goVelocityStepNum]の要素を機械的に計算してどんどん末尾appendしていく. cmdVelに応じてきまる

    跳躍中でなければ、
    emergencyStepModeなら、footstepNodesList[0]が末尾の要素でかつ現在のdstCoordsのままだとバランスが取れないなら、footstepNodesList[1]に両脚が横に並ぶ位置に一歩歩くnodeが末尾に入る. (modifyFootStepsと併用せよ)
    modifyFootStepsなら、footstepNodesList[0]の現在のdstCoordsのままだとバランスが取れないか、今swing期でfootstepNodesList[0]終了時に着地する予定の要素のdstCoordsが着地可能領域上にないなら、footstepNodesList[0]の、今swing期でfootstepNodesList[0]終了時に着地する予定の要素を修正する. また、それ以降一回でもswingする要素の位置を平行移動する

  */
  bool calcFootSteps(const GaitParam& gaitParam, const double& dt, bool useActState,
                     GaitParam::DebugData& debugData, //for Log
                     std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const;

protected:
  // 早づきしたらremainTimeをdtに減らしてすぐに次のnodeへ移る. この機能が無いと少しでもロボットが傾いて早づきするとジャンプするような挙動になる.
  void checkEarlyTouchDown(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const GaitParam& gaitParam, double dt) const;
  // stableGoStop.
  void checkStableGoStop(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const GaitParam& gaitParam) const;
  // footstepNodesListをdtだけ進める
  bool goNextFootStepNodesList(const GaitParam& gaitParam, double dt,
                               std::vector<GaitParam::FootStepNodes>& footstepNodesList, std::vector<cnoid::Position>& srcCoords, std::vector<cnoid::Position>& dstCoordsOrg, double& remainTimeOrg, std::vector<GaitParam::SwingState_enum>& swingState, double& elapsedTime, double& relLandingHeight) const;
  // emergengy step.
  void checkEmergencyStep(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const GaitParam& gaitParam) const;
  // 着地位置・タイミング修正
  void modifyFootSteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, // input & output
                       GaitParam::DebugData& debugData, //for Log
                       const GaitParam& gaitParam) const;

  // thetaとlegHullとstrideLimitationHullから、実際のstrideLimitationhullを求める. 支持脚(水平)座標系. strideLimitationHullの要素数が1以上なら、返り値も必ず1以上
  std::vector<cnoid::Vector3> calcRealStrideLimitationHull(const int& swingLeg, const double& theta, const std::vector<std::vector<cnoid::Vector3> >& legHull, const std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >& defaultTranslatePos, const std::vector<std::vector<cnoid::Vector3> >& strideLimitationHull) const;
  // footstepNodesList[idx:] idxより先のstepの位置をgenerate frameで(左から)transformだけ動かす
  void transformFutureSteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, int index, const cnoid::Position& transform/*generate frame*/) const;
  // footstepNodesList[idx:] idxより先のstepの位置をgenerate frameでtransformだけ動かす
  void transformFutureSteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, int index, const cnoid::Vector3& transform/*generate frame*/) const;
  // indexのsupportLegが次にswingするまでの間の位置を、generate frameで(左から)transformだけ動かす
  void transformCurrentSupportSteps(int leg, std::vector<GaitParam::FootStepNodes>& footstepNodesList, int index, const cnoid::Position& transform/*generate frame*/) const;
  // footstepNodesの次の一歩を作る. RLEGとLLEGどちらをswingすべきかも決める
  void calcDefaultNextStep(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const GaitParam& gaitParam, const cnoid::Vector3& offset = cnoid::Vector3::Zero() /*leg frame*/, bool stableStart = true) const;
  // footstepNodesの次の一歩を作る.
  GaitParam::FootStepNodes calcDefaultSwingStep(const int& swingLeg, const GaitParam::FootStepNodes& footstepNodes, const GaitParam& gaitParam, const cnoid::Vector3& offset = cnoid::Vector3::Zero(), bool startWithSingleSupport = false) const;
  GaitParam::FootStepNodes calcDefaultDoubleSupportStep(const GaitParam::FootStepNodes& footstepNodes, double doubleSupportTime, GaitParam::FootStepNodes::refZmpState_enum endRefZmpState) const;
};

#endif
