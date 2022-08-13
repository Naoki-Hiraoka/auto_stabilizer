#ifndef FOOTSTEPGENERATOR_H
#define FOOTSTEPGENERATOR_H

#include "GaitParam.h"


class FootStepGenerator{
public:
  // FootStepGeneratorでしか使わないパラメータ
  double defaultStepTime = 0.8; // [s]. goPosやgoVelocityのときに自動生成されるfootstep
  double defaultStrideLimitationTheta = 0.261799; // [rad]. goPosやgoVelocityのときに自動生成されるfootstepの上下限. 支持脚相対. default 15[rad].
  std::vector<std::vector<cnoid::Vector3> > defaultStrideLimitationHull = std::vector<std::vector<cnoid::Vector3> >{std::vector<cnoid::Vector3>{cnoid::Vector3(0.15,-0.18,0),cnoid::Vector3(-0.15,-0.18,0),cnoid::Vector3(-0.15,-0.35,0),cnoid::Vector3(0.15,-0.35,0)},std::vector<cnoid::Vector3>{cnoid::Vector3(0.15,0.35,0),cnoid::Vector3(-0.15,0.35,0),cnoid::Vector3(-0.15,0.18,0),cnoid::Vector3(0.15,0.18,0)}}; // 要素数2. 0: rleg用, 1: lleg用. goPosやgoVelocityのときに自動生成されるfootstepの上下限の凸包. 反対の脚のEndEffector frame(Z軸は鉛直)で表現した着地可能領域(自己干渉やIKの考慮が含まれる). あったほうが扱いやすいのでZ成分があるが、Z成分は0でないといけない. 凸形状で,上から見て半時計回り. thetaとは独立に評価されるので、defaultStrideLimitationThetaだけ傾いていても大丈夫なようにせよ
  double defaultDoubleSupportTime = 0.12; // [s]. defaultStepTime未満である必要がある.
  double defaultStepHeight = 0.07; // [s].
  unsigned int goVelocityStepNum = 6; // 1以上
  bool isModifyFootSteps = true; // 着地位置時間修正を行うかどうか
  bool isEmergencyStepMode = false; // footstepNodesList[0]が末尾の要素でかつ現在のdstCoordsのままだとバランスが取れないなら、footstepNodesList[1]に両脚が横に並ぶ位置に一歩歩くnodeが末尾に入る. (modifyFootSteps=trueのときのみ有効)
  double overwritableMinTime = 0.5; // 0より大きい. 次indexまでの残り時間がこの値を下回るようには着地時間修正を行わない. もともと下回っている場合には、その値を下回るようには着地時刻修正を行わない. 最低限、この時間で足上げ足下げが行えるような時間にすること.
  double overwritableMaxTime = 1.0; // overwritableMinTimeより大きい. 次indexまでの残り時間がこの値を上回るようには着地時間修正を行わない
  double overwritableMaxSwingVelocity = 1.0; //0より大きい [m/s]. 今の遊脚の位置のXYから着地位置のXYまで移動するための速度がこの値を上回るようには着地位置時間修正を行わない
  std::vector<std::vector<cnoid::Vector3> > safeLegHull = std::vector<std::vector<cnoid::Vector3> >(2, std::vector<cnoid::Vector3>{cnoid::Vector3(0.075,0.055,0.0),cnoid::Vector3(-0.075,0.055,0.0),cnoid::Vector3(-0.075,-0.055,0.0),cnoid::Vector3(0.075,-0.055,0.0)}); // 要素数2. rleg: 0. lleg: 1. leg frame.  凸形状で,上から見て半時計回り. Z成分はあったほうが計算上扱いやすいからありにしているが、0でなければならない. 大きさはgaitParam.legHull以下
  std::vector<std::vector<cnoid::Vector3> > overwritableStrideLimitationHull = std::vector<std::vector<cnoid::Vector3> >{std::vector<cnoid::Vector3>{cnoid::Vector3(0.3,-0.18,0),cnoid::Vector3(-0.3,-0.18,0),cnoid::Vector3(-0.3,-0.30,0),cnoid::Vector3(-0.15,-0.45,0),cnoid::Vector3(0.15,-0.45,0),cnoid::Vector3(0.3,-0.30,0)},std::vector<cnoid::Vector3>{cnoid::Vector3(0.3,0.30,0),cnoid::Vector3(0.15,0.45,0),cnoid::Vector3(-0.15,0.45,0),cnoid::Vector3(-0.3,0.30,0),cnoid::Vector3(-0.3,0.18,0),cnoid::Vector3(0.3,0.18,0)}}; // 要素数2. 0: rleg用, 1: lleg用. 着地位置修正時に自動生成されるfootstepの上下限の凸包. 反対の脚のEndEffector frame(Z軸は鉛直)で表現した着地可能領域(自己干渉やIKの考慮が含まれる). あったほうが扱いやすいのでZ成分があるが、Z成分は0でないといけない. 凸形状で,上から見て半時計回り. thetaとは独立に評価されるので、defaultStrideLimitationThetaだけ傾いていても大丈夫なようにせよ. 斜め方向の角を削るなどして、IKが解けるようにせよ
  double contactDecisionThreshold = 50.0; // [N]. generate frameで遊脚が着地時に鉛直方向にこの大きさ以上の力を受けたら接地とみなして、EarlyTouchDown処理を行う
  double goalOffset = -0.05; // [m]. 遊脚軌道生成時に、generate frameで鉛直方向に, 目標着地位置に対して加えるオフセット. FootStepGeneratorのcheckEarlyTouchDownと組み合わせて使う

  // FootStepGeneratorでしか使わないパラメータ. startAutoBalancer時に初期化が必要
  bool isGoVelocityMode = false; // 進行方向に向けてfootStepNodesList[1] ~ footStepNodesList[goVelocityStepNum]の要素をfootstepNodesList[0]から機械的に計算してどんどん位置修正&末尾appendしていく.
  cnoid::Vector3 cmdVel = cnoid::Vector3::Zero(); // X[m/s] Y[m/s] theta[rad/s]. Z軸はgenerate frame鉛直

  std::vector<std::vector<cnoid::Vector2> > steppable_region; // 要素数任意. generate frame. endCoordsが存在できる領域
  std::vector<double> steppable_height; // 要素数はsteppable_regionと同じ. generate frame. 各polygonごとのおおよその値. そのpolygonに届くかどうかの判定と、
  double relLandingHeight; // generate frame. 現在の遊脚のfootstepNodesList[0]のdstCoordsのZ
  cnoid::Vector3 relLandingNormal; // generate frame. 現在の遊脚のfootstepNodesList[0]のdstCoordsのZ軸の方向

protected:
  mutable std::vector<cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6> > actLegWrenchFilter = std::vector<cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6> >(2, cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6>(50.0, cnoid::Vector6::Zero()));  // 要素数2. rleg: 0. lleg: 1. generate frame. endeffector origin. cutoff 50hz. contactDecisionThresholdを用いた接触判定に用いる
public:
  // startAutoBalancer時に呼ばれる
  void reset(){
    isGoVelocityMode = false;
    cmdVel = cnoid::Vector3::Zero();
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
  bool initFootStepNodesList(const cnoid::BodyPtr& genRobot, const GaitParam& gaitParam,
                             std::vector<GaitParam::FootStepNodes>& o_footstepNodesList, std::vector<cnoid::Position>& o_srcCoords, std::vector<cnoid::Position>& o_dstCoordsOrg, std::vector<bool>& o_prevSupportPhase) const;

  class StepNode {
  public:
    int l_r; // 0: RLEG. 1: LLEG
    cnoid::Position coords;
    double stepHeight, stepTime;
  public:
    StepNode () : l_r(RLEG), coords(cnoid::Position::Identity()), stepHeight(), stepTime(){};
    StepNode (const int _l_r, const cnoid::Position& _coords, const double _stepHeight, const double _stepTime)
      : l_r(_l_r), coords(_coords), stepHeight(_stepHeight), stepTime(_stepTime) {};
    friend std::ostream &operator<<(std::ostream &os, const StepNode &sn) {
      os << "footstep" << std::endl;
      os << "  name = [" << ((sn.l_r==LLEG)?std::string("lleg"):
                             std::string("rleg")) << "]" << std::endl;
      os << "  pos =";
      os << (sn.coords.translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
      os << "  rot =";
      os << (sn.coords.linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
      os << "  step_height = " << sn.stepHeight << "[m], step_time = " << sn.stepTime << "[s]";
      return os;
    };
  };
  /*
    footstepNodesList[1]開始時のsupport/swingの状態を上書きによって変更する場合は、footstepNodesList[0]の終了時の状態が両脚支持でかつその期間の時間がdefaultDoubleSupportTimeよりも短いなら延長する

    外部からsetFootStepsやgoPosをされると呼ばれる
    footstepNodesList[1]~がその値で上書きされる. ただしfootstepNodesList[0]終了時にswing状態の足をfootstepNodesList[1]開始時にsupportにする必要がある場合は、footstepNodesList[0]の直後に両脚が横に並ぶ位置に一歩歩いてその足を下ろすnodeが挿入される.
    footstepsの0番目の要素は、実際には歩かず、基準座標としてのみ使われる. footstepNodesList[0].dstCoordsのZ軸を鉛直に直した座標系と、footstepsの0番目の要素のZ軸を鉛直に直した座標系が一致するように座標変換する.
   */
  bool setFootSteps(const GaitParam& gaitParam, const std::vector<StepNode>& footsteps,
                    std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const;

  // footstepNodesListの末尾に両脚が横に並ぶ位置に2歩歩くnodeが入る. 外部からgoVelocityModeをfalseにすること
  bool goStop(const GaitParam& gaitParam,
                    std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const;

  /*
    footstepNodesList[1]開始時のsupport/swingの状態を上書きによって変更する場合は、footstepNodesList[0]の終了時の状態が両脚支持でかつその期間の時間がdefaultDoubleSupportTimeよりも短いなら延長する

    goVelocityModeなら、進行方向に向けてfootStepNodesList[1] ~ footStepNodesList[goVelocityStepNum]の要素を機械的に計算してどんどん末尾appendしていく. cmdVelに応じてきまる

    跳躍中でなければ、
    emergencyStepModeなら、footstepNodesList[0]が末尾の要素でかつ現在のdstCoordsのままだとバランスが取れないなら、footstepNodesList[1]に両脚が横に並ぶ位置に一歩歩くnodeが末尾に入る. (modifyFootStepsと併用せよ)
    modifyFootStepsなら、footstepNodesList[0]の現在のdstCoordsのままだとバランスが取れないか、今swing期でfootstepNodesList[0]終了時に着地する予定の要素のdstCoordsが着地可能領域上にないなら、footstepNodesList[0]の、今swing期でfootstepNodesList[0]終了時に着地する予定の要素を修正する. また、それ以降一回でもswingする要素の位置を平行移動する

  */
  bool calcFootSteps(const GaitParam& gaitParam, const double& dt, bool useActState,
                     std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const;

  // footstepNodesListをdtだけ進める
  bool advanceFootStepNodesList(const GaitParam& gaitParam, double dt,
                                std::vector<GaitParam::FootStepNodes>& o_footstepNodesList, std::vector<cnoid::Position>& o_srcCoords, std::vector<cnoid::Position>& o_dstCoordsOrg, std::vector<bool>& o_prevSupportPhase) const;

protected:
  // footstepNodesList[1:]の着地位置(XYZ,yaw)をcmdVelに基づき更新する
  void updateGoVelocitySteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const std::vector<cnoid::Vector3>& defaultTranslatePos) const;
  // footstepNodesList[idx:] idxより先のstepの位置をtransformOrigin frameでtransformだけ動かす
  void transformFutureSteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, int index, const cnoid::Position& transformOrigin/*generate frame*/, const cnoid::Position& transform/*transformOrigin frame*/) const;
  // footstepNodesList[idx:] idxより先のstepの位置をgenerate frameでtransformだけ動かす
  void transformFutureSteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, int index, const cnoid::Vector3& transform/*generate frame*/) const;
  // 現在のsupportLegが次にswingするまでの間の位置を、generate frameでtransformだけ動かす
  void transformCurrentSupportSteps(int leg, std::vector<GaitParam::FootStepNodes>& footstepNodesList, const cnoid::Position& transform/*generate frame*/) const;
  // footstepNodesの次の一歩を作る. RLEGとLLEGどちらをswingすべきかも決める
  void calcDefaultNextStep(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const std::vector<cnoid::Vector3>& defaultTranslatePos, const cnoid::Vector3& offset = cnoid::Vector3::Zero()) const;
  // footstepNodesの次の一歩を作る.
  GaitParam::FootStepNodes calcDefaultSwingStep(const int& swingLeg, const GaitParam::FootStepNodes& footstepNodes, const std::vector<cnoid::Vector3>& defaultTranslatePos, const cnoid::Vector3& offset = cnoid::Vector3::Zero(), bool startWithSingleSupport = false) const;
  GaitParam::FootStepNodes calcDefaultDoubleSupportStep(const GaitParam::FootStepNodes& footstepNodes) const;
  // 着地位置・タイミング修正
  void modifyFootSteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const GaitParam& gaitParam) const;
  // 早づきしたらremainTimeをdtに減らしてすぐに次のnodeへ移る. この機能が無いと少しでもロボットが傾いて早づきするとジャンプするような挙動になる. 遅づきに備えるために、着地位置を下方にオフセットさせる
  void checkEarlyTouchDown(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const GaitParam& gaitParam, double dt) const;
};

#endif
