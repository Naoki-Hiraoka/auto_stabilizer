#include "FootStepGenerator.h"
#include "MathUtil.h"
#include <cnoid/EigenUtil>
#include <opencv2/imgproc.hpp>

bool FootStepGenerator::initFootStepNodesList(const GaitParam& gaitParam,
                                              std::vector<GaitParam::FootStepNodes>& o_footstepNodesList, std::vector<cnoid::Position>& o_srcCoords, std::vector<cnoid::Position>& o_dstCoordsOrg, double& o_remainTimeOrg, std::vector<GaitParam::SwingState_enum>& o_swingState, std::vector<bool>& o_isLandingGainPhase, double& o_elapsedTime, std::vector<bool>& o_prevSupportPhase) const{
  // footStepNodesListを初期化する
  std::vector<GaitParam::FootStepNodes> footstepNodesList(1);
  cnoid::Position rlegCoords = gaitParam.genRobot->link(gaitParam.eeParentLink[RLEG])->T()*gaitParam.eeLocalT[RLEG];
  cnoid::Position llegCoords = gaitParam.genRobot->link(gaitParam.eeParentLink[LLEG])->T()*gaitParam.eeLocalT[LLEG];
  footstepNodesList[0].dstCoords = {rlegCoords, llegCoords};
  footstepNodesList[0].isSupportPhase = {(gaitParam.isManualControlMode[RLEG].getGoal() == 0.0), (gaitParam.isManualControlMode[LLEG].getGoal() == 0.0)};
  footstepNodesList[0].remainTime = 0.0;
  if(footstepNodesList[0].isSupportPhase[RLEG] && !footstepNodesList[0].isSupportPhase[LLEG]) footstepNodesList[0].endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::RLEG;
  else if(!footstepNodesList[0].isSupportPhase[RLEG] && footstepNodesList[0].isSupportPhase[LLEG]) footstepNodesList[0].endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::LLEG;
  else footstepNodesList[0].endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::MIDDLE;
  std::vector<cnoid::Position> srcCoords = footstepNodesList[0].dstCoords;
  std::vector<cnoid::Position> dstCoordsOrg = footstepNodesList[0].dstCoords;
  double remainTimeOrg = footstepNodesList[0].remainTime;
  std::vector<GaitParam::SwingState_enum> swingState(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++) swingState[i] = GaitParam::SWING_PHASE;
  std::vector<bool> isLandingGainPhase(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++) isLandingGainPhase[i] = false;
  std::vector<bool> prevSupportPhase(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++) prevSupportPhase[i] = footstepNodesList[0].isSupportPhase[i];
  double elapsedTime = 0.0;

  o_prevSupportPhase = prevSupportPhase;
  o_footstepNodesList = footstepNodesList;
  o_srcCoords = srcCoords;
  o_dstCoordsOrg = dstCoordsOrg;
  o_remainTimeOrg = remainTimeOrg;
  o_swingState = swingState;
  o_isLandingGainPhase = isLandingGainPhase;
  o_elapsedTime = elapsedTime;

  return true;
}

bool FootStepGenerator::setFootSteps(const GaitParam& gaitParam, const std::vector<StepNode>& footsteps,
                                     std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const{
  if(!gaitParam.isStatic()){ // 静止中でないと無効
    o_footstepNodesList = gaitParam.footstepNodesList;
    return false;
  }

  if(footsteps.size() <= 1) { // 何もしない
    o_footstepNodesList = gaitParam.footstepNodesList;
    return true;
  }

  if(footsteps[0].l_r == footsteps[1].l_r){ // 基準が無い. 無効
    o_footstepNodesList = gaitParam.footstepNodesList;
    return false;
  }

  if((!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && footsteps[1].l_r == LLEG) ||
     (!gaitParam.footstepNodesList[0].isSupportPhase[LLEG] && footsteps[1].l_r == RLEG)){ // 空中の足を支持脚にしようとしている. 無効
    o_footstepNodesList = gaitParam.footstepNodesList;
    return false;
  }

  for(int i=1;i<footsteps.size();i++){
    if((footsteps[i-1].l_r == RLEG && footsteps[i-1].swingEnd == true && footsteps[i].l_r == LLEG) ||
       (footsteps[i-1].l_r == LLEG && footsteps[i-1].swingEnd == true && footsteps[i].l_r == RLEG)){ // 空中の足を支持脚にしようとしている. 無効
      o_footstepNodesList = gaitParam.footstepNodesList;
      return false;
    }
  }

  std::vector<GaitParam::FootStepNodes> footstepNodesList;
  footstepNodesList.push_back(gaitParam.footstepNodesList[0]);

  if(footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){ // 両足支持期を延長
    // 現在の時刻から突然refZmpTrajが変化すると、大きなZMP入力変化が必要になる. いまの位置でrefZmpTrajをthis->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio)の間とめて、次にthis->defaultStepTime * this->defaultDoubleSupportRatioの間で次の支持脚側に動かす
    footstepNodesList.back().remainTime = this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio);
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
  }

  // footstepsの0番目の要素は、実際には歩かず、基準座標としてのみ使われる.
  // footstepsの反対側の足が最後についた位置姿勢のZ軸を鉛直に直した座標系からみたfootstepsのi番目の要素の位置姿勢に、
  // footstepNodesList.back().dstCoordsのZ軸を鉛直に直した座標系から見た次の一歩の着地位置がなるように、座標変換する.
  std::vector<cnoid::Position> legPoseInFootSteps(2); // footStepsの足が最後についた位置姿勢. footSteps frame
  for(int i=0;i<2;i++) legPoseInFootSteps[footsteps[i].l_r] = mathutil::orientCoordToAxis(footsteps[i].coords, cnoid::Vector3::UnitZ());
  for(int i=1;i<footsteps.size();i++){
    if(footstepNodesList.back().endRefZmpState != GaitParam::FootStepNodes::refZmpState_enum::RLEG && footsteps[i].l_r == LLEG){ // 両足支持期
      footstepNodesList.back().endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::RLEG;
    }else if(footstepNodesList.back().endRefZmpState != GaitParam::FootStepNodes::refZmpState_enum::LLEG && footsteps[i].l_r == RLEG){ // 両足支持期
      footstepNodesList.back().endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::LLEG;
    }

    GaitParam::FootStepNodes fs;
    int swingLeg = footsteps[i].l_r;
    int supportLeg = swingLeg == RLEG ? LLEG: RLEG;
    fs.dstCoords[supportLeg] = footstepNodesList.back().dstCoords[supportLeg];
    {
      cnoid::Position transform = legPoseInFootSteps[supportLeg].inverse() * footsteps[i].coords; // supportLeg相対(Z軸は鉛直)での次のswingLegの位置
      // 着地位置をoverwritableStrideLimitationでリミット. swingEndのときでもリミットしたほうが良い
      cnoid::Vector3 localZ = transform.linear() * cnoid::Vector3::Zero();
      double theta = cnoid::rpyFromRot(mathutil::orientCoordToAxis(transform.linear(), cnoid::Vector3::Zero()))[2];
      theta = mathutil::clamp(theta, this->overwritableStrideLimitationMinTheta[swingLeg], this->overwritableStrideLimitationMaxTheta[swingLeg]);
      transform.linear() = mathutil::orientCoordToAxis(cnoid::AngleAxis(theta, cnoid::Vector3::UnitZ()).toRotationMatrix(), localZ);
      std::vector<cnoid::Vector3> strideLimitationHull = this->calcRealStrideLimitationHull(swingLeg, theta, gaitParam.legHull, gaitParam.defaultTranslatePos, this->overwritableStrideLimitationHull);
      transform.translation().head<2>() = mathutil::calcNearestPointOfHull(transform.translation(), strideLimitationHull).head<2>();
      fs.dstCoords[swingLeg] = mathutil::orientCoordToAxis(footstepNodesList.back().dstCoords[supportLeg], cnoid::Vector3::UnitZ()) * transform;
    }
    legPoseInFootSteps[swingLeg] = mathutil::orientCoordToAxis(footsteps[i].coords, cnoid::Vector3::UnitZ());
    fs.isSupportPhase[supportLeg] = true;
    fs.isSupportPhase[swingLeg] = false;
    fs.remainTime = footsteps[i].stepTime * (1 - this->defaultDoubleSupportRatio);
    fs.endRefZmpState = supportLeg == RLEG ? GaitParam::FootStepNodes::refZmpState_enum::RLEG : GaitParam::FootStepNodes::refZmpState_enum::LLEG;
    double stepHeight = std::max(0.0, footsteps[i].stepHeight);
    double beforeHeight = footstepNodesList.back().isSupportPhase[swingLeg] ? stepHeight : 0.0;
    double afterHeight = footsteps[i].swingEnd ? 0.0 : stepHeight;
    fs.stepHeight[swingLeg] = {beforeHeight,afterHeight};
    fs.touchVel[swingLeg] = footsteps[i].swingEnd ? 0.0 : this->touchVel;
    fs.goalOffset[swingLeg] = 0.0;
    footstepNodesList.push_back(fs);

    if(!footsteps[i].swingEnd) footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), footsteps[i].stepTime * this->defaultDoubleSupportRatio, swingLeg == RLEG ? GaitParam::FootStepNodes::refZmpState_enum::RLEG : GaitParam::FootStepNodes::refZmpState_enum::LLEG));
  }

  if(footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){
    // refZmpを両足の中心へ戻す
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), footstepNodesList.back().endRefZmpState));
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), GaitParam::FootStepNodes::refZmpState_enum::MIDDLE)); // 末尾の両足支持期を延長. これがないと重心が目標位置に収束する前に返ってしまい, emergencyStepが無限に誘発する footGudedBalanceTimeを0.4程度に小さくすると収束が速くなるのでこの処理が不要になるのだが、今度はZ方向に振動しやすい
  }

  o_footstepNodesList = footstepNodesList;
  return true;
}

bool FootStepGenerator::goPos(const GaitParam& gaitParam, double x/*m*/, double y/*m*/, double th/*deg*/,
                              std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const{
  if(!gaitParam.isStatic()){ // 静止中でないと無効
    o_footstepNodesList = gaitParam.footstepNodesList;
    return false;
  }

  std::vector<GaitParam::FootStepNodes> footstepNodesList;
  footstepNodesList.push_back(gaitParam.footstepNodesList[0]);

  cnoid::Position currentPose;
  {
    cnoid::Position rleg = mathutil::orientCoordToAxis(footstepNodesList.back().dstCoords[RLEG], cnoid::Vector3::UnitZ());
    rleg.translation() -= rleg.linear() * gaitParam.defaultTranslatePos[RLEG].value();
    cnoid::Position lleg = mathutil::orientCoordToAxis(footstepNodesList.back().dstCoords[LLEG], cnoid::Vector3::UnitZ());
    lleg.translation() -= lleg.linear() * gaitParam.defaultTranslatePos[LLEG].value();
    currentPose = mathutil::calcMidCoords(std::vector<cnoid::Position>{rleg, lleg}, std::vector<double>{footstepNodesList.back().isSupportPhase[RLEG] ? 1.0 : 0.0, footstepNodesList.back().isSupportPhase[LLEG] ? 1.0 : 0.0});
  }
  cnoid::Position trans;
  trans.translation() = cnoid::Vector3(x, y, 0.0);
  trans.linear() = Eigen::AngleAxisd(th * M_PI / 180.0, cnoid::Vector3::UnitZ()).toRotationMatrix();
  const cnoid::Position goalPose = currentPose * trans; // generate frame. Z軸は鉛直

  int steps = 0;
  while(steps < 100){
    cnoid::Vector3 diff;
    cnoid::Position trans = currentPose.inverse() * goalPose; // currentPose frame
    diff.head<2>() = trans.translation().head<2>(); //currentPose frame. [m]
    diff[2] = cnoid::rpyFromRot(trans.linear())[2]; // currentPose frame. [rad]
    if(steps >= 1 && // 最低1歩は歩く. (そうしないと、両脚がdefaultTranslatePosとは違う開き方をしているときにgoPos(0,0,0)すると、defaultTranslatePosに戻れない)
       (diff.head<2>().norm() < 1e-3 * 0.1) && (std::abs(diff[2]) < 0.5 * M_PI / 180.0)) break;
    this->calcDefaultNextStep(footstepNodesList, gaitParam, diff);
    steps++;
    {
      cnoid::Position rleg = mathutil::orientCoordToAxis(footstepNodesList.back().dstCoords[RLEG], cnoid::Vector3::UnitZ());
      rleg.translation() -= rleg.linear() * gaitParam.defaultTranslatePos[RLEG].value();
      cnoid::Position lleg = mathutil::orientCoordToAxis(footstepNodesList.back().dstCoords[LLEG], cnoid::Vector3::UnitZ());
      lleg.translation() -= lleg.linear() * gaitParam.defaultTranslatePos[LLEG].value();
      currentPose = mathutil::calcMidCoords(std::vector<cnoid::Position>{rleg, lleg}, std::vector<double>{footstepNodesList[footstepNodesList.size()-2].isSupportPhase[LLEG] ? 1.0 : 0.0, footstepNodesList[footstepNodesList.size()-2].isSupportPhase[RLEG] ? 1.0 : 0.0}); // 前回swingした足の位置を見る
    }
  }

  if(steps >= 100) { // goalに到達しない
    o_footstepNodesList = gaitParam.footstepNodesList;
    return false;
  }

  // 両脚が横に並ぶ位置に1歩歩く.
  for(int i=0;i<1;i++){
    this->calcDefaultNextStep(footstepNodesList, gaitParam);
  }

  // refZmpを両足の中心へ戻す
  footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), footstepNodesList.back().endRefZmpState));
  footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
  footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), GaitParam::FootStepNodes::refZmpState_enum::MIDDLE)); // 末尾の両足支持期を延長. これがないと重心が目標位置に収束する前に返ってしまい, emergencyStepが無限に誘発する footGudedBalanceTimeを0.4程度に小さくすると収束が速くなるのでこの処理が不要になるのだが、今度はZ方向に振動しやすい

  o_footstepNodesList = footstepNodesList;
  return true;
}


bool FootStepGenerator::goStop(const GaitParam& gaitParam,
            std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const {
  if(gaitParam.isStatic()){
    o_footstepNodesList = gaitParam.footstepNodesList;
    return true;
  }

  std::vector<GaitParam::FootStepNodes> footstepNodesList = gaitParam.footstepNodesList;

  // footstepNodesList[0]と[1]は変えない. footstepNodesList[1]以降で次に両足支持期になるときを探し、それ以降のstepを上書きする. 片足支持期の状態が末尾の要素になると、片足立ちで止まるという状態を意味することに注意
  for(int i=1;i<footstepNodesList.size();i++){
    if(footstepNodesList[i].isSupportPhase[RLEG] && footstepNodesList[i].isSupportPhase[LLEG]){
      footstepNodesList.resize(i+1);
      break;
    }
  }

  // 両脚が横に並ぶ位置に1歩歩く.
  for(int i=0;i<1;i++){
    this->calcDefaultNextStep(footstepNodesList, gaitParam);
  }

  // refZmpを両足の中心へ戻す
  footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), footstepNodesList.back().endRefZmpState));
  footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
  footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), GaitParam::FootStepNodes::refZmpState_enum::MIDDLE)); // 末尾の両足支持期を延長. これがないと重心が目標位置に収束する前に返ってしまい, emergencyStepが無限に誘発する footGudedBalanceTimeを0.4程度に小さくすると収束が速くなるのでこの処理が不要になるのだが、今度はZ方向に振動しやすい

  o_footstepNodesList = footstepNodesList;
  return true;

}

// FootStepNodesListをdtすすめる
bool FootStepGenerator::procFootStepNodesList(const GaitParam& gaitParam, const double& dt, bool useActState,
                                              std::vector<GaitParam::FootStepNodes>& o_footstepNodesList, std::vector<cnoid::Position>& o_srcCoords, std::vector<cnoid::Position>& o_dstCoordsOrg, double& o_remainTimeOrg, std::vector<GaitParam::SwingState_enum>& o_swingState, std::vector<bool>& o_isLandingGainPhase, double& o_elapsedTime, std::vector<bool>& o_prevSupportPhase, double& relLandingHeight) const{
  std::vector<GaitParam::FootStepNodes> footstepNodesList = gaitParam.footstepNodesList;
  std::vector<bool> prevSupportPhase = gaitParam.prevSupportPhase;
  double elapsedTime = gaitParam.elapsedTime;
  std::vector<cnoid::Position> srcCoords = gaitParam.srcCoords;
  std::vector<cnoid::Position> dstCoordsOrg = gaitParam.dstCoordsOrg;
  double remainTimeOrg = gaitParam.remainTimeOrg;
  std::vector<GaitParam::SwingState_enum> swingState = gaitParam.swingState;
  std::vector<bool> isLandingGainPhase = gaitParam.isLandingGainPhase;

  if(useActState){
    // 早づきしたらremainTimeにかかわらずすぐに次のnodeへ移る(remainTimeをdtにする). この機能が無いと少しでもロボットが傾いて早づきするとジャンプするような挙動になる.
    this->checkEarlyTouchDown(footstepNodesList, gaitParam, dt);
  }

  // footstepNodesListを進める
  footstepNodesList[0].remainTime = std::max(0.0, footstepNodesList[0].remainTime - dt);
  elapsedTime += dt;
  for(int i=0;i<NUM_LEGS;i++) prevSupportPhase[i] = footstepNodesList[0].isSupportPhase[i];

  if(footstepNodesList[0].remainTime <= 0.0 && footstepNodesList.size() > 1){ // 次のfootstepNodesListのindexに移る.
    if(this->isModifyFootSteps && this->isStableGoStopMode && useActState){
      // footstepNodesList[0]で着地位置修正を行っていたら、footstepNodesListがemergencyStepNumのサイズになるまで歩くnodeが末尾に入る.
      this->checkStableGoStop(footstepNodesList, gaitParam);
    }

    // footstepNodesList[1]へ進む
    this->goNextFootStepNodesList(gaitParam, dt,
                                  footstepNodesList, srcCoords, dstCoordsOrg, remainTimeOrg, swingState, isLandingGainPhase, elapsedTime, relLandingHeight);
  }

  o_footstepNodesList = footstepNodesList;
  o_prevSupportPhase = prevSupportPhase;
  o_elapsedTime = elapsedTime;
  o_srcCoords = srcCoords;
  o_dstCoordsOrg = dstCoordsOrg;
  o_remainTimeOrg = remainTimeOrg;
  o_swingState = swingState;
  o_isLandingGainPhase = isLandingGainPhase;

  return true;
}

bool FootStepGenerator::calcFootSteps(const GaitParam& gaitParam, const double& dt, bool useActState,
                                      GaitParam::DebugData& debugData, //for Log
                                      std::vector<GaitParam::FootStepNodes>& o_footstepNodesList, std::vector<GaitParam::SwingState_enum>& o_swingState) const{
  std::vector<GaitParam::FootStepNodes> footstepNodesList = gaitParam.footstepNodesList;
  std::vector<GaitParam::SwingState_enum> swingState = gaitParam.swingState;

  // goVelocityModeなら、進行方向に向けてfootStepNodesList[2] ~ footStepNodesList[goVelocityStepNum]の要素を機械的に計算してどんどん末尾appendしていく. cmdVelに応じてきまる
  if(this->isGoVelocityMode){
    // footstepNodesList[0]と[1]は変えない. footstepNodesList[1]以降で次に両足支持期になるときを探し、それ以降のstepを上書きする. 片足支持期の状態が末尾の要素になると、片足立ちで止まるという状態を意味することに注意
    for(int i=1;i<footstepNodesList.size();i++){
      if(footstepNodesList[i].isSupportPhase[RLEG] && footstepNodesList[i].isSupportPhase[LLEG]){
        footstepNodesList.resize(i+1);
        break;
      }
    }
    while(footstepNodesList.size() <= this->goVelocityStepNum){
      this->calcDefaultNextStep(footstepNodesList, gaitParam, gaitParam.cmdVel * this->defaultStepTime);
    }
  }

  if(useActState){
    if(this->isModifyFootSteps && this->isEmergencyStepMode){
      this->checkEmergencyStep(footstepNodesList, gaitParam);
    }

    if(this->isModifyFootSteps){
      this->modifyFootSteps(footstepNodesList, swingState, debugData, gaitParam);
    }
  }

  o_footstepNodesList = footstepNodesList;
  o_swingState = swingState;

  return true;
}

bool FootStepGenerator::goNextFootStepNodesList(const GaitParam& gaitParam, double dt,
                                                std::vector<GaitParam::FootStepNodes>& footstepNodesList, std::vector<cnoid::Position>& srcCoords, std::vector<cnoid::Position>& dstCoordsOrg, double& remainTimeOrg, std::vector<GaitParam::SwingState_enum>& swingState, std::vector<bool>& isLandingGainPhase, double& elapsedTime, double& relLandingHeight) const{
  // 今のgenCoordsとdstCoordsが異なるなら、将来のstepの位置姿勢をそれに合わせてずらす.
  // early touch downまたはlate touch downが発生していると、今のgenCoordsとdstCoordsが異なる.
  //   今のgenCoordsが正しい地形を表していることが期待されるので、将来のstepの位置姿勢をgenCoordsにあわせてずらしたくなる
  //   ところが実際には、genCoordsは位置制御指令関節角度から計算されるものなので、実際の地形とは誤差がある. (特に高さ方向に1cmくらい、実際よりも高いと誤認識する)
  //   そのため、両足支持期のときに、平らな地面を歩いているのに、後から地面についた足の位置がもとから地面についていた足の位置よりも高く逆運動学が解かれて、その結果身体が後から地面についた足の方に傾く. すると、その傾きによる重心位置の誤差を補正するために、特に左右方向に目標ZMPが8cmくらい大きくふれてしまう. これは許容できないので、基本的にはずらしてはいけない.
  // early touch downまたはlate touch downが発生していると、今のgenCoordsとdstCoordsが異なるが、無視して次のnodeの間にそのnodeのもとのdstCoordsにそのまま補間してゆっくり遷移することにする.
  // しかし、あまりにもずれが大きい場合は、遷移速度が無視できないほど大きくなるので、将来のstepの位置姿勢をgenCoordsにあわせてずらした方がよい
  // 触覚を精度良く利用したければ、「指令関節角度」という概念を使うのをやめて、actual angleに基づく手法に変えるべきかと
  // 基本的に、環境の地形は、触覚からは取得せず、視覚から取得する方針. 転倒等でロボットの身体が大きく傾いてearly touch down, late touch downした場合にのみ、やむを得ず触覚に基づいてずらす、という扱い.

  //   footstepNodesList[1:]で一回でも遊脚になった以降は、位置とYawをずらす. footstepNodesList[1:]で最初に遊脚になる脚があるならその反対の脚の偏差にあわせてずらす. そうでないなら両脚の中心(+-defaultTranslatePos)の偏差にあわせてずらす
  //   footstepNodesList[1]が支持脚なら、遊脚になるまで、位置姿勢をその足の今の偏差にあわせてずらす.

  if (stairTime != 0) {
    footstepNodesList[1].remainTime += 0.15;
  }
  stairTime = 0;

  for(int i=1;i<footstepNodesList.size();i++){
    if(footstepNodesList[i].isSupportPhase[RLEG] && !footstepNodesList[i].isSupportPhase[LLEG]){ // LLEGが次に最初に遊脚になる
      cnoid::Position origin = mathutil::orientCoordToAxis(footstepNodesList[i-1].dstCoords[RLEG], cnoid::Vector3::UnitZ()); // generate frame. 一つ前のRLEGの位置を基準にずらす
      cnoid::Position offsetedPos = mathutil::orientCoordToAxis(gaitParam.genCoords[RLEG].value() * footstepNodesList[0].dstCoords[RLEG].inverse() * footstepNodesList[i-1].dstCoords[RLEG], cnoid::Vector3::UnitZ()); // generate frame. 一つ前のRLEGはこの位置にずれている
      cnoid::Position transform = offsetedPos * origin.inverse(); // generate frame
      if(transform.translation().norm() < this->contactModificationThreshold) transform = cnoid::Position::Identity(); // ズレが大きい場合のみずらす
      transform.translation() -= destFootstepOffset;
      this->transformFutureSteps(footstepNodesList, i, transform); // footstepNodesList[1:]で一回でも遊脚になった以降の脚の、位置とYawをずらす
      break;
    }else if(footstepNodesList[i].isSupportPhase[LLEG] && !footstepNodesList[i].isSupportPhase[RLEG]){ // RLEGが次に最初に遊脚になる
      cnoid::Position origin = mathutil::orientCoordToAxis(footstepNodesList[i-1].dstCoords[LLEG], cnoid::Vector3::UnitZ()); // generate frame. 一つ前のLLEGの位置を基準にずらす
      cnoid::Position offsetedPos = mathutil::orientCoordToAxis(gaitParam.genCoords[LLEG].value() * footstepNodesList[0].dstCoords[LLEG].inverse() * footstepNodesList[i-1].dstCoords[LLEG], cnoid::Vector3::UnitZ()); // generate frame. 一つ前のLLEGはこの位置にずれている
      cnoid::Position transform = offsetedPos * origin.inverse(); // generate frame
      if(transform.translation().norm() < this->contactModificationThreshold) transform = cnoid::Position::Identity(); // ズレが大きい場合のみずらす
      transform.translation() -= destFootstepOffset;
      this->transformFutureSteps(footstepNodesList, i, transform); // footstepNodesList[1:]で一回でも遊脚になった以降の脚の、位置とYawをずらす
      break;
    }else if(!footstepNodesList[i].isSupportPhase[LLEG] && !footstepNodesList[i].isSupportPhase[RLEG]){ // RLEGとLLEG同時に次に最初に遊脚になる
      cnoid::Position rleg = footstepNodesList[i-1].dstCoords[RLEG]; // generate frame
      rleg.translation() -= rleg.linear() * gaitParam.defaultTranslatePos[RLEG].value();
      cnoid::Position lleg = footstepNodesList[i-1].dstCoords[LLEG];
      lleg.translation() -= lleg.linear() * gaitParam.defaultTranslatePos[LLEG].value();
      cnoid::Position origin = mathutil::orientCoordToAxis(mathutil::calcMidCoords({rleg, lleg}, {1.0, 1.0}), cnoid::Vector3::UnitZ()); // 一つ前の両脚の中心の位置を基準にずらす
      cnoid::Position offseted_rleg = gaitParam.genCoords[RLEG].value() * footstepNodesList[0].dstCoords[RLEG].inverse() * footstepNodesList[i-1].dstCoords[RLEG]; // generate frame
      offseted_rleg.translation() -= rleg.linear() * gaitParam.defaultTranslatePos[RLEG].value();
      cnoid::Position offseted_lleg = gaitParam.genCoords[LLEG].value() * footstepNodesList[0].dstCoords[LLEG].inverse() * footstepNodesList[i-1].dstCoords[LLEG];
      offseted_lleg.translation() -= lleg.linear() * gaitParam.defaultTranslatePos[LLEG].value();
      cnoid::Position offsetedPos = mathutil::orientCoordToAxis(mathutil::calcMidCoords({offseted_rleg, offseted_lleg}, {1.0, 1.0}), cnoid::Vector3::UnitZ());
      cnoid::Position transform = offsetedPos * origin.inverse(); // generate frame
      if(transform.translation().norm() < this->contactModificationThreshold) transform = cnoid::Position::Identity(); // ズレが大きい場合のみずらす
      this->transformFutureSteps(footstepNodesList, i, transform); // footstepNodesList[1:]で一回でも遊脚になった以降の脚の、位置とYawをずらす
      break;
    }
  }
  destFootstepOffset=cnoid::Vector3::Zero();
  for(int i=0;i<NUM_LEGS;i++){
    if(footstepNodesList[1].isSupportPhase[i] && gaitParam.elapsedTime > 0.1){
      //cnoid::Position targetCoords = footstepNodesList[0].dstCoords[i];
      //targetCoords.translation() = gaitParam.actEEPose[i].translation();
      //cnoid::Position transform = targetCoords * footstepNodesList[0].dstCoords[i].inverse(); // generate frame
      cnoid::Position transform = gaitParam.genCoords[i].value() * footstepNodesList[0].dstCoords[i].inverse();
      if(transform.translation().norm() < this->contactModificationThreshold) transform = cnoid::Position::Identity(); // ズレが大きい場合のみずらす
      this->transformCurrentSupportSteps(i, footstepNodesList, 1, transform); // 遊脚になるまで、位置姿勢をその足の今の偏差にあわせてずらす.
    }
  }

  // footstepNodesListをpop front
  footstepNodesList.erase(footstepNodesList.begin()); // vectorではなくlistにするべきかもしれないが、要素数がそんなに大きくないのでよしとする
  for(int i=0;i<NUM_LEGS;i++) {
    srcCoords[i] = gaitParam.genCoords[i].value();
    dstCoordsOrg[i] = footstepNodesList[0].dstCoords[i];
    swingState[i] = GaitParam::SWING_PHASE;
    isLandingGainPhase[i] = false;
  }
  remainTimeOrg = footstepNodesList[0].remainTime;
  elapsedTime = 0.0;
  relLandingHeight = -1e15;

  return true;
}

void FootStepGenerator::transformFutureSteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, int index, const cnoid::Position& transform/*generate frame*/) const{
  for(int l=0;l<NUM_LEGS;l++){
    bool swinged = false;
    for(int i=index;i<footstepNodesList.size();i++){
      if(!footstepNodesList[i].isSupportPhase[l]) swinged = true;
      if(swinged) footstepNodesList[i].dstCoords[l] = transform * footstepNodesList[i].dstCoords[l];
    }
  }
}

void FootStepGenerator::transformFutureSteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, int index, const cnoid::Vector3& transform/*generate frame*/) const{
  for(int l=0;l<NUM_LEGS;l++){
    bool swinged = false;
    for(int i=index;i<footstepNodesList.size();i++){
      if(!footstepNodesList[i].isSupportPhase[l]) swinged = true;
      if(swinged) footstepNodesList[i].dstCoords[l].translation() += transform;
    }
  }
}

// indexのsupportLegが次にswingするまでの間の位置を、generate frameでtransformだけ動かす
void FootStepGenerator::transformCurrentSupportSteps(int leg, std::vector<GaitParam::FootStepNodes>& footstepNodesList, int index, const cnoid::Position& transform/*generate frame*/) const{
  assert(0<=leg && leg < NUM_LEGS);
  for(int i=index;i<footstepNodesList.size();i++){
    if(!footstepNodesList[i].isSupportPhase[leg]) return;
    footstepNodesList[i].dstCoords[leg] = transform * footstepNodesList[i].dstCoords[leg];
  }
}

void FootStepGenerator::calcDefaultNextStep(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const GaitParam& gaitParam, const cnoid::Vector3& offset /*leg frame*/, bool stableStart) const{
  if(footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){
    if(footstepNodesList.back().endRefZmpState == GaitParam::FootStepNodes::refZmpState_enum::MIDDLE){
      // offsetを、両足の中間からの距離と解釈する(これ以外のケースでは支持脚からの距離と解釈する)
      cnoid::Position rleg = footstepNodesList[0].dstCoords[RLEG];
      rleg.translation() -= rleg.linear() * gaitParam.defaultTranslatePos[RLEG].value();
      cnoid::Position lleg = footstepNodesList[0].dstCoords[LLEG];
      lleg.translation() -= lleg.linear() * gaitParam.defaultTranslatePos[LLEG].value();
      cnoid::Position midCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{rleg, lleg}, std::vector<double>{1.0, 1.0});
      rleg = mathutil::orientCoordToAxis(rleg, cnoid::Vector3::UnitZ());
      lleg = mathutil::orientCoordToAxis(lleg, cnoid::Vector3::UnitZ());
      midCoords = mathutil::orientCoordToAxis(midCoords, cnoid::Vector3::UnitZ());
      // どっちをswingしてもいいので、進行方向に近いLegをswingする
      cnoid::Vector2 rlegTolleg = (gaitParam.defaultTranslatePos[LLEG].value() - gaitParam.defaultTranslatePos[RLEG].value()).head<2>(); // leg frame
      if(rlegTolleg.dot(offset.head<2>()) > 0) { // LLEGをswingする
        if(stableStart && footstepNodesList.size() == 1) { // 現在の時刻から突然refZmpTrajが変化すると、大きなZMP入力変化が必要になる. いまの位置でrefZmpTrajをthis->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio)の間とめる
          footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
        }
        // this->defaultStepTime * this->defaultDoubleSupportRatioの間で次の支持脚側に動かす
        if(footstepNodesList.size() <= 2) { // 末尾に両足支持期を追加
          footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::RLEG));
        }else{ // 末尾の両足支持期を直接編集
          footstepNodesList.back().remainTime = this->defaultStepTime * this->defaultDoubleSupportRatio; // 両足支持期を延長 or 短縮
          footstepNodesList.back().endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::RLEG;
        }
        cnoid::Position rlegTomidCoords = rleg.inverse() * midCoords;
        cnoid::Vector3 offset_rlegLocal;
        offset_rlegLocal.head<2>() = rlegTomidCoords.translation().head<2>() + (rlegTomidCoords.linear() * cnoid::Vector3(offset[0],offset[1],0.0)).head<2>();
        offset_rlegLocal[2] = cnoid::rpyFromRot(rlegTomidCoords.linear())[2] + offset[2];
        footstepNodesList.push_back(calcDefaultSwingStep(LLEG, footstepNodesList.back(), gaitParam, offset_rlegLocal)); // LLEGをswingする
        footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::LLEG));
      }else{ // RLEGをswingする
        if(stableStart && footstepNodesList.size() == 1) { // 現在の時刻から突然refZmpTrajが変化すると、大きなZMP入力変化が必要になる. いまの位置でrefZmpTrajをthis->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio)の間とめる
          footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
        }
        // this->defaultStepTime * this->defaultDoubleSupportRatioの間で次の支持脚側に動かす
        if(footstepNodesList.size() <= 2) { // 末尾に両足支持期を追加
          footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::LLEG));
        }else{ // 末尾の両足支持期を直接編集
          footstepNodesList.back().remainTime = this->defaultStepTime * this->defaultDoubleSupportRatio; // 両足支持期を延長 or 短縮
          footstepNodesList.back().endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::LLEG;
        }
        cnoid::Position llegTomidCoords = lleg.inverse() * midCoords;
        cnoid::Vector3 offset_llegLocal;
        offset_llegLocal.head<2>() = llegTomidCoords.translation().head<2>() + (llegTomidCoords.linear() * cnoid::Vector3(offset[0],offset[1],0.0)).head<2>();
        offset_llegLocal[2] = cnoid::rpyFromRot(llegTomidCoords.linear())[2] + offset[2];
        footstepNodesList.push_back(calcDefaultSwingStep(RLEG, footstepNodesList.back(), gaitParam, offset_llegLocal)); // RLEGをswingする
        footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::RLEG));
      }
    }else if(footstepNodesList.back().endRefZmpState == GaitParam::FootStepNodes::refZmpState_enum::LLEG){ // 前回LLEGをswingした
        footstepNodesList.push_back(calcDefaultSwingStep(RLEG, footstepNodesList.back(), gaitParam, offset)); // RLEGをswingする
        footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::RLEG));
    }else{ // 前回RLEGをswingした
        footstepNodesList.push_back(calcDefaultSwingStep(LLEG, footstepNodesList.back(), gaitParam, offset)); // LLEGをswingする
        footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::LLEG));
    }
  }else if(footstepNodesList.back().isSupportPhase[RLEG] && !footstepNodesList.back().isSupportPhase[LLEG]){ // LLEGが浮いている
    footstepNodesList.push_back(calcDefaultSwingStep(LLEG, footstepNodesList.back(), gaitParam, offset, true)); // LLEGをswingする. startWithSingleSupport
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::LLEG));
  }else if(!footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){ // RLEGが浮いている
    footstepNodesList.push_back(calcDefaultSwingStep(RLEG, footstepNodesList.back(), gaitParam, offset, true)); // RLEGをswingする. startWithSingleSupport
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::RLEG));
  }// footstepNodesListの末尾の要素が両方falseであることは無い
}

GaitParam::FootStepNodes FootStepGenerator::calcDefaultSwingStep(const int& swingLeg, const GaitParam::FootStepNodes& footstepNodes, const GaitParam& gaitParam, const cnoid::Vector3& offset, bool startWithSingleSupport) const{
  GaitParam::FootStepNodes fs;
  int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;

  cnoid::Position transform = cnoid::Position::Identity(); // supportLeg相対(Z軸は鉛直)での次のswingLegの位置
  double theta = mathutil::clamp(offset[2],this->defaultStrideLimitationMinTheta[swingLeg],this->defaultStrideLimitationMaxTheta[swingLeg]);
  transform.linear() = cnoid::Matrix3(Eigen::AngleAxisd(theta, cnoid::Vector3::UnitZ()));
  transform.translation() = - gaitParam.defaultTranslatePos[supportLeg].value() + cnoid::Vector3(offset[0], offset[1], 0.0) + transform.linear() * gaitParam.defaultTranslatePos[swingLeg].value();
  std::vector<cnoid::Vector3> strideLimitationHull = this->calcRealStrideLimitationHull(swingLeg, theta, gaitParam.legHull, gaitParam.defaultTranslatePos, this->defaultStrideLimitationHull);
  transform.translation() = mathutil::calcNearestPointOfHull(transform.translation(), strideLimitationHull);

  fs.dstCoords[supportLeg] = footstepNodes.dstCoords[supportLeg];
  cnoid::Position prevOrigin = mathutil::orientCoordToAxis(footstepNodes.dstCoords[supportLeg], cnoid::Vector3::UnitZ());
  fs.dstCoords[swingLeg] = prevOrigin * transform;
  fs.isSupportPhase[supportLeg] = true;
  fs.isSupportPhase[swingLeg] = false;
  fs.remainTime = this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio);
  fs.endRefZmpState = (supportLeg == RLEG) ? GaitParam::FootStepNodes::refZmpState_enum::RLEG : GaitParam::FootStepNodes::refZmpState_enum::LLEG;
  if(!startWithSingleSupport) fs.stepHeight[swingLeg] = {this->defaultStepHeight,this->defaultStepHeight};
  else fs.stepHeight[swingLeg] = {0.0,this->defaultStepHeight};
  fs.touchVel[swingLeg] = this->touchVel;
  fs.goalOffset[swingLeg] = 0.0;
  return fs;
}

GaitParam::FootStepNodes FootStepGenerator::calcDefaultDoubleSupportStep(const GaitParam::FootStepNodes& footstepNodes, double doubleSupportTime, GaitParam::FootStepNodes::refZmpState_enum endRefZmpState) const{
  GaitParam::FootStepNodes fs;
  for(int i=0;i<NUM_LEGS;i++){
    fs.dstCoords[i] = footstepNodes.dstCoords[i];
    fs.isSupportPhase[i] = true;
    fs.stepHeight[i] = {0.0,0.0};
  }
  fs.remainTime = doubleSupportTime;
  fs.endRefZmpState = endRefZmpState;
  return fs;
}

inline std::ostream &operator<<(std::ostream &os, const std::vector<std::pair<std::vector<cnoid::Vector3>, double> >& candidates){
  for(int i=0;i<candidates.size();i++){
    os << "candidates[" << i << "] " << candidates[i].second << "s" << std::endl;
    os << candidates[i].first << std::endl;
  }
  return os;
}

void FootStepGenerator::modifyFootSteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, std::vector<GaitParam::SwingState_enum>& swingState, // input & output
                                        GaitParam::DebugData& debugData, //for Log
                                        const GaitParam& gaitParam) const{
  debugData.cpViewerLog[18] = 0;
  // TODO gaitParam.lの処理がちゃんと考えられていない
  // 現在片足支持期で、次が両足支持期であるときのみ、行う
  if(!(footstepNodesList.size() > 1 &&
       (footstepNodesList[1].isSupportPhase[RLEG] && footstepNodesList[1].isSupportPhase[LLEG]) &&
       ((footstepNodesList[0].isSupportPhase[RLEG] && !footstepNodesList[0].isSupportPhase[LLEG]) || (!footstepNodesList[0].isSupportPhase[RLEG] && footstepNodesList[0].isSupportPhase[LLEG]))))
     return;

  // CapturePointが遊脚の着地位置に行かない場合には行わない
  if((!footstepNodesList[0].isSupportPhase[RLEG] && footstepNodesList[1].endRefZmpState != GaitParam::FootStepNodes::refZmpState_enum::RLEG) ||
     (!footstepNodesList[0].isSupportPhase[LLEG] && footstepNodesList[1].endRefZmpState != GaitParam::FootStepNodes::refZmpState_enum::LLEG))
    return;

  //swingLeg, supportLeg
  int swingLeg = footstepNodesList[0].isSupportPhase[RLEG] ? LLEG : RLEG;
  int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;
  cnoid::Position supportPose = gaitParam.genCoords[supportLeg].value(); // TODO. 支持脚のgenCoordsとdstCoordsが異なることは想定していない
  cnoid::Position supportPoseHorizontal = mathutil::orientCoordToAxis(supportPose, cnoid::Vector3::UnitZ());

  debugData.cpViewerLog[19] = swingState[swingLeg];

  // GROUND_PHASEでは位置時間修正を行わない.
  if(swingState[swingLeg] == GaitParam::GROUND_PHASE) return;

  // stopCurrentPositionが発動しているなら、着地位置時間修正を行っても無駄なので行わない
  if(footstepNodesList[0].stopCurrentPosition[swingLeg]) return;

  // 現在の遊脚軌道目標(これを現在の遊脚位置と考える)
  cnoid::Position nowCoords;
  cnoid::Vector6 nowVel;
  gaitParam.genCoords[swingLeg].getGoal(nowCoords, nowVel);
  cnoid::Vector3 nowPos = nowCoords.translation();

  // dx = w ( x - z - l)
  cnoid::Vector3 actDCM = gaitParam.actCog + gaitParam.actCogVel.value() / gaitParam.omega;

  cnoid::Vector3 prevPos = footstepNodesList[0].dstCoords[swingLeg].translation();
  double prevHeight = -1e10;
  double prevMinLength = 0.010; //前回posからの距離が10mm以内の足場と同じ高さの足場しか同じ足場としない
  for(int i=0;i<gaitParam.steppableRegion.size();i++){
    if (mathutil::isInsideHull(prevPos, gaitParam.steppableRegion[i])) {
      prevHeight = 0.01 * std::floor(100 * gaitParam.steppableHeight[i]);
      break;
    }
    cnoid::Vector3 tmpPos = mathutil::calcNearestPointOfHull(prevPos, gaitParam.steppableRegion[i]);
    if ((tmpPos - prevPos).head(2).norm() < prevMinLength) {
      prevHeight = 0.01 * std::floor(100 * gaitParam.steppableHeight[i]);
      prevMinLength = (tmpPos - prevPos).head(2).norm();
    }
  }

  //double minTime = std::max(this->overwritableMinTime, this->overwritableMinStepTime + stairTime - gaitParam.elapsedTime); // 次indexまでの残り時間がthis->overwritableMinTimeを下回るようには着地時間修正を行わない. 現index開始時からの経過時間がthis->overwritableStepMinTimeを下回るようには着地時間修正を行わない.
  //minTime = std::min(minTime, footstepNodesList[0].remainTime); // もともと下回っている場合には、その値を下回るようには着地時刻修正を行わない.
  //double maxTime = std::max(this->overwritableMaxStepTime - gaitParam.elapsedTime, minTime); // 現index開始時からの経過時間がthis->overwritableStepMaxTimeを上回るようには着地時間修正を行わない.
  //maxTime = std::max(maxTime, footstepNodesList[0].remainTime); // もともと上回っている場合には、その>値を上回るようには着地時刻修正を行わない.
  //maxTime += footstepNodesList[1].remainTime;


  //strideLimitationHull, 遊脚の幾何的な着地可能領域
  std::vector<cnoid::Vector3> strideLimitationHull; // generate frame. overwritableStrideLimitationHullの範囲内の着地位置(自己干渉・IKの考慮が含まれる). Z成分には0を入れる
  for(int i=0;i<this->overwritableStrideLimitationHull[swingLeg].size();i++){
    cnoid::Vector3 p = supportPoseHorizontal * this->overwritableStrideLimitationHull[swingLeg][i];
    strideLimitationHull.emplace_back(p[0],p[1],0.0);
  }
  debugData.strideLimitationHull = strideLimitationHull; // for debeg

  //高さごとのreachableCaptureRegion
  //高さは1cm刻み、辞書型になっている。
  std::map<double, std::vector<cnoid::Vector3> > reachableCaptureRegionHulls;
  //基本高さのみ計算
  {
    std::vector<cnoid::Vector3> tmpReachableCaptureRegionHull = std::vector<cnoid::Vector3>();
    if (prevHeight > -1e5) {
      calcReachableCaptureRegion(tmpReachableCaptureRegionHull, prevHeight, actDCM, gaitParam, debugData);
    } else {//prevHeightがない
      calcReachableCaptureRegion(tmpReachableCaptureRegionHull, 0.01 * std::floor(100 * footstepNodesList[0].dstCoords[swingLeg].translation()[2]), actDCM, gaitParam, debugData);
    }
    reachableCaptureRegionHulls.insert(std::make_pair(0.01 * std::floor(100 * footstepNodesList[0].dstCoords[swingLeg].translation()[2]), tmpReachableCaptureRegionHull));
    debugData.reachableCaptureRegionHull = tmpReachableCaptureRegionHull; // for debug
  }

  //支持脚接触凸包
  std::vector<cnoid::Vector3> genSafeSupportLegHull;
  genSafeSupportLegHull.resize(this->safeLegHull[supportLeg].size());
  for (int i=0; i<this->safeLegHull[supportLeg].size(); i++) {
    genSafeSupportLegHull[i] = supportPoseHorizontal * this->safeLegHull[supportLeg][i] + gaitParam.l;
    debugData.cpViewerLog[i*2+0] = genSafeSupportLegHull[i][0];
    debugData.cpViewerLog[i*2+1] = genSafeSupportLegHull[i][1];
  }

  //修正前目標着地位置
  //基本的に前回の修正後目標着地位置を用いるが、少しずつオリジナルの目標着地位置に近づけていく
  cnoid::Vector3 destPos = 0.995 * footstepNodesList[0].dstCoords[swingLeg].translation() + 0.005 * gaitParam.dstCoordsOrg[swingLeg].translation();
  double destTime = 0.995 * footstepNodesList[0].remainTime + 0.005 * std::max(0.0, (gaitParam.remainTimeOrg/* + stairTime*/) - gaitParam.elapsedTime);
  debugData.cpViewerLog[8] = destPos[0];
  debugData.cpViewerLog[9] = destPos[1];

  cnoid::Vector3 newPos = cnoid::Vector3(1e+10, 1e+10, 0);
  cnoid::Vector3 newShort = cnoid::Vector3(1e+12, 1e+12, 0);
  double newTime = 0;
  double newHeight = footstepNodesList[0].dstCoords[swingLeg].translation()[2];
  bool newPhaseChangeFlag = false;
  int newDebugState = 0;

  for(int i=0;i<std::max(1, (int)(gaitParam.steppableRegion.size()));i++){ //gaitParam.steppableRegion.size()が0のときも１度だけ行う
    cnoid::Vector3 tmpPos = destPos;
    cnoid::Vector3 tmpShort = cnoid::Vector3::Zero();
    double tmpTime = destTime;
    bool isIn = false;
    bool tmpPhaseChangeFlag = false;
    int tmpDebugState = 9;
    double flooredSteppableHeight;
    if (gaitParam.steppableRegion.size() == 0) { //SR受け取っていない時
      flooredSteppableHeight = 0.01 * std::floor(100 * footstepNodesList[0].dstCoords[swingLeg].translation()[2]);
    } else {
      flooredSteppableHeight = 0.01 * std::floor(100 * gaitParam.steppableHeight[i]);
    }

    //HEIHGT_FIX_SWING_PHASE及びDOWN_PHASEは同じ高さのSR内にしか着地位置修正を行わない TODO:本当は同じ足場にすべき
    if (gaitParam.steppableRegion.size() > 0 && (swingState[swingLeg] == GaitParam::HEIGHT_FIX_SWING_PHASE || swingState[swingLeg] == GaitParam::DOWN_PHASE) && (prevHeight != flooredSteppableHeight || nowPos[2] < flooredSteppableHeight)) continue;

    //strideLimitationを考慮したsteppableRegion
    std::vector<cnoid::Vector3> tmpSteppableRegion;
    if (gaitParam.steppableRegion.size() == 0) { //steppableRegionを受け取っていないときはstrideLimitationをそのまま用いる
      tmpSteppableRegion = strideLimitationHull;
    } else {
      tmpSteppableRegion = mathutil::calcIntersectConvexHull(gaitParam.steppableRegion[i], strideLimitationHull);
    }
    if(tmpSteppableRegion.size() < 3) continue;

    //reachableCaptureRegionを計算
    if (reachableCaptureRegionHulls.find(flooredSteppableHeight) == reachableCaptureRegionHulls.end()) { //既存のcaptureRegionにない
      std::vector<cnoid::Vector3> tmpReachableCaptureRegionHull = std::vector<cnoid::Vector3>();
      double fuga = calcReachableCaptureRegion(tmpReachableCaptureRegionHull, flooredSteppableHeight, actDCM, gaitParam, debugData); //TODO:RCRの計算のためにflooredSteppableHeightを使用している
      reachableCaptureRegionHulls.insert(std::make_pair(flooredSteppableHeight, tmpReachableCaptureRegionHull));
    }

    //reachableCaptureRegionを考慮
    if (reachableCaptureRegionHulls.at(flooredSteppableHeight).size() >= 3) { //RCRが存在している
      std::vector<cnoid::Vector3> targetRegion;
      targetRegion = mathutil::calcIntersectConvexHull(tmpSteppableRegion, reachableCaptureRegionHulls[flooredSteppableHeight]);

      //modify landing pos
      if(targetRegion.size() < 3){ //重なりがない　角運動量補償 or 即下りる
        std::vector<cnoid::Vector3> p, q;
        double distance = mathutil::calcNearestPointOfTwoHull(tmpSteppableRegion, reachableCaptureRegionHulls[flooredSteppableHeight], p, q);
        tmpPos = mathutil::calcNearestPointOfHull(destPos, p);
        cnoid::Vector3 tmpcpPos = mathutil::calcNearestPointOfHull(tmpPos, q);
        tmpShort = (tmpcpPos - tmpPos);
        tmpShort[2] = 0;
        tmpTime = 0;
        //tmpPosに遊脚がたどり着ける保証はない・・・が、maxSwingVelの範囲で近づいてくれるはず
        
        tmpDebugState = 1;

        //支持脚側に倒れそうになったときは現在位置に即下りる
        if ((genSafeSupportLegHull[0] - genSafeSupportLegHull[1]).cross(tmpPos - genSafeSupportLegHull[1])[0] * (genSafeSupportLegHull[0] - genSafeSupportLegHull[1]).cross(tmpcpPos - genSafeSupportLegHull[1])[0] < 0 &&
            (genSafeSupportLegHull[3] - genSafeSupportLegHull[4]).cross(tmpPos - genSafeSupportLegHull[4])[0] * (genSafeSupportLegHull[3] - genSafeSupportLegHull[4]).cross(tmpcpPos - genSafeSupportLegHull[4])[0] < 0 ) {

          tmpPos = mathutil::calcNearestPointOfHull(nowPos, targetRegion); //現在位置に一番近い場所
          tmpShort = cnoid::Vector3(1e+10, 1e+10, 0) + cnoid::Vector3((nowPos - tmpPos).head(2).norm(), 0, 0);
          tmpTime = 0;
          tmpDebugState = 2;
        }
      } else {
        isIn = mathutil::isInsideHull(destPos, targetRegion);
        tmpPos = mathutil::calcNearestPointOfHull(destPos, targetRegion);
        tmpShort = cnoid::Vector3::Zero();
        tmpDebugState = 3;
      }

      //modify landing time
      //足平長方形仮定
      //TODO Wheel
      if (swingState[swingLeg] == GaitParam::SWING_PHASE) {
        modifyTime(tmpTime, tmpPhaseChangeFlag, tmpTime, cnoid::Vector3(tmpPos[0], tmpPos[1], flooredSteppableHeight), actDCM, gaitParam, debugData);
      }
    } else if (reachableCaptureRegionHulls.at(flooredSteppableHeight).size() == 2) { //RCRは存在しないので、できるだけ早く最善の場所へ行く
      std::vector<cnoid::Vector3> targetRegion;
      cnoid::Vector3 tmpcpPos = mathutil::calcNearestPointOfHull(destPos, reachableCaptureRegionHulls[flooredSteppableHeight]);
      tmpPos = mathutil::calcNearestPointOfHull(tmpcpPos, tmpSteppableRegion);

      //支持脚側に倒れそうになったときは現在位置に即下りる
      if ((genSafeSupportLegHull[0] - genSafeSupportLegHull[1]).cross(tmpPos - genSafeSupportLegHull[1])[0] * (genSafeSupportLegHull[0] - genSafeSupportLegHull[1]).cross(tmpcpPos - genSafeSupportLegHull[1])[0] < 0 &&
          (genSafeSupportLegHull[3] - genSafeSupportLegHull[4]).cross(tmpPos - genSafeSupportLegHull[4])[0] * (genSafeSupportLegHull[3] - genSafeSupportLegHull[4]).cross(tmpcpPos - genSafeSupportLegHull[4])[0] < 0 ) {

        tmpPos = mathutil::calcNearestPointOfHull(nowPos, targetRegion); //現在位置に一番近い場所
        tmpShort = cnoid::Vector3(1e+3, 1e+3, 0) + cnoid::Vector3((nowPos - tmpPos).head(2).norm(), 0, 0);
        modifyTime(tmpTime, tmpPhaseChangeFlag, 0, cnoid::Vector3(tmpPos[0], tmpPos[1], flooredSteppableHeight), actDCM, gaitParam, debugData); //できるだけ早く着地
        tmpDebugState = 2;
      } else {
        tmpShort = cnoid::Vector3(1e+3, 1e+3, 0) + cnoid::Vector3((tmpcpPos - tmpPos).head(2).norm(), 0, 0);
        modifyTime(tmpTime, tmpPhaseChangeFlag, 0, cnoid::Vector3(tmpPos[0], tmpPos[1], flooredSteppableHeight), actDCM, gaitParam, debugData); //できるだけ早く着地
        tmpDebugState = 6;
      }
    } else { //reachableCaptureRegionが存在しない
      if (swingState[swingLeg] == GaitParam::SWING_PHASE) {
        //できるだけ早く現在目標位置に着地する
        std::vector<cnoid::Vector3> targetRegion = tmpSteppableRegion;
        tmpPos = mathutil::calcNearestPointOfHull(footstepNodesList[0].dstCoords[swingLeg].translation(), targetRegion); //現在目標位置に一番近い場所
        tmpShort = cnoid::Vector3(1e+11, 1e+11, 0); //Short優先度一番低く TODO これをそのまま角運動量補正に使わないこと！！！！！
        modifyTime(tmpTime, tmpPhaseChangeFlag, 0, cnoid::Vector3(tmpPos[0], tmpPos[1], flooredSteppableHeight), actDCM, gaitParam, debugData); //できるだけ早く着地
        tmpDebugState = 4;
      } else {
        //着地位置修正をせずそのまま下りる
        tmpPos = footstepNodesList[0].dstCoords[swingLeg].translation();
        tmpShort = cnoid::Vector3(1e+11, 1e+11, 0); //Short優先度一番低く TODO これをそのまま角運動量補正に使わないこと！！！！！
        tmpDebugState = 5;
      }
    }

    //update newPos
    if (tmpShort.head(2).norm() < newShort.head(2).norm() || (tmpShort.head(2).norm() == newShort.head(2).norm() && (destPos - tmpPos).head(2).norm() < (destPos - newPos).head(2).norm())) {
      newPos = tmpPos;
      newShort = tmpShort;
      newDebugState = tmpDebugState;
      if (swingState[swingLeg] == GaitParam::SWING_PHASE) {
        newTime = tmpTime;
        newPhaseChangeFlag = tmpPhaseChangeFlag;
        if (gaitParam.steppableRegion.size() == 0) {
          newHeight = footstepNodesList[0].dstCoords[swingLeg].translation()[2];
        } else {
          newHeight = std::max(gaitParam.steppableHeight[i], footstepNodesList[0].dstCoords[swingLeg].translation()[2]);
        }
        //newHeight = footstepNodesList[0].dstCoords[swingLeg].translation()[2];
      }
    }
    if(isIn) break;
  }
  debugData.cpViewerLog[18] = newDebugState;

  if(newPos.head(2).norm() > 1e+8){
    //着地位置修正を行わない
    debugData.cpViewerLog[18] = 7;
    return;
  }

  //landingHeightから受け取った値を用いて着地高さを変更
  //ココでのHeight変更はDOWN_PHASEも受け付ける
  if (gaitParam.relLandingHeight > -1e+10) {
    double tmp = gaitParam.relLandingHeight - newHeight;
    newHeight = gaitParam.relLandingHeight;
  }

  //次の周回からstairTimeを有効化
  if (std::abs(newHeight - gaitParam.srcCoords[swingLeg].translation()[2]) > 0.05) {
    stairTime = 0.3;
  } else {
    stairTime = 0.0;
  }

  cnoid::Vector3 displacement = cnoid::Vector3(newPos[0], newPos[1], newHeight) - footstepNodesList[0].dstCoords[swingLeg].translation();
  //destFootstepOffset = cnoid::Vector3(newPos[0], newPos[1], 0) - gaitParam.dstCoordsOrg[swingLeg].translation();
  //destFootstepOffset[2] = 0;
  destFootstepOffset = cnoid::Vector3(std::min(0.2, std::max(-0.2, 1.0 * (newPos[0] - gaitParam.dstCoordsOrg[swingLeg].translation()[0]))), std::min(0.2, std::max(-0.2, 1.0 * (newPos[1] - gaitParam.dstCoordsOrg[swingLeg].translation()[1]))), 0);
  //displacement[2] = 0.0;
  this->transformFutureSteps(footstepNodesList, 0, displacement);
  if (swingState[swingLeg] == GaitParam::SWING_PHASE) {
    footstepNodesList[0].remainTime = newTime;
    if (newPhaseChangeFlag) { //遊脚を最速で下ろす予定のときはHEIGH_FIX_SWING_PHASEへ移行する
      swingState[swingLeg] = GaitParam::HEIGHT_FIX_SWING_PHASE;
    }
  }
  debugData.cpViewerLog[20] = footstepNodesList[0].remainTime;
  debugData.cpViewerLog[21] = gaitParam.elapsedTime;
  debugData.cpViewerLog[37] = newPhaseChangeFlag;
  debugData.cpViewerLog[38] = prevHeight;

  //landingHeightから受け取った値を用いて着地姿勢を変更
  if (gaitParam.relLandingHeight > -1e+10) {
    for(int i=0; i<footstepNodesList.size(); i++) {
      footstepNodesList[i].dstCoords[swingLeg].linear() = mathutil::orientCoordToAxis(gaitParam.dstCoordsOrg[swingLeg].linear(), gaitParam.relLandingNormal);
    }
  }
}


//遊脚可到達性、支持脚等速移動（脚車輪）を考慮したCaptureRegion計算
//足平が長方形であると仮定、遊脚軌道がXY分離可能と仮定
double FootStepGenerator::calcReachableCaptureRegion(std::vector<cnoid::Vector3>& reachableCaptureRegionHull, const double& stepHeight, const cnoid::Vector3& actDCM, const GaitParam& gaitParam, GaitParam::DebugData& debugData) const {
  double crErrorNum = 0;
  //遊脚・支持脚
  int swingLeg = gaitParam.footstepNodesList[0].isSupportPhase[RLEG] ? LLEG : RLEG;
  int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;
  cnoid::Position supportPose = gaitParam.genCoords[supportLeg].value();
  cnoid::Position supportPoseHorizontal = mathutil::orientCoordToAxis(supportPose, cnoid::Vector3::UnitZ());

  // 現在の遊脚軌道目標(これを現在の遊脚位置と考える)
  cnoid::Position swingCoords;
  cnoid::Vector6 swingVel;
  gaitParam.genCoords[swingLeg].getGoal(swingCoords, swingVel);
  cnoid::Vector3 swingPos = swingCoords.translation();


  double liftTime = 0;
  double tmin = 0;
  double swingZMinTime; //足の上げ下げのための最小時間
  if (gaitParam.swingState[swingLeg] == GaitParam::SWING_PHASE) {
    double targetHeight = std::max(gaitParam.srcCoords[swingLeg].translation()[2] + gaitParam.footstepNodesList[0].stepHeight[swingLeg][0], stepHeight + gaitParam.footstepNodesList[0].stepHeight[swingLeg][1]);
    double airHeight = targetHeight - gaitParam.footstepNodesList[0].stepHeight[swingLeg][1] * 0.7;
    if (airHeight > swingPos[2]) {
      swingZMinTime = (2*airHeight - swingPos[2] - stepHeight) / gaitParam.maxSwingVel[2];
    } else {
      swingZMinTime = std::max(0.0, (double)(swingPos[2] - stepHeight)) / gaitParam.maxSwingVel[2];
    }

    if (airHeight > swingPos[2]) {
      liftTime = (airHeight - swingPos[2]) / gaitParam.maxSwingVel[2];
    }
  } else {
    swingZMinTime = std::max(0.0, (double)(swingPos[2] - stepHeight)) / gaitParam.maxSwingVel[2];
    liftTime = 0;
  }
  tmin = liftTime + gaitParam.delayTimeOffset + swingXYMarginTime + std::min(0.1, gaitParam.footstepNodesList[1].remainTime * 0.5); //0.05は遅着きをケアするため。遊脚が動けない時間

  //supportPoseHorizontal座標系に直す（xy分離のため） これ以降Z成分に意味がなくなる
  cnoid::Vector3 cp = supportPoseHorizontal.inverse() * actDCM;
  swingPos = supportPoseHorizontal.inverse() * swingPos;

  //cnoid::Vector3 vmax = cnoid::Vector3(1.5, 1.5, 0.0);;

  //safeLegHullが長方形であることを仮定(xy分離のため)
  cnoid::Vector3 zmpMin = cnoid::Vector3::Zero();
  cnoid::Vector3 zmpMax = cnoid::Vector3::Zero();
  for (int i=0; i<this->safeLegHull[supportLeg].size(); i++) {
    if (zmpMin[0]  > this->safeLegHull[supportLeg][i][0]) zmpMin[0] = this->safeLegHull[supportLeg][i][0];
    if (zmpMax[0]  < this->safeLegHull[supportLeg][i][0]) zmpMax[0] = this->safeLegHull[supportLeg][i][0];
    if (zmpMin[1]  > this->safeLegHull[supportLeg][i][1]) zmpMin[1] = this->safeLegHull[supportLeg][i][1];
    if (zmpMax[1]  < this->safeLegHull[supportLeg][i][1]) zmpMax[1] = this->safeLegHull[supportLeg][i][1];
  }

  //debugData.cpViewerLog[15] = omega;
  //debugData.cpViewerLog[16] = cp[0];
  //debugData.cpViewerLog[17] = cp[1];
  //debugData.cpViewerLog[18] = swingPose.translation()[0];
  //debugData.cpViewerLog[19] = swingPose.translation()[1];

  std::vector<std::vector<double> > samplingTime = std::vector<std::vector<double> >{std::vector<double>(), std::vector<double>(), std::vector<double>()};//0...x, 1...y, 2...merge
  if (gaitParam.swingState[swingLeg] != GaitParam::SWING_PHASE) {
    //samplingTime[2].push_back(swingZMinTime + gaitParam.delayTimeOffset + std::min(0.05, gaitParam.footstepNodesList[1].remainTime * 0.2));
    if (gaitParam.footstepNodesList[0].remainTime + std::min(0.1, gaitParam.footstepNodesList[1].remainTime * 0.5) <= tmin) {
      reachableCaptureRegionHull = std::vector<cnoid::Vector3>();
      return 3;
    }
    samplingTime[2].push_back(gaitParam.footstepNodesList[0].remainTime + std::min(0.1, gaitParam.footstepNodesList[1].remainTime * 0.5));
  } else {
    double minTime = swingZMinTime + gaitParam.delayTimeOffset + std::min(0.1, gaitParam.footstepNodesList[1].remainTime * 0.5);
    double maxTime = 10.0; //無限大を避けるため

    //CRが存在するMinMaxTimeを計算し、CR凸包のためのサンプリングタイムを決定
    std::vector<double> extTime{0, 0};
    for (int i=0; i<2; i++) { //X,Y
      double tmpMin_zmpMin, tmpMax_zmpMin, tmpMin_zmpMax, tmpMax_zmpMax;
      tmpMin_zmpMin = tmpMin_zmpMax = tmin, tmpMax_zmpMin = tmpMax_zmpMax = 20;
      double distance_zmpMin = calcCRMinMaxTime(tmpMin_zmpMin, tmpMax_zmpMin, 0.001, gaitParam.omega, cp[i], zmpMin[i], tmin, gaitParam.maxSwingVel[i], swingPos[i]);
      double distance_zmpMax = calcCRMinMaxTime(tmpMin_zmpMax, tmpMax_zmpMax, 0.001, gaitParam.omega, cp[i], zmpMax[i], tmin, gaitParam.maxSwingVel[i], swingPos[i]);
      double insideFlag = false;
      if (fcp(tmin, gaitParam.omega, cp[i], zmpMax[i]) < swingPos[i] && swingPos[i] < fcp(tmin, gaitParam.omega, cp[i], zmpMin[i])) { //zmpMinMax反転するので注意
        samplingTime[i].push_back(tmin);
        insideFlag = true;
      }
      if (fcp(maxTime, gaitParam.omega, cp[i], zmpMax[i]) < fsw(maxTime, tmin, gaitParam.maxSwingVel[i], swingPos[i]) && fsw(maxTime, tmin, -gaitParam.maxSwingVel[i], swingPos[i]) < fcp(maxTime, gaitParam.omega, cp[i], zmpMin[i])) {
        samplingTime[i].push_back(maxTime);
      }

      if (!insideFlag && distance_zmpMin > 0 && distance_zmpMax > 0) {
        //CRない
        if (distance_zmpMin < distance_zmpMax) {
          extTime[i] = tmpMin_zmpMin;
        } else {
          extTime[i] = tmpMin_zmpMax;
        }
      }
      if (distance_zmpMin == 0) {
        samplingTime[i].push_back(tmpMin_zmpMin);
        samplingTime[i].push_back(tmpMax_zmpMin);
      }
      if (distance_zmpMax == 0) {
        samplingTime[i].push_back(tmpMin_zmpMax);
        samplingTime[i].push_back(tmpMax_zmpMax);
      }

      std::sort(samplingTime[i].begin(), samplingTime[i].end());
    }

    //CRない時
    if (extTime[0] != 0 || extTime[1] != 0) {
      crErrorNum = 1;
      double addTime = 0;
      if (extTime[0] != 0 &&  extTime[1] != 0) {
        addTime = std::min(extTime[0], extTime[1]);
      } else {
        addTime = std::max(extTime[0], extTime[1]);
      }
      addTime = std::max(minTime, addTime);
      samplingTime[2].push_back(addTime);
    } else {
      //min max time との比較
      for (int i = 0; i < 2; i++) {
        if (samplingTime[i].back() < minTime || maxTime < samplingTime[i][0]) { //遊脚ZのminmaxTimeの範囲にない
          crErrorNum = 2;
          reachableCaptureRegionHull = std::vector<cnoid::Vector3>();
          return 2;
        }
        if (samplingTime[i][0] < minTime) samplingTime[i].push_back(minTime);
        if (maxTime < samplingTime[i].back()) samplingTime[i].push_back(maxTime);
        std::sort(samplingTime[i].begin(), samplingTime[i].end());
      }

      if(samplingTime[0].size() < 2 || samplingTime[1].size() < 2) { //起きないはず
        std::cout << "ssssssssssssssssssaaaaaaaaaaaaaaaaaaaaaaammmmmmmmmmmmmmmmmmmmmppppppppppppppppppppplllllllllllllllllllllllliiiiiiiiiiiiiiiiiiiiinnnnnnnnnnnnnnnnnnnnnggggggggggggggggeeeeeeeeeeeeeeerrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrroooooooooooooooooooooooooorrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr" << std::endl;
        samplingTime[0].push_back(tmin);
        samplingTime[0].push_back(maxTime);
        samplingTime[1].push_back(tmin);
        samplingTime[1].push_back(maxTime);
        std::cout << samplingTime[0].size() << " " << samplingTime[1].size() << std::endl;
      }

      double tmpMin = std::max(std::max(samplingTime[0][0], samplingTime[1][0]), minTime);
      double tmpMax = std::min(std::min(samplingTime[0].back(), samplingTime[1].back()), maxTime);
      for (int i = 0; i < 2; i++) {
        for (int j = 0; j < samplingTime[i].size(); j++) {
          if (tmpMin <= samplingTime[i][j] && samplingTime[i][j] <= tmpMax) samplingTime[2].push_back(samplingTime[i][j]);
        }
      }

      for (int i = 1; i < 4; i++) { //補間
        samplingTime[2].push_back(tmpMin + (tmpMax - tmpMin) * double(i) / 4.0);
      }
      std::sort(samplingTime[2].begin(), samplingTime[2].end());
    }
  }

  //CR凸包を計算
  std::vector<cv::Point2f> cpList;
  for (int i=0; i<samplingTime[2].size(); i++) {
    double cpMaxX = fcp(samplingTime[2][i], gaitParam.omega, cp[0], zmpMin[0]);
    double cpMinX = fcp(samplingTime[2][i], gaitParam.omega, cp[0], zmpMax[0]);
    double cpMaxY = fcp(samplingTime[2][i], gaitParam.omega, cp[1], zmpMin[1]);
    double cpMinY = fcp(samplingTime[2][i], gaitParam.omega, cp[1], zmpMax[1]);

    double swMaxX = fsw(samplingTime[2][i], tmin, gaitParam.maxSwingVel[0], swingPos[0]);
    double swMinX = fsw(samplingTime[2][i], tmin, -gaitParam.maxSwingVel[0], swingPos[0]);
    double swMaxY = fsw(samplingTime[2][i], tmin, gaitParam.maxSwingVel[1], swingPos[1]);
    double swMinY = fsw(samplingTime[2][i], tmin, -gaitParam.maxSwingVel[1], swingPos[1]);

    //if (std::max(cpMinX, swMinX) <= std::min(cpMaxX, swMaxX) && std::max(cpMinY, swMinY) <= std::min(swMaxY, swMaxY)) {
    //  cpList.emplace_back(cv::Point2f(std::max(cpMinX, swMinX), std::max(cpMinY, swMinY)));
    //  cpList.emplace_back(cv::Point2f(std::max(cpMinX, swMinX), std::min(cpMaxY, swMaxY)));
    //  cpList.emplace_back(cv::Point2f(std::min(cpMaxX, swMaxX), std::max(cpMinY, swMinY)));
    //  cpList.emplace_back(cv::Point2f(std::min(cpMaxX, swMaxX), std::min(cpMaxY, swMaxY)));
    //} else {
    //  //SWING_PHASE以外のときのみ起きうる
    //  //std::cout << "okasiiyo " << cpMaxX << " " << cpMinX << " " << cpMaxY << " " << cpMinY << " " << swMaxX << " " << swMinX << " " << swMaxY << " " << swMinY << std::endl;
    //  reachableCaptureRegionHull = std::vector<cnoid::Vector3>();
    //  return 3;
    //}

    std::vector<cnoid::Vector3> cpRegion = std::vector<cnoid::Vector3>{cnoid::Vector3(cpMaxX, cpMaxY, 0), cnoid::Vector3(cpMinX, cpMaxY, 0), cnoid::Vector3(cpMinX, cpMinY, 0), cnoid::Vector3(cpMaxX, cpMinY, 0)};
    std::vector<cnoid::Vector3> swRegion = std::vector<cnoid::Vector3>{cnoid::Vector3(swMaxX, swMaxY, 0), cnoid::Vector3(swMinX, swMaxY, 0), cnoid::Vector3(swMinX, swMinY, 0), cnoid::Vector3(swMaxX, swMinY, 0)};
    std::vector<cnoid::Vector3> intersectRegion = mathutil::calcIntersectConvexHull(cpRegion, swRegion);
    if (intersectRegion.size() == 0) {
      std::vector<cnoid::Vector3> tmp;
      intersectRegion = std::vector<cnoid::Vector3>();
      mathutil::calcNearestPointOfTwoHull(swRegion, cpRegion, intersectRegion, tmp);
    }
    for (int j=0; j<intersectRegion.size(); j++) {
      cpList.emplace_back(cv::Point2f(intersectRegion[j][0], intersectRegion[j][1]));
    }
  }
  std::vector<cv::Point2f> hull;
  cv::convexHull(cpList, hull);

  reachableCaptureRegionHull.resize(hull.size());
  for (int i=0; i<hull.size(); i++) {
    reachableCaptureRegionHull[i] = supportPoseHorizontal * cnoid::Vector3(hull[i].x, hull[i].y, 0); //座標系戻す
  }

  return crErrorNum;
}

//ニュートン法によりminmaxtimeを計算
double FootStepGenerator::calcCRMinMaxTime(double& minTime, double& maxTime, double delta, double omega, double cp, double zmp, double tmin, double vmax, double sw) const{
  double tmpTime;
  if (cp < zmp) { //fcpが増加していくようにする
    cp = -cp;
    zmp = -zmp;
    sw = -sw;
  }
  if (sw > fcp(tmin, omega, cp, zmp)) { //t=tminですでにswのほうが前にある場合
    tmpTime = maxTime;
    do{
      double dx = dfcp(tmpTime, omega, cp, zmp) - dfsw(tmpTime, -vmax);
      tmpTime = tmpTime - (fcp(tmpTime, omega, cp, zmp) - fsw(tmpTime, tmin, -vmax, sw))/dx;
    } while (!(fcp(tmpTime, omega, cp, zmp) - fsw(tmpTime, tmin, -vmax, sw) > 0 && fcp(tmpTime - delta, omega, cp, zmp) - fsw(tmpTime - delta, tmin, -vmax, sw) < 0));
    minTime = tmpTime;

    tmpTime = maxTime;
    do{
      double dx = dfcp(tmpTime, omega, cp, zmp) - dfsw(tmpTime, vmax);
      tmpTime = tmpTime - (fcp(tmpTime, omega, cp, zmp) - fsw(tmpTime, tmin, vmax, sw))/dx;
    } while (!(fcp(tmpTime, omega, cp, zmp) - fsw(tmpTime, tmin, vmax, sw) > 0 && fcp(tmpTime - delta, omega, cp, zmp) - fsw(tmpTime - delta, tmin, vmax, sw) < 0));
    maxTime = tmpTime - delta;
  } else {
    double extTime = calc_extTime(omega, cp, zmp, vmax);
    if (fcp(extTime, omega, cp, zmp) - fsw(extTime, tmin, vmax, sw) < 0 && extTime > tmin) {
      tmpTime = tmin;
      do{
        double dx = dfcp(tmpTime, omega, cp, zmp) - dfsw(tmpTime, vmax);
        tmpTime = tmpTime - (fcp(tmpTime, omega, cp, zmp) - fsw(tmpTime, tmin, vmax, sw))/dx;
      } while (!(fcp(tmpTime, omega, cp, zmp) - fsw(tmpTime, tmin, vmax, sw) > 0 && fcp(tmpTime + delta, omega, cp, zmp) - fsw(tmpTime + delta, tmin, vmax, sw) < 0)); //ここだけはf(tmin)>0が確定しているので本当はdo_while文でなくても良い
      minTime = tmpTime + delta;

      tmpTime = std::max(extTime + 1.0, maxTime);
      do{
        double dx = dfcp(tmpTime, omega, cp, zmp) - dfsw(tmpTime, vmax);
        tmpTime = tmpTime - (fcp(tmpTime, omega, cp, zmp) - fsw(tmpTime, tmin, vmax, sw))/dx;
      } while (!(fcp(tmpTime, omega, cp, zmp) - fsw(tmpTime, tmin, vmax, sw) > 0 && fcp(tmpTime - delta, omega, cp, zmp) - fsw(tmpTime - delta, tmin, vmax, sw) < 0));
      maxTime = tmpTime - delta;
    } else { //reachableCRが存在しない
      minTime = maxTime = std::max(tmin, extTime);
      return fcp(extTime, omega, cp, zmp) - fsw(extTime, tmin, vmax, sw);
    }
  }
  return 0;
}

//time変更
double FootStepGenerator::modifyTime(double& retTime, bool& phaseChangeFlag, const double& dstTime, const cnoid::Vector3& _dstPos, const cnoid::Vector3& actDCM, const GaitParam& gaitParam, GaitParam::DebugData& debugData) const {
  //遊脚・支持脚
  int swingLeg = gaitParam.footstepNodesList[0].isSupportPhase[RLEG] ? LLEG : RLEG;
  int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;
  cnoid::Position supportPose = gaitParam.genCoords[supportLeg].value();
  cnoid::Position supportPoseHorizontal = mathutil::orientCoordToAxis(supportPose, cnoid::Vector3::UnitZ());

  // 現在の遊脚軌道目標(これを現在の遊脚位置と考える)
  cnoid::Position nowCoords;
  cnoid::Vector6 nowVel;
  gaitParam.genCoords[swingLeg].getGoal(nowCoords, nowVel);
  cnoid::Vector3 nowPos = nowCoords.translation();

  double swingZMinTime; //足の上げ下げのための最小時間
  double liftTime = 0;
  double downTime = 0;
  double targetHeight = std::max(gaitParam.srcCoords[swingLeg].translation()[2] + gaitParam.footstepNodesList[0].stepHeight[swingLeg][0], _dstPos[2] + gaitParam.footstepNodesList[0].stepHeight[swingLeg][1]);
  double airHeight = targetHeight - gaitParam.footstepNodesList[0].stepHeight[swingLeg][1] * 0.7;
  double downHeight = targetHeight -  gaitParam.footstepNodesList[0].stepHeight[swingLeg][1] * 0.4;//LegCoordsで設定するdownHeightより小さくする（そうしないとずっとswingBetterにあたって死ぬ）
  bool stairLift = false;
  {
    if (airHeight > nowPos[2]) {
      swingZMinTime = (2*airHeight - nowPos[2] - (_dstPos[2] + 0.01)) / gaitParam.maxSwingVel[2];
      stairLift = true;
    } else {
      swingZMinTime = std::max(0.0, (double)(nowPos[2] - (_dstPos[2] + 0.01))) / gaitParam.maxSwingVel[2];
      stairLift = false;
    }
    if (airHeight > nowPos[2] && (_dstPos - nowPos).norm() > 0.01) { //真下に下げるときはairHeight以下でもliftTime=0
      liftTime = (airHeight - nowPos[2]) / gaitParam.maxSwingVel[2];
    }
    if (gaitParam.swingState[swingLeg] == GaitParam::SWING_PHASE) {
      downTime = std::max(0.0, (downHeight - _dstPos[2]) / gaitParam.maxSwingVel[2]);
    }
  }
  double swingZMinRemainTime = std::max(gaitParam.delayTimeOffset, swingZMinTime + gaitParam.delayTimeOffset);

  //supportPoseHorizontal座標系に直す 以下Z成分は意味がなくなる
  cnoid::Vector3 cp = supportPoseHorizontal.inverse() * actDCM;
  nowPos = supportPoseHorizontal.inverse() * nowPos;
  cnoid::Vector3 dstPos = supportPoseHorizontal.inverse() * _dstPos;

  //safeLegHullが長方形であることを仮定(xy分離のため)
  cnoid::Vector3 zmpMin = cnoid::Vector3::Zero();
  cnoid::Vector3 zmpMax = cnoid::Vector3::Zero();
  for (int i=0; i<this->safeLegHull[supportLeg].size(); i++) {
    if (zmpMin[0]  > this->safeLegHull[supportLeg][i][0]) zmpMin[0] = this->safeLegHull[supportLeg][i][0];
    if (zmpMax[0]  < this->safeLegHull[supportLeg][i][0]) zmpMax[0] = this->safeLegHull[supportLeg][i][0];
    if (zmpMin[1]  > this->safeLegHull[supportLeg][i][1]) zmpMin[1] = this->safeLegHull[supportLeg][i][1];
    if (zmpMax[1]  < this->safeLegHull[supportLeg][i][1]) zmpMax[1] = this->safeLegHull[supportLeg][i][1];
  }

  double swingXMinRemainTime = liftTime + std::abs((dstPos - nowPos)[0]) / gaitParam.maxSwingVel[0] + gaitParam.delayTimeOffset + std::min(downTime, swingXYMarginTime);
  double swingYMinRemainTime = liftTime + std::abs((dstPos - nowPos)[1]) / gaitParam.maxSwingVel[1] + gaitParam.delayTimeOffset + std::min(downTime, swingXYMarginTime);
  double swingMinRemainTime = std::max({swingXMinRemainTime, swingYMinRemainTime, swingZMinRemainTime});

  //SWING_PHASEの時、通常の遊脚軌道を保つことができるRemainTime 無視しても良い
  //MAXの５割の速度でXY移動するものと仮定
  double swingXBetterRemainTime = liftTime + std::max(downTime, swingXYMarginTime) + std::abs((dstPos - nowPos)[0]) / (gaitParam.maxSwingVel[0]*0.5) + gaitParam.delayTimeOffset;
  double swingYBetterRemainTime = liftTime + std::max(downTime, swingXYMarginTime) + std::abs((dstPos - nowPos)[1]) / (gaitParam.maxSwingVel[1]*0.5) + gaitParam.delayTimeOffset;
  double swingXYBetterRemainTime = std::max(swingXBetterRemainTime, swingYBetterRemainTime);

  double constMaxTime = 10.0; //無限大を防ぐ

  std::vector<double> cpMinRemainTime = std::vector<double>{-1, -1, -1}; //X,Y,merge
  std::vector<double> cpMaxRemainTime = std::vector<double>{-1, -1, -1};
  for (int i = 0; i < 2; i++) { //X,Y
    double tmpa = (dstPos[i] - zmpMin[i]) / (cp[i] - zmpMin[i]);
    if (tmpa > 1) {
      tmpa = std::log(tmpa) / gaitParam.omega;
    } else {
      tmpa = 100*constMaxTime; //infinity
    }
    double tmpb = (dstPos[i] - zmpMax[i]) / (cp[i] - zmpMax[i]);
    if (tmpb >= 1) { //tmpb==1の時に0と無限になるようにするため
      tmpb = std::log(tmpb) / gaitParam.omega;
    } else {
      tmpb = 100*constMaxTime; //infinity
    }
    if (tmpa < tmpb) {
      cpMinRemainTime[i] = tmpa;
      cpMaxRemainTime[i] = tmpb;
    } else {
      cpMinRemainTime[i] = tmpb;
      cpMaxRemainTime[i] = tmpa;
    }
  }
  cpMinRemainTime[2] = std::max(cpMinRemainTime[0], cpMinRemainTime[1]) - gaitParam.footstepNodesList[1].remainTime; //cp目標は次の支持脚の終わりでも良い
  cpMaxRemainTime[2] = std::min(cpMaxRemainTime[0], cpMaxRemainTime[1]) - std::min(0.1, gaitParam.footstepNodesList[1].remainTime * 0.5); //遅着きケア

  double returnTime = dstTime;
  if (cpMaxRemainTime[2] < swingMinRemainTime || constMaxTime < cpMinRemainTime[2]) { //cp条件を無視
    returnTime = std::max(swingMinRemainTime, std::min(constMaxTime, dstTime));
  } else {
    double minTime = std::max(swingMinRemainTime, cpMinRemainTime[2]);
    double maxTime = std::min(constMaxTime, cpMaxRemainTime[2]);
    if (gaitParam.swingState[swingLeg] == GaitParam::SWING_PHASE && swingXYBetterRemainTime < maxTime) {
      minTime = std::max(minTime, swingXYBetterRemainTime);
    }

    returnTime = std::max(minTime, std::min(maxTime, dstTime));
  }

  //swingZMinRemainTimeにあたっている場合、SWING_PHASEからHEIGHT_FIX_SWINT_PHASEへ移行させる
  phaseChangeFlag = false;
  if (returnTime == swingZMinRemainTime && !stairLift) phaseChangeFlag = true;

  debugData.cpViewerLog[22] = swingXMinRemainTime;
  debugData.cpViewerLog[23] = swingYMinRemainTime;
  debugData.cpViewerLog[24] = swingZMinRemainTime;
  debugData.cpViewerLog[25] = swingXBetterRemainTime;
  debugData.cpViewerLog[26] = swingYBetterRemainTime;
  debugData.cpViewerLog[27] = swingXYBetterRemainTime;
  debugData.cpViewerLog[28] = cpMinRemainTime[2];
  debugData.cpViewerLog[29] = cpMaxRemainTime[2];
  debugData.cpViewerLog[30] = dstTime;
  debugData.cpViewerLog[31] = returnTime;
  debugData.cpViewerLog[32] = phaseChangeFlag;
  debugData.cpViewerLog[33] = downTime;
  debugData.cpViewerLog[34] = downHeight;
  debugData.cpViewerLog[35] = _dstPos[2];
  debugData.cpViewerLog[36] = gaitParam.srcCoords[swingLeg].translation()[2];

  retTime = returnTime;

  return 0;
}

// 早づきしたらremainTimeが0になるまで今の位置で止める.
//   - この機能が無いと少しでもロボットが傾いて早づきするとジャンプするような挙動になる.
//   - remainTimeが0になるまで待たないと、本来の着地位置に着地したときに、DCMが次の着地位置まで移動しきっていなくて、かつ、支持脚状態が急激に変化するので、ZMP入力が急激に変化してしまう.
//   - ただし、この方法だと、早づきするときは転びそうになっているときなのですぐに次の一歩を踏み出さないと転んでしまうので、待つぶん弱い
// remainTimeが0になっても地面についていなかったら、remainTimeを少しずつ延長し着地位置を下方に少しずつオフセットさせる
//   - remainTimeが0のときには本来の着地位置に行くようにしないと、着地タイミングがrefZmpよりも常に早すぎ・遅すぎになるので良くない
//   - ただし、この方法だと、遅づきしたときに着地時刻が遅くなるのでDCMが移動しずぎてしまっているので、転びやすい.
void FootStepGenerator::checkEarlyTouchDown(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const GaitParam& gaitParam, double dt) const{
  for(int i=0;i<NUM_LEGS;i++){
    actLegWrenchFilter[i].passFilter(gaitParam.actEEWrench[i], dt);
  }

  if(footstepNodesList.size() > 1){
    for(int leg = 0; leg<NUM_LEGS; leg++){
      if(!footstepNodesList[0].isSupportPhase[leg] && footstepNodesList[1].isSupportPhase[leg] && //現在swing期で次support期
         gaitParam.isLandingGainPhase[leg] && // DOWN_PHASE
         footstepNodesList[0].stopCurrentPosition[leg] == false){ // まだ地面についていない

        if(actLegWrenchFilter[leg].value()[2] > this->contactDetectionThreshold /*generate frame. ロボットが受ける力*/) {// 力センサの値が閾値以上
          //if(footstepNodesList[0].stopCurrentPosition[leg] == false) {
          //  if (gaitParam.elapsedTime > 0.1){
          //    footstepNodesList[0].dstCoords[leg].translation() = gaitParam.actEEPose[leg].translation();
          //    footstepNodesList[1].dstCoords[leg].translation() = gaitParam.actEEPose[leg].translation();
          //  }
          //}
          footstepNodesList[0].stopCurrentPosition[leg] = true;
        }else if(footstepNodesList[0].remainTime <= dt && // remainTimeが0になる
                 footstepNodesList[0].touchVel[leg] > 0.0 && // touchVelが0ならいつまでもつかないのでgoaloffsetを適用しない
                 footstepNodesList[0].goalOffset[leg] > this->goalOffset){ // まだgoalOffsetまで下ろしていない
          footstepNodesList[0].remainTime += dt;
          footstepNodesList[0].goalOffset[leg] = std::max(this->goalOffset, footstepNodesList[0].goalOffset[leg] - footstepNodesList[0].touchVel[leg] * dt);
        }
      }
    }

    //両足支持期延長
    if((footstepNodesList[0].isSupportPhase[RLEG] && footstepNodesList[0].isSupportPhase[LLEG]) && footstepNodesList[0].remainTime <= dt) { //現在両足支持期、remainTime=0
      std::vector<std::vector<cnoid::Vector3> > safeSingleLegVertices = std::vector<std::vector<cnoid::Vector3> >{std::vector<cnoid::Vector3>(), std::vector<cnoid::Vector3>()};
      std::vector<cnoid::Vector3> safeDoubleLegVertices;
      std::vector<std::vector<cnoid::Vector3> > safeSingleLegHull = std::vector<std::vector<cnoid::Vector3> >{std::vector<cnoid::Vector3>(), std::vector<cnoid::Vector3>()};
      std::vector<cnoid::Vector3> safeDoubleLegHull;
      for(int i=0; i<NUM_LEGS; i++) {//i=2を仮定
        for(int j=0; j<this->safeLegHull[i].size(); j++){
          safeSingleLegVertices[i].push_back(gaitParam.actEEPose[i] * this->safeLegHull[i][j]); // generate frame
          safeDoubleLegVertices.push_back(gaitParam.actEEPose[i] * this->safeLegHull[i][j]); // generate frame
        }
        safeSingleLegHull[i] = mathutil::calcConvexHull(safeSingleLegVertices[i]); // generate frame. Z成分はてきとう
      }
      safeDoubleLegHull = mathutil::calcConvexHull(safeDoubleLegVertices); // generate frame. Z成分はてきとう
      cnoid::Vector3 actDCM = gaitParam.actCog + gaitParam.actCogVel.value() / gaitParam.omega; // generate frame
      cnoid::Vector3 genDCM = gaitParam.genCog + gaitParam.genCogVel / gaitParam.omega;

      if(footstepNodesList[1].isSupportPhase[RLEG] && !footstepNodesList[1].isSupportPhase[LLEG] && //次が右足支持期
               gaitParam.elapsedTime <= 0.3 && //最長時間
               mathutil::isInsideHull(actDCM, safeDoubleLegHull) && // actDCMが両足支持凸包内
               !mathutil::isInsideHull(actDCM, safeSingleLegHull[RLEG])){ // actDCMが右足支持凸包外
        footstepNodesList[0].remainTime += dt;
      } else if(!footstepNodesList[1].isSupportPhase[RLEG] && footstepNodesList[1].isSupportPhase[LLEG] && //次が左足支持期
               gaitParam.elapsedTime <= 0.3 && //最長時間
               mathutil::isInsideHull(actDCM, safeDoubleLegHull) && // actDCMが両足支持凸包内
               !mathutil::isInsideHull(actDCM, safeSingleLegHull[LLEG])){ // actDCMが右足支持凸包外
        footstepNodesList[0].remainTime += dt;
      } else {
      }
    } else {
    }
  }
}

// emergengy step.
void FootStepGenerator::checkEmergencyStep(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const GaitParam& gaitParam) const{
  // 現在静止状態で、CapturePointがsafeLegHullの外にあるなら、footstepNodesListがemergencyStepNumのサイズになるまで歩くnodeが末尾に入る.
  if(!(footstepNodesList.size() == 1 && footstepNodesList[0].remainTime == 0)) return; // static 状態でないなら何もしない

  std::vector<cnoid::Vector3> supportVertices; // generate frame
  for(int i=0;i<NUM_LEGS;i++){
    if(footstepNodesList[0].isSupportPhase[i]){
      for(int j=0;j<this->safeLegHull[i].size();j++){
        supportVertices.push_back(gaitParam.actEEPose[i] * this->safeLegHull[i][j]); // generate frame
      }
    }
  }
  std::vector<cnoid::Vector3> supportHull = mathutil::calcConvexHull(supportVertices); // generate frame. Z成分はてきとう

  // dx = w ( x - z - l)
  cnoid::Vector3 actDCM = gaitParam.actCog + gaitParam.actCogVel.value() / gaitParam.omega; // generate frame
  cnoid::Vector3 actCMP = actDCM - gaitParam.l; // generate frame

  if(!mathutil::isInsideHull(actCMP, supportHull) || // supportHullに入っていない
     (actCMP.head<2>() - gaitParam.refZmpTraj[0].getStart().head<2>()).norm() > this->emergencyStepCpCheckMargin // 目標重心位置からの距離が閾値以上 (supportHullによるチェックだけだと両足支持の場合の左右方向の踏み出しがなかなか起こらず、左右方向の踏み出しは得意ではないため、踏み出しが起きたときには時既に遅しで転倒してしまう)
     ){
    cnoid::Vector3 dir = gaitParam.footMidCoords.value().linear().transpose() * (actCMP - gaitParam.footMidCoords.value().translation()); // footmidcoords frame
    dir[2] = 0.0;
    if(dir.norm() > 0.0) dir = dir.normalized();
    while(footstepNodesList.size() <= this->emergencyStepNum){
      this->calcDefaultNextStep(footstepNodesList, gaitParam, 1e-6 * dir, false); // 現在両足で支持している場合、dirの方向の足を最初にswingする. 急ぐのでstableStart=false
    }
    // refZmpを両足の中心へ戻す
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), footstepNodesList.back().endRefZmpState));
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), GaitParam::FootStepNodes::refZmpState_enum::MIDDLE)); // 末尾の両足支持期を延長. これがないと重心が目標位置に収束する前に返ってしまい, emergencyStepが無限に誘発する footGudedBalanceTimeを0.4程度に小さくすると収束が速くなるのでこの処理が不要になるのだが、今度はZ方向に振動しやすい
  }
}

// Stable Go Stop
void FootStepGenerator::checkStableGoStop(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const GaitParam& gaitParam) const{
  // 着地位置修正を行ったなら、footstepNodesListがemergencyStepNumのサイズになるまで歩くnodeが末尾に入る.
  for(int i=0;i<NUM_LEGS; i++){
    if((footstepNodesList[0].dstCoords[i].translation().head<2>() - gaitParam.dstCoordsOrg[i].translation().head<2>()).norm() > 0.01){ // 1cm以上着地位置修正を行ったなら
      while(footstepNodesList.size() >= 2 &&
            (footstepNodesList[footstepNodesList.size()-2].isSupportPhase[RLEG] && footstepNodesList[footstepNodesList.size()-2].isSupportPhase[LLEG]) &&
            (footstepNodesList[footstepNodesList.size()-1].isSupportPhase[RLEG] && footstepNodesList[footstepNodesList.size()-1].isSupportPhase[LLEG])){
        footstepNodesList.pop_back(); // 末尾に両足支持期が連続している場合、最初のものを除いて削除
      }
      while(footstepNodesList.size() < this->emergencyStepNum){
        // 歩を追加
        this->calcDefaultNextStep(footstepNodesList, gaitParam);
      }
      // refZmpを両足の中心へ戻す
      footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), footstepNodesList.back().endRefZmpState));
      footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
      footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), GaitParam::FootStepNodes::refZmpState_enum::MIDDLE)); // 末尾の両足支持期を延長. これがないと重心が目標位置に収束する前に返ってしまい, emergencyStepが無限に誘発する footGudedBalanceTimeを0.4程度に小さくすると収束が速くなるのでこの処理が不要になるのだが、今度はZ方向に振動しやすい

      break;
    }
  }
}

std::vector<cnoid::Vector3> FootStepGenerator::calcRealStrideLimitationHull(const int& swingLeg, const double& theta, const std::vector<std::vector<cnoid::Vector3> >& legHull, const std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >& defaultTranslatePos, const std::vector<std::vector<cnoid::Vector3> >& strideLimitationHull) const{
  std::vector<cnoid::Vector3> realStrideLimitationHull = strideLimitationHull[swingLeg]; // 支持脚(水平)座標系. strideLimitationHullの要素数は1以上あることが保証されているという仮定

  int supportLeg = swingLeg == RLEG ? LLEG : RLEG;
  cnoid::Matrix3 swingR = cnoid::AngleAxis(theta, cnoid::Vector3::UnitZ()).toRotationMatrix();
  {
    // legCollisionを考慮
    cnoid::Vector3 supportLegToSwingLeg = defaultTranslatePos[swingLeg].value() - defaultTranslatePos[supportLeg].value(); // 支持脚(水平)座標系.
    supportLegToSwingLeg[2] = 0.0;
    if(supportLegToSwingLeg.norm() > 0.0){
      cnoid::Vector3 supportLegToSwingLegDir = supportLegToSwingLeg.normalized();
      std::vector<cnoid::Vector3> tmp;
      const std::vector<cnoid::Vector3>& supportLegHull = legHull[supportLeg];
      double supportLegHullSize = mathutil::findExtreams(supportLegHull, supportLegToSwingLegDir, tmp);
      std::vector<cnoid::Vector3> swingLegHull;
      for(int i=0;i<legHull[swingLeg].size();i++) swingLegHull.push_back(swingR * legHull[swingLeg][i]);
      double swingLegHullSize = mathutil::findExtreams(swingLegHull, - supportLegToSwingLegDir, tmp);
      double minDist = supportLegHullSize + this->legCollisionMargin + swingLegHullSize;
      std::vector<cnoid::Vector3> minDistHull; // supportLegToSwingLeg方向にminDistを満たすHull
      {
        cnoid::Position p = cnoid::Position::Identity();
        p.translation() = minDist * supportLegToSwingLegDir;
        p.linear() = (cnoid::Matrix3() << supportLegToSwingLegDir[0], -supportLegToSwingLegDir[1], 0.0,
                                          supportLegToSwingLegDir[1], supportLegToSwingLegDir[0] , 0.0,
                                          0.0,                        0.0,                         1.0).finished();
        minDistHull.push_back(p * cnoid::Vector3(0.0, -1e10, 0.0));
        minDistHull.push_back(p * cnoid::Vector3(1e10, -1e10, 0.0));
        minDistHull.push_back(p * cnoid::Vector3(1e10, 1e10, 0.0));
        minDistHull.push_back(p * cnoid::Vector3(0.0, 1e10, 0.0));
      }

      std::vector<cnoid::Vector3> nextRealStrideLimitationHull = mathutil::calcIntersectConvexHull(realStrideLimitationHull, minDistHull);
      if(nextRealStrideLimitationHull.size() > 0) realStrideLimitationHull = nextRealStrideLimitationHull;
    }
  }

  {
    // swingLegから見てもsupportLegから見てもStrideLimitationHullの中にあることを確認 (swing中の干渉を防ぐ)
    const std::vector<cnoid::Vector3>& strideLimitationHullFromSupport = realStrideLimitationHull;
    std::vector<cnoid::Vector3> strideLimitationHullFromSwing(realStrideLimitationHull.size());
    for(int i=0;i<realStrideLimitationHull.size();i++){
      strideLimitationHullFromSwing[i] = swingR * realStrideLimitationHull[i];
    }
    std::vector<cnoid::Vector3> nextRealStrideLimitationHull = mathutil::calcIntersectConvexHull(strideLimitationHullFromSupport, strideLimitationHullFromSwing);
    if(nextRealStrideLimitationHull.size() > 0) realStrideLimitationHull = nextRealStrideLimitationHull;
  }

  return realStrideLimitationHull;
}
