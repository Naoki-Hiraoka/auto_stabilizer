#include "FootStepGenerator.h"
#include "MathUtil.h"
#include <cnoid/EigenUtil>

bool FootStepGenerator::initFootStepNodesList(const GaitParam& gaitParam,
                                              std::vector<GaitParam::FootStepNodes>& o_footstepNodesList, std::vector<cnoid::Position>& o_srcCoords, std::vector<cnoid::Position>& o_dstCoordsOrg, double& o_remainTimeOrg, std::vector<bool>& o_prevSupportPhase) const{
  // footStepNodesListを初期化する
  std::vector<GaitParam::FootStepNodes> footstepNodesList(1);
  cnoid::Position rlegCoords = gaitParam.genRobot->link(gaitParam.eeParentLink[RLEG])->T()*gaitParam.eeLocalT[RLEG];
  cnoid::Position llegCoords = gaitParam.genRobot->link(gaitParam.eeParentLink[LLEG])->T()*gaitParam.eeLocalT[LLEG];
  footstepNodesList[0].dstCoords = {rlegCoords, llegCoords};
  footstepNodesList[0].isSupportPhase = {(gaitParam.isManualControlMode[RLEG].getGoal() == 0.0), (gaitParam.isManualControlMode[LLEG].getGoal() == 0.0)};
  footstepNodesList[0].remainTime = 0.0;
  std::vector<cnoid::Position> srcCoords = footstepNodesList[0].dstCoords;
  std::vector<cnoid::Position> dstCoordsOrg = footstepNodesList[0].dstCoords;
  double remainTimeOrg = footstepNodesList[0].remainTime;

  std::vector<bool> prevSupportPhase(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++){
    prevSupportPhase[i] = footstepNodesList[0].isSupportPhase[i];
  }

  o_prevSupportPhase = prevSupportPhase;
  o_footstepNodesList = footstepNodesList;
  o_srcCoords = srcCoords;
  o_dstCoordsOrg = dstCoordsOrg;
  o_remainTimeOrg = remainTimeOrg;

  return true;
}

bool FootStepGenerator::setFootSteps(const GaitParam& gaitParam, const std::vector<StepNode>& footsteps,
                                     std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const{
  if(footsteps.size() <= 1) { // 何もしない
    o_footstepNodesList = gaitParam.footstepNodesList;
    return true;
  }

  if(!gaitParam.isStatic()){ // 静止中でないと無効
    o_footstepNodesList = gaitParam.footstepNodesList;
    return false;
  }

  std::vector<GaitParam::FootStepNodes> footstepNodesList;
  footstepNodesList.push_back(gaitParam.footstepNodesList[0]);

  if(footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){ // 両足支持期を延長
    // 現在の時刻から突然refZmpTrajが変化すると、大きなZMP入力変化が必要になる. いまの位置でrefZmpTrajをthis->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio)の間とめて、次にthis->defaultStepTime * this->defaultDoubleSupportRatioの間で次の支持脚側に動かす
    footstepNodesList.back().remainTime = this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio);
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio));
  }


  // footstepsの0番目の要素は、実際には歩かず、基準座標としてのみ使われる. footstepNodesList[0].dstCoordsのZ軸を鉛直に直した座標系と、footstepsの0番目の要素のZ軸を鉛直に直した座標系が一致するように座標変換する.
  if(!footstepNodesList.back().isSupportPhase[RLEG] && footsteps[0].l_r == RLEG){ // RLEGを下ろす必要がある
    this->calcDefaultNextStep(footstepNodesList, gaitParam.defaultTranslatePos);
  }else if(!footstepNodesList.back().isSupportPhase[LLEG] && footsteps[0].l_r == LLEG){ // LLEGを下ろす必要がある
    this->calcDefaultNextStep(footstepNodesList, gaitParam.defaultTranslatePos);
  }
  cnoid::Position trans;
  {
    cnoid::Position genCoords = mathutil::orientCoordToAxis(footstepNodesList.back().dstCoords[footsteps[0].l_r], cnoid::Vector3::UnitZ());
    cnoid::Position refCoords = mathutil::orientCoordToAxis(footsteps[0].coords, cnoid::Vector3::UnitZ());
    trans = genCoords * refCoords.inverse();
  }

  for(int i=1;i<footsteps.size();i++){
    if(!footstepNodesList.back().isSupportPhase[RLEG] && footsteps[i].l_r == LLEG){ // RLEGを下ろす必要がある
      this->calcDefaultNextStep(footstepNodesList, gaitParam.defaultTranslatePos);
    }else if(!footstepNodesList.back().isSupportPhase[LLEG] && footsteps[i].l_r == RLEG){ // LLEGを下ろす必要がある
      this->calcDefaultNextStep(footstepNodesList, gaitParam.defaultTranslatePos);
    }
    GaitParam::FootStepNodes fs;
    int swingLeg = footsteps[i].l_r;
    int supportLeg = swingLeg == RLEG ? LLEG: RLEG;
    fs.dstCoords[supportLeg] = footstepNodesList.back().dstCoords[supportLeg];
    fs.dstCoords[swingLeg] = trans * footsteps[i].coords;
    fs.isSupportPhase[supportLeg] = true;
    fs.isSupportPhase[swingLeg] = false;
    fs.remainTime = footsteps[i].stepTime * (1 - this->defaultDoubleSupportRatio);
    double stepHeight = std::max(0.0, footsteps[i].stepHeight);
    double beforeHeight = footstepNodesList.back().isSupportPhase[swingLeg] ? stepHeight : 0.0;
    double afterHeight = footsteps[i].swingEnd ? 0.0 : stepHeight;
    fs.stepHeight[swingLeg] = {beforeHeight,afterHeight};
    footstepNodesList.push_back(fs);

    if(!footsteps[i].swingEnd) footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), footsteps[i].stepTime * this->defaultDoubleSupportRatio));
  }

  // if(footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){
  //   footstepNodesList.back().remainTime = std::max(footstepNodesList.back().remainTime, this->defaultStepTime); // 末尾の両足支持期を延長. これがないと重心が目標位置に収束する前に返ってしまい, emergencyStepが無限に誘発する? -> footGudedBalanceTimeを0.4程度に小さくするとよい
  // }

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

  if(footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){ // 両足支持期を延長
    // 現在の時刻から突然refZmpTrajが変化すると、大きなZMP入力変化が必要になる. いまの位置でrefZmpTrajをthis->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio)の間とめて、次にthis->defaultStepTime * this->defaultDoubleSupportRatioの間で次の支持脚側に動かす
    footstepNodesList.back().remainTime = this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio);
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio));
  }

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
    this->calcDefaultNextStep(footstepNodesList, gaitParam.defaultTranslatePos, diff);
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
    this->calcDefaultNextStep(footstepNodesList, gaitParam.defaultTranslatePos);
  }

  // footstepNodesList.back().remainTime = std::max(footstepNodesList.back().remainTime, this->defaultStepTime); // 末尾の両足支持期を延長. これがないと重心が目標位置に収束する前に返ってしまい, emergencyStepが無限に誘発する -> footGudedBalanceTimeを0.4程度に小さくするとよい

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

  if(footstepNodesList.size()>2) footstepNodesList.erase(footstepNodesList.begin()+2,footstepNodesList.end());

  // 両脚が横に並ぶ位置に2歩歩く.
  for(int i=0;i<2;i++){
    this->calcDefaultNextStep(footstepNodesList, gaitParam.defaultTranslatePos);
  }

  // footstepNodesList.back().remainTime = std::max(footstepNodesList.back().remainTime, this->defaultStepTime); // 末尾の両足支持期を延長. これがないと重心が目標位置に収束する前に返ってしまい, emergencyStepが無限に誘発する -> footGudedBalanceTimeを0.4程度に小さくするとよい

  o_footstepNodesList = footstepNodesList;
  return true;

}

// FootStepNodesListをdtすすめる
bool FootStepGenerator::procFootStepNodesList(const GaitParam& gaitParam, const double& dt, bool useActState,
                                              std::vector<GaitParam::FootStepNodes>& o_footstepNodesList, std::vector<cnoid::Position>& o_srcCoords, std::vector<cnoid::Position>& o_dstCoordsOrg, double& o_remainTimeOrg, std::vector<GaitParam::SwingState_enum>& o_swingState, double& o_elapsedTime, std::vector<bool>& o_prevSupportPhase) const{
  std::vector<GaitParam::FootStepNodes> footstepNodesList = gaitParam.footstepNodesList;
  std::vector<bool> prevSupportPhase = gaitParam.prevSupportPhase;
  double elapsedTime = gaitParam.elapsedTime;
  std::vector<cnoid::Position> srcCoords = gaitParam.srcCoords;
  std::vector<cnoid::Position> dstCoordsOrg = gaitParam.dstCoordsOrg;
  double remainTimeOrg = gaitParam.remainTimeOrg;
  std::vector<GaitParam::SwingState_enum> swingState = gaitParam.swingState;

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
                                  footstepNodesList, srcCoords, dstCoordsOrg, remainTimeOrg, swingState, elapsedTime);
  }

  o_footstepNodesList = footstepNodesList;
  o_prevSupportPhase = prevSupportPhase;
  o_elapsedTime = elapsedTime;
  o_srcCoords = srcCoords;
  o_dstCoordsOrg = dstCoordsOrg;
  o_remainTimeOrg = remainTimeOrg;
  o_swingState = swingState;

  return true;
}

bool FootStepGenerator::calcFootSteps(const GaitParam& gaitParam, const double& dt, bool useActState,
                                      std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const{
  std::vector<GaitParam::FootStepNodes> footstepNodesList = gaitParam.footstepNodesList;

  // goVelocityModeなら、進行方向に向けてfootStepNodesList[1] ~ footStepNodesList[goVelocityStepNum]の要素を機械的に計算してどんどん末尾appendしていく. cmdVelに応じてきまる
  if(this->isGoVelocityMode){
    if(gaitParam.prevSupportPhase[RLEG] != gaitParam.footstepNodesList[0].isSupportPhase[RLEG] ||
       gaitParam.prevSupportPhase[LLEG] != gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){ // footstepの切り替わりのタイミング. this->updateGoVelocityStepsを毎周期呼ぶと、数値誤差でだんだん変にずれてくるので.
      this->updateGoVelocitySteps(footstepNodesList, gaitParam.defaultTranslatePos, gaitParam.cmdVel);
    }
    while(footstepNodesList.size() <= this->goVelocityStepNum){
      if(footstepNodesList.size() == 1 &&
         footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){ // 両足支持期を延長
        // 現在の時刻から突然refZmpTrajが変化すると、大きなZMP入力変化が必要になる. いまの位置でrefZmpTrajをthis->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio)の間とめて、次にthis->defaultStepTime * this->defaultDoubleSupportRatioの間で次の支持脚側に動かす
        footstepNodesList.back().remainTime = this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio);
        footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio));
      }

      this->calcDefaultNextStep(footstepNodesList, gaitParam.defaultTranslatePos, gaitParam.cmdVel * this->defaultStepTime);
    }
  }

  if(useActState){
    if(this->isModifyFootSteps && this->isEmergencyStepMode){
      this->checkEmergencyStep(footstepNodesList, gaitParam);
    }

    if(this->isModifyFootSteps){
      this->modifyFootSteps(footstepNodesList, gaitParam);
    }
  }

  o_footstepNodesList = footstepNodesList;

  return true;
}

bool FootStepGenerator::goNextFootStepNodesList(const GaitParam& gaitParam, double dt,
                                                std::vector<GaitParam::FootStepNodes>& footstepNodesList, std::vector<cnoid::Position>& srcCoords, std::vector<cnoid::Position>& dstCoordsOrg, double& remainTimeOrg, std::vector<GaitParam::SwingState_enum>& swingState, double& elapsedTime) const{
  // 今のgenCoordsとdstCoordsが異なるなら、将来のstepの位置姿勢をそれに合わせてずらす. (特にgoalOffsetがある場合にはこの処理が必要)
  //   footstepNodesList[1:]で一回でも遊脚になった以降は、位置とYawをずらす. footstepNodesList[1:]で最初に遊脚になる脚があるならその反対の脚の偏差にあわせてずらす. そうでないなら両脚の中心(+-defaultTranslatePos)の偏差にあわせてずらす
  //   footstepNodesList[1]が支持脚なら、遊脚になるまで、位置姿勢をその足の今の偏差にあわせてずらす.
  for(int i=1;i<footstepNodesList.size();i++){
    if(footstepNodesList[i].isSupportPhase[RLEG] && !footstepNodesList[i].isSupportPhase[LLEG]){ // LLEGが次に最初に遊脚になる
      cnoid::Position origin = mathutil::orientCoordToAxis(footstepNodesList[i-1].dstCoords[RLEG], cnoid::Vector3::UnitZ()); // generate frame. 一つ前のRLEGの位置を基準にずらす
      cnoid::Position offsetedPos = mathutil::orientCoordToAxis(gaitParam.genCoords[RLEG].value() * footstepNodesList[0].dstCoords[RLEG].inverse() * footstepNodesList[i-1].dstCoords[RLEG], cnoid::Vector3::UnitZ()); // generate frame. 一つ前のRLEGはこの位置にずれている
      cnoid::Position transform = offsetedPos * origin.inverse(); // generate frame
      this->transformFutureSteps(footstepNodesList, i, transform); // footstepNodesList[1:]で一回でも遊脚になった以降の脚の、位置とYawをずらす
      break;
    }else if(footstepNodesList[i].isSupportPhase[LLEG] && !footstepNodesList[i].isSupportPhase[RLEG]){ // RLEGが次に最初に遊脚になる
      cnoid::Position origin = mathutil::orientCoordToAxis(footstepNodesList[i-1].dstCoords[LLEG], cnoid::Vector3::UnitZ()); // generate frame. 一つ前のLLEGの位置を基準にずらす
      cnoid::Position offsetedPos = mathutil::orientCoordToAxis(gaitParam.genCoords[LLEG].value() * footstepNodesList[0].dstCoords[LLEG].inverse() * footstepNodesList[i-1].dstCoords[LLEG], cnoid::Vector3::UnitZ()); // generate frame. 一つ前のLLEGはこの位置にずれている
      cnoid::Position transform = offsetedPos * origin.inverse(); // generate frame
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
      this->transformFutureSteps(footstepNodesList, i, transform); // footstepNodesList[1:]で一回でも遊脚になった以降の脚の、位置とYawをずらす
      break;
    }
  }
  for(int i=0;i<NUM_LEGS;i++){
    if(footstepNodesList[1].isSupportPhase[i]){
      cnoid::Position transform = gaitParam.genCoords[i].value() * footstepNodesList[0].dstCoords[i].inverse(); // generate frame
      this->transformCurrentSupportSteps(i, footstepNodesList, 1, transform); // 遊脚になるまで、位置姿勢をその足の今の偏差にあわせてずらす.
    }
  }

  // footstepNodesListをpop front
  footstepNodesList.erase(footstepNodesList.begin()); // vectorではなくlistにするべきかもしれないが、要素数がそんなに大きくないのでよしとする
  for(int i=0;i<NUM_LEGS;i++) {
    srcCoords[i] = gaitParam.genCoords[i].value();
    dstCoordsOrg[i] = footstepNodesList[0].dstCoords[i];
    swingState[i] = GaitParam::LIFT_PHASE;
  }
  remainTimeOrg = footstepNodesList[0].remainTime;
  elapsedTime = 0.0;

  return true;
}

void FootStepGenerator::updateGoVelocitySteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >& defaultTranslatePos, const cnoid::Vector3& cmdVel) const{
  for(int i=1;i<footstepNodesList.size()-1;i++){
    if(((footstepNodesList[i].isSupportPhase[RLEG] && !footstepNodesList[i].isSupportPhase[LLEG]) || (!footstepNodesList[i].isSupportPhase[RLEG] && footstepNodesList[i].isSupportPhase[LLEG])) && // 今が片脚支持期
       (footstepNodesList[i+1].isSupportPhase[RLEG] && footstepNodesList[i+1].isSupportPhase[LLEG])){ // 次が両足支持期
      int swingLeg = footstepNodesList[i].isSupportPhase[RLEG] ? LLEG : RLEG;
      int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;
      cnoid::Vector3 offset = cmdVel * (footstepNodesList[i].remainTime + footstepNodesList[i+1].remainTime);
      cnoid::Position transform = cnoid::Position::Identity(); // supportLeg相対(Z軸は鉛直)での次のswingLegの位置
      transform.linear() = cnoid::Matrix3(Eigen::AngleAxisd(mathutil::clamp(offset[2],this->defaultStrideLimitationTheta), cnoid::Vector3::UnitZ()));
      transform.translation() = - defaultTranslatePos[supportLeg].value() + cnoid::Vector3(offset[0], offset[1], 0.0) + transform.linear() * defaultTranslatePos[swingLeg].value();
      transform.translation() = mathutil::calcNearestPointOfHull(transform.translation(), this->defaultStrideLimitationHull[swingLeg]);
      cnoid::Position prevOrigin = mathutil::orientCoordToAxis(footstepNodesList[i-1].dstCoords[supportLeg], cnoid::Vector3::UnitZ());
      cnoid::Position dstCoords = prevOrigin * transform;

      cnoid::Position origin = mathutil::orientCoordToAxis(footstepNodesList[i].dstCoords[swingLeg], cnoid::Vector3::UnitZ());
      cnoid::Position trans = dstCoords * origin.inverse();
      this->transformFutureSteps(footstepNodesList, i, trans);
    }
  }
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

void FootStepGenerator::calcDefaultNextStep(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >& defaultTranslatePos, const cnoid::Vector3& offset /*leg frame*/) const{
  if(footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){
    footstepNodesList.back().remainTime = std::max(footstepNodesList.back().remainTime, this->defaultStepTime * this->defaultDoubleSupportRatio); // 両足支持期を延長
    if(footstepNodesList.size() == 1 ||
       (footstepNodesList[footstepNodesList.size()-2].isSupportPhase[RLEG] && footstepNodesList[footstepNodesList.size()-2].isSupportPhase[LLEG]) ||
       (!footstepNodesList[footstepNodesList.size()-2].isSupportPhase[RLEG] && !footstepNodesList[footstepNodesList.size()-2].isSupportPhase[LLEG]) ){
      // offsetを、両足の中間からの距離と解釈する(これ以外のケースでは支持脚からの距離と解釈する)
      cnoid::Position rleg = footstepNodesList[0].dstCoords[RLEG];
      rleg.translation() -= rleg.linear() * defaultTranslatePos[RLEG].value();
      cnoid::Position lleg = footstepNodesList[0].dstCoords[LLEG];
      lleg.translation() -= lleg.linear() * defaultTranslatePos[LLEG].value();
      cnoid::Position midCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{rleg, lleg}, std::vector<double>{1.0, 1.0});
      rleg = mathutil::orientCoordToAxis(rleg, cnoid::Vector3::UnitZ());
      lleg = mathutil::orientCoordToAxis(lleg, cnoid::Vector3::UnitZ());
      midCoords = mathutil::orientCoordToAxis(midCoords, cnoid::Vector3::UnitZ());
      // どっちをswingしてもいいので、進行方向に近いLegをswingする
      cnoid::Vector2 rlegTolleg = (defaultTranslatePos[LLEG].value() - defaultTranslatePos[RLEG].value()).head<2>(); // leg frame
      if(rlegTolleg.dot(offset.head<2>()) > 0) {
        cnoid::Position rlegTomidCoords = rleg.inverse() * midCoords;
        cnoid::Vector3 offset_rlegLocal;
        offset_rlegLocal.head<2>() = rlegTomidCoords.translation().head<2>() + (rlegTomidCoords.linear() * cnoid::Vector3(offset[0],offset[1],0.0)).head<2>();
        offset_rlegLocal[2] = cnoid::rpyFromRot(rlegTomidCoords.linear())[2] + offset[2];
        footstepNodesList.push_back(calcDefaultSwingStep(LLEG, footstepNodesList.back(), defaultTranslatePos, offset_rlegLocal)); // LLEGをswingする
        footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio));
      }else{
        cnoid::Position llegTomidCoords = lleg.inverse() * midCoords;
        cnoid::Vector3 offset_llegLocal;
        offset_llegLocal.head<2>() = llegTomidCoords.translation().head<2>() + (llegTomidCoords.linear() * cnoid::Vector3(offset[0],offset[1],0.0)).head<2>();
        offset_llegLocal[2] = cnoid::rpyFromRot(llegTomidCoords.linear())[2] + offset[2];
        footstepNodesList.push_back(calcDefaultSwingStep(RLEG, footstepNodesList.back(), defaultTranslatePos, offset_llegLocal)); // RLEGをswingする
        footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio));
      }
    }else if(footstepNodesList[footstepNodesList.size()-2].isSupportPhase[RLEG]){ // 前回LLEGをswingした
        footstepNodesList.push_back(calcDefaultSwingStep(RLEG, footstepNodesList.back(), defaultTranslatePos, offset)); // RLEGをswingする
        footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio));
    }else{ // 前回RLEGをswingした
        footstepNodesList.push_back(calcDefaultSwingStep(LLEG, footstepNodesList.back(), defaultTranslatePos, offset)); // LLEGをswingする
        footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio));
    }
  }else if(footstepNodesList.back().isSupportPhase[RLEG] && !footstepNodesList.back().isSupportPhase[LLEG]){ // LLEGが浮いている
    footstepNodesList.push_back(calcDefaultSwingStep(LLEG, footstepNodesList.back(), defaultTranslatePos, offset, true)); // LLEGをswingする. startWithSingleSupport
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio));
  }else if(!footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){ // RLEGが浮いている
    footstepNodesList.push_back(calcDefaultSwingStep(RLEG, footstepNodesList.back(), defaultTranslatePos, offset, true)); // RLEGをswingする. startWithSingleSupport
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio));
  }// footstepNodesListの末尾の要素が両方falseであることは無い
}

GaitParam::FootStepNodes FootStepGenerator::calcDefaultSwingStep(const int& swingLeg, const GaitParam::FootStepNodes& footstepNodes, const std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >& defaultTranslatePos, const cnoid::Vector3& offset, bool startWithSingleSupport) const{
  GaitParam::FootStepNodes fs;
  int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;

  cnoid::Position transform = cnoid::Position::Identity(); // supportLeg相対(Z軸は鉛直)での次のswingLegの位置
  transform.linear() = cnoid::Matrix3(Eigen::AngleAxisd(mathutil::clamp(offset[2],this->defaultStrideLimitationTheta), cnoid::Vector3::UnitZ()));
  transform.translation() = - defaultTranslatePos[supportLeg].value() + cnoid::Vector3(offset[0], offset[1], 0.0) + transform.linear() * defaultTranslatePos[swingLeg].value();
  transform.translation() = mathutil::calcNearestPointOfHull(transform.translation(), this->defaultStrideLimitationHull[swingLeg]);

  fs.dstCoords[supportLeg] = footstepNodes.dstCoords[supportLeg];
  cnoid::Position prevOrigin = mathutil::orientCoordToAxis(footstepNodes.dstCoords[supportLeg], cnoid::Vector3::UnitZ());
  fs.dstCoords[swingLeg] = prevOrigin * transform;
  fs.isSupportPhase[supportLeg] = true;
  fs.isSupportPhase[swingLeg] = false;
  fs.remainTime = this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio);
  if(!startWithSingleSupport) fs.stepHeight[swingLeg] = {this->defaultStepHeight,this->defaultStepHeight};
  else fs.stepHeight[swingLeg] = {0.0,this->defaultStepHeight};
  return fs;
}

GaitParam::FootStepNodes FootStepGenerator::calcDefaultDoubleSupportStep(const GaitParam::FootStepNodes& footstepNodes, double doubleSupportTime) const{
  GaitParam::FootStepNodes fs;
  for(int i=0;i<NUM_LEGS;i++){
    fs.dstCoords[i] = footstepNodes.dstCoords[i];
    fs.isSupportPhase[i] = true;
    fs.stepHeight[i] = {0.0,0.0};
  }
  fs.remainTime = doubleSupportTime;
  return fs;
}

inline std::ostream &operator<<(std::ostream &os, const std::vector<std::pair<std::vector<cnoid::Vector3>, double> >& candidates){
  for(int i=0;i<candidates.size();i++){
    os << "candidates[" << i << "] " << candidates[i].second << "s" << std::endl;
    for(int j=0;j<candidates[i].first.size();j++){
      os << candidates[i].first[j].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]"));
    }
    os << std::endl;
  }
  return os;
}

void FootStepGenerator::modifyFootSteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const GaitParam& gaitParam) const{
  // 現在片足支持期で、次が両足支持期であるときのみ、行う
  if(!(footstepNodesList.size() > 1 &&
       (footstepNodesList[1].isSupportPhase[RLEG] && footstepNodesList[1].isSupportPhase[LLEG]) &&
       ((footstepNodesList[0].isSupportPhase[RLEG] && !footstepNodesList[0].isSupportPhase[LLEG]) || (!footstepNodesList[0].isSupportPhase[RLEG] && footstepNodesList[0].isSupportPhase[LLEG]))))
     return;

  // 残り時間がoverwritableRemainTimeを下回っている場合、着地位置時間修正を行わない.
  if(footstepNodesList[0].remainTime < this->overwritableRemainTime) return;

  // one step capturabilityに基づき、footstepNodesList[0]のremainTimeとdstCoordsを修正する.
  int swingLeg = footstepNodesList[0].isSupportPhase[RLEG] ? LLEG : RLEG;
  int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;
  cnoid::Position swingPose = gaitParam.genCoords[swingLeg].value();
  cnoid::Position supportPose = gaitParam.genCoords[supportLeg].value(); // TODO. 支持脚のgenCoordsとdstCoordsが異なることは想定していない
  cnoid::Position supportPoseHorizontal = mathutil::orientCoordToAxis(supportPose, cnoid::Vector3::UnitZ());

  // dx = w ( x - z - l)
  cnoid::Vector3 actDCM = gaitParam.actCog + gaitParam.actCogVel.value() / gaitParam.omega;

  /*
    capturable: ある時刻t(overwritableMinTime<=t<=overwritableMaxTime)が存在し、時刻tに着地すれば転倒しないような着地位置.
    reachable: ある時刻t(overwritableMinTime<=t<=overwritableMaxTime)が存在し、今の脚の位置からの距離が時刻tに着地することができる範囲である着地位置.
    strideLimitation: overwritableStrideLimitationHullの範囲内の着地位置(自己干渉・IKの考慮が含まれる).
    steppable: 着地可能な地形であるような着地位置

    * capturableとreachableの積集合を考えるときは、各時刻のtごとで考える

    優先度(小さいほうが高い)
    1. strideLimitation: 絶対満たす
    1. reachable: 絶対満たす
    2. steppable: 達成不可の場合は、考慮しない
    3. capturable: 達成不可の場合は、可能な限り近い位置. 複数ある場合は時間が速い方優先. (次の一歩に期待) (角運動量 TODO)
    4. もとの着地位置(dstCoordsOrg): 達成不可の場合は、各hullの中の最も近い位置をそれぞれ求めて、着地位置修正前の進行方向(遊脚のsrcCoordsからの方向)に最も進むもの優先 (支持脚からの方向にすると、横歩き時に後ろ足の方向が逆になってしまう)
    5. もとの着地時刻(remainTimeOrg): 達成不可の場合は、可能な限り近い時刻
   */

  std::vector<std::pair<std::vector<cnoid::Vector3>, double> > candidates; // first: generate frame. 着地領域(convex Hull). second: 着地時刻. サイズが0になることはない

  // 1. strideLimitation と reachable
  {
    std::vector<double> samplingTimes;
    samplingTimes.push_back(footstepNodesList[0].remainTime);
    int sample = 10;
    double minTime = std::max(this->overwritableMinTime, this->overwritableMinStepTime - gaitParam.elapsedTime); // 次indexまでの残り時間がthis->overwritableMinTimeを下回るようには着地時間修正を行わない. 現index開始時からの経過時間がthis->overwritableStepMinTimeを下回るようには着地時間修正を行わない.
    minTime = std::min(minTime, footstepNodesList[0].remainTime); // もともと下回っている場合には、その値を下回るようには着地時刻修正を行わない.
    double maxTime = std::max(this->overwritableMaxStepTime - gaitParam.elapsedTime, minTime); // 現index開始時からの経過時間がthis->overwritableStepMaxTimeを上回るようには着地時間修正を行わない.
    maxTime = std::max(maxTime, footstepNodesList[0].remainTime); // もともと上回っている場合には、その値を上回るようには着地時刻修正を行わない.
    for(int i=0;i<=sample;i++) {
      double t = minTime + (maxTime - minTime) / sample * i;
      if(t != footstepNodesList[0].remainTime) samplingTimes.push_back(t);
    }

    std::vector<cnoid::Vector3> strideLimitationHull; // generate frame. overwritableStrideLimitationHullの範囲内の着地位置(自己干渉・IKの考慮が含まれる). Z成分には0を入れる
    for(int i=0;i<this->overwritableStrideLimitationHull[swingLeg].size();i++){
      cnoid::Vector3 p = supportPoseHorizontal * this->overwritableStrideLimitationHull[swingLeg][i];
      strideLimitationHull.emplace_back(p[0],p[1],0.0);
    }

    for(int i=0;i<samplingTimes.size();i++){
      double t = samplingTimes[i];
      std::vector<cnoid::Vector3> reachableHull; // generate frame. 今の脚の位置からの距離が時刻tに着地することができる範囲. Z成分には0を入れる
      int segment = 8;
      for(int j=0; j < segment; j++){
        reachableHull.emplace_back(swingPose.translation()[0] + this->overwritableMaxSwingVelocity * t * std::cos(2 * M_PI / segment * j),
                                   swingPose.translation()[1] + this->overwritableMaxSwingVelocity * t * std::sin(2 * M_PI / segment * j),
                                   0.0);
      }
      std::vector<cnoid::Vector3> hull = mathutil::calcIntersectConvexHull(reachableHull, strideLimitationHull);
      if(hull.size() > 0) candidates.emplace_back(hull, t);
    }

    if(candidates.size() == 0) candidates.emplace_back(std::vector<cnoid::Vector3>{footstepNodesList[0].dstCoords[swingLeg].translation()}, footstepNodesList[0].remainTime); // まず起こらないと思うが念の為
  }
  // std::cerr << "strideLimitation と reachable" << std::endl;
  // std::cerr << candidates << std::endl;

  // 2. steppable: 達成不可の場合は、考慮しない
  // TODO. Z高さの扱い.(DOWN_PHASEのときはfootstepNodesList[0]のdstCoordsはgenCoordsよりも高い位置に変更されることはない) (高低差と時間の関係)

  // 3. capturable: 達成不可の場合は、可能な限り近い位置. 複数ある場合は時間が速い方優先. (次の一歩に期待) (角運動量 TODO)
  // 次の両足支持期終了時に入るケースでもOKにしたい
  {
    std::vector<std::vector<cnoid::Vector3> > capturableHulls; // 要素数と順番はcandidatesに対応
    for(int i=0;i<candidates.size();i++){
      std::vector<cnoid::Vector3> capturableVetices; // generate frame. 時刻tに着地すれば転倒しないような着地位置. Z成分には0を入れる
      for(double t = candidates[i].second; t <= candidates[i].second + footstepNodesList[1].remainTime; t += footstepNodesList[1].remainTime){ // 接地する瞬間と、次の両足支持期の終了時. 片方だけだと特に横歩きのときに厳しすぎる.
        for(int j=0;j<this->safeLegHull[supportLeg].size();j++){
          cnoid::Vector3 zmp = supportPose * this->safeLegHull[supportLeg][j];// generate frame
          cnoid::Vector3 endDCM = (actDCM - zmp - gaitParam.l) * std::exp(gaitParam.omega * t) + zmp + gaitParam.l; // generate frame. 着地時のDCM
          // for(int k=0;k<this->safeLegHull[swingLeg].size();k++){
          //   cnoid::Vector3 p = endDCM - gaitParam.l - footstepNodesList[0].dstCoords[swingLeg].linear() * this->safeLegHull[swingLeg][k]; // こっちのほうが厳密であり、着地位置時刻修正を最小限にできるが、ロバストさに欠ける
          //   capturableVetices.emplace_back(p[0], p[1], 0.0);
          // }
          cnoid::Vector3 p = endDCM - gaitParam.l - footstepNodesList[0].dstCoords[swingLeg].linear() * gaitParam.copOffset[swingLeg].value();
          capturableVetices.emplace_back(p[0], p[1], 0.0);
        }
      }
      capturableHulls.push_back(mathutil::calcConvexHull(capturableVetices)); // generate frame. 時刻tに着地すれば転倒しないような着地位置. Z成分には0を入れる
    }

    std::vector<std::pair<std::vector<cnoid::Vector3>, double> > nextCandidates;
    for(int i=0;i<candidates.size();i++){
      std::vector<cnoid::Vector3> hull = mathutil::calcIntersectConvexHull(candidates[i].first, capturableHulls[i]);
      if(hull.size() > 0) nextCandidates.emplace_back(hull, candidates[i].second);
    }
    if(nextCandidates.size() > 0) candidates = nextCandidates;
    else{
      // 達成不可の場合は、時間が速い方優先(次の一歩に期待). 複数ある場合は可能な限り近い位置.
      //   どうせこの一歩ではバランスがとれないので、位置よりも速く次の一歩に移ることを優先したほうが良い
      double minTime = std::numeric_limits<double>::max();
      double minDistance = std::numeric_limits<double>::max();
      cnoid::Vector3 minp;
      for(int i=0;i<candidates.size();i++){
        if(candidates[i].second <= minTime){
          std::vector<cnoid::Vector3> p, q;
          double distance = mathutil::calcNearestPointOfTwoHull(candidates[i].first, capturableHulls[i], p, q); // candidates[i].first, capturableHulls[i]は重なっていない・接していない
          if(candidates[i].second < minTime ||
             (candidates[i].second == minTime && distance < minDistance)){
            minTime = candidates[i].second;
            minDistance = distance;
            nextCandidates.clear();
            nextCandidates.emplace_back(p,minTime); // pは、最近傍が点の場合はその点が入っていて、最近傍が線分の場合はその線分の両端点が入っている
          }else if(candidates[i].second == minTime && distance == minDistance){
            nextCandidates.emplace_back(p,minTime);
          }
        }
      }
      candidates = nextCandidates;
    }
  }

  // std::cerr << "capturable" << std::endl;
  // std::cerr << candidates << std::endl;

  // 4. もとの着地位置: 達成不可の場合は、各hullの中の最も近い位置をそれぞれ求めて、着地位置修正前の進行方向(遊脚のsrcCoordsからの方向)に最も進むもの優先 (支持脚からの方向にすると、横歩き時に後ろ足の方向が逆になってしまう)
  {
    std::vector<std::pair<std::vector<cnoid::Vector3>, double> > nextCandidates;
    for(int i=0;i<candidates.size();i++){
      if(mathutil::isInsideHull(gaitParam.dstCoordsOrg[swingLeg].translation(), candidates[i].first)){
        nextCandidates.emplace_back(std::vector<cnoid::Vector3>{gaitParam.dstCoordsOrg[swingLeg].translation()},candidates[i].second);
      }
    }
    if(nextCandidates.size() > 0) candidates = nextCandidates;
    else{ // 達成不可の場合は、各hullの中の最も近い位置をそれぞれ求めて、着地位置修正前の進行方向(遊脚のsrcCoordsからの方向)に最も進むもの優先
      cnoid::Vector3 dir = gaitParam.dstCoordsOrg[swingLeg].translation() - gaitParam.srcCoords[swingLeg].translation(); dir[2] = 0.0;
      if(dir.norm() != 0){ //各hullの中の最も近い位置をそれぞれ求めて、着地位置修正前の進行方向(遊脚のsrcCoordsからの方向)に最も進むもの優先
        dir = dir.normalized();
        double maxVel = - std::numeric_limits<double>::max();
        for(int i=0;i<candidates.size();i++){
          cnoid::Vector3 p = mathutil::calcNearestPointOfHull(gaitParam.dstCoordsOrg[swingLeg].translation(), candidates[i].first);
          double vel = (p - gaitParam.srcCoords[swingLeg].translation()).dot(dir);
          if(vel > maxVel){
            maxVel = vel;
            nextCandidates.clear();
            nextCandidates.emplace_back(std::vector<cnoid::Vector3>{p}, candidates[i].second);
          }else if (vel == maxVel){
            maxVel = vel;
            nextCandidates.emplace_back(std::vector<cnoid::Vector3>{p}, candidates[i].second);
          }
        }
      }else{ // 進行方向が定義できない. //各hullの中の最も近い位置をそれぞれ求めて、遊脚のsrcCoordsからの距離が最も小さいもの優先
        double minVel = + std::numeric_limits<double>::max();
        for(int i=0;i<candidates.size();i++){
          cnoid::Vector3 p = mathutil::calcNearestPointOfHull(gaitParam.dstCoordsOrg[swingLeg].translation(), candidates[i].first);
          double vel = (p - gaitParam.srcCoords[swingLeg].translation()).norm();
          if(vel < minVel){
            minVel = vel;
            nextCandidates.clear();
            nextCandidates.emplace_back(std::vector<cnoid::Vector3>{p}, candidates[i].second);
          }else if (vel == minVel){
            minVel = vel;
            nextCandidates.emplace_back(std::vector<cnoid::Vector3>{p}, candidates[i].second);
          }
        }
      }
      candidates = nextCandidates;
    }
  }

  // std::cerr << "pos" << std::endl;
  // std::cerr << candidates << std::endl;

  // 5. もとの着地時刻(remainTimeOrg): 達成不可の場合は、可能な限り近い時刻
  {
    std::vector<std::pair<std::vector<cnoid::Vector3>, double> > nextCandidates;
    double targetRemainTime = gaitParam.remainTimeOrg - gaitParam.elapsedTime; // 負になっているかもしれない.
    double minDiffTime = std::numeric_limits<double>::max();
    for(int i=0;i<candidates.size();i++){
      double diffTime = std::abs(candidates[i].second - targetRemainTime);
      if(diffTime < minDiffTime){
        minDiffTime = diffTime;
        nextCandidates.clear();
        nextCandidates.push_back(candidates[i]);
      }else if(diffTime == minDiffTime){
        minDiffTime = diffTime;
        nextCandidates.push_back(candidates[i]);
      }
    }
    candidates = nextCandidates;
  }

  // std::cerr << "time" << std::endl;
  // std::cerr << candidates << std::endl;

  // 修正を適用
  cnoid::Vector3 nextDstCoordsPos = candidates[0].first[0];
  cnoid::Vector3 displacement = nextDstCoordsPos - footstepNodesList[0].dstCoords[swingLeg].translation();
  displacement[2] = 0.0;
  this->transformFutureSteps(footstepNodesList, 0, displacement);
  footstepNodesList[0].remainTime = candidates[0].second;
}

// 早づきしたらremainTimeをdtに減らしてすぐに次のnodeへ移る. この機能が無いと少しでもロボットが傾いて早づきするとジャンプするような挙動になる. 遅づきに備えるために、着地位置を下方にオフセットさせる
void FootStepGenerator::checkEarlyTouchDown(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const GaitParam& gaitParam, double dt) const{
  for(int i=0;i<NUM_LEGS;i++){
    actLegWrenchFilter[i].passFilter(gaitParam.actEEWrench[i], dt);
  }

  // 現在swing期で次support期のLegがあって、
  // そのLegがDOWN_PHASEかつ力センサの値が閾値以上になった場合に、すぐに次のnodeに移る
  if(footstepNodesList.size() > 1 &&
     ((!footstepNodesList[0].isSupportPhase[RLEG] && footstepNodesList[1].isSupportPhase[RLEG] && //現在swing期で次support期
       gaitParam.swingState[RLEG] == GaitParam::DOWN_PHASE && // DOWN_PHASE
       actLegWrenchFilter[RLEG].value()[2] > this->contactDetectionThreshold /*generate frame. ロボットが受ける力*/) // 力センサの値が閾値以上
      ||
      (!footstepNodesList[0].isSupportPhase[LLEG] && footstepNodesList[1].isSupportPhase[LLEG] && //現在swing期で次support期
       gaitParam.swingState[LLEG] == GaitParam::DOWN_PHASE && // DOWN_PHASE
       actLegWrenchFilter[LLEG].value()[2] > this->contactDetectionThreshold /*generate frame. ロボットが受ける力*/))){ // 力センサの値が閾値以上
    footstepNodesList[0].remainTime = dt;
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
      if(footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){
        footstepNodesList.back().remainTime = this->defaultStepTime * this->defaultDoubleSupportRatio; // 末尾の両足支持期を短縮 & 延長
      }
      this->calcDefaultNextStep(footstepNodesList, gaitParam.defaultTranslatePos, 1e-6 * dir); // 現在両足で支持している場合、dirの方向の足を最初にswingする
    }
  }
}

// Stable Go Stop
void FootStepGenerator::checkStableGoStop(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const GaitParam& gaitParam) const{
  // 着地位置修正を行ったなら、footstepNodesListがemergencyStepNumのサイズになるまで歩くnodeが末尾に入る.
  for(int i=0;i<NUM_LEGS; i++){
    if((footstepNodesList[0].dstCoords[i].translation().head<2>() - gaitParam.dstCoordsOrg[i].translation().head<2>()).norm() > 0.01){ // 1cm以上着地位置修正を行ったなら
      while(footstepNodesList.size() < this->emergencyStepNum){
        if(footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){
          footstepNodesList.back().remainTime = this->defaultStepTime * this->defaultDoubleSupportRatio; // 末尾の両足支持期を短縮 & 延長
        }
        this->calcDefaultNextStep(footstepNodesList, gaitParam.defaultTranslatePos);
      }
    }
  }
}
