#include "FootStepGenerator.h"
#include "MathUtil.h"

bool FootStepGenerator::initFootStepNodesList(const cnoid::BodyPtr& genRobot, const GaitParam& gaitParam,
                                              std::vector<GaitParam::FootStepNodes>& o_footstepNodesList, std::vector<cnoid::Position>& o_srcCoords, std::vector<bool>& o_prevSupportPhase) const{
  // footStepNodesListを初期化する
  std::vector<GaitParam::FootStepNodes> footstepNodesList(1);
  std::vector<cnoid::Position> srcCoords(NUM_LEGS);
  cnoid::Position rlegCoords = genRobot->link(gaitParam.eeParentLink[RLEG])->T()*gaitParam.eeLocalT[RLEG];
  cnoid::Position llegCoords = genRobot->link(gaitParam.eeParentLink[LLEG])->T()*gaitParam.eeLocalT[LLEG];
  footstepNodesList[0].dstCoords = {rlegCoords, llegCoords};
  footstepNodesList[0].isSupportPhase = {true, true};
  footstepNodesList[0].remainTime = 0.0;

  std::vector<bool> prevSupportPhase(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++){
    prevSupportPhase[i] = footstepNodesList[0].isSupportPhase[i];
  }

  o_prevSupportPhase = prevSupportPhase;
  o_footstepNodesList = footstepNodesList;
  o_srcCoords = srcCoords;

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

  if(footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){
    footstepNodesList.back().remainTime = this->defaultDoubleSupportTime;
  }else if(!footstepNodesList.back().isSupportPhase[RLEG] && footsteps[1].l_r == LLEG){ // RLEGを下ろす必要がある
    this->calcDefaultNextStep(footstepNodesList, gaitParam.defaultTranslatePos);
  }else if(!footstepNodesList.back().isSupportPhase[LLEG] && footsteps[1].l_r == RLEG){ // LLEGを下ろす必要がある
    this->calcDefaultNextStep(footstepNodesList, gaitParam.defaultTranslatePos);
  }

  // footstepsの0番目の要素は、実際には歩かず、基準座標としてのみ使われる. footstepNodesList[0].dstCoordsのZ軸を鉛直に直した座標系と、footstepsの0番目の要素のZ軸を鉛直に直した座標系が一致するように座標変換する.
  cnoid::Position trans;
  {
    cnoid::Position genCoords = mathutil::orientCoordToAxis(footstepNodesList.back().dstCoords[footsteps[0].l_r], cnoid::Vector3::UnitZ());
    cnoid::Position refCoords = mathutil::orientCoordToAxis(footsteps[0].coords, cnoid::Vector3::UnitZ());
    trans = genCoords * refCoords.inverse();
  }

  for(int i=1;i<footsteps.size();i++){
    GaitParam::FootStepNodes fs;
    int swingLeg = footsteps[i].l_r;
    int supportLeg = swingLeg == RLEG ? LLEG: RLEG;
    fs.dstCoords[supportLeg] = footstepNodesList.back().dstCoords[supportLeg];
    fs.dstCoords[swingLeg] = trans * footsteps[i].coords;
    fs.isSupportPhase[supportLeg] = true;
    fs.isSupportPhase[swingLeg] = false;
    if(footsteps[i].stepTime > this->defaultDoubleSupportTime){
      fs.remainTime = footsteps[i].stepTime - this->defaultDoubleSupportTime;
    }else{
      fs.remainTime = this->defaultStepTime - this->defaultDoubleSupportTime;
    }
    double stepHeight = std::max(0.0, footsteps[i].stepHeight);
    if(footstepNodesList.back().isSupportPhase[swingLeg]){
      fs.stepHeight[swingLeg] = {stepHeight,stepHeight};
    }else{
      fs.stepHeight[swingLeg] = {0.0,stepHeight};
    }
    footstepNodesList.push_back(fs);

    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back()));
  }

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

  o_footstepNodesList = footstepNodesList;
  return true;

}

bool FootStepGenerator::calcFootSteps(const GaitParam& gaitParam, const double& dt,
                                      std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const{
  std::vector<GaitParam::FootStepNodes> footstepNodesList = gaitParam.footstepNodesList;

  // goVelocityModeなら、進行方向に向けてfootStepNodesList[1] ~ footStepNodesList[goVelocityStepNum]の要素を機械的に計算してどんどん末尾appendしていく. cmdVelに応じてきまる
  if(this->isGoVelocityMode){
    if(footstepNodesList[0].remainTime <= dt){ // footstepの切り替わりのタイミング. this->updateGoVelocityStepsを毎周期呼ぶと、数値誤差でだんだん変にずれてくるので
      this->updateGoVelocitySteps(footstepNodesList, gaitParam.defaultTranslatePos);
    }
    while(footstepNodesList.size() < this->goVelocityStepNum){
      this->calcDefaultNextStep(footstepNodesList, gaitParam.defaultTranslatePos, this->cmdVel * this->defaultStepTime);
    }
  }

  if(this->isModifyFootSteps && this->isEmergencyStepMode){
    // TODO
  }

  if(this->isModifyFootSteps){
    this->modifyFootSteps(footstepNodesList, gaitParam);
  }

  o_footstepNodesList = footstepNodesList;
  return true;
}

bool FootStepGenerator::advanceFootStepNodesList(const GaitParam& gaitParam, double dt,
                                                 std::vector<GaitParam::FootStepNodes>& o_footstepNodesList, std::vector<cnoid::Position>& o_srcCoords, std::vector<bool>& o_prevSupportPhase) const{
  // prevSupportPhaseを記録
  std::vector<bool> prevSupportPhase(2);
  for(int i=0;i<NUM_LEGS;i++) prevSupportPhase[i] = gaitParam.footstepNodesList[0].isSupportPhase[i];

  // footstepNodesListを進める
  std::vector<GaitParam::FootStepNodes> footstepNodesList = gaitParam.footstepNodesList;
  std::vector<cnoid::Position> srcCoords = gaitParam.srcCoords;
  footstepNodesList[0].remainTime = std::max(0.0, footstepNodesList[0].remainTime - dt);
  if(footstepNodesList[0].remainTime <= 0.0 && footstepNodesList.size() > 1){
    for(int i=0;i<NUM_LEGS;i++) srcCoords[i] = gaitParam.genCoords[i].value();
    footstepNodesList.erase(footstepNodesList.begin()); // vectorではなくlistにするべき?
  }

  o_prevSupportPhase = prevSupportPhase;
  o_footstepNodesList = footstepNodesList;
  o_srcCoords = srcCoords;

  return true;
}

void FootStepGenerator::updateGoVelocitySteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const std::vector<cnoid::Vector3>& defaultTranslatePos) const{
  for(int i=1;i<footstepNodesList.size()-1;i++){
    if(((footstepNodesList[i].isSupportPhase[RLEG] && !footstepNodesList[i].isSupportPhase[LLEG]) || (!footstepNodesList[i].isSupportPhase[RLEG] && footstepNodesList[i].isSupportPhase[LLEG])) && // 今が片脚支持期
       (footstepNodesList[i+1].isSupportPhase[RLEG] && footstepNodesList[i+1].isSupportPhase[LLEG])){ // 次が両足支持期
      int swingLeg = footstepNodesList[i].isSupportPhase[RLEG] ? LLEG : RLEG;
      int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;
      cnoid::Vector3 offset = this->cmdVel * (footstepNodesList[i].remainTime + footstepNodesList[i+1].remainTime);
      cnoid::Position transform = cnoid::Position::Identity(); // supportLeg相対(Z軸は鉛直)での次のswingLegの位置
      transform.linear() = cnoid::Matrix3(Eigen::AngleAxisd(mathutil::clamp(offset[2],this->defaultStrideLimitationTheta), cnoid::Vector3::UnitZ()));
      transform.translation() = - defaultTranslatePos[supportLeg] + cnoid::Vector3(offset[0], offset[1], 0.0) + transform.linear() * defaultTranslatePos[swingLeg];
      transform.translation() = mathutil::calcNearestPointOfHull(transform.translation(), this->defaultStrideLimitationHull[swingLeg]);
      cnoid::Position prevOrigin = mathutil::orientCoordToAxis(footstepNodesList[i-1].dstCoords[supportLeg], cnoid::Vector3::UnitZ());
      cnoid::Position dstCoords = prevOrigin * transform;

      cnoid::Position origin = mathutil::orientCoordToAxis(footstepNodesList[i].dstCoords[swingLeg], cnoid::Vector3::UnitZ());
      cnoid::Position displacement = origin.inverse() * dstCoords;
      this->transformFutureSteps(footstepNodesList, i, origin, displacement);
    }
  }
}

void FootStepGenerator::transformFutureSteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, int index, const cnoid::Position& transformOrigin/*generate frame*/, const cnoid::Position& transform/*transform Origin frame*/) const{
  cnoid::Position trans = transformOrigin * transform * transformOrigin.inverse();
  for(int l=0;l<NUM_LEGS;l++){
    bool swinged = false;
    for(int i=index;i<footstepNodesList.size();i++){
      if(!footstepNodesList[i].isSupportPhase[l]) swinged = true;
      if(swinged) footstepNodesList[i].dstCoords[l] = trans * footstepNodesList[i].dstCoords[l];
    }
  }
}

void FootStepGenerator::calcDefaultNextStep(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const std::vector<cnoid::Vector3>& defaultTranslatePos, const cnoid::Vector3& offset) const{
  if(footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){
    footstepNodesList.back().remainTime = std::max(footstepNodesList.back().remainTime, this->defaultDoubleSupportTime);
    if(footstepNodesList.size() == 1 ||
       (footstepNodesList[footstepNodesList.size()-2].isSupportPhase[RLEG] && footstepNodesList[footstepNodesList.size()-2].isSupportPhase[LLEG]) ||
       (!footstepNodesList[footstepNodesList.size()-2].isSupportPhase[RLEG] && !footstepNodesList[footstepNodesList.size()-2].isSupportPhase[LLEG]) ){
      // どっちをswingしてもいいので、進行方向に近いLegをswingする
      cnoid::Vector2 rlegTolleg = (defaultTranslatePos[LLEG] - defaultTranslatePos[RLEG]).head<2>();
      if(rlegTolleg.dot(offset.head<2>()) > 0) {
        footstepNodesList.push_back(calcDefaultSwingStep(LLEG, footstepNodesList.back(), defaultTranslatePos, offset)); // LLEGをswingする
        footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back()));
      }else{
        footstepNodesList.push_back(calcDefaultSwingStep(RLEG, footstepNodesList.back(), defaultTranslatePos, offset)); // RLEGをswingする
        footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back()));
      }
    }else if(footstepNodesList[footstepNodesList.size()-2].isSupportPhase[RLEG]){ // 前回LLEGをswingした
        footstepNodesList.push_back(calcDefaultSwingStep(RLEG, footstepNodesList.back(), defaultTranslatePos, offset)); // RLEGをswingする
        footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back()));
    }else{ // 前回RLEGをswingした
        footstepNodesList.push_back(calcDefaultSwingStep(LLEG, footstepNodesList.back(), defaultTranslatePos, offset)); // LLEGをswingする
        footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back()));
    }
  }else if(footstepNodesList.back().isSupportPhase[RLEG] && !footstepNodesList.back().isSupportPhase[LLEG]){ // LLEGが浮いている
    footstepNodesList.push_back(calcDefaultSwingStep(LLEG, footstepNodesList.back(), defaultTranslatePos, offset, true)); // LLEGをswingする. startWithSingleSupport
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back()));
  }else if(!footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){ // RLEGが浮いている
    footstepNodesList.push_back(calcDefaultSwingStep(RLEG, footstepNodesList.back(), defaultTranslatePos, offset, true)); // RLEGをswingする. startWithSingleSupport
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back()));
  }// footstepNodesListの末尾の要素が両方falseであることは無い
}

GaitParam::FootStepNodes FootStepGenerator::calcDefaultSwingStep(const int& swingLeg, const GaitParam::FootStepNodes& footstepNodes, const std::vector<cnoid::Vector3>& defaultTranslatePos, const cnoid::Vector3& offset, bool startWithSingleSupport) const{
  GaitParam::FootStepNodes fs;
  int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;

  cnoid::Position transform = cnoid::Position::Identity(); // supportLeg相対(Z軸は鉛直)での次のswingLegの位置
  transform.linear() = cnoid::Matrix3(Eigen::AngleAxisd(mathutil::clamp(offset[2],this->defaultStrideLimitationTheta), cnoid::Vector3::UnitZ()));
  transform.translation() = - defaultTranslatePos[supportLeg] + cnoid::Vector3(offset[0], offset[1], 0.0) + transform.linear() * defaultTranslatePos[swingLeg];
  transform.translation() = mathutil::calcNearestPointOfHull(transform.translation(), this->defaultStrideLimitationHull[swingLeg]);

  fs.dstCoords[supportLeg] = footstepNodes.dstCoords[supportLeg];
  cnoid::Position prevOrigin = mathutil::orientCoordToAxis(footstepNodes.dstCoords[supportLeg], cnoid::Vector3::UnitZ());
  fs.dstCoords[swingLeg] = prevOrigin * transform;
  fs.isSupportPhase[supportLeg] = true;
  fs.isSupportPhase[swingLeg] = false;
  fs.remainTime = this->defaultStepTime - this->defaultDoubleSupportTime;
  if(!startWithSingleSupport) fs.stepHeight[swingLeg] = {this->defaultStepHeight,this->defaultStepHeight};
  else fs.stepHeight[swingLeg] = {0.0,this->defaultStepHeight};
  return fs;
}

GaitParam::FootStepNodes FootStepGenerator::calcDefaultDoubleSupportStep(const GaitParam::FootStepNodes& footstepNodes) const{
  GaitParam::FootStepNodes fs;
  for(int i=0;i<NUM_LEGS;i++){
    fs.dstCoords[i] = footstepNodes.dstCoords[i];
    fs.isSupportPhase[i] = true;
    fs.stepHeight[i] = {0.0,0.0};
  }
  fs.remainTime = this->defaultDoubleSupportTime;
  return fs;
}

void FootStepGenerator::modifyFootSteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, const GaitParam& gaitParam) const{
  // 現在片足支持期で、次が両足支持期であるときのみ、行う
  if(!(footstepNodesList.size() > 1 &&
       (footstepNodesList[1].isSupportPhase[RLEG] && footstepNodesList[1].isSupportPhase[LLEG]) &&
       (footstepNodesList[0].isSupportPhase[RLEG] && !footstepNodesList[0].isSupportPhase[LLEG]) || (!footstepNodesList[0].isSupportPhase[RLEG] && footstepNodesList[0].isSupportPhase[LLEG])))
     return;

  // 次indexまでの残り時間がこの値未満の場合は着地位置時間修正を行わない.
  if(footstepNodesList[0].remainTime < this->overwritableMinTime) return;

  // one step capturabilityに基づき、footstepNodesList[0]のremainTimeとdstCoordsを修正する.
  int swingLeg = footstepNodesList[0].isSupportPhase[RLEG] ? LLEG : RLEG;
  int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;
  cnoid::Position swingPose = gaitParam.genCoords[swingLeg].value();
  cnoid::Position supportPose = gaitParam.genCoords[supportLeg].value();
  cnoid::Position supportPoseHorizontal = mathutil::orientCoordToAxis(supportPose, cnoid::Vector3::UnitZ());

  // dx = w ( x - z - l)
  double w = std::sqrt(gaitParam.g/gaitParam.refdz); // TODO refforceZ
  cnoid::Vector3 l = cnoid::Vector3::Zero();
  l[2] = gaitParam.refdz;
  cnoid::Vector3 actDCM = gaitParam.actCog + gaitParam.actCogVel.value() / w;

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
    3. capturable: 達成不可の場合は、(次の一歩に期待)
    4. もとの着地位置: 達成不可の場合は、可能な限り近く. 複数ある場合は進行方向優先
    5. もとの着地時刻: 達成不可の場合は、可能な限り近く
   */

  std::vector<cnoid::Vector3> capturableReachableHull; // generate frame. ある時刻tが存在し、今の脚の位置からの距離が時刻tに着地することができる範囲内で、かつ、時刻tに着地すれば転倒しないような着地位置の凸包. Zは成分はてきとうで良い
  {
    std::vector<std::vector<cnoid::Vector3> > capturableReachableHulls; // generate frame. ある時刻tが存在し、今の脚の位置からの距離が時刻tに着地することができる範囲内で、かつ、時刻tに着地すれば転倒しないような着地位置の凸包の集合. Z成分はてきとうで良い
    std::vector<double> samplingTimes;
    samplingTimes.push_back(footstepNodesList[0].remainTime);
    for(int i=0;i<=10;i++) samplingTimes.push_back(this->overwritableMinTime + (this->overwritableMaxTime - this->overwritableMinTime) * 0.1 * i);
    for(int i=0;i<samplingTimes.size();i++){
      double t = samplingTimes[i];
      std::vector<cnoid::Vector3> reachableHull; // generate frame. 今の脚の位置からの距離が時刻tに着地することができる範囲. Z成分には0を入れる
      for(int j=0; j < 16; j++){
        reachableHull.emplace_back(swingPose.translation()[0] + this->overwritableMaxSwingVelocity * t * std::cos(M_PI / 8 * j),
                                   swingPose.translation()[1] + this->overwritableMaxSwingVelocity * t * std::sin(M_PI / 8 * j),
                                   0.0);
      }
      std::vector<cnoid::Vector3> capturableVetices; // generate frame. 時刻tに着地すれば転倒しないような着地位置. Z成分には0を入れる
      for(int j=0;j<this->safeLegHull[supportLeg].size();j++){
        cnoid::Vector3 zmp = supportPose * this->safeLegHull[supportLeg][j];// generate frame
        cnoid::Vector3 endDCM = (actDCM - zmp - l) * std::exp(w * t) + zmp + l; // generate frame. 着地時のDCM
        for(int k=0;k<this->safeLegHull[swingLeg].size();k++){
          cnoid::Vector3 p = endDCM - swingPose.linear() * this->safeLegHull[swingLeg][k];
          capturableVetices.emplace_back(p[0], p[1], 0.0);
        }
      }
      std::vector<cnoid::Vector3> capturableHull = mathutil::calcConvexHull(capturableVetices); // generate frame. 時刻tに着地すれば転倒しないような着地位置. Z成分には0を入れる
      capturableReachableHulls.push_back(mathutil::calcIntersectConvexHull(reachableHull, capturableHull));
    }
    std::vector<cnoid::Vector3> capturableReachableVertices;
    for(int i=0;i<capturableReachableHulls.size();i++){
      std::copy(capturableReachableHulls[i].begin(), capturableReachableHulls[i].end(), std::back_inserter(capturableReachableVertices));
    }
    capturableReachableHull = mathutil::calcConvexHull(capturableReachableVertices);
  }

  std::vector<cnoid::Vector3> strideLimitationHull; // generate frame. overwritableStrideLimitationHullの範囲内の着地位置(自己干渉・IKの考慮が含まれる). Z成分には0を入れる
  for(int i=0;i<this->overwritableStrideLimitationHull[swingLeg].size();i++){
    cnoid::Vector3 p = supportPoseHorizontal * this->overwritableStrideLimitationHull[swingLeg][i];
    strideLimitationHull.emplace_back(p[0],p[1],0.0);
  }

  std::vector<cnoid::Vector3> capturableStepHullStrideLimited = mathutil::calcIntersectConvexHull(capturableReachableHull,strideLimitationHull); // generate frame.

  // TODO vision
}
