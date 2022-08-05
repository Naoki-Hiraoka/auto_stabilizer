#include "FootStepGenerator.h"
#include "MathUtil.h"

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
    if(footstepNodesList[0].remainTime <= dt){ // footstepの切り替わりのタイミング
      this->updateGoVelocitySteps(footstepNodesList, gaitParam.defaultTranslatePos);
    }
    while(footstepNodesList.size() < this->goVelocityStepNum){
      this->calcDefaultNextStep(footstepNodesList, gaitParam.defaultTranslatePos, this->cmdVel * this->defaultStepTime);
    }
  }

  if(this->modifyFootSteps && this->isEmergencyStepMode){
    // TODO
  }

  if(this->modifyFootSteps){
    // TODO
  }

  o_footstepNodesList = footstepNodesList;
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

      cnoid::Position displacement = dstCoords * footstepNodesList[i].dstCoords[swingLeg].inverse();
      this->transformFutureSteps(footstepNodesList, i, displacement);
    }
  }
}

void FootStepGenerator::transformFutureSteps(std::vector<GaitParam::FootStepNodes>& footstepNodesList, int index, const cnoid::Position& transform) const{
  for(int l=0;l<NUM_LEGS;l++){
    bool swinged = false;
    for(int i=index;i<footstepNodesList.size();i++){
      if(!footstepNodesList[i].isSupportPhase[l]) swinged = true;
      if(swinged) footstepNodesList[i].dstCoords[l] = transform * footstepNodesList[i].dstCoords[l];
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
    }else if(footstepNodesList[footstepNodesList.size()-2].isSupportPhase[RLEG]){
        footstepNodesList.push_back(calcDefaultSwingStep(RLEG, footstepNodesList.back(), defaultTranslatePos, offset)); // RLEGをswingする
        footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back()));
    }else{
        footstepNodesList.push_back(calcDefaultSwingStep(LLEG, footstepNodesList.back(), defaultTranslatePos, offset)); // LLEGをswingする
        footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back()));
    }
  }else if(footstepNodesList.back().isSupportPhase[RLEG] && !footstepNodesList.back().isSupportPhase[LLEG]){
    footstepNodesList.push_back(calcDefaultSwingStep(LLEG, footstepNodesList.back(), defaultTranslatePos, offset, true)); // LLEGをswingする. startWithSingleSupport
    footstepNodesList.push_back(calcDefaultDoubleSupportStep(footstepNodesList.back()));
  }else if(!footstepNodesList.back().isSupportPhase[RLEG] && footstepNodesList.back().isSupportPhase[LLEG]){
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
