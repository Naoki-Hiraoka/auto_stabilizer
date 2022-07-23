#include "FootStepGenerator.h"
#include "MathUtil.h"

bool FootStepGenerator::setFootSteps(const GaitParam& gaitParam, const std::vector<StepNode>& footsteps, std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const{
  if(footsteps.size() <= 1) { // 何もしない
    o_footstepNodesList = gaitParam.footstepNodesList;
    return true;
  }

  std::vector<GaitParam::FootStepNodes> footstepNodesList;
  footstepNodesList.push_back(gaitParam.footstepNodesList[0]);

  // footstepNodesList[1]開始時のsupport/swingの状態を上書きによって変更する場合は、footstepNodesList[0]の終了時の状態が両脚支持でかつその期間の時間がdefaultDoubleSupportTimeよりも短いなら延長する
  if((gaitParam.isSupportPhaseEnd(RLEG,0) && gaitParam.isSupportPhaseEnd(LLEG,0)) && // footstepNodesList[0]の終了時の状態が両脚支持
     std::min({gaitParam.footstepNodesList[0].remainTime, gaitParam.footstepNodesList[0].supportTime[RLEG], gaitParam.footstepNodesList[0].supportTime[LLEG]}) < this->defaultDoubleSupportTime &&
     gaitParam.footstepNodesList.size() > 1 &&
     ((gaitParam.isSupportPhaseStart(RLEG,1) && footsteps[1].l_r == RLEG) || (gaitParam.isSupportPhaseStart(LLEG,1) && footsteps[1].l_r == LLEG) || (!gaitParam.isSupportPhaseStart(RLEG,1) && footsteps[1].l_r == LLEG) || (gaitParam.isSupportPhaseStart(LLEG,1) && footsteps[1].l_r == RLEG))
     ){
    double extendTime =this->defaultDoubleSupportTime - std::min({gaitParam.footstepNodesList[0].remainTime, gaitParam.footstepNodesList[0].supportTime[RLEG], gaitParam.footstepNodesList[0].supportTime[LLEG]});
    footstepNodesList[0].remainTime += extendTime;
    footstepNodesList[0].supportTime[RLEG] += extendTime;
    footstepNodesList[0].supportTime[LLEG] += extendTime;
  }

  // footstepNodesList[0]終了時にswing状態の足をfootstepNodesList[1]開始時にsupportにする必要がある場合は、footstepNodesList[0]の直後に両脚が横に並ぶ位置に一歩歩いてその足を下ろすnodeが挿入される.
  if((!gaitParam.isSupportPhaseEnd(RLEG,0) && footsteps[1].l_r == LLEG) || // RLEGを下ろす必要がある
     (!gaitParam.isSupportPhaseEnd(LLEG,0) && footsteps[1].l_r == RLEG)) { // LLEGを下ろす必要がある.
    GaitParam::FootStepNodes fs;
    int supportLeg = footsteps[1].l_r;
    int swingLeg = supportLeg == RLEG ? LLEG: RLEG;

    fs.dstCoords[supportLeg] = footstepNodesList.back().dstCoords[supportLeg];
    fs.dstCoords[swingLeg] = mathutil::orientCoordToAxis(footstepNodesList.back().dstCoords[supportLeg], cnoid::Vector3::UnitZ());
    fs.dstCoords[swingLeg].translation() += fs.dstCoords[swingLeg].linear() * (- this->defaultTranslatePos[supportLeg] + this->defaultTranslatePos[swingLeg]);
    fs.supportTime[supportLeg] = std::numeric_limits<double>::max();
    fs.supportTime[swingLeg] = this->defaultDoubleSupportTime;
    fs.remainTime = this->defaultStepTime;
    fs.stepHeight[swingLeg] = {0.0,this->defaultStepHeight};
    footstepNodesList.push_back(fs);
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
    fs.supportTime[supportLeg] = std::numeric_limits<double>::max();
    fs.supportTime[swingLeg] = this->defaultDoubleSupportTime;
    fs.remainTime = this->defaultStepTime;
    fs.stepHeight[swingLeg] = {this->defaultStepHeight,this->defaultStepHeight};
    footstepNodesList.push_back(fs);
  }

  o_footstepNodesList = footstepNodesList;
  return true;
}

bool FootStepGenerator::calcFootSteps(const GaitParam& gaitParam, const double& dt, std::vector<GaitParam::FootStepNodes>& o_footstepNodesList) const{


  return true;
}
