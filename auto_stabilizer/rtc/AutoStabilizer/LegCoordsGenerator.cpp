#include "LegCoordsGenerator.h"
#include "MathUtil.h"

#define DEBUG true

void LegCoordsGenerator::initLegCoords(const GaitParam& gaitParam,
                                       std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> >& o_refZmpTraj, std::vector<cpp_filters::TwoPointInterpolatorSE3>& o_genCoords) const{
  std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> > refZmpTraj;
  std::vector<cpp_filters::TwoPointInterpolatorSE3> genCoords;

  cnoid::Position rlegCoords = gaitParam.footstepNodesList[0].dstCoords[RLEG];
  cnoid::Position llegCoords = gaitParam.footstepNodesList[0].dstCoords[LLEG];

  genCoords.emplace_back(rlegCoords, cnoid::Vector6::Zero(), cnoid::Vector6::Zero(), cpp_filters::HOFFARBIB);
  genCoords.emplace_back(llegCoords, cnoid::Vector6::Zero(), cnoid::Vector6::Zero(), cpp_filters::HOFFARBIB);
  cnoid::Vector3 zmp = 0.5 * (rlegCoords.translation() + rlegCoords.linear()*gaitParam.copOffset[RLEG]) + 0.5 * (llegCoords.translation() + llegCoords.linear()*gaitParam.copOffset[LLEG]);
  refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(zmp,zmp,0.0));

  o_refZmpTraj = refZmpTraj;
  o_genCoords = genCoords;
}

void LegCoordsGenerator::calcLegCoords(const GaitParam& gaitParam, double dt, bool useActStates,
                                       std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> >& o_refZmpTraj, std::vector<cpp_filters::TwoPointInterpolatorSE3>& o_genCoords, cpp_filters::TwoPointInterpolatorSE3& o_footMidCoords, std::vector<GaitParam::FootStepNodes::SwingState_enum>& o_swingState) const{
  // swing期は、remainTime - supportTime - delayTimeOffset後にdstCoordsに到達するようなantececdent軌道を生成し(genCoords.getGoal()の値)、その軌道にdelayTimeOffset遅れで滑らかに追従するような軌道(genCoords.value()の値)を生成する.
  //   rectangle以外の軌道タイプや跳躍についてはひとまず考えない TODO
  //   srcCoordsとdstCoordsを結ぶ軌道を生成する. srcCoordsの高さ+[0]とdstCoordsの高さ+[1]の高い方(heightとおく)に上げるようなrectangle軌道を生成する
  //     dstに到達するまでの時間が(height-dstCoords)/touchVel以下の場合、dstまで直線で移動する
  //     そうで無い場合、今の位置がheight - eps(ひとまず0)よりも低ければ、heightの高さまで上げてからdstCoordsの上空(XYと回転はdstCoordsと同じ)まで直線で移動する軌道を,残りの自由に使える時間で実行する
  //                     今の位置がheight - eps(ひとまず0)以上であれば、dstCoordsの上空(XYと回転はdstCoordsと同じ)まで直線で移動する軌道を,残りの自由に使える時間で実行する
  //                     両者はチャタリングしそうだが、delayTimeOffset遅れで滑らかに追従するので大丈夫
  // support期は、現FootStepNodesの終了時にdstCoordsに到達するような軌道を線形補間によって生成する.

  // refZmpTrajを更新し進める
  std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> > refZmpTraj = gaitParam.refZmpTraj;
  {
    std::vector<cnoid::Position> stepCoords{gaitParam.genCoords[RLEG].value(),gaitParam.genCoords[LLEG].value()}; // for文中の現在の脚の位置
    cnoid::Vector3 refZmp = refZmpTraj[0].getStart(); // for文中の現在のrefzmp
    refZmpTraj.clear();
    for(int i=0;i<gaitParam.footstepNodesList.size();i++){
      if(gaitParam.footstepNodesList[i].remainTime == 0) continue;

      // 跳躍についてはひとまず考えない TODO
      // if(!gaitParam.footstepNodesList[i].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[i].isSupportPhase[LLEG])

      if(!gaitParam.footstepNodesList[i].isSupportPhase[RLEG] && gaitParam.footstepNodesList[i].isSupportPhase[LLEG]){// 右脚がswing. refzmpは左脚の位置
        cnoid::Position llegStartCoords = stepCoords[LLEG];
        cnoid::Position llegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[LLEG]; // このfootstepNode終了時にdstCoordsに行くように線形補間
        cnoid::Vector3 zmpStartPos = llegStartCoords.translation() + llegStartCoords.linear()*gaitParam.copOffset[LLEG];
        cnoid::Vector3 zmpGoalPos = llegGoalCoords.translation() + llegGoalCoords.linear()*gaitParam.copOffset[LLEG];
        refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(zmpStartPos,zmpGoalPos,gaitParam.footstepNodesList[i].remainTime));
        stepCoords[RLEG] = gaitParam.footstepNodesList[i].dstCoords[RLEG];
        stepCoords[LLEG] = llegGoalCoords;
        refZmp = zmpGoalPos;
      }else if(gaitParam.footstepNodesList[i].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[i].isSupportPhase[LLEG]){ // 左脚がswing. refzmpは右脚の位置
        cnoid::Position rlegStartCoords = stepCoords[RLEG];
        cnoid::Position rlegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[RLEG]; // このfootstepNode終了時にdstCoordsに行くように線形補間
        cnoid::Vector3 zmpStartPos = rlegStartCoords.translation() + rlegStartCoords.linear()*gaitParam.copOffset[RLEG];
        cnoid::Vector3 zmpGoalPos = rlegGoalCoords.translation() + rlegGoalCoords.linear()*gaitParam.copOffset[RLEG];
        refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(zmpStartPos,zmpGoalPos,gaitParam.footstepNodesList[i].remainTime));
        stepCoords[RLEG] = rlegGoalCoords;
        stepCoords[LLEG] = gaitParam.footstepNodesList[i].dstCoords[LLEG];
        refZmp = zmpGoalPos;
      }else{ // 両脚がsupport. refzmpは一つ前の区間と一つ後の区間を線形につなぐ
        cnoid::Position rlegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[RLEG];
        cnoid::Position llegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[LLEG];
        cnoid::Vector3 rlegCOP = rlegGoalCoords.translation() + rlegGoalCoords.linear()*gaitParam.copOffset[RLEG];
        cnoid::Vector3 llegCOP = llegGoalCoords.translation() + llegGoalCoords.linear()*gaitParam.copOffset[LLEG];
        cnoid::Vector3 zmpStartPos = refZmp;
        cnoid::Vector3 zmpGoalPos;
        if(i==gaitParam.footstepNodesList.size()-1 || //末尾. 以降は末尾の状態がずっと続くとして扱うので、refzmpは両足の中心
           gaitParam.footstepNodesList[i+1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[i+1].isSupportPhase[LLEG] // 次も両脚支持. refzmpは両足の中心
           ){
          zmpGoalPos = 0.5 * rlegCOP + 0.5 * llegCOP;
        }else if(!gaitParam.footstepNodesList[i].isSupportPhase[RLEG] && gaitParam.footstepNodesList[i].isSupportPhase[LLEG]){ // 次は右脚がswing
          zmpGoalPos = llegCOP;
        }else if(gaitParam.footstepNodesList[i].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[i].isSupportPhase[LLEG]){ // 次は左脚がswing
          zmpGoalPos = rlegCOP;
        }else{// 次は跳躍. TODO
          zmpGoalPos = 0.5 * rlegCOP + 0.5 * llegCOP;
        }
        refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(zmpStartPos,zmpGoalPos,gaitParam.footstepNodesList[i].remainTime));
        stepCoords[RLEG] = rlegGoalCoords;
        stepCoords[LLEG] = llegGoalCoords;
        refZmp = zmpGoalPos;
      }
    }
    // 末尾に1sぶん加える. そうしないと終端条件が厳しすぎる
    refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(refZmp,refZmp, 1.0));
    // dtだけ進める
    if(refZmpTraj[0].getTime() <= dt){
      if(refZmpTraj.size() > 1) refZmpTraj.erase(refZmpTraj.begin());
      else refZmpTraj[0] = footguidedcontroller::LinearTrajectory<cnoid::Vector3>(refZmpTraj[0].getGoal(),refZmpTraj[0].getGoal(),0.0);
    }else{
      refZmpTraj[0] = footguidedcontroller::LinearTrajectory<cnoid::Vector3>(refZmpTraj[0].getStart()+refZmpTraj[0].getSlope()*dt,refZmpTraj[0].getGoal(),refZmpTraj[0].getTime()-dt);
    }
  }

  // genCoordsを進める
  std::vector<cpp_filters::TwoPointInterpolatorSE3> genCoords = gaitParam.genCoords;
  std::vector<GaitParam::FootStepNodes::SwingState_enum> swingState = gaitParam.footstepNodesList[0].swingState;
  for(int i=0;i<NUM_LEGS;i++){
    if(gaitParam.footstepNodesList[0].isSupportPhase[i]) { // 支持脚
      cnoid::Position nextCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{genCoords[i].value(),gaitParam.footstepNodesList[0].dstCoords[i]},
                                                           std::vector<double>{std::max(0.0,gaitParam.footstepNodesList[0].remainTime - dt), dt}); // このfootstepNode終了時にdstCoordsに行くように線形補間
      genCoords[i].reset(nextCoords);
    }else{ // 遊脚
      double swingTime = gaitParam.footstepNodesList[0].remainTime;
      double height = std::max(gaitParam.srcCoords[i].translation()[2] + gaitParam.footstepNodesList[0].stepHeight[i][0],
                               gaitParam.footstepNodesList[0].dstCoords[i].translation()[2] + gaitParam.footstepNodesList[0].stepHeight[i][1]); // 足上げ高さ. generate frame
      cnoid::Position srcCoords = gaitParam.srcCoords[i];
      cnoid::Position dstCoords = gaitParam.footstepNodesList[0].dstCoords[i];
      dstCoords.translation()[2] += gaitParam.footstepNodesList[0].goalOffset[i];
      cnoid::Position antecedentCoords = genCoords[i].getGoal(); // 今のantecedent軌道の位置
      double touchDownTime = (antecedentCoords.translation()[2] - dstCoords.translation()[2]) / this->touchVel; // 地面につくのに要する時間
      // phase transition
      if(swingState[i] == GaitParam::FootStepNodes::LIFT_PHASE){
        if(swingTime <= this->delayTimeOffset) swingState[i] = GaitParam::FootStepNodes::DOWN_PHASE;
        else if(antecedentCoords.translation()[2] >= height) swingState[i] = GaitParam::FootStepNodes::SWING_PHASE;
      }else if(swingState[i] == GaitParam::FootStepNodes::SWING_PHASE){
        if(swingTime <= this->delayTimeOffset) swingState[i] = GaitParam::FootStepNodes::DOWN_PHASE;
        else if(antecedentCoords.translation()[2] < dstCoords.translation()[2]) swingState[i] = GaitParam::FootStepNodes::LIFT_PHASE;
        else if((antecedentCoords.translation() - dstCoords.translation()).head<2>().norm() <= 1e-3 &&
                swingTime - this->delayTimeOffset <= touchDownTime) swingState[i] = GaitParam::FootStepNodes::DOWN_PHASE;
      }else{
        // 一度DOWN_PHASEになったら別のPHASEになることはない
      }
      if(swingState[i] == GaitParam::FootStepNodes::LIFT_PHASE){
        double length0 = std::abs(height - antecedentCoords.translation()[2]);
        double length1 = (dstCoords.translation() - antecedentCoords.translation()).head<2>().norm();
        double length2 = std::abs(height - dstCoords.translation()[2]);
        double totalLength = length0 + length1 + length2 * this->finalDistanceWeight;
        double ratio = std::min(dt / (swingTime - this->delayTimeOffset), 1.0); // LIFT_PHASEのとき必ずswingTime - this->delayTimeOffset>0
        cnoid::Vector3 goal;
        if(ratio * totalLength < length0){
          goal = antecedentCoords.translation();
          goal[2] += ratio * totalLength;
        }else if(ratio * totalLength < length0 + length1){
          goal = antecedentCoords.translation();
          goal[2] = height;
          goal.head<2>() += (dstCoords.translation() - antecedentCoords.translation()).head<2>().normalized() * (ratio * totalLength - length0);
        }else{
          goal = dstCoords.translation();
          goal[2] = height - (ratio * totalLength - length0 - length1) / this->finalDistanceWeight;
        }
        cnoid::Position nextCoords;
        nextCoords.translation() = goal;
        nextCoords.linear() = mathutil::calcMidRot(std::vector<cnoid::Matrix3>{antecedentCoords.linear(),dstCoords.linear()},
                                                   std::vector<double>{std::max(0.0,swingTime - this->delayTimeOffset - dt), dt}); // dstCoordsの傾きになるように線形補間
        genCoords[i].setGoal(nextCoords, this->delayTimeOffset);
        genCoords[i].interpolate(dt);
      }else if(swingState[i] == GaitParam::FootStepNodes::SWING_PHASE){
        cnoid::Vector3 viaPos = dstCoords.translation();
        viaPos[2] = height;
        double length1 = (viaPos - antecedentCoords.translation()).norm();
        double length2 = std::abs(height - dstCoords.translation()[2]);
        double totalLength = length1 + length2 * this->finalDistanceWeight;
        double ratio = std::min(dt / (swingTime - this->delayTimeOffset), 1.0); // SWING_PHASEのとき必ずswingTime - this->delayTimeOffset>0
        cnoid::Vector3 goal;
        if(ratio * totalLength < length1){
          goal = antecedentCoords.translation();
          goal += (viaPos - antecedentCoords.translation()).normalized() * (ratio * totalLength);
        }else{
          goal = dstCoords.translation();
          goal[2] = height - (ratio * totalLength - length1) / this->finalDistanceWeight;
        }
        cnoid::Position nextCoords;
        nextCoords.translation() = goal;
        nextCoords.linear() = mathutil::calcMidRot(std::vector<cnoid::Matrix3>{antecedentCoords.linear(),dstCoords.linear()},
                                                   std::vector<double>{std::max(0.0,swingTime - this->delayTimeOffset - dt), dt}); // dstCoordsの傾きになるように線形補間
        genCoords[i].setGoal(nextCoords, this->delayTimeOffset);
        genCoords[i].interpolate(dt);
      }else{ // DOWN_PHASE
        if(swingTime <= this->delayTimeOffset){
          genCoords[i].setGoal(dstCoords, swingTime);
          genCoords[i].interpolate(dt);
        }else{
          double ratio = std::min(dt / (swingTime - this->delayTimeOffset), 1.0); // swingTime - this->delayTimeOffset>0
          cnoid::Vector3 goal = antecedentCoords.translation() + (dstCoords.translation() - antecedentCoords.translation()) * ratio;
          cnoid::Position nextCoords;
          nextCoords.translation() = goal;
          nextCoords.linear() = mathutil::calcMidRot(std::vector<cnoid::Matrix3>{antecedentCoords.linear(),dstCoords.linear()},
                                                     std::vector<double>{std::max(0.0,swingTime - this->delayTimeOffset - dt), dt}); // dstCoordsの傾きになるように線形補間
          genCoords[i].setGoal(nextCoords, this->delayTimeOffset);
          genCoords[i].interpolate(dt);
        }
      }
    }
  }

  // footMidCoordsを進める
  cpp_filters::TwoPointInterpolatorSE3 footMidCoords = gaitParam.footMidCoords;
  {
    cnoid::Position rleg = mathutil::orientCoordToAxis(gaitParam.footstepNodesList[0].dstCoords[RLEG], cnoid::Vector3::UnitZ());
    rleg.translation() -= rleg.linear() * gaitParam.defaultTranslatePos[RLEG];
    cnoid::Position lleg = mathutil::orientCoordToAxis(gaitParam.footstepNodesList[0].dstCoords[LLEG], cnoid::Vector3::UnitZ());
    lleg.translation() -= lleg.linear() * gaitParam.defaultTranslatePos[LLEG];
    if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){ // 両足支持
      cnoid::Position midCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{rleg, lleg}, std::vector<double>{1.0, 1.0});
      footMidCoords.setGoal(midCoords, gaitParam.footstepNodesList[0].remainTime);
    }else if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){ // 右足支持
      if(gaitParam.footstepNodesList.size() > 1 &&
         (gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[1].isSupportPhase[LLEG]) // 次が両足支持
         ){
        rleg = mathutil::orientCoordToAxis(gaitParam.footstepNodesList[1].dstCoords[RLEG], cnoid::Vector3::UnitZ());
        rleg.translation() -= rleg.linear() * gaitParam.defaultTranslatePos[RLEG];
        lleg = mathutil::orientCoordToAxis(gaitParam.footstepNodesList[1].dstCoords[LLEG], cnoid::Vector3::UnitZ());
        lleg.translation() -= lleg.linear() * gaitParam.defaultTranslatePos[LLEG];
        cnoid::Position midCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{rleg, lleg}, std::vector<double>{1.0, 1.0});
        footMidCoords.setGoal(midCoords, gaitParam.footstepNodesList[0].remainTime + gaitParam.footstepNodesList[1].remainTime);
      }else{
        cnoid::Position midCoords = rleg;
        footMidCoords.setGoal(midCoords, gaitParam.footstepNodesList[0].remainTime);
      }
    }else if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){ // 左足支持
      if(gaitParam.footstepNodesList.size() > 1 &&
         (gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[1].isSupportPhase[LLEG]) // 次が両足支持
         ){
        rleg = mathutil::orientCoordToAxis(gaitParam.footstepNodesList[1].dstCoords[RLEG], cnoid::Vector3::UnitZ());
        rleg.translation() -= rleg.linear() * gaitParam.defaultTranslatePos[RLEG];
        lleg = mathutil::orientCoordToAxis(gaitParam.footstepNodesList[1].dstCoords[LLEG], cnoid::Vector3::UnitZ());
        lleg.translation() -= lleg.linear() * gaitParam.defaultTranslatePos[LLEG];
        cnoid::Position midCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{rleg, lleg}, std::vector<double>{1.0, 1.0});
        footMidCoords.setGoal(midCoords, gaitParam.footstepNodesList[0].remainTime + gaitParam.footstepNodesList[1].remainTime);
      }else{
        cnoid::Position midCoords = lleg;
        footMidCoords.setGoal(midCoords, gaitParam.footstepNodesList[0].remainTime);
      }
    }else{ // 滞空期 TODO
      if(gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[1].isSupportPhase[LLEG]) { // 次が右足支持
        cnoid::Position midCoords = rleg;
        footMidCoords.setGoal(midCoords, gaitParam.footstepNodesList[0].remainTime);
      }else if(!gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[1].isSupportPhase[LLEG]) { // 次が左足支持
        cnoid::Position midCoords = lleg;
        footMidCoords.setGoal(midCoords, gaitParam.footstepNodesList[0].remainTime);
      }else{
        cnoid::Position midCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{rleg, lleg}, std::vector<double>{1.0, 1.0});
        footMidCoords.setGoal(midCoords, gaitParam.footstepNodesList[0].remainTime);
      }
    }

    footMidCoords.interpolate(dt);
  }

  o_refZmpTraj = refZmpTraj;
  o_genCoords = genCoords;
  o_footMidCoords = footMidCoords;
  o_swingState = swingState;
}

void LegCoordsGenerator::calcCOMCoords(const GaitParam& gaitParam, double dt, double mass, cnoid::Vector3& o_genNextCog, cnoid::Vector3& o_genNextCogVel) const{
  double w = std::sqrt(gaitParam.g/gaitParam.refdz); // TODO refforceZ
  cnoid::Vector3 l = cnoid::Vector3::Zero();
  l[2] = gaitParam.refdz;
  cnoid::Vector3 genZmp;
  if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] || gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    cnoid::Vector3 genDCM = gaitParam.genCog + gaitParam.genCogVel / w;
    genZmp = footguidedcontroller::calcFootGuidedControl(w,l,genDCM,gaitParam.refZmpTraj);
    if(genZmp[2] >= gaitParam.genCog[2]) genZmp = gaitParam.genCog; // 下向きの力は受けられないので
    else{
      cnoid::Vector3 genZmpOrg = genZmp;
      // truncate zmp inside polygon.
      std::vector<cnoid::Vector3> vertices; // generate frame. 支持点の集合
      for(int i=0;i<NUM_LEGS;i++){
        if(!gaitParam.footstepNodesList[0].isSupportPhase[i]) continue;
        for(int j=0;j<gaitParam.legHull[i].size();j++){
          vertices.push_back(gaitParam.genCoords[i].value()*gaitParam.legHull[i][j]);
        }
      }
      genZmp = mathutil::calcInsidePointOfPolygon3D(genZmp,vertices,gaitParam.genCog);
      //zmpがpolygon外に出たとしてもcogを進行方向に少しでもいいから動かす. そうしないとcogが無限遠に発散する恐れあり.
      for(int i=0;i<2;i++){
        if((genZmpOrg[i]-gaitParam.genCog[i]) > 0.001){
          if((genZmp[i]-gaitParam.genCog[i]) < 0.001) genZmp[i] = gaitParam.genCog[i] + 0.001;
        }else if((genZmpOrg[i]-gaitParam.genCog[i]) < -0.001){
          if((genZmp[i]-gaitParam.genCog[i]) > -0.001) genZmp[i] = gaitParam.genCog[i] - 0.001;
        }
      }
      // TODO 角運動量オフセット
    }
  }else{ // 跳躍期
    genZmp = gaitParam.genCog;
  }
  cnoid::Vector3 genNextCog,genNextCogVel,genNextForce;
  footguidedcontroller::updateState(w,l,gaitParam.genCog,gaitParam.genCogVel,genZmp,mass,dt,
                                    genNextCog, genNextCogVel, genNextForce);
  o_genNextCog = genNextCog;
  o_genNextCogVel = genNextCogVel;

  return;
}
