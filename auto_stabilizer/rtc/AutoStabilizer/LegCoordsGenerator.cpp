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
  cnoid::Vector3 zmp;
  if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    zmp = 0.5 * (rlegCoords.translation() + rlegCoords.linear()*gaitParam.copOffset[RLEG].value()) + 0.5 * (llegCoords.translation() + llegCoords.linear()*gaitParam.copOffset[LLEG].value());
  }else if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG]){
    zmp = rlegCoords.translation() + rlegCoords.linear()*gaitParam.copOffset[RLEG].value();
  }else{
    zmp = llegCoords.translation() + llegCoords.linear()*gaitParam.copOffset[LLEG].value();
  }
  refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(zmp,zmp,0.0));

  o_refZmpTraj = refZmpTraj;
  o_genCoords = genCoords;
}

void LegCoordsGenerator::calcLegCoords(const GaitParam& gaitParam, double dt, bool useActStates,
                                       std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> >& o_refZmpTraj, std::vector<cpp_filters::TwoPointInterpolatorSE3>& o_genCoords, std::vector<GaitParam::SwingState_enum>& o_swingState, std::vector<bool>& o_isLandingGainPhase) const{
  // swing期は、remainTime - supportTime - delayTimeOffset後にdstCoordsに到達するようなantececdent軌道を生成し(genCoords.getGoal()の値)、その軌道にdelayTimeOffset遅れで滑らかに追従するような軌道(genCoords.value()の値)を生成する.
  //   rectangle以外の軌道タイプや跳躍についてはひとまず考えない TODO
  //   srcCoordsとdstCoordsを結ぶ軌道を生成する. srcCoordsの高さ+[0]とdstCoordsの高さ+[1]の高い方(heightとおく)に上げるようなrectangle軌道を生成する
  // support期は、現FootStepNodesの終了時にdstCoordsに到達するような軌道を線形補間によって生成する.

  // refZmpTrajを更新し進める
  std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> > refZmpTraj = gaitParam.refZmpTraj;
  {
    cnoid::Vector3 refZmp = refZmpTraj[0].getStart(); // for文中の現在のrefzmp
    refZmpTraj.clear();
    // footstepNodesListのサイズが1, footstepNodesList[0].remainTimeが0のときに、copOffsetのパラメータが滑らかに変更になる場合がある. それに対応できるように
    for(int i=0;i<gaitParam.footstepNodesList.size();i++){
      cnoid::Vector3 zmpGoalPos;

      if(gaitParam.footstepNodesList[i].endRefZmpState == GaitParam::FootStepNodes::refZmpState_enum::RLEG){
        cnoid::Position rlegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[RLEG]; // このfootstepNode終了時にdstCoordsに行くように線形補間
        zmpGoalPos = rlegGoalCoords.translation() + rlegGoalCoords.linear()*gaitParam.copOffset[RLEG].value();
      }else if(gaitParam.footstepNodesList[i].endRefZmpState == GaitParam::FootStepNodes::refZmpState_enum::LLEG){
        cnoid::Position llegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[LLEG]; // このfootstepNode終了時にdstCoordsに行くように線形補間
        zmpGoalPos = llegGoalCoords.translation() + llegGoalCoords.linear()*gaitParam.copOffset[LLEG].value();
      }else{ //gaitParam.footstepNodesList[i].endRefZmpState == GaitParam::FootStepNodes::refZmpState_enum::MIDDLE
        cnoid::Position rlegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[RLEG];
        cnoid::Position llegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[LLEG];
        cnoid::Vector3 rlegCOP = rlegGoalCoords.translation() + rlegGoalCoords.linear()*gaitParam.copOffset[RLEG].value();
        cnoid::Vector3 llegCOP = llegGoalCoords.translation() + llegGoalCoords.linear()*gaitParam.copOffset[LLEG].value();
        zmpGoalPos = 0.5 * rlegCOP + 0.5 * llegCOP;
      }
      refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(refZmp,zmpGoalPos,std::max(gaitParam.footstepNodesList[i].remainTime, dt)));
      refZmp = zmpGoalPos;

      if(i >= this->previewStepNum - 1){
        // 片足支持期で終わるのではなく、両足支持期のrefZmpの位置まで予見した方が性能が良い
        if(/*NOT*/!(((gaitParam.footstepNodesList[i].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[i].isSupportPhase[LLEG]) || (!gaitParam.footstepNodesList[i].isSupportPhase[RLEG] && gaitParam.footstepNodesList[i].isSupportPhase[LLEG])) && // 片足支持期
                    ((!(i==gaitParam.footstepNodesList.size()-1)) && gaitParam.footstepNodesList[i+1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[i+1].isSupportPhase[LLEG])) // 次が両足支持期
           ) break;
      }
    }

    // footGuidedBalanceTime[s]に満たない場合、満たないぶんだけ末尾に加える. そうしないと終端条件が厳しすぎる. 一方で、常に末尾にfootGuidedBalanceTime[s]だけ加えると、終端条件がゆるすぎて重心を動かすのが遅すぎる?
    double totalTime = 0;
    for(int i=0;i<refZmpTraj.size();i++) totalTime += refZmpTraj[i].getTime();
    if(totalTime < this->footGuidedBalanceTime){
      refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(refZmp,refZmp, std::max(this->footGuidedBalanceTime - totalTime, dt)));
    }

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
  std::vector<GaitParam::SwingState_enum> swingState = gaitParam.swingState;
  std::vector<bool> isLandingGainPhase = gaitParam.isLandingGainPhase;
  for(int leg=0;leg<NUM_LEGS;leg++){
    if(gaitParam.footstepNodesList[0].stopCurrentPosition[leg]){ // for early touch down. 今の位置に止める
      cnoid::Position tmp = genCoords[leg].value();
      //for(int i=0;i<3;i++){
      //  double tmpvel = std::max(-gaitParam.maxSwingVel[i], std::min(gaitParam.maxSwingVel[i], (gaitParam.footstepNodesList[0].dstCoords[leg].translation()[i] - genCoords[leg].value().translation()[i]) / (gaitParam.footstepNodesList[0].remainTime + dt)));
      //  tmp.translation()[i] += tmpvel * dt;
      //}
      //tmp.translation() = (genCoords[leg].value().translation() * (gaitParam.footstepNodesList[0].remainTime/*-dt*/) + gaitParam.footstepNodesList[0].dstCoords[leg].translation() * dt) / (gaitParam.footstepNodesList[0].remainTime + dt);
      //genCoords[leg].reset(tmp);
      genCoords[leg].reset(genCoords[leg].value());
      continue;
    }
    if(gaitParam.footstepNodesList[0].isSupportPhase[leg]) { // 支持脚
      cnoid::Position nextCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{genCoords[leg].value(),gaitParam.footstepNodesList[0].dstCoords[leg]},
                                                           std::vector<double>{std::max(0.0,gaitParam.footstepNodesList[0].remainTime - dt), dt}); // このfootstepNode終了時にdstCoordsに行くように線形補間
      genCoords[leg].reset(nextCoords);
    }else{ // 遊脚
      //TODO 片足立ち状態を無視 stepHeight[i][0] == stepHeight[i][1]　を仮定
      double height = std::max(gaitParam.srcCoords[leg].translation()[2] + gaitParam.footstepNodesList[0].stepHeight[leg][0],
                               gaitParam.footstepNodesList[0].dstCoords[leg].translation()[2] + gaitParam.footstepNodesList[0].stepHeight[leg][1]); // 足上げ高さ. generate frame
      double airHeight = height - gaitParam.footstepNodesList[0].stepHeight[leg][1] * 0.7; // 足上げ高さ. generate frame
      double downHeight = height -  gaitParam.footstepNodesList[0].stepHeight[leg][1] * 0.3;

      cnoid::Position srcCoords = gaitParam.srcCoords[leg];
      cnoid::Position dstCoords = gaitParam.footstepNodesList[0].dstCoords[leg];
      dstCoords.translation()[2] += gaitParam.footstepNodesList[0].goalOffset[leg]; //TODO この書き方だとまずい　（maxVelとgoalOffsetVelが違う時）
      cnoid::Position antecedentCoords;
      cnoid::Vector6 antecedentVel;
      genCoords[leg].getGoal(antecedentCoords, antecedentVel);

      cnoid::Vector3 goalPos = antecedentCoords.translation();
      cnoid::Vector6 goalVel = antecedentVel;

      //PHASE移行
      if ((swingState[leg] == GaitParam::SWING_PHASE || swingState[leg] == GaitParam::HEIGHT_FIX_SWING_PHASE) && antecedentCoords.translation()[2] > dstCoords.translation()[2] && gaitParam.footstepNodesList[0].remainTime - gaitParam.delayTimeOffset <= (antecedentCoords.translation()[2] - dstCoords.translation()[2]) / gaitParam.maxSwingVel[2]) {//足下げ始め
        swingState[leg] = GaitParam::DOWN_PHASE;
      }
      if (gaitParam.footstepNodesList[0].remainTime <= gaitParam.delayTimeOffset || (swingState[leg] == GaitParam::DOWN_PHASE && antecedentCoords.translation()[2] < dstCoords.translation()[2])) {
        swingState[leg] = GaitParam::GROUND_PHASE;
      }

      //isLandingGainPhase
      if (gaitParam.footstepNodesList[0].remainTime < 0.08) {
        isLandingGainPhase[leg] = true;
      }

      //Z
      if (swingState[leg] == GaitParam::GROUND_PHASE || swingState[leg] == GaitParam::DOWN_PHASE) { //着地位置がdst
        double tmpvel = 0;
        if (antecedentCoords.translation()[2] > dstCoords.translation()[2]) {
          tmpvel = std::min(gaitParam.maxSwingVel[2]*1.1, std::max(-gaitParam.maxSwingVel[2]*1.1, -(antecedentCoords.translation()[2] - dstCoords.translation()[2]) / std::max(dt, gaitParam.footstepNodesList[0].remainTime - gaitParam.delayTimeOffset)));
        }
        goalPos[2] = goalPos[2] + dt*tmpvel;
      } else { //着地位置がheight
        if (antecedentCoords.translation()[2] > height) {
          goalPos[2] = goalPos[2] - dt*gaitParam.maxSwingVel[2];
        } else if (antecedentCoords.translation()[2] < height) {
          goalPos[2] = goalPos[2] + dt*gaitParam.maxSwingVel[2];
        }
        if (std::abs(antecedentCoords.translation()[2] - height) < dt*gaitParam.maxSwingVel[2]) {
          goalPos[2] = height;
        }
      }

      //ZVel
      if (swingState[leg] == GaitParam::SWING_PHASE || swingState[leg] == GaitParam::HEIGHT_FIX_SWING_PHASE) {
        goalVel = (cnoid::Vector6() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished(); // pはgenerate frame. RはgoalCoords frame.
      } else if (swingState[leg] == GaitParam::DOWN_PHASE){
        //goalVel = (cnoid::Vector6() << 0.0, 0.0, (gaitParam.footstepNodesList[0].remainTime - gaitParam.delayTimeOffset /*-dt*/) * antecedentVel[2] + dt * (-gaitParam.footstepNodesList[0].touchVel[leg]), 0.0, 0.0, 0.0).finished(); // pはgenerate frame. RはgoalCoords frame.
        goalVel = (cnoid::Vector6() << 0.0, 0.0, 0.9 * antecedentVel[2] + 0.1 * (-gaitParam.footstepNodesList[0].touchVel[leg]), 0.0, 0.0, 0.0).finished();
      } else {
        goalVel = (cnoid::Vector6() << 0.0, 0.0, -gaitParam.footstepNodesList[0].touchVel[leg], 0.0, 0.0, 0.0).finished(); // pはgenerate frame. RはgoalCoords frame.
      }

      //XY
      if (swingState[leg] == GaitParam::GROUND_PHASE) {
        //GROUND_PHASEではXY方向には動かない
      } else if (swingState[leg] == GaitParam::SWING_PHASE && antecedentCoords.translation()[2] < airHeight) {
        //SWING_PHASEでairheightを下回っている時はXY方向には動かない
      } else {
        if (gaitParam.footstepNodesList[0].remainTime - (downHeight - dstCoords.translation()[2]) / gaitParam.maxSwingVel[2] - gaitParam.delayTimeOffset > 0) { //通常移動
          cnoid::Vector3 tmpvel = cnoid::Vector3::Zero();
          for (int j = 0; j < 2; j++) {
            tmpvel[j] = std::min(gaitParam.maxSwingVel[j], std::max(-gaitParam.maxSwingVel[j], (dstCoords.translation() - antecedentCoords.translation())[j] / (gaitParam.footstepNodesList[0].remainTime - (downHeight - dstCoords.translation()[2]) / gaitParam.maxSwingVel[2] - gaitParam.delayTimeOffset)));
          }
          goalPos.head(2) = goalPos.head(2) + dt*tmpvel.head(2);
        } else {//通常移動後、着地位置修正のみ
          for (int j = 0; j < 2; j++) {
            if (antecedentCoords.translation()[j] > dstCoords.translation()[j]) {
              goalPos[j] = goalPos[j] - dt*gaitParam.maxSwingVel[j];
            } else if (antecedentCoords.translation()[j] < dstCoords.translation()[j]) {
              goalPos[j] = goalPos[j] + dt*gaitParam.maxSwingVel[j];
            }
            if (std::abs(antecedentCoords.translation()[j] - dstCoords.translation()[j]) < dt*gaitParam.maxSwingVel[j]) {
              goalPos[j] = dstCoords.translation()[j];
            }
          }
        }
      }

      //補間を行う
      cnoid::Position nextCoords;
      nextCoords.translation() = goalPos;
      nextCoords.linear() = mathutil::calcMidRot(std::vector<cnoid::Matrix3>{antecedentCoords.linear(),dstCoords.linear()},
                                                 std::vector<double>{std::max(0.0,gaitParam.footstepNodesList[0].remainTime - gaitParam.delayTimeOffset - dt), dt}); // dstCoordsについたときにdstCoordsの傾きになるように線形補間
      //genCoords[leg].setGoal(nextCoords, goalVel, std::min(gaitParam.footstepNodesList[0].remainTime, gaitParam.delayTimeOffset));
      genCoords[leg].setGoal(nextCoords, goalVel, std::max(dt, std::min(gaitParam.footstepNodesList[0].remainTime - 0.020/*遊脚が追従するまでの時間*/, gaitParam.delayTimeOffset)));
      genCoords[leg].interpolate(dt);

    }
  }

  o_refZmpTraj = refZmpTraj;
  o_genCoords = genCoords;
  o_swingState = swingState;
  o_isLandingGainPhase = isLandingGainPhase;
}

void LegCoordsGenerator::calcCOMCoords(const GaitParam& gaitParam, double dt, cnoid::Vector3& o_genNextCog, cnoid::Vector3& o_genNextCogVel, cnoid::Vector3& o_genNextCogAcc) const{
  cnoid::Vector3 genZmp;
  if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] || gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    cnoid::Vector3 genDCM = gaitParam.genCog + gaitParam.genCogVel / gaitParam.omega;
    genZmp = footguidedcontroller::calcFootGuidedControl(gaitParam.omega,gaitParam.l,genDCM,gaitParam.refZmpTraj);
    if(genZmp[2] >= gaitParam.genCog[2]) genZmp = gaitParam.genCog; // 下向きの力は受けられないので
    else{
      cnoid::Vector3 genZmpOrg = genZmp;
      // truncate zmp inside polygon.
      std::vector<cnoid::Vector3> vertices; // generate frame. 支持点の集合
      for(int i=0;i<NUM_LEGS;i++){
        if(!gaitParam.footstepNodesList[0].isSupportPhase[i]) continue;
        for(int j=0;j<gaitParam.legHull[i].size();j++){
          cnoid::Vector3 p = gaitParam.genCoords[i].value()*gaitParam.legHull[i][j];
          if(p[2] > gaitParam.actCog[2] - 1e-2) p[2] = gaitParam.actCog[2] - 1e-2; // 重心よりも支持点が高いと射影が破綻するので
          vertices.push_back(p);
        }
      }
      genZmp = mathutil::calcInsidePointOfPolygon3D(genZmp,vertices,gaitParam.genCog - cnoid::Vector3(gaitParam.l[0],gaitParam.l[1], 0.0));
      //zmpがpolygon外に出たとしてもcogを進行方向に少しでもいいから動かす. そうしないとcogが無限遠に発散する恐れあり.
      for(int i=0;i<2;i++){
        if((genZmpOrg[i]-gaitParam.genCog[i]+gaitParam.l[i]) > 0.001){
          if((genZmp[i]-gaitParam.genCog[i]+gaitParam.l[i]) < 0.001) genZmp[i] = gaitParam.genCog[i] - gaitParam.l[i] + 0.001;
        }else if((genZmpOrg[i]-gaitParam.genCog[i]+gaitParam.l[i]) < -0.001){
          if((genZmp[i]-gaitParam.genCog[i]+gaitParam.l[i]) > -0.001) genZmp[i] = gaitParam.genCog[i] - gaitParam.l[i] - 0.001;
        }
      }
      // TODO 角運動量オフセット
    }
  }else{ // 跳躍期
    genZmp = gaitParam.genCog;
  }
  cnoid::Vector3 genNextCog,genNextCogVel,genNextCogAcc,genNextForce;
  footguidedcontroller::updateState(gaitParam.omega,gaitParam.l,gaitParam.genCog,gaitParam.genCogVel,genZmp,gaitParam.genRobot->mass(),dt,
                                    genNextCog, genNextCogVel, genNextCogAcc, genNextForce);
  o_genNextCog = genNextCog;
  o_genNextCogVel = genNextCogVel;
  o_genNextCogAcc = genNextCogAcc;

  return;
}
