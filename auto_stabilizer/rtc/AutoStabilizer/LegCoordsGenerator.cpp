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
                                       std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> >& o_refZmpTraj, std::vector<cpp_filters::TwoPointInterpolatorSE3>& o_genCoords, std::vector<GaitParam::SwingState_enum>& o_swingState) const{
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

      if(!gaitParam.footstepNodesList[i].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[i].isSupportPhase[LLEG]){
        // 跳躍についてはひとまず考えない TODO
      }else if(!gaitParam.footstepNodesList[i].isSupportPhase[RLEG] && gaitParam.footstepNodesList[i].isSupportPhase[LLEG]){// 右脚がswing. refzmpは左脚の位置
        cnoid::Position llegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[LLEG]; // このfootstepNode終了時にdstCoordsに行くように線形補間
        cnoid::Vector3 zmpGoalPos = llegGoalCoords.translation() + llegGoalCoords.linear()*gaitParam.copOffset[LLEG].value();
        refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(refZmp,zmpGoalPos,std::max(gaitParam.footstepNodesList[i].remainTime, dt)));
        refZmp = zmpGoalPos;
      }else if(gaitParam.footstepNodesList[i].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[i].isSupportPhase[LLEG]){ // 左脚がswing. refzmpは右脚の位置
        cnoid::Position rlegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[RLEG]; // このfootstepNode終了時にdstCoordsに行くように線形補間
        cnoid::Vector3 zmpGoalPos = rlegGoalCoords.translation() + rlegGoalCoords.linear()*gaitParam.copOffset[RLEG].value();
        refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(refZmp,zmpGoalPos,std::max(gaitParam.footstepNodesList[i].remainTime, dt)));
        refZmp = zmpGoalPos;
      }else{ // 両脚がsupport. refzmpは一つ前の区間と一つ後の区間を線形につなぐ
        cnoid::Position rlegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[RLEG];
        cnoid::Position llegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[LLEG];
        cnoid::Vector3 rlegCOP = rlegGoalCoords.translation() + rlegGoalCoords.linear()*gaitParam.copOffset[RLEG].value();
        cnoid::Vector3 llegCOP = llegGoalCoords.translation() + llegGoalCoords.linear()*gaitParam.copOffset[LLEG].value();
        cnoid::Vector3 zmpGoalPos;
        if(i==gaitParam.footstepNodesList.size()-1 || //末尾. 以降は末尾の状態がずっと続くとして扱うので、refzmpは両足の中心
           (gaitParam.footstepNodesList[i+1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[i+1].isSupportPhase[LLEG]) // 次も両脚支持. refzmpは両足の中心
           ){
          zmpGoalPos = 0.5 * rlegCOP + 0.5 * llegCOP;
        }else if(!gaitParam.footstepNodesList[i+1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[i+1].isSupportPhase[LLEG]){ // 次は右脚がswing
          zmpGoalPos = llegCOP;
        }else if(gaitParam.footstepNodesList[i+1].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[i+1].isSupportPhase[LLEG]){ // 次は左脚がswing
          zmpGoalPos = rlegCOP;
        }else{// 次は跳躍. TODO
          zmpGoalPos = 0.5 * rlegCOP + 0.5 * llegCOP;
        }
        refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(refZmp,zmpGoalPos,std::max(gaitParam.footstepNodesList[i].remainTime, dt)));
        refZmp = zmpGoalPos;
      }

      if(i >= this->previewStepNum - 1){
        if(/*NOT*/!(((gaitParam.footstepNodesList[i].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[i].isSupportPhase[LLEG]) || (!gaitParam.footstepNodesList[i].isSupportPhase[RLEG] && gaitParam.footstepNodesList[i].isSupportPhase[LLEG])) && // 片足支持期
                    ((!(i==gaitParam.footstepNodesList.size()-1)) && gaitParam.footstepNodesList[i+1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[i+1].isSupportPhase[LLEG])) // 次が両足支持期
           ) break;
      }
    }

    // dtだけ進める
    if(refZmpTraj[0].getTime() <= dt){
      if(refZmpTraj.size() > 1) refZmpTraj.erase(refZmpTraj.begin());
      else refZmpTraj[0] = footguidedcontroller::LinearTrajectory<cnoid::Vector3>(refZmpTraj[0].getGoal(),refZmpTraj[0].getGoal(),0.0);
    }else{
      refZmpTraj[0] = footguidedcontroller::LinearTrajectory<cnoid::Vector3>(refZmpTraj[0].getStart()+refZmpTraj[0].getSlope()*dt,refZmpTraj[0].getGoal(),refZmpTraj[0].getTime()-dt);
    }

    if(gaitParam.footstepNodesList.size() == 1){
      // footGuidedBalanceTime[s]に満たない場合、満たないぶんだけ末尾に加える. そうしないと終端条件が厳しすぎる. 一方で、常に末尾にfootGuidedBalanceTime[s]だけ加えると、終端条件がゆるすぎて重心を動かすのが遅すぎる.
      double totalTime = 0;
      for(int i=0;i<refZmpTraj.size();i++) totalTime += refZmpTraj[i].getTime();
      if(totalTime < this->footGuidedBalanceTime){
        refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(refZmp,refZmp, std::max(this->footGuidedBalanceTime - totalTime, dt)));
      }
    }
  }

  // genCoordsを進める
  std::vector<cpp_filters::TwoPointInterpolatorSE3> genCoords = gaitParam.genCoords;
  std::vector<GaitParam::SwingState_enum> swingState = gaitParam.swingState;
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
      cnoid::Position dstCoordsWithOffset = gaitParam.footstepNodesList[0].dstCoords[i];
      double offset = 0.0;
      if(gaitParam.footstepNodesList.size() > 1 && gaitParam.footstepNodesList[1].isSupportPhase[i]) offset = this->goalOffset;
      dstCoordsWithOffset.translation()[2] += offset;
      cnoid::Position antecedentCoords = genCoords[i].getGoal(); // 今のantecedent軌道の位置
      double touchDownTime = (antecedentCoords.translation()[2] - dstCoordsWithOffset.translation()[2]) / this->touchVel; // 地面につくのに要する時間
      double goalOffsetTime = std::abs(offset) / this->touchVel; // dstCoordsWithOffsetにつくgoalOffsetTime前に傾きはdstCoordsの姿勢になる. そうしないと、goalOffset<0のときに、目標の傾き(特にyaw旋回)に至るまえに着地してしまう. overwritebleMinTime > goalOffsetTime + delayTimeOffsetにすること

      // phase transition
      if(swingState[i] == GaitParam::LIFT_PHASE){
        if(swingTime <= this->delayTimeOffset) swingState[i] = GaitParam::DOWN_PHASE;
        else if(antecedentCoords.translation()[2] >= height - 1e-3) swingState[i] = GaitParam::SWING_PHASE;
      }else if(swingState[i] == GaitParam::SWING_PHASE){
        if(swingTime <= this->delayTimeOffset) swingState[i] = GaitParam::DOWN_PHASE;
        else if(antecedentCoords.translation()[2] < dstCoords.translation()[2]) swingState[i] = GaitParam::LIFT_PHASE;
        else if((antecedentCoords.translation() - dstCoordsWithOffset.translation()).head<2>().norm() <= 1e-3 &&
                swingTime - this->delayTimeOffset <= touchDownTime) swingState[i] = GaitParam::DOWN_PHASE;
      }else{
        // 一度DOWN_PHASEになったら別のPHASEになることはない
      }

      if(swingState[i] == GaitParam::LIFT_PHASE){
        double length0 = std::abs(height - antecedentCoords.translation()[2]);
        double length1 = (dstCoordsWithOffset.translation() - antecedentCoords.translation()).head<2>().norm();
        double length2 = std::abs(height - dstCoordsWithOffset.translation()[2]);
        double totalLength = length0 + length1 + length2 * this->finalDistanceWeight;
        double ratio = std::min(dt / (swingTime - this->delayTimeOffset), 1.0); // LIFT_PHASEのとき必ずswingTime - this->delayTimeOffset>0
        cnoid::Vector3 goal;
        if(ratio * totalLength < length0){
          goal = antecedentCoords.translation();
          goal[2] += ratio * totalLength;
        }else{
          goal = antecedentCoords.translation();
          goal[2] = height;
        }
        cnoid::Position nextCoords;
        nextCoords.translation() = goal;
        nextCoords.linear() = mathutil::calcMidRot(std::vector<cnoid::Matrix3>{antecedentCoords.linear(),dstCoordsWithOffset.linear()},
                                                   std::vector<double>{std::max(0.0,swingTime - this->delayTimeOffset - goalOffsetTime - dt), dt}); // もとのdstCoordsの高さについたときにdstCoordsの傾きになるように線形補間
        genCoords[i].setGoal(nextCoords, this->delayTimeOffset);
        genCoords[i].interpolate(dt);
      }else if(swingState[i] == GaitParam::SWING_PHASE){
        cnoid::Vector3 viaPos = dstCoordsWithOffset.translation();
        viaPos[2] = height;
        double length1 = (viaPos - antecedentCoords.translation()).norm();
        double length2 = std::abs(height - dstCoordsWithOffset.translation()[2]);
        double totalLength = length1 + length2 * this->finalDistanceWeight;
        double ratio = std::min(dt / (swingTime - this->delayTimeOffset), 1.0); // SWING_PHASEのとき必ずswingTime - this->delayTimeOffset>0
        cnoid::Vector3 goal;
        if(ratio * totalLength < length1){
          goal = antecedentCoords.translation();
          goal += (viaPos - antecedentCoords.translation()).normalized() * (ratio * totalLength);
        }else{
          goal = dstCoordsWithOffset.translation();
          goal[2] = height;
        }
        double dstCoordsRatio = 1.0 - std::abs((offset * this->finalDistanceWeight) / totalLength); // もとのdstCoords (offsetなし)に至るタイミング
        cnoid::Position nextCoords;
        nextCoords.translation() = goal;
        nextCoords.linear() = mathutil::calcMidRot(std::vector<cnoid::Matrix3>{antecedentCoords.linear(),dstCoordsWithOffset.linear()},
                                                   std::vector<double>{std::max(0.0,swingTime - this->delayTimeOffset - goalOffsetTime - dt), dt}); // もとのdstCoordsの高さについたときにdstCoordsの傾きになるように線形補間
        genCoords[i].setGoal(nextCoords, this->delayTimeOffset);
        genCoords[i].interpolate(dt);
      }else{ // DOWN_PHASE
        if(swingTime <= this->delayTimeOffset){
          genCoords[i].setGoal(dstCoordsWithOffset, swingTime);
          genCoords[i].interpolate(dt);
        }else{
          double ratio = std::min(dt / (swingTime - this->delayTimeOffset), 1.0); // swingTime - this->delayTimeOffset>0
          cnoid::Vector3 goal = antecedentCoords.translation() + (dstCoordsWithOffset.translation() - antecedentCoords.translation()) * ratio;
          cnoid::Position nextCoords;
          nextCoords.translation() = goal;
          nextCoords.linear() = mathutil::calcMidRot(std::vector<cnoid::Matrix3>{antecedentCoords.linear(),dstCoordsWithOffset.linear()},
                                                     std::vector<double>{std::max(0.0,swingTime - this->delayTimeOffset - goalOffsetTime - dt), dt}); // もとのdstCoordsの高さについたときにdstCoordsの傾きになるように線形補間
          genCoords[i].setGoal(nextCoords, this->delayTimeOffset);
          genCoords[i].interpolate(dt);
        }
      }
    }
  }

  o_refZmpTraj = refZmpTraj;
  o_genCoords = genCoords;
  o_swingState = swingState;
}

void LegCoordsGenerator::calcCOMCoords(const GaitParam& gaitParam, double dt, cnoid::Vector3& o_genNextCog, cnoid::Vector3& o_genNextCogVel) const{
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
  cnoid::Vector3 genNextCog,genNextCogVel,genNextForce;
  footguidedcontroller::updateState(gaitParam.omega,gaitParam.l,gaitParam.genCog,gaitParam.genCogVel,genZmp,gaitParam.genRobot->mass(),dt,
                                    genNextCog, genNextCogVel, genNextForce);
  o_genNextCog = genNextCog;
  o_genNextCogVel = genNextCogVel;

  return;
}
