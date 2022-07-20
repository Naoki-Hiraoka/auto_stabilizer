#include "LegCoordsGenerator.h"
#include "FootGuidedController.h"
#include "MathUtil.h"

namespace legcoordsgenerator{

  void procGaitParam(const GaitParam& gaitParam, double dt){ // gaitParamをdt進める
    // genCoords
    // remainTime
    // refZmp
  }

  void calcCOMZMPCoords(const GaitParam& gaitParam, double dt, double g, double mass, cnoid::Vector3& genNextCog, cnoid::Vector3& genNextCogVel){
    std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> > refZmpTraj;
    {
      std::vector<cnoid::Position> stepCoords{gaitParam.genCoords[0],gaitParam.genCoords[1]}; // for文中の現在の脚の位置
      cnoid::Vector3 refZmp = gaitParam.refZmp; // for文中の現在のrefzmp
      for(int i=0;i<gaitParam.footstepNodesList.size();i++){
        double remainTime = gaitParam.footstepNodesList[i].remainTime;

        // 跳躍についてはひとまず考えない TODO
        // if(gaitParam.footstepNodesList[i].remainTime > gaitParam.footstepNodesList[i].supportTime[0] &&
        //    gaitParam.footstepNodesList[i].remainTime > gaitParam.footstepNodesList[i].supportTime[1])

        if(remainTime > gaitParam.footstepNodesList[i].supportTime[0] && remainTime <= gaitParam.footstepNodesList[i].supportTime[1]){// 右脚がswing. refzmpは左脚の位置
          double swingTime = remainTime - gaitParam.footstepNodesList[i].supportTime[0];
          cnoid::Position llegStartCoords = stepCoords[1];
          cnoid::Position llegGoalCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{stepCoords[1],gaitParam.footstepNodesList[i].dstCoords[1]},std::vector<double>{swingTime, remainTime-swingTime}); // このfootstepNode終了時にdstCoordsに行くように線形補間
          cnoid::Vector3 zmpStartPos = llegStartCoords.translation() + llegStartCoords.linear()*gaitParam.copOffset[1];
          cnoid::Vector3 zmpGoalPos = llegGoalCoords.translation() + llegGoalCoords.linear()*gaitParam.copOffset[1];
          refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(zmpStartPos,zmpGoalPos,swingTime));
          stepCoords[0] = gaitParam.footstepNodesList[i].dstCoords[0];
          stepCoords[1] = llegGoalCoords;
          refZmp = zmpGoalPos;
          remainTime -= swingTime;
        }else if(gaitParam.footstepNodesList[i].remainTime <= gaitParam.footstepNodesList[i].supportTime[0] && gaitParam.footstepNodesList[i].remainTime > gaitParam.footstepNodesList[i].supportTime[1]){ // 左脚がswing. refzmpは右脚の位置
          double swingTime = remainTime - gaitParam.footstepNodesList[i].supportTime[1];
          cnoid::Position rlegStartCoords = stepCoords[0];
          cnoid::Position rlegGoalCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{stepCoords[0],gaitParam.footstepNodesList[i].dstCoords[0]},std::vector<double>{swingTime, remainTime-swingTime}); // このfootstepNode終了時にdstCoordsに行くように線形補間
          cnoid::Vector3 zmpStartPos = rlegStartCoords.translation() + rlegStartCoords.linear()*gaitParam.copOffset[0];
          cnoid::Vector3 zmpGoalPos = rlegGoalCoords.translation() + rlegGoalCoords.linear()*gaitParam.copOffset[0];
          refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(zmpStartPos,zmpGoalPos,swingTime));
          stepCoords[0] = rlegGoalCoords;
          stepCoords[1] = gaitParam.footstepNodesList[i].dstCoords[1];
          refZmp = zmpGoalPos;
          remainTime -= swingTime;
        }
        if(remainTime <= gaitParam.footstepNodesList[i].supportTime[0] && remainTime <= gaitParam.footstepNodesList[i].supportTime[1]){ // 両脚がsupport. refzmpは一つ前の区間と一つ後の区間を線形につなぐ
          cnoid::Position rlegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[0];
          cnoid::Position llegGoalCoords = gaitParam.footstepNodesList[i].dstCoords[1];
          cnoid::Vector3 rlegCOP = rlegGoalCoords.translation() + rlegGoalCoords.linear()*gaitParam.copOffset[0];
          cnoid::Vector3 llegCOP = llegGoalCoords.translation() + llegGoalCoords.linear()*gaitParam.copOffset[1];
          cnoid::Vector3 zmpStartPos = refZmp;
          cnoid::Vector3 zmpGoalPos;
          if(i==gaitParam.footstepNodesList.size()-1 || //末尾. 以降は末尾の状態がずっと続くとして扱うので、refzmpは両足の中心
             gaitParam.footstepNodesList[i+1].remainTime <= gaitParam.footstepNodesList[i+1].supportTime[0] && gaitParam.footstepNodesList[i+1].remainTime <= gaitParam.footstepNodesList[i+1].supportTime[1] // 次も両脚支持. refzmpは両足の中心
             ){
            zmpGoalPos = 0.5 * rlegCOP + 0.5 * llegCOP;
          }else if(gaitParam.footstepNodesList[i+1].remainTime > gaitParam.footstepNodesList[i+1].supportTime[0] && gaitParam.footstepNodesList[i+1].remainTime <= gaitParam.footstepNodesList[i+1].supportTime[1]){ // 次は右脚がswing
            zmpGoalPos = llegCOP;
          }else if(gaitParam.footstepNodesList[i+1].remainTime <= gaitParam.footstepNodesList[i+1].supportTime[0] && gaitParam.footstepNodesList[i+1].remainTime > gaitParam.footstepNodesList[i+1].supportTime[1]){ // 次は左脚がswing
            zmpGoalPos = rlegCOP;
          }else{// 跳躍についてはひとまず考えない TODO
          }
          refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(zmpStartPos,zmpGoalPos,gaitParam.footstepNodesList[i].remainTime));
          stepCoords[0] = rlegGoalCoords;
          stepCoords[1] = llegGoalCoords;
          refZmp = zmpGoalPos;
          remainTime = 0.0;
        }
      }
    }

    double w = std::sqrt(g/gaitParam.dz);
    cnoid::Vector3 l = cnoid::Vector3::Zero();
    l[2] = gaitParam.dz;
    cnoid::Vector3 genDCM = gaitParam.genCog + w * gaitParam.genCogVel;
    cnoid::Vector3 genZmp = footguidedcontroller::calcFootGuidedControl(w,l,genDCM,refZmpTraj);
    // check zmp in polygon TODO
    cnoid::Vector3 genNextForce;
    footguidedcontroller::updateState(w,l,gaitParam.genCog,gaitParam.genCogVel,genZmp,mass,dt,
                                      genNextCog, genNextCogVel, genNextForce);
    return;
  }

  bool calcNextCoords(GaitParam& gaitParam, double dt, double g, double mass){
    procGaitParam(gaitParam,dt);
    cnoid::Vector3 genNextCog,genNextCogVel;
    calcCOMZMPCoords(gaitParam, dt, g, mass,genNextCog,genNextCogVel);
    gaitParam.genCog = genNextCog;
    gaitParam.genCogVel = genNextCogVel;
    return true;
  }
}
