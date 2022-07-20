#ifndef LEGCOORDSGENERATOR_H
#define LEGCOORDSGENERATOR_H

#include <cnoid/EigenTypes>
#include <vector>
#include <cpp_filters/TwoPointInterpolator.h>

namespace legcoordsgenerator{
  class GaitParam {
  public:
    class FootStepNodes {
    public:
      std::vector<cnoid::Position> dstCoords; // 要素数2. rleg: 0. lleg: 1. generate frame. 終了時の位置
      std::vector<double> supportTime; // 要素数2. rleg: 0. lleg: 1. remainTimeがこの値以下なら、support期. ずっとsupport期の場合はinfinity(remainTimeが後から長くなることがあるので), ずっとswing期の場合は0にする. swing期の間に、curCoordsからdstCoordsに移動しきるようにswing軌道が生成される. support期のときは、curCoordsからdstCoordsまで直線的に補間がなされる
      double remainTime; // step time

      // 遊脚軌道用パラメータ
      std::vector<double> stepHeight; // 要素数2. rleg: 0. lleg: 1. swing期には、srcCoordsとdstCoordsの高い方よりもさらにstepHeightだけ高い位置に上げるような軌道を生成する
    };
    std::vector<FootStepNodes> footstepNodesList; // 要素数1以上. 0番目が現在の状態. 末尾の要素以降は、末尾の状態がずっと続くとして扱われる.
    std::vector<cnoid::Position> srcCoords; // 要素数2. rleg: 0. lleg: 1. generate frame. footstepNodesList[0]開始時の位置を保持する. 基本的にはfootstepNodesList[-1]のdstCoordsと同じ
    std::vector<cpp_filters::TwoPointInterpolatorSE3> genCoords; // 要素数2. rleg: 0. lleg: 1. generate frame. 現在の位置
    std::vector<std::vector<cnoid::Vector2> > legPolygon; // 要素数2. rleg: 0. lleg: 1. leg frame.
    std::vector<cnoid::Vector3> copOffset; // 要素数2. rleg: 0. lleg: 1. leg frame. 足裏COPの目標位置. 幾何的な位置はcopOffset無しで考えるが、目標COPを考えるときはcopOffsetを考慮する
    cnoid::Vector3 refZmp; // generate frame. footstepNodesListを単純に線形補間して計算される現在の目標zmp
    cnoid::Vector3 actCog; // generate frame.  現在のCOM
    cnoid::Vector3 actCogVel;  // generate frame.  現在のCOM速度
    cnoid::Vector3 genCog; // generate frame.  現在のCOM
    cnoid::Vector3 genCogVel;  // generate frame.  現在のCOM速度
    double dz; // generate frame. 支持脚からのCogの目標高さ. 0より大きい

    // 遊脚軌道用パラメータ
    double delayTimeOffset; // 0以上. swing期は、remainTime - supportTime - delayTimeOffset後にdstCoordsに到達するようなrectangle軌道を生成し、その軌道にdelayTimeOffset遅れで滑らかに追従するような軌道を生成する
    double touchVel; // 0より大きい. 足を下ろすときの速さ
    //cnoid::Vector3 goal_off; // TODO
  };

  void calcLegCoords(GaitParam& gaitParam, double dt);

  void calcCOMZMPCoords(const GaitParam& gaitParam, double dt, double g, double mass, cnoid::Vector3& genNextCog, cnoid::Vector3& genNextCogVel);

  bool calcNextCoords(GaitParam& gaitParam, double dt, double g, double mass);
}

#endif
