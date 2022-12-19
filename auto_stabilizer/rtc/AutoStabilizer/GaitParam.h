#ifndef GAITPARAM_H
#define GAITPARAM_H

#include <sys/time.h>
#include <cnoid/EigenTypes>
#include <vector>
#include <limits>
#include <cpp_filters/TwoPointInterpolator.h>
#include <cpp_filters/FirstOrderLowPassFilter.h>
#include <joint_limit_table/JointLimitTable.h>
#include "FootGuidedController.h"

enum leg_enum{RLEG=0, LLEG=1, NUM_LEGS=2};

class GaitParam {
  // このクラスのメンバ変数は、全てfiniteである(nanやinfが無い)ことが仮定されている. 外部から値をセットするときには、finiteでない値を入れないようにすること

  // robotは、rootLinkがFreeJointでなければならない
public:
  // constant parameter
  std::vector<std::string> eeName; // constant. 要素数2以上. 0番目がrleg, 1番目がllegという名前である必要がある
  std::vector<std::string> eeParentLink; // constant. 要素数と順序はeeNameと同じ. 必ずrobot->link(parentLink)がnullptrではないことを約束する. そのため、毎回robot->link(parentLink)がnullptrかをチェックしなくても良い
  std::vector<cnoid::Position> eeLocalT; // constant. 要素数と順序はeeNameと同じ. Parent Link Frame

  std::vector<double> maxTorque; // constant. 要素数と順序はnumJoints()と同じ. 単位は[Nm]. 0以上
  std::vector<std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > > jointLimitTables; // constant. 要素数と順序はnumJoints()と同じ. for genRobot.
  std::vector<std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > > jointLimitTablesTqc; // constant. 要素数と順序はnumJoints()と同じ. for actRobotTqc.

  const double g = 9.80665; // 重力加速度

public:
  // parameter
  std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> > copOffset = std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >{cpp_filters::TwoPointInterpolator<cnoid::Vector3>(cnoid::Vector3(0.0,0.02,0.0),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB),cpp_filters::TwoPointInterpolator<cnoid::Vector3>(cnoid::Vector3(0.0,-0.02,0.0),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB)}; // 要素数2. rleg: 0. lleg: 1. endeffector frame. 足裏COPの目標位置. 幾何的な位置はcopOffset.value()無しで考えるが、目標COPを考えるときはcopOffset.value()を考慮する. クロスできたりジャンプできたりする脚でないと左右方向(外側向き)の着地位置修正は難しいので、その方向に転びそうになることが極力ないように内側にcopをオフセットさせておくとよい.  単位[m]. 滑らかに変化する.
  std::vector<std::vector<cnoid::Vector3> > legHull = std::vector<std::vector<cnoid::Vector3> >(2, std::vector<cnoid::Vector3>{cnoid::Vector3(0.115,0.065,0.0),cnoid::Vector3(-0.095,0.065,0.0),cnoid::Vector3(-0.095,-0.065,0.0),cnoid::Vector3(0.115,-0.065,0.0)}); // 要素数2. rleg: 0. lleg: 1. endeffector frame.  凸形状で,上から見て半時計回り. Z成分はあったほうが計算上扱いやすいからありにしているが、0でなければならない. 単位[m]. JAXONでは、COPがY -0.1近くにくるとギア飛びしやすいので、少しYの下限を少なくしている
  std::vector<double> muTrans = std::vector<double>(2, 0.5); // 要素数2. 0: rleg, 1: lleg. 足裏の摩擦係数. 0より大きい. 大股で歩くときはZMP-COMの位置関係的に、垂直抗力と並進力の比がそこまで大きな差にならないので、0.1~0.3程度だと足りない場合がある.
  std::vector<double> muRot = std::vector<double>(2, 0.05); // 要素数2. 0: rleg, 1: lleg. 足裏の回転摩擦係数. 0より大きい. 旋回歩行時に必要なので、小さすぎてはいけない
  std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> > defaultTranslatePos = std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >(2,cpp_filters::TwoPointInterpolator<cnoid::Vector3>(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB)); // goPos, goVelocity, setFootSteps等するときの右脚と左脚の中心からの相対位置. また、reference frameとgenerate frameの対応付けに用いられる. (Z軸は鉛直). Z成分はあったほうが計算上扱いやすいからありにしているが、0でなければならない. RefToGenFrameConverter(handFixMode)が「左右」という概念を使うので、X成分も0でなければならない. 単位[m] 滑らかに変化する.
  std::vector<cpp_filters::TwoPointInterpolator<double> > isManualControlMode = std::vector<cpp_filters::TwoPointInterpolator<double> >(2, cpp_filters::TwoPointInterpolator<double>(0.0, 0.0, 0.0, cpp_filters::LINEAR)); // 要素数2. 0: rleg. 1: lleg. 0~1. 連続的に変化する. 1ならicEETargetPoseに従い、refEEWrenchに応じて重心をオフセットする. 0ならImpedanceControlをせず、refEEWrenchを無視し、DampingControlを行わない. 静止状態で無い場合や、支持脚の場合は、勝手に0になる. 両足が同時に1になることはない. 1にするなら、RefToGenFrameConverter.refFootOriginWeightを0にしたほうが良い. 1の状態でStartAutoStabilizerすると、遊脚で始まる

  std::vector<bool> jointControllable; // 要素数と順序はnumJoints()と同じ. falseの場合、qやtauはrefの値をそのまま出力する(writeOutputPort時にref値で上書き). IKでは動かさない(ref値をそのまま). トルク計算では目標トルクを通常通り計算する. このパラメータはMODE_IDLEのときにしか変更されない

public:
  // from data port
  cnoid::BodyPtr refRobotRaw; // reference. reference world frame
  std::vector<cnoid::Vector6> refEEWrenchOrigin; // 要素数と順序はeeNameと同じ.FootOrigin frame. EndEffector origin. ロボットが受ける力
  std::vector<cpp_filters::TwoPointInterpolatorSE3> refEEPoseRaw; // 要素数と順序はeeNameと同じ. reference world frame. EEPoseはjoint angleなどと比べて遅い周期で届くことが多いので、interpolaterで補間する.
  cnoid::BodyPtr actRobotRaw; // actual. actual imu world frame
  class Collision {
  public:
    std::string link1 = "";
    cnoid::Vector3 point1 = cnoid::Vector3::Zero(); // link1 frame
    std::string link2 = "";
    cnoid::Vector3 point2 = cnoid::Vector3::Zero(); // link2 frame
    cnoid::Vector3 direction21 = cnoid::Vector3::UnitX(); // generate frame
    double distance = 0.0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  std::vector<Collision> selfCollision;
  std::vector<std::vector<cnoid::Vector3> > steppableRegion; // generate frame. 着地可能領域の凸包の集合. 要素数0なら、全ての領域が着地可能として扱われる. Z成分はあったほうが計算上扱いやすいからありにしているが、0でなければならない. 
  std::vector<double> steppableHeight; // generate frame. 要素数と順序はsteppableRegionと同じ。steppableRegionの各要素の重心Z.
  double relLandingHeight = -1e15; // generate frame. 現在の遊脚のfootstepNodesList[0]のdstCoordsのZ. -1e10未満なら、relLandingHeightとrelLandingNormalは無視される. footStepNodesListsの次のnodeに移るたびにFootStepGeneratorによって-1e15に上書きされる.
  cnoid::Vector3 relLandingNormal = cnoid::Vector3::UnitZ(); // generate frame. 現在の遊脚のfootstepNodesList[0]のdstCoordsのZ軸の方向. ノルムは常に1
public:
  // AutoStabilizerの中で計算更新される.

  // refToGenFrameConverter
  cnoid::BodyPtr refRobot; // reference. generate frame
  std::vector<cnoid::Position> refEEPose; // 要素数と順序はeeNameと同じ.generate frame
  std::vector<cnoid::Vector6> refEEWrench; // 要素数と順序はeeNameと同じ.generate frame. EndEffector origin. ロボットが受ける力
  double refdz = 1.0; // generate frame. 支持脚からのCogの目標高さ. 0より大きい
  cpp_filters::TwoPointInterpolatorSE3 footMidCoords = cpp_filters::TwoPointInterpolatorSE3(cnoid::Position::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB); // generate frame. Z軸は鉛直. 支持脚の位置姿勢(Z軸は鉛直)にdefaultTranslatePosを適用したものの間をつなぐ. interpolatorによって連続的に変化する. reference frameとgenerate frameの対応付けに用いられる

  // actToGenFrameConverter
  cnoid::BodyPtr actRobot; // actual. generate frame
  cnoid::Vector3 actCog; // generate frame.
  cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3> actCogVel = cpp_filters::FirstOrderLowPassFilter<cnoid::Vector3>(3.5, cnoid::Vector3::Zero());  // generate frame.  現在のCOM速度. cutoff=4.0Hzは今の歩行時間と比べて遅すぎる気もするが、実際のところ問題なさそう? もとは4Hzだったが、 静止時に衝撃が加わると上下方向に左右交互に振動することがあるので少し小さくする必要がある. 3Hzにすると、追従性が悪くなってギアが飛んだ
  cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6> actRootVel = cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6>(3.5, cnoid::Vector6::Zero()); // generate frame. 現在のroot速度. rootLink origin. actCogVelと同程度. これを使ってfilterした後の値がactRobot->rootLink()->v()/w()に入るので、現在のroot速度を使いたいときはactRootVelではなくactRobot->rootLink()->v()/w()を使う
  std::vector<cnoid::Position> actEEPose; // 要素数と順序はeeNameと同じ.generate frame
  std::vector<cnoid::Vector6> actEEWrench; // 要素数と順序はeeNameと同じ.generate frame. EndEffector origin. ロボットが受ける力

  // ExternalForceHandler
  double omega = std::sqrt(g / refdz); // DCMの計算に用いる. 0より大きい
  cnoid::Vector3 l = cnoid::Vector3(0, 0, refdz); // generate frame. FootGuidedControlで外力を計算するときの、ZMP-重心の相対位置に対するオフセット. また、CMPの計算時にDCMに対するオフセット(CMP + l = DCM). 連続的に変化する.

  // ImpedanceController
  std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6> > icEEOffset; // 要素数と順序はeeNameと同じ.generate frame. endEffector origin. icで計算されるオフセット
  std::vector<cnoid::Position> icEETargetPose; // 要素数と順序はeeNameと同じ.generate frame. icで計算された目標位置姿勢. icEETargetPose = icEEOffset + refEEPose

  // CmdVelGenerator
  cnoid::Vector3 cmdVel = cnoid::Vector3::Zero(); // X[m/s] Y[m/s] theta[rad/s]. Z軸はgenerate frame鉛直. support leg frame. 不連続に変化する

  // FootStepGenerator
  /*
    FootStepActionList[0], [1], [2] ...はそれぞれ、FootStepNodesList[0]終了時, [1]終了時, [2]終了時のタイミングに作用する
    FootStepActionList.size()==0なら、FootStepNodesListは今のnodeが終了したらその状態で止まる(isStatic)

    FootStepActionList[0]は、原則変更されない.
    FootStepActionList[0]のdstCoordsやremainTimeが、着地位置時刻修正などによって直接編集される. FootStepActionList[0]のremainTimeが突然0近くに減ることはない(突然FootStepNodesList[0]が終了しFootStepNodesList[1]に移ることはある)ので、FootStepNodesList[0]のremainTimeの時間で補間するようなプログラムは安全
    FootStepActionList.size()==1の場合は、FootStepActionList[0]が変更可.
  */
  class FootStepAction { // footstepnode切り替わりのタイミングでの目標. 同時に複数のactionを並列で実行することには対応していない. precondition, result方式にすると自律探索できるようになるのでよいが、2足歩行なら人間が全て書き下すことが可能なので後回し.
  public:
    /*
      (他に理由が無ければ、前のnode終了時にisSupportPhase=trueの足の上にrefZmpが乗るように動かす. MAKE_CONTACTがfootStepActionListの末尾になることはないので、両足支持期でstatic状態になった場合、必ず両足の中間にrefZmpがある)
      KEEP_PREVIOUS:
      BREAK_CONTACT: 前のnode終了時にrefZmpがその足から外れるように動かし、次のnode開始時からはその足がisSupportPhase=falseになる.
                     前のnodeが両足ともisSupportPhase=trueでなければならない
      MAKE_CONTACT: 前のnode終了時にその足を目標接触位置に移動しかつ接触するように動かし、次のnode開始時からはその足がisSupportPhase=trueになる.「次」及び「次の次」のnode終了時にrefZmpがその足に乗り反対の足から外れていなければならない. このactionがfootStepActionListの末尾であってはならない
                    前のnodeがこの脚はisSupportPhase=false, 反対の脚はisSupportPhase=trueでなければならない
      MOVE: 前のnode終了時にその足を目標位置に移動するように動かす.
            前のnodeがこの脚はisSupportPhase=false, 反対の脚はisSupportPhase=trueでなければならない
    */
    enum class type_enum{KEEP_PREVIOUS, BREAK_CONTACT, MAKE_CONTACT, MOVE};
    type_enum type = type_enum::KEEP_PREVIOUS;
    double remainTime = 0.0; // 前のnodeが終了するまでの時間
    int targetLeg = RLEG; // BREAK_CONTACT, MAKE_CONTACT, MOVEのときのみ使用
    cnoid::Position dstCoordsLocal = cnoid::Position::Identity(); // 前のnode終了時の目標位置. MAKE_CONTACT, MOVEのときのみ使用. 反対の足を水平にした座標系で表現. (跳躍は想定されていない!).
    double stepHeight = 0.0; // BREAK_CONTACT, MAKE_CONTACTのときのみ使用. 接触点の何m上空に足を上げるか

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  std::vector<FootStepAction> footStepActionList; // 上位から与えられた目標FootStep
  class FootStepNodes { // footstepの状態を表す
  public:
    /*
      各足につきそれぞれ、remainTime 後に dstCoordsに動く.

      footstepNodesList[0]のisSupportPhaseは、変更されない
      footstepNodesList[0]のdstCoordsを変更する場合には、footstepNodesList[0]のremainTimeの小ささに応じて変更量を小さくする. remainTimeがほぼゼロなら変更量もほぼゼロ.
      footstepNodesList[0]は、!footstepNodesList[0].isSupportPhase && footstepNodesList[1].isSupportPhaseの足がある場合に、突然footstepNodesList[1]に遷移する場合がある(footStepGeneratorのearlyTouchDown)
      それ以外には、footstepNodesList[0]のremainTimeが突然0になることはない
      footstepNodesList[1]のisSupportPhaseは、footstepNodesListのサイズが1である場合を除いて変更されない.

      右脚支持期の直前のnodeのendRefZmpStateはRLEGでなければならない
      右脚支持期のnodeのendRefZmpStateはRLEGでなければならない
      左脚支持期の直前のnodeのendRefZmpStateはLLEGでなければならない
      左脚支持期のnodeのendRefZmpStateはLLEGでなければならない
      両脚支持期の直前のnodeのendRefZmpStateはRLEG or LLEG or MIDDLEでなければならない.
      両脚支持期のnodeのendRefZmpStateはRLEG or LLEG or MIDDLEでなければならない.
      footstepNodesList[0]のendRefZmpStateは変更されない.
      footstepNodesList[0]のendRefZmpStateは、isStatic()である場合を除いて変更されない.
    */
    std::vector<cnoid::Position> srcCoords = std::vector<cnoid::Position>(NUM_LEGS,cnoid::Position::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame. このnode開始時の目標位置. 支持脚は今のactualの値で常に上書きされる.
    std::vector<cnoid::Position> dstCoords = std::vector<cnoid::Position>(NUM_LEGS,cnoid::Position::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame. このnode終了時の目標位置.
    std::vector<bool> isSupportPhase = std::vector<bool>(NUM_LEGS, true); // 要素数2. rleg: 0. lleg: 1. このnode開始時から終了時まで、地面に接触して接触力を発揮しているかどうか. footstepNodesListの末尾の要素が両方falseであることは無い
    double remainTime = 0.0; // このnodeの残り時間
    std::vector<bool> beginRefZmpState = std::vector<bool>(NUM_LEGS, true); // 要素数2. rleg: 0. lleg: 1. このnode開始時のrefZmpの位置. 一つ前のnodeのendRefZmpStateと同じでなければならない
    std::vector<bool> endRefZmpState = std::vector<bool>(NUM_LEGS, true); // 要素数2. rleg: 0. lleg: 1. このnode終了時のrefZmpの位置.
    std::vector<int> isEndRefZmpStateFixed = std::vector<int>(NUM_LEGS, 0); // 要素数2. rleg: 0. lleg: 1. このendRefZmpStateの値を次のactionで変更していけないかどうか. 0なら可. 2以上の場合、次のnodeに1小さい値が入る. MAKE_CONTACTのpost conditionによる

    // 遊脚軌道用パラメータ
    std::vector<std::vector<double> > stepHeight = std::vector<std::vector<double> >(NUM_LEGS,std::vector<double>(2,0)); // 要素数2. rleg: 0. lleg: 1. swing期には、srcCoordsの高さ+[0]とdstCoordsの高さ+[1]の高い方に上げるような軌道を生成する
    std::vector<double> touchVel = std::vector<double>(NUM_LEGS,0.0); // 0より大きい. 単位[m/s]. 足を下ろすときの速さ


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  std::vector<FootStepNodes> footstepNodesList = std::vector<FootStepNodes>(1); // 要素数1以上. 0番目が現在の状態. 末尾の要素以降は、末尾の状態がずっと続くとして扱われる.
  std::vector<cnoid::Position> srcCoords = std::vector<cnoid::Position>(NUM_LEGS,cnoid::Position::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame. 現在のfootstep開始時のgenCoords
  std::vector<cnoid::Position> dstCoordsOrg = std::vector<cnoid::Position>(NUM_LEGS,cnoid::Position::Identity()); // 要素数2. rleg: 0. lleg: 1. generate frame. 現在のfootstep開始時のdstCoords
  double remainTimeOrg = 0.0; // 現在のfootstep開始時のremainTime
  enum SwingState_enum{LIFT_PHASE, SWING_PHASE, DOWN_PHASE};
  std::vector<SwingState_enum> swingState = std::vector<SwingState_enum>(NUM_LEGS,LIFT_PHASE); // 要素数2. rleg: 0. lleg: 0. isSupportPhase = falseの脚は、footstep開始時はLIFT_PHASEで、LIFT_PHASE->SWING_PHASE->DOWN_PHASEと遷移する. 一度DOWN_PHASEになったら次のfootstepが始まるまで別のPHASEになることはない. DOWN_PHASEのときはfootstepNodesList[0]のdstCoordsはgenCoordsよりも高い位置に変更されることはない. isSupportPhase = trueの脚は、swingStateは参照されない(常にLIFT_PHASEとなる).
  double elapsedTime = 0.0; // 現在のfootstep開始時からの経過時間
  std::vector<bool> prevSupportPhase = std::vector<bool>{true, true}; // 要素数2. rleg: 0. lleg: 1. 一つ前の周期でSupportPhaseだったかどうか
  std::vector<bool> stopCurrentPosition = std::vector<bool>(NUM_LEGS, false); // 遊脚を現在の位置で強制的に止めるかどうか
  std::vector<double> goalOffset = std::vector<double>(NUM_LEGS,0.0); // [m]. 遊脚軌道生成時に、遅づきの場合、generate frameで鉛直方向に, 目標着地位置に対して加えるオフセット. 0から始まって少しずつ小さくなっていく. 0以下.

  // LegCoordsGenerator
  std::vector<cpp_filters::TwoPointInterpolatorSE3> genCoords = std::vector<cpp_filters::TwoPointInterpolatorSE3>(NUM_LEGS, cpp_filters::TwoPointInterpolatorSE3(cnoid::Position::Identity(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB)); // 要素数2. rleg: 0. lleg: 1. generate frame. 現在の位置
  std::vector<footguidedcontroller::LinearTrajectory<cnoid::Vector3> > refZmpTraj = {footguidedcontroller::LinearTrajectory<cnoid::Vector3>(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),0.0)}; // 要素数1以上. generate frame. footstepNodesListを単純に線形補間して計算される現在の目標zmp軌道

  cnoid::Vector3 genCog; // generate frame. abcで計算された目標COM
  cnoid::Vector3 genCogVel;  // generate frame.  abcで計算された目標COM速度
  cnoid::Vector3 genCogAcc;  // generate frame.  abcで計算された目標COM加速度
  std::vector<cnoid::Position> abcEETargetPose; // 要素数と順序はeeNameと同じ.generate frame. abcで計算された目標位置姿勢
  std::vector<cnoid::Vector6> abcEETargetVel; // 要素数と順序はeeNameと同じ.generate frame. endeffector origin. abcで計算された目標速度
  std::vector<cnoid::Vector6> abcEETargetAcc; // 要素数と順序はeeNameと同じ.generate frame. endeffector origin. abcで計算された目標加速度

  // Stabilizer
  cnoid::BodyPtr actRobotTqc; // output. 関節トルク制御用. (actRobotと同じだが、uの値として指令関節トルクが入っている)

  // FullbodyIKSolver
  cnoid::BodyPtr genRobot; // output. 関節位置制御用

  // for debug data
  class DebugData {
  public:
    // FootStepGenerator
    std::vector<cnoid::Vector3> strideLimitationHull = std::vector<cnoid::Vector3>(); // generate frame. overwritableStrideLimitationHullの範囲内の着地位置(自己干渉・IKの考慮が含まれる). Z成分には0を入れる
    std::vector<std::vector<cnoid::Vector3> > capturableHulls = std::vector<std::vector<cnoid::Vector3> >(); // generate frame. 要素数と順番はcandidatesに対応
    std::vector<double> cpViewerLog = std::vector<double>(37, 0.0);

    // Stabilizer
    cnoid::Vector3 stTargetZmp; // generate frame. stで計算された目標ZMP
    std::vector<cnoid::Vector6> stEETargetWrench; // 要素数と順序はeeNameと同じ.generate frame. EndEffector origin. ロボットが受ける力
  };
  DebugData debugData; // デバッグ用のOutPortから出力するためのデータ. AutoStabilizer内の制御処理では使われることは無い. そのため、モード遷移や初期化等の処理にはあまり注意を払わなくて良い

public:
  bool isStatic() const{ // 現在static状態かどうか
    return this->footstepNodesList.size() == 1 && this->footstepNodesList[0].remainTime == 0.0;
  }

public:
  void init(const cnoid::BodyPtr& robot){
    maxTorque.resize(robot->numJoints(), std::numeric_limits<double>::max());
    jointLimitTables.resize(robot->numJoints());
    jointLimitTablesTqc.resize(robot->numJoints());
    jointControllable.resize(robot->numJoints(), true);
    refRobotRaw = robot->clone();
    refRobotRaw->calcForwardKinematics(); refRobotRaw->calcCenterOfMass();
    actRobotRaw = robot->clone();
    actRobotRaw->calcForwardKinematics(); actRobotRaw->calcCenterOfMass();
    refRobot = robot->clone();
    refRobot->calcForwardKinematics(); refRobot->calcCenterOfMass();
    actRobot = robot->clone();
    actRobot->calcForwardKinematics(); actRobot->calcCenterOfMass();
    actRobotTqc = robot->clone();
    actRobotTqc->calcForwardKinematics(); actRobotTqc->calcCenterOfMass();
    genRobot = robot->clone();
    genRobot->calcForwardKinematics(); genRobot->calcCenterOfMass();
  }

  void push_backEE(const std::string& name_, const std::string& parentLink_, const cnoid::Position& localT_){
    eeName.push_back(name_);
    eeParentLink.push_back(parentLink_);
    eeLocalT.push_back(localT_);
    refEEWrenchOrigin.push_back(cnoid::Vector6::Zero());
    refEEPoseRaw.push_back(cpp_filters::TwoPointInterpolatorSE3(cnoid::Position::Identity(), cnoid::Vector6::Zero(),cnoid::Vector6::Zero(), cpp_filters::HOFFARBIB));
    refEEPose.push_back(cnoid::Position::Identity());
    refEEWrench.push_back(cnoid::Vector6::Zero());
    actEEPose.push_back(cnoid::Position::Identity());
    actEEWrench.push_back(cnoid::Vector6::Zero());
    icEEOffset.push_back(cpp_filters::TwoPointInterpolator<cnoid::Vector6>(cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(), cpp_filters::HOFFARBIB));
    icEETargetPose.push_back(cnoid::Position::Identity());
    abcEETargetPose.push_back(cnoid::Position::Identity());
    abcEETargetVel.push_back(cnoid::Vector6::Zero());
    abcEETargetAcc.push_back(cnoid::Vector6::Zero());
    debugData.stEETargetWrench.push_back(cnoid::Vector6::Zero());
  }

  // startAutoStabilizer時に呼ばれる
  void reset(){
    for(int i=0;i<NUM_LEGS;i++){
      copOffset[i].reset(copOffset[i].getGoal());
      defaultTranslatePos[i].reset(defaultTranslatePos[i].getGoal());
      isManualControlMode[i].reset(isManualControlMode[i].getGoal());
    }

    // 現在の支持脚からの..という性質のportなので、リセットする必要がある
    steppableRegion.clear();
    steppableHeight.clear();
    relLandingHeight = -1e15;
    relLandingNormal = cnoid::Vector3::UnitZ();
  }

  // 毎周期呼ばれる. 内部の補間器をdtだけ進める
  void update(double dt){
    for(int i=0;i<NUM_LEGS;i++){
      copOffset[i].interpolate(dt);
      defaultTranslatePos[i].interpolate(dt);
      isManualControlMode[i].interpolate(dt);
    }
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // for debug
  mutable struct timeval prevTime;
  void resetTime() const { gettimeofday(&prevTime, NULL);}
  void printTime(const std::string& message="") const {
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    std::cerr << message << (currentTime.tv_sec - prevTime.tv_sec) + (currentTime.tv_usec - prevTime.tv_usec) * 1e-6 << std::endl;
  }
};

// for debug

inline std::ostream &operator<<(std::ostream &os, const std::vector<GaitParam::FootStepNodes>& footstepNodesList) {
  for(int i=0;i<footstepNodesList.size();i++){
    os << "footstep" << i << std::endl;
    os << " RLEG: " << std::endl;
    os << "  pos: " << (footstepNodesList[i].dstCoords[RLEG].translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
    os << "  rot: " << (footstepNodesList[i].dstCoords[RLEG].linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
    os << "  height = " << footstepNodesList[i].stepHeight[RLEG][0] << " " << footstepNodesList[i].stepHeight[RLEG][1] << std::endl;
    os << "  goaloffset = " << footstepNodesList[i].goalOffset[RLEG] << std::endl;
    os << " LLEG: " << std::endl;
    os << "  pos: " << (footstepNodesList[i].dstCoords[LLEG].translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
    os << "  rot: " << (footstepNodesList[i].dstCoords[LLEG].linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
    os << "  height = " << footstepNodesList[i].stepHeight[LLEG][0] << " " << footstepNodesList[i].stepHeight[LLEG][1] << std::endl;
    os << "  goaloffset = " << footstepNodesList[i].goalOffset[LLEG] << std::endl;
    os << " time = " << footstepNodesList[i].remainTime << "[s]" << std::endl;;
  }
  return os;
};

inline std::ostream &operator<<(std::ostream &os, const GaitParam& gaitParam) {
  os << "current" << std::endl;
  os << " RLEG: " << std::endl;
  os << "  pos: " << (gaitParam.genCoords[RLEG].value().translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
  os << "  rot: " << (gaitParam.genCoords[RLEG].value().linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
  os << " LLEG: " << std::endl;
  os << "  pos: " << (gaitParam.genCoords[LLEG].value().translation()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]")) << std::endl;
  os << "  rot: " << (gaitParam.genCoords[LLEG].value().linear()).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", " [", "]")) << std::endl;
  os << gaitParam.footstepNodesList << std::endl;
  return os;
};

inline std::ostream &operator<<(std::ostream &os, const std::vector<cnoid::Vector3>& polygon){
  for(int j=0;j<polygon.size();j++){
    os << polygon[j].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", " [", "]"));
  }
  return os;
}

#endif
