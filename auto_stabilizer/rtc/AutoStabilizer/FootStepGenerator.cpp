#include "FootStepGenerator.h"

namespace footstepgenerator{
  void calcFootSteps(GaitParam& gaitParam, const double& dt){
    // footstepNodesList[1]以降の要素は、要素の数から中身まであらゆる点が、この関数内で上書きされる可能性がある
    // footstepNodesList[0]の要素は、remainTimeと、現在swing期間中の脚のdstCoordsのXYZとZ軸方向のみ、この関数内で上書きされる可能性がある

    // TODO
  }
};
