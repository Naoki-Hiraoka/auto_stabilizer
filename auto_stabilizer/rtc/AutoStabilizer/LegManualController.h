#ifndef LEGMANUALCONTROLLER_H
#define LEGMANUALCONTROLLER_H

#include "GaitParam.h"

class LegManualController {
public:
  // LegDefaultControllerだけが使うパラメータ

public:
  bool legManualControl(const GaitParam& gaitParam, double dt,
                        std::vector<cpp_filters::TwoPointInterpolatorSE3>& o_genCoords, std::vector<GaitParam::FootStepNodes>& o_footstepNodesList, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_isManualControlMode) const;
};

#endif
