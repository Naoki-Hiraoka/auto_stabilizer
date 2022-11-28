/* -*- coding:utf-8-unix mode:c++ -*- */

#ifndef FOOTGILDEDCONTROLLER_H_
#define FOOTGILDEDCONTROLLER_H_
#include <iostream>
#include <Eigen/Eigen>

namespace footguidedcontroller{
  template <typename T>
  class LinearTrajectory
  {
  private:
    T a, start, goal;
    double time;
  public:
    LinearTrajectory(const T& _start, const T& _goal, const double _t)
      : start(_start), goal(_goal) {
      time = std::max(_t, 0.0);
      if(time==0) a = _start*0.0;
      else a = (_goal - _start) / time;
    };
    const T& getSlope() const { return a; };
    const T& getStart() const { return start; };
    const T& getGoal() const { return goal; };
    double getTime() const { return time; };
  };

  /*
    J = \int^T_0 (u - u^r)^2 dt
    \dot x = w ( x - u - l)
    x(0) = x0
    x(T) = u^r(T) + l
    u^r = std::vector<LinearTrajectory>
  */
  // ur_のサイズは1以上でなければならない. ur_にtime=0の要素があっても、その要素のstartとgoalが同じなら破綻しない. ただし、ur_のtimeの和が0だと破綻する
  template <typename T> T calcFootGuidedControl(const double& w, const T& l, const T& x0, const std::vector<LinearTrajectory<T> >& ur_) {
    const int n = ur_.size();

    std::vector<LinearTrajectory<T> > ur;
    ur.reserve(n + 2);
    ur.push_back(LinearTrajectory<T>(x0-l, x0-l, 0.0)); // j=0
    std::copy(ur_.begin(), ur_.end(), std::back_inserter(ur)); // j=1 ~ j=n
    ur.push_back(LinearTrajectory<T>(ur_.back().getGoal(),ur_.back().getGoal(), 0.0)); // j=n+1

    T u = x0*0.0;
    double Tj = 0.0;
    for(int j=0;j<=n;j++){
      Tj += ur.at(j).getTime();
      u += exp(- w * Tj) * (ur.at(j).getGoal() - ur.at(j+1).getStart() + (ur.at(j).getSlope() - ur.at(j+1).getSlope()) / w);
    }

    if((1 - exp(-2 * w * Tj)) == 0.0) { // ゼロ除算チェック
      std::cerr << "[calcFootGuidedControl] (1 - exp(-2 * w * Tj))==0 !" << std::endl;
      return ur[1].getStart();
    }

    return ur[1].getStart() + 2 / (1 - exp(-2 * w * Tj)) * u;
  };

  template <typename T> void updateState(const double& w, const T& l, const T& c, const T& dc, const T& u, double m, double dt,
                                         T& o_c, T& o_dc, T& o_ddc, T& o_f/*uから受ける力*/) {
    o_c = c + dc * dt;
    o_dc = dc + w * w * (c - u - l) * dt;
    o_ddc = w * w * (c - u - l);
    o_f = m * w * w * (c - u);
    return;
  };

};

#endif /*FOOTGILDEDCONTROLLER_H_H_*/
