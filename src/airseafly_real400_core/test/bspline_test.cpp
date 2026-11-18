#include <airseafly_real400/core.hpp>
#include <iostream>
#include "bspline_opt/uniform_bspline.h"
#include "uniform_bspline.cpp"
using namespace airseafly_real400;
int main() {
  try {
    // Trajectory callback timing is covered by plan_manage/test/test_trajectory_time.py.
    Eigen::MatrixXd p(3,8);
    for(int i=0;i<8;++i) p.col(i)=V3(i*.02,0,1.5);
    ego_planner::UniformBspline trajectory(p,3,.1);
    trajectory.setPhysicalLimits(.7,.8,.05);double ratio;
    require(trajectory.checkFeasibility(ratio),"nominal trajectory failed limits");
    auto velocity=trajectory.getDerivative();
    for(double t=0;t<trajectory.getTimeSum();t+=.01) {
      require(std::abs(trajectory.evaluateDeBoorT(t)[2]-1.5)<1e-10,"1.5m height changed");
      require(velocity.evaluateDeBoorT(t).norm()<.7,"velocity violation");
    }
    trajectory.setPhysicalLimits(.1,.8,.05);
    require(!trajectory.checkFeasibility(ratio) && ratio>1,"lower dynamic speed limit ignored");
    for(int i=0;i<8;++i) p.col(i)=V3(i*.06,i*.06,1.5);
    ego_planner::UniformBspline diagonal(p,3,.1);diagonal.setPhysicalLimits(.7,.8,.05);
    // The retained EGO feasibility check constrains each derivative axis.
    // Preserve that existing contract; a Euclidean-norm policy is a planner change.
    require(diagonal.checkFeasibility(ratio),"feasible per-axis diagonal rejected");
    for(int i=0;i<8;++i) p.col(i)=V3(i*.09,0,1.5);
    ego_planner::UniformBspline excessive(p,3,.1);excessive.setPhysicalLimits(.7,.8,.05);
    require(!excessive.checkFeasibility(ratio),"per-axis velocity violation accepted");
    std::cout<<"PASS current B-spline: altitude, derivatives, dynamic and per-axis limits\n";
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
