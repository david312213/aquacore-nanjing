// Real EGO manager, optimizer, spline and A* with the existing synthetic map/ROS doubles.
#include <plan_manage/planner_manager.h>
#include <cassert>
#include <iostream>
using namespace ego_planner;
using Eigen::Vector3d;

int main() {
  ros::NodeHandle nh;
  nh.values = {{"manager/max_vel", .3}, {"manager/max_acc", .5},
    {"manager/control_points_distance", .4}, {"manager/feasibility_tolerance", .05},
    {"manager/min_replan_distance", .2}, {"manager/drone_id", 0},
    {"optimization/lambda_smooth", 1}, {"optimization/lambda_collision", .5},
    {"optimization/lambda_feasibility", .1}, {"optimization/lambda_fitness", 1},
    {"optimization/dist0", .5}, {"optimization/swarm_clearance", .5},
    {"optimization/max_vel", .3}, {"optimization/max_acc", .5}};
  EGOPlannerManager manager;
  manager.initPlanModules(nh, std::make_shared<PlanningVisualization>());
  manager.deliverTrajToOptimizer(); manager.setDroneIdtoOpt();
  const Vector3d zero = Vector3d::Zero(), start(0., 0., .8);
  int cases = 0;
  for (double distance : {0., .000001, .18, .20, .22, .25})
    for (double speed : {-.1, 0., .12, .3}) {
      Vector3d goal = start + Vector3d(distance,0.,0.), velocity(speed,0.,0.);
      assert(manager.planGlobalTraj(start,velocity,zero,goal,zero,zero));
      assert(manager.global_data_.global_traj_.evaluate(0).allFinite());
      assert(manager.reboundReplan(start,velocity,zero,goal,zero,true,false));
      auto curve = manager.local_data_.position_traj_;
      auto v = curve.getDerivative(); auto a = v.getDerivative();
      double duration = curve.getTimeSum();
      assert((curve.evaluateDeBoorT(0.)-start).norm()<1e-8);
      assert((v.evaluateDeBoorT(0.)-velocity).norm()<1e-8);
      assert((curve.evaluateDeBoorT(duration)-goal).norm()<1e-8);
      assert(v.evaluateDeBoorT(duration).norm()<1e-8);
      for (int i=0;i<=1000;++i) {
        double t=duration*i/1000.;
        assert(v.evaluateDeBoorT(t).cwiseAbs().maxCoeff()<=.31511);
        assert(a.evaluateDeBoorT(t).cwiseAbs().maxCoeff()<=.52511);
        assert(manager.grid_map_->getInflateOccupancy(curve.evaluateDeBoorT(t))==0);
      }
      ++cases;
    }
  // The endpoint is fixed even if collision optimization would prefer to move it.
  const Vector3d goal(.18,0.,.8);
  manager.grid_map_->occupied = [goal](const Vector3d &p) {return (p-goal).norm()<.05;};
  assert(!manager.reboundReplan(start,zero,zero,goal,zero,true,false));
  std::cout << "PASS: " << cases << " real optimizer short approaches; occupied endpoint rejected\n";
}
