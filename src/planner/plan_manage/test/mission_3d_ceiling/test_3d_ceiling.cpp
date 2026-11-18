#include <plan_manage/planner_manager.h>
#include <cassert>
#include <iostream>
using namespace ego_planner;
using Eigen::Vector3d;

static ros::NodeHandle parameters() {
  ros::NodeHandle nh;
  nh.values = {{"manager/max_vel", .3}, {"manager/max_acc", .5},
    {"manager/control_points_distance", .4}, {"manager/feasibility_tolerance", .05},
    {"manager/drone_id", 0}, {"optimization/lambda_smooth", 1},
    {"optimization/lambda_collision", .5}, {"optimization/lambda_feasibility", .1},
    {"optimization/lambda_fitness", 1}, {"optimization/dist0", .5},
    {"optimization/swarm_clearance", .5}, {"optimization/max_vel", .3},
    {"optimization/max_acc", .5}};
  return nh;
}

int main() {
  VirtualCeiling ceiling;
  assert(!ceiling.occupied(100.));
  ceiling.height = .9;
  assert(!ceiling.occupied(.8) && !ceiling.occupied(.899999));
  assert(ceiling.occupied(.9) && ceiling.occupied(100.));
  ceiling.origin_z = 5.;
  assert(!ceiling.occupied(5.8) && ceiling.occupied(5.9));
  ceiling.height = -1.;
  assert(!ceiling.occupied(100.));

  auto map = std::make_shared<GridMap>();
  map->ceiling.height = .9;
  map->floor.height = .2;
  AStar astar;
  astar.initGridMap(map, Eigen::Vector3i(70,70,40));
  // A low wall forces vertical motion. A restored 3D A* can climb below .9.
  map->occupied = [](const Vector3d &p) { return std::abs(p.x()) < .25 && p.z() < .65; };
  assert(astar.AstarSearch(.1, Vector3d(-1,0,.4), Vector3d(1,0,.4)));
  double peak = 0.;
  for (const auto &p: astar.getPath()) {
    peak = std::max(peak, p.z());
    assert(p.z() > .2 && p.z() < .9 && map->getInflateOccupancy(p) == 0);
  }
  assert(peak >= .65);
  // The same wall reaching the ceiling has no overflight route.
  map->occupied = [](const Vector3d &p) { return std::abs(p.x()) < .25 && p.z() < 1.2; };
  assert(!astar.AstarSearch(.1, Vector3d(-1,0,.8), Vector3d(1,0,.8)));
  // A hanging wall has a route only below the configured floor. Reject it.
  map->occupied = [](const Vector3d &p) { return std::abs(p.x()) < .25 && p.z() > .15; };
  assert(!astar.AstarSearch(.1, Vector3d(-1,0,.8), Vector3d(1,0,.8)));
  // Sideways detours at .8 remain available under the .9 ceiling.
  map->occupied = [](const Vector3d &p) { return std::abs(p.x()) < .25 && std::abs(p.y()) < .3; };
  assert(astar.AstarSearch(.1, Vector3d(-1,0,.8), Vector3d(1,0,.8)));
  for (const auto &p: astar.getPath()) assert(p.z() > .2 && p.z() < .9);
  std::cout << "PASS original 3D A*: climb, ceiling/floor-blocked routes, side detour\n";

  auto nh = parameters();
  EGOPlannerManager manager;
  manager.initPlanModules(nh, std::make_shared<PlanningVisualization>());
  manager.deliverTrajToOptimizer(); manager.setDroneIdtoOpt();
  manager.grid_map_->ceiling.height = .9;
  manager.grid_map_->floor.height = .2;
  const Vector3d zero = Vector3d::Zero();
  // Distinct start/end Z proves optimization no longer freezes Z.
  assert(manager.planGlobalTraj(Vector3d(-1,0,.5),zero,zero,Vector3d(1,0,.8),zero,zero));
  assert(manager.reboundReplan(Vector3d(-1,0,.5),zero,zero,Vector3d(1,0,.8),zero,true,false));
  auto curve = manager.local_data_.position_traj_;
  assert(curve.getControlPoint().row(2).maxCoeff()-curve.getControlPoint().row(2).minCoeff() > .2);
  for (int i=0;i<=10000;++i) {
    auto p = curve.evaluateDeBoorT(curve.getTimeSum()*i/10000.);
    assert(p.allFinite() && p.z()>.2 && p.z()<.9);
  }
  // Optimizer takes the existing inflated-map interface, including the ceiling.
  manager.grid_map_->occupied = [](const Vector3d &p) {
    return std::abs(p.x()) < .25 && std::abs(p.y()) < .3;
  };
  assert(manager.reboundReplan(Vector3d(-1,0,.8),zero,zero,Vector3d(1,0,.8),zero,true,false));
  curve = manager.local_data_.position_traj_;
  double side = 0.;
  for (int i=0;i<=10000;++i) {
    auto p = curve.evaluateDeBoorT(curve.getTimeSum()*i/10000.);
    assert(p.allFinite() && manager.grid_map_->getInflateOccupancy(p)==0);
    side = std::max(side,std::abs(p.y()));
  }
  assert(side>.3);
  std::cout << "PASS original optimizer: variable Z, full sampled paths below ceiling, obstacle detour\n";
}
