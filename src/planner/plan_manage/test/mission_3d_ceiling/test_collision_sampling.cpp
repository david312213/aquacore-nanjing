// Actual optimizer/B-splines, offline map/transport only. No hardware or ROS.
#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/trajectory_collision.h>
#include <cassert>
#include <iostream>

using namespace ego_planner;
using Eigen::Vector3d;

static ros::NodeHandle parameters() {
  ros::NodeHandle nh;
  nh.values = {{"optimization/lambda_smooth",1}, {"optimization/lambda_collision",.5},
    {"optimization/lambda_feasibility",.1}, {"optimization/lambda_fitness",1},
    {"optimization/dist0",.5}, {"optimization/swarm_clearance",.5},
    {"optimization/max_vel",.3}, {"optimization/max_acc",.5}};
  return nh;
}

int main() {
  auto map = std::make_shared<GridMap>(); map->resolution = .11;
  auto nh = parameters();
  BsplineOptimizer optimizer;
  optimizer.setParam(nh); optimizer.setEnvironment(map);
  Eigen::MatrixXd linear(3,10);
  for (int i=0; i<10; ++i) linear.col(i) = Vector3d(.2*i-.2,0,.8);
  optimizer.initControlPoints(linear);
  for (int i=0; i<10; ++i) optimizer.ref_pts_.push_back(linear.col(std::min(i+1,9)));
  Eigen::MatrixXd refined;
  assert(optimizer.BsplineOptimizeTrajRefine(linear,1.,refined));
  // A free goal with an obstacle only in the final third was accepted before.
  map->occupied = [](const Vector3d &p) { return std::abs(p.x()-1.17)<.06; };
  assert(!optimizer.BsplineOptimizeTrajRefine(linear,1.,refined));

  Eigen::MatrixXd bent(3,10);
  const double x[10] = {-.1,0,.1,.1,0,-.1,.2,.2,.3,.4};
  const double y[10] = {0,0,0,1,2,2,1,0,0,0};
  for (int i=0; i<10; ++i) bent.col(i) = Vector3d(x[i],y[i],.8);
  UniformBspline detour(bent,3,1.);
  const auto obstacle = detour.evaluateDeBoorT(.20*detour.getTimeSum()).eval();
  map->occupied = [obstacle](const Vector3d &p) { return (p-obstacle).norm()<.2; };
  double collision_time = -1.;
  auto occupied = [&](const Vector3d &p) { return map->getInflateOccupancy(p)!=0; };
  assert(firstTrajectoryCollision(detour,map->resolution,occupied,collision_time));
  assert(collision_time>0. && collision_time<detour.getTimeSum()*2./3.);
  map->occupied = [](const Vector3d &) { return false; };
  assert(!firstTrajectoryCollision(detour,map->resolution,occupied,collision_time));
  // Verify a motion bound on samples, including an offset execution start.
  const double step = trajectoryCollisionStep(detour,map->resolution);
  Vector3d previous = detour.evaluateDeBoorT(0.);
  for (double t=step; t<detour.getTimeSum(); t+=step) {
    const Vector3d current = detour.evaluateDeBoorT(t);
    assert((current-previous).norm()<=map->resolution/2.+1e-9);
    previous = current;
  }
  const Vector3d endpoint = detour.evaluateDeBoorT(detour.getTimeSum());
  map->occupied = [endpoint](const Vector3d &p) { return (p-endpoint).norm()<1e-10; };
  assert(firstTrajectoryCollision(detour,map->resolution,occupied,collision_time,detour.getTimeSum()/2.));
  assert(std::abs(collision_time-detour.getTimeSum())<1e-9);

  // A stationary stop or a returning curve must not divide by endpoint distance.
  UniformBspline stopped(Eigen::MatrixXd::Constant(3,6,.8),3,1.);
  assert(!firstTrajectoryCollision(stopped,map->resolution,[](const Vector3d &) { return false; },collision_time));
  assert(firstTrajectoryCollision(stopped,map->resolution,[](const Vector3d &) { return true; },collision_time));
  assert(firstTrajectoryCollision(stopped,0.,[](const Vector3d &) { return false; },collision_time));
  assert(firstTrajectoryCollision(stopped,map->resolution,[](const Vector3d &) { return false; },collision_time,
                                  std::numeric_limits<double>::quiet_NaN()));
  bent.row(0).setZero();
  UniformBspline returning(bent,3,1.);
  assert(!firstTrajectoryCollision(returning,map->resolution,[](const Vector3d &) { return false; },collision_time));
  std::cout << "PASS collision sampling: curved-path obstacle, final-third rejection, free path, endpoint, stationary/returning paths\n";
}
