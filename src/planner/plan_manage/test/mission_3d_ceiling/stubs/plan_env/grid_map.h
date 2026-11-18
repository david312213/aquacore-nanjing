#pragma once
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <functional>
#include <plan_env/virtual_ceiling.h>
#include <plan_env/virtual_floor.h>
class GridMap {
public:
  using Ptr = std::shared_ptr<GridMap>;
  double resolution = 0.15;
  ego_planner::VirtualCeiling ceiling;
  ego_planner::VirtualFloor floor;
  std::function<int(const Eigen::Vector3d &)> occupied = [](const Eigen::Vector3d &) { return 0; };
  void initMap(ros::NodeHandle &) {}
  double getResolution() const { return resolution; }
  int getInflateOccupancy(const Eigen::Vector3d &point) const {
    if (ceiling.occupied(point.z()) || floor.occupied(point.z())) return 1;
    if (!point.allFinite() || point.x() < -3 || point.x() > 3 ||
        point.y() < -2 || point.y() > 2 || point.z() < 0 || point.z() > 3) return -1;
    return occupied(point);
  }
};
