// Run on an isolated ROS master only. No controller or actuator is started.
#include <plan_env/grid_map.h>
#include <cassert>
#include <iostream>

static void parameters(ros::NodeHandle &nh, double height, double origin) {
  nh.setParam("grid_map/resolution", .15);
  nh.setParam("grid_map/map_size_x", 10.);
  nh.setParam("grid_map/map_size_y", 10.);
  nh.setParam("grid_map/map_size_z", 4.);
  nh.setParam("grid_map/ground_height", -.01);
  nh.setParam("grid_map/skip_pixel", 2);
  nh.setParam("grid_map/virtual_ceil_height", height);
  nh.setParam("grid_map/virtual_ceil_origin_z", origin);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ceiling_grid_map_test");
  ros::NodeHandle nh("~first");
  parameters(nh, .9, 0.);
  GridMap map; map.initMap(nh);
  assert(!map.initialCloudMapReady()); // ceiling is not a sensor observation
  for (double z: {.8, .899999}) {
    assert(map.getOccupancy(Eigen::Vector3d(0,0,z)) == 0);
    assert(map.getInflateOccupancy(Eigen::Vector3d(0,0,z)) == 0);
  }
  for (double z: {.9, 1., 100.}) {
    assert(map.getOccupancy(Eigen::Vector3d(0,0,z)) == 1);
    assert(map.getInflateOccupancy(Eigen::Vector3d(0,0,z)) == 1);
  }
  Eigen::Vector3i index; map.posToIndex(Eigen::Vector3d(0,0,1.),index);
  assert(map.getOccupancy(index)==1 && map.isKnownOccupied(index));
  assert(!map.isUnknown(index) && !map.isKnownFree(index));
  map.setOccupied(Eigen::Vector3d(0,0,.8));
  assert(map.getInflateOccupancy(Eigen::Vector3d(0,0,.8))==1);
  map.resetBuffer();
  assert(map.getInflateOccupancy(Eigen::Vector3d(0,0,.8))==0);
  assert(map.getInflateOccupancy(Eigen::Vector3d(0,0,.9))==1);
  assert(!map.initialCloudMapReady());

  ros::NodeHandle shifted("~shifted"); parameters(shifted,.9,.37);
  GridMap second; second.initMap(shifted);
  assert(second.getInflateOccupancy(Eigen::Vector3d(0,0,1.17))==0);
  assert(second.getInflateOccupancy(Eigen::Vector3d(0,0,1.28))==1);
  ros::NodeHandle disabled("~disabled"); parameters(disabled,-1.,0.);
  GridMap third; third.initMap(disabled);
  assert(third.getInflateOccupancy(Eigen::Vector3d(0,0,1.))==0);
  std::cout << "PASS actual GridMap: exact boundary, .8 cruise, reference offset, reset, disabled, sensor readiness\n";
}
