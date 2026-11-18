// Run on an isolated ROS master only. No controller or actuator is started.
#include <plan_env/grid_map.h>
#include <cassert>
#include <iostream>

static void parameters(ros::NodeHandle &nh, double floor, double origin) {
  nh.setParam("grid_map/resolution", .15);
  nh.setParam("grid_map/map_size_x", 3.);
  nh.setParam("grid_map/map_size_y", 3.);
  nh.setParam("grid_map/map_size_z", 3.);
  nh.setParam("grid_map/ground_height", -.01);
  nh.setParam("grid_map/skip_pixel", 2);
  nh.setParam("grid_map/virtual_floor_height", floor);
  nh.setParam("grid_map/virtual_floor_origin_z", origin);
  nh.setParam("grid_map/virtual_ceil_height", 1.);
  nh.setParam("grid_map/virtual_ceil_origin_z", origin);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "floor_grid_map_test");
  for (double origin: {0., .37, -.2}) {
    ros::NodeHandle nh("~map_" + std::to_string(int((origin+1)*100)));
    parameters(nh, .2, origin);
    GridMap map; map.initMap(nh);
    const double boundary = origin + .2;
    assert(!map.initialCloudMapReady());
    for (double z: {boundary, boundary-1e-6, -100.}) {
      const Eigen::Vector3d p(0,0,z);
      assert(map.getOccupancy(p) == 1 && map.getInflateOccupancy(p) == 1);
      assert(!map.isUnknown(p));
    }
    // The boundary must not consume a whole voxel or add obstacle inflation.
    for (double z: {boundary+1e-6, origin+.8}) {
      const Eigen::Vector3d p(0,0,z);
      assert(map.getOccupancy(p) == 0 && map.getInflateOccupancy(p) == 0);
    }
    Eigen::Vector3i index;
    map.posToIndex(Eigen::Vector3d(0,0,boundary-.15), index);
    assert(map.getOccupancy(index)==1 && map.isKnownOccupied(index));
    assert(!map.isUnknown(index) && !map.isKnownFree(index));
    const Eigen::Vector3d cruise(0,0,origin+.8);
    map.setOccupied(cruise);
    assert(map.getInflateOccupancy(cruise)==1);
    map.resetBuffer();
    assert(map.getInflateOccupancy(cruise)==0);
    assert(map.getInflateOccupancy(Eigen::Vector3d(0,0,boundary))==1);
    assert(map.getInflateOccupancy(Eigen::Vector3d(0,0,origin+1.))==1);
    assert(!map.initialCloudMapReady());
  }
  ros::NodeHandle disabled("~disabled"); parameters(disabled,-1.,0.);
  GridMap legacy; legacy.initMap(disabled);
  assert(legacy.getInflateOccupancy(Eigen::Vector3d(0,0,.1))==0);
  ros::NodeHandle zero("~zero"); parameters(zero,0.,0.);
  GridMap ground; ground.initMap(zero);
  assert(ground.getInflateOccupancy(Eigen::Vector3d(0,0,0.))==1);
  assert(ground.getInflateOccupancy(Eigen::Vector3d(0,0,.000001))==0);
  std::cout << "PASS actual GridMap floor: exact boundary, offsets, reset, ceiling, disabled, sensor readiness\n";
}
