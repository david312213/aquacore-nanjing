// Isolated ROS master only. Tests published maps without launching EGO or hardware.
#include <plan_env/grid_map.h>
#include <cassert>
#include <functional>
#include <iostream>

static void waitUntil(const std::function<bool()> &ready, const char *label) {
  const auto end = ros::WallTime::now() + ros::WallDuration(4.0);
  while (ros::ok() && ros::WallTime::now() < end) {
    ros::spinOnce();
    if (ready()) return;
    ros::WallDuration(.01).sleep();
  }
  std::cerr << "Timeout: " << label << '\n';
  std::abort();
}

static bool contains(const sensor_msgs::PointCloud2ConstPtr &message,
                     const Eigen::Vector3d &point) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*message, cloud);
  for (const auto &p : cloud)
    if ((Eigen::Vector3d(p.x, p.y, p.z) - point).norm() < 1e-5) return true;
  return false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ceiling_visualization_test");
  ros::NodeHandle nh("~map");
  nh.setParam("grid_map/resolution", .15);
  nh.setParam("grid_map/map_size_x", 3.);
  nh.setParam("grid_map/map_size_y", 3.);
  nh.setParam("grid_map/map_size_z", 2.);
  nh.setParam("grid_map/ground_height", 0.);
  nh.setParam("grid_map/skip_pixel", 2);
  nh.setParam("grid_map/p_occ", .6);
  nh.setParam("grid_map/virtual_ceil_height", .9);
  nh.setParam("grid_map/virtual_ceil_origin_z", 0.);
  nh.setParam("grid_map/virtual_floor_height", .2);
  nh.setParam("grid_map/virtual_floor_origin_z", 0.);
  nh.setParam("grid_map/visualization_truncate_height", 1.8);
  // Start without the display parameter: default must hide the virtual ceiling.
  GridMap map;
  map.initMap(nh);
  map.resetBuffer();
  // Directly populated buffers have no sensor observation. The normal timer
  // correctly waits for one; publish these test buffers explicitly instead.
  auto waitForMaps = [&](const std::function<bool()> &ready, const char *label) {
    waitUntil([&]() {
      map.publishMapInflate(true);
      map.publishMap();
      return ready();
    }, label);
  };
  const Eigen::Vector3d low(.3, .3, .65), high(.3, .3, 1.1);
  for (const auto &point : {low, high}) {
    map.setOccupancy(point);
    map.setOccupied(point);
  }
  Eigen::Vector3i index;
  Eigen::Vector3d low_center, high_center;
  map.posToIndex(low, index); map.indexToPos(index, low_center);
  map.posToIndex(high, index); map.indexToPos(index, high_center);
  sensor_msgs::PointCloud2ConstPtr raw, inflated;
  auto raw_sub = nh.subscribe<sensor_msgs::PointCloud2>("grid_map/occupancy", 1,
      [&](const sensor_msgs::PointCloud2ConstPtr &msg) { raw = msg; });
  auto inflated_sub = nh.subscribe<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 1,
      [&](const sensor_msgs::PointCloud2ConstPtr &msg) { inflated = msg; });
  auto hidden = [&]() { return raw && inflated && raw->width == 2 && inflated->width == 2; };
  auto checkObstaclesAndCeiling = [&]() {
    assert(contains(raw, low_center) && contains(raw, high_center));
    assert(contains(inflated, low_center) && contains(inflated, high_center));
    assert(map.getOccupancy(Eigen::Vector3d(0, 0, .9)) == 1);
    assert(map.getInflateOccupancy(Eigen::Vector3d(0, 0, .9)) == 1);
    assert(map.getOccupancy(Eigen::Vector3d(0, 0, .2)) == 1);
    assert(map.getInflateOccupancy(Eigen::Vector3d(0, 0, .2)) == 1);
    assert(map.getInflateOccupancy(Eigen::Vector3d(0, 0, .65)) == 0);
    assert(!map.initialCloudMapReady());
  };
  waitForMaps(hidden, "default-hidden point clouds");
  checkObstaclesAndCeiling();
  std::cout << "Default hidden: raw=" << raw->width << " inflated=" << inflated->width << '\n';

  nh.setParam("grid_map/show_virtual_ceiling", true);
  waitForMaps([&]() { return raw->width > 2 && inflated->width > 2; }, "visible ceiling");
  checkObstaclesAndCeiling();
  std::cout << "Visible: raw=" << raw->width << " inflated=" << inflated->width << '\n';

  nh.setParam("grid_map/show_virtual_ceiling", false);
  waitForMaps(hidden, "ceiling hidden again");
  checkObstaclesAndCeiling();
  nh.setParam("grid_map/show_virtual_floor", true);
  waitForMaps([&]() { return raw->width > 2 && inflated->width > 2; }, "visible floor");
  checkObstaclesAndCeiling();
  for (const auto &msg: {raw, inflated}) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    for (const auto &p: cloud)
      assert(p.z <= .2 || (Eigen::Vector3d(p.x,p.y,p.z)-low_center).norm()<1e-5 ||
                         (Eigen::Vector3d(p.x,p.y,p.z)-high_center).norm()<1e-5);
  }
  nh.setParam("grid_map/show_virtual_floor", false);
  waitForMaps(hidden, "floor hidden again");
  checkObstaclesAndCeiling();
  std::cout << "PASS actual GridMap: virtual ceiling hidden/toggled in both clouds, "
               "floor independently toggled, real obstacles retained, planning boundaries unchanged\n";
}
