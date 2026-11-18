cmake_minimum_required(VERSION 3.0.2)
project(airseafly_real400)
add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS
  roscpp nav_msgs sensor_msgs geometry_msgs std_msgs tf2_ros
  pcl_ros pcl_conversions airseafly_real400_core airseafly_real400_msgs)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_executable(frame_cloud_bridge src/frame_cloud_bridge.cpp)
add_dependencies(frame_cloud_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(frame_cloud_bridge ${catkin_LIBRARIES} ${PCL_LIBRARIES})

catkin_install_python(PROGRAMS scripts/set_imu_message_intervals.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
