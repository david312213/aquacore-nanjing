cmake_minimum_required(VERSION 3.0.2)
project(airseafly_real400_mission)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs nav_msgs sensor_msgs std_msgs
  std_srvs mavros_msgs quadrotor_msgs airseafly_real400_msgs cv_bridge)
find_package(Eigen3 REQUIRED)
catkin_python_setup()
catkin_package()
add_library(airseafly_fixed_min_snap SHARED native/fixed_min_snap.cpp)
target_include_directories(airseafly_fixed_min_snap PRIVATE
  ${EIGEN3_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} vendor/gcopter/include)
set_target_properties(airseafly_fixed_min_snap PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED YES)
install(TARGETS airseafly_fixed_min_snap
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES vendor/gcopter/LICENSE vendor/gcopter/README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/vendor/gcopter)
catkin_install_python(PROGRAMS
  scripts/compute_guard.py
  scripts/delivery_route.py
  scripts/drop_test.py
  scripts/drop_points.py
  scripts/hover_drop.py
  scripts/fly_waypoints.py
  scripts/mission_cli.py
  scripts/mission_manager.py
  scripts/native_ekf_snapshot.py
  scripts/native_flight_report.py
  scripts/native_ground_check.py
  scripts/native_lio_bridge.py
  scripts/native_lio_profile.py
  scripts/native_planning_cloud.py
  scripts/record_delivery_route.py
  scripts/record_waypoints.py
  scripts/record_drop_points.py
  scripts/rc_payload_control.py
  scripts/localization_gate_node.py
  scripts/red_point_preview.py
  scripts/red_point_worker.py
  scripts/safe_land.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_fixed_trajectory.py)
  catkin_add_nosetests(test/test_fixed_trajectory_runtime.py)
  catkin_add_nosetests(test/test_fixed_trajectory_manager.py)
  catkin_add_nosetests(test/test_fixed_trajectory_preview.py)
  catkin_add_nosetests(test/test_native_fixed_waypoint_route.py)
  catkin_add_nosetests(test/test_basic_flight_entry.py)
  catkin_add_nosetests(test/test_delivery_route_entry.py)
  catkin_add_nosetests(test/test_delivery_alignment.py)
  catkin_add_nosetests(test/test_single_drop_entry.py)
  catkin_add_nosetests(test/test_single_drop.py)
  catkin_add_nosetests(test/test_drop_points.py)
  catkin_add_nosetests(test/test_drop_points_entry.py)
  catkin_add_nosetests(test/test_native_drop_points_manager.py)
  catkin_add_nosetests(test/test_hover_drop_entry.py)
  catkin_add_nosetests(test/test_ego_goal_identity.py)
  catkin_add_nosetests(test/test_four_magnet_payload.py)
  catkin_add_nosetests(test/test_guided_route.py)
  catkin_add_nosetests(test/test_main_servo_test.py)
  catkin_add_nosetests(test/test_nanjing_configuration.py)
  catkin_add_nosetests(test/test_native_covariance.py)
  catkin_add_nosetests(test/test_native_delivery_manager.py)
  catkin_add_nosetests(test/test_native_single_drop_manager.py)
  catkin_add_nosetests(test/test_native_delivery_route.py)
  catkin_add_nosetests(test/test_native_flight_report.py)
  catkin_add_nosetests(test/test_native_freshness.py)
  catkin_add_nosetests(test/test_native_full_runtime.py)
  catkin_add_nosetests(test/test_native_landing_confirmation.py)
  catkin_add_nosetests(test/test_native_localization_health.py)
  catkin_add_nosetests(test/test_native_payload_transport.py)
  catkin_add_nosetests(test/test_native_planning_geometry.py)
  catkin_add_nosetests(test/test_native_reference.py)
  catkin_add_nosetests(test/test_native_waypoint_manager.py)
  catkin_add_nosetests(test/test_native_waypoint_route.py)
  catkin_add_nosetests(test/test_payload.py)
  catkin_add_nosetests(test/test_payload_slot_pwm.py)
  catkin_add_nosetests(test/test_planning_launch.py)
  catkin_add_nosetests(test/test_red_point.py)
  catkin_add_nosetests(test/test_red_point_entry.py)
  catkin_add_nosetests(test/test_red_point_worker.py)
  catkin_add_nosetests(test/test_resource_snapshot.py)
  catkin_add_nosetests(test/test_rc_payload.py)
  catkin_add_nosetests(test/test_localization_gate.py)
  catkin_add_nosetests(test/test_safe_land.py)
  catkin_add_nosetests(test/test_servo_reset.py)
  catkin_add_nosetests(test/test_waypoint_route.py)
endif()
