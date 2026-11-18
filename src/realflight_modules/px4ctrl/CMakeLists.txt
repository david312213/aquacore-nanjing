cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  quadrotor_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  uav_utils
  mavros
  mavros_msgs
  nav_msgs
  std_srvs
  airseafly_real400_msgs
  message_generation
)

add_service_files(FILES PayloadPD.srv ControlOwner.srv)
generate_messages()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  include
)

add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)

add_dependencies(px4ctrl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(px4ctrl_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(native_owner_test test/native_owner_test.cpp)
  add_test(NAME native_owner_test COMMAND native_owner_test)
  add_executable(native_watchdog_test test/native_watchdog_test.cpp)
  add_test(NAME native_watchdog_test COMMAND native_watchdog_test)
  add_executable(native_setpoint_test test/native_setpoint_test.cpp)
  add_test(NAME native_setpoint_test COMMAND native_setpoint_test)
  add_executable(landing_request_test test/landing_request_test.cpp)
  add_test(NAME landing_request_test COMMAND landing_request_test)
  add_executable(payload_pd_test test/payload_pd_test.cpp)
  add_test(NAME payload_pd_test COMMAND payload_pd_test)
endif()
