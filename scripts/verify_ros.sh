#!/bin/sh
# Parse launch files without starting a ROS master, nodes or actuators.
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
load_ros_environment() {
    set +u
    . /opt/ros/noetic/setup.sh
    . "$project_dir/build_ws/devel/setup.sh"
}
load_ros_environment
set -u
export ROS_MASTER_URI=http://127.0.0.1:11341
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
roslaunch --nodes airseafly_real400 base_sensors.launch
roslaunch --nodes airseafly_real400_mission native_localization.launch publish_planning_interfaces:=true
roslaunch --nodes px4ctrl run_ctrl.launch controller_config:="$project_dir/config/px4ctrl.yaml" native_output_rate_hz:=50 native_stop_on_mode_exit:=true
roslaunch --nodes airseafly_real400_mission ego_worker.launch session:=offline yaw:=0.0
roslaunch --nodes airseafly_real400_mission safe_land.launch
roslaunch --nodes airseafly_real400_mission red_point_camera.launch owner_ns:=offline_preview \
    image_topic:=/down_camera/image_raw camera_info_topic:=/down_camera/camera_info \
    video_device:=/dev/video0 autoexposure:=true exposure:=100
python3 -c 'import numpy; import unittest; unittest.main(module=None)' discover \
    -s "$project_dir/src/airseafly_real400_mission/test" -p test_planning_launch.py
