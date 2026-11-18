#!/bin/sh
set -e
native_project=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
native_setup="$native_project/build_ws/devel/setup.sh"
native_controller_config="$native_project/config/px4ctrl.yaml"
load_native_environment() {
    . /opt/ros/noetic/setup.sh
    . "$native_setup"
}
load_native_environment
export ROS_MASTER_URI=http://127.0.0.1:11341
. "$native_project/scripts/ros_network.sh"
printf 'px4ctrl 参数文件：%s\n' "$native_controller_config" >&2
exec roslaunch px4ctrl run_ctrl.launch "controller_config:=$native_controller_config" "$@"
