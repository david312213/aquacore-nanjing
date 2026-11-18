#!/bin/sh
set -e
native_project=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if [ "${1:-}" = --resolved-tuning ]; then
    shift
else
    exec "${AIRSEAFLY_PYTHON:-python3}" "$native_project/scripts/startup_tuning.py" localization "$@"
fi
localization_check=0
if [ "${1:-}" = --check ]; then
    localization_check=1
    shift
fi
native_setup="$native_project/build_ws/devel/setup.sh"
load_native_environment() {
    . /opt/ros/noetic/setup.sh
    . "$native_setup"
}
load_native_environment
export ROS_MASTER_URI=http://127.0.0.1:11341
. "$native_project/scripts/ros_network.sh"
if [ "$localization_check" -eq 1 ]; then
    exec roslaunch --nodes airseafly_real400_mission native_localization.launch publish_planning_interfaces:=true "$@"
fi
exec roslaunch airseafly_real400_mission native_localization.launch publish_planning_interfaces:=true "$@"
