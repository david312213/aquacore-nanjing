#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
load_ros_environment() {
    set +u
    . /opt/ros/noetic/setup.sh
    . "$project_dir/build_ws/devel/setup.sh"
}
load_ros_environment
set -u
export ROS_MASTER_URI=http://127.0.0.1:11341
. "$project_dir/scripts/ros_network.sh"
exec python3 "$project_dir/tools/mapping/live_octomap_monitor.py" \
    --config "$project_dir/config/fastlio.yaml" \
    /airseafly/lio/cloud_body:=/cloud_registered_body \
    /airseafly/lio/raw_odom:=/Odometry "$@"
