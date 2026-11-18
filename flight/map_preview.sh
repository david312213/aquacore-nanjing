#!/bin/sh
set -e
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
. /opt/ros/noetic/setup.sh
. "$project_dir/build_ws/devel/setup.sh"
export ROS_MASTER_URI=http://127.0.0.1:11341
. "$project_dir/scripts/ros_network.sh"
exec python3 "$project_dir/scripts/start_map_preview.py" "$@"
