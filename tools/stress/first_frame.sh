#!/bin/sh
# Cold-start the red-point detector like a mission image window and report its first frames.
# Needs the three flight terminals running. Receive-only; never arms or commands.
# Usage: sh tools/stress/first_frame.sh [trials] [seconds_each]
set -e
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
. /opt/ros/noetic/setup.sh
. "$project_dir/build_ws/devel/setup.sh"
export ROS_MASTER_URI=http://127.0.0.1:11341
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
exec "${AIRSEAFLY_PYTHON:-python3}" "$project_dir/tools/stress/first_frame_probe.py" "$@"
