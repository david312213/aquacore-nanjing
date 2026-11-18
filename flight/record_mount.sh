#!/bin/sh
# Capture MID360/PX4 data using the existing sensor terminal; no flight commands.
set -e
mount_project=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
case "${1:-}" in
    -h|--help|--dry-run)
        exec "${AIRSEAFLY_PYTHON:-python3}" "$mount_project/tools/calibration/mid360_px4/record.py" "$@" ;;
esac
load_mount_environment() {
    . /opt/ros/noetic/setup.sh
    . "$mount_project/build_ws/devel/setup.sh"
}
load_mount_environment
export ROS_MASTER_URI=http://127.0.0.1:11341
. "$mount_project/scripts/ros_network.sh"
exec python3 -u "$mount_project/tools/calibration/mid360_px4/record.py" "$@"
