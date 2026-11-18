#!/bin/sh
# Independent observation entry. Only its own DF200 launch is stopped on exit.
set -e
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
red_point_python=${AIRSEAFLY_PYTHON:-python3}
red_point_offline=false
for red_point_arg in "$@"; do
    case "$red_point_arg" in --dry-run|--image|--image=*|--help|-h) red_point_offline=true ;; esac
done
if [ "$red_point_offline" = false ]; then
    . /opt/ros/noetic/setup.sh
    . "$project_dir/build_ws/devel/setup.sh"
    export ROS_MASTER_URI=http://127.0.0.1:11341
    . "$project_dir/scripts/ros_network.sh"
fi
exec "$red_point_python" "$project_dir/src/airseafly_real400_mission/scripts/red_point_preview.py" "$@"
