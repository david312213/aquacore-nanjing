#!/bin/sh
# Only subscribe to planning_odometry. No takeoff, control or payload commands.
set -e
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
route_python=${AIRSEAFLY_PYTHON:-python3}
if [ -z "${AIRSEAFLY_PYTHON:-}" ] && [ -x "$project_dir/flight/.venv/bin/python" ]; then
    route_python="$project_dir/flight/.venv/bin/python"
fi
route_offline=false
for route_arg in "$@"; do
    case "$route_arg" in --demo|--help|-h) route_offline=true ;; esac
done
if [ "$route_offline" = false ]; then
    . /opt/ros/noetic/setup.sh
    . "$project_dir/build_ws/devel/setup.sh"
    export ROS_MASTER_URI=http://127.0.0.1:11341
    . "$project_dir/scripts/ros_network.sh"
fi
exec "$route_python" "$project_dir/src/airseafly_real400_mission/scripts/record_drop_points.py" "$@"
