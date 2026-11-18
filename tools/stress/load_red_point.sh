#!/bin/sh
# Ground-only load: the same red_point_worker.py + flight/red_point.yaml that delivery-route
# starts in flight. Detection only: no flight, payload or servo command. Ctrl-C stops it and
# its own DF200 launch. The fixed session matches the topic stress_monitor.py watches.
# Usage: sh load_red_point.sh [config.yaml]   (default flight/red_point.yaml)
set -e
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
session=5f3a0c1e-2b7d-4c41-9e8a-1a2b3c4d5e6f
config=${1:-$project_dir/flight/red_point.yaml}
device=$(sed -n 's/^ *video_device: *\([^ #]*\).*/\1/p' "$config" | head -1)
if [ ! -e "$device" ]; then
    echo "下视相机不存在：$device" >&2
    echo "先修好相机（lsusb 应看到 2993:0858），否则这一阶段测不到识别负载。" >&2
    exit 1
fi
. /opt/ros/noetic/setup.sh
. "$project_dir/build_ws/devel/setup.sh"
export ROS_MASTER_URI=http://127.0.0.1:11341
. "$project_dir/scripts/ros_network.sh"
exec "${AIRSEAFLY_PYTHON:-python3}" "$project_dir/src/airseafly_real400_mission/scripts/red_point_worker.py" \
    --config "$config" --session "$session"
