#!/bin/sh
# Build only. Never launch sensors, controllers or a flight task.
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if [ ! -f /opt/ros/noetic/setup.sh ]; then
    echo '需要 Ubuntu / ROS Noetic 构建环境；本机可运行 scripts/test.sh 离线检查。' >&2
    exit 2
fi
load_ros_environment() { set +u; . /opt/ros/noetic/setup.sh; }
load_ros_environment
set -u
python3 "$project_dir/scripts/prepare_workspace.py"
sdk_source="$project_dir/src/realflight_modules/mid360_fastlio/src/livox_ros_driver2/Livox-SDK2"
cmake -S "$sdk_source" -B "$project_dir/build_ws/livox-sdk" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$project_dir/runtime/livox-sdk"
cmake --build "$project_dir/build_ws/livox-sdk" --parallel "${AQUACORE_BUILD_JOBS:-2}"
cmake --install "$project_dir/build_ws/livox-sdk"
cd "$project_dir/build_ws"
catkin_make -j"${AQUACORE_BUILD_JOBS:-2}" \
    -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DAQUACORE_FASTLIO_DATA_DIR="$project_dir/data/fastlio" \
    -DLIVOX_SDK_STATIC="$project_dir/runtime/livox-sdk/lib/liblivox_lidar_sdk_static.a" "$@"
