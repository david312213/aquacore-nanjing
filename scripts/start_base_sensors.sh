#!/bin/sh
# Environment adapter for the single native ROS launch file.
set -e
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
# Resolve the shared YAML before loading ROS. The private branch receives only
# launch arguments that the read-only Python adapter has already validated.
if [ "${1:-}" = --resolved-tuning ]; then
    shift
else
    exec "${AIRSEAFLY_PYTHON:-python3}" "$project_dir/scripts/startup_tuning.py" sensors "$@"
fi
base_sensors_check=0
case "${1:-}" in
    -h|--help)
        cat <<'HELP'
基础组合入口：MAVROS、MID360、FAST-LIO、9通道MAIN6/7舵机控制及两条200Hz请求。
用法：sh scripts/start_base_sensors.sh
只解析节点、不启动：start_base_sensors.sh --check
可传launch参数，例如：start_base_sensors.sh imu_interval_us:=5000
实际launch：airseafly_real400 base_sensors.launch
本终端保持运行，Ctrl+C停止这组基础节点；不启动px4ctrl、EGO、YOLO或投放任务。
HELP
        exit 0 ;;
    --check) base_sensors_check=1; shift ;;
esac

# A no-argument function keeps launch arguments out of catkin setup.
load_base_ros_environment() {
    . /opt/ros/noetic/setup.sh
    . "$project_dir/build_ws/devel/setup.sh"
}
load_base_ros_environment
export ROS_MASTER_URI=http://127.0.0.1:11341
. "$project_dir/scripts/ros_network.sh"

if [ "$base_sensors_check" -eq 1 ]; then
    exec roslaunch --nodes airseafly_real400 base_sensors.launch "$@"
fi

# Preserve the existing start_sensor.sh checks; no new flight conditions.
if pgrep -x QGroundControl >/dev/null || pgrep -x mavros_node >/dev/null; then
    echo 'Close QGroundControl and any existing MAVROS before taking the PX4 USB port.' >&2
    exit 2
fi
if [ ! -r "$project_dir/config/aircraft/MID360_config.json" ]; then
    echo 'MID-360 host-specific configuration is missing.' >&2
    exit 2
fi
mkdir -p "$project_dir/data/fastlio/Log" "$project_dir/data/fastlio/PCD"
# Informational only: the address a remote RViz should connect to right now.
base_lan_ip=$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<NF;i++) if($i=="src"){print $(i+1); exit}}')
if [ -n "${ROS_HOSTNAME:-}" ] && [ -n "$base_lan_ip" ]; then
    echo "远程 RViz：Mac 上运行 bash tools/remote_rviz/mac_rviz.sh $base_lan_ip" >&2
fi
exec roslaunch -p 11341 airseafly_real400 base_sensors.launch "$@"
