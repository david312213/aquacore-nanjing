#!/bin/sh
# Read live planning-frame position. This never starts a flight task.
set -e
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
    echo '查看当前位置：sh flight/position.sh'
    echo '先运行传感器和定位终端；输出本次 fast_lio_local_enu 的 X/Y/Z，单位米。'
    echo '实时显示，Ctrl-C 退出；--once 只看一个样本。'
    exit 0
fi
. /opt/ros/noetic/setup.sh
. "$project_dir/build_ws/devel/setup.sh"
export ROS_MASTER_URI=http://127.0.0.1:11341
. "$project_dir/scripts/ros_network.sh"
echo '当前位置（fast_lio_local_enu，米）；航点只填写 x 和 y：'
case "${1:-}" in
    --once) exec rostopic echo -n 1 /airseafly/native/planning_odometry/pose/pose/position ;;
    '') exec rostopic echo /airseafly/native/planning_odometry/pose/pose/position ;;
    *) echo '支持 --once / --help' >&2; exit 2 ;;
esac
