#!/bin/sh
# Explicit dependency installation, without changing flight-controller settings.
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if [ "${1:---check}" != '--install' ]; then
    python3 "$project_dir/scripts/check_project.py" --environment
    echo '安装现有 ROS Noetic 工作环境所需依赖：sh scripts/setup_ubuntu.sh --install'
    exit 0
fi
test -f /opt/ros/noetic/setup.sh || { echo '请先准备 Ubuntu 20.04 / ROS Noetic 环境。' >&2; exit 2; }
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config python3-venv python3-pip \
    python3-rosdep python3-nose python3-numpy python3-yaml python3-opencv \
    libeigen3-dev libpcl-dev libapr1-dev libboost-all-dev \
    ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-usb-cam \
    ros-noetic-cv-bridge \
    ros-noetic-octomap-server liboctomap-dev
load_ros_environment() { set +u; . /opt/ros/noetic/setup.sh; }
load_ros_environment
set -u
python3 "$project_dir/scripts/prepare_workspace.py"
rosdep install --from-paths "$project_dir/build_ws/src" --ignore-src --rosdistro noetic -r -y
