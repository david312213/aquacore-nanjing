#!/bin/sh
# Ground-only load: ego_worker.launch with the map/motion arguments of the 10-27 22:55 crash
# (data/mission/20261027_225452_9781). No goal is sent. Its position_cmd goes to a session
# topic that only the mission manager relays; px4ctrl listens on /position_cmd, so nothing
# reaches PX4. Ctrl-C stops it; run again to repeat the startup transient.
set -e
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
. /opt/ros/noetic/setup.sh
. "$project_dir/build_ws/devel/setup.sh"
export ROS_MASTER_URI=http://127.0.0.1:11341
. "$project_dir/scripts/ros_network.sh"
session=stress_$(date +%H%M%S)
exec roslaunch airseafly_real400_mission ego_worker.launch "session:=$session" \
    yaw:=0.0 height_origin_z:=0.0 max_vel:=0.5 max_acc:=0.25 \
    odometry_topic:=/airseafly/native/planning_odometry \
    cloud_topic:=/airseafly/native/planning_cloud world_frame:=fast_lio_local_enu \
    map_size_x:=100.0 map_size_y:=50.0 map_size_z:=3.0 map_resolution:=0.18 \
    map_inflation:=0.36 map_max_height:=2.4 map_min_height:=-1.0 "$@"
