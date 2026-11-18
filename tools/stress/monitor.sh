#!/bin/sh
# Receive-only stress monitor on the flight master. Publishes nothing; never arms.
set -e
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
case " $* " in
    *" --no-ros "*|*" --summarize "*|*" --help "*|*" -h "*) ;;
    *)
        . /opt/ros/noetic/setup.sh
        . "$project_dir/build_ws/devel/setup.sh"
        export ROS_MASTER_URI=http://127.0.0.1:11341
        . "$project_dir/scripts/ros_network.sh"
        ;;
esac
exec "${AIRSEAFLY_PYTHON:-python3}" "$project_dir/tools/stress/stress_monitor.py" "$@"
