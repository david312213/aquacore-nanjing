#!/bin/sh
# Standalone camera preview/recording; no ROS environment or flight services.
set -e
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
exec "${AIRSEAFLY_PYTHON:-python3}" "$project_dir/tools/camera/record_video.py" "$@"
