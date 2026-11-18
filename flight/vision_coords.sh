#!/bin/sh
# Print aircraft XYZ, red-point center XY and delta XY using shared detection.
# Requires the existing sensors/localization terminals; no flight or payload IO.
set -e
flight_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec sh "$flight_dir/red_point.sh" --no-window --confirmed-only "$@"
