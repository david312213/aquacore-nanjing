#!/bin/sh
# Handheld observation only: no takeoff, motion or payload commands.
set -e
flight_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec sh "$flight_dir/red_point.sh" --target red --confirmed-only --print-hz 1 "$@"
