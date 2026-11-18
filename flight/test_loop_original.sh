#!/bin/sh
# Same ordinary-route parameters; one closed lap using the original timing.
set -e
flight_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec sh "$flight_dir/fly_points.sh" "$@" --timing original \
    --set navigation.backend=fixed_min_snap --set navigation.laps=1
