#!/bin/sh
set -e
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
exec "${AIRSEAFLY_PYTHON:-python3}" "$project_dir/scripts/flight_entry.py" hover-drop "$@"
