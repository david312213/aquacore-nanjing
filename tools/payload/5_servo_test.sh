#!/bin/sh
set -e
flight_dir=$(CDPATH= cd -P -- "$(dirname -- "$0")" && pwd)
flight_python=${AIRSEAFLY_PYTHON:-python3}
if [ -z "${AIRSEAFLY_PYTHON:-}" ] && [ -x "$flight_dir/.venv/bin/python" ]; then flight_python="$flight_dir/.venv/bin/python"; fi
exec "$flight_python" "$flight_dir/servo_test.py" "$@"
