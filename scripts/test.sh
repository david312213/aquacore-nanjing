#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
exec "${AIRSEAFLY_PYTHON:-python3}" "$project_dir/scripts/run_tests.py" "$@"
