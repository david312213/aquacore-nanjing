#!/bin/sh
set -e
native_home=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec sh "$native_home/start_px4_native_controller.sh" native_output_rate_hz:=50 native_stop_on_mode_exit:=true "$@"
