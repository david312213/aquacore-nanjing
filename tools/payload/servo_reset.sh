#!/bin/sh
# 每次执行关闭 MAIN6、MAIN7 到配置的900 μs并保持；--dry-run 只预览。
set -e
flight_dir=$(CDPATH= cd -P -- "$(dirname -- "$0")" && pwd)
exec sh "$flight_dir/5_servo_test.sh" reset "$@"
