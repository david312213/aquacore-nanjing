#!/bin/sh
# 持久投放操作：编号1～4依次 MAIN6/1190、MAIN7/1190、MAIN6/1350、MAIN7/1350。
# 使用配送配置和现有 MAV_CMD_DO_SET_ACTUATOR（187）；退出后不自动恢复。
# lock 关闭对应舵机到900；--dry-run 只显示命令；退出不自动回位。
set -e
flight_dir=$(CDPATH= cd -P -- "$(dirname -- "$0")" && pwd)
exec sh "$flight_dir/4_delivery_route.sh" payload "$@"
