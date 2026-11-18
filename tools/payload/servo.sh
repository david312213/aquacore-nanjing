#!/bin/sh
# 地面开合测试：不带参数进入已配置投放编号菜单（支持 1/2/3/4）。
# 单次动作：sh tools/payload/servo.sh open 2（打开）或 close 2（闭合）。
# 原 release / lock 命令仍可使用；开合值读取现有 payload 配置。
# 当前双舵机配置持续保持输出；--dry-run 只预览，不动作。
set -e
servo_dir=$(CDPATH= cd -P -- "$(dirname -- "$0")" && pwd)
exec sh "$servo_dir/5_servo_test.sh" "$@"
