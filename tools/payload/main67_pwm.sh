#!/bin/sh
# MAIN6/7 分别测试：交互输入“6 1500”或“7 1500”；也可 --port 6 1500。
set -e
pwm_dir=$(CDPATH= cd -P -- "$(dirname -- "$0")" && pwd)
exec sh "$pwm_dir/main1_pwm.sh" --main67 "$@"
