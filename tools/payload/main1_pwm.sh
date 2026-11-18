#!/bin/sh
# MAIN1 PWM 保持测试；不带数值进入交互模式，--dry-run 仅离线预览。
set -e
pwm_dir=$(CDPATH= cd -P -- "$(dirname -- "$0")" && pwd)
pwm_python=${AIRSEAFLY_PYTHON:-python3}
if [ -z "${AIRSEAFLY_PYTHON:-}" ] && [ -x "$pwm_dir/.venv/bin/python" ]; then pwm_python="$pwm_dir/.venv/bin/python"; fi
exec "$pwm_python" "$pwm_dir/main1_pwm.py" "$@"
