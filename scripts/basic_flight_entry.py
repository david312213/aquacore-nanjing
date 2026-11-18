#!/usr/bin/env python3
"""Basic flight commands sharing the existing workspace/ROS adapter."""
import json
from pathlib import Path
import sys

from flight_entry import PROJECT, execute_command

BASIC_CONFIG = PROJECT/'flight/flight.yaml'
PACKAGE = PROJECT/'src/airseafly_real400_mission'


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    action = args.pop(0)
    if action == 'takeoff':
        if any(arg in ('-h', '--help') for arg in args):
            print('sh flight/takeoff.sh [--height 米] [--takeoff-speed 米/秒] '
                  '[--takeoff-acceleration 米/秒²] [--flight-config YAML] [--dry-run]')
            print('默认 flight/flight.yaml；原地起飞后持续保持，关闭任务终端不取消已建立的保持流。')
            return 0
    if action in ('takeoff', 'fly-points', 'hold', 'status'):
        command = 'run' if action == 'fly-points' else action
        return execute_command([sys.executable, str(PACKAGE/'scripts/fly_waypoints.py'),
                                command, *args], offline=any(
            arg in ('--dry-run', '--show-config', '--help', '-h') for arg in args))
    if action == 'safe-land':
        if any(arg in ('-h', '--help') for arg in args):
            print('sh flight/safe_land.sh [--dry-run]')
            print('调用现有 /px4ctrl/force_land，原生模式请求 PX4 AUTO.LAND；此入口不直接停桨。')
            print('仅启动降落请求与状态观察，不启动传感器、MAVROS 或第二套控制器。')
            return 0
        if any(arg not in ('--dry-run', '--show-config') for arg in args):
            raise ValueError('safe_land.sh 仅支持 --dry-run / --help')
        if args:
            print(json.dumps(dict(dry_run=True, service='/px4ctrl/force_land',
                                  native_mode='AUTO.LAND', starts_controller=False), ensure_ascii=False))
            return 0
        return execute_command(['roslaunch', 'airseafly_real400_mission', 'safe_land.launch'])
    raise ValueError('未知基础命令：'+action)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, OSError, RuntimeError) as exc:
        print('基础飞行命令未能继续：'+str(exc), file=sys.stderr)
        sys.exit(2)
