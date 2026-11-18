#!/usr/bin/env python3
"""Nanjing workspace environment adapter; tuning lives in flight/flight.yaml."""
import os
from pathlib import Path
import sys
PROJECT = Path(__file__).resolve().parents[1]


def main(argv=None, flight_config=None):
    args = list(sys.argv[1:] if argv is None else argv)
    action = args.pop(0) if args else 'help'
    tuning = Path(flight_config or PROJECT/'flight/flight.yaml').resolve()
    if action not in ('sensors', 'localization', 'controller', 'delivery-route', 'drop-test', 'drop-points', 'hover-drop',
                      'single-drop'):
        raise ValueError('未知入口：'+action)
    if action in ('delivery-route', 'drop-test', 'drop-points', 'hover-drop', 'single-drop'):
        script_name = {'delivery-route': 'delivery_route.py', 'drop-test': 'drop_test.py',
                       'drop-points': 'drop_points.py', 'single-drop': 'single_drop.py',
                       'hover-drop': 'hover_drop.py'}[action]
        return execute_command([sys.executable,
            str(PROJECT/'src/airseafly_real400_mission/scripts'/script_name),
            '--flight-config', str(tuning), *args],
            offline=any(arg in ('--dry-run', '--show-config', '--help', '-h') for arg in args))
    forwarded, index = [], 0
    while index < len(args):
        arg = args[index]
        if arg == '--flight-config':
            index += 1
            if index == len(args):
                raise ValueError('--flight-config 需要路径')
            tuning = Path(args[index]).expanduser().resolve()
        elif arg.startswith('--flight-config='):
            tuning = Path(arg.split('=', 1)[1]).expanduser().resolve()
        else:
            forwarded.append('--dry-run' if arg == '--show-config' else arg)
        index += 1
    os.environ['ROS_LOG_DIR'] = str(PROJECT/'data/ros')
    os.environ['AIRSEAFLY_FLIGHT_CONFIG'] = str(tuning)
    if action == 'controller':
        if any(arg in ('--dry-run', '--help', '-h') for arg in forwarded):
            import json
            print(json.dumps(dict(dry_run=True, start_action='controller_only',
                flight_config=str(tuning), recording=False, cameras_enabled=False,
                takeoff=False, payload=False, landing=False,
                controller_config=str(PROJECT/'config/px4ctrl.yaml'),
                launch_arguments=['native_output_rate_hz:=50', 'native_stop_on_mode_exit:=true']+
                    [arg for arg in forwarded if arg not in ('--dry-run', '--help', '-h')]),
                ensure_ascii=False, indent=2))
            return 0
    else:
        forwarded = ['--flight-config', str(tuning), *forwarded]
    script = PROJECT/'scripts'/dict(sensors='start_base_sensors.sh',
        localization='start_ego_px4_localization.sh', controller='start_ego_px4_controller.sh')[action]
    os.execv('/bin/sh', ['sh', str(script), *forwarded])


def execute_command(command_args, offline=False):
    """Load only this workspace for a live command; previews need no ROS."""
    if offline:
        os.execvp(command_args[0], command_args)
    setup = PROJECT/'build_ws/devel/setup.sh'
    if not Path('/opt/ros/noetic/setup.sh').is_file() or not setup.is_file():
        raise ValueError('本机没有机载ROS环境；本地可追加 --dry-run 查看配置。需在机载编译环境启动飞行。')
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11341'
    os.environ['ROS_LOG_DIR'] = str(PROJECT/'data/ros')
    sys.path.insert(0, str(PROJECT/'src/airseafly_real400_mission/src'))
    from airseafly_real400_mission.ros_network import apply_ros_address
    apply_ros_address(os.environ)
    # Pass every path/argument separately; never interpolate user XY or --set
    # into shell text. Keep command arguments out of catkin's setup function.
    script = 'setup_file=$1; shift; load_env() { . /opt/ros/noetic/setup.sh; . "$setup_file"; }; load_env; exec "$@"'
    os.execv('/bin/sh', ['sh', '-ec', script, 'flight', str(setup), *command_args])


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(2)
