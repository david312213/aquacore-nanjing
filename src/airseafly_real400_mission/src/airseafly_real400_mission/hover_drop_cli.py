"""Native takeoff followed by explicit, numbered manual releases while hovering."""
import argparse
import importlib.util
import json
from pathlib import Path
import uuid

from .configuration import project_root, runtime_paths
from .delivery_route_cli import manager_client, resolve_config
from . import native_parameters
from .payload import actuator_command, configured_slots
from .payload_console import run_hover_drop_console
from .rpc import request


def parser():
    result = argparse.ArgumentParser(description='原地起飞悬停 → 输入 1/2/3/4 手动投放；q 当前位置降落')
    result.add_argument('--config', help='原机器配置文件')
    result.add_argument('--flight-config', type=Path, default=project_root()/'flight/flight.yaml')
    result.add_argument('--set', dest='overrides', action='append', default=[])
    result.add_argument('--json', action='store_true')
    result.add_argument('--verbose', action='store_true')
    native_parameters.add_arguments(result)
    return result


def preview(config):
    options = []
    for slot in configured_slots(config['payload']):
        command = actuator_command(slot, 'release', config['payload'])
        options.append({key: command[key] for key in ('slot', 'main_output', 'channel', 'pwm_us')})
    return dict(mode='hover_drop', takeoff_mode='basic_takeoff',
                height_m=config['flight']['cruise_height_m'], height_reference='takeoff_origin_z',
                config_profile='delivery-route', payload_options=options,
                automatic_release=False, automatic_reset=False, visual=False, navigation=False,
                stages=['takeoff', 'manual_release_menu'],
                controls={'1/2/3/4': 'release_selected_once', 'h 米数': 'change_height',
                          'q': 'land_current_position', 'Ctrl-C/EOF': 'exit_menu_keep_hover'},
                parameters=native_parameters.summary(config))


def show_preview(value, as_json=False):
    if as_json:
        print(json.dumps(value, ensure_ascii=False, indent=2))
        return
    print('原地起飞 %.2f m；起飞流程完成后持续悬停，输入编号并回车投放。' % value['height_m'])
    for row in value['payload_options']:
        print('  %d → MAIN%d，%g μs' % (row['slot'], row['main_output'], row['pwm_us']))
    print('每次输入只发送一次，重复编号也按本次输入发送；不自动关闭复位。')
    print('h 米数：调整悬停高度；q：当前位置降落；菜单中 Ctrl-C/EOF：退出菜单并保持悬停。')


def land_current():
    # Import ROS only for the user's explicit q command. The existing landing
    # observer requests once and waits for actual landing/disarm confirmation.
    path = project_root()/'src/airseafly_real400_mission/scripts/safe_land.py'
    spec = importlib.util.spec_from_file_location('hover_drop_safe_land', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.main(stop_on_terminal=True)


def watch_completed(client, paths, args, config, response, task_id, height_id=None):
    completed = {}
    extra = {} if height_id is None else {'expected_height_id': height_id}
    code = client.watch(paths, args, config, finish_task=True,
                        after=response.get('event_start', 0), completed_status=completed,
                        expected_task_id=task_id, **extra)
    output = completed.get('output') or {}
    # watch returns zero after Ctrl-C too. Only its completion snapshot opens
    # (or resumes) input; this adds no flight wait or position criterion.
    if code or not (output.get('done') or output.get('state') == 'COMPLETE'):
        print('当前运动未完成，投放菜单已结束。')
        return code or 1
    return 0


def main(argv=None):
    args = parser().parse_args(argv)
    config = resolve_config(args)
    value = preview(config)
    value['dry_run'] = args.dry_run
    show_preview(value, args.json)
    if args.dry_run:
        return 0
    paths = runtime_paths()
    client = manager_client()
    client.connect_or_start(paths, config)
    timeout = float(config['payload']['ack_timeout_s']) + 5.
    result = request(paths['socket'], dict(command='start', mode='basic_takeoff',
                                         resolved_config=config), timeout=timeout)
    if not result.get('ok'):
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 1
    task_id = result['task_id']
    code = watch_completed(client, paths, args, config, result, task_id)
    if code:
        return code

    def send(slot, action):
        # Use the existing manual payload transport and receipt policy. A new
        # explicit input is one request; no automatic retry/reset is added.
        return request(paths['socket'], dict(command='payload', slot=slot, action=action,
                                            resolved_config=config), timeout=timeout)

    def change_height(height):
        height_id = str(uuid.uuid4())
        response = request(paths['socket'], dict(command='manual_height', task_id=task_id,
                           request_id=height_id, height_m=height))
        if not response.get('ok'):
            print(json.dumps(response, ensure_ascii=False, indent=2))
            return 1
        return watch_completed(client, paths, args, config, response, task_id, height_id)

    return run_hover_drop_console(config, send, land_current, change_height=change_height)
