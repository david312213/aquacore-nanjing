#!/usr/bin/env python3
"""Resolve startup tuning before ROS starts; dry runs never import or contact ROS."""
import argparse
from copy import deepcopy
import importlib.util
import json
import os
from pathlib import Path
import sys

import yaml

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT/'src/airseafly_real400_mission/src'))
from airseafly_real400_mission.map_tuning import (
    MAP_FIELDS, LIDAR_FIELDS, resolve_map_tuning, map_tuning_report,
    ego_launch_arguments, lidar_launch_arguments)
from airseafly_real400_mission.localization_tuning import (
    ARGUMENTS, localization_settings, localization_launch_arguments)
from airseafly_real400_mission.flight_configuration import base_configuration


def parse_arguments(argv):
    parser = argparse.ArgumentParser(description='启动时读取集中地图/雷达参数；--dry-run 不启动任何节点')
    parser.add_argument('mode', choices=('sensors', 'localization'))
    parser.add_argument('--flight-config', default=os.environ.get('AIRSEAFLY_FLIGHT_CONFIG',
                       str(PROJECT/'flight/flight.yaml')))
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--check', action='store_true', help='传感器入口：仅解析 ROS 节点')
    parser.add_argument('--blind', type=float, help='传感器入口：本次雷达近点过滤距离（米），覆盖 YAML')
    args, extra = parser.parse_known_args(argv)
    forwarded = []
    for value in extra:
        if ':=' not in value:
            parser.error('未知参数：'+value)
        key, value = value.split(':=', 1)
        if key == 'flight_config':
            args.flight_config = value
        else:
            forwarded.append(key+':='+value)
    if args.check and args.mode not in ('sensors', 'localization'):
        parser.error('--check 只适用于 sensors / localization')
    if args.blind is not None:
        if args.mode != 'sensors':
            parser.error('--blind 只适用于 sensors；修改后须重启传感器')
        forwarded.append('lidar_blind:='+str(args.blind))
    args.flight_config = str(Path(args.flight_config).expanduser().resolve())
    return args, forwarded


def common_configuration(path):
    with Path(path).open() as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or not isinstance(document.get('common', {}), dict):
        raise ValueError('飞行调参文件 common 必须是 YAML 对象')
    config = deepcopy(document.get('common', {}))
    config['_flight_configuration'] = dict(path=path, sources={})
    return config


def resolve_startup(args, forwarded):
    config = common_configuration(args.flight_config)
    fields = LIDAR_FIELDS if args.mode == 'sensors' else {}
    reverse = {argument:key for key, (argument, _param) in fields.items()}
    trial_reverse = {argument:key for key, argument in ARGUMENTS.items()}
    remaining = []
    for item in forwarded:
        name, value = item.split(':=', 1)
        if name in trial_reverse:
            key = trial_reverse[name]
            config.setdefault('localization', {})[key] = value if key.endswith('_mode') else yaml.safe_load(value)
            continue
        if name not in reverse:
            remaining.append(item)
            continue
        key = reverse[name]
        group = config.setdefault('lidar', {})
        group[key] = yaml.safe_load(value)
        prefix = 'lidar.'
        config['_flight_configuration']['sources'][prefix+key] = '命令行 '+name
    effective = resolve_map_tuning(config, PROJECT) if args.mode != 'localization' else {}
    report = map_tuning_report(config, PROJECT) if args.mode != 'localization' else {}
    arguments = lidar_launch_arguments(effective, PROJECT) if args.mode == 'sensors' else []
    effective['localization'] = localization_settings(config)
    arguments += localization_launch_arguments(effective['localization'], args.mode)
    if args.mode == 'sensors':
        arguments.append('flight_config:='+args.flight_config)
        effective['rc_payload'] = dict(base_configuration()['rc_payload'], **config.get('rc_payload', {}))
        cameras = config.get('sensors', {}).get('cameras_enabled', False)
        if type(cameras) is not bool:
            raise ValueError('sensors.cameras_enabled 必须是 true 或 false')
        if not any(item.startswith('cameras_enabled:=') for item in remaining):
            arguments.append('cameras_enabled:='+str(cameras).lower())
    return effective, report, arguments+remaining


def main(argv=None):
    args, forwarded = parse_arguments(sys.argv[1:] if argv is None else argv)
    effective, report, launch_args = resolve_startup(args, forwarded)
    script = PROJECT/'scripts'/('start_base_sensors.sh' if args.mode == 'sensors'
                               else 'start_ego_px4_localization.sh')
    if args.check:
        launch_args.insert(0, '--check')
    if args.dry_run:
        result = dict(
            dry_run=True, start_action=args.mode+'_only', takeoff=False, payload=False, landing=False)
        result['localization'] = effective['localization']
        if args.mode == 'sensors':
            result['rc_payload'] = effective['rc_payload']
        result.update(flight_config=args.flight_config, mapping=report,
                      launch_arguments=launch_args)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return
    if args.mode == 'sensors':
        print('雷达启动参数：blind=%.3f m（%s）。' %
              (effective['lidar']['blind_m'], report['lidar.blind_m']['source']), flush=True)
    os.execv('/bin/sh', ['sh', str(script), '--resolved-tuning', *launch_args])


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, KeyError) as error:
        print('启动参数错误：'+str(error), file=sys.stderr)
        sys.exit(2)
