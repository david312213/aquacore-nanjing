"""Resolve startup-only mapping tuning, preserving the local aircraft's legacy values.

This module never writes calibration files, touches ROS or changes live parameters.
"""
from math import isfinite
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml

from .configuration import project_root

MAP_SIZE_DEFAULTS = dict(size_x_m=100., size_y_m=50., size_z_m=3.)
MAP_FIELDS = {
    **{'size_'+axis+'_m': ('map_size_'+axis, 'grid_map/map_size_'+axis) for axis in 'xyz'},
    'resolution_m': ('map_resolution', 'grid_map/resolution'),
    'inflation_m': ('map_inflation', 'grid_map/obstacles_inflation'),
    'max_height_m': ('map_max_height', 'grid_map/virtual_ceil_height'),
    'min_height_m': ('map_min_height', 'grid_map/virtual_floor_height'),
}
LIDAR_FIELDS = {
    'blind_m': ('lidar_blind', 'preprocess/blind'),
    'surface_voxel_m': ('lidar_surface_voxel', 'filter_size_surf'),
    'map_voxel_m': ('lidar_map_voxel', 'filter_size_map'),
    'point_filter_num': ('lidar_point_filter_num', 'point_filter_num'),
}
PLANNER_XML = 'src/planner/plan_manage/launch/advanced_param_exp.xml'
LIO_LAUNCH = 'src/realflight_modules/mid360_fastlio/src/FAST_LIO/launch/mapping_mid360.launch'


def _legacy_param(path, parameter):
    tree = ET.parse(path).getroot()
    for item in tree.iter('param'):
        if item.get('name') != parameter:
            continue
        value = item.get('value', '')
        match = re.fullmatch(r'\$\(arg ([^)]+)\)', value)
        if match:
            argument = next((node for node in tree.findall('arg')
                             if node.get('name') == match.group(1)), None)
            value = argument.get('default') if argument is not None else None
        if value is not None:
            return yaml.safe_load(value)
    raise ValueError('无法读取旧参数 %s: %s' % (parameter, path))


def _sections(config):
    if not isinstance(config, dict):
        raise ValueError('飞行配置必须是对象')
    planning = config.get('planning', {})
    if not isinstance(planning, dict):
        raise ValueError('planning 必须是对象')
    map_config, lidar = planning.get('map', {}), config.get('lidar', {})
    if not isinstance(map_config, dict) or not isinstance(lidar, dict):
        raise ValueError('planning.map 和 lidar 必须是对象')
    return map_config, lidar


def resolve_map_tuning(config, project=None):
    """Return effective planning.map/lidar values; missing keys use this install's files.

    null preserves legacy values. The new floor defaults to disabled for older
    snapshots/installations that never had a lower boundary.
    """
    root = Path(project or project_root())
    map_config, lidar = _sections(config)
    result = {'planning': {'map': {}}, 'lidar': {}}
    for key, (_argument, parameter) in MAP_FIELDS.items():
        value = map_config.get(key)
        if value is None:
            value = (MAP_SIZE_DEFAULTS[key] if key in MAP_SIZE_DEFAULTS else
                     -1.0 if key == 'min_height_m' else _legacy_param(root/PLANNER_XML, parameter))
        result['planning']['map'][key] = value
    for key, (_argument, parameter) in LIDAR_FIELDS.items():
        value = lidar.get(key)
        if value is None:
            if key == 'blind_m':
                with (root/'config/fastlio.yaml').open() as stream:
                    value = yaml.safe_load(stream)['preprocess']['blind']
            else:
                value = _legacy_param(root/LIO_LAUNCH, parameter)
        result['lidar'][key] = value
    _validate_values(result)
    return result


def _validate_values(config):
    groups = [('planning.map', config['planning']['map']), ('lidar', config['lidar'])]
    for group, fields in groups:
        for key, value in fields.items():
            name = group+'.'+key
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
                raise ValueError(name+' 必须是有限数值')
            if key == 'point_filter_num':
                if not isinstance(value, int) or value < 1:
                    raise ValueError(name+' 必须是大于等于 1 的整数')
            elif key in ('inflation_m', 'blind_m'):
                if value < 0:
                    raise ValueError(name+' 不得小于 0')
            elif key not in ('min_height_m', 'max_height_m') and value <= 0:
                raise ValueError(name+' 必须大于 0')
    # Negative values disable the respective boundary, as for the legacy ceiling.
    mapping = config['planning']['map']
    floor, ceiling = mapping.get('min_height_m', -1), mapping.get('max_height_m', -1)
    if floor >= 0 and ceiling >= 0 and floor >= ceiling:
        raise ValueError('planning.map.min_height_m 必须低于 max_height_m')


def validate_map_tuning(config, project=None):
    resolve_map_tuning(config, project)


def freeze_map_tuning(config, project=None):
    """Freeze the values into a task snapshot so later edits cannot change its map."""
    resolved = resolve_map_tuning(config, project)
    config.setdefault('planning', {}).setdefault('map', {}).update(resolved['planning']['map'])
    config.setdefault('lidar', {}).update(resolved['lidar'])
    return config


def ego_map_values(config, overrides=None, two_waypoint_gate=False, project=None):
    """Resolve this worker's map without changing the shared/task snapshot."""
    resolved = resolve_map_tuning(config, project)
    values = dict(resolved['planning']['map'])
    values.update({key: value for key, value in (overrides or {}).items()
                   if key in MAP_FIELDS and value is not None})
    _validate_values({'planning': {'map': values}, 'lidar': {}})
    return values


def ego_launch_arguments(config, overrides=None, two_waypoint_gate=False):
    """Forward map limits, including the floor only for the selected gate profile."""
    values = ego_map_values(config, overrides, two_waypoint_gate)
    from .virtual_boundary import launch_arguments
    return [argument+':='+str(values[key]) for key, (argument, _parameter) in MAP_FIELDS.items()] + launch_arguments(config)


def lidar_launch_arguments(config, project=None):
    resolved = resolve_map_tuning(config, project)
    return [argument+':='+str(resolved['lidar'][key])
            for key, (argument, _parameter) in LIDAR_FIELDS.items()]


def map_tuning_report(config, project=None):
    root = Path(project or project_root()).resolve()
    resolved = resolve_map_tuning(config, root)
    sections = _sections(config)
    metadata = config.get('_flight_configuration', {}).get('sources', {})
    provenance = config.get('_tuning_sources', {})
    overrides = {item.split('=', 1)[0] for item in config.get('_overrides', ())
                 if isinstance(item, str) and '=' in item}
    report = {}
    for group, original, effective in (
            ('planning.map', sections[0], resolved['planning']['map']),
            ('lidar', sections[1], resolved['lidar'])):
        for key, value in effective.items():
            name = group+'.'+key
            fallback = root/(PLANNER_XML if group == 'planning.map' else
                              'config/fastlio.yaml' if key == 'blind_m' else LIO_LAUNCH)
            if original.get(key) is None:
                source = ('兼容默认（虚拟地板关闭）' if key == 'min_height_m' else str(fallback))
            elif any(name == key or name.startswith(key+'.') for key in overrides):
                recorded = metadata.get(name, '--set')
                source = recorded if recorded.startswith('--') else '--set'
            else:
                source = provenance.get(name, metadata.get(name, config.get('_config_path', 'common')))
            report[name] = dict(value=value, source=source,
                takes_effect='下次任务启动' if group == 'planning.map' else '重启 1 号传感器终端')
    return report
