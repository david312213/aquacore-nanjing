"""One tuning file; common defaults then the entry's task preset then CLI."""
from copy import deepcopy
from pathlib import Path
import yaml
from .configuration import expand, load_config, project_root

TASKS = ('takeoff', 'waypoint-route', 'delivery-route')


def base_configuration(path=None, project=None):
    return load_config(path or Path(project or project_root())/'config/mission.yaml', project=project)


def leaves(value, prefix=''):
    for key, item in value.items():
        if str(key).startswith('_'):
            continue
        name = prefix+'.'+str(key) if prefix else str(key)
        if isinstance(item, dict):
            yield from leaves(item, name)
        else:
            yield name, item


def merge(config, overlay, source, sources):
    if not isinstance(overlay, dict):
        raise ValueError(source+' 必须是 YAML 对象')
    for key, value in overlay.items():
        if isinstance(value, dict) and value:
            current = config.setdefault(key, {})
            if not isinstance(current, dict):
                raise ValueError('配置分组类型不一致: '+str(key))
            merge(current, value, source, {})
        else:
            config[key] = deepcopy(value)
    sources.update({key: source for key, _ in leaves(overlay)})


def apply_overrides(config, overrides, sources, override_sources=None):
    for item in overrides:
        key, separator, raw = item.partition('=')
        if not separator:
            raise ValueError('参数覆盖格式为 分组.参数=值')
        parts, section = key.split('.'), config
        for part in parts[:-1]:
            if not isinstance(section.get(part), dict):
                raise ValueError('配置分组不存在: '+key)
            section = section[part]
        if parts[-1] not in section:
            raise ValueError('配置参数不存在: '+key)
        section[parts[-1]] = yaml.safe_load(raw)
        sources[key] = (override_sources or {}).get(key, '--set')


def apply_flight_configuration(config, path=None, task=None, overrides=(), project=None, override_sources=None):
    path = Path(path or Path(project or project_root())/'flight/flight.yaml').expanduser().resolve()
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict):
        raise ValueError('飞行调参文件必须是 YAML 对象')
    document, config, sources = expand(document, project or project_root()), deepcopy(config), {}
    merge(config, document.get('common', {}), 'common', sources)
    if task is not None:
        if task not in TASKS:
            raise ValueError('未知南京飞行任务: '+str(task))
        if task not in document.get('mission_profiles', {}):
            raise ValueError('飞行调参文件缺少 mission_profiles.'+task)
        merge(config, document['mission_profiles'][task], 'mission_profiles.'+task, sources)
    apply_overrides(config, overrides, sources, override_sources)
    config['_overrides'] = list(overrides)
    config['_flight_configuration'] = dict(path=str(path), task=task, sources=sources)
    # Snapshot map and lidar values once; no watching or runtime hot reload.
    from .map_tuning import freeze_map_tuning
    if task in ('waypoint-route', 'delivery-route'):
        from .virtual_boundary import freeze_boundary
        fixed = config.get('navigation', {}).get('backend') == 'fixed_min_snap'
        if fixed:
            # A recorded boundary is only a preview overlay in this mode. A
            # missing EGO map file must not become a new fixed-route flight gate.
            try:
                freeze_boundary(config, project or project_root())
            except (OSError, ValueError, yaml.YAMLError) as exc:
                config['_virtual_boundary'] = dict(enabled=False, diagnostic=str(exc))
            boundary = config['_virtual_boundary']
            boundary['configured_enabled'] = bool(config.get('planning', {}).get('virtual_boundary', {}).get('enabled'))
            boundary.update(enabled=False, effective=False, visualization_only=True)
        else:
            freeze_boundary(config, project or project_root())
    return freeze_map_tuning(config, project)


def load_observer_configuration(path=None, flight_config=None, project=None):
    return apply_flight_configuration(base_configuration(path, project), flight_config, project=project)


def report(config):
    metadata = config.get('_flight_configuration', {})
    sources = {key: config.get('_config_path') for key, _ in leaves(config)}
    sources.update(config.get('_tuning_sources', {})); sources.update(metadata.get('sources', {}))
    def timing(key):
        if key.startswith('rc_payload.'):
            return '重启 1 号传感器终端'
        if key.startswith(('localization.', 'lidar.')):
            return '重启对应传感器/定位终端'
        return '下次任务；运行中不热更新'
    return dict(machine_config=config.get('_config_path'), flight_config=metadata.get('path'),
        task=metadata.get('task'), effective={key:dict(value=value, source=sources.get(key)) for key,value in leaves(config)},
        takes_effect={key:timing(key) for key in sources})
