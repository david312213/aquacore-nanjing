"""Workspace paths and minimal Nanjing runtime configuration."""
import copy
import os
from pathlib import Path
import yaml


def project_root():
    return Path(__file__).resolve().parents[4]


def default_config_path():
    return project_root()/'config/mission.yaml'


def expand(value, project):
    if isinstance(value, dict):
        return {str(k): expand(v, project) for k, v in value.items()}
    if isinstance(value, list):
        return [expand(v, project) for v in value]
    if isinstance(value, str):
        return os.path.expanduser(value.replace('${PROJECT}', str(project)))
    return value


def load_config(path=None, overrides=(), project=None):
    path = Path(path or default_config_path()).expanduser().resolve()
    with path.open() as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError('配置文件必须为 YAML 对象')
    config = copy.deepcopy(config)
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
    config = expand(config, project or project_root())
    config['_config_path'], config['_overrides'] = str(path), list(overrides)
    return config


def runtime_paths():
    prefix = '/tmp/airseafly_mission_%d' % os.getuid()
    return dict(socket=prefix+'.sock', manager_lock=prefix+'.lock',
                compute_lock='/tmp/airseafly_compute_%d.lock' % os.getuid())
