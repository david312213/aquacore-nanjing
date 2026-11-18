"""Resolve the two recorded waypoint profiles once per task."""
import math


def resolve_profiles(config):
    flight = config['flight']
    defaults = config.get('waypoint_profiles', {})
    overrides = {v.split('=', 1)[0] for v in config.get('_overrides', ())}
    result = {}
    for name in ('cruise', 'gate'):
        values = defaults.get(name, {})
        height_key = 'gate_height_m' if name == 'gate' else 'cruise_height_m'
        fallback = flight.get(height_key) or flight['cruise_height_m']
        height = values.get('height_m')
        if height is None or 'flight.'+height_key in overrides:
            height = fallback
        inflation = values.get('inflation_m')
        if inflation is None or 'planning.map.inflation_m' in overrides:
            inflation = config['planning']['map']['inflation_m']
        for key, value in (('height_m', height), ('inflation_m', inflation)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or (value <= 0 if key == 'height_m' else value < 0):
                raise ValueError('waypoint_profiles.'+name+'.'+key+' 数值无效')
        result[name] = dict(height_m=float(height), inflation_m=float(inflation))
    return result


def point_profiles(document):
    return document.get('point_profiles', ['cruise'] * len(document['points']))


def entry_points(document):
    """Version 3 entry points flown once before the loop; none for older files."""
    return [list(xy) for xy in document.get('entry_points', [])]


def entry_profiles(document):
    return list(document.get('entry_profiles', ['cruise'] * len(entry_points(document))))


def gate_velocity(config):
    """planning.gate_velocity_mps for the fixed route; None keeps one route speed."""
    value = config.get('planning', {}).get('gate_velocity_mps')
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError('planning.gate_velocity_mps 须为 null 或正数')
    return float(value)


def point_velocities(config, names):
    """Per-point speed limits: gate points get gate_velocity_mps, others None."""
    value = gate_velocity(config)
    return [value if value is not None and name == 'gate' else None for name in names]
