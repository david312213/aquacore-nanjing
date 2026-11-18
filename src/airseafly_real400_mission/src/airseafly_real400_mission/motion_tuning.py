"""Resolve recorded-route EGO limits without modifying configuration snapshots."""
import math
LIMITS = ('max_velocity_mps', 'max_acceleration_mps2')

def motion_limits(config, two_waypoint_gate=False, rviz=False):
    result = {key: config['planning'][key] for key in LIMITS}
    for key, value in result.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('planning.' + key + ' 必须为正的有限数值')
    return result

def motion_sources(config, two_waypoint_gate=False, rviz=False):
    sources = config.get('_flight_configuration', {}).get('sources', {})
    return {key: dict(key='planning.'+key, source=sources.get('planning.'+key, 'config')) for key in LIMITS}

def motion_profile(two_waypoint_gate=False, rviz=False):
    return 'recorded_route'

def validate_motion_tuning(config):
    motion_limits(config)
