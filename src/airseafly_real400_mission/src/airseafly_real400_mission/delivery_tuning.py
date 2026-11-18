"""Existing native coordinate descent/recovery and actuator tolerances."""
import math

def _section(config, name):
    value = config.get(name, {})
    if not isinstance(value, dict):
        raise ValueError(name + ' 必须是 YAML 对象')
    return value


def _number(value, name, zero=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or (value < 0 if zero else value <= 0)):
        raise ValueError(name + (' 必须是大于等于零的有限数值' if zero else ' 必须是大于零的有限数值'))
    return float(value)


def delivery_tuning_summary(config):
    geometry, payload = _section(config, 'motion'), _section(config, 'payload')
    mode = geometry.get('native_setpoint_mode', 'position_velocity')
    if mode not in ('position_velocity', 'position_yaw'):
        raise ValueError('motion.native_setpoint_mode 必须为 position_velocity 或 position_yaw')
    first = _number(geometry.get('first_arrival_radius_m', .10), 'motion.first_arrival_radius_m')
    x = _number(geometry.get('second_xy_tolerance_m', .05), 'motion.second_xy_tolerance_m')
    y = geometry.get('second_y_tolerance_m')
    y = x if y is None else _number(y, 'motion.second_y_tolerance_m')
    z = _number(geometry.get('second_z_tolerance_m', min(.05, config['flight']['height_tolerance_m'])),
                'motion.second_z_tolerance_m')
    dwell = _number(payload.get('release_dwell_s', 0.), 'payload.release_dwell_s', zero=True)
    speed = payload.get('release_max_speed_mps')
    if speed is not None:
        speed = _number(speed, 'payload.release_max_speed_mps')
    return dict(native_setpoint_mode=mode, velocity_feedforward=mode == 'position_velocity',
                first_arrival_radius_m=first, second_tolerance_xyz_m=[x, y, z],
                release_dwell_s=dwell, release_max_speed_mps=speed,
                release_stability_enabled=dwell > 0. or speed is not None)

def validate_delivery_tuning(config):
    delivery_tuning_summary(config)
