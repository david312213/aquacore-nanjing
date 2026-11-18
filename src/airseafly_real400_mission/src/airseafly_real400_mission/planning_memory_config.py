"""Validated, opt-in lidar memory for both legs of ego_land (no ROS)."""
import math


DEFAULTS = dict(enabled=False, voxel_m=.10, radius_m=4., max_age_s=60.,
                min_hits=2, z_max_m=1.6)


def memory_options(supplied):
    options = dict(DEFAULTS, **supplied)
    prefix = 'planning memory '
    if not isinstance(options['enabled'], bool):
        raise ValueError(prefix+'enabled 必须是 true 或 false')
    for key in ('voxel_m', 'radius_m', 'max_age_s', 'z_max_m'):
        value = options[key]
        if key == 'z_max_m' and value is None:
            continue
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value) or key != 'z_max_m' and value <= 0):
            raise ValueError(prefix+key+' 必须是有效的有限数值')
        options[key] = float(value)
    if (isinstance(options['min_hits'], bool) or not isinstance(options['min_hits'], int)
            or options['min_hits'] < 1):
        raise ValueError(prefix+'min_hits 必须是 ≥1 的整数')
    return options
