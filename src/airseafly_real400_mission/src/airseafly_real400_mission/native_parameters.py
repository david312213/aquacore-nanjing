"""User-facing mission reference limits; these never write PX4 parameters."""
import math

VERTICAL_SPEED_FIELDS = ('drop_speed_mps', 'recover_speed_mps')

# CLI flag, argparse name, configuration key, units/meaning.
OPTIONS = (
    ('--height', 'height', 'flight.cruise_height_m', '兼容原用法：同时设置起飞/航点和过门高度，m'),
    ('--takeoff-height', 'takeoff_height', 'flight.cruise_height_m', '起飞、航点及投后回升高度，m；独立于过门/投放'),
    ('--gate-height', 'gate_height', 'flight.gate_height_m', '门口、中点、降落点航程的相对高度，m'),
    ('--drop-height', 'drop_height', 'flight.drop_height_m', '相对起飞基准的投放高度，m'),
    ('--height-tolerance', 'height_tolerance', 'flight.height_tolerance_m', '沿用的到高容差，m'),
    ('--takeoff-speed', 'takeoff_speed', 'flight.native_takeoff_speed_mps', '原生起飞参考速度，m/s；省略沿用原机'),
    ('--takeoff-acceleration', 'takeoff_acceleration', 'flight.native_takeoff_acceleration_mps2', '原生起飞参考加速度，m/s²'),
    ('--speed', 'speed', 'planning.max_velocity_mps', 'EGO航点最大速度，m/s'),
    ('--acceleration', 'acceleration', 'planning.max_acceleration_mps2', 'EGO最大加速度，m/s²'),
    ('--align-speed', 'align_speed', 'motion.native_horizontal_speed_mps', '原生对准及投放口补偿水平速度，m/s'),
    ('--vertical-speed', 'vertical_speed', 'flight.vertical_speed_mps', '兼容原用法：同时设置投放下降、投后回升速度，m/s'),
    ('--drop-speed', 'drop_speed', 'flight.drop_speed_mps', '投放下降速度，m/s'),
    ('--recover-speed', 'recover_speed', 'flight.recover_speed_mps', '投放后回升速度，m/s'),
    ('--align-acceleration', 'align_acceleration', 'motion.native_acceleration_mps2',
     '原生对准、投放口补偿及下降参考加速度，m/s²；未单独设回升加速度时回升也用此值'),
    ('--recover-acceleration', 'recover_acceleration', 'motion.recover_acceleration_mps2',
     '投放后回升参考加速度，m/s²；省略或 null 沿用对准加速度'),
)


def add_arguments(parser):
    for flag, dest, _key, meaning in OPTIONS:
        parser.add_argument(flag, dest=dest, type=float, help=meaning)
    parser.add_argument('--delivery-setpoint-mode', choices=('position_yaw', 'position_velocity'),
                        help='本次投放指令：position_yaw只发位置/航向；position_velocity沿用速度前馈')
    parser.add_argument('--dry-run', '--show-config', dest='dry_run', action='store_true',
                        help='只显示生效配置，不连接ROS、不起飞、不操作舵机')


def positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(name+' 必须是大于零的有限数值')
    return float(value)


def recover_acceleration(config):
    """Post-drop climb acceleration; null keeps the shared alignment value."""
    motion = config['motion']
    value = motion.get('recover_acceleration_mps2')
    return motion.get('native_acceleration_mps2', .50) if value is None else value


def delivery_motion(config):
    """Native delivery reference limits shown in previews."""
    motion = config['motion']
    return dict(align_speed_mps=motion.get('native_horizontal_speed_mps', .10),
                drop_speed_mps=vertical_speed(config, 'drop'),
                recover_speed_mps=vertical_speed(config, 'recover'),
                align_acceleration_mps2=motion.get('native_acceleration_mps2', .50),
                recover_acceleration_mps2=recover_acceleration(config),
                recover_acceleration_separate=motion.get('recover_acceleration_mps2') is not None)


def delivery_motion_line(value):
    return ('投放运动：对准水平 %.2f m/s，下降 %.2f m/s，回升 %.2f m/s；对准/下降加速度 %.2f m/s²，'
            '回升加速度 %.2f m/s²%s。' % (value['align_speed_mps'], value['drop_speed_mps'],
            value['recover_speed_mps'], value['align_acceleration_mps2'], value['recover_acceleration_mps2'],
            '' if value['recover_acceleration_separate'] else '（未单独设置，沿用对准加速度）'))


def vertical_speed(config, stage):
    """Old aircraft files keep their shared speed until a stage is specified."""
    flight = config['flight']
    value = flight.get(stage+'_speed_mps')
    return flight['vertical_speed_mps'] if value is None else value


def overrides(args):
    result = []
    for flag, dest, key, _meaning in OPTIONS:
        value = getattr(args, dest, None)
        if value is not None:
            result.append(key+'='+str(positive(value, flag)))
            if flag == '--height':
                result.append('flight.gate_height_m='+str(positive(value, flag)))
            elif flag == '--vertical-speed':
                result.extend('flight.'+field+'='+str(positive(value, flag))
                              for field in VERTICAL_SPEED_FIELDS)
    return result


def gate_height(config):
    """Older configurations keep their shared height until explicitly separated."""
    flight = config['flight']
    value = flight.get('gate_height_m')
    return flight['cruise_height_m'] if value is None else value


def validate(config):
    from .native_reference import reference_mode
    from .native_task_feedback import feedback_source
    from .localization_tuning import localization_settings
    from .motion_tuning import validate_motion_tuning
    from .map_tuning import validate_map_tuning
    from .delivery_tuning import validate_delivery_tuning
    reference_mode(config)
    feedback_source(config)
    localization_settings(config)
    validate_motion_tuning(config)
    validate_map_tuning(config)
    validate_delivery_tuning(config)
    for _flag, _dest, key, _meaning in OPTIONS:
        value = config
        for part in key.split('.'):
            if not isinstance(value, dict) or part not in value:
                value = None
                break
            value = value[part]
        if value is not None:
            positive(value, key)


def summary(config, takeoff_speed=None, two_waypoint_gate=False, rviz=False):
    from .motion_tuning import motion_limits, motion_sources
    from .map_tuning import resolve_map_tuning, ego_map_values
    from .delivery_tuning import delivery_tuning_summary
    from .native_task_feedback import feedback_source
    from .localization_tuning import localization_settings
    flight, geometry = config['flight'], config['motion']
    planned, mapping = motion_limits(config), resolve_map_tuning(config)
    speed = flight.get('native_takeoff_speed_mps') if takeoff_speed is None else takeoff_speed
    return dict(native_reference_mode=flight.get('native_reference_mode', 'estimate_compensated'),
        native_task_feedback=feedback_source(config), localization=localization_settings(config),
        height_m=flight['cruise_height_m'], takeoff_height_m=flight['cruise_height_m'],
        gate_height_m=gate_height(config), drop_height_m=flight['drop_height_m'],
        height_tolerance_m=flight['height_tolerance_m'], takeoff_speed_mps=speed,
        takeoff_speed_source='original_controller_config' if speed is None else 'effective',
        takeoff_acceleration_mps2=flight.get('native_takeoff_acceleration_mps2', .50),
        ego_motion_profile='recorded_route', effective_motion_sources=motion_sources(config),
        motion_takes_effect='下次任务启动；运行中不热更新',
        ego_speed_mps=planned['max_velocity_mps'], ego_acceleration_mps2=planned['max_acceleration_mps2'],
        align_speed_mps=geometry.get('native_horizontal_speed_mps', .10),
        vertical_speed_mps=flight['vertical_speed_mps'], drop_speed_mps=vertical_speed(config, 'drop'),
        recover_speed_mps=vertical_speed(config, 'recover'),
        align_acceleration_mps2=geometry.get('native_acceleration_mps2', .50),
        recover_acceleration_mps2=recover_acceleration(config),
        recover_acceleration_source=('motion.recover_acceleration_mps2'
                                     if geometry.get('recover_acceleration_mps2') is not None
                                     else 'motion.native_acceleration_mps2'),
        map=ego_map_values(config), lidar=mapping['lidar'], delivery=delivery_tuning_summary(config))
