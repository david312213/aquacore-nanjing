"""Opt-in localization trials, resolved before starting ROS or a mission."""
import math


DEFAULTS = dict(prediction_mode='off', covariance_mode='fixed',
                max_imu_gap_s=.05, max_prediction_horizon_s=1.,
                pose_covariance_scale=1., pose_covariance_floor_enabled=True,
                position_variance=.0025, orientation_variance=.0025,
                velocity_variance=.01, angular_velocity_variance=.0025)
COVARIANCE_ARGUMENTS = ('covariance_mode', 'pose_covariance_scale',
                        'pose_covariance_floor_enabled', 'position_variance',
                        'orientation_variance', 'velocity_variance',
                        'angular_velocity_variance')
ARGUMENTS = dict(prediction_mode='prediction_mode', covariance_mode='covariance_mode',
                 max_imu_gap_s='prediction_max_imu_gap',
                 max_prediction_horizon_s='prediction_max_prediction_horizon')
ARGUMENTS.update({key: key for key in COVARIANCE_ARGUMENTS})


def localization_settings(config):
    section = config.get('localization', {})
    if not isinstance(section, dict):
        raise ValueError('localization 必须是 YAML 对象')
    values = {key: section.get(key, default) for key, default in DEFAULTS.items()}
    if values['prediction_mode'] not in ('off', 'shadow', 'replace'):
        raise ValueError('localization.prediction_mode 必须是 off、shadow 或 replace')
    if values['covariance_mode'] not in ('fixed', 'source_pose'):
        raise ValueError('localization.covariance_mode 必须是 fixed 或 source_pose')
    if not isinstance(values['pose_covariance_floor_enabled'], bool):
        raise ValueError('localization.pose_covariance_floor_enabled 必须是 true 或 false 布尔值')
    for key in ('max_imu_gap_s', 'max_prediction_horizon_s', 'pose_covariance_scale',
                'position_variance', 'orientation_variance', 'velocity_variance',
                'angular_velocity_variance'):
        value = values[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('localization.'+key+' 必须是大于零的有限数值')
        values[key] = float(value)
    if values['covariance_mode'] == 'source_pose' and values['prediction_mode'] != 'replace':
        raise ValueError('source_pose 需要 prediction_mode=replace：旧预测器没有同时间的有效协方差')
    return values


def localization_launch_arguments(values, mode):
    keys = COVARIANCE_ARGUMENTS if mode == 'localization' else (
        'prediction_mode', 'max_imu_gap_s', 'max_prediction_horizon_s')
    return [ARGUMENTS[key]+':='+(str(values[key]).lower() if isinstance(values[key], bool)
                               else str(values[key])) for key in keys]
