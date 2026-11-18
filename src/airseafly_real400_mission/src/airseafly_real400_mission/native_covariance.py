"""Optional FAST-LIO pose covariance transport, without ROS.

The repropagated predictor supplies pose covariance in the order
``[position_world, left/world small-angle error]`` at the odometry stamp.
The frozen alignment and mount calibration are treated as fixed. Their
uncertainty is not estimated here. Source pose covariance may be scaled and
optionally floored; neither operation establishes its measured accuracy.

Odometry does not contain pose--twist cross covariance. In particular, rotating
world velocity into body axes needs attitude--velocity cross terms. Therefore
this experiment transports source *pose* covariance only and deliberately keeps
the existing fixed twist variances. It does not claim calibrated velocity noise.
"""
import numbers
import numpy as np

from .native_motion import rotation, vector


FIXED = 'fixed'
SOURCE_POSE = 'source_pose'


def validate_mode(value):
    if value not in (FIXED, SOURCE_POSE):
        raise ValueError('covariance_mode must be fixed or source_pose')
    return value


def skew(value):
    x, y, z = vector(value)
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def _positive_number(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Real):
        raise ValueError(name+' must be a finite positive number')
    result = float(value)
    if not np.isfinite(result) or result <= 0.:
        raise ValueError(name+' must be a finite positive number')
    return result


def validate_settings(variances, *, pose_covariance_scale=1.,
                      pose_covariance_floor_enabled=True, angular_velocity_variance=None):
    """Validate configuration separately from fallible incoming source data."""
    try:
        values = tuple(variances)
    except TypeError as exc:
        raise ValueError('odometry variances must contain three numbers') from exc
    if len(values) != 3:
        raise ValueError('odometry variances must contain three numbers')
    result = {name: _positive_number(value, name) for name, value in zip(
        ('position_variance', 'orientation_variance', 'velocity_variance'), values)}
    # None preserves historical callers that supplied only the three values.
    angular = result['orientation_variance'] if angular_velocity_variance is None else angular_velocity_variance
    result['angular_velocity_variance'] = _positive_number(angular, 'angular_velocity_variance')
    result['pose_covariance_scale'] = _positive_number(pose_covariance_scale, 'pose_covariance_scale')
    if type(pose_covariance_floor_enabled) is not bool:
        raise ValueError('pose_covariance_floor_enabled must be a boolean')
    result['pose_covariance_floor_enabled'] = pose_covariance_floor_enabled
    return result


def fixed_covariances(variances, lever, angular_velocity_variance=None):
    """Configured fixed variances, also used when optional pose floors are on.

    Pose angular variance is in rad²; the separate twist angular variance is in
    (rad/s)². Only legacy three-value callers reuse the former numeric value.
    """
    settings = validate_settings(variances, angular_velocity_variance=angular_velocity_variance)
    pv, rv, vv = (settings[key] for key in ('position_variance', 'orientation_variance', 'velocity_variance'))
    t = vector(lever)
    pose = np.diag([pv + float(t @ t)*rv]*3 + [rv]*3)
    twist = np.diag([vv]*3 + [settings['angular_velocity_variance']]*3)
    return pose, twist


def checked_source_pose(values):
    covariance = np.asarray(values, dtype=float)
    if covariance.size != 36:
        raise ValueError('source pose covariance must have 36 entries')
    covariance = covariance.reshape(6, 6)
    if not np.isfinite(covariance).all():
        raise ValueError('source pose covariance contains nonfinite entries')
    if np.any(np.diag(covariance) <= 0.):
        raise ValueError('source pose covariance has unknown or nonpositive variances')
    scale = float(np.max(np.abs(covariance)))
    if np.max(np.abs(covariance-covariance.T)) > 1e-8*scale:
        raise ValueError('source pose covariance is asymmetric')
    covariance = .5*(covariance+covariance.T)
    lowest = float(np.linalg.eigvalsh(covariance)[0])
    if lowest < -1e-10*scale:
        raise ValueError('source pose covariance is not positive semidefinite')
    # Only remove floating-point roundoff, never repair a materially invalid
    # covariance or interpret ROS's all-zero/negative unknown sentinels as valid.
    if lowest < 0.:
        covariance = covariance - lowest*np.eye(6)
    return covariance


def pose_jacobian(frame, q_wi):
    """Map source left/world pose errors to local body-origin pose errors.

    p_local = R_nw (p_world - R_wb t_bi) + t_nw.
    For R_wb' = Exp(dtheta_world) R_wb, the position derivative is
    +R_nw [R_wb t_bi]x; output angular errors are R_nw dtheta_world.
    """
    if frame.r_nw is None:
        raise ValueError('frame is not initialized')
    alignment = frame.r_nw
    lever_world = rotation(q_wi) @ frame.r_bi.T @ frame.t_bi
    jacobian = np.zeros((6, 6))
    jacobian[:3, :3] = alignment
    jacobian[:3, 3:] = alignment @ skew(lever_world)
    jacobian[3:, 3:] = alignment
    return jacobian


def covariances(mode, source_pose, frame, q_wi, variances, *, pose_covariance_scale=1.,
                pose_covariance_floor_enabled=True, angular_velocity_variance=None):
    """Return pose, twist and provenance; invalid source falls back explicitly."""
    validate_mode(mode)
    settings = validate_settings(variances, pose_covariance_scale=pose_covariance_scale,
        pose_covariance_floor_enabled=pose_covariance_floor_enabled,
        angular_velocity_variance=angular_velocity_variance)
    # Keep validation outside the source-data fallback: a configuration typo
    # must fail startup rather than silently selecting a different noise model.
    pose, twist = fixed_covariances(variances, frame.t_bi, settings['angular_velocity_variance'])
    report = dict(pose='fixed', twist='fixed', source_pose_valid=None, fallback_reason='',
                  pose_scale_applied=None, pose_floor_applied=False)
    if mode == FIXED:
        return pose, twist, report
    try:
        raw = checked_source_pose(source_pose)
        jacobian = pose_jacobian(frame, q_wi)
        transformed = settings['pose_covariance_scale'] * (jacobian @ raw @ jacobian.T)
        transformed = .5*(transformed+transformed.T)
        if not np.isfinite(transformed).all():
            raise ValueError('transformed pose covariance contains nonfinite entries')
        if settings['pose_covariance_floor_enabled']:
            # Add only the diagonal shortfall. The complete matrix is scaled
            # above, preserving PSD and scaling its cross terms consistently.
            transformed += np.diag(np.maximum(0., np.diag(pose)-np.diag(transformed)))
        report.update(pose='source_pose_with_floors' if settings['pose_covariance_floor_enabled'] else 'source_pose',
                      source_pose_valid=True, pose_scale_applied=settings['pose_covariance_scale'],
                      pose_floor_applied=settings['pose_covariance_floor_enabled'])
        return transformed, twist, report
    except (ValueError, TypeError, np.linalg.LinAlgError) as exc:
        report.update(pose='fixed_fallback', source_pose_valid=False, fallback_reason=str(exc))
        return pose, twist, report
