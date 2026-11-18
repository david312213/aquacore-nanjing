"""PX4 task feedback in the existing frozen FAST-LIO planning coordinates.

The localization bridge already aligns the ENU axes. Only the origin offset is
captured on the ground; it must never follow subsequent estimator drift.
"""
import math

from .native_motion import vector
from .native_reference import LIO_FRAME, DIRECT

PX4_FRAME = 'px4_local_enu'


def feedback_source(config):
    flight = config.get('flight', {})
    source = flight.get('native_task_feedback', 'lio')
    if source not in ('lio', 'px4'):
        raise ValueError('flight.native_task_feedback 必须是 lio 或 px4')
    if source == 'px4':
        if flight.get('controller') != 'px4_native':
            raise ValueError('PX4任务反馈试验需要 flight.controller=px4_native')
        if flight.get('native_reference_mode') != DIRECT:
            raise ValueError('PX4任务反馈试验需要 native_reference_mode=lio_direct，禁止重复补偿坐标')
    return source


def validate_task_mode(config, mode):
    return feedback_source(config)


def task_frame(config):
    feedback_source(config)
    return LIO_FRAME


def source_label(config):
    return 'PX4_EKF' if feedback_source(config) == 'px4' else 'FAST_LIO'


class FrozenPx4TaskFrame:
    """One ground registration, shared by feedback, planner and all setpoints."""
    def __init__(self, lio_position, px4_position, pose_stamp):
        # These values are also saved as ROS parameters. tuple/list alone
        # preserves NumPy scalars, which XML-RPC cannot marshal.
        self.ground_origin = tuple(float(v) for v in vector(lio_position))
        self.translation_xyz = tuple(float(v) for v in vector(px4_position)-vector(lio_position))
        self.pose_stamp = float(pose_stamp)
        if not math.isfinite(self.pose_stamp) or self.pose_stamp <= 0.:
            raise ValueError('invalid PX4 task-frame timestamp')

    def feedback(self, raw):
        packet = dict(raw)
        packet.update(position=tuple(vector(raw['position'])-vector(self.translation_xyz)),
            frame=LIO_FRAME, source_frame=PX4_FRAME, position_source='PX4_EKF',
            translation_xyz=self.translation_xyz, frame_stamp=self.pose_stamp)
        return packet

    def to_px4(self, command, feedback):
        if command.get('frame') != LIO_FRAME:
            raise ValueError('PX4任务指令必须使用统一雷达世界系，禁止混入原生PX4坐标')
        converted = dict(command)
        converted['position'] = tuple(vector(command['position'])+vector(self.translation_xyz))
        converted['velocity'] = tuple(vector(command['velocity']))
        converted['yaw'] = float(command['yaw'])
        if not math.isfinite(converted['yaw']):
            raise ValueError('invalid PX4 task yaw')
        if 'acceleration' in command:
            converted['acceleration'] = tuple(vector(command['acceleration']))
            converted['yaw_rate'] = float(command.get('yaw_rate', 0.))
            if not math.isfinite(converted['yaw_rate']):
                raise ValueError('invalid PX4 task yaw rate')
        converted.update(frame=PX4_FRAME, source_frame=LIO_FRAME,
            position_source='PX4_EKF', reference_mode='px4_task_frozen',
            lio_reference_xyz=tuple(vector(command['position'])),
            translation_xyz=self.translation_xyz, velocity_offset_xyz=(0., 0., 0.),
            frame_stamp=self.pose_stamp, px4_pose_stamp=float(feedback['pose_stamp']),
            px4_velocity_stamp=float(feedback['velocity_stamp']))
        return converted
