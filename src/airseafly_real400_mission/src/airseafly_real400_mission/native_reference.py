"""FAST-LIO task coordinates and the final PX4 setpoint conversion.

The bridge has already corrected the IMU lever arm and aligned all three axes
with local ENU using a frozen rotation. No EKF position enters the task frame.
The trial can send that frame directly; matched estimator differences are then
diagnostics only. The compensated mode remains available for the old entry.
"""
from bisect import bisect_left
import math
from .native_motion import vector


LIO_FRAME = 'fast_lio_local_enu'
COMPENSATED = 'estimate_compensated'
DIRECT = 'lio_direct'


def reference_mode(config):
    mode = config.get('flight', {}).get('native_reference_mode', COMPENSATED)
    if mode not in (COMPENSATED, DIRECT):
        raise ValueError('flight.native_reference_mode 必须是 estimate_compensated 或 lio_direct')
    if mode == DIRECT and config.get('flight', {}).get('controller') != 'px4_native':
        raise ValueError('lio_direct 需要 flight.controller=px4_native')
    return mode


class LioReference:
    def __init__(self):
        self.latest = None
        self.history = []

    def feed(self, stamp, position, velocity):
        stamp = float(stamp)
        if not math.isfinite(stamp):
            raise ValueError('nonfinite FAST-LIO timestamp')
        packet = dict(stamp=stamp, pose_stamp=stamp,
                      position=tuple(vector(position)), velocity=tuple(vector(velocity)),
                      frame=LIO_FRAME, position_source='FAST_LIO')
        self.latest = packet
        index = bisect_left([p['stamp'] for p in self.history], stamp)
        if index < len(self.history) and self.history[index]['stamp'] == stamp:
            self.history[index] = packet
        else:
            self.history.insert(index, packet)
        self.history = self.history[-128:]
        return packet

    def sample_at(self, stamp, field='position'):
        if not self.history:
            raise ValueError('missing FAST-LIO position for coordinate conversion')
        index = bisect_left([p['stamp'] for p in self.history], stamp)
        if index < len(self.history) and self.history[index]['stamp'] == stamp:
            return vector(self.history[index][field]), stamp, 'exact'
        if index == 0 or index == len(self.history):
            sample = self.history[0 if index == 0 else -1]
            return vector(sample[field]), sample['stamp'], 'nearest'
        after, before = self.history[index], self.history[index-1]
        fraction = (stamp-before['stamp'])/(after['stamp']-before['stamp'])
        position = vector(before[field]) + fraction*(vector(after[field])-vector(before[field]))
        return position, stamp, 'interpolated'

    def position_at(self, stamp):
        return self.sample_at(stamp)

    def to_px4(self, command, feedback, map_velocity=False, mode=COMPENSATED):
        if mode not in (COMPENSATED, DIRECT):
            raise ValueError('unknown native reference mode: '+str(mode))
        if not feedback or not feedback.get('pose_received', True):
            raise ValueError('missing PX4 pose for coordinate conversion')
        stamp = float(feedback['pose_stamp'])
        if not math.isfinite(stamp):
            raise ValueError('nonfinite PX4 pose timestamp')
        # Match the radar position to the estimator sample, rather than
        # interpreting ordinary transport delay as an origin displacement.
        radar, matched_stamp, alignment = self.position_at(stamp)
        position_delta = vector(feedback['position']) - radar
        translation = position_delta if mode == COMPENSATED else vector((0., 0., 0.))
        velocity = vector(command['velocity'])
        velocity_offset = vector((0., 0., 0.))
        velocity_delta = None
        velocity_matched_stamp = velocity_alignment = None
        velocity_stamp = float(feedback.get('velocity_stamp', stamp))
        if map_velocity:
            if 'velocity' not in feedback or not math.isfinite(velocity_stamp):
                raise ValueError('missing PX4 velocity for coordinate conversion')
            radar_velocity, velocity_matched_stamp, velocity_alignment = self.sample_at(velocity_stamp, 'velocity')
            velocity_delta = vector(feedback['velocity']) - radar_velocity
            if mode == COMPENSATED:
                velocity_offset = velocity_delta
                velocity = velocity + velocity_offset
        converted = dict(position=tuple(vector(command['position'])+translation),
                    velocity=tuple(velocity), yaw=float(command['yaw']),
                    frame='px4_local_enu', source_frame=LIO_FRAME,
                    lio_reference_xyz=tuple(vector(command['position'])),
                    translation_xyz=tuple(translation), px4_pose_stamp=stamp,
                    velocity_offset_xyz=tuple(velocity_offset), px4_velocity_stamp=velocity_stamp,
                    lio_matched_stamp=matched_stamp, sample_alignment=alignment,
                    reference_mode=mode,
                    estimator_position_delta_xyz=tuple(position_delta),
                    estimator_velocity_delta_xyz=None if velocity_delta is None else tuple(velocity_delta),
                    position_match_dt_s=stamp-matched_stamp,
                    position_time_matched=alignment != 'nearest',
                    lio_velocity_matched_stamp=velocity_matched_stamp,
                    velocity_sample_alignment=velocity_alignment,
                    velocity_match_dt_s=None if velocity_matched_stamp is None else velocity_stamp-velocity_matched_stamp,
                    velocity_time_matched=velocity_alignment in ('exact', 'interpolated'))
        if 'acceleration' in command:
            converted['acceleration'] = tuple(vector(command['acceleration']))
            converted['yaw_rate'] = float(command.get('yaw_rate', 0.))
        for key in ('source_stamp', 'trajectory_id', 'session_id', 'reference_source', 'setpoint_mode'):
            if key in command:
                converted[key] = command[key]
        return converted
