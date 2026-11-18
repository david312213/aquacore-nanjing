"""Native coordinate-segment primitives extracted from the prior flight adapter."""
from copy import deepcopy
import math
from .native_motion import StraightLine, vector
from .native_parameters import recover_acceleration, vertical_speed
from .native_reference import LIO_FRAME
from .native_freshness import fresh_stamp
from .delivery_tuning import delivery_tuning_summary


class NativeSegmentMixin:
    uses_native = True
    record_control_trace = True

    def __init__(self, config=None):
        super().__init__(config)
        self.delivery_tuning = delivery_tuning_summary(self.config)
        self.native_feedback = self.native_lio_feedback = self.native_px4_command = None
        self.native_origin = self.native_goal = self.native_motion = None
        self.native_active = self.native_held = False
        self.native_yaw = 0.
        self.native_profile = 'waypoint'
        self.native_pending = self.native_return_to = self.native_epoch = None
        self.native_offset = None
        self.native_landing_requested = False
        self.native_arrivals, self.native_results = [], []
        self.native_handoff_start = None
        self._delivery_waiting_reason = ''
        self._delivery_release_height = None

    def on_native_lio_feedback(self, packet):
        data = deepcopy(packet)
        for field in ('position', 'velocity'):
            data[field] = tuple(vector(data[field]))
        if not math.isfinite(float(data['stamp'])):
            raise ValueError('invalid FAST-LIO feedback')
        data.update(frame=LIO_FRAME, position_source='FAST_LIO')
        self.native_lio_feedback = data

    def _native_phase(self, phase):
        self._state('DROP', phase, 'NONE')

    def on_native_owner_result(self, requested, accepted, owner, message, capture=None):
        if requested != self.native_pending:
            return
        self.native_pending = None
        if not accepted:
            if requested == 'PX4_AUTO_LAND':
                self._native_land_unconfirmed('failed', message)
            else:
                self.hold('原生控制权交接失败：'+message)
            return
        if requested == 'PX4_HOLD':
            self.native_held = True
        elif requested == 'PX4_AUTO_LAND':
            self._native_land_queued()

    def _delivery_tolerances(self, first=False):
        return self.delivery_tuning['second_tolerance_xyz_m']

    def on_native_feedback(self, packet):
        data = deepcopy(packet)
        for field in ('position', 'velocity'):
            data[field] = tuple(vector(data[field]))
        if not all(math.isfinite(float(data[k])) for k in ('stamp', 'pose_stamp', 'yaw')):
            raise ValueError('invalid PX4 feedback')
        self.native_feedback = data


    def native_ready(self, now):
        # Diagnostic health/enable flags remain visible but do not gate tasks.
        return bool(self.native_lio_feedback is not None and self.native_feedback is not None
                    and self.native_feedback.get('pose_received', True))


    def _delivery_control_valid(self, now):
        f = self.native_feedback or {}
        return bool(self.native_ready(now) and f.get('armed') and f.get('offboard')
                    and f.get('owner') == 'PX4_NATIVE' and not f.get('manual_paused')
                    and not f.get('watchdog_fault') and not self.native_held
                    and self.state == 'DROP')


    def _owner(self, owner):
        if self.native_pending is None:
            self.native_pending = owner
            self._effect('native_owner', owner=owner)


    def _move(self, goal, phase):
        self.native_goal = tuple(vector(goal))
        cfg = self.config['motion']
        self.native_motion = StraightLine(self.native_lio_feedback['position'], self.native_goal, self._now,
            horizontal=cfg.get('native_horizontal_speed_mps', .10),
            vertical=self._vertical_speed(phase),
            acceleration=self._acceleration(phase))
        self._native_phase(phase)
        self._effect('native_target', phase=phase, profile=self.native_profile,
                     target_xyz=list(self.native_goal), frame=LIO_FRAME, position_source='FAST_LIO')


    def _acceleration(self, phase):
        # Only the post-drop climb may use its own value; alignment moves,
        # offset and descents keep motion.native_acceleration_mps2.
        if phase == 'NATIVE_RECOVER':
            return recover_acceleration(self.config)
        return self.config['motion'].get('native_acceleration_mps2', .50)


    def _vertical_speed(self, phase):
        if phase == 'NATIVE_RECOVER':
            return vertical_speed(self.config, 'recover')
        if phase in ('NATIVE_DESCEND', 'NATIVE_INTERMEDIATE_DESCEND'):
            return vertical_speed(self.config, 'drop')
        return self.config['flight']['vertical_speed_mps']


    def _settled(self, now, first=False):
        if not self.native_ready(now):
            return False
        f = self.native_lio_feedback
        delta = [abs(a-b) for a,b in zip(f['position'], self.native_goal)]
        cfg = self.config['motion']
        tolerances = self._delivery_tolerances(first=first)
        arrived = (math.sqrt(sum(v*v for v in delta)) <= cfg['first_arrival_radius_m'] if first else
                   all(error <= tolerance + 1e-9 for error, tolerance in zip(delta, tolerances)))
        completed = self.native_motion is None or self.native_motion.sample(now)[2]
        if not arrived or not completed:
            return False
        if not any(item['phase']==self.phase for item in self.native_arrivals):
            arrival=dict(phase=self.phase, elapsed_s=now-self.native_epoch,
                         frame=LIO_FRAME, position_source='FAST_LIO',
                         target_xyz=list(self.native_goal), actual_xyz=list(f['position']),
                         error_xyz=[a-b for a,b in zip(self.native_goal,f['position'])],
                         speed_mps=math.sqrt(sum(v*v for v in f['velocity'])))
            self.native_arrivals.append(arrival)
            self._effect('native_arrival', profile=self.native_profile, **arrival)
        return True


    def _recover(self, next_phase='DETECT'):
        if self.native_lio_feedback is None or self.native_origin is None:
            self.hold('回升时缺少雷达位置或起飞基准')
            return
        self.native_active, self.native_held = True, False
        self.native_pending = None
        self.native_yaw = self.native_feedback['yaw'] if self.native_feedback else self.yaw
        self.native_return_to = next_phase
        self._move((*self.native_lio_feedback['position'][:2],
                    self.native_origin[2]+self.config['flight']['cruise_height_m']), 'NATIVE_RECOVER')


    def _recovery_arrived(self, now):
        completed = self.native_motion is None or self.native_motion.sample(now)[2]
        return bool(completed and self.native_ready(now) and self.native_goal is not None)


    def _request_native_release(self, now):
        # Keep the existing 0.30 s localization standard at the actuator step.
        # A missing/stale sample waits here; it does not pause the mission.
        if (not self._delivery_control_valid(now)
                or not fresh_stamp((self.native_lio_feedback or {}).get('stamp'), now)):
            self._delivery_waiting_reason = '等待有效定位与当前自动控制权，尚未释放'
            return
        if self.native_lio_feedback is not None and self.native_origin is not None:
            self._delivery_release_height = self.native_lio_feedback['position'][2] - self.native_origin[2]
        self._state('DROP','WAIT_ACK','NONE')
        self._payload_result = None
        self.payload_outcome = 'pending'
        self._payload_sent_at = now
        self._payload_request = self._effect_sequence+1
        self._effect('release',slot=self.next_slot,request_id=self._payload_request)


    def snapshot(self):
        output = super().snapshot()
        output['native'] = dict(profile=self.native_profile, active=self.native_active,
            held=self.native_held, feedback=deepcopy(self.native_feedback),
            lio_feedback=deepcopy(self.native_lio_feedback), px4_command=deepcopy(self.native_px4_command),
            position_source='FAST_LIO', origin_xyz=self.native_origin, target_xyz=self.native_goal,
            frame=LIO_FRAME, pending=self.native_pending, offset=deepcopy(self.native_offset),
            reference_finished=self.native_motion is None or self.native_motion.sample(self._now)[2],
            arrivals=deepcopy(self.native_arrivals), results=deepcopy(self.native_results),
            handoff_start_xyz=self.native_handoff_start, delivery_tuning=deepcopy(self.delivery_tuning))
        if self.native_active:
            output['control'] = 'NONE' if self.native_held or self.phase in ('LANDING', 'LANDED') else 'PX4_NATIVE'
            p, v, _ = self.native_motion.sample(self._now) if self.native_motion else (self.native_goal,(0.,0.,0.),True)
            output['native_command'] = dict(position=p, velocity=v, yaw=self.native_yaw,
                                            frame=LIO_FRAME, position_source='FAST_LIO')
            if self.delivery_tuning['native_setpoint_mode'] == 'position_yaw':
                output['native_command'].update(setpoint_mode='position_yaw', velocity=(0.,0.,0.))
            if self.native_lio_feedback and self.native_goal:
                error = [a-b for a,b in zip(self.native_goal,self.native_lio_feedback['position'])]
                output['native']['error_xyz'] = error
                output['telemetry'].update(height_target_z=self.native_goal[2], height_error_m=error[2],
                    arrival_metric='reference_complete' if self.phase=='NATIVE_RECOVER' else 'xyz_axes',
                    arrival_tolerance_xyz_m=self._delivery_tolerances(), arrival=dict(elapsed_s=0.,required_s=0.))
        return output
