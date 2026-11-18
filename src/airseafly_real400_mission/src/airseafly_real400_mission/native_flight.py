"""Pure mission lifecycle for EGO references executed entirely by PX4.

Tasks normally use corrected FAST-LIO body-origin ENU positions and PX4 yaw.
The optional takeoff/hover trial uses PX4 ENU feedback and its own ground origin.
This mixin chooses references; NativeRuntime owns ROS streaming and mode RPCs.
"""
from copy import deepcopy

from .flight_core import FlightCore
from .native_motion import StraightLine
from .native_parameters import summary as parameter_summary, positive, vertical_speed
from .native_task_feedback import feedback_source, validate_task_mode, task_frame, source_label


class NativeFlightMixin:
    @property
    def task_position_feedback(self):
        return (getattr(self, 'native_task_feedback', None) if feedback_source(self.config) == 'px4'
                else self.native_lio_feedback)

    @property
    def full_native(self):
        return self.config['flight'].get('controller', 'px4ctrl') == 'px4_native'

    def __init__(self, config=None):
        super().__init__(config)
        self.native_flight_phase = 'IDLE'
        self.native_takeoff_speed = self.config['flight'].get('native_takeoff_speed_mps') or .30
        self._native_takeoff_motion = None
        self._native_takeoff_start = None
        self._native_takeoff_reference = None
        self._native_takeoff_complete = False
        self._native_hold_anchor = None
        self._native_hold_yaw = None
        self._manual_height_motion = None
        self._manual_height_reference = None
        self.manual_height = None
        self.native_task_feedback = None
        self.native_landing_state = 'idle'
        self.native_landing_reason = ''
        self.native_landing_attempts = 0
        self._native_land_requested_at = None

    def start(self, *args, **kwargs):
        validate_task_mode(self.config, args[0] if args else kwargs.get('mode'))
        output = super().start(*args, **kwargs)
        if not self.full_native:
            return output
        self._native_takeoff_start = tuple(self.position)
        self._native_takeoff_reference = (tuple(self.position), (0., 0., 0.))
        self.native_flight_phase = 'PREPARE' if self.state == 'TAKEOFF' else 'TRACK'
        return self.snapshot()

    def begin_native_flight(self):
        """Capture ground in the selected task frame before generating a climb."""
        if not self.full_native:
            return
        if self.state == 'TAKEOFF':
            ground = tuple(self.native_origin if self.native_origin is not None else self.origin_xyz)
            self.origin_xyz = ground
            self.cruise_z = ground[2] + self.config['flight']['cruise_height_m']
            self.drop_z = ground[2] + self.config['flight']['drop_height_m']
            self._native_takeoff_start = ground
            self._native_takeoff_reference = (ground, (0., 0., 0.))
            self.command_position = (ground[0], ground[1], self.cruise_z)
            self.native_flight_phase = 'PREPARE'
            # start() has queued one ordinary takeoff effect. Keep its endpoint
            # in the same task frame as the references that follow it.
            for effect in self._effects:
                if effect['type'] == 'takeoff':
                    effect['position_xyz'] = list(self.command_position)
                    effect['target_world_z'] = self.cruise_z

    def _native_takeoff_feedback_ready(self):
        feedback = self.native_feedback or {}
        return (feedback.get('owner') == 'PX4_NATIVE' and feedback.get('armed') is True
                and feedback.get('offboard') is True)

    def _takeoff_arrived(self, now, control_state):
        # Finish sending the climb; actual height error and dwell do not gate the next route leg.
        return self._takeoff_control_ready(control_state)

    def _takeoff_control_ready(self, control_state):
        return self._native_takeoff_feedback_ready() and self._native_takeoff_complete

    def begin_manual_height(self, height_m, request_id, now):
        """Continue this takeoff's hover at a new height, without a new task/owner."""
        height_m = positive(height_m, '目标高度')
        if (not self.full_native or self.mode != 'takeoff' or self.state != 'COMPLETE'
                or self.phase not in ('TAKEOFF_COMPLETE', 'MANUAL_HEIGHT_COMPLETE')):
            raise ValueError('本次起飞悬停已中断或尚未完成，未提交调高')
        task, feedback = self.task_position_feedback, self.native_feedback
        if task is None or feedback is None:
            raise ValueError('缺少当前位姿，无法生成调高参考')
        start = tuple(task['position'])
        goal = (start[0], start[1], self.origin_xyz[2] + height_m)
        direction = 'up' if goal[2] > start[2] else 'down' if goal[2] < start[2] else 'hold'
        stage = 'recover' if direction == 'up' else 'drop'
        speed = vertical_speed(self.config, stage)
        acceleration = self.config['motion']['native_acceleration_mps2']
        motion = StraightLine(start, goal, now, vertical=speed, acceleration=acceleration)
        self._now = float(now)
        self.position, self.yaw = start, float(feedback['yaw'])
        self.command_position, self.command_yaw = goal, self.yaw
        self.velocity = (0., 0., 0.)
        self.done, self.ended_at, self.reason = False, None, ''
        self._manual_height_motion = motion
        self._manual_height_reference = (start, (0., 0., 0.))
        self.manual_height = dict(request_id=request_id, target_height_m=height_m,
            start_height_m=start[2]-self.origin_xyz[2], target_xyz=list(goal), yaw=self.yaw,
            direction=direction, speed_mps=speed, acceleration_mps2=acceleration,
            started_at_s=float(now), duration_s=motion.duration, state='moving')
        self.native_flight_phase = 'MANUAL_HEIGHT'
        self._state('MANUAL_HEIGHT', 'MANUAL_HEIGHT_MOVE', 'NONE')
        return self.snapshot()

    def tick(self, now, position, yaw, control_state=None):
        landing_handoff = self.phase in ('WAIT_LAND', 'WAIT_LAND_MODE', 'LANDING', 'LANDED')
        if self.full_native and feedback_source(self.config) == 'px4' and not landing_handoff:
            task = self.task_position_feedback
            if task is None:
                self.hold('缺少PX4任务位置或速度，停止任务推进')
                return super().tick(now, self.position, self.yaw, control_state)
            position = task['position']
            yaw = self.native_feedback['yaw']
        if self.full_native and self.state == 'TAKEOFF':
            if self._native_takeoff_feedback_ready() and self._native_takeoff_motion is None:
                start = self._native_takeoff_start or tuple(position)
                self._native_takeoff_motion = StraightLine(start, self.command_position, now,
                    vertical=self.native_takeoff_speed,
                    acceleration=self.config['flight'].get('native_takeoff_acceleration_mps2', .50))
                self.native_flight_phase = 'ASCEND'
            if self._native_takeoff_motion is not None:
                p, v, completed = self._native_takeoff_motion.sample(now)
                self._native_takeoff_reference = (p, v)
                self._native_takeoff_complete = completed
        if self.full_native and self.state == 'MANUAL_HEIGHT' and self._manual_height_motion is not None:
            p, v, completed = self._manual_height_motion.sample(now)
            self._manual_height_reference = (p, v)
            if completed:
                self._now = float(now)
                self._manual_height_motion = None
                self.manual_height['state'] = 'complete'
                self.native_flight_phase = 'TRACK'
                self._state('COMPLETE', 'MANUAL_HEIGHT_COMPLETE', 'NONE')
                self.done = True
                self.reason = '高度参考已执行完，持续保持目标高度；实际高度仅显示'
        output = super().tick(now, position, yaw, control_state)
        if self.full_native and self.state != 'TAKEOFF' and self.native_flight_phase in ('PREPARE', 'ASCEND'):
            self.native_flight_phase = 'TRACK'
            output['native']['flight_phase'] = self.native_flight_phase
        return output


    def _native_land_queued(self):
        """An owner-service acknowledgement queues landing; it is not mode feedback."""
        self.native_landing_requested = True
        self._land_sent = False
        self._native_land_requested_at = self._now
        self.native_landing_state = 'requesting'
        self.native_landing_reason = '降落请求已排队，等待实际 AUTO.LAND 模式'
        self.native_landing_attempts = 0
        feedback = self.task_position_feedback or {}
        self.native_goal = self.command_position = tuple(feedback.get('position', self.position))
        self.native_yaw = self.command_yaw = float((self.native_feedback or {}).get('yaw', self.yaw))
        self.native_motion = None
        self.velocity = (0., 0., 0.)
        self._state(self.state, 'WAIT_LAND_MODE', 'NONE')

    def _native_land_unconfirmed(self, state, reason):
        self.native_landing_state = state
        self.native_landing_reason = str(reason or '未收到实际 AUTO.LAND 模式')
        self.hold('降落未确认：'+self.native_landing_reason)
        self._effect('native_landing_unconfirmed', landing_state=state,
                     reason=self.native_landing_reason, attempts=self.native_landing_attempts)

    def _tick_native_landing(self):
        """Shared LAND/DROP handoff, confirmed only by current vehicle state."""
        if self.phase not in ('WAIT_LAND', 'WAIT_LAND_MODE', 'LANDING'):
            return False
        if self.phase == 'WAIT_LAND':
            if not self.native_landing_requested:
                self._owner('PX4_AUTO_LAND')
            return True
        feedback = self.native_feedback or {}
        stamp = feedback.get('stamp')
        confirmation_expired = (self.phase == 'WAIT_LAND_MODE' and self._native_land_requested_at is not None
                                and self._now-self._native_land_requested_at >= 10.)
        self.native_landing_attempts = feedback.get('landing_attempts', self.native_landing_attempts)
        completed = feedback.get('landed') is True and feedback.get('armed') is False
        gateway_state = feedback.get('landing_state')
        if gateway_state in ('failed', 'cancelled') and not completed:
            self._native_land_unconfirmed(gateway_state, feedback.get('landing_reason'))
            return True
        # PX4 may disarm and leave AUTO.LAND before the mission receives its
        # intermediate mode sample. Actual on-ground + disarmed feedback is
        # also a completed handoff, regardless of the post-landing mode name.
        if self.phase == 'WAIT_LAND_MODE' and (feedback.get('mode') == 'AUTO.LAND' or completed):
            self.native_landing_state = 'confirmed'
            self.native_landing_reason = ('已落地上锁' if completed else
                                          '实际 AUTO.LAND 模式已确认，等待落地上锁')
            self._land_sent = True
            self._state(self.state, 'LANDING', 'NONE')
            self._effect('native_landing_confirmed', mode=feedback.get('mode'),
                         stamp_s=stamp, landed_and_disarmed=completed)
        if self.phase == 'LANDING':
            self.on_landed()
        elif confirmation_expired:
            # The gateway's bounded 8-second attempt runs first. Allow its
            # terminal status to arrive before declaring a missing confirmation.
            self._native_land_unconfirmed('failed', '10秒内未确认实际 AUTO.LAND 模式')
        return True

    def _tick_landing(self, control_state):
        self._tick_native_landing()

    def on_landed(self):
        if self.native_landing_requested:
            feedback = self.native_feedback or {}
            if (self.native_landing_state != 'confirmed'
                    or feedback.get('landed') is not True or feedback.get('armed') is not False):
                return
        return super().on_landed()

    def hold(self, reason):
        radar = self.task_position_feedback or {}
        feedback = self.native_feedback or {}
        self._native_hold_anchor = tuple(radar.get('position', self.position))
        self._native_hold_yaw = float(feedback.get('yaw', self.yaw))
        self._manual_height_motion = None
        self._manual_height_reference = None
        if self.manual_height is not None:
            self.manual_height.update(state='cancelled', reason=str(reason))
        FlightCore.hold(self, reason)
        self.native_flight_phase = 'HOLD'
        self.native_pending = None
        self.native_motion = self._native_takeoff_motion = None
        self.native_held = True
        self.native_goal = self._native_hold_anchor
        self.native_yaw = self._native_hold_yaw
        self.command_position = self._native_hold_anchor
        self.command_yaw = self._native_hold_yaw
        # Runtime keeps this task-frame anchor alive without an owner handoff.

    def snapshot(self):
        output = super().snapshot()
        output['native']['landing'] = dict(state=self.native_landing_state,
            attempts=self.native_landing_attempts, reason=self.native_landing_reason,
            requested_at_s=self._native_land_requested_at)
        if not self.full_native:
            return output
        if self.state == 'TAKEOFF':
            output['telemetry'].update(arrival_metric='reference_complete',
                                       height=dict(elapsed_s=0., required_s=0.))
        output['implementation'] = 'ego_px4_full_' + self.native_profile
        output['flight_controller'] = 'px4_native'
        output['controller'] = 'PX4_NATIVE'
        frame, label = task_frame(self.config), source_label(self.config)
        output['native'].update(full_flight=True, flight_phase=self.native_flight_phase,
                                position_source=label, frame=frame,
                                task_feedback=deepcopy(self.task_position_feedback),
                                motion_parameters=parameter_summary(self.config, self.native_takeoff_speed,
                                    two_waypoint_gate=self.coordinate_route_active))
        if self.manual_height is not None:
            output['manual_height'] = deepcopy(self.manual_height)
            if self.phase in ('MANUAL_HEIGHT_MOVE', 'MANUAL_HEIGHT_COMPLETE'):
                goal_z = self.manual_height['target_xyz'][2]
                output['telemetry'].update(arrival_metric='reference_complete',
                    height_target_z=goal_z, height_error_m=goal_z-self.position[2],
                    height=dict(elapsed_s=0., required_s=0.))
        if self.phase in ('LANDING', 'LANDED'):
            output['control'] = 'NONE'
            output['reference_source'] = 'AUTO_LAND'
            return output
        if output['control'] == 'EGO':
            output['reference_source'] = 'EGO'
            return output
        if self.state == 'TAKEOFF':
            p, v = self._native_takeoff_reference or (tuple(self.position), (0., 0., 0.))
            command = dict(position=p, velocity=v, yaw=self.command_yaw)
            source = 'TAKEOFF'
        elif self.state == 'MANUAL_HEIGHT':
            p, v = self._manual_height_reference
            command = dict(position=p, velocity=v, yaw=self.command_yaw)
            source = 'MANUAL_HEIGHT'
        elif self.state == 'HOLD':
            command = dict(position=self._native_hold_anchor or tuple(self.position),
                           velocity=(0., 0., 0.), yaw=self._native_hold_yaw
                           if self._native_hold_yaw is not None else self.yaw)
            source = 'HOLD'
        elif self.native_active and output.get('native_command'):
            command = deepcopy(output['native_command'])
            source = 'RECOVER' if self.phase == 'NATIVE_RECOVER' else 'ALIGN'
        else:
            command = dict(position=tuple(output['position_xyz']), velocity=(0., 0., 0.),
                           yaw=output['yaw'])
            source = 'HOLD'
        command.update(frame=frame, position_source=label)
        # Alignment remains p/v/yaw as in the tested native profile. EGO adds
        # its analytic acceleration and yaw-rate in the runtime adapter.
        output['native_command'] = command
        output['control'] = 'PX4_NATIVE'
        output['reference_source'] = source
        output['native']['target_xyz'] = list(command['position'])
        if self.task_position_feedback:
            output['native']['error_xyz'] = [a-b for a,b in
                zip(command['position'], self.task_position_feedback['position'])]
        output['telemetry'].update(coordinate_frame=frame, position_source=label)
        return output
