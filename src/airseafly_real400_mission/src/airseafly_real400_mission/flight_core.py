"""Shared recorded-route lifecycle; no target search or image-task state machine.

The caller supplies the current localization frame. Effects, goal identities,
arrival tolerances and payload acknowledgements retain the established semantics.
"""
from copy import deepcopy
import math

from .configuration import load_config
from .telemetry import present_engine
from .compute_groups import members


STATE_IDS = {'IDLE': 0, 'TAKEOFF': 1, 'APPROACH': 4, 'DROP': 6,
             'HOLD': 0, 'COMPLETE': 0, 'LAND': 8, 'MANUAL_HEIGHT': 10}


def merge(base, updates):
    result = deepcopy(base)
    for key, value in (updates or {}).items():
        result[key] = merge(result[key], value) if isinstance(value, dict) and isinstance(result.get(key), dict) else deepcopy(value)
    return result


def xyz(value):
    result = tuple(float(v) for v in value)
    if len(result) != 3 or not all(math.isfinite(v) for v in result):
        raise ValueError('position must contain three finite world coordinates')
    return result


class FlightCore:
    def __init__(self, config=None):
        self.config = merge(load_config(), config)
        self._effect_sequence = self._navigation_sequence = 0
        self._effects = []
        self.origin_xyz = self.takeoff_yaw = self.landing_xy = None
        self.started_at = self.ended_at = None
        self.state, self.phase, self.mode = 'IDLE', '', ''
        self.group = 'NONE'
        self._ready_groups = set()
        self._now, self._last_tick = 0., None
        self.position = self.command_position = (0., 0., 0.)
        self.yaw = self.command_yaw = 0.
        self.velocity = (0., 0., 0.)
        self.active = None
        self.reason = ''
        self.done = False
        self.drops_completed = self._progress_epoch = 0
        self._arrival_since = self._height_since = None
        self._goal = None
        self._goal_sent = self._goal_acked = self._navigation_finished = False
        self.goal_purpose = ''
        self._navigation_hold = self.position
        self._payload_request = self._payload_sent_at = self._payload_result = None
        self._settle_until = None
        self.payload_outcome = None
        self.locked_payload_slots = []
        self.late_payload_results = []
        self._land_sent = False
        self.cruise_z = self.config['flight']['cruise_height_m']
        self.drop_z = self.config['flight']['drop_height_m']
        self.takeoff_stability = None

    def _effect(self, kind, **values):
        self._effect_sequence += 1
        values.update(type=kind, effect_id=self._effect_sequence)
        self._effects.append(values)
        return self._effect_sequence

    def _group(self, group):
        if group != self.group:
            self.group = group
            self._ready_groups.intersection_update(members(group))

    def _state(self, state, phase='', group=None):
        if (state, phase) != (self.state, self.phase):
            self.phase_started_at = self._now
        self.state, self.phase = state, phase
        self._arrival_since = self._height_since = None
        if group is not None:
            self._group(group)

    def start(self, mode, position, yaw, now, target=None, origin=None, slot=None,
              stop_after=None, scan_xy=None):
        if mode != 'takeoff':
            raise ValueError('公共飞行层只接受南京任务的起飞初始化')
        position, yaw, now = xyz(position), float(yaw), float(now)
        if not math.isfinite(yaw) or not math.isfinite(now):
            raise ValueError('yaw/time must be finite')
        effect_sequence, navigation_sequence = self._effect_sequence, self._navigation_sequence
        self.__init__(self.config)
        self._effect_sequence, self._navigation_sequence = effect_sequence, navigation_sequence
        self.origin_xyz = position
        self.started_at = self._now = self._last_tick = now
        self.mode, self.position, self.yaw = mode, position, yaw
        self.takeoff_yaw = self.command_yaw = yaw
        self.cruise_z = position[2] + self.config['flight']['cruise_height_m']
        self.drop_z = position[2] + self.config['flight']['drop_height_m']
        self._state('TAKEOFF', 'ASCEND', 'NONE')
        self.command_position = (*position[:2], self.cruise_z)
        self._effect('takeoff', height_m=self.config['flight']['cruise_height_m'],
                     position_xyz=list(self.command_position), yaw=yaw)
        return self.snapshot()

    def _begin_navigation(self, target, purpose):
        self._navigation_hold = self.position
        self.command_position = xyz(target)
        self.command_yaw = self.yaw
        self.goal_purpose = purpose
        self._navigation_sequence += 1
        self._goal = self._navigation_sequence
        self._goal_sent = self._goal_acked = self._navigation_finished = False
        self.velocity = (0., 0., 0.)
        self._state('APPROACH', 'WAIT_EGO', 'EGO')

    def on_navigation_yaw(self, yaw):
        if self.state != 'APPROACH' or not self._goal_acked:
            return False
        self.command_yaw = float(yaw)
        return True

    def on_group_ready(self, group, ready=True):
        if group in members(self.group):
            self._ready_groups.add(group) if ready else self._ready_groups.discard(group)

    def on_goal_received(self, goal_id=None):
        if self.state == 'APPROACH' and self._goal_sent and (goal_id is None or goal_id == self._goal):
            self._goal_acked = True
            self.phase = 'FLY'

    def on_navigation_finished(self, goal_id):
        # Recorded Nanjing routes use position arrival, as before cleanup.
        return False

    def on_navigation_failed(self, goal_id):
        # Neither recorded-route mode enabled the old visual goal replacement.
        return False

    @property
    def navigation_arrival_policy(self):
        return 'position'

    @property
    def navigation_arrival_radius_m(self):
        return self.config['flight']['arrival_radius_m']

    @property
    def navigation_arrival_metric(self):
        return 'xyz'

    @property
    def navigation_arrival_dwell_s(self):
        return self.config['flight']['arrival_dwell_s']

    @property
    def coordinate_route_active(self):
        return False

    @property
    def final_route_deadline_enabled(self):
        return False  # Compatibility diagnostic; recorded routes have no deadline.

    @property
    def next_slot(self):
        return None

    def _tick_navigation(self, now):
        if not self._goal_sent:
            if 'EGO' in self._ready_groups:
                self._effect('goal', goal_id=self._goal, position_xyz=list(self.command_position),
                             yaw=self.command_yaw, purpose=self.goal_purpose)
                self._goal_sent = True
                self.phase = 'WAIT_GOAL_ACK'
            return
        if not self._goal_acked:
            return
        distance = math.sqrt(sum((actual-desired)**2 for actual, desired in zip(self.position, self.command_position)))
        if distance > self.navigation_arrival_radius_m + 1e-9:
            self._arrival_since = None
            return
        if self._arrival_since is None:
            self._arrival_since = now
        if now-self._arrival_since + 1e-9 >= self.navigation_arrival_dwell_s:
            self._on_custom_arrival()

    def _on_custom_arrival(self):
        return False

    def _begin_coordinate_landing(self):
        self.command_position, self.command_yaw = self.position, self.yaw
        self.velocity = (0., 0., 0.)
        self._state('LAND', 'WAIT_LAND', 'NONE')

    def on_payload_result(self, success, request_id=None, message='', outcome=None):
        if self._payload_request is None or request_id is not None and request_id != self._payload_request:
            return
        if self.state == 'HOLD' or self.payload_outcome in ('unknown', 'rejected', 'not_sent'):
            self.late_payload_results.append(dict(success=bool(success), request_id=request_id,
                                                 message=str(message), outcome=outcome))
            return
        if self.state == 'DROP' and self.phase == 'WAIT_ACK' and self._payload_result is None:
            self._payload_result = (bool(success), str(message))
            self.payload_outcome = outcome or ('accepted' if success else 'rejected')

    def _record_payload_success(self, released_slot, target, now, position):
        self.drops_completed += 1
        self._progress_epoch += 1

    def _tick_drop(self, now, dt, control_state):
        # The recorded route invokes this only after receiving a real result.
        if self.phase != 'WAIT_ACK' or self._payload_result is None:
            return
        success, message = self._payload_result
        if not success:
            if self.payload_outcome == 'unknown' and self.next_slot not in self.locked_payload_slots:
                self.locked_payload_slots.append(self.next_slot)
            self.hold(('释放结果未知，暂停且不自动重试：' if self.payload_outcome == 'unknown'
                       else '释放未发送，任务暂停：' if self.payload_outcome == 'not_sent'
                       else 'payload command rejected: ') + message)
            return
        self._record_payload_success(self.next_slot, self.active, now, self.position)
        self._settle_until = now + self.config['payload']['settle_s']
        self._state('DROP', 'SETTLE', 'NONE')

    def on_landed(self):
        if self.phase == 'LANDING' and self._land_sent:
            self._state('COMPLETE', 'LANDED', 'NONE')
            self.done = True
            self.reason = '{} payload commands acknowledged; landing completed'.format(self.drops_completed)

    def hold(self, reason):
        self.reason, self._effects = str(reason), []
        self.command_position, self.command_yaw = self.position, self.yaw
        self.velocity = (0., 0., 0.)
        self._state('HOLD', '', 'NONE')

    def tick(self, now, position, yaw, control_state=None):
        now, yaw, position = float(now), float(yaw), xyz(position)
        if not math.isfinite(now) or not math.isfinite(yaw):
            raise ValueError('time/yaw must be finite')
        if self._last_tick is not None and now < self._last_tick:
            raise ValueError('tick time must be monotonic')
        dt = 0. if self._last_tick is None else min(now-self._last_tick, .2)
        self._now, self._last_tick, self.position, self.yaw = now, now, position, yaw
        if self.state == 'TAKEOFF' and self._takeoff_arrived(now, control_state):
            if self.mode == 'takeoff':
                self._state('COMPLETE', 'TAKEOFF_COMPLETE', 'NONE')
                self.done = True
            else:
                self._after_takeoff(preserve_reference=True)
        elif self.state == 'APPROACH':
            self._tick_navigation(now)
        elif self.state == 'DROP':
            self._tick_drop(now, dt, control_state)
        elif self.state == 'LAND':
            self._tick_landing(control_state)
        if self.state in ('HOLD', 'COMPLETE') and self.ended_at is None:
            self.ended_at = now
        output = self.snapshot()
        output['effects'], self._effects = self._effects, []
        output['red_point_window'] = self.red_point_window()
        return output

    def red_point_window(self):
        """Engines without visual alignment never request the down detector."""
        return None

    def snapshot(self):
        control = 'EGO' if self.state == 'APPROACH' and self._goal_acked else 'HOLD'
        if self.state in ('IDLE', 'TAKEOFF') or self.phase in ('WAIT_LAND', 'LANDING', 'LANDED'):
            control = 'NONE'
        position = self._navigation_hold if self.state == 'APPROACH' and control == 'HOLD' else self.command_position
        return dict(telemetry=present_engine(self), state=self.state, state_id=STATE_IDS[self.state],
            phase=self.phase, mode=self.mode, desired_group=self.group, control=control,
            takeoff_stability=None, position_xyz=list(position), velocity_xyz=list(self.velocity),
            yaw=self.command_yaw, effects=[], goal_id=self._goal, goal_purpose=self.goal_purpose,
            navigation_goal_search=None, landing_xy=list(self.landing_xy) if self.landing_xy is not None else None,
            active_target_id=None, payload_slot=self.next_slot, payload_total=0,
            drops_completed=self.drops_completed, done=self.done, reason=self.reason,
            payload_outcome=self.payload_outcome, locked_payload_slots=list(self.locked_payload_slots),
            late_payload_results=deepcopy(self.late_payload_results))
