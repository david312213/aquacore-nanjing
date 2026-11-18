"""Fixed visual corrections: red once at drop points, H twice at landing.

At a drop point the aircraft descends to the red-point height first and aligns
once there; the green-ring stage is used only by its standalone tooling.

Observations already contain image-time FAST-LIO projections. This layer selects
a target and advances native segments; it never runs a detector or changes EGO.
The final landing reuses the drop sequence with H-pad targets, then descends to
a low height and hands off to the existing AUTO.LAND instead of releasing.
"""
from copy import deepcopy
import math

from .native_reference import LIO_FRAME


FRAME_PHASES = {'NATIVE_FIRST_FRAME': 1, 'NATIVE_SECOND_FRAME': 2,
                'LAND_FIRST_FRAME': 1, 'LAND_SECOND_FRAME': 2}
MOVE_PHASES = {'NATIVE_FIRST_MOVE': 1, 'NATIVE_SECOND_MOVE': 2,
               'LAND_FIRST_MOVE': 1, 'LAND_SECOND_MOVE': 2}
INTERMEDIATE_PHASES = ('NATIVE_INTERMEDIATE_DESCEND', 'LAND_INTERMEDIATE_DESCEND')
DESCEND_PHASES = ('NATIVE_DESCEND', 'LAND_DESCEND')
ALIGNMENT_PHASES = tuple(FRAME_PHASES) + tuple(MOVE_PHASES) + INTERMEDIATE_PHASES + DESCEND_PHASES + (
    'NATIVE_OFFSET_MOVE',)
DEFAULTS = dict(frame_timeout_s=15., motion_timeout_margin_s=15.,
                retake_limit=1, retry_xy_relax_m=.08, intermediate_height_m=.5)
# User-confirmed: 5 s per H image, no retake; on timeout descend at the current
# XY. Final height is above the takeoff reference, then AUTO.LAND touches down.
LANDING_DEFAULTS = dict(enabled=True, frame_timeout_s=5., final_height_m=.15, final_speed_mps=.30)
TARGETS = {('drop', 1): ('green_circle', 'green_candidates', '绿色圆心'),
           ('drop', 2): ('red_point', 'candidates', '红点'),
           ('land', 1): ('landing_pad', 'landing_candidates', 'H 标'),
           ('land', 2): ('landing_pad', 'landing_candidates', 'H 标')}


def landing_settings(value):
    result = dict(LANDING_DEFAULTS, **(value or {}))
    if not isinstance(result['enabled'], bool):
        raise ValueError('landing_alignment.enabled 必须是 true/false')
    for key in ('frame_timeout_s', 'final_height_m', 'final_speed_mps'):
        number = result[key]
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number) or number <= 0:
            raise ValueError('landing_alignment.%s 必须是正数' % key)
        result[key] = float(number)
    return result


class DeliveryAlignmentMixin:
    landing_alignment_supported = True

    def __init__(self, config=None):
        super().__init__(config)
        options = self.config.get('delivery_route_options', {})
        self.visual_alignment = dict(DEFAULTS)
        self.visual_alignment.update(options.get('visual_alignment', {}))
        self.landing_alignment = landing_settings(options.get('landing_alignment'))
        self.landing_alignment_result = None
        self.red_point_session_id = None
        self.red_point_image_sequence = None
        self._reset_route_alignment()

    @property
    def landing_alignment_enabled(self):
        return self.landing_alignment_supported and self.landing_alignment['enabled']

    def _alignment_limits(self):
        if self._alignment_purpose != 'land':
            return self.visual_alignment
        return dict(self.visual_alignment, retake_limit=0,
                    frame_timeout_s=self.landing_alignment['frame_timeout_s'])

    def set_red_point_session(self, session_id):
        """Manager authenticates sessions; None invalidates the current source."""
        if session_id != self.red_point_session_id:
            self.red_point_session_id = session_id
            self.red_point_image_sequence = None

    def red_point_window(self):
        """Identity of the image phase that consumes frames, or None outside it.

        The camera may be warm before this phase, but detection runs only in
        this window. A retake re-enters a frame phase with a new identity.
        The existing frame_timeout_s starts here without waiting for readiness.
        H waits use the same windows; the manager reads the LAND_ prefix to
        start the H detector instead of the red-point detector.
        """
        if self.state != 'DROP' or self.phase not in FRAME_PHASES or self._delivery_fallback_active:
            return None
        return '%s@%.6f' % (self.phase, self.phase_started_at)

    def _reset_route_alignment(self):
        self.native_fixes = []
        self.native_offset = None
        self.native_alignment_stage = 0
        self._delivery_retakes = 0
        self._delivery_fallback_active = False
        self._delivery_fallback_reason = ''
        self._delivery_forced_height = False
        self._delivery_release_pending = False
        self._delivery_release_height = None
        self._delivery_waiting_reason = ''
        self._delivery_offset_status = 'pending'
        self._delivery_recorded_xy = None
        self._intermediate_descent_timed_out = False
        self._alignment_purpose = 'drop'

    def _begin_alignment(self, xy, purpose):
        self._reset_route_alignment()
        self._alignment_purpose = purpose
        self._delivery_recorded_xy = tuple(xy)
        self.native_goal = tuple((self.native_lio_feedback or {}).get('position', self.position))
        self.native_motion = None
        if purpose == 'drop':
            # User-confirmed: one red-point alignment only. Descend straight to
            # the identification height over the recorded point, then image.
            self.native_alignment_stage = 2
            self._move((*self._delivery_recorded_xy, self.visual_alignment['intermediate_height_m']),
                       'NATIVE_INTERMEDIATE_DESCEND')
            return
        self.native_alignment_stage = 1
        self._native_phase(self._phase_name('FIRST_FRAME'))

    def _begin_route_alignment(self, point):
        self._begin_alignment(point['xy'], 'drop')

    def _begin_landing_alignment(self, xy):
        self._begin_alignment(xy, 'land')

    def _phase_name(self, suffix):
        return ('LAND_' if self._alignment_purpose == 'land' else 'NATIVE_') + suffix

    def _vertical_speed(self, phase):
        if phase == 'LAND_DESCEND':
            return self.landing_alignment['final_speed_mps']
        if phase == 'LAND_INTERMEDIATE_DESCEND':
            return super()._vertical_speed('NATIVE_INTERMEDIATE_DESCEND')
        return super()._vertical_speed(phase)

    def _delivery_tolerances(self, first=False):
        first = first or self.phase in ('NATIVE_FIRST_FRAME', 'NATIVE_FIRST_MOVE',
                                         'LAND_FIRST_FRAME', 'LAND_FIRST_MOVE')
        values = ([self.config['motion']['first_arrival_radius_m']] * 3 if first
                  else list(super()._delivery_tolerances()))
        if self._delivery_retakes:
            values[0] += self.visual_alignment['retry_xy_relax_m']
            values[1] += self.visual_alignment['retry_xy_relax_m']
        return values

    def _settled(self, now, first=False):
        # The original first-arrival sphere becomes per-axis tolerances on the
        # one approved retake, so only X/Y are relaxed, never Z.
        return super()._settled(now, first=first and not self._delivery_retakes)

    def _alignment_deadline(self):
        if self._delivery_release_pending:
            return None
        limits = self._alignment_limits()
        if self.phase in FRAME_PHASES:
            return self.phase_started_at + limits['frame_timeout_s']
        if self.phase in ALIGNMENT_PHASES:
            end = (self.native_motion.started + self.native_motion.duration
                   if self.native_motion is not None else self.phase_started_at)
            return end + limits['motion_timeout_margin_s']
        return None

    def on_red_point_observation(self, packet, now=None):
        """Consume a new frame from the manager-selected session, without age gates.

        Required envelope: existing airseafly.red_point.v1 fields plus session_id
        and monotonically increasing positive integer image_sequence.
        now is the ROS receipt clock, independent of the image timestamp. It
        prevents a callback between mission ticks from bypassing a deadline.
        Frames received while moving are consumed, but cannot alter its endpoint.
        """
        if not isinstance(packet, dict) or packet.get('schema') != 'airseafly.red_point.v1':
            return False
        sequence = packet.get('image_sequence')
        if (self.red_point_session_id is None or packet.get('session_id') != self.red_point_session_id
                or isinstance(sequence, bool) or not isinstance(sequence, int)
                or sequence <= 0
                or self.red_point_image_sequence is not None and sequence <= self.red_point_image_sequence):
            return False
        self.red_point_image_sequence = sequence
        if (self.state != 'DROP' or self.phase not in FRAME_PHASES
                or self._delivery_fallback_active or not self.native_ready(self._now)):
            return False
        try:
            received_at = self._now if now is None else float(now)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(received_at):
            return False
        received_at = max(self._now, received_at)
        if received_at + 1e-9 >= self._alignment_deadline():
            return False  # The tick owns the expired stage's one-way fallback.
        if packet.get('frame_id') != LIO_FRAME:
            return False
        anchor = self.native_fixes[-1]['target_xyz'][:2] if self.native_fixes else self._delivery_recorded_xy
        candidates = []
        stage = FRAME_PHASES[self.phase]
        expected_kind, key, _ = TARGETS[(self._alignment_purpose, stage)]
        supplied = packet.get(key, [])
        if not isinstance(supplied, (list, tuple)):
            return True
        for candidate in supplied:
            if not isinstance(candidate, dict) or not candidate.get('target_confirmed'):
                continue
            if candidate.get('class_name', 'red_point') != expected_kind:
                continue
            xy = candidate.get('xy_m')
            if (candidate.get('projection_status') != 'ok_near_level'
                    or not isinstance(xy, (list, tuple)) or len(xy) != 2
                    or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in xy)):
                continue
            candidates.append(candidate)
        if not candidates:
            return True
        selected = min(candidates, key=lambda candidate:
                       sum((candidate['xy_m'][i]-anchor[i])**2 for i in (0, 1)))
        stage = FRAME_PHASES[self.phase]
        goal = (*selected['xy_m'], self.native_goal[2])
        self.native_alignment_stage = stage
        self.native_yaw = self.native_feedback['yaw']
        self.native_fixes.append(dict(number=len(self.native_fixes)+1, stage=stage,
            target_xyz=list(goal), image_stamp_s=packet.get('image_stamp_s'),
            image_sequence=sequence, session_id=self.red_point_session_id,
            pose=deepcopy(packet.get('pose')), yaw=self.native_yaw, frame=LIO_FRAME,
            selection_anchor_xy=list(anchor), candidate=deepcopy(selected)))
        self._now = received_at
        self._move(goal, self._phase_name('FIRST_MOVE' if stage == 1 else 'SECOND_MOVE'))
        return True

    def _activate_delivery_fallback(self, reason):
        if not self._delivery_fallback_active:
            self._delivery_fallback_active = True
            self._delivery_fallback_reason = reason
            self._effect('native_delivery_fallback', reason=reason, phase=self.phase,
                         route_index=self.route_index, retakes=self._delivery_retakes)

    def _begin_alignment_descent(self):
        if self._alignment_purpose == 'land':
            # Straight down from the held XY: the last fix, or where the
            # current H wait started when no H was seen in time.
            z = self.native_origin[2] + self.landing_alignment['final_height_m']
            self._move((*self.native_goal[:2], z), 'LAND_DESCEND')
            return
        xy = self.native_fixes[-1]['target_xyz'][:2] if self.native_fixes else self._delivery_recorded_xy
        self._move((*xy, self.drop_z), 'NATIVE_DESCEND')

    def _finish_landing_alignment(self):
        """Hold the aligned XY at the low height; AUTO.LAND does the touchdown."""
        actual = tuple((self.native_lio_feedback or {}).get('position', self.position))
        self.landing_alignment_result = self._route_alignment_status()
        self._effect('landing_alignment_complete', fixes=len(self.native_fixes),
                     fallback=self._delivery_fallback_active, reason=self._delivery_fallback_reason,
                     target_xyz=list(self.native_goal), actual_xyz=list(actual))
        hold, yaw = tuple(self.native_goal), self.native_yaw
        self.native_active = False
        self.native_motion = None
        self._begin_coordinate_landing()
        self.command_position, self.command_yaw = hold, yaw

    def _retake_route_image(self):
        phase, old_goal = self.phase, tuple(self.native_goal)
        self._delivery_retakes += 1
        self.native_goal = (*self.native_lio_feedback['position'][:2], old_goal[2])
        self.native_motion = None
        self._native_phase(phase.replace('MOVE', 'FRAME'))
        self._effect('native_alignment_retake', phase=phase, retakes=self._delivery_retakes,
                     route_index=self.route_index, previous_target_xyz=list(old_goal),
                     tolerance_xyz_m=self._delivery_tolerances(first=MOVE_PHASES[phase] == 1))

    def _queue_route_release(self, now, freeze_actual=False):
        if not self._delivery_release_pending:
            self._delivery_release_pending = True
            if freeze_actual:
                self.native_goal = self.command_position = tuple(self.native_lio_feedback['position'])
                self.native_yaw = self.command_yaw = self.native_feedback['yaw']
                self.native_motion = None
                self.velocity = (0., 0., 0.)
        self._request_native_release(now)

    def _after_alignment_descent(self, now):
        forward, left = self.mount_offsets[str(self.route_drop_slot)]
        if forward == 0. and left == 0.:
            self._delivery_offset_status = 'zero'
            self._queue_route_release(now, freeze_actual=self._delivery_fallback_active)
            return
        # One immutable outlet target, including when descent timed out. Its Z
        # is the measured height; the approved forced descent is not reissued.
        c, s = math.cos(self.native_yaw), math.sin(self.native_yaw)
        dx, dy = -(c*forward-s*left), -(s*forward+c*left)
        anchor = (*self.native_goal[:2], self.native_lio_feedback['position'][2])
        goal = (anchor[0]+dx, anchor[1]+dy, anchor[2])
        self.native_offset = dict(slot=self.route_drop_slot, anchor_xyz=list(anchor),
            target_xyz=list(goal), mount_offset_body_m=[forward, left], delta_xy=[dx, dy])
        self._delivery_offset_status = 'moving'
        self._move(goal, 'NATIVE_OFFSET_MOVE')

    def _tick_route_alignment(self, now):
        if self.phase not in ALIGNMENT_PHASES:
            return False
        if not self.native_ready(now):
            self._delivery_waiting_reason = '等待原有定位输入'
            return True
        self._delivery_waiting_reason = ''
        if self._delivery_release_pending:
            self._request_native_release(now)
            return True
        deadline = self._alignment_deadline()
        expired = now + 1e-9 >= deadline
        if self.phase in FRAME_PHASES:
            if expired:
                self._activate_delivery_fallback('等待有效%s识别超时' %
                    TARGETS[(self._alignment_purpose, FRAME_PHASES[self.phase])][2])
                self._begin_alignment_descent()
        elif self.phase in MOVE_PHASES:
            stage = MOVE_PHASES[self.phase]
            if self._settled(now, first=stage == 1):
                if stage == 1:
                    self._move((*self.native_goal[:2], self.visual_alignment['intermediate_height_m']),
                               self._phase_name('INTERMEDIATE_DESCEND'))
                else:
                    self._begin_alignment_descent()
            elif expired:
                if self._delivery_retakes < self._alignment_limits()['retake_limit']:
                    self._retake_route_image()
                else:
                    self._activate_delivery_fallback('重拍后修正仍未到位' if self._delivery_retakes else '视觉修正到位超时')
                    self._begin_alignment_descent()
        elif self.phase == 'LAND_DESCEND':
            # Hand off once the low-height reference ends; no extra hover
            # near the ground waiting for a tolerance.
            if self.native_motion is None or self.native_motion.sample(now)[2]:
                self._finish_landing_alignment()
        elif self.phase in INTERMEDIATE_PHASES:
            arrived = self._settled(now)
            if arrived or expired:
                self._intermediate_descent_timed_out = not arrived
                if not arrived:
                    self.native_goal = self.command_position = tuple(self.native_lio_feedback['position'])
                    self.native_yaw = self.command_yaw = self.native_feedback['yaw']
                    self.velocity = (0., 0., 0.)
                    self._effect('native_intermediate_descent_timeout', actual_xyz=list(self.native_goal))
                self.native_alignment_stage = 2
                self.native_motion = None
                self._native_phase(self._phase_name('SECOND_FRAME'))
        elif self.phase == 'NATIVE_DESCEND':
            if self._settled(now):
                self._after_alignment_descent(now)
            elif expired:
                self._activate_delivery_fallback('投放下降到位超时')
                self._delivery_forced_height = (abs(self.native_lio_feedback['position'][2]-self.native_goal[2])
                                               > self._delivery_tolerances()[2] + 1e-9)
                self._after_alignment_descent(now)
        elif self.phase == 'NATIVE_OFFSET_MOVE':
            arrived = self._settled(now)
            if arrived or expired:
                if not arrived:
                    self._activate_delivery_fallback('投放口偏移到位超时')
                self._delivery_offset_status = 'completed' if arrived else 'forced'
                self._queue_route_release(now, freeze_actual=self._delivery_fallback_active)
        return True

    def _route_alignment_status(self):
        deadline = self._alignment_deadline() if self.state == 'DROP' else None
        waiting = self._delivery_waiting_reason
        target = TARGETS[(self._alignment_purpose, 2 if self.native_alignment_stage == 2 else 1)]
        if not waiting and self.phase in FRAME_PHASES:
            waiting = '等待有效%s识别' % target[2]
            if self._intermediate_descent_timed_out:
                waiting = '中间下降超时，保持实测位置；'+waiting
        limits = self._alignment_limits()
        return dict(stage=self.native_alignment_stage,
            required_samples=1 if self._alignment_purpose == 'drop' else 2,
            purpose=self._alignment_purpose, target_kind=target[0],
            intermediate_height_m=self.visual_alignment['intermediate_height_m'],
            intermediate_descent_timed_out=self._intermediate_descent_timed_out,
            samples=len(self.native_fixes), fixes=deepcopy(self.native_fixes),
            session_id=self.red_point_session_id, image_sequence=self.red_point_image_sequence,
            recorded_xy=self._delivery_recorded_xy, offset=deepcopy(self.native_offset),
            fallback=dict(active=self._delivery_fallback_active, reason=self._delivery_fallback_reason,
                retakes=self._delivery_retakes, retake_limit=limits['retake_limit'],
                frame_timeout_s=limits['frame_timeout_s'],
                remaining_s=None if deadline is None else max(0., deadline-self._now),
                waiting_reason=waiting, effective_tolerance_xyz_m=self._delivery_tolerances(),
                offset_status=self._delivery_offset_status, forced_height=self._delivery_forced_height,
                release_actual_height_m=self._delivery_release_height))
