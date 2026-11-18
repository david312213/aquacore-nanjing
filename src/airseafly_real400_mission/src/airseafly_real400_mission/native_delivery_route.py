"""Configured ordered laps with red-point corrections at last-lap delivery points.

This engine chooses goals and payload actions only. NativeRuntime, the existing
EGO worker and px4ctrl remain the sole reference/control path. Navigation uses
the existing position-arrival policy; descent and recovery reuse native motion.
"""
from copy import deepcopy
import math

from .flight_core import FlightCore
from .guided_route import LOOP_SECTION_NAMES, all_route_points, validate_route
from .native_segment import NativeSegmentMixin
from .native_flight import NativeFlightMixin
from .native_parameters import gate_height
from .delivery_alignment import DeliveryAlignmentMixin


def validate_laps(value):
    """Cruise laps before the delivery loop; 0 goes straight to delivery."""
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError('delivery_route_options.laps 必须是 0 或正整数（巡航闭环圈数）')
    return value


def build_itinerary(document, laps=1):
    """Flight order for `laps` cruise laps followed by the delivery pass.

    Version 4 files have separate entry, cruise-loop, delivery-loop and return
    sections. Older one-route files fly their original order with the shared
    route as the cruise loop: laps cruise laps, then one dropping lap.
    """
    laps = validate_laps(laps)
    document = validate_route(document, require_complete=True)
    if document['version'] == 4:
        return _build_loop_itinerary(document, laps)
    return _build_shared_itinerary(document, laps+1)


def _build_loop_itinerary(document, laps):
    """Entry (lap 0) -> cruise loop x laps -> delivery loop with drops (lap
    laps+1) -> return points -> HOME -> land.

    Every cruise lap but the last closes back on the loop's first point; the
    last lap ends on the loop's last point and flies straight to the delivery
    loop's first point (user-confirmed 2026-11-19).
    """
    steps = []
    drop_lap = laps+1

    def navigation(source, index, lap, closing=False):
        point = document[source][index]
        steps.append(dict(kind='navigation', lap=lap, xy=list(point['xy']),
                          height_role='gate' if point['kind'] == 'gate' else 'cruise',
                          route_index=index, route_source=source, stage=point['stage'],
                          closing=closing, completes_lap=lap if closing else None, point=deepcopy(point)))

    for index in range(len(document['entry'])):
        navigation('entry', index, 0)
    cruise = document['cruise_loop']
    for lap in range(1, laps+1 if cruise else 1):
        for index in range(len(cruise)):
            # Every later lap starts on the previous lap's closure point.
            if index or lap == 1:
                navigation('cruise_loop', index, lap)
        if lap < laps:
            navigation('cruise_loop', 0, lap, closing=True)
        else:
            # The last lap ends on its last point: that arrival completes it.
            steps[-1]['completes_lap'] = lap
    for index, point in enumerate(document['delivery_loop']):
        navigation('delivery_loop', index, drop_lap)
        if point['kind'] == 'drop':
            steps.append(dict(kind='drop', lap=drop_lap, xy=list(point['xy']),
                              height_role='drop', recover_height_role='cruise',
                              route_index=index, route_source='delivery_loop',
                              stage=point['stage'], drop_slot=point['drop_slot'], point=deepcopy(point)))
    for index in range(len(document['return'])):
        navigation('return', index, drop_lap)
    steps.append(dict(kind='navigation', lap=None, xy=list(document['home']),
                      height_role='cruise', route_index=None, route_source='home', stage='home',
                      closing=False, completes_lap=drop_lap))
    steps.append(dict(kind='land', lap=None, xy=list(document['home']),
                      height_role='land', route_index=None, route_source='home', stage='home'))
    return steps


def _build_shared_itinerary(document, laps):
    """Legacy one-route order; home is not part of any lap.

    laps N: N-1 loaded laps, then the same order dropping at each drop point
    on lap N (contest: 2). laps 1: one lap only, dropping as it goes.
    Every lap, the dropping lap included, flies all recorded points and closes
    back at the first point; HOME is reached from that first point (2026-11-18).
    """
    drop_lap = validate_laps(laps)
    document = validate_route(document, require_complete=True)
    points, steps = document['route'], []

    def navigation(index, lap, closing=False):
        point = points[index]
        steps.append(dict(kind='navigation', lap=lap, xy=list(point['xy']),
                          height_role='gate' if point['kind'] == 'gate' else 'cruise',
                          route_index=index, route_source='shared', stage=point['stage'], closing=closing,
                          completes_lap=lap if closing else None))

    def drop(index):
        point = points[index]
        steps.append(dict(kind='drop', lap=drop_lap, xy=list(point['xy']),
                          height_role='drop', recover_height_role='cruise',
                          route_index=index, route_source='shared',
                          stage=point['stage'], drop_slot=point['drop_slot']))

    for lap in range(1, drop_lap+1):
        # Every lap after the first reaches its first point from the previous
        # loaded lap's closure, which already is that point.
        dropping = lap == drop_lap
        for index in range(len(points)):
            if index or lap == 1:
                navigation(index, lap)
            if dropping and points[index]['kind'] == 'drop':
                drop(index)
        # The dropping lap also closes at the first point (already dropped
        # there if it is a drop point; closing never drops again).
        navigation(0, lap, closing=True)
    steps.append(dict(kind='navigation', lap=None, xy=list(document['home']),
                      height_role='cruise', route_index=None, route_source='home', stage='home',
                      closing=False, completes_lap=None))
    steps.append(dict(kind='land', lap=None, xy=list(document['home']),
                      height_role='land', route_index=None, route_source='home', stage='home'))
    return steps


def describe_laps(itinerary):
    """Plain-text flight order: one line per lap, then the return to HOME.

    Points use their recorded number; [投] marks a drop, and each loaded lap's
    closure back to the first point is marked.
    """
    if any(step.get('route_source') in LOOP_SECTION_NAMES for step in itinerary):
        return describe_sections(itinerary)

    def name(step):
        stage = step['stage']
        if stage.startswith('drop') and stage[4:].isdigit():
            text = '投放'+stage[4:]
        else:
            text = '圆环' if step['height_role'] == 'gate' else '普通'
        return '%d.%s' % (step['route_index']+1, text)
    laps = {}
    for step in itinerary:
        if step['lap'] is None:
            continue
        names = laps.setdefault(step['lap'], [])
        if step['kind'] == 'navigation':
            names.append(name(step)+('（闭环）' if step['closing'] else ''))
        elif step['kind'] == 'drop':
            # The dropping lap's first drop can happen at the previous lap's closure point.
            if not names or names[-1].split('（', 1)[0] != name(step):
                names.append(name(step))
            names[-1] += '[投]'
    lines = ['第%d圈：%s' % (lap, ' → '.join(names)) for lap, names in sorted(laps.items())]
    last = laps[max(laps)][-1].split('（', 1)[0] if laps else 'HOME'
    return lines+['返航：%s → HOME 降落' % last]


def _section_point_name(step):
    stage = step['stage']
    if stage.startswith('drop') and stage[4:].isdigit():
        text = '投放'+stage[4:]
    else:
        text = '圆环' if step['height_role'] == 'gate' else '普通'
    return '%d.%s' % (step['route_index']+1, text)


def describe_sections(itinerary):
    """Plain-text order of a version 4 route: one line per section."""
    sections, cruise_laps = {}, 0
    for step in itinerary:
        source = step.get('route_source')
        if source not in LOOP_SECTION_NAMES:
            continue
        names = sections.setdefault(source, [])
        if source == 'cruise_loop':
            cruise_laps = max(cruise_laps, step['lap'])
            if step['lap'] != 1 or step.get('closing'):
                continue  # Every cruise lap flies the same order; list one.
        if step['kind'] == 'drop':
            names[-1] += '[投]'
        else:
            names.append(_section_point_name(step)+('（闭环）' if step.get('closing') else ''))
    lines = []
    for source in ('entry', 'cruise_loop', 'delivery_loop', 'return'):
        names = sections.get(source)
        if not names:
            continue
        label = LOOP_SECTION_NAMES[source]
        text = ' → '.join(names)
        if source == 'cruise_loop':
            label += ' ×%d圈' % cruise_laps
            if cruise_laps > 1:
                text += '（前 %d 圈回到 %s 闭环；第 %d 圈从 %s 直接进入投放环）' % (
                    cruise_laps-1, names[0], cruise_laps, names[-1])
            else:
                text += '（从 %s 直接进入投放环）' % names[-1]
        lines.append('%s：%s' % (label, text))
    return lines+['→ HOME 降落']


def delivery_timing_mode(config):
    """navigation.timing for the loaded-lap leg; unset keeps the legacy switch.

    --no-loop-retiming (local_retiming_enabled False) always means original.
    """
    from .local_retiming import timing_mode
    if not config.get('delivery_route_options', {}).get('local_retiming_enabled', True):
        return 'original'
    return timing_mode(config, default='optimized')


def build_leg(config, itinerary, first, start_xyz, ground_z):
    """One fixed leg: navigation steps from `first` up to the next drop or land.

    Points in between are passed continuously; the leg ends at rest at its
    last point. Heights and the optional gate speed match the EGO goals.
    """
    from .fixed_trajectory import build_fixed_leg
    from .route_profiles import gate_velocity
    last = first
    while itinerary[last+1]['kind'] == 'navigation':
        last += 1
    heights = dict(cruise=config['flight']['cruise_height_m'], gate=gate_height(config))
    cap = gate_velocity(config)
    steps = itinerary[first:last+1]
    stops = [(*step['xy'], ground_z+heights[step['height_role']]) for step in steps]
    caps = [cap if step['height_role'] == 'gate' else None for step in steps]
    limits = config['planning']
    trajectory = build_fixed_leg(start_xyz, stops, limits['max_velocity_mps'], limits['max_acceleration_mps2'],
                                 caps, [dict(itinerary_index=index) for index in range(first, last+1)])
    mode = delivery_timing_mode(config)
    # Only the loaded-lap leg is retimed; the leg that ends in landing (the
    # dropping lap's closure plus HOME) keeps the original timing.
    if (mode != 'original' and itinerary[last+1]['kind'] != 'land'
            and any(step.get('closing') or step.get('route_source') == 'cruise_loop' for step in steps)):
        from .local_retiming import retime_loaded_leg, timing_dynamics
        trajectory = retime_loaded_leg(trajectory, caps,
            [(step.get('route_source', 'shared'), step['route_index']) for step in steps],
            limits['max_velocity_mps'], limits['max_acceleration_mps2'], timing_dynamics(config, mode))
    return last, trajectory


def plan_fixed_legs(config, itinerary, ground_xyz):
    """Every fixed leg from recorded points, each later leg from the previous stop.

    Used before takeoff and by the preview. In flight each leg is rebuilt from
    the actual post-drop reference, so these are close but not exact.
    """
    legs, index = [], 0
    start = (*ground_xyz[:2], ground_xyz[2]+config['flight']['cruise_height_m'])
    while index < len(itinerary):
        if itinerary[index]['kind'] != 'navigation':
            index += 1
            continue
        last, trajectory = build_leg(config, itinerary, index, start, ground_xyz[2])
        legs.append(dict(first=index, last=last, trajectory=trajectory))
        start, index = trajectory.end, last+1
    return legs


class DeliveryRouteEngine(DeliveryAlignmentMixin, NativeFlightMixin, NativeSegmentMixin, FlightCore):
    mission_mode = 'delivery_route'
    route_laps = 1  # Cruise laps; drop tests have none, only the contest route reads the option.

    def _state(self, state, phase='', group=None):
        # Mapping continues while native references perform descent, release
        # and recovery. Worker lifetime does not select the control source.
        # This also covers WAIT_ACK and SETTLE entered by the shared mixins.
        # Fixed legs never use EGO, so there is no map to keep alive.
        if state == 'DROP':
            group = 'NONE' if getattr(self, 'fixed_navigation', False) else 'EGO'
        super()._state(state, phase, group)

    def __init__(self, config=None):
        config = deepcopy(config or {})
        geometry = config.setdefault('motion', {})
        geometry.setdefault('native_profile', 'waypoint')
        geometry.setdefault('first_arrival_radius_m', .10)
        super().__init__(config)
        self._load_delivery_plan()
        self.itinerary_index = -1
        supplied = self.config.get('delivery_route_options', {}).get('mount_offsets_body_m', {})
        self.mount_offsets = {str(slot): (0., 0.) for slot in range(1, 5)}
        for slot, pair in supplied.items():
            if (str(slot) not in self.mount_offsets or not isinstance(pair, (list, tuple))
                    or len(pair) != 2 or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                            or not math.isfinite(v) for v in pair)):
                raise ValueError('mount_offsets_body_m 必须按 1～4 配置 [前, 左] 米制偏移')
            self.mount_offsets[str(slot)] = tuple(float(v) for v in pair)
        self.route_lap = 1
        self.completed_laps = 0
        self.route_index = None
        self.route_source = 'delivery'
        self.route_closing = False
        self.route_returning_home = False
        self.route_released_slots = []
        self.route_released_points = set()
        self.route_drop_slot = None
        self.navigation_backend = self.config.get('navigation', {}).get('backend', 'ego')
        if self.navigation_backend not in ('ego', 'fixed_min_snap'):
            raise ValueError('navigation.backend 必须为 ego 或 fixed_min_snap')
        self.fixed_trajectory = None
        self.fixed_leg_sequence = 0
        self._fixed_leg = None
        self._fixed_progress = None
        self._fixed_event_index = 0
        self._fixed_last_end = None
        self._fixed_yaw = None
        self._prepared_fixed_first_leg = None

    @property
    def fixed_navigation(self):
        return self.navigation_backend == 'fixed_min_snap'

    def _preflight_itinerary(self, position):
        """Itinerary solved before takeoff. Drop tests return to the takeoff XY."""
        return self.itinerary

    def _load_delivery_plan(self):
        """Choose recorded route content independently of the shared lifecycle."""
        self.route_document = validate_route(self.config['delivery_route'], require_complete=True)
        # Every recorded point in flying order; each step also carries its own point.
        self.route_points = all_route_points(self.route_document)
        self.route_home = tuple(self.route_document['home'])
        self.route_laps = validate_laps(self.config.get('delivery_route_options', {}).get('laps', 1))
        self.itinerary = build_itinerary(self.route_document, self.route_laps)

    def start(self, mode, position, yaw, now, target=None, origin=None, slot=None,
              stop_after=None, scan_xy=None):
        if mode != self.mission_mode:
            raise ValueError('此投放任务只使用 '+self.mission_mode+' 任务模式')
        prepared = []
        if self.fixed_navigation:
            # Solve every leg before queuing takeoff, so invalid input or a
            # missing math library cannot leave a partially started mission.
            prepared = plan_fixed_legs(self.config, self._preflight_itinerary(position), tuple(position))
        # Reuse the original ground registration and native takeoff lifecycle.
        super().start('takeoff', position, yaw, now, origin=origin)
        # FlightCore.start resets the engine; keep the solved first leg after
        # that reset, so takeoff completion does not rerun the optimizer.
        self._prepared_fixed_first_leg = (prepared[0] if prepared and
            prepared[0]['trajectory'].summary().get('local_retiming') else None)
        self.mode = self.mission_mode
        self.landing_xy = self.route_home
        return self.snapshot()

    def begin_native_flight(self):
        super().begin_native_flight()
        # Native registration can refresh the ground pose after start().
        # Refresh this reference while still preparing on the ground, rather
        # than run the optimizer when the takeoff reference finishes.
        prepared = self._prepared_fixed_first_leg
        if prepared is not None and self.state == 'TAKEOFF':
            start = (*self.origin_xyz[:2], self.cruise_z)
            if start != prepared['trajectory'].nodes[0]:
                try:
                    last, trajectory = build_leg(self.config, self.itinerary, prepared['first'],
                                                 start, self.origin_xyz[2])
                    self._prepared_fixed_first_leg = dict(prepared, last=last, trajectory=trajectory)
                except (RuntimeError, ValueError):
                    # Preserve the original in-flight solve/error handling.
                    self._prepared_fixed_first_leg = None



    def _after_takeoff(self, preserve_reference=False, preserve_yaw=False):
        # The inherited takeoff completion calls this hook. It starts the first
        # recorded point directly; visual worker readiness never gates flight.
        self._advance_itinerary()
        self._navigation_hold = (*self.origin_xyz[:2], self.cruise_z)

    @property
    def next_slot(self):
        return self.route_drop_slot

    def _advance_itinerary(self):
        # A completed recovery reference can still have measured height error.
        # Preserve its endpoint during EGO preparation. Ordinary navigation
        # arrivals retain the existing current-position wait reference.
        recovering = self.phase == 'NATIVE_RECOVER'
        recovery_goal = tuple(self.native_goal) if recovering and self.native_goal is not None else None
        recovery_z = self.native_goal[2] if recovering else None
        step = self._select_step(self.itinerary_index+1)
        self.native_active = False
        self.native_motion = None
        self.native_pending = None
        self.route_drop_slot = None
        if step['kind'] == 'drop':
            self._begin_route_drop(self._step_point(step))
        elif step['kind'] == 'hover':
            self._begin_drop_hover(recovery_goal)
        elif step['kind'] == 'land' and self.landing_alignment_enabled:
            self._begin_native_alignment()
            self._effect('landing_alignment_start', home_xy=list(step['xy']))
            self._begin_landing_alignment(step['xy'])
        elif step['kind'] == 'land':
            self._begin_coordinate_landing()
        elif self.fixed_navigation:
            self._begin_fixed_leg(recovery_goal)
        else:
            # Reuse the same EGO worker across both laps and native drops.
            # Goal identities isolate the previous point's acknowledgements
            # and trajectory without discarding the running map.
            height = gate_height(self.config) if step['height_role'] == 'gate' else self.config['flight']['cruise_height_m']
            self._begin_navigation((*step['xy'], self.origin_xyz[2]+height), self.mission_mode)
            if recovery_z is not None:
                self._navigation_hold = (*self.position[:2], recovery_z)
            self._effect('delivery_route_point', lap=step['lap'], index=self.route_index,
                         route_source=self.route_source,
                         stage=step['stage'], action_kind=step['kind'], closing=self.route_closing,
                         position_xyz=list(self.command_position))

    def _begin_drop_hover(self, recovery_goal):
        """Finish by holding the recovery endpoint (or current reference).

        No landing is requested; hold.sh / safe_land.sh end it as usual.
        """
        goal = recovery_goal or tuple(self.command_position)
        self.command_position = tuple(goal)
        yaw = getattr(self, 'native_yaw', None)
        self.command_yaw = self.yaw if yaw is None else yaw
        self.velocity = (0., 0., 0.)
        self._state('COMPLETE', 'DROP_HOVER', 'NONE')
        self.done = True
        self.reason = '投放完成，持续悬停在回升位置；可用 hold.sh 保持或 safe_land.sh 降落'
        self._effect('drop_hover', position_xyz=list(goal))

    def _step_point(self, step=None):
        """The recorded point of a step; older itineraries index route_points."""
        if step is None:
            if not 0 <= self.itinerary_index < len(self.itinerary):
                return None
            step = self.itinerary[self.itinerary_index]
        if step.get('point') is not None:
            return step['point']
        index = step.get('route_index')
        return None if index is None else self.route_points[index]

    def _route_stage(self):
        """(stage, name) for the terminal and panel."""
        names = dict(entry=('entry', '进场'), cruise_loop=('loaded_lap', '巡航闭环'),
                     delivery_loop=('delivery', '投放环'), **{'return': ('return', '返航')})
        if self.route_source in names:
            return names[self.route_source]
        if (self.route_document or {}).get('version') == 4:
            return 'return', '返航'
        # Shared route: laps cruise laps, then the dropping lap.
        return ('loaded_lap', '满载绕圈') if self.route_lap <= self.route_laps else ('delivery', '投放阶段')

    def _section_length(self):
        if (self.route_document or {}).get('version') == 4 and self.route_source in LOOP_SECTION_NAMES:
            return len(self.route_document[self.route_source])
        return len(self.route_points)

    def _select_step(self, index):
        self.itinerary_index = index
        step = self.itinerary[index]
        self.route_index = step['route_index']
        self.route_source = step.get('route_source', 'home' if step['stage'] == 'home' else 'delivery')
        self.route_lap = step['lap'] or self.route_lap
        self.route_closing = step.get('closing', False)
        self.route_returning_home = step['stage'] == 'home'
        return step

    def _step_reached(self, kind='delivery_route_point_arrived', **extra):
        step = self.itinerary[self.itinerary_index]
        self._effect(kind, lap=step['lap'], index=self.route_index, stage=step['stage'],
                     route_source=self.route_source, closing=self.route_closing, **extra)
        if step.get('completes_lap'):
            self.completed_laps = step['completes_lap']
            self._effect('delivery_route_lap_complete', completed_laps=self.completed_laps)
        if self.route_returning_home:
            self._effect('delivery_route_home_arrived', home_xy=list(self.route_home))

    def _on_custom_arrival(self):
        if self.goal_purpose != self.mission_mode:
            return super()._on_custom_arrival()
        self._step_reached()
        self._advance_itinerary()
        return True

    def _begin_fixed_leg(self, recovery_goal):
        """Fly every navigation step up to the next drop or land as one leg.

        A leg starts where the reference is: the recovery endpoint after a
        drop, the previous leg's end after a skipped drop, else the takeoff end.
        """
        if recovery_goal is not None:
            start, yaw = recovery_goal, self.native_yaw
        elif self._fixed_last_end is not None:
            start, yaw = self._fixed_last_end, self._fixed_yaw
        else:
            start, yaw = (*self.origin_xyz[:2], self.cruise_z), self.takeoff_yaw
        first = self.itinerary_index
        try:
            prepared = self._prepared_fixed_first_leg
            self._prepared_fixed_first_leg = None
            if (prepared is not None and first == prepared['first'] and
                    tuple(start) == prepared['trajectory'].nodes[0]):
                last, trajectory = prepared['last'], prepared['trajectory']
            else:
                last, trajectory = build_leg(self.config, self.itinerary, first, start, self.origin_xyz[2])
        except (RuntimeError, ValueError) as exc:
            self.hold('固定航段求解失败：'+str(exc))
            return
        self.fixed_leg_sequence += 1
        trajectory.leg_sequence = self.fixed_leg_sequence
        self.fixed_trajectory = trajectory
        self._fixed_leg = (first, last)
        self._fixed_progress = trajectory.sample(0.)
        self._fixed_event_index = 0
        self._fixed_yaw = float(yaw)
        self.command_position = tuple(self._fixed_progress['position'])
        self.command_yaw = self._fixed_yaw
        self.velocity = (0., 0., 0.)
        self.goal_purpose = self.mission_mode
        self._state('APPROACH', 'FIXED_TRAJECTORY', 'NONE')
        self._effect('delivery_route_fixed_leg', sequence=self.fixed_leg_sequence,
                     first_step=first, last_step=last, duration_s=trajectory.duration,
                     local_retiming=trajectory.summary().get('local_retiming'),
                     start_xyz=list(start), end_xyz=list(trajectory.end))
        for index in range(first, last+1):
            step = self.itinerary[index]
            height = gate_height(self.config) if step['height_role'] == 'gate' else self.config['flight']['cruise_height_m']
            self._effect('delivery_route_point', lap=step['lap'], index=step['route_index'],
                         route_source=step.get('route_source', 'delivery'),
                         stage=step['stage'], action_kind=step['kind'], closing=step.get('closing', False),
                         position_xyz=[*step['xy'], self.origin_xyz[2]+height], fixed_leg=self.fixed_leg_sequence)

    def on_fixed_progress(self, progress):
        """Consume the runtime's last sample; never advance from manager time."""
        if (not self.fixed_navigation or self.fixed_trajectory is None or self.state != 'APPROACH'
                or self.phase not in ('FIXED_TRAJECTORY', 'FIXED_ARRIVAL') or progress is None):
            return False
        elapsed = float(progress['elapsed_s'])
        if elapsed < self._fixed_progress['elapsed_s']:
            return False
        self._fixed_progress = deepcopy(progress)
        self.command_position = tuple(progress['position'])
        self.velocity = tuple(progress['velocity'])
        self.command_yaw = self._fixed_yaw
        if self.phase != 'FIXED_TRAJECTORY':
            return True
        measured = progress.get('actual_position_xyz')
        actual = list(self.position if measured is None else measured)
        events = self.fixed_trajectory.events
        # The leg's own end is reached by actual arrival below, not by time.
        while (self._fixed_event_index < len(events)-1 and
               events[self._fixed_event_index]['elapsed_s'] <= elapsed):
            event = events[self._fixed_event_index]
            self._fixed_event_index += 1
            self._select_step(event['itinerary_index'])
            self._step_reached('delivery_route_reference_passed', elapsed_s=event['elapsed_s'],
                               actual_position_xyz=actual)
        if progress['completed']:
            self._select_step(self._fixed_leg[1])
            self.command_position = tuple(self.fixed_trajectory.end)
            self.velocity = (0., 0., 0.)
            self._arrival_since = None
            # Option A: hand over with the existing XYZ arrival radius and dwell.
            self._state('APPROACH', 'FIXED_ARRIVAL', 'NONE')
            self._effect('delivery_route_fixed_leg_reference_complete', sequence=self.fixed_leg_sequence,
                         elapsed_s=elapsed, position_xyz=list(self.command_position), actual_position_xyz=actual)
        return True

    def _tick_navigation(self, now):
        if not self.fixed_navigation:
            return super()._tick_navigation(now)
        if self.phase != 'FIXED_ARRIVAL':
            return  # The runtime samples the reference; see on_fixed_progress.
        if self._fixed_leg_lands:
            self._tick_home_arrival(now)
            return
        if math.dist(self.position, self.fixed_trajectory.end) > self.navigation_arrival_radius_m + 1e-9:
            self._arrival_since = None
            return
        if self._arrival_since is None:
            self._arrival_since = now
        if now-self._arrival_since + 1e-9 >= self.navigation_arrival_dwell_s:
            self._finish_fixed_leg()

    @property
    def _fixed_leg_lands(self):
        """The leg ends at the landing position (HOME or the takeoff XY)."""
        last = self._fixed_leg[1] if self._fixed_leg else None
        return last is not None and last+1 < len(self.itinerary) and self.itinerary[last+1]['kind'] == 'land'

    @property
    def home_arrival_xy_m(self):
        return self.config['flight'].get('home_arrival_xy_m', .05)

    def _tick_home_arrival(self, now):
        """User-confirmed 2026-11-01: horizontal XY within 5 cm for the existing
        dwell, height not included. 15 s after the reference ends (the approved
        motion grace) landing starts at the actual position instead."""
        error = math.dist(self.position[:2], self.fixed_trajectory.end[:2])
        inside = error <= self.home_arrival_xy_m + 1e-9
        if not inside:
            self._arrival_since = None
        elif self._arrival_since is None:
            self._arrival_since = now
        if inside and now-self._arrival_since + 1e-9 >= self.navigation_arrival_dwell_s:
            self._finish_fixed_leg()
        elif now + 1e-9 >= self.phase_started_at + self.visual_alignment['motion_timeout_margin_s']:
            self._effect('home_arrival_timeout', xy_error_m=error, xy_tolerance_m=self.home_arrival_xy_m,
                         actual_position_xyz=list(self.position))
            self._finish_fixed_leg()

    def _finish_fixed_leg(self):
        self._step_reached(actual_position_xyz=list(self.position))
        self._fixed_last_end = tuple(self.fixed_trajectory.end)
        self.fixed_trajectory = None
        self._fixed_leg = None
        self._advance_itinerary()

    def hold(self, reason):
        super().hold(reason)
        # Revocation is permanent for this task; progress cannot restart it.
        self.fixed_trajectory = None

    def _native_phase(self, phase):
        self._state('DROP', phase, 'EGO')

    def _begin_route_drop(self, point):
        if self.route_index in self.route_released_points:
            self._effect('delivery_route_point_skipped', index=self.route_index,
                         route_source=self.route_source,
                         reason='本趟该录制投放点已收到成功回执')
            self._advance_itinerary()
            return
        self.route_drop_slot = point['drop_slot']
        self._begin_native_alignment()
        self._begin_route_alignment(point)

    def _begin_native_alignment(self):
        self.native_active, self.native_held = True, False
        self.native_pending = None
        self.native_yaw = float((self.native_feedback or {}).get('yaw', self.yaw))
        self.native_epoch = self._now
        self.native_arrivals = []

    def _tick_drop(self, now, dt, control_state):
        if self._tick_route_alignment(now):
            return
        if self.phase == 'WAIT_ACK':
            # Native automatic release already waits for its real reply rather
            # than invoking the legacy ACK timer. Preserve that behavior.
            if self._payload_result is not None:
                FlightCore._tick_drop(self, now, dt, control_state)
        elif self.phase == 'SETTLE':
            if now + 1e-9 >= self._settle_until:
                self._recover('DELIVERY_ROUTE_NEXT')
        elif self.phase == 'NATIVE_RECOVER':
            if self._recovery_arrived(now):
                self._advance_itinerary()
        else:
            super()._tick_drop(now, dt, control_state)

    def _record_payload_success(self, released_slot, target, now, position):
        super()._record_payload_success(released_slot, target, now, position)
        self.route_released_slots.append(released_slot)
        self.route_released_points.add(self.route_index)
        result = dict(route_index=self.route_index, slot=released_slot,
                      route_source=self.route_source,
                      release_acknowledged=True, alignment=self._route_alignment_status())
        self.native_results.append(result)
        self._effect('delivery_route_release_ack', lap=self.route_lap,
                     index=self.route_index, slot=released_slot, route_source=self.route_source,
                     physical_release_verified=False, alignment=deepcopy(result['alignment']))

    def snapshot(self):
        output = super().snapshot()
        output['implementation'] = 'delivery_route_px4_native'
        output['payload_total'] = sum(point['kind'] == 'drop' for point in self.route_points)
        if self.state == 'DROP' and self.native_active and self.native_goal is not None:
            # Display the fixed native endpoint; native_command remains the
            # sampled segment reference used by the runtime control path.
            output['position_xyz'] = list(self.native_goal)
            output['yaw'] = self.native_yaw
            actual = (self.native_lio_feedback or {}).get('position', self.position)
            yaw = float((self.native_feedback or {}).get('yaw', self.yaw))
            error = [goal-position for goal, position in zip(self.native_goal, actual)]
            output['telemetry'].update(horizontal_distance_m=math.hypot(*error[:2]),
                distance_3d_m=math.sqrt(sum(value*value for value in error)),
                height_target_z=self.native_goal[2], height_error_m=error[2],
                actual_yaw_rad=yaw, target_yaw_rad=self.native_yaw,
                yaw_error_rad=math.atan2(math.sin(self.native_yaw-yaw), math.cos(self.native_yaw-yaw)))
        stage, stage_name = self._route_stage()
        active_points = self.route_points
        output['delivery_route'] = dict(lap=self.route_lap, completed_laps=self.completed_laps,
            stage=stage, stage_name=stage_name, route_source=self.route_source,
            index=self.route_index, total_points=self._section_length(), closing=self.route_closing,
            delivery_points=len(self.route_points), loaded_points=len(self.route_points),
            itinerary_index=self.itinerary_index, itinerary_steps=len(self.itinerary),
            returning_home=self.route_returning_home,
            home_xy=list(self.route_home) if self.route_home is not None else None,
            current_point=deepcopy(self._step_point()) if self.route_index is not None else None,
            released_slots=list(self.route_released_slots),
            released_points=sorted(self.route_released_points),
            visual_alignment=self._route_alignment_status(),
            landing_alignment=dict(self.landing_alignment, enabled=self.landing_alignment_enabled,
                                   result=deepcopy(self.landing_alignment_result)),
            backend=self.navigation_backend)
        if self.fixed_navigation and self.state == 'APPROACH' and self.phase in ('FIXED_TRAJECTORY', 'FIXED_ARRIVAL'):
            progress = self._fixed_progress or {}
            output['delivery_route'].update(fixed_leg=self.fixed_leg_sequence,
                elapsed_s=progress.get('elapsed_s', 0.),
                duration_s=self.fixed_trajectory.duration if self.fixed_trajectory else None)
            if self.phase == 'FIXED_TRAJECTORY':
                output['control'] = output['reference_source'] = 'FIXED_TRAJECTORY'
            output['position_xyz'] = list(self.command_position)
            output['native_command'].update(position=self.command_position, velocity=self.velocity,
                acceleration=tuple(progress.get('acceleration', (0., 0., 0.))), yaw=self._fixed_yaw)
            output['native']['target_xyz'] = list(self.command_position)
            output['telemetry'].update(navigation_backend='fixed_min_snap',
                navigation_notice='固定轨迹：实时绕障、障碍膨胀和EGO虚拟围墙不生效',
                phase_name=('固定航段 %d：按参考时间飞行' % self.fixed_leg_sequence
                            if self.phase == 'FIXED_TRAJECTORY' else
                            '固定航段 %d：参考已到降落点，水平 %.2f m / %.1f s 判断到点，%g s 后原地降落' % (
                                self.fixed_leg_sequence, self.home_arrival_xy_m, self.navigation_arrival_dwell_s,
                                self.visual_alignment['motion_timeout_margin_s'])
                            if self._fixed_leg_lands else
                            '固定航段 %d：参考已到终点，按 %.2f m / %.1f s 判断到点' % (
                                self.fixed_leg_sequence, self.navigation_arrival_radius_m,
                                self.navigation_arrival_dwell_s)))
        labels = {'NATIVE_FIRST_FRAME': '第一次开环对准：等待绿色圆心识别',
                  'NATIVE_FIRST_MOVE': '第一次开环对准：飞向固定绿色圆心',
                  'NATIVE_SECOND_FRAME': '开环对准：等待红点识别',
                  'NATIVE_SECOND_MOVE': '开环对准：飞向固定靶心',
                  'NATIVE_INTERMEDIATE_DESCEND': '到点下降：前往红点识别高度',
                  'NATIVE_DESCEND': '投放：下降至投放高度',
                  'NATIVE_OFFSET_MOVE': '投放：执行一次投放口偏移',
                  'WAIT_ACK': '等待舵机投放指令回执', 'SETTLE': '释放后等待动作完成',
                  'NATIVE_RECOVER': '投放后回升',
                  'LANDED': '巡航%d圈及投放完成，已落地上锁' % self.route_laps,
                  'LAND_FIRST_FRAME': 'H 标第一次对准：等待识别',
                  'LAND_FIRST_MOVE': 'H 标第一次对准：飞向固定 H 中心',
                  'LAND_INTERMEDIATE_DESCEND': 'H 标：下降至第二次识别高度',
                  'LAND_SECOND_FRAME': 'H 标第二次对准：等待识别',
                  'LAND_SECOND_MOVE': 'H 标第二次对准：飞向固定 H 中心',
                  'LAND_DESCEND': 'H 标：下降至近地高度，随后交 AUTO.LAND'}
        if self.phase in labels:
            output['telemetry']['phase_name'] = labels[self.phase]
        elif self.state == 'APPROACH' and self.route_source in LOOP_SECTION_NAMES:
            number = '%d/%d' % (self.route_index+1, self._section_length())
            if self.route_source == 'cruise_loop':
                label = ('第%d/%d圈闭环：返回巡航首点' % (self.route_lap, self.route_laps) if self.route_closing
                         else '第%d/%d圈：航点%s' % (self.route_lap, self.route_laps, number))
            else:
                label = '%s %s' % (LOOP_SECTION_NAMES[self.route_source], number)
            output['telemetry']['phase_name'] = stage_name + '：' + label + '；' + output['telemetry']['phase_name']
        elif self.state == 'APPROACH':
            label = '返回起降点' if self.route_returning_home else (
                '第%d圈闭环：返回绕圈首点' % self.route_lap if self.route_closing else
                '第%d圈：%s航点%d/%d' % (self.route_lap,
                    '共用' if self.route_source == 'shared' else '投放路线', self.route_index+1, len(active_points)))
            output['telemetry']['phase_name'] = stage_name + '：' + label + '；' + output['telemetry']['phase_name']
        return output
