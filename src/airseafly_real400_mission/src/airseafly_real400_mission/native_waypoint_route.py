"""Recorded XY routes using native takeoff and selectable navigation references.

The route has one ground reference and two selectable height/map profiles.
Optional entry points are flown once, in order, before the loop. EGO then
visits each loop point once; fixed MINCO references traverse the configured
laps and hold the first loop point. Neither path enters visual or payload stages.
NativeRuntime remains responsible for continuously streaming references.
"""
from copy import deepcopy

from .flight_core import FlightCore
from .native_segment import NativeSegmentMixin
from .native_flight import NativeFlightMixin
from .waypoint_route import validate_route
from .route_profiles import resolve_profiles, point_profiles, point_velocities, entry_points, entry_profiles


class NativeTakeoffEngine(NativeFlightMixin, NativeSegmentMixin, FlightCore):
    """Native-only takeoff and hold, on the ordinary mission manager endpoint."""
    def __init__(self, config=None):
        config = deepcopy(config or {})
        if config.get('flight', {}).get('controller') != 'px4_native':
            raise ValueError('基础起飞和普通航点任务必须使用 flight.controller=px4_native')
        geometry = config.setdefault('motion', {})
        geometry.setdefault('native_profile', 'waypoint')
        super().__init__(config)

    def start(self, mode, *args, **kwargs):
        if mode not in ('basic_takeoff', 'takeoff'):
            raise ValueError('基础起飞入口只接受 basic_takeoff')
        return super().start('takeoff', *args, **kwargs)



    @property
    def next_slot(self):
        return None


class WaypointRouteEngine(NativeTakeoffEngine):
    def __init__(self, config=None):
        config = deepcopy(config)
        self.profiles = resolve_profiles(config)
        config['flight']['cruise_height_m'] = self.profiles['cruise']['height_m']
        super().__init__(config)
        self.route_document = validate_route(self.config['waypoint_route'], require_points=True)
        self.route_points = [tuple(xy) for xy in self.route_document['points']]
        self.route_profile_names = point_profiles(self.route_document)
        self.entry_points = [tuple(xy) for xy in entry_points(self.route_document)]
        self.entry_profile_names = entry_profiles(self.route_document)
        self.route_entry_index = None  # Set while flying toward an entry point.
        self.route_index = None
        self.route_completed_points = 0
        self.navigation_backend = self.config.get('navigation', {}).get('backend', 'ego')
        if self.navigation_backend not in ('ego', 'fixed_min_snap'):
            raise ValueError('navigation.backend 必须为 ego 或 fixed_min_snap')
        self.route_laps = self.config.get('navigation', {}).get('laps', 2)
        self.fixed_trajectory = None
        self._fixed_summary = None
        self._fixed_start = None
        self._fixed_progress = None
        self._fixed_event_index = 0
        self._fixed_passed_indices = set()
        self.fixed_completed_laps = 0
        self.fixed_lap = 0

    def start(self, mode, position, yaw, now, target=None, origin=None, slot=None,
              stop_after=None, scan_xy=None):
        if mode != 'waypoint_route':
            raise ValueError('普通航点路线只使用 waypoint_route 任务模式')
        # Solve before queuing takeoff, so invalid input or a missing math
        # library cannot leave a partially started route behind.
        prepared = self._build_fixed_trajectory(tuple(position)) if self.fixed_navigation else None
        # Keep the established ground registration and native takeoff lifecycle.
        super().start('takeoff', position, yaw, now, origin=origin)
        self.mode = 'waypoint_route'
        if prepared is not None:
            self._set_fixed_trajectory(prepared)
        return self.snapshot()

    @property
    def fixed_navigation(self):
        return self.navigation_backend == 'fixed_min_snap'

    def _build_fixed_trajectory(self, ground):
        from .fixed_trajectory import build_fixed_trajectory
        points = [(*xy, ground[2] + self.profiles[profile]['height_m'])
                  for xy, profile in zip(self.route_points, self.route_profile_names)]
        start = (*ground[:2], ground[2] + self.profiles['cruise']['height_m'])
        limits = self.config['planning']
        entries = [(*xy, ground[2] + self.profiles[profile]['height_m'])
                   for xy, profile in zip(self.entry_points, self.entry_profile_names)]
        trajectory = build_fixed_trajectory(start, points, self.route_laps,
                                      limits['max_velocity_mps'], limits['max_acceleration_mps2'],
                                      point_velocities(self.config, self.route_profile_names),
                                      entries, point_velocities(self.config, self.entry_profile_names) if entries else None)
        from .waypoint_timing import apply_waypoint_timing
        return apply_waypoint_timing(trajectory, self.config, self.route_profile_names, self.entry_profile_names)

    def _current_profile(self):
        if self.route_entry_index is not None:
            return self.entry_profile_names[self.route_entry_index]
        return self.route_profile_names[self.route_index] if self.route_index is not None else 'cruise'

    def _current_xy(self):
        if self.route_entry_index is not None:
            return self.entry_points[self.route_entry_index]
        return self.route_points[self.route_index] if self.route_index is not None else None

    def _set_fixed_trajectory(self, trajectory):
        self.fixed_trajectory = trajectory
        initial = trajectory.sample(0.)
        self._fixed_start = tuple(initial['position'])
        self._fixed_progress = initial
        # The preview keeps the full path; recurring status only needs compact
        # metadata, not hundreds of samples on every manager publication.
        self._fixed_summary = {key: value for key, value in trajectory.summary().items()
                               if key != 'sample_path'}

    def begin_native_flight(self):
        super().begin_native_flight()
        if self.fixed_navigation and self.state == 'TAKEOFF':
            # Runtime can register a corrected ground frame during prepare().
            # This still runs before its first takeoff effect is dispatched.
            start = (*self.origin_xyz[:2], self.cruise_z)
            if self.fixed_trajectory is None or self._fixed_start != start:
                self._set_fixed_trajectory(self._build_fixed_trajectory(self.origin_xyz))

    @property
    def next_slot(self):
        return None

    def _after_takeoff(self, preserve_reference=False, preserve_yaw=False):
        # FlightCore invokes this hook once the climb reference is complete.
        if self.fixed_navigation:
            if self.entry_points:
                self.route_entry_index, self.route_index = 0, None
            else:
                self.route_index = 0
            self.route_target_z = self.origin_xyz[2] + self.profiles[self._current_profile()]['height_m']
            self.command_position = self._fixed_start
            self.command_yaw = self.takeoff_yaw
            self.velocity = (0., 0., 0.)
            self.native_active = False
            self.native_motion = self.native_pending = None
            self.goal_purpose = 'waypoint_route'
            self._state('APPROACH', 'FIXED_TRAJECTORY', 'NONE')
            self._effect('waypoint_route_fixed_started', backend=self.navigation_backend,
                         laps=self.route_laps, duration_s=self.fixed_trajectory.duration,
                         timing_mode=self.config.get('navigation', {}).get('timing', 'original'),
                         local_retiming=(self._fixed_summary or {}).get('local_retiming'),
                         position_xyz=list(self.command_position))
            return
        if self.entry_points:
            self._begin_entry_point(0)
        else:
            self._begin_route_point(0)
        # Preserve the takeoff XY and target Z while EGO prepares the first leg.
        self._navigation_hold = (*self.origin_xyz[:2], self.cruise_z)

    def _begin_entry_point(self, index):
        """EGO toward one entry point; the same worker and map as the loop."""
        previous_z = self.command_position[2]
        self.route_entry_index, self.route_index = index, None
        profile = self.profiles[self.entry_profile_names[index]]
        self.route_target_z = self.origin_xyz[2] + profile['height_m']
        self.native_active = False
        self.native_motion = None
        self.native_pending = None
        self._begin_navigation((*self.entry_points[index], self.route_target_z), 'waypoint_route')
        self._navigation_hold = (*self.position[:2], previous_z)
        self._effect('waypoint_route_entry_point', index=index, total_entry_points=len(self.entry_points),
                     position_xyz=list(self.command_position), profile=self.entry_profile_names[index],
                     **profile)

    def _begin_route_point(self, index):
        previous_z = self.command_position[2]
        self.route_entry_index = None
        self.route_index = index
        profile = self.profiles[self.route_profile_names[index]]
        self.route_target_z = self.origin_xyz[2] + profile['height_m']
        self.native_active = False
        self.native_motion = None
        self.native_pending = None
        # Keep one EGO worker and its map for the route. Each goal has its own
        # identity; NativeRuntime discards the preceding goal's commands.
        self._begin_navigation((*self.route_points[index], self.route_target_z), 'waypoint_route')
        # Height is always relative to the one captured ground origin, including
        # the stationary reference used during the existing EGO handoff.
        self._navigation_hold = (*self.position[:2], previous_z)
        self._effect('waypoint_route_point', index=index, total_points=len(self.route_points),
                     position_xyz=list(self.command_position), profile=self.route_profile_names[index],
                     **profile)

    def _tick_navigation(self, now):
        if self.fixed_navigation:
            # Runtime is the sole clock and sampler for fixed references. The
            # manager consumes its progress without an arrival/dwell gate.
            return
        before = len(self._effects)
        super()._tick_navigation(now)
        for effect in self._effects[before:]:
            if effect['type'] == 'goal':
                effect['map_inflation_m'] = self.profiles[self._current_profile()]['inflation_m']
                effect['waypoint_profile'] = self._current_profile()

    def on_fixed_progress(self, progress):
        """Consume the last runtime sample; never advance from manager time."""
        if (not self.fixed_navigation or self.fixed_trajectory is None
                or self.state != 'APPROACH' or self.phase != 'FIXED_TRAJECTORY'
                or progress is None):
            return False
        elapsed = float(progress['elapsed_s'])
        if elapsed < self._fixed_progress['elapsed_s']:
            return False
        self._fixed_progress = deepcopy(progress)
        self.command_position = tuple(progress['position'])
        self.velocity = tuple(progress['velocity'])
        self.command_yaw = self.takeoff_yaw
        measured = progress.get('actual_position_xyz')
        actual_position = list(self.position if measured is None else measured)
        events = self.fixed_trajectory.events
        while self._fixed_event_index < len(events) and events[self._fixed_event_index]['elapsed_s'] <= elapsed:
            event = events[self._fixed_event_index]
            self._fixed_event_index += 1
            if 'entry_index' in event:
                self.route_entry_index, self.route_index = event['entry_index'], None
                self.route_target_z = self.origin_xyz[2] + self.profiles[self._current_profile()]['height_m']
                self._effect('waypoint_route_entry_passed', index=self.route_entry_index,
                             total_entry_points=len(self.entry_points), elapsed_s=event['elapsed_s'],
                             position_xyz=[*self._current_xy(), self.route_target_z],
                             actual_position_xyz=actual_position)
                # Toward the next entry point, or the first loop point.
                if self.route_entry_index+1 < len(self.entry_points):
                    self.route_entry_index += 1
                else:
                    self.route_entry_index, self.route_index = None, 0
                continue
            self.route_entry_index = None
            self.route_index = event['waypoint_index']
            self.fixed_lap = event['lap']
            self.fixed_completed_laps = event['completed_laps']
            self._fixed_passed_indices.add(self.route_index)
            self.route_completed_points = len(self._fixed_passed_indices)
            self.route_target_z = self.origin_xyz[2] + self.profiles[self.route_profile_names[self.route_index]]['height_m']
            self._effect('waypoint_route_reference_passed', index=self.route_index,
                         total_points=len(self.route_points), elapsed_s=event['elapsed_s'],
                         lap=self.fixed_lap, completed_laps=self.fixed_completed_laps,
                         position_xyz=[*self.route_points[self.route_index], self.route_target_z],
                         actual_position_xyz=actual_position)
        if progress['completed']:
            self.command_position = tuple(self.fixed_trajectory.end)
            self.velocity = (0., 0., 0.)
            self._state('COMPLETE', 'FIXED_TRAJECTORY_COMPLETE', 'NONE')
            self.done = True
            description = ('参考两圈完成' if self.fixed_completed_laps == 2 else
                           '参考%d圈完成' % self.fixed_completed_laps if self.fixed_completed_laps else
                           '单点参考完成')
            self.reason = description + '，持续保持首点；实测误差仅显示'
            self._effect('waypoint_route_fixed_complete', total_points=len(self.route_points),
                         completed_laps=self.fixed_completed_laps, elapsed_s=elapsed,
                         position_xyz=list(self.command_position),
                         actual_position_xyz=actual_position)
        return True

    def hold(self, reason):
        super().hold(reason)
        # Revocation is permanent for this task; progress cannot restart it.
        self.fixed_trajectory = None

    def _on_custom_arrival(self):
        if self.fixed_navigation:
            return False
        if self.goal_purpose != 'waypoint_route':
            return super()._on_custom_arrival()
        if self.route_entry_index is not None:
            self._effect('waypoint_route_entry_arrived', index=self.route_entry_index,
                         total_entry_points=len(self.entry_points), position_xyz=list(self.command_position))
            if self.route_entry_index+1 < len(self.entry_points):
                self._begin_entry_point(self.route_entry_index+1)
            else:
                self._begin_route_point(0)
            return True
        self.route_completed_points = self.route_index + 1
        self._effect('waypoint_route_point_arrived', index=self.route_index,
                     total_points=len(self.route_points), position_xyz=list(self.command_position))
        if self.route_completed_points < len(self.route_points):
            self._begin_route_point(self.route_completed_points)
        else:
            # Keep the recorded endpoint, even when arrival was declared within
            # the existing position tolerance. Actual error is diagnostic only.
            self.command_position = (*self.route_points[-1], self.route_target_z)
            self.velocity = (0., 0., 0.)
            self._state('COMPLETE', 'WAYPOINT_ROUTE_COMPLETE', 'NONE')
            self.done = True
            self.reason = '全部航点已完成，持续保持末个航点；可使用保持或当前位置降落指令'
            self._effect('waypoint_route_complete', total_points=len(self.route_points),
                         position_xyz=list(self.command_position))
        return True

    def snapshot(self):
        output = super().snapshot()
        output['implementation'] = 'waypoint_route_px4_native'
        output['payload_total'] = 0
        output['waypoint_route'] = dict(index=self.route_index, total_points=len(self.route_points),
            completed_points=self.route_completed_points,
            current_xy=list(self._current_xy()) if self._current_xy() is not None else None,
            target_z=getattr(self, 'route_target_z', self.cruise_z),
            profile=self._current_profile(),
            entry_index=self.route_entry_index, total_entry_points=len(self.entry_points),
            section='entry' if self.route_entry_index is not None else 'loop',
            profiles=deepcopy(self.profiles), height_m=self.profiles[self._current_profile()]['height_m'],
            origin_xyz=list(self.origin_xyz) if self.origin_xyz is not None else None,
            finish_action='hold_first' if self.fixed_navigation else 'hold_last',
            backend=self.navigation_backend)
        if self.fixed_navigation:
            progress = self._fixed_progress or {}
            output['waypoint_route'].update(laps=self.route_laps, lap=self.fixed_lap,
                timing_mode=self.config.get('navigation', {}).get('timing', 'original'),
                completed_laps=self.fixed_completed_laps, reference_passed_points=self._fixed_event_index,
                elapsed_s=progress.get('elapsed_s', 0.),
                duration_s=(self._fixed_summary or {}).get('duration_s'),
                trajectory=deepcopy(self._fixed_summary),
                obstacle_avoidance=False, obstacle_inflation=False, ego_virtual_walls=False)
            output['telemetry'].update(arrival_metric='reference_complete',
                arrival=dict(elapsed_s=0., required_s=0., metric='reference_complete'),
                navigation_backend='fixed_min_snap',
                navigation_notice='固定轨迹：实时绕障、障碍膨胀和EGO虚拟围墙不生效')
            if self.state == 'APPROACH' and self.phase == 'FIXED_TRAJECTORY':
                output['control'] = output['reference_source'] = 'FIXED_TRAJECTORY'
                output['position_xyz'] = list(self.command_position)
                output['native_command'].update(position=self.command_position, velocity=self.velocity,
                    acceleration=tuple(progress.get('acceleration', (0., 0., 0.))), yaw=self.takeoff_yaw)
                output['native']['target_xyz'] = list(self.command_position)
                output['native']['error_xyz'] = [desired-actual for desired, actual in
                                                zip(self.command_position, self.position)]
                output['telemetry']['phase_name'] = ('固定轨迹；进场点 %d/%d' % (
                    self.route_entry_index+1, len(self.entry_points)) if self.route_entry_index is not None else
                    '固定轨迹；参考已完成 %d/%d 圈' % (self.fixed_completed_laps, self.route_laps))
            elif self.phase == 'FIXED_TRAJECTORY_COMPLETE':
                output['telemetry']['phase_name'] = self.reason
            output['waypoint_route']['reference_position_xyz'] = list(self.command_position)
            output['waypoint_route']['actual_error_xyz'] = [desired-actual for desired, actual in
                                                           zip(self.command_position, self.position)]
        elif self.state == 'APPROACH' and self.route_entry_index is not None:
            output['telemetry']['phase_name'] = '进场点 %d/%d；%s' % (
                self.route_entry_index+1, len(self.entry_points), output['telemetry']['phase_name'])
        elif self.state == 'APPROACH':
            output['telemetry']['phase_name'] = '普通航点 %d/%d；%s' % (
                self.route_index+1, len(self.route_points), output['telemetry']['phase_name'])
        elif self.phase == 'WAYPOINT_ROUTE_COMPLETE':
            output['telemetry']['phase_name'] = '航点已全部完成，持续保持末点'
        return output
