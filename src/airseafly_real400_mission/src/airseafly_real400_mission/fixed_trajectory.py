"""Precomputed fixed routes: MINCO XY, monotone Z, analytic Python sampling.

The native library is used only while preparing a route. Runtime sampling has
no native calls, online planning, clipping, arrival gates, or dwell conditions.
"""
from bisect import bisect_right
import ctypes
import ctypes.util
import math
import os
from pathlib import Path

import numpy as np


UPSTREAM_REVISION = 'e0444f6d47b84f972ced91746b05feb36ce1fd4f'
SOURCE = 'gcopter_minco_s4nu_fixed'


def _xyz(value):
    result = tuple(float(component) for component in value)
    if len(result) != 3 or not all(math.isfinite(component) for component in result):
        raise ValueError('fixed trajectory coordinates must be finite XYZ triples')
    return result


def _library_candidates():
    override = os.environ.get('AIRSEAFLY_MIN_SNAP_LIBRARY')
    if override:
        yield override
        return
    names = ('libairseafly_fixed_min_snap.so', 'libairseafly_fixed_min_snap.dylib')
    # This also resolves source files symlinked into the catkin workspace.
    workspace = Path(__file__).resolve().parents[4]
    paths = [workspace/'build_ws'/'devel'/'lib', workspace/'devel'/'lib',
             workspace/'install'/'lib', workspace/'runtime'/'fixed_trajectory']
    paths.extend(Path(prefix)/'lib' for prefix in os.environ.get('CMAKE_PREFIX_PATH', '').split(os.pathsep)
                 if prefix)
    paths.extend(Path(prefix) for prefix in os.environ.get('LD_LIBRARY_PATH', '').split(os.pathsep)
                 if prefix)
    for path in paths:
        for name in names:
            yield str(path/name)
    located = ctypes.util.find_library('airseafly_fixed_min_snap')
    if located:
        yield located


def _load_library():
    failures = []
    for path in _library_candidates():
        try:
            library = ctypes.CDLL(path)
            library.airseafly_min_snap_abi_version.restype = ctypes.c_int
            if library.airseafly_min_snap_abi_version() != 1:
                raise RuntimeError('unsupported MINCO C ABI version')
            double_pointer = ctypes.POINTER(ctypes.c_double)
            library.airseafly_min_snap_solve.argtypes = [
                ctypes.c_int, double_pointer, double_pointer, ctypes.c_double,
                ctypes.c_double, double_pointer, double_pointer,
                ctypes.POINTER(ctypes.c_char), ctypes.c_int]
            library.airseafly_min_snap_solve.restype = ctypes.c_int
            return library
        except (OSError, AttributeError, RuntimeError) as exc:
            if Path(path).exists() or os.environ.get('AIRSEAFLY_MIN_SNAP_LIBRARY'):
                failures.append(str(exc))
    detail = '; '.join(failures)
    raise RuntimeError('固定轨迹 MINCO 动态库未就绪；起飞前运行 scripts/build_fixed_trajectory.sh '
                       '或 scripts/build.sh 构建，必要时设置 AIRSEAFLY_MIN_SNAP_LIBRARY。' + detail)


def _warp_state(piece, x):
    """Base time and its first three derivatives, x warped seconds into a piece.

    A ramp blends the rate r0 -> r1 with the quintic smoothstep h(u), so the
    rate and its first two derivatives are continuous with neighboring pieces.
    """
    if piece['kind'] == 'line':
        return piece['tau0']+piece['r0']*x, piece['r0'], 0., 0.
    length = piece['s1']-piece['s0']
    u, change = x/length, piece['r1']-piece['r0']
    smooth = u**3*(10.+u*(-15.+6.*u))
    integral = u**4*(2.5+u*(-3.+u))
    return (piece['tau0']+piece['r0']*x+change*length*integral, piece['r0']+change*smooth,
            change*30.*u*u*(1.-u)**2/length, change*60.*u*(1.-u)*(1.-2.*u)/(length*length))


class FixedTrajectory:
    """Immutable-by-convention reference, with normalized ascending coefficients.

    warp optionally re-times the same path (gate speed). Without it, sampling
    is exactly the original single-time-scale reference.
    """
    def __init__(self, nodes, durations, coefficients, events, metadata, warp=None, retiming=None):
        self.nodes = tuple(_xyz(point) for point in nodes)
        self.durations = tuple(float(value) for value in durations)
        self.coefficients = np.asarray(coefficients, dtype=float).reshape((-1, 3, 8))
        self.coefficients.setflags(write=False)
        self._ends = tuple(np.cumsum(self.durations).tolist())
        self._starts = (0.,) + self._ends[:-1] if self.durations else ()
        self.base_duration = self._ends[-1] if self._ends else 0.
        self.warp = None
        if warp is not None:
            self.warp = dict(scale=float(warp['scale']), pieces=[dict(p) for p in warp['pieces']])
            self._warp_starts = [piece['s0'] for piece in self.warp['pieces']]
        self.duration = (self.warp['scale']*self.warp['pieces'][-1]['s1'] if self.warp
                         else self.base_duration)
        self.retiming = None
        if retiming is not None:
            from .local_retiming import LocalTimeMap
            self.retiming = LocalTimeMap(retiming)
            self.duration = self.retiming.duration
        self.end = self.nodes[-1]
        self.events = [dict(event) for event in events]
        self._event_times = [event['elapsed_s'] for event in self.events]
        self._metadata = dict(metadata)
        # Derivatives are prepared once, so the 50 Hz path only does Horner
        # evaluation and powers of a duration. No C++ entrypoint is involved.
        derivatives = [self.coefficients]
        for order in range(1, 4):
            prior = derivatives[-1]
            derivatives.append(prior[:, :, 1:]*np.arange(1, prior.shape[2]))
        self._derivatives = tuple(derivatives)

    def _sample_segment(self, index, unit_time, order):
        result = np.zeros(3)
        for coefficient in self._derivatives[order][index].T[::-1]:
            result = result*unit_time+coefficient
        if order:
            result /= self.durations[index]**order
        return tuple(float(value) for value in result)

    def sample(self, elapsed_s):
        elapsed_s = float(elapsed_s)
        if not math.isfinite(elapsed_s):
            raise ValueError('trajectory elapsed time must be finite')
        elapsed = min(self.duration, max(0., elapsed_s))
        complete = elapsed >= self.duration
        event_index = bisect_right(self._event_times, elapsed)-1
        event = self.events[event_index] if event_index >= 0 else {}
        if complete:
            position, velocity, acceleration, jerk = self.end, (0.,)*3, (0.,)*3, (0.,)*3
            segment = max(0, len(self.durations)-1)
        elif self.retiming is not None:
            tau, rate, rate1, rate2 = self.retiming.state(elapsed)
            segment, position, velocity, acceleration, jerk = self._sample_time_map(tau, rate, rate1, rate2)
        elif self.warp is None:
            segment = bisect_right(self._ends, elapsed)
            unit_time = (elapsed-self._starts[segment])/self.durations[segment]
            position, velocity, acceleration, jerk = (
                self._sample_segment(segment, unit_time, order) for order in range(4))
        else:
            segment, position, velocity, acceleration, jerk = self._sample_warped(elapsed)
        return dict(position=position, velocity=velocity, acceleration=acceleration,
                    jerk=jerk, elapsed_s=elapsed, completed=complete, segment_index=segment,
                    completed_laps=event.get('completed_laps', 0), lap=event.get('lap', 0))

    def _locate(self, tau):
        segment = min(bisect_right(self._ends, tau), len(self.durations)-1)
        unit = (tau-self._starts[segment])/self.durations[segment]
        return segment, min(1., max(0., unit))

    def _sample_warped(self, elapsed):
        # Chain rule through base time tau(s) and the global scale t = scale*s.
        scale = self.warp['scale']
        s = elapsed/scale
        piece = self.warp['pieces'][max(0, bisect_right(self._warp_starts, s)-1)]
        tau, rate, rate1, rate2 = _warp_state(piece, s-piece['s0'])
        segment, unit = self._locate(min(self.base_duration, max(0., tau)))
        p, d1, d2, d3 = (np.array(self._sample_segment(segment, unit, order)) for order in range(4))
        values = (p, d1*rate/scale, (d2*rate*rate+d1*rate1)/scale**2,
                  (d3*rate**3+3.*d2*rate*rate1+d1*rate2)/scale**3)
        return (segment,)+tuple(tuple(float(v) for v in value) for value in values)

    def _sample_time_map(self, tau, rate, rate1, rate2):
        segment, unit = self._locate(min(self.base_duration, max(0., tau)))
        p, d1, d2, d3 = (np.array(self._sample_segment(segment, unit, order)) for order in range(4))
        values = (p, d1*rate, d2*rate*rate+d1*rate1,
                  d3*rate**3+3.*d2*rate*rate1+d1*rate2)
        return (segment,)+tuple(tuple(float(v) for v in value) for value in values)

    def range_maximum(self, tau0, tau1, order):
        """Exact max |d^order p / d tau^order| over base time [tau0, tau1]."""
        maximum = 0.
        for index, (start, duration) in enumerate(zip(self._starts, self.durations)):
            if start+duration < tau0 or start > tau1:
                continue
            u0 = min(1., max(0., (tau0-start)/duration))
            u1 = min(1., max(0., (tau1-start)/duration))
            rows = self._derivatives[order][index]
            squared = sum(np.polynomial.polynomial.polymul(row, row) for row in rows)
            critical = np.polynomial.polynomial.polyder(squared)
            roots = np.polynomial.polynomial.polyroots(critical) if np.any(critical) else []
            candidates = [u0, u1]+[r.real for r in roots if abs(r.imag) < 1e-7 and u0 < r.real < u1]
            for unit in candidates:
                maximum = max(maximum, float(np.linalg.norm(self._sample_segment(index, unit, order))))
        return maximum

    def summary(self):
        return dict(self._metadata, duration_s=self.duration, base_duration_s=self.base_duration,
                    durations_s=list(self.durations), segment_count=len(self.durations),
                    end_xyz=list(self.end), events=[dict(event) for event in self.events])

    def to_dict(self):
        value = dict(nodes=[list(point) for point in self.nodes], durations=list(self.durations),
                     coefficients=self.coefficients.tolist(), events=[dict(e) for e in self.events],
                     metadata=dict(self._metadata))
        if self.warp is not None:
            value['warp'] = dict(scale=self.warp['scale'], pieces=[dict(p) for p in self.warp['pieces']])
        if self.retiming is not None:
            value['retiming'] = dict(self.retiming.value)
        return value

    @classmethod
    def from_dict(cls, value):
        return cls(value['nodes'], value['durations'], value['coefficients'],
                   value['events'], value['metadata'], value.get('warp'), value.get('retiming'))


def _time_warp(trajectory, node_limits, max_acceleration):
    """Re-time the same path so the speed through limited nodes meets their cap.

    Stretching MINCO segment times does not do this: a long segment makes snap
    cheap and the curve bulges faster. Instead each run of consecutive limited
    nodes becomes one window held at a constant rate k = cap / exact peak speed,
    entered and left by quintic rate ramps outside the window. Windows whose
    ramps would overlap merge. One global scale then meets the acceleration
    bound. Returns None when no cap binds. Preparation time only.
    """
    total = trajectory.base_duration
    times = list(trajectory._starts)+[total]
    windows, index = [], 0
    while index < len(node_limits):
        if node_limits[index] is None:
            index += 1
            continue
        end = index
        while end+1 < len(node_limits) and node_limits[end+1] is not None:
            end += 1
        windows.append(dict(t0=times[index], t1=times[end], cap=min(node_limits[index:end+1])))
        index = end+1

    def settle(window):
        peak = trajectory.range_maximum(window['t0'], window['t1'], 1)
        rate = min(1., window['cap']/peak/(1.+1e-9)) if peak > 0. else 1.
        window.update(peak=peak, rate=rate)
        for side in ('down', 'up'):
            length = 0.
            # Ramp length: |p''| r^2 + |p'| |r'| stays near the limit, |r'| <= 1.875 |dr| / L.
            for _ in range(8):
                span = length*(1.+rate)/2.
                a, b = ((window['t0']-span, window['t0']) if side == 'down'
                        else (window['t1'], window['t1']+span))
                a, b = max(0., a), min(total, b)
                room = max(max_acceleration-trajectory.range_maximum(a, b, 2), .25*max_acceleration)
                updated = 1.875*trajectory.range_maximum(a, b, 1)*(1.-rate)/room
                done = abs(updated-length) <= 1e-6*max(1., updated)
                length = updated
                if done:
                    break
            # At the route ends a ramp is shortened; the global scale covers it.
            room = window['t0'] if side == 'down' else total-window['t1']
            window[side] = min(length, 2.*room/(1.+rate))
        return window

    windows = [w for w in (settle(w) for w in windows) if w['rate'] < 1.]
    merged = True
    while merged and len(windows) > 1:
        merged = False
        for i in range(len(windows)-1):
            a, b = windows[i], windows[i+1]
            if a['t1']+a['up']*(1.+a['rate'])/2. > b['t0']-b['down']*(1.+b['rate'])/2.:
                windows[i:i+2] = [settle(dict(t0=a['t0'], t1=b['t1'], cap=min(a['cap'], b['cap'])))]
                merged = True
                break
    if not windows:
        return None
    pieces, cursor = [], dict(s=0., tau=0.)

    def add(kind, r0, r1, length):
        if length > 0.:
            pieces.append(dict(kind=kind, s0=cursor['s'], s1=cursor['s']+length,
                               tau0=cursor['tau'], r0=r0, r1=r1))
            cursor['s'] += length
            cursor['tau'] += length*(r0+r1)/2.

    for window in windows:
        rate = window['rate']
        add('line', 1., 1., window['t0']-window['down']*(1.+rate)/2.-cursor['tau'])
        add('ramp', 1., rate, window['down'])
        add('line', rate, rate, (window['t1']-cursor['tau'])/rate)
        add('ramp', rate, 1., window['up'])
    add('line', 1., 1., total-cursor['tau'])
    bound = 0.
    for piece in pieces:
        a = piece['tau0']
        b = a+(piece['s1']-piece['s0'])*(piece['r0']+piece['r1'])/2.
        top = max(piece['r0'], piece['r1'])
        value = trajectory.range_maximum(a, b, 2)*top*top
        if piece['kind'] == 'ramp':
            value += (trajectory.range_maximum(a, b, 1)*1.875*abs(piece['r1']-piece['r0']) /
                      (piece['s1']-piece['s0']))
        bound = max(bound, value)
    scale = max(1., math.sqrt(bound/max_acceleration))*(1.+1e-10)
    return dict(scale=scale, pieces=pieces), windows, bound


def _warped_time(warp, tau):
    """Elapsed seconds at base time tau; the inverse of the monotone warp."""
    for piece in warp['pieces']:
        length = piece['s1']-piece['s0']
        if tau <= piece['tau0']+length*(piece['r0']+piece['r1'])/2. or piece is warp['pieces'][-1]:
            if piece['kind'] == 'line':
                x = (tau-piece['tau0'])/piece['r0']
            else:
                low, high = 0., length
                for _ in range(80):
                    middle = (low+high)/2.
                    if _warp_state(piece, middle)[0] < tau:
                        low = middle
                    else:
                        high = middle
                x = (low+high)/2.
            return warp['scale']*(piece['s0']+min(length, max(0., x)))
    return 0.


def build_fixed_trajectory(start_xyz, points_xyz, laps, max_velocity, max_acceleration,
                           point_velocities=None, entry_xyz=None, entry_velocities=None):
    """Solve start -> P1 -> (P2..Pn -> P1)*laps, without internal stops.

    waypoint_index is the original zero-based recorded point index. Duplicate
    points retain their events at the same timestamp. Initial entry is lap=0;
    returning to P1 increments completed_laps. A one-location route has zero
    effective laps and simply approaches that point and holds.

    point_velocities optionally gives recorded points a lower speed cap (None
    keeps the route limit). The path is unchanged; only its timing slows
    through those points and between consecutive ones. A cap at or above the
    achieved speed there has no effect, and all None returns the original.

    entry_xyz optionally gives points flown once, in order, between the start
    and P1 (lap 0). Their events carry entry_index instead of waypoint_index.
    """
    start = _xyz(start_xyz)
    points = [_xyz(point) for point in points_xyz]
    if not points:
        raise ValueError('fixed trajectory requires at least one recorded waypoint')
    if isinstance(laps, bool) or int(laps) != laps or int(laps) < 1:
        raise ValueError('navigation.laps must be a positive integer')
    laps = int(laps)
    max_velocity, max_acceleration = _limits(max_velocity, max_acceleration)
    point_velocities = _caps(point_velocities, len(points), 'point_velocities must match the recorded waypoints')
    entries = [_xyz(point) for point in (entry_xyz or ())]
    entry_velocities = _caps(entry_velocities, len(entries), 'entry_velocities must match the entry points')
    effective_laps = laps if len(set(points)) > 1 else 0
    nodes, events = [start], []
    for index, point in enumerate(entries):
        if point != nodes[-1]:
            nodes.append(point)
        events.append(dict(node_index=len(nodes)-1, entry_index=index, lap=0, completed_laps=0))

    def visit(index, lap, completed_laps):
        if points[index] != nodes[-1]:
            nodes.append(points[index])
        events.append(dict(node_index=len(nodes)-1, waypoint_index=index,
                           lap=lap, completed_laps=completed_laps))

    if effective_laps:
        visit(0, 0, 0)
        for lap in range(1, laps+1):
            for index in range(1, len(points)):
                # A recorded closure still gets its own original point event.
                completed = lap if index == len(points)-1 and points[index] == points[0] else lap-1
                visit(index, lap, completed)
            visit(0, lap, lap)
    else:
        for index in range(len(points)):
            visit(index, 0, 0)
    metadata = dict(requested_laps=laps, effective_laps=effective_laps, waypoint_count=len(points),
                    entry_count=len(entries),
                    completion_label='参考两圈完成' if effective_laps == 2 else '参考航线完成')
    caps = [entry_velocities[e['entry_index']] if 'entry_index' in e else point_velocities[e['waypoint_index']]
            for e in events]
    return _solve_fixed(start, nodes, events, caps, max_velocity, max_acceleration, metadata)


def build_fixed_leg(start_xyz, stops_xyz, max_velocity, max_acceleration, stop_velocities=None,
                    stop_events=None):
    """Solve start -> S1 -> ... -> Sn, at rest at both ends, without internal stops.

    One leg of a route that pauses at its end (a drop point or home). Each stop
    gets one event; stop_events optionally adds caller fields to them. The
    same MINCO solver, timing and optional speed caps as the lap route.
    """
    start = _xyz(start_xyz)
    stops = [_xyz(point) for point in stops_xyz]
    if not stops:
        raise ValueError('fixed leg requires at least one stop')
    max_velocity, max_acceleration = _limits(max_velocity, max_acceleration)
    stop_velocities = _caps(stop_velocities, len(stops), 'stop_velocities must match the stops')
    stop_events = [dict() for _ in stops] if stop_events is None else [dict(e) for e in stop_events]
    if len(stop_events) != len(stops):
        raise ValueError('stop_events must match the stops')
    nodes, events = [start], []
    for index, (point, extra) in enumerate(zip(stops, stop_events)):
        if point != nodes[-1]:
            nodes.append(point)
        events.append(dict(extra, node_index=len(nodes)-1, stop_index=index))
    metadata = dict(stop_count=len(stops), completion_label='参考航段完成')
    return _solve_fixed(start, nodes, events, stop_velocities, max_velocity, max_acceleration, metadata)


def _limits(max_velocity, max_acceleration):
    max_velocity, max_acceleration = float(max_velocity), float(max_acceleration)
    if not all(math.isfinite(value) and value > 0. for value in (max_velocity, max_acceleration)):
        raise ValueError('fixed trajectory velocity and acceleration limits must be finite and positive')
    return max_velocity, max_acceleration


def _caps(values, count, message):
    values = [None]*count if values is None else list(values)
    if len(values) != count:
        raise ValueError(message)
    for value in values:
        if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float)) or
                                  not math.isfinite(value) or value <= 0.):
            raise ValueError('point velocity caps must be None or finite and positive')
    return values


def _solve_fixed(start, nodes, events, event_caps, max_velocity, max_acceleration, metadata):
    """Shared MINCO solve and optional gate time warp; event_caps align with events."""
    distances = np.array([math.dist(a, b) for a, b in zip(nodes, nodes[1:])], dtype=float)
    # A literal d/v duration for a nearly coincident knot forces extreme
    # derivatives next to an ordinary segment, so minimum-snap interpolation
    # can make huge excursions even before floating-point conditioning fails.
    # Share one mean travel-time baseline across segments. This parameterizes
    # continuous movement; it adds no dwell, point rejection, or runtime gate.
    mean_distance = float(np.mean(distances)) if len(distances) else 0.
    durations = (distances+mean_distance)/max_velocity
    coefficients = np.empty((len(durations), 3, 8), dtype=float)
    maxima = np.array([0., 0., 1.], dtype=float)
    if len(durations):
        library = _load_library()
        coordinates = np.ascontiguousarray(nodes, dtype=float)
        pointer = ctypes.POINTER(ctypes.c_double)
        error = ctypes.create_string_buffer(1024)
        result = library.airseafly_min_snap_solve(
            len(durations), coordinates.ctypes.data_as(pointer), durations.ctypes.data_as(pointer),
            max_velocity, max_acceleration, coefficients.ctypes.data_as(pointer),
            maxima.ctypes.data_as(pointer), error, len(error))
        if result:
            raise RuntimeError('固定轨迹 MINCO 求解失败：'+error.value.decode('utf-8', errors='replace'))
        if not (np.isfinite(coefficients).all() and np.isfinite(durations).all() and
                np.isfinite(maxima).all()):
            raise RuntimeError('固定轨迹 MINCO 返回非有限数值')
    node_times = [0.]+np.cumsum(durations).tolist()
    for event in events:
        event['elapsed_s'] = node_times[event['node_index']]
    metadata = dict(metadata, source=SOURCE, upstream_revision=UPSTREAM_REVISION,
                    start_xyz=list(start),
                    requested_limits=dict(max_velocity_mps=max_velocity,
                                          max_acceleration_mps2=max_acceleration),
                    max_velocity_mps=float(maxima[0]), max_acceleration_mps2=float(maxima[1]),
                    time_scale=float(maxima[2]),
                    initial_timing='(segment_distance_m + mean_segment_distance_m) / max_velocity_mps',
                    mean_segment_distance_m=mean_distance,
                    coefficient_basis='ascending_normalized_segment_time',
                    realtime_avoidance=False,
                    obstacle_inflation=False, ego_virtual_boundary=False)
    base = FixedTrajectory(nodes, durations, coefficients, events, metadata)
    # A node takes the lowest cap of the events at it; the start has none.
    node_limits = [None]*len(nodes)
    for event, value in zip(events, event_caps):
        if value is not None:
            index = event['node_index']
            node_limits[index] = min(float(value), node_limits[index] or math.inf)
    if not len(durations) or all(value is None for value in node_limits):
        return base
    warped = _time_warp(base, node_limits, max_acceleration)
    if warped is None:
        return base
    warp, windows, bound = warped
    scale = warp['scale']
    for event in events:
        event['elapsed_s'] = _warped_time(warp, event['elapsed_s'])
    limited = max(w['peak']*w['rate'] for w in windows)/scale
    metadata.update(
        event_velocity_limits_mps=[None if v is None else float(v) for v in event_caps],
        time_warp=True, warp_time_scale=scale,
        base_max_velocity_mps=metadata['max_velocity_mps'],
        base_max_acceleration_mps2=metadata['max_acceleration_mps2'],
        # Through a window the speed is exactly rate*peak; elsewhere it never
        # exceeds the base speed, so these are upper bounds, not sampled values.
        limited_peak_velocity_mps=limited,
        max_velocity_mps=max(limited, metadata['max_velocity_mps']/scale),
        max_acceleration_mps2=bound/(scale*scale), extrema='upper_bound',
        gate_windows=[dict(start_s=_warped_time(warp, w['t0']), end_s=_warped_time(warp, w['t1']),
                           cap_mps=w['cap'], rate=w['rate'], base_peak_mps=w['peak'])
                      for w in windows])
    return FixedTrajectory(nodes, durations, coefficients, events, metadata, warp)
