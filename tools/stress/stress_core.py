"""Pure helpers for the ground stress monitor: /proc parsing, ROS header bytes, phase summary.

No ROS or filesystem access here so the logic can be unit-tested off the aircraft.
Thresholds below only label the report; they never stop or gate anything.
"""
import json
import re
import struct
from collections import OrderedDict

ODOM_AGE_FLAG_S = 0.5      # 10-27 crash: /Odometry age climbed 0.5 s -> 6 s before OFFBOARD exit
PREDICTION_HORIZON_S = 1.0  # config/mission.yaml max_prediction_horizon_s

# First match wins; order matters (worker before camera, roslaunch before mission).
GROUP_PATTERNS = [
    ('stress_monitor', r'stress_monitor\.py'),
    ('red_point', r'red_point_worker|red_point_preview'),
    ('down_camera', r'usb_cam_node'),
    ('fast_lio', r'__name:=laserMapping|fastlio_mapping'),
    ('livox', r'__name:=livox_lidar_publisher|livox_ros_driver2_node'),
    ('mavros', r'__name:=mavros\b|mavros_node'),
    ('px4ctrl', r'__name:=px4ctrl|px4ctrl_node'),
    ('ego', r'ego_planner_node|traj_server'),
    ('native_bridge', r'__name:=native_lio_bridge|__name:=native_planning_cloud'),
    ('ros_core', r'rosmaster|rosout|roslaunch'),
    ('mission', r'airseafly_real400_mission|flight_entry\.py'),
    ('qgc', r'QGroundControl'),
    ('remote_desktop', r'/usr/NX/|nxnode|nxcodec|nxserver'),
    ('desktop', r'gnome-shell|Xorg|pulseaudio'),
]
_COMPILED = [(name, re.compile(pattern)) for name, pattern in GROUP_PATTERNS]


def classify(cmdline):
    for name, pattern in _COMPILED:
        if pattern.search(cmdline):
            return name
    return 'other'


def parse_cpu_line(text):
    """Return (busy, total) jiffies from the aggregate 'cpu ' line of /proc/stat."""
    for line in text.splitlines():
        if line.startswith('cpu '):
            values = [int(v) for v in line.split()[1:]]
            idle = values[3] + (values[4] if len(values) > 4 else 0)  # idle + iowait
            total = sum(values[:8])  # exclude guest columns, already counted in user
            return total - idle, total
    raise ValueError('no aggregate cpu line')


def cpu_percent(before, after):
    busy = after[0] - before[0]
    total = after[1] - before[1]
    return 100.0 * busy / total if total > 0 else 0.0


def parse_pid_jiffies(stat_text):
    """utime+stime from /proc/<pid>/stat; comm may contain spaces or ')'."""
    fields = stat_text[stat_text.rindex(')') + 2:].split()
    return int(fields[11]) + int(fields[12])


def group_percent(before, after, groups, total_delta):
    """Per-group CPU as % of one core. before/after: {pid: jiffies}; groups: {pid: name}.

    total_delta is the aggregate /proc/stat jiffy delta across all cores, so one core
    corresponds to total_delta / ncpu jiffies. Processes born mid-interval count from 0.
    """
    out = {}
    if total_delta <= 0:
        return out
    for pid, value in after.items():
        delta = value - before.get(pid, 0)
        if delta > 0:
            name = groups.get(pid, 'other')
            out[name] = out.get(name, 0) + delta
    return {name: 100.0 * jiffies / total_delta for name, jiffies in out.items()}


def header_stamp(buff):
    """Seconds from a serialized message whose first field is std_msgs/Header."""
    if len(buff) < 12:
        return None
    secs, nsecs = struct.unpack_from('<II', buff, 4)
    stamp = secs + nsecs * 1e-9
    return stamp if stamp > 0 else None


def _string_at(buff, offset):
    length, = struct.unpack_from('<I', buff, offset)
    start = offset + 4
    return buff[start:start + length].decode('utf-8', 'replace'), start + length


def string_payload(buff):
    """Data of a serialized std_msgs/String."""
    return _string_at(buff, 0)[0]


def mavros_state(buff):
    """(connected, armed, mode) from a serialized mavros_msgs/State."""
    _, offset = _string_at(buff, 12)  # header: seq, stamp, frame_id
    connected, armed = bool(buff[offset]), bool(buff[offset + 1])
    mode, _ = _string_at(buff, offset + 4)
    return connected, armed, mode


def prediction_status(text, now):
    """Correction lag, output lag and withheld count from FAST-LIO /prediction/status."""
    values = json.loads(text)
    correction = values.get('last_correction_stamp_s')
    output = values.get('last_output_stamp_s')
    return dict(
        pred_corr_lag_s=None if correction is None else now - correction,
        pred_out_lag_s=None if output is None else now - output,
        pred_withheld=values.get('withheld'),
        pred_last_withheld=values.get('last_withheld') or '')


STREAM_NAMES = ['lidar', 'lidar_imu', 'lio_odom', 'lio_high', 'planning_odom', 'red_point']
SAMPLE_FIELDS = (['wall_time', 'phase', 'cpu_pct', 'cpu_max_core_pct', 'load1', 'freq_mhz_min',
                  'freq_mhz_mean', 'temp_c', 'throttle_count', 'mem_avail_mb'] +
                 ['%s_%s' % (name, kind) for name in STREAM_NAMES for kind in ('hz', 'age_s')] +
                 ['pred_corr_lag_s', 'pred_out_lag_s', 'pred_withheld', 'pred_last_withheld',
                  'loc_healthy', 'fcu_connected', 'armed', 'mode'])
TEXT_FIELDS = {'phase', 'mode', 'pred_last_withheld'}


def parse_sample(row):
    out = {}
    for key, value in row.items():
        if key in TEXT_FIELDS:
            out[key] = value
        else:
            out[key] = None if value in ('', None) else float(value)
    return out


def _values(rows, key):
    return [r[key] for r in rows if r.get(key) is not None]


def _percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * len(ordered)))] if ordered else None


def _delta(rows, key):
    values = _values(rows, key)
    return values[-1] - values[0] if len(values) > 1 else 0


def summarize(samples, group_rows):
    """Per-phase report. samples: parsed SAMPLE_FIELDS dicts; group_rows: (phase, group, pct)."""
    phases = OrderedDict()
    for row in samples:
        phases.setdefault(row['phase'], []).append(row)
    group_sums = {}
    for phase, group, pct in group_rows:
        bucket = group_sums.setdefault(phase, {})
        bucket[group] = bucket.get(group, 0) + pct
    report = []
    for phase, rows in phases.items():
        times = _values(rows, 'wall_time')
        cpu = _values(rows, 'cpu_pct')
        lidar_hz = _values(rows, 'lidar_hz')
        unhealthy = 0.0
        for a, b in zip(rows, rows[1:]):
            if a.get('loc_healthy') == 0:
                unhealthy += b['wall_time'] - a['wall_time']
        item = OrderedDict(
            phase=phase, duration_s=times[-1] - times[0] if len(times) > 1 else 0,
            cpu_mean=sum(cpu) / len(cpu) if cpu else None, cpu_p95=_percentile(cpu, .95),
            cpu_max=max(cpu) if cpu else None,
            core_max=max(_values(rows, 'cpu_max_core_pct') or [None]),
            temp_max=max(_values(rows, 'temp_c') or [None]),
            freq_min=min(_values(rows, 'freq_mhz_min') or [None]),
            throttle_delta=_delta(rows, 'throttle_count'),
            lidar_hz_median=_percentile(lidar_hz, .5), lidar_hz_min=min(lidar_hz or [None]),
            odom_age_max=max(_values(rows, 'lio_odom_age_s') or [None]),
            high_age_max=max(_values(rows, 'lio_high_age_s') or [None]),
            pred_corr_lag_max=max(_values(rows, 'pred_corr_lag_s') or [None]),
            withheld_delta=_delta(rows, 'pred_withheld'),
            loc_unhealthy_s=unhealthy, armed_seen=any(r.get('armed') == 1 for r in rows))
        sums = group_sums.get(phase, {})
        count = max(1, len(rows))
        item['top_groups'] = [(g, round(v / count, 1)) for g, v in
                              sorted(sums.items(), key=lambda kv: -kv[1])[:6]]
        item['flags'] = flags_for(item)
        report.append(item)
    return report


def flags_for(item):
    flags = []
    if (item['odom_age_max'] or 0) > ODOM_AGE_FLAG_S:
        flags.append('LIO_LAG')
    if (item['pred_corr_lag_max'] or 0) > PREDICTION_HORIZON_S:
        flags.append('PRED_HORIZON')
    if item['withheld_delta'] > 0:
        flags.append('WITHHELD')
    if item['loc_unhealthy_s'] > 0:
        flags.append('LOC_UNHEALTHY')
    if item['lidar_hz_median'] and (item['lidar_hz_min'] or 0) < .5 * item['lidar_hz_median']:
        flags.append('LIDAR_DROP')
    if item['throttle_delta'] > 0:
        flags.append('THROTTLE')
    if item['armed_seen']:
        flags.append('ARMED!')
    return flags


def format_report(report):
    def f(value, digits=1):
        return '-' if value is None else ('%.*f' % (digits, value))
    lines = ['%-16s %6s %6s %6s %8s %8s %6s %6s %6s %5s  %s' % (
        'phase', 'dur_s', 'cpu%', 'cpu95', 'odomAge', 'corrLag', 'wthld', 'unhlth', 'lidHz',
        'tC', 'flags / top CPU groups (% of one core)')]
    for r in report:
        groups = ' '.join('%s=%s' % g for g in r['top_groups'][:4])
        lines.append('%-16s %6s %6s %6s %8s %8s %6d %6s %6s %5s  %s | %s' % (
            r['phase'][:16], f(r['duration_s'], 0), f(r['cpu_mean']), f(r['cpu_p95']),
            f(r['odom_age_max'], 2), f(r['pred_corr_lag_max'], 2), r['withheld_delta'],
            f(r['loc_unhealthy_s']), f(r['lidar_hz_min']), f(r['temp_max'], 0),
            ','.join(r['flags']) or 'ok', groups))
    return '\n'.join(lines)
