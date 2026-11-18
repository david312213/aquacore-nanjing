#!/usr/bin/env python3
"""Generate an offline XY, horizontal speed, acceleration, height and battery current plot from mission logs."""
import argparse
from collections import defaultdict
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.font_manager import fontManager
import numpy as np


PROJECT = Path(__file__).resolve().parents[2]
CST = timezone(timedelta(hours=8))
FIXED = 'FIXED_TRAJECTORY'
WINDOW_S = .20


def records(path):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except ValueError:
                if not line.endswith('\n'):  # Last line may still be in flight to disk.
                    break
                raise ValueError('日志存在无法解析的完整行：'+str(path))
            if isinstance(row, dict):
                yield row


def vector(item, key):
    data = item.get(key)
    if isinstance(data, list) and len(data) == 3 and all(isinstance(v, (int, float)) for v in data):
        if np.isfinite(data).all():
            return data[:2]
    return [np.nan, np.nan]


def flight_rows(path):
    tasks = defaultdict(list)
    for row in records(path):
        control = row.get('control_status') or {}
        command = row.get('command') or {}
        feedback = row.get('task_feedback') or {}
        if (row.get('task_id') and control.get('armed') and control.get('offboard')
                and control.get('owner') == 'PX4_NATIVE' and not control.get('manual_paused')
                and command.get('frame') and command['frame'] == feedback.get('frame')
                and isinstance(row.get('stamp'), (int, float))):
            tasks[row['task_id']].append(row)
    return tasks


def find_flight(source, task_id):
    if source is None:
        files = sorted((PROJECT/'data/mission').glob('*/native_trajectory.jsonl'), reverse=True)
    else:
        files = [source/'native_trajectory.jsonl' if source.is_dir() else source]
    for path in files:
        tasks = flight_rows(path)
        candidates = [(task, rows) for task, rows in tasks.items()
                      if (task_id is None or task == task_id)
                      and any(row.get('reference_source') != 'HOLD' for row in rows)]
        if candidates:
            task, rows = max(candidates, key=lambda pair: pair[1][-1]['stamp'])
            return path.resolve(), task, rows
    raise ValueError('没有找到匹配且有解锁、OFFBOARD 反馈的飞行任务。')


def split_runs(rows):
    """Keep separate legs, frame changes and gaps from creating artificial lines."""
    group, last_key, last_time = [], None, None
    for row in rows:
        command = row['command']
        key = (row.get('reference_source'), command.get('trajectory_id'), command['frame'])
        stamp = row['stamp']
        if group and (key != last_key or not 0 < stamp-last_time < .30):
            yield group
            group = []
        group.append(row)
        last_key, last_time = key, stamp
    if group:
        yield group


def ground_z(rows):
    """Takeoff ground Z used by the mission for every relative height."""
    for row in rows:
        origin = row.get('ground_origin')
        if isinstance(origin, list) and len(origin) == 3 and isinstance(origin[2], (int, float)):
            return float(origin[2])
    return None


def height_series(rows, field, stamp_keys, origin, ground):
    times, heights = [], []
    for row in rows:
        value = row[field]
        stamp = next((value[key] for key in stamp_keys
                      if isinstance(value.get(key), (int, float)) and value[key] > 0), row['stamp'])
        t = stamp-origin
        if times and t <= times[-1]:
            continue
        position = value.get('position')
        z = position[2] if isinstance(position, list) and len(position) == 3 else None
        times.append(t)
        heights.append(z-ground if isinstance(z, (int, float)) and np.isfinite(z) else np.nan)
    return np.asarray(times, dtype=float), np.asarray(heights, dtype=float)


def series(rows, field, stamp_keys, origin):
    times, positions, velocities, accelerations = [], [], [], []
    for row in rows:
        value = row[field]
        stamp = next((value[key] for key in stamp_keys
                      if isinstance(value.get(key), (int, float)) and value[key] > 0), row['stamp'])
        t = stamp-origin
        if times and t <= times[-1]:
            continue
        times.append(t)
        positions.append(vector(value, 'position'))
        velocities.append(vector(value, 'velocity'))
        accelerations.append(vector(value, 'acceleration'))
    return tuple(np.asarray(values) for values in (times, positions, velocities, accelerations))


def estimated_acceleration(times, velocity):
    """Differentiate XY velocity vectors over a centered window, within each logged run."""
    result = np.full(len(times), np.nan)
    half = WINDOW_S/2
    valid = np.isfinite(velocity).all(axis=1)
    # Treat missing velocity and irregular feedback gaps as interpolation boundaries.
    cuts = np.flatnonzero((np.diff(times) > half) | ~valid[:-1] | ~valid[1:])+1
    for indices in np.split(np.arange(len(times)), cuts):
        if len(indices) < 2:
            continue
        t, v = times[indices], velocity[indices]
        usable = (t-half >= t[0]) & (t+half <= t[-1])
        center = t[usable]
        if len(center):
            before = np.column_stack([np.interp(center-half, t, axis) for axis in v.T])
            after = np.column_stack([np.interp(center+half, t, axis) for axis in v.T])
            result[indices[usable]] = np.linalg.norm(after-before, axis=1)/WINDOW_S
    return result


def battery_series(rows, origin):
    """Battery samples logged from /mavros/battery; returns times, discharge current (A), voltage (V)."""
    times, currents, voltages, last = [], [], [], None
    for row in rows:
        battery = row.get('battery')
        if not isinstance(battery, dict):
            continue
        stamp = battery.get('stamp')
        stamp = stamp if isinstance(stamp, (int, float)) and stamp > 0 else row['stamp']
        if stamp == last:
            continue  # The 50 Hz trace repeats the latest battery message.
        last = stamp
        value = lambda key: battery[key] if isinstance(battery.get(key), (int, float)) else np.nan
        times.append(stamp-origin)
        currents.append(value('current'))
        voltages.append(value('voltage'))
    times, currents, voltages = (np.asarray(v, dtype=float) for v in (times, currents, voltages))
    order = np.argsort(times, kind='stable')
    times, currents, voltages = times[order], currents[order], voltages[order]
    finite = currents[np.isfinite(currents)]
    if len(finite) and np.median(finite) < 0:
        currents = -currents  # MAVROS reports discharge as negative current.
    return times, currents, voltages


def consumed_mah(times, currents):
    good = np.isfinite(currents)
    if good.sum() < 2:
        return None
    t, i = times[good], currents[good]
    dt = np.diff(t)
    keep = (dt > 0) & (dt < 2.)  # Do not bridge long logging gaps.
    return float(np.sum((i[:-1]+i[1:])[keep]/2*dt[keep])/3.6)


def peak(items):
    valid = [(float(t), float(value)) for ts, values in items for t, value in zip(ts, values)
             if np.isfinite(value)]
    return max(valid, key=lambda pair: pair[1]) if valid else None


def plot(path, task, rows, output):
    font = next((name for name in ('Noto Sans CJK SC', 'Noto Sans CJK JP', 'Hiragino Sans GB',
                                   'PingFang SC', 'Microsoft YaHei')
                 if any(item.name == name for item in fontManager.ttflist)), None)
    plt.rcParams.update({'font.family': font or 'DejaVu Sans', 'axes.unicode_minus': False,
                         'axes.spines.top': False, 'axes.spines.right': False, 'font.size': 10})
    tr = lambda zh, en: zh if font else en
    fixed = any(row.get('reference_source') == FIXED for row in rows)
    selected = [row for row in rows if row.get('reference_source') == FIXED] if fixed else rows
    origin = selected[0]['stamp']
    fig = plt.figure(figsize=(13.2, 13.0), constrained_layout=True)
    grid = fig.add_gridspec(4, 2, width_ratios=(1.05, 1.2))
    xy = fig.add_subplot(grid[:, 0])
    speed = fig.add_subplot(grid[0, 1])
    accel = fig.add_subplot(grid[1, 1], sharex=speed)
    height = fig.add_subplot(grid[2, 1], sharex=speed)
    power = fig.add_subplot(grid[3, 1], sharex=speed)
    ground = ground_z(rows)
    observed_height, planned_height = [], []
    observed_speed, planned_speed, observed_accel, planned_accel = [], [], [], []
    actual_runs = []
    plan_xy = []
    points = None
    leg = path.parent/('fixed_trajectory-'+task+'.json')
    if leg.is_file():
        saved = json.loads(leg.read_text(encoding='utf-8'))
        points = saved.get('route', {}).get('points')
    for run in split_runs(selected):
        rt, rp, rv, ra = series(run, 'command', ('source_stamp',), origin)
        at, ap, av, aa = series(run, 'task_feedback', ('velocity_stamp', 'pose_stamp', 'stamp'), origin)
        measured_speed, reference_speed = np.linalg.norm(av, axis=1), np.linalg.norm(rv, axis=1)
        measured_accel = np.linalg.norm(aa, axis=1)
        derived = estimated_acceleration(at, av)
        actual_runs.append((ap, measured_speed))
        plan_xy.append(rp)
        observed_speed.append((at, measured_speed))
        planned_speed.append((rt, reference_speed))
        observed_accel.append((at, measured_accel if np.isfinite(measured_accel).any() else derived))
        planned_accel.append((rt, np.linalg.norm(ra, axis=1)))
        if ground is not None:
            planned_height.append(height_series(run, 'command', ('source_stamp',), origin, ground))
            observed_height.append(height_series(run, 'task_feedback', ('pose_stamp', 'stamp'), origin, ground))
        if np.isfinite(measured_accel).any():
            accel.plot(at, measured_accel, color='#087e71', label=tr('实测加速度', 'Measured acceleration'))
        else:
            accel.plot(at, derived, color='#087e71', label=tr('由实测速度估算（0.20 s）', 'From measured velocity (0.20 s)'))
    actual_peak, reference_peak = peak(observed_speed), peak(planned_speed)
    speed_cap = max(.1, actual_peak[1] if actual_peak else 0)
    scale = plt.Normalize(0, speed_cap)
    collection = None
    for rp in plan_xy:
        xy.plot(*rp.T, color='#65758b', ls='--', lw=1.5, label=tr('规划轨迹', 'Reference trajectory'))
    for ap, velocity in actual_runs:
        pair = np.stack((ap[:-1], ap[1:]), axis=1)
        color = (velocity[:-1]+velocity[1:])/2
        good = np.isfinite(pair).all(axis=(1, 2)) & np.isfinite(color)
        if good.any():
            collection = LineCollection(pair[good], cmap='viridis', norm=scale, linewidths=2.7, zorder=3)
            collection.set_array(color[good])
            xy.add_collection(collection)
        xy.plot(*ap.T, color='#138279', alpha=.30, lw=1.5)
    if points:
        xy.scatter(*np.array(points).T, s=18, c='#132843', zorder=4, label=tr('录制航点', 'Recorded waypoint'))
        for index, point in enumerate(points, 1):
            xy.annotate(str(index), point, xytext=(4, 4), textcoords='offset points', fontsize=8)
    if collection is not None:
        bar = fig.colorbar(collection, ax=xy, pad=.035, shrink=.79)
        bar.set_label(tr('实际水平速度 (m/s)', 'Actual horizontal speed (m/s)'))
    for ts, values in planned_speed:
        speed.plot(ts, values, color='#65758b', ls='--', label=tr('规划速度', 'Reference speed'))
    for ts, values in observed_speed:
        speed.plot(ts, values, color='#087e71', label=tr('实测速度', 'Measured speed'))
    for ts, values in planned_accel:
        if np.isfinite(values).any():
            accel.plot(ts, values, color='#65758b', ls='--', label=tr('规划加速度', 'Reference acceleration'))
    for ax, marker, unit in ((speed, actual_peak, 'm/s'), (accel, peak(observed_accel), 'm/s²')):
        if marker is not None:
            ax.scatter(*marker, color='#b84425', marker='*', s=95, zorder=5)
            ax.annotate(f'{marker[1]:.2f} {unit}', marker, xytext=(5, 7), textcoords='offset points',
                        color='#9c351d', fontsize=10)
        ax.set_ylim(bottom=0)
        ax.margins(x=.04, y=.16)
    height_stats = None
    if observed_height:
        for ts, values in planned_height:
            height.plot(ts, values, color='#65758b', ls='--', label=tr('规划高度', 'Reference height'))
        for ts, values in observed_height:
            height.plot(ts, values, color='#087e71', label=tr('实测高度', 'Measured height'))
        high, low = peak(observed_height), peak([(ts, -values) for ts, values in observed_height])
        errors = np.concatenate([values-np.interp(ts, rts, rvalues)
                                 for (ts, values), (rts, rvalues) in zip(observed_height, planned_height)
                                 if len(rts) and len(ts)])
        errors = errors[np.isfinite(errors)]
        if high is not None and low is not None and len(errors):
            height_stats = dict(high=high, low=(low[0], -low[1]), over=float(errors.max()),
                                under=float(errors.min()), rms=float(np.sqrt(np.mean(errors**2))))
            height.scatter(*high, color='#b84425', marker='*', s=95, zorder=5)
            height.annotate(f'{high[1]:.2f} m', high, xytext=(5, 7), textcoords='offset points',
                            color='#9c351d', fontsize=10)
        height.margins(x=.04, y=.25)
    else:
        height.text(.5, .5, tr('此日志没有起飞基准，无法换算离地高度', 'No takeoff origin in this log'),
                    transform=height.transAxes, ha='center', va='center', color='#65758b')
    height_title = tr('离地高度（相对本次起飞点）', 'Height above takeoff point')
    if height_stats:
        height_title += tr(f'：比参考最多高 {height_stats["over"]:+.2f} m、最多低 {height_stats["under"]:+.2f} m',
                           f': max {height_stats["over"]:+.2f} / min {height_stats["under"]:+.2f} m vs reference')
    bt, current, voltage = battery_series(selected, origin)
    current_peak = peak([(bt, current)])
    current_mean = float(np.nanmean(current)) if np.isfinite(current).any() else None
    used = consumed_mah(bt, current)
    if current_peak is not None:
        power.plot(bt, current, color='#c0571f', label=tr('电流', 'Current'))
        power.scatter(*current_peak, color='#b84425', marker='*', s=95, zorder=5)
        power.annotate(f'{current_peak[1]:.1f} A', current_peak, xytext=(5, 7), textcoords='offset points',
                       color='#9c351d', fontsize=10)
        power.set_ylim(bottom=0)
        power.margins(x=.04, y=.16)
        if current_mean is not None:
            power.axhline(current_mean, color='#c0571f', ls=':', lw=1,
                          label=tr(f'平均 {current_mean:.1f} A', f'Mean {current_mean:.1f} A'))
    else:
        power.text(.5, .5, tr('此日志没有电流记录', 'No battery current in this log'),
                   transform=power.transAxes, ha='center', va='center', color='#65758b')
    volts = None
    if np.isfinite(voltage).any():
        volts = power.twinx()
        volts.spines['right'].set_visible(True)
        volts.plot(bt, voltage, color='#3a5f9e', lw=1.1, alpha=.75, label=tr('电压', 'Voltage'))
        volts.set_ylabel('V', color='#3a5f9e')
        volts.tick_params(axis='y', colors='#3a5f9e')
        volts.margins(y=.25)
    title = tr('电池电流', 'Battery current')
    if used is not None:
        title += tr(f'（本段消耗约 {used:.0f} mAh）', f' (~{used:.0f} mAh used)')
    xy.set_aspect('equal', adjustable='datalim')
    xy.set(xlabel='X (m)', ylabel='Y (m)', title=tr('XY 轨迹', 'XY trajectory'))
    speed.set(ylabel='m/s', title=tr('水平速度', 'Horizontal speed'))
    accel.set(ylabel='m/s²', title=tr('水平加速度', 'Horizontal acceleration'))
    height.set(ylabel='m', title=height_title)
    power.set(ylabel='A', title=title,
              xlabel=tr('距本图记录段开始 (s)', 'Elapsed from plotted segment start (s)'))
    for ax in (xy, speed, accel, height, power):
        ax.grid(alpha=.18)
        handles, labels = ax.get_legend_handles_labels()
        if ax is power and volts is not None:
            extra = volts.get_legend_handles_labels()
            handles, labels = handles+extra[0], labels+extra[1]
        legend = dict(zip(labels, handles))
        if legend:
            ax.legend(list(legend.values()), list(legend), loc='best', frameon=False, fontsize=9)
    time = datetime.fromtimestamp(rows[0]['stamp'], CST).strftime('%Y-%m-%d %H:%M:%S')
    scope = tr('固定轨迹段', 'Fixed trajectory segments') if fixed else tr('任务控制段', 'Task control segments')
    fig.suptitle(f'{time}  |  {scope}', fontsize=16, color='#162c46')
    output = (output or PROJECT/'data/analysis'/path.parent.name/f'{task[:8]}-flight.png').resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=165)
    plt.close(fig)
    print('日志：'+str(path))
    print(f'任务：{task}  |  {scope}')
    print('图：'+str(output))
    for name, value, unit in (
        ('实际速度峰值', actual_peak, 'm/s'), ('规划速度峰值', reference_peak, 'm/s'),
        ('实际加速度峰值（无加速度字段时为速度差分估算）', peak(observed_accel), 'm/s²'),
        ('规划加速度峰值', peak(planned_accel), 'm/s²'), ('电流峰值', current_peak, 'A')
    ):
        if value is not None:
            print(f'{name}：{value[1]:.2f} {unit}（图中 {value[0]:.2f} s）')
    if height_stats:
        print(f'实测高度最高：{height_stats["high"][1]:.2f} m（图中 {height_stats["high"][0]:.2f} s），'
              f'最低：{height_stats["low"][1]:.2f} m（图中 {height_stats["low"][0]:.2f} s）')
        print(f'高度相对参考：最多高 {height_stats["over"]:+.2f} m，最多低 {height_stats["under"]:+.2f} m，'
              f'RMS {height_stats["rms"]:.3f} m')
    if current_mean is not None:
        print(f'平均电流：{current_mean:.2f} A')
    if used is not None:
        print(f'本段消耗：约 {used:.0f} mAh')
    if current_peak is None:
        print('电流：此日志没有 battery 记录（更新 native_runtime 后的新飞行才有）')


def main():
    parser = argparse.ArgumentParser(description='读取飞行日志并生成 XY 轨迹、水平速度、水平加速度、离地高度和电池电流图。')
    parser.add_argument('log', nargs='?', type=Path, help='任务日志目录或 native_trajectory.jsonl，默认最新实飞')
    parser.add_argument('--task-id', help='日志内的完整任务 ID')
    parser.add_argument('--output', type=Path, help='输出 PNG 路径，默认保存在 data/analysis 下')
    args = parser.parse_args()
    try:
        path, task, rows = find_flight(args.log, args.task_id)
        plot(path, task, rows, args.output)
    except (OSError, ValueError) as exc:
        parser.exit(2, f'{exc}\n')


if __name__ == '__main__':
    main()
