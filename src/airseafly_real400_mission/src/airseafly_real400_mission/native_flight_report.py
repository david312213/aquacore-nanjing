"""Offline diagnostics for PX4-native references. No ROS or flight-control calls."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path


def _number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _vector(value):
    return (list(map(float, value)) if isinstance(value, (list, tuple))
            and len(value) == 3 and all(_number(v) for v in value) else None)


def _distance(a, b):
    return math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))


def _percentile(values, fraction):
    values = sorted(values)
    index = (len(values)-1)*fraction
    lo, hi = math.floor(index), math.ceil(index)
    return values[lo]+(values[hi]-values[lo])*(index-lo)


def vector_metrics(errors, unit):
    """Sample-weighted statistics, never a stability pass/fail judgment."""
    if not errors:
        return dict(available=False, samples=0, unit=unit,
                    reason='没有可计算的同坐标系参考与实际反馈样本')
    norms = [math.sqrt(sum(x*x for x in error)) for error in errors]
    count = len(errors)
    return dict(available=True, samples=count, unit=unit,
                mean_error_xyz=[sum(e[i] for e in errors)/count for i in range(3)],
                rmse_xyz=[math.sqrt(sum(e[i]**2 for e in errors)/count) for i in range(3)],
                rmse_3d=math.sqrt(sum(n*n for n in norms)/count),
                rmse_xy=math.sqrt(sum(e[0]**2+e[1]**2 for e in errors)/count),
                max_3d=max(norms), p95_3d=_percentile(norms, .95))


def load_rows(path):
    rows, rejected = [], Counter()
    with Path(path).open(encoding='utf-8') as stream:
        for line in stream:
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError('record is not an object')
            except (ValueError, TypeError):
                rejected['malformed_json'] += 1
                continue
            if not _number(row.get('stamp')):
                rejected['invalid_stamp'] += 1
                continue
            rows.append(row)
    return rows, dict(rejected)


def _partition(rows):
    groups, current, identity, last_stamp = [], None, None, None
    rejected = Counter()
    for row in rows:
        task_id = row.get('task_id')
        if not isinstance(task_id, str) or not task_id:
            rejected['missing_task_id'] += 1
            current, identity, last_stamp = None, None, None
            continue
        command = row.get('command') or {}
        feedback = row.get('task_feedback') or {}
        if not isinstance(command, dict) or not isinstance(feedback, dict):
            rejected['invalid_sample'] += 1
            continue
        # Contiguous runs stay separate, even if an old task ID reappears.
        key = (task_id, command.get('frame'), feedback.get('position_source'))
        if current is None or key != identity:
            current, identity, last_stamp = [], key, None
            groups.append(current)
        if last_stamp is not None and row['stamp'] <= last_stamp:
            rejected['duplicate_or_backwards_stamp'] += 1
            continue
        last_stamp = row['stamp']
        current.append(row)
    return [group for group in groups if group], rejected


def _native_control(row):
    status = row.get('control_status') or {}
    if not isinstance(status, dict):
        status = {}
    return not (status.get('armed') is False or status.get('offboard') is False
                or status.get('manual_paused') is True
                or status.get('owner') not in (None, 'PX4_NATIVE'))


def _samples(rows):
    """Validate observations independently of flight admission rules."""
    output, rejected = [], Counter()
    previous_position_stamp, previous_velocity_stamp = None, None
    unconfirmed_control = 0
    for row in rows:
        command, feedback = row.get('command') or {}, row.get('task_feedback') or {}
        status = row.get('control_status') or {}
        if not isinstance(status, dict):
            status = {}
        if not _native_control(row):
            rejected['not_native_armed_offboard'] += 1
            output.append((row, None, None))
            continue
        if not all(key in status for key in ('armed', 'offboard', 'owner')):
            unconfirmed_control += 1
        if not command.get('frame') or command.get('frame') != feedback.get('frame'):
            rejected['missing_or_mismatched_frame'] += 1
            output.append((row, None, None))
            continue
        position, velocity = None, None
        for field, previous in (('position', previous_position_stamp), ('velocity', previous_velocity_stamp)):
            stamp = feedback.get('pose_stamp' if field == 'position' else 'velocity_stamp', feedback.get('stamp'))
            ref, actual = _vector(command.get(field)), _vector(feedback.get(field))
            if ref is None or actual is None or not _number(stamp):
                rejected['invalid_'+field+'_feedback'] += 1
                continue
            if previous is not None and stamp <= previous:
                rejected['duplicate_or_backwards_'+field+'_stamp'] += 1
                continue
            if field == 'position':
                previous_position_stamp, position = stamp, [a-b for a, b in zip(ref, actual)]
            else:
                previous_velocity_stamp, velocity = stamp, [a-b for a, b in zip(ref, actual)]
        output.append((row, position, velocity))
    return output, dict(rejected), unconfirmed_control


def _takeoff(samples):
    eligible, begun, ended = [], False, False
    for row, error, _ in samples:
        source = row.get('reference_source')
        if begun and not _native_control(row):
            ended = True
        if source == 'TAKEOFF' and not ended:
            begun = True
        elif begun and source != 'HOLD':
            ended = True
        if begun and not ended and error is not None:
            target = _vector(row.get('takeoff_target_xyz'))
            # A TAKEOFF state has an explicit fixed endpoint, unlike command.position.
            if target is None and row.get('state') == 'TAKEOFF':
                target = _vector(row.get('target_xyz'))
            if target is not None:
                if source == 'HOLD' and _distance(row['command']['position'], target) > 1e-9:
                    ended = True
                else:
                    eligible.append((row, target))
    if not eligible:
        return dict(available=False, samples=0,
                    reason='缺少起飞段实际反馈或固定起飞终点；不使用移动参考高度推算超调')
    targets = [target for _, target in eligible]
    if any(_distance(target, targets[0]) > 1e-9 for target in targets):
        return dict(available=False, samples=len(eligible), reason='本段起飞终点发生变化，不能合并计算超调')
    final_z = targets[0][2]
    actual_z = [row['task_feedback']['position'][2] for row, _ in eligible]
    origins = [_vector(row.get('ground_origin')) for row, _ in eligible]
    origin = origins[0] if origins and all(x is not None for x in origins) else None
    if origin is not None and any(_distance(x, origin) > 1e-9 for x in origins):
        origin = None
    reference_finished = any(_distance(row['command']['position'], targets[0]) <= 1e-9
                             and _vector(row['command'].get('velocity')) == [0., 0., 0.]
                             for row, _ in eligible)
    return dict(available=True, samples=len(eligible), target_world_z=final_z,
                target_height_m=None if origin is None else final_z-origin[2],
                peak_height_m=None if origin is None else max(actual_z)-origin[2],
                observed_overshoot_m=max(0., max(actual_z)-final_z),
                reference_finished_observed=reference_finished,
                interval_s=[eligible[0][0]['stamp'], eligible[-1][0]['stamp']],
                coverage='仅起飞段及随后同终点保持；不包含后续 EGO 或人工调高，未观测部分未知')


def _hover(samples):
    segments, segment, target = [], [], None
    for row, error, _ in samples:
        command = row.get('command') or {}
        position, velocity = _vector(command.get('position')), _vector(command.get('velocity'))
        feedback = row.get('task_feedback') or {}
        eligible = (row.get('reference_source') == 'HOLD' and position is not None
                    and velocity is not None and max(abs(v) for v in velocity) <= 1e-9
                    and _native_control(row) and command.get('frame')
                    and command.get('frame') == feedback.get('frame'))
        same = target is not None and position is not None and _distance(position, target) <= 1e-9
        if segment and (not eligible or not same):
            segments.append(segment)
            segment, target = [], None
        if eligible and error is not None:
            if not segment:
                target = position
            segment.append((row, error))
    if segment:
        segments.append(segment)
    reports = []
    for segment in segments:
        first, last = segment[0][0], segment[-1][0]
        reports.append(dict(target_xyz=first['command']['position'],
            state=first.get('state'), phase=first.get('phase'),
            interval_s=[first['stamp'], last['stamp']], duration_s=last['stamp']-first['stamp'],
            position_error=vector_metrics([error for _, error in segment], 'm')))
    return dict(available=bool(reports), segments=reports,
                reason=None if reports else '没有固定位置、零速度参考的有效 HOLD 段；未设置停稳时长筛选')


def _tracking(samples, source):
    selected = [sample for sample in samples if sample[0].get('reference_source') == source]
    by_waypoint = {}
    for row, position_error, velocity_error in selected:
        index = row.get('waypoint_index')
        key = str(index) if isinstance(index, int) and not isinstance(index, bool) else 'unknown'
        by_waypoint.setdefault(key, []).append((row, position_error, velocity_error))
    def metrics(part):
        return dict(position_error=vector_metrics([p for _, p, _ in part if p is not None], 'm'),
                    velocity_error=vector_metrics([v for _, _, v in part if v is not None], 'm/s'))
    result = metrics(selected)
    result['waypoints'] = {index: metrics(part) for index, part in by_waypoint.items()}
    return result


def assess(rows, rejected=None, task_id=None):
    groups, partition_rejected = _partition(rows)
    totals = Counter(rejected or {})
    totals.update(partition_rejected)
    reports = []
    for group in groups:
        if task_id is not None and group[0]['task_id'] != task_id:
            continue
        samples, sample_rejected, unconfirmed = _samples(group)
        reports.append(dict(task_id=group[0]['task_id'], segment=len(reports)+1,
            mode=group[0].get('mode'), coordinate_frame=(group[0].get('command') or {}).get('frame'),
            feedback_source=(group[0].get('task_feedback') or {}).get('position_source'),
            samples=len(group), interval_s=[group[0]['stamp'], group[-1]['stamp']],
            rejected=sample_rejected, control_status_unconfirmed_samples=unconfirmed,
            takeoff=_takeoff(samples), hover=_hover(samples), ego=_tracking(samples, 'EGO'),
            fixed_trajectory=_tracking(samples, 'FIXED_TRAJECTORY')))
    issues = []
    if totals.get('missing_task_id'):
        issues.append('部分记录没有 task_id，无法可靠区分任务，未并入任务指标。')
    if not reports:
        issues.append('没有可归属的任务数据；所有指标均不可计算。')
    return dict(format=1, tasks=reports, rejected=dict(totals), issues=issues,
        method='同条记录的参考减实际反馈；逐样本统计，不插值，不作时间加权；重复或倒退时间单独计数。',
        diagnostic_only=True, automatically_applied=False,
        limits='数值仅描述已记录片段，缺失数据不表示零误差或通过；不判定实飞验收，不修改参数或控制任务。')


def analyze(path, task_id=None):
    path = Path(path)
    if path.is_dir():
        path = path/'native_trajectory.jsonl'
    rows, rejected = load_rows(path)
    result = assess(rows, rejected, task_id=task_id)
    result['source'] = str(path.resolve())
    return result


def report_text(result):
    lines = ['原生起飞与航线跟踪离线报告', result['method']]
    for task in result['tasks']:
        lines.append('\n任务 %s · 第 %d 段 · %s · %d 条记录' %
                     (task['task_id'], task['segment'], task['mode'] or '未知模式', task['samples']))
        takeoff = task['takeoff']
        lines.append('起飞高度超调：%.3f m（已观测片段）' % takeoff['observed_overshoot_m']
                     if takeoff['available'] else '起飞高度超调：不可计算，'+takeoff['reason'])
        if takeoff['available']:
            lines.append('  '+takeoff['coverage'])
            if not takeoff['reference_finished_observed']:
                lines.append('  未记录到起飞参考终点，超调统计不覆盖完整响应。')
        hover = task['hover']
        if hover['available']:
            for index, segment in enumerate(hover['segments'], 1):
                error = segment['position_error']
                lines.append('悬停段 %d：%.2f s，位置 RMSE %.3f m，最大偏差 %.3f m' %
                             (index, segment['duration_s'], error['rmse_3d'], error['max_3d']))
        else:
            lines.append('悬停偏差：不可计算，'+hover['reason'])
        for source, label in (('ego', 'EGO'), ('fixed_trajectory', '固定轨迹')):
            for key, description in (('position_error', '位置跟踪'), ('velocity_error', '速度跟踪')):
                metric = task[source][key]
                name = label+' '+description
                lines.append('%s：RMSE %.3f %s，P95 %.3f %s（%d 样本）' %
                             (name, metric['rmse_3d'], metric['unit'], metric['p95_3d'], metric['unit'], metric['samples'])
                             if metric['available'] else name+'：不可计算，'+metric['reason'])
        if task['control_status_unconfirmed_samples']:
            lines.append('控制权状态缺失：%d 条，不能确认这些记录处于实际飞控跟踪状态。' %
                         task['control_status_unconfirmed_samples'])
        if task['rejected']:
            lines.append('未纳入指标的观察字段：'+json.dumps(task['rejected'], ensure_ascii=False, sort_keys=True))
    if result['rejected']:
        lines.append('\n日志排除计数：'+json.dumps(result['rejected'], ensure_ascii=False, sort_keys=True))
    lines.extend(result['issues'])
    lines.append('\n'+result['limits'])
    return '\n'.join(lines)+'\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description='离线分析 native_trajectory.jsonl；不连接 ROS、不改参数。')
    parser.add_argument('log', help='native_trajectory.jsonl 或所在目录')
    parser.add_argument('--task-id', help='只报告指定任务，默认逐任务分段报告')
    parser.add_argument('--json', dest='json_output', metavar='PATH', help='另存 JSON；- 表示只向标准输出打印 JSON')
    args = parser.parse_args(argv)
    try:
        result = analyze(args.log, args.task_id)
        serialized = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
        if args.json_output == '-':
            print(serialized, end='')
        else:
            if args.json_output:
                output = Path(args.json_output)
                source = Path(result['source'])
                if output.resolve() == source:
                    raise ValueError('报告路径不能覆盖输入轨迹日志')
                output.write_text(serialized, encoding='utf-8')
            print(report_text(result), end='')
    except (OSError, ValueError) as exc:
        parser.exit(2, '离线报告失败：%s\n' % exc)
    return 0
