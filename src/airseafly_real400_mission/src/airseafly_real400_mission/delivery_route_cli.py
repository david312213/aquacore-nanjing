"""Recorded delivery entry: preview locally or submit to the existing manager."""
import argparse
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import sys

from .configuration import project_root, runtime_paths
from .flight_configuration import base_configuration, apply_flight_configuration
from .guided_route import LOOP_SECTION_NAMES, all_route_points, load_route, validate_route
from .delivery_alignment import landing_settings
from .native_delivery_route import build_itinerary, delivery_timing_mode, describe_laps
from . import native_parameters
from .payload import actuator_command
from .rpc import request


def parser():
    result = argparse.ArgumentParser(description='delivery_route_options.laps = 巡航闭环圈数（默认 1）：进场点 → 巡航闭环 N 圈 → 投放环在各投放点下降至识别高度、红点对准、下降投放 → 返航点 → HOME 降落；旧的共用一套点航线为巡航 N 圈后第 N+1 圈投放，laps: 0 为第一圈边飞边投')
    result.add_argument('command', nargs='?', default='run', choices=('run', 'payload', 'status', 'hold'))
    result.add_argument('action', nargs='?', choices=('lock', 'release'), help='payload：关闭或释放对应投放位')
    result.add_argument('slot', nargs='?', type=int, choices=(1, 2, 3, 4), help='payload：投放编号1～4，对应MAIN6/7的两档')
    result.add_argument('--route', type=Path, default=project_root()/'data/routes/delivery_route.yaml')
    result.add_argument('--config', help='原机配置文件')
    result.add_argument('--flight-config', type=Path, default=project_root()/'flight/flight.yaml')
    result.add_argument('--set', dest='overrides', action='append', default=[])
    result.add_argument('--no-loop-retiming', action='store_true',
                        help='本次任务关闭绕圈局部时间分配优化，使用原固定轨迹配时')
    result.add_argument('--preview-html', type=Path,
                        help='轨迹 HTML 预览保存位置；默认与航线同目录的 <航线名>_preview.html')
    result.add_argument('--json', action='store_true')
    result.add_argument('--verbose', action='store_true')
    native_parameters.add_arguments(result)
    return result


def resolve_config(args):
    overrides = list(args.overrides)+native_parameters.overrides(args)
    if getattr(args, 'delivery_setpoint_mode', None):
        overrides.append('motion.native_setpoint_mode='+args.delivery_setpoint_mode)
    config = apply_flight_configuration(base_configuration(args.config), args.flight_config,
                                        task='delivery-route', overrides=overrides)
    if getattr(args, 'no_loop_retiming', False):  # Contest entry only.
        config['delivery_route_options']['local_retiming_enabled'] = False
    native_parameters.validate(config)
    return config


def navigation_summary(config):
    """Backend and the handoff check, shared by the contest and drop-test previews."""
    backend = config.get('navigation', {}).get('backend', 'ego')
    return dict(backend=backend, arrival_radius_m=config['flight']['arrival_radius_m'],
                arrival_dwell_s=config['flight']['arrival_dwell_s'],
                home_arrival_xy_m=config['flight'].get('home_arrival_xy_m', .05),
                home_arrival_timeout_s=config['delivery_route_options']['visual_alignment']
                    .get('motion_timeout_margin_s', 15.))


def home_arrival_text(value):
    return ('降落点水平 %.2f m 内持续 %.1f s 才开始降落，参考结束 %g s 后仍未进入则在当时位置降落'
            % (value['home_arrival_xy_m'], value['arrival_dwell_s'], value['home_arrival_timeout_s']))


def navigation_line(value):
    if value['backend'] == 'fixed_min_snap':
        return ('点间用固定轨迹航段，不运行 EGO；参考到投放点后 XYZ %.2f m 内持续 %.1f s 才开始投放；%s。'
                % (value['arrival_radius_m'], value['arrival_dwell_s'], home_arrival_text(value)))
    return '点间用 EGO 导航；XYZ %.2f m 内持续 %.1f s 算到点。' % (value['arrival_radius_m'], value['arrival_dwell_s'])


def navigation_timing(planned, itinerary):
    """Partition reference time at waypoint events, excluding all flight actions/waits."""
    last_lap = max(step['lap'] or 0 for step in itinerary)
    laps = {lap: 0. for lap in range(1, last_lap)}
    result = dict(entry_s=0., handoff_s=0., delivery_navigation_s=0., return_home_s=0.)
    first = True
    for leg in planned:
        previous = 0.
        for event in leg['trajectory'].events:
            step = itinerary[event['itinerary_index']]
            duration = event['elapsed_s']-previous
            previous = event['elapsed_s']
            if first or step['lap'] == 0:
                # Takeoff to the first point, plus any recorded entry points.
                result['entry_s'] += duration
                first = False
            elif step['lap'] is None or step.get('route_source') == 'return':
                result['return_home_s'] += duration
            elif step['lap'] < last_lap:
                laps[step['lap']] += duration
            else:
                result['delivery_navigation_s'] += duration
    result['loaded_laps'] = [dict(lap=lap, duration_s=duration) for lap, duration in laps.items()]
    result['delivery_lap_s'] = (result['handoff_s']+result['delivery_navigation_s']+result['return_home_s']
                                + (result['entry_s'] if last_lap == 1 else 0.))
    result['total_navigation_s'] = sum(leg['trajectory'].duration for leg in planned)
    return result


def fixed_preview(config, document, itinerary):
    """Each fixed leg from the recorded points; ground assumed at HOME, Z=0."""
    from .native_delivery_route import plan_fixed_legs
    from .route_profiles import gate_velocity
    legs = []
    planned = plan_fixed_legs(config, itinerary, (*document['home'], 0.))
    for leg in planned:
        summary = leg['trajectory'].summary()
        legs.append(dict(first_step=leg['first']+1, last_step=leg['last']+1, ends=itinerary[leg['last']+1]['kind'],
                         duration_s=summary['duration_s'], max_velocity_mps=summary['max_velocity_mps'],
                         max_acceleration_mps2=summary['max_acceleration_mps2'],
                         local_retiming=summary.get('local_retiming'),
                         gate_peak_velocity_mps=summary.get('limited_peak_velocity_mps')))
    return dict(legs=legs, gate_velocity_mps=gate_velocity(config),
                navigation_timing=navigation_timing(planned, itinerary),
                flight_time_s=sum(leg['duration_s'] for leg in legs),
                arrival=dict(navigation_summary(config), radius_m=config['flight']['arrival_radius_m'],
                             dwell_s=config['flight']['arrival_dwell_s']))


def preview(config, document):
    document = validate_route(document, require_complete=True)
    heights = dict(cruise=config['flight']['cruise_height_m'],
                   gate=native_parameters.gate_height(config), drop=config['flight']['drop_height_m'])
    backend = config.get('navigation', {}).get('backend', 'ego')
    if backend not in ('ego', 'fixed_min_snap'):
        raise ValueError('navigation.backend 必须为 ego 或 fixed_min_snap')
    fixed = backend == 'fixed_min_snap'
    navigation_note = ('航点之间用固定轨迹航段：到投放点或 HOME 停下，中间点连续通过；不运行 EGO，不实时绕障。'
                       if fixed else
                       '航点切换及投放下降、回执、回升期间保持同一 EGO 和地图运行；投放动作由原生参考执行。')
    laps = config['delivery_route_options'].get('laps', 1)
    itinerary = build_itinerary(document, laps)
    points = all_route_points(document)
    sections = document['version'] == 4
    alignment = deepcopy(config['delivery_route_options']['visual_alignment'])
    landing = landing_settings(config['delivery_route_options'].get('landing_alignment'))
    for step in itinerary:
        if step['height_role'] in heights:
            step['height_m'] = heights[step['height_role']]
        if step['kind'] == 'drop':
            command = actuator_command(step['drop_slot'], 'release', config['payload'])
            step.update(main_output=command['main_output'], pwm_us=command['pwm_us'],
                        alignment_samples=1,
                        mount_offset_body_m=deepcopy(config['delivery_route_options']['mount_offsets_body_m'][str(step['drop_slot'])]))
    return dict(mode='delivery_route', frame=document['frame'], home=document['home'], backend=backend,
                fixed_legs=fixed_preview(config, document, itinerary) if fixed else None,
                route_points=len(points),
                loaded_route_points=len(document['cruise_loop'] if sections else document['route']),
                section_points=({name: len(document[name]) for name in LOOP_SECTION_NAMES} if sections else None),
                heights_relative_to_takeoff_m=heights,
                recording_mode=document.get('mode', 'wizard'), laps=laps,
                local_retiming_enabled=delivery_timing_mode(config) != 'original',
                timing_mode=delivery_timing_mode(config),
                drop_points=sum(point['kind'] == 'drop' for point in points),
                controller=config['flight']['controller'],
                reference_mode=config['flight']['native_reference_mode'],
                payload_channels=config['payload']['actuator_indices'],
                payload_main_outputs=config['payload'].get('main_outputs', {}),
                visual_alignment=alignment, landing_alignment=landing,
                delivery_motion=native_parameters.delivery_motion(config),
                recognition_fallback_xy='last_visual_target_or_recorded_waypoint',
                success_identity='route_index',
                itinerary=itinerary,
                notes=['同次定位录点和飞行，XY 不重置；录点不采用手持时的高度。',
                       (('起飞后经 %d 个进场点进入巡航闭环，巡航 %d 圈（前几圈回到巡航首点闭环，最后一圈从巡航最后一点直接飞到投放环第一点）；随后飞单独录制的投放环，在投放点依次投放，经 %d 个返航点回 HOME 降落。' % (
                           len(document['entry']), laps, len(document['return'])))
                        if sections else
                        ('第一阶段满载绕圈 %d 圈，每圈回到绕圈首点；第二阶段第 %d 圈按同一套航点在投放点依次投放，走完全部航点回到绕圈首点后再回 HOME 降落。' % (
                           laps, laps+1)
                        if laps else
                        '只飞一圈：按录制顺序飞，到投放点停下投放，走完全部航点回到第一个点后再回 HOME 降落。')),
                       navigation_note,
                       '到点先降至识别高度，取一次红点图生成固定目标，移动中不跟踪图像；按录制航点选靶，重拍时按上次目标选靶。',
                       '识别超时沿最后视觉 XY 下降，无视觉目标时沿录制航点下降；下降超时按实测位置和高度投放。',
                       '识别进程独立运行；相机或识别异常按无识别处理，不停止航线；每个投放点分别记录成功回执。',
                       ('起飞后执行此序列；圆环段按固定轨迹曲线穿过，先用 --dry-run 检查各航段。' if fixed else
                        '起飞后执行此序列；圆环段沿用 EGO 逐点导航，实际穿环路径仍需单独确认。'),
                       '预览不连接飞机，不验证实际航迹、离地高度或五分钟完赛时间。'])


def show_preview(value, as_json=False):
    if as_json:
        print(json.dumps(value, ensure_ascii=False, indent=2))
        return
    if value.get('timing_mode'):
        from .local_retiming import TIMING_LABELS
        print('绕圈配时版本：%s（navigation.timing=%s）' % (
            TIMING_LABELS[value['timing_mode']], value['timing_mode']))
    if not value['local_retiming_enabled']:
        print('本次任务已关闭绕圈局部时间分配优化。')
    heights = value['heights_relative_to_takeoff_m']
    order = '→'.join(str(slot) for slot in range(1, value['drop_points']+1))
    if value.get('section_points'):
        counts = value['section_points']
        print('航线：分段录点，进场 %d 点 → 巡航闭环 %d 点 ×%d 圈 → 投放环 %d 点（%d 个投放点，按 %s 释放）→ 返航 %d 点 → HOME 降落。' % (
            counts['entry'], counts['cruise_loop'], value['laps'], counts['delivery_loop'],
            value['drop_points'], order, counts['return']))
    else:
        print('航线：%s，%d 个点、%d 个投放点；%s按 %s 释放；最后回 HOME 降落。' % (
            {'free': '自由录点（绕圈和投放共用）'}.get(value['recording_mode'], '九步向导'), value['route_points'],
            value['drop_points'], ('巡航 %d 圈后第 %d 圈' % (value['laps'], value['laps']+1)
                                   if value['laps'] else '一圈，第一圈边飞边'), order))
        print('绕圈和投放共用同一套航点，只需录制一次。')
    print('巡航 %.2fm / 投放 %.2fm / 圆环段 %.2fm，均相对本次起飞基准。' %
          (heights['cruise'], heights['drop'], heights['gate']))
    print(native_parameters.delivery_motion_line(value['delivery_motion']))
    alignment = value['visual_alignment']
    print('中间红点识别高度：定位 Z=%.2fm；中间下降超时保持实测 XYZ 继续识别。' % alignment['intermediate_height_m'])
    print('取图每次最多 %gs；轨迹结束后到位宽限 %gs；每点额外重拍最多 %d 次，重拍后 XY 各放宽 %gm。' %
          (alignment['frame_timeout_s'], alignment['motion_timeout_margin_s'],
           alignment['retake_limit'], alignment['retry_xy_relax_m']))
    print('先沿用原生起飞，然后：')
    for index, step in enumerate(value['itinerary'], 1):
        source = step.get('route_source')
        if source in LOOP_SECTION_NAMES:
            label = ('巡航第%d圈' % step['lap']) if source == 'cruise_loop' else LOOP_SECTION_NAMES[source]
        else:
            label = '返航' if step['lap'] is None else '第%d圈' % step['lap']
        xy = 'XY=(%.3f, %.3f)' % tuple(step['xy'])
        if step['kind'] == 'drop':
            text = '下降至识别高度 → 红点对准 → 下降 → 投放%d（MAIN%d，%g μs）并等待原回执 → 回升；偏移[前,左]=%s m' % (step['drop_slot'], step['main_output'], step['pwm_us'], step['mount_offset_body_m'])
        elif step['kind'] == 'land' and value['landing_alignment']['enabled']:
            landing = value['landing_alignment']
            text = ('H 标对准 → 下降至定位 Z=%.2fm → H 标对准 → 以 %.2fm/s 下降至 %.2fm → AUTO.LAND；'
                    '每次取图 %gs，超时在当前 XY 直接下降，不重拍' % (
                        alignment['intermediate_height_m'], landing['final_speed_mps'],
                        landing['final_height_m'], landing['frame_timeout_s']))
        elif step['kind'] == 'land':
            text = 'HOME 坐标降落'
        else:
            source = ('共用航点' if step.get('route_source') == 'shared' else
                      LOOP_SECTION_NAMES.get(step.get('route_source'), ''))
            point = '%s第%d点 ' % (source, step['route_index']+1) if source else ''
            text = '%s%s %s，高度 %.2fm' % (point, step['stage'], xy, step['height_m'])
            if step.get('closing'):
                text += '（闭环复用%s首点）' % ('巡航' if source == '巡航闭环' else '绕圈')
        print('%02d. %s：%s' % (index, label, text))
    legs = value.get('fixed_legs')
    if legs:
        gate = legs['gate_velocity_mps']
        print('固定轨迹航段（按 HOME 地面 Z=0、上一停点出发估算；实飞以投放后实际回升位置重算）：')
        for number, leg in enumerate(legs['legs'], 1):
            text = '  航段%d：步骤%02d～%02d → %s，%.1f s，速度极值 %.3f m/s，加速度极值 %.3f m/s²' % (
                number, leg['first_step'], leg['last_step'], '投放' if leg['ends'] == 'drop' else '降落',
                leg['duration_s'], leg['max_velocity_mps'], leg['max_acceleration_mps2'])
            if leg['gate_peak_velocity_mps'] is not None:
                text += '，圆环段峰值 %.3f m/s' % leg['gate_peak_velocity_mps']
            print(text)
            timing_detail = leg.get('local_retiming')
            if timing_detail:
                if timing_detail['applied']:
                    print('    局部时间分配：同一空间曲线 %.2f → %.2f s，沿用现有速度/加速度/圆环上限。' % (
                        timing_detail['baseline_duration_s'], timing_detail['duration_s']))
                    from .local_retiming import describe_dynamics
                    if describe_dynamics(timing_detail):
                        print('    '+describe_dynamics(timing_detail))
                else:
                    print('    局部时间分配未采用，沿用原参考：'+timing_detail.get('diagnostic', timing_detail['reason']))
        print('航段飞行合计 %.1f s（不含起飞、对准、投放、回升、降落）；圆环段速度：%s。' % (
            legs['flight_time_s'], '未单独设置' if gate is None else '%.2f m/s' % gate))
        timing = legs['navigation_timing']
        print('导航时间拆分（仅参考轨迹，不含实际到位等待）：')
        print('  起飞高度到首个航点：%.2f s' % timing['entry_s'])
        for lap in timing['loaded_laps']:
            print('  第%d圈绕圈闭环：%.2f s' % (lap['lap'], lap['duration_s']))
        print('  %s导航：%.2f s；%s：%.2f s' % ('投放环' if value.get('section_points') else '投放圈',
            timing['delivery_navigation_s'], '经返航点回 HOME' if value.get('section_points') else '返 HOME',
            timing['return_home_s']))
        print('  最后一圈导航合计（含返航）：%.2f s；全部导航：%.2f s，不是完整任务耗时。' % (
            timing['delivery_lap_s'], timing['total_navigation_s']))
        print('每段参考到投放点后，XYZ 距离 %.2f m 内持续 %.1f s 才开始投放；%s。' % (
            legs['arrival']['radius_m'], legs['arrival']['dwell_s'], home_arrival_text(legs['arrival'])))
    print('飞行顺序：')
    for line in describe_laps(value['itinerary']):
        print('  '+line)
    for note in value['notes']:
        print(note)


def manager_client():
    # Reuse the existing connection, event reporting and Ctrl-C hold behavior,
    # without importing the old visual mission wrapper.
    path = project_root()/'src/airseafly_real400_mission/scripts/mission_cli.py'
    spec = importlib.util.spec_from_file_location('delivery_manager_client', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main(argv=None):
    arguments = parser()
    args = arguments.parse_args(argv)
    if args.command == 'payload':
        if args.action is None or args.slot is None:
            arguments.error('payload 需要 lock|release 和编号 1～4')
    elif args.action is not None or args.slot is not None:
        arguments.error('动作和编号只用于 payload')
    paths = runtime_paths()
    if args.command in ('status', 'hold'):
        if args.dry_run:
            print(json.dumps(dict(command=args.command, dry_run=True), ensure_ascii=False))
            return 0
        result = request(paths['socket'], dict(command=args.command))
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result.get('ok') else 1
    config = resolve_config(args)
    if args.command == 'run':
        document = load_route(args.route, require_complete=True)
        config['delivery_route'] = document
        show_preview(preview(config, document), args.json)
        # --json keeps stdout pure JSON: only an explicit --preview-html writes there.
        if (args.dry_run and config.get('navigation', {}).get('backend') == 'fixed_min_snap'
                and (args.preview_html or not args.json)):
            from .delivery_route_html import default_preview_path, write_delivery_preview_html
            out = sys.stderr if args.json else sys.stdout
            try:
                path = write_delivery_preview_html(args.preview_html or default_preview_path(args.route),
                                                   config, document)
                print('轨迹预览（浏览器打开）：'+path, file=out)
            except (OSError, ValueError) as exc:
                print('轨迹预览未能生成：'+str(exc), file=out)
        body = dict(command='start', mode='delivery_route', resolved_config=config)
    else:
        command = actuator_command(args.slot, args.action, config['payload'])
        # JSON uses null for the five untouched channels; MAVLink receives NaN
        # inside the existing payload transport, exactly as before.
        command['params'] = [None if i+1 != command['channel'] else command['normalized'] for i in range(6)]
        print(json.dumps(dict(dry_run=args.dry_run, payload=command), ensure_ascii=False, indent=2))
        body = dict(command='payload', action=args.action, slot=args.slot, resolved_config=config)
    if args.dry_run:
        return 0
    client = manager_client()
    client.connect_or_start(paths, config)
    result = request(paths['socket'], body, timeout=float(config['payload']['ack_timeout_s'])+5.)
    if not result.get('ok'):
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 1
    if args.command == 'payload':
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    print(json.dumps(dict(type='started', data=result), ensure_ascii=False))
    return client.watch(paths, args, config, finish_task=True, after=result.get('event_start', 0))
