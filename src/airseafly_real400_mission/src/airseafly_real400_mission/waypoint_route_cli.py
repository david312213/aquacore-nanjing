"""Ordinary recorded routes: continuous fixed laps or the original EGO route."""
import argparse
import json
from pathlib import Path

from .configuration import project_root, runtime_paths
from .flight_configuration import base_configuration, apply_flight_configuration
from .waypoint_route import DEFAULT_ROUTE, load_route_snapshot
from . import native_parameters
from .rpc import request
from .route_profiles import resolve_profiles, point_profiles, entry_points, entry_profiles


def parser():
    result = argparse.ArgumentParser(description='普通航线：固定轨迹连续绕圈后保持首点；可选择原 EGO 逐点航线')
    result.add_argument('command', nargs='?', default='run', choices=('run', 'takeoff', 'hold', 'status'))
    result.add_argument('--route', type=Path, default=project_root()/DEFAULT_ROUTE)
    result.add_argument('--config', help='原机器配置文件')
    result.add_argument('--flight-config', type=Path, default=project_root()/'flight/flight.yaml')
    result.add_argument('--set', dest='overrides', action='append', default=[])
    result.add_argument('--json', action='store_true')
    result.add_argument('--verbose', action='store_true')
    result.add_argument('--timing', choices=('original', 'optimized', 'thrust'),
                        help='固定普通航线配时对比：original 原配时，optimized 使用既有绕圈局部配时算法')
    result.add_argument('--preview-html', type=Path, help='保存可拖动查看的完整轨迹 HTML 预览；不会启动飞行')
    result.add_argument('--preview-origin', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                        help='仅用于离线预览的起飞地面 XYZ；默认首点 XY、Z=0')
    native_parameters.add_arguments(result)
    return result


def resolve_config(args):
    config = apply_flight_configuration(base_configuration(args.config), args.flight_config,
        task='takeoff' if args.command == 'takeoff' else 'waypoint-route',
        overrides=list(args.overrides)+native_parameters.overrides(args))
    if config['flight']['controller'] != 'px4_native':
        raise ValueError('基础试飞入口使用 flight.controller=px4_native，配置不能选择旧控制器')
    if args.command == 'run':
        profile = resolve_profiles(config)['cruise']
        config['flight']['cruise_height_m'] = profile['height_m']
        config['planning']['map']['inflation_m'] = profile['inflation_m']
        if args.timing is not None:
            config.setdefault('navigation', {})['timing'] = args.timing
    native_parameters.validate(config)
    return config


def write_route_preview(route, flight_config, output):
    """Recorder hook: the same fixed-route HTML as fly_points.sh --preview-html."""
    args = parser().parse_args(['--route', str(route), '--flight-config', str(flight_config)])
    config = resolve_config(args)
    if config.get('navigation', {}).get('backend', 'ego') != 'fixed_min_snap':
        raise ValueError('当前 navigation.backend 不是 fixed_min_snap，没有固定轨迹可预览')
    document, source = load_route_snapshot(route, require_points=True)
    config['waypoint_route'] = document
    config['recorded_route_source'] = source
    from .fixed_trajectory_preview import route_preview, write_preview_html
    return write_preview_html(output, route_preview(config, document, None, True))


def preview(config, document, origin=None, include_samples=False):
    profiles = resolve_profiles(config)
    names = point_profiles(document)
    height = profiles['cruise']['height_m']
    parameters = native_parameters.summary(config)
    backend = config.get('navigation', {}).get('backend', 'ego')
    if backend not in ('ego', 'fixed_min_snap'):
        raise ValueError('navigation.backend 必须是 ego 或 fixed_min_snap')
    fixed = backend == 'fixed_min_snap'
    value = dict(mode='waypoint_route', backend=backend, frame=document['frame'], route_points=len(document['points']),
        route_source=config.get('recorded_route_source'),
        virtual_boundary=config.get('_virtual_boundary', {'enabled': False}),
        height_m=height, height_reference='takeoff_origin_z', controller=config['flight']['controller'],
        reference_mode=config['flight']['native_reference_mode'],
        parameters=parameters, finish_action='HOLD', payload=False, visual=False, landing=False,
        arrival=dict(policy='position', metric='xyz', radius_m=config['flight']['arrival_radius_m'],
                     dwell_s=config['flight']['arrival_dwell_s']),
        entry_points=len(entry_points(document)),
        itinerary=[dict(kind='takeoff', height_m=height)] + [
            dict(kind='entry', index=i, xy=list(xy), profile=name, **profiles[name])
            for i, (xy, name) in enumerate(zip(entry_points(document), entry_profiles(document)))] + [
            dict(kind='navigation', index=i, xy=list(xy), profile=names[i], **profiles[names[i]])
            for i, xy in enumerate(document['points'])] + [
            dict(kind='hold', xy=list(document['points'][-1]), height_m=profiles[names[-1]]['height_m'])])
    if fixed:
        from .fixed_trajectory_preview import route_preview
        value['fixed_trajectory'] = route_preview(config, document, origin, include_samples)
        value['arrival'] = dict(policy='trajectory_time', metric='reference', radius_m=None, dwell_s=0.)
        value['itinerary'][-1] = dict(kind='hold', xy=list(document['points'][0]),
                                     height_m=profiles[names[0]]['height_m'])
        value.update(realtime_avoidance=False, ego_map_effective=False,
                     timing_mode=config.get('navigation', {}).get('timing', 'original'),
                     laps=value['fixed_trajectory']['effective_laps'])
    return value


def show_preview(value, as_json=False):
    if as_json:
        print(json.dumps(value, ensure_ascii=False, indent=2))
        return
    if value.get('route_source'):
        print('本次航点文件：'+value['route_source']['path']+'；SHA256='+value['route_source']['sha256'])
    fixed = value.get('backend') == 'fixed_min_snap'
    entry = ('经 %d 个进场点 → ' % value['entry_points']) if value.get('entry_points') else ''
    if fixed:
        print('起飞 %.2f m → %s连续经过 %d 个闭环航点，闭环 %d 圈 → 首点悬停。' % (
            value['height_m'], entry, value['route_points'], value['laps']))
    else:
        print('起飞 %.2f m → %sEGO 依次到达 %d 个普通航点 → 末点悬停。' % (value['height_m'], entry, value['route_points']))
    print('全部高度使用同一本次起飞基准；XY 来自同次定位，录点时手持高度不参与飞行。')
    boundary = value.get('virtual_boundary', {})
    if fixed:
        print('固定轨迹：不实时绕障；EGO 地图膨胀、高度边界和虚拟围墙不约束飞行。')
        if boundary.get('corners'):
            print('边界仅作预览对照：'+str(boundary['corners']))
        if boundary.get('diagnostic'):
            print('边界预览诊断：'+boundary['diagnostic'])
    elif boundary.get('enabled'):
        print('虚拟围墙：'+boundary['source']['path']+'；四角='+str(boundary['corners'])+'；高度范围（相对起飞点）='+str((boundary['bottom_m'], boundary['top_m']))+'；null 表示不封顶/底')
    else:
        print('虚拟围墙：关闭')
    p = value['parameters']
    print('起飞 %.2f m/s / %.2f m/s²；航线 %.2f m/s / %.2f m/s²；EGO 地图上限 %.2f m。' % (
        p['takeoff_speed_mps'], p['takeoff_acceleration_mps2'], p['ego_speed_mps'],
        p['ego_acceleration_mps2'], p['map']['max_height_m']))
    for step in value['itinerary']:
        if step['kind'] == 'entry':
            print('  进场点%d：%s X=%+.3f Y=%+.3f，高度 %.2f m%s' % (step['index']+1, step['profile'], *step['xy'], step['height_m'],
                '' if fixed else '，膨胀 %.2f m' % step['inflation_m']))
        elif step['kind'] == 'navigation':
            print('  点%d：%s X=%+.3f Y=%+.3f，高度 %.2f m%s' % (step['index']+1, step['profile'], *step['xy'], step['height_m'],
                '' if fixed else '，膨胀 %.2f m' % step['inflation_m']))
    if fixed:
        t = value['fixed_trajectory']
        from .local_retiming import TIMING_LABELS
        print('配时版本：%s（navigation.timing=%s）' % (
            TIMING_LABELS.get(value.get('timing_mode'), value.get('timing_mode')), value.get('timing_mode')))
        if value.get('timing_mode') in ('optimized', 'thrust'):
            detail = t.get('local_retiming') or {}
            if detail.get('applied'):
                print('局部配时已采用：原导航参考 %.3f s → 优化后 %.3f s；空间曲线和速度/加速度上限不变。' % (
                    detail['baseline_duration_s'], t['duration_s']))
                from .local_retiming import describe_dynamics
                if describe_dynamics(detail):
                    print(describe_dynamics(detail))
            else:
                print('局部配时未采用，沿用原参考：'+str(detail.get('reason', '无可优化的运动段')))
                if detail.get('diagnostic'):print('配时说明：'+str(detail['diagnostic']))
        else:
            print('当前为原配时，未启用局部时间分配优化。')
        print(t['origin_note'])
        print('参考总时长 %.2f s；速度极值 %.3f m/s，加速度极值 %.3f m/s²%s。' % (
            t['duration_s'], t['max_velocity_mps'], t['max_acceleration_mps2'],
            '（上界）' if t.get('extrema') == 'upper_bound' else ''))
        if not t.get('gate_points'):
            print('全部为普通点，未应用过门限速。')
        elif t.get('gate_velocity_mps') is None:
            print('过门速度：未单独设置，全程同一航线速度。')
        elif not t.get('time_warp'):
            print('过门速度 %.2f m/s（%d 个过门点）：过门处原本就不超过该速度，轨迹不变。' % (
                t['gate_velocity_mps'], t['gate_points']))
        else:
            print('过门速度 %.2f m/s（%d 个过门点，%d 段减速区）：过门处峰值 %.3f m/s；曲线不变，'
                  '只在门前减速、门后加速；未过门时长 %.2f s。' % (
                t['gate_velocity_mps'], t['gate_points'], len(t['gate_windows']),
                t['limited_peak_velocity_mps'], t['base_duration_s']))
        print('普通点和圈间连续通过；按参考时间推进，无逐点到位等待。不起视觉、不投放、不自动降落。')
        if value.get('preview_html'):
            print('完整轨迹预览：'+value['preview_html'])
    else:
        print('到点沿用 XYZ 距离 %.2f m / 持续 %.2f s；不起视觉、不投放、不自动返航或降落。' % (
            value['arrival']['radius_m'], value['arrival']['dwell_s']))


def takeoff_preview(config, dry_run=False):
    return dict(dry_run=dry_run, command='takeoff', mode='basic_takeoff',
        controller=config['flight']['controller'], parameters=native_parameters.summary(config),
        payload_sequence=[], stages=['takeoff', 'hold'], finish_action='HOLD',
        reference_stream_after_completion=True, payload=False, visual=False, landing=False)


def manager_client():
    # The existing RPC client owns connection, status and scoped Ctrl-C hold.
    import importlib.util
    path = project_root()/'src/airseafly_real400_mission/scripts/mission_cli.py'
    spec = importlib.util.spec_from_file_location('waypoint_manager_client', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main(argv=None):
    args = parser().parse_args(argv)
    paths = runtime_paths()
    if args.command in ('hold', 'status'):
        if args.dry_run:
            print(json.dumps(dict(command=args.command, dry_run=True)))
            return 0
        result = request(paths['socket'], dict(command=args.command))
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result.get('ok') else 1
    config = resolve_config(args)
    if args.command == 'takeoff':
        value = takeoff_preview(config, args.dry_run)
        if args.json:
            print(json.dumps(value, ensure_ascii=False, indent=2))
        else:
            print('原地起飞至本次起飞基准 + %.2f m 后持续悬停；不执行航点、视觉或投放。' %
                  config['flight']['cruise_height_m'])
            print('本次参数：'+json.dumps(value['parameters'], ensure_ascii=False))
        mode = 'basic_takeoff'
    else:
        document, source = load_route_snapshot(args.route, require_points=True)
        config['waypoint_route'] = document
        config['recorded_route_source'] = source
        value = preview(config, document, args.preview_origin, include_samples=bool(args.preview_html))
        if args.preview_html:
            if value['backend'] != 'fixed_min_snap':
                raise ValueError('--preview-html 用于 fixed_min_snap 固定轨迹')
            from .fixed_trajectory_preview import write_preview_html
            value['preview_html'] = write_preview_html(args.preview_html, value['fixed_trajectory'])
            value['fixed_trajectory'].pop('samples', None)
        show_preview(value, args.json)
        mode = 'waypoint_route'
    if args.dry_run or args.preview_html:
        return 0
    client = manager_client()
    client.connect_or_start(paths, config)
    result = request(paths['socket'], dict(command='start', mode=mode, resolved_config=config),
                     timeout=float(config['payload']['ack_timeout_s'])+5.)
    if not result.get('ok'):
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 1
    print(json.dumps(dict(type='started', data=result), ensure_ascii=False))
    return client.watch(paths, args, config, finish_task=True, after=result.get('event_start', 0),
                        expected_task_id=result.get('task_id'))
