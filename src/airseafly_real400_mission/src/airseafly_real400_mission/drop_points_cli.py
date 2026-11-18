"""Preview or run one to four recorded deliveries with the existing manager."""
import argparse
from copy import deepcopy
import json
from pathlib import Path

from .configuration import project_root, runtime_paths
from .delivery_route_cli import resolve_config, manager_client, navigation_line, navigation_summary
from .drop_points import select_points
from .waypoint_route import DEFAULT_ROUTE, load_route_snapshot
from .native_drop_points import build_drop_points_itinerary
from .payload import actuator_command
from . import native_parameters
from .rpc import request


def parser():
    result = argparse.ArgumentParser(description='起飞 → 依次前往录点、下降识别红点对准一次、下降投放 → 回起飞点降落')
    result.add_argument('--route', type=Path, default=project_root()/DEFAULT_ROUTE)
    result.add_argument('--count', type=int, choices=(1, 2, 3, 4), help='只投前 N 个点；默认全部已录点')
    result.add_argument('--config', help='原机配置文件')
    result.add_argument('--flight-config', type=Path, default=project_root()/'flight/flight.yaml')
    result.add_argument('--set', dest='overrides', action='append', default=[])
    result.add_argument('--json', action='store_true')
    result.add_argument('--verbose', action='store_true')
    native_parameters.add_arguments(result)
    return result


def preview(config):
    document = select_points(config['drop_points'])
    targets = []
    for index, point in enumerate(document['points']):
        slot = index+1
        command = actuator_command(slot, 'release', config['payload'])
        targets.append(dict(point=slot, xy=list(point), slot=slot,
            main_output=command['main_output'], pwm_us=command['pwm_us'],
            servo_stage=1 if slot <= 2 else 2,
            mount_offset_body_m=deepcopy(config['delivery_route_options']['mount_offsets_body_m'][str(slot)])))
    return dict(mode='drop_points', frame=document['frame'], targets=targets, payload_total=len(targets),
        route_source=config.get('recorded_route_source'),
        heights_relative_to_takeoff_m=dict(cruise=config['flight']['cruise_height_m'],
                                          drop=config['flight']['drop_height_m']),
        home_source='takeoff_origin_xy', home_xy=None, finish_action='return_home_and_land',
        ignored_entry_points=len(document.get('entry_points', [])),
        itinerary=build_drop_points_itinerary(document), navigation=navigation_summary(config),
        delivery_motion=native_parameters.delivery_motion(config),
        visual_alignment=deepcopy(config['delivery_route_options']['visual_alignment']))


def show_preview(value, as_json=False):
    if as_json:
        print(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))
        return
    heights = value['heights_relative_to_takeoff_m']
    if value.get('route_source'):
        print('本次航点文件：'+value['route_source']['path']+'；SHA256='+value['route_source']['sha256'])
    print('录点投放测试：起飞/巡航/回升 %.2fm，下降投放 %.2fm；共 %d 点，最后回本次起飞点降落。' %
          (heights['cruise'], heights['drop'], value['payload_total']))
    if value.get('ignored_entry_points'):
        print('提示：航点文件另有 %d 个进场点（record_points.sh 录制），投放测试不使用，只按下列点投放。' %
              value['ignored_entry_points'])
    print(native_parameters.delivery_motion_line(value['delivery_motion']))
    for target in value['targets']:
        print('点%d XY=(%+.3f, %+.3f) → 下降至识别高度 → 红点对准 → 下降 → MAIN%d 第%d档 %g μs → 回升' %
              (target['point'], *target['xy'], target['main_output'], target['servo_stage'], target['pwm_us']))
    vision = value['visual_alignment']
    print('中间红点识别高度：定位 Z=%.2fm；中间下降超时保持实测 XYZ 继续识别。' % vision['intermediate_height_m'])
    print('取图每次最多 %gs，参考运动结束后宽限 %gs，每点最多额外重拍 %d 次。' %
          (vision['frame_timeout_s'], vision['motion_timeout_margin_s'], vision['retake_limit']))
    print('识别超时沿最后视觉目标或录点下降；下降超时按实测位置和高度释放；每点只发一次并等真实回执。')
    print(navigation_line(value['navigation']))
    print('录点与飞行使用同一次定位；不自动复位舵机，返航点使用实际起飞位置。')


def main(argv=None):
    args = parser().parse_args(argv)
    recorded, source = load_route_snapshot(args.route, require_points=True)
    document = select_points(recorded, args.count)
    config = resolve_config(args)
    config['drop_points'] = document
    config['recorded_route_source'] = source
    show_preview(preview(config), args.json)
    if args.dry_run:
        return 0
    paths = runtime_paths()
    client = manager_client()
    client.connect_or_start(paths, config)
    result = request(paths['socket'], dict(command='start', mode='drop_points', resolved_config=config),
                     timeout=float(config['payload']['ack_timeout_s'])+5.)
    if not result.get('ok'):
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 1
    print(json.dumps(dict(type='started', data=result), ensure_ascii=False))
    return client.watch(paths, args, config, finish_task=True, after=result.get('event_start', 0),
                        expected_task_id=result.get('task_id'))
