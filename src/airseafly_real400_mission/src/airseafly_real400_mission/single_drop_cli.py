"""One target and one selected payload stage, followed by return and landing."""
import argparse
from copy import deepcopy
import json
from pathlib import Path

from .configuration import project_root, runtime_paths
from .delivery_route_cli import resolve_config, manager_client, navigation_line, navigation_summary
from .native_single_drop import validate_drop_test, build_single_drop_itinerary
from . import native_parameters
from .payload import actuator_command
from .rpc import request


DEFAULT_POINT = 'data/routes/single_drop_point.yaml'


def parser():
    result = argparse.ArgumentParser(description=
        '单点投放测试：起飞 → 指定XY → 下降至识别高度 → 红点对准 → 指定投放口释放 → 回升 → 回起飞点降落')
    result.add_argument('--x', type=float, required=True, help='当前定位坐标系的目标 X，m；不是相对位移')
    result.add_argument('--y', type=float, required=True, help='当前定位坐标系的目标 Y，m；不是相对位移')
    result.add_argument('--slot', type=int, choices=(1, 2, 3, 4), required=True,
        help='只释放此编号：1=MAIN6第一档，2=MAIN7第一档，3=MAIN6第二档，4=MAIN7第二档')
    result.add_argument('--config', help='原机配置文件')
    result.add_argument('--flight-config', type=Path, default=project_root()/'flight/flight.yaml')
    result.add_argument('--set', dest='overrides', action='append', default=[])
    result.add_argument('--json', action='store_true')
    result.add_argument('--verbose', action='store_true')
    native_parameters.add_arguments(result)
    return result


def preview(config):
    target = validate_drop_test(config['drop_test'])
    hover = target['finish'] == 'hover'
    command = actuator_command(target['slot'], 'release', config['payload'])
    height, drop = config['flight']['cruise_height_m'], config['flight']['drop_height_m']
    return dict(mode='drop_test', frame='fast_lio_local_enu', target_xy=target['target_xy'],
        home_source=None if hover else 'takeoff_origin_xy', home_xy=None,
        finish_action='hover_after_recovery' if hover else 'return_home_and_land',
        point_source=config.get('single_drop_point_source'),
        config_profile='delivery-route', heights_relative_to_takeoff_m=dict(cruise=height, drop=drop),
        payload_total=1, selected_payload=dict(slot=target['slot'],
            main_output=command['main_output'], channel=command['channel'], pwm_us=command['pwm_us'],
            servo_stage=1 if target['slot'] <= 2 else 2),
        mount_offset_body_m=deepcopy(config['delivery_route_options']['mount_offsets_body_m'][str(target['slot'])]),
        visual_alignment=deepcopy(config['delivery_route_options']['visual_alignment']),
        recognition_fallback_xy='last_visual_target_or_requested_xy',
        itinerary=build_single_drop_itinerary(target['target_xy'], target['slot'], finish=target['finish']),
        navigation=navigation_summary(config), delivery_motion=native_parameters.delivery_motion(config),
        stages=['takeoff', 'navigate_target', 'first_visual_correction', 'second_visual_correction',
                'descend', 'release_once', 'ack_and_settle', 'recover']+(
                    ['hover'] if hover else ['return_home', 'land']))


def show_preview(value, as_json=False):
    if as_json:
        print(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))
        return
    heights, payload, vision = (value[key] for key in
                               ('heights_relative_to_takeoff_m', 'selected_payload', 'visual_alignment'))
    hover = value['finish_action'] == 'hover_after_recovery'
    if value.get('point_source'):
        print('投放点文件：%s；SHA256=%s' % (value['point_source']['path'], value['point_source']['sha256']))
    print('单点测试：起飞 %.2fm → X=%+.3f Y=%+.3f → 下降至识别高度 → 红点对准 → 下降至 %.2fm。' %
          (heights['cruise'], *value['target_xy'], heights['drop']))
    print('仅投放编号 %d：MAIN%d 第%d档，%g μs；成功回执及原有动作等待后回升，%s。' %
          (payload['slot'], payload['main_output'], payload['servo_stage'], payload['pwm_us'],
           '回升到 %.2fm 后在投放点上方持续悬停，不返航、不降落' % heights['cruise'] if hover else '回起飞点降落'))
    if hover:
        print('XY 是当前定位坐标，不是相对位移；结束后用 sh flight/safe_land.sh 降落，或 sh flight/hold.sh 停止任务。')
    else:
        print('XY 是当前定位坐标，不是相对位移；返航点使用本次实际起飞位置，不需要录制比赛航线。')
    print(native_parameters.delivery_motion_line(value['delivery_motion']))
    line = navigation_line(value['navigation'])
    print(line.split('；降落点')[0]+'。' if hover and '；降落点' in line else line)
    print('中间红点识别高度：定位 Z=%.2fm；下降超时保持实测 XYZ 继续识别。' % vision['intermediate_height_m'])
    print('沿用 delivery-route 参数：取图最多 %gs，运动结束后宽限 %gs，每点额外重拍 %d 次。' %
          (vision['frame_timeout_s'], vision['motion_timeout_margin_s'], vision['retake_limit']))
    print('识别超时仍下降投放；没有视觉修正时使用指定XY，下降超时按实测高度释放；偏移[前,左]=%s m。' %
          value['mount_offset_body_m'])


def main(argv=None):
    args = parser().parse_args(argv)
    config = resolve_config(args)
    config['drop_test'] = validate_drop_test(dict(target_xy=[args.x, args.y], slot=args.slot))
    return run(config, args)


def recorded_parser():
    result = argparse.ArgumentParser(description=
        '录点单投放：起飞 → 录好的投放点 → 下降至识别高度 → 红点对准 → 指定投放口释放 → 回升 → 原地悬停')
    result.add_argument('--point', type=Path, default=project_root()/DEFAULT_POINT,
        help='record_single_drop.sh 录的投放点文件，默认 '+DEFAULT_POINT)
    result.add_argument('--slot', type=int, choices=(1, 2, 3, 4), default=1,
        help='投放编号，默认 1：1=MAIN6第一档，2=MAIN7第一档，3=MAIN6第二档，4=MAIN7第二档')
    result.add_argument('--config', help='原机配置文件')
    result.add_argument('--flight-config', type=Path, default=project_root()/'flight/flight.yaml')
    result.add_argument('--set', dest='overrides', action='append', default=[])
    result.add_argument('--json', action='store_true')
    result.add_argument('--verbose', action='store_true')
    native_parameters.add_arguments(result)
    return result


def load_single_point(path):
    """The one recorded XY and the exact bytes' digest shown before flight."""
    from .waypoint_route import load_route_snapshot
    document, source = load_route_snapshot(path, require_points=True)
    if len(document['points']) != 1:
        raise ValueError('单投放点文件应只有 1 个点，当前 %d 个；用 record_single_drop.sh 重新录' % len(document['points']))
    return document['points'][0], source


def recorded_main(argv=None):
    args = recorded_parser().parse_args(argv)
    config = resolve_config(args)
    target, source = load_single_point(args.point)
    config['single_drop_point_source'] = source
    config['drop_test'] = validate_drop_test(dict(target_xy=target, slot=args.slot, finish='hover'))
    return run(config, args)


def run(config, args):
    show_preview(preview(config), args.json)
    if args.dry_run:
        return 0
    paths = runtime_paths()
    client = manager_client()
    client.connect_or_start(paths, config)
    result = request(paths['socket'], dict(command='start', mode='drop_test', resolved_config=config),
                     timeout=float(config['payload']['ack_timeout_s'])+5.)
    if not result.get('ok'):
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 1
    print(json.dumps(dict(type='started', data=result), ensure_ascii=False))
    return client.watch(paths, args, config, finish_task=True, after=result.get('event_start', 0),
                        expected_task_id=result.get('task_id'))
