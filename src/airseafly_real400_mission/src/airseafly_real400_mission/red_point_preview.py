"""Standalone DF200 observation UI. ROS imports are confined to live mode."""
import argparse
import json
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty

import cv2
import numpy as np

from .red_point import RedPointSession, annotate, detect_targets as detect
# Re-export the existing UI-facing names for callers and observation tests.
from .red_point_runtime import (
    PROJECT, DEFAULT_CONFIG, CAMERA_LAUNCH, load_config, CameraOwner,
    ProducerIdentity, PreviewInputs, attach_ros, process_image,
)


WINDOW = 'DF200 red point - q: quit'


class ConsoleReporter:
    """Throttle terminal text only; never detection, projection or ROS output."""
    def __init__(self, rate_hz, confirmed_only=False, target_kind="red", show_pixels=False):
        self.interval = 1. / rate_hz
        self.confirmed_only = confirmed_only
        self.target_kind = target_kind
        self.show_pixels = show_pixels
        self.next_print = 0.
        self.previous_state = None

    def observe(self, status, now):
        if self.target_kind == 'green':
            status = dict(status, candidates=status.get('green_candidates', []))
        if self.confirmed_only:
            confirmed = [candidate for candidate in status['candidates']
                         if candidate.get('target_confirmed', False)]
            if not confirmed:
                self.previous_state = None
                return None
            status = dict(status, candidates=confirmed)
        state = (len(status['candidates']), status['projection_status'],
                 status['localization_epoch'], status.get('image_error', ''),
                 tuple(sorted((bool(c.get('target_confirmed', False)),
                               c.get('confirmation_reason', 'unconfirmed'))
                              for c in status['candidates'])))
        changed = state != self.previous_state
        self.previous_state = state
        if not changed and now < self.next_print:
            return None
        self.next_print = now + self.interval
        return self.format_confirmed(status, self.show_pixels) if self.confirmed_only else self.format(status)

    @staticmethod
    def format_confirmed(status, show_pixels=False):
        def coordinates(value):
            return '不可计算' if value is None else '('+', '.join('{:.3f}'.format(v) for v in value)+')m'
        pose = status['pose']
        aircraft = coordinates(pose['position'] if pose is not None else None)
        lines = []
        for index, candidate in enumerate(status['candidates'], 1):
            kind = '绿色估算圆心' if candidate.get('class_name') == 'green_circle' else '红点'
            line = '飞机XYZ={} | {}{}中心XY={} | ΔXY={}'.format(
                aircraft, kind, index, coordinates(candidate['xy_m']), coordinates(candidate['offset_xy_m']))
            if show_pixels:
                line += ' | 像素=({:.1f}, {:.1f})'.format(*candidate['pixel'])
            lines.append(line)
        return '\n'.join(lines)

    @staticmethod
    def format(status):
        pose = status['pose']
        height = status['camera_height_m']
        height_text = '不可计算' if height is None else '{:.3f}m'.format(height)
        timing = ('无定位' if pose is None else
                  'Δt={:+.1f}ms，最近样本Δt={:+.1f}ms（{}）'.format(
                      pose['dt_s']*1000, pose['nearest_dt_s']*1000, pose['method']))
        lines = ['图像t={:.6f} | 坐标系={} | 高度={} | {} | {}'.format(
            status['image_stamp_s'], status['frame_id'], height_text, timing, status['projection_status'])]
        if pose is not None:
            lines.append('飞机XYZ(图像时刻)=({:.3f}, {:.3f}, {:.3f})m | 横滚/俯仰/航向=({:.1f}, {:.1f}, {:.1f})°'.format(
                *pose['position'], *pose['rpy_deg']))
        lines.append('检测耗时={:.1f}ms'.format(status['detection_ms']))
        if not status['candidates']:
            lines.append('未检测到红点' if not status.get('image_error') else '图像处理异常：无红点结果')
        for index, candidate in enumerate(status['candidates'], 1):
            error, tolerance = candidate.get('center_error_px'), candidate.get('center_tolerance_px')
            centers = (' | 共心误差={:.2f}px / 容许={:.2f}px'.format(error, tolerance)
                       if error is not None and tolerance is not None else '')
            if not candidate.get('target_confirmed', False):
                reasons = dict(missing_green_outer='缺少可拟合的绿环外缘',
                               missing_white_inner='缺少可拟合的中央白圆',
                               invalid_nesting='白圆与绿环的包含关系或尺寸比例不符',
                               not_concentric='三个中心偏差过大')
                reason = candidate.get('confirmation_reason', 'unconfirmed')
                lines.append('红点 C{} | 像素=({:.1f}, {:.1f}) | 未确认：{}{}'.format(
                    index, *candidate['pixel'], reasons.get(reason, reason), centers))
                continue
            xy, offset = candidate['xy_m'], candidate['offset_xy_m']
            xy_text = '不可计算' if xy is None else '({:.3f}, {:.3f})m'.format(*xy)
            offset_text = '不可计算' if offset is None else '({:+.3f}, {:+.3f})m'.format(*offset)
            lines.append('红点 C{} | 已确认 | 像素=({:.1f}, {:.1f}) | 目标XY={} | 相对飞机ΔXY={}{}'.format(
                index, *candidate['pixel'], xy_text, offset_text, centers))
        return '\n'.join(lines)


class TerminalKeys:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd) if os.isatty(self.fd) else None
        if self.saved is not None:
            tty.setcbreak(self.fd)
        return self

    def read(self):
        if select.select([self.fd], [], [], 0)[0]:
            return os.read(self.fd, 1).decode('ascii', errors='ignore').lower()
        return ''

    def __exit__(self, *_):
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)


def open_window(disabled):
    if disabled or (sys.platform.startswith('linux') and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))):
        return False
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 1280, 720)
        waiting = np.zeros((720, 1280, 3), np.uint8)
        cv2.putText(waiting, 'WAITING FOR IMAGE', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 160, 255), 2)
        cv2.imshow(WINDOW, waiting)
        return True
    except cv2.error:
        print('没有可用窗口：终端自动打印坐标，按 q 退出；结果仍发布到 ROS 观察话题。', file=sys.stderr)
        return False


def window_is_open():
    # OpenCV 4.2/GTK returns -1 for WND_PROP_VISIBLE even for an open window.
    # AUTOSIZE is supported there: 0/1 describes an existing window, while -1
    # (or an exception on other backends) means that the window was destroyed.
    try:
        return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_AUTOSIZE) >= 0
    except cv2.error:
        return False


def select_preview_targets(status, target):
    """Filter display observations only; no mission target or flight commands."""
    selected = dict(status)
    if target == 'green':
        selected.update(candidates=[], confirmed_count=0)
    elif target == 'red':
        selected.update(green_candidates=[], green_count=0)
    return selected


def offline(args, config):
    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('无法读取图片：'+str(args.image))
    candidates, elapsed = detect(image, config)
    # The supplied photograph is not a calibrated DF200 frame. Do not invent
    # image timestamps, pose, principal point or metric coordinates for it.
    status = RedPointSession(config).describe(candidates, image.shape[1::-1], 0., elapsed, offline=True)
    status = select_preview_targets(status, getattr(args, 'target', 'both'))
    status['source_image'] = str(Path(args.image).resolve())
    canvas = annotate(image, status)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output), canvas):
            raise ValueError('无法写入标注图片：'+str(output))
    if args.status_json:
        output = Path(args.status_json)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(status, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(status, ensure_ascii=False, indent=2, allow_nan=False))
    if open_window(args.no_window):
        try:
            cv2.imshow(WINDOW, canvas)
            while window_is_open():
                if cv2.waitKey(30) & 0xff in (ord('q'), 27):
                    break
        finally:
            cv2.destroyWindow(WINDOW)
    return 0


def live(args, config):
    import rospy
    import rosgraph
    import roslaunch
    from cv_bridge import CvBridge, CvBridgeError
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    rospy.init_node('red_point_preview', anonymous=True)
    master = rosgraph.Master(rospy.get_name())
    bridge, inputs = CvBridge(), PreviewInputs(RedPointSession(config))
    identity = ProducerIdentity([config['topics']['raw_odometry'], config['topics']['odometry']])
    # Check master before constructing ROSLaunchParent so this entry never
    # creates its own sensor/localization stack or a different ROS master.
    identity.changed(master)

    def launch_factory(path, arguments):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        return roslaunch.parent.ROSLaunchParent(uuid, [(path, arguments)])

    owner = CameraOwner(config, master, launch_factory, rospy.get_name()+'/camera')
    publishers, subscribers = attach_ros(rospy, locals(), config, inputs)
    window = open_window(args.no_window)
    last_message, next_identity = None, 0.
    target = getattr(args, 'target', 'both')
    console = ConsoleReporter(config['console']['rate_hz'], getattr(args, 'confirmed_only', False),
                              target_kind=target, show_pixels=target != 'both')
    displayed_epoch = None
    try:
        print('红点独立预览：只发布观察结果。采用近水平投影，横滚/俯仰仅显示，不做全姿态补偿。', flush=True)
        print('自动使用本次定位零点 Z=0：定位须在靶面所在的地面初始化；相机高度=定位 Z+{:.3f}m。'.format(
            config['projection']['camera_height_at_reference_m']), flush=True)
        print('自动打印坐标（{} 次/秒）；C1/C2 仅为本帧候选顺序。窗口或终端按 q 退出。'.format(
            config['console']['rate_hz']), flush=True)
        print('手持测试：绿色估算圆心，无需红点；约 1 米处观察。' if target == 'green' else
              '手持测试：绿环、白圆包围的小红点；约 0.5 米处观察。' if target == 'red' else
              '预览两类圆心；终端默认显示红点结果。', flush=True)
        if target != 'both':
            print('高度只作手持参考，不触发动作；没有定位时仍显示像素，米制坐标显示不可计算。', flush=True)
        print('DF200：'+owner.start(), flush=True)
        with TerminalKeys() as keys:
            while not rospy.is_shutdown():
                key = keys.read()
                if window:
                    code = cv2.waitKey(1) & 0xff
                    if code != 255:
                        key = chr(code).lower()
                    if not window_is_open():
                        print('预览窗口已关闭，正在退出红点预览。', flush=True)
                        break
                if key in ('q', '\x03', '\x1b'):
                    print('收到退出按键，正在退出红点预览。', flush=True)
                    break
                now = time.monotonic()
                if now >= next_identity:
                    try:
                        if identity.changed(master):
                            with inputs.lock:
                                inputs.session.reset_localization('localization_producer_restarted')
                            print('定位发布进程已变化：清除旧定位样本，继续以本次定位零点 Z=0 换算高度。', flush=True)
                    except (OSError, rosgraph.MasterError, rosgraph.MasterFailure) as exc:
                        rospy.logwarn_throttle(5., 'Cannot observe localization producer identity: %s', exc)
                    next_identity = now+1.  # Observation polling, never an age gate.
                message = inputs.take_image()
                if message is not None:
                    last_message = message
                with inputs.lock:
                    if displayed_epoch != inputs.session.epoch:
                        message = last_message
                if message is not None:
                    image, status = process_image(message, bridge, config, inputs,
                        owner.mode, rospy.get_time(), detector=detect,
                        errors=(CvBridgeError, cv2.error, ValueError, TypeError))
                    status = select_preview_targets(status, target)
                    displayed_epoch = status['localization_epoch']
                    publishers[0].publish(String(data=json.dumps(status, ensure_ascii=False, allow_nan=False)))
                    image_subscribed = publishers[1].get_num_connections() > 0
                    if window or image_subscribed:
                        canvas = annotate(image, status)
                        if image_subscribed:
                            annotated = bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
                            annotated.header = message.header  # Preserve source time, not processing time.
                            publishers[1].publish(annotated)
                        if window:
                            cv2.imshow(WINDOW, canvas)
                    output = console.observe(status, now)
                    if output is not None:
                        print(output, flush=True)
                # UI polling only; no pose bracketing, settling or retry wait.
                if message is None:
                    time.sleep(.01)
    finally:
        owner.close()
        for subscriber in subscribers:
            subscriber.unregister()
        if window:
            cv2.destroyWindow(WINDOW)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='DF200 红点像素与局部 XY 预览；无飞行/投放动作。')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--image', type=Path, help='离线图片：只识别像素，不假设相机/定位')
    parser.add_argument('--output', type=Path, help='离线标注图片输出')
    parser.add_argument('--status-json', type=Path, help='离线观察结果 JSON 输出')
    parser.add_argument('--no-window', action='store_true', help='不显示窗口；终端自动打印坐标，按 q 退出')
    parser.add_argument('--confirmed-only', action='store_true',
                        help='仅打印绿环、白圆、红点共心确认后的飞机与靶心坐标；未确认时不打印')
    parser.add_argument('--target', choices=('both', 'green', 'red'), default='both',
                        help='只观察指定圆心；green=第一次，red=第二次，均不控制飞机')
    parser.add_argument('--print-hz', type=float, help='终端打印频率；不改变识别频率')
    parser.add_argument('--dry-run', action='store_true', help='只显示生效配置，不连接 ROS/相机')
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        if args.print_hz is not None:
            if not np.isfinite(args.print_hz) or args.print_hz <= 0:
                raise ValueError('--print-hz 必须为正数')
            config['console']['rate_hz'] = args.print_hz
        if args.dry_run:
            print(json.dumps(dict(dry_run=True, config_path=str(args.config.resolve()), config=config,
                confirmed_only=args.confirmed_only, target=args.target,
                camera_launch=str(CAMERA_LAUNCH), publishes=[config['topics'][key] for key in ('status', 'annotated')],
                flight_commands=False, payload_commands=False, persistent_ground_reference=False,
                height_reference=dict(source='localization_origin', z_m=0.),
                height_formula='planning_odometry.z + camera_height_at_reference_m'),
                ensure_ascii=False, indent=2, allow_nan=False))
            return 0
        if args.image:
            return offline(args, config)
        if args.output or args.status_json:
            parser.error('--output 和 --status-json 用于 --image 离线模式')
        return live(args, config)
    except (OSError, ValueError, KeyError, ImportError) as exc:
        print('红点预览：'+str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
