#!/usr/bin/env python3
"""Record ordinary XY waypoints from planning_odometry; never command a UAV."""
import argparse
import os
from pathlib import Path
import select
import signal
import sys
import termios
import threading
import tty

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from airseafly_real400_mission.guided_route import RecorderKeyDecoder
from airseafly_real400_mission.waypoint_route import DEFAULT_ROUTE, WaypointRecorder, load_route, save_route


PROJECT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = PROJECT/DEFAULT_ROUTE
TOPIC = '/airseafly/native/planning_odometry'
HELP = '''普通航点录制：先录从起飞点进入闭环的进场点，再录闭环航点，按希望飞行的顺序走到各个位置。
进场点：引导录 3 个，录满自动进入闭环；Enter 可提前进入闭环。进场点只飞一次，之后绕闭环 navigation.laps 圈。
1：选择巡航点；2：选择过门点；随后空格记录最新 X/Y 和类型，可连续录任意数量航点。
X：删除最后记录的航点。
Q / Ctrl+C：保存并退出；没有航点也可以保存。
只读定位，不控制飞机或投放；高度在飞行配置中设置。
普通飞行和 1～4 点投放共用此文件；比赛九步录点仍单独保存。'''


class RecorderSession:
    def __init__(self, recorder, delivery=False, profiles=None, boundary=False, single=False):
        self.recorder = recorder
        self.delivery = delivery
        self.single = single
        self.boundary = boundary
        self.profiles = profiles or {}
        self.lock = threading.Lock()

    def observe_xy(self, x, y):
        with self.lock:
            self.recorder.observe_xy(x, y)

    def on_odometry(self, message):
        position = message.pose.pose.position
        try:
            self.observe_xy(position.x, position.y)
        except ValueError:
            with self.lock:
                self.recorder.latest_xy = None

    def key(self, key):
        with self.lock:
            return self.recorder.key(key)

    def document(self):
        with self.lock:
            return self.recorder.to_dict()

    def status(self):
        with self.lock:
            xy = self.recorder.latest_xy
            count = len(self.recorder.document['points'])
            position = '未收到坐标' if xy is None else 'XY=(%+.3f, %+.3f)米' % tuple(xy)
            if self.boundary:
                return '场地四角：已录 %d/4｜%s｜沿边依次录；空格记录，X 撤销，Q 保存' % (count, position)
            if self.single:
                point = self.recorder.document['points']
                recorded = ('已录 X=%+.3f Y=%+.3f' % tuple(point[0])) if point else '未录'
                return '单投放点：%s｜%s｜空格记录（再按覆盖），X 撤销，Q 保存退出' % (recorded, position)
            if self.delivery:
                mapping = ('MAIN6 第一档', 'MAIN7 第一档', 'MAIN6 第二档', 'MAIN7 第二档')
                next_point = ('下一点 %d → %s' % (count+1, mapping[count])) if count < 4 else '四点已录满'
                return '已记录 %d 个投放点｜%s｜%s｜空格记录，X 撤销，Q 保存退出' % (count, next_point, position)
            if getattr(self.recorder, 'sections', False):
                document = self.recorder.document
                section = ('进场点 %d/3（录满自动进入闭环，Enter 提前结束）' % (len(document['entry_points'])+1)
                           if self.recorder.section == 'entry' else '闭环航点 %d' % (count+1))
                label = '过门' if self.recorder.selected_profile == 'gate' else '巡航'
                return '进场 %d 点｜闭环 %d 点｜下一点：%s｜%s｜当前：%s｜1 巡航 / 2 过门，空格记录，X 撤销，Q 保存退出' % (
                    len(document['entry_points']), count, section, position, label)
            label = '过门' if self.recorder.selected_profile == 'gate' else '巡航'
            selected = self.profiles.get(self.recorder.selected_profile)
            if selected:
                label += ' 高度 %.2fm / 膨胀 %.2fm' % (selected['height_m'], selected['inflation_m'])
            return '已记录 %d 个航点｜下一点 %d｜%s｜当前：%s｜1 巡航 / 2 过门，空格记录，X 撤销，Q 保存退出' % (count, count+1, position, label)


def show_event(event):
    kind = event['kind']
    if kind in ('recorded', 'removed'):
        label = ('已覆盖为' if event.get('replaced') else '已记录') if kind == 'recorded' else '已撤销'
        print('\r\033[K%s%s %d XY=(%+.3f, %+.3f)' % (label, '进场点' if event.get('section') == 'entry' else '航点',
                                                      event['index'], *event['xy']), flush=True)
        if event.get('next_section'):
            print('\r\033[K进场点已录 3 个，进入闭环航点', flush=True)
    elif kind == 'section_finished':
        print('\r\033[K进入闭环航点', flush=True)
    elif kind == 'profile_selected':
        print('已选择：'+('过门' if event['profile'] == 'gate' else '巡航')+'；空格记录', flush=True)
    elif kind == 'ignored':
        print('\r\033[K'+event['message'], flush=True)


def terminal_loop(session, shutdown=lambda: False):
    """Use the contest recorder's key decoding and terminal/signal handling."""
    fd = sys.stdin.fileno()
    original = termios.tcgetattr(fd)
    handlers, running = {}, [True]
    decoder = RecorderKeyDecoder()

    def stop(_signal, _frame):
        running[0] = False

    try:
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            handlers[signum] = signal.signal(signum, stop)
        tty.setcbreak(fd)
        attributes = termios.tcgetattr(fd)
        attributes[0] &= ~(termios.IXON | termios.IXOFF)
        termios.tcsetattr(fd, termios.TCSANOW, attributes)
        print('\033[?2004h', end='', flush=True)
        while running[0] and not shutdown():
            print('\r\033[K'+session.status(), end='', flush=True)
            if not select.select([fd], [], [], .2)[0]:
                continue
            data = os.read(fd, 128)
            if not data:
                break
            for key in decoder.feed(data):
                event = session.key(key)
                show_event(event)
                if event['kind'] == 'save_exit':
                    running[0] = False
                    break
    finally:
        for signum, handler in handlers.items():
            signal.signal(signum, handler)
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, original)
        except (termios.error, OSError):
            pass
        print('\033[?2004l\r\033[K', flush=True)


def stdin_loop(session, stream):
    print('离线输入：xy X Y 更新示例坐标；space / x / q 执行录点按键。')
    for raw in stream:
        command = raw.strip()
        if not command or command.startswith('#'):
            continue
        fields = command.split()
        if fields[0].lower() == 'xy':
            if len(fields) != 3:
                raise ValueError('示例坐标格式：xy 1.2 -0.3')
            session.observe_xy(float(fields[1]), float(fields[2]))
        else:
            key = {'space': ' ', 'x': 'x', 'q': 'q', '1': '1', '2': '2', 'enter': '\n'}.get(command.lower())
            if key is None:
                raise ValueError('离线输入仅接受 xy X Y、space、enter、x、q')
            event = session.key(key)
            show_event(event)
            if event['kind'] == 'save_exit':
                break
        print(session.status(), flush=True)


def main(argv=None, *, delivery=False, boundary=False, single=False):
    recorder_class, loader, saver = WaypointRecorder, load_route, save_route
    default_output, help_text = DEFAULT_OUTPUT, HELP
    if single:
        from airseafly_real400_mission.drop_points import SingleDropPointRecorder, load_points, save_points
        recorder_class, loader, saver = SingleDropPointRecorder, load_points, save_points
        default_output = PROJECT/'data/routes/single_drop_point.yaml'
        help_text = '''单投放录点：拿着飞机走到投放靶标正上方，按空格记录这一个点的 X/Y。
再按空格会用最新位置覆盖旧点；X：撤销；Q / Ctrl+C：保存退出。
保存到 data/routes/single_drop_point.yaml，单独一份，不覆盖普通航点和比赛航线。
只读定位，不控制飞机或舵机；保持同次定位，用 sh flight/single_drop.sh 执行。'''
        delivery = True  # Same display and saver family as drop points.
    if delivery and not single:
        from airseafly_real400_mission.drop_points import DropPointRecorder, load_points, save_points
        recorder_class, loader, saver = DropPointRecorder, load_points, save_points
        help_text = '''投放测试录点：按希望投放的顺序走到各个目标位置，只记录 X/Y。
空格：记录一点；X：撤销最后一点；Q / Ctrl+C：保存退出。
录 1～4 个点，对应 MAIN6 第一档、MAIN7 第一档、MAIN6 第二档、MAIN7 第二档。
起飞点不需要录入；投放完成自动返回实际起飞点降落。
与 record_points.sh 共用航点文件；只读定位，不控制飞机或舵机。
保持同次定位，使用 drop_points.sh 执行。'''
    if boundary:
        from airseafly_real400_mission.virtual_boundary import BoundaryRecorder, load_recording, save_recording, DEFAULT_BOUNDARY
        recorder_class, loader, saver = BoundaryRecorder, load_recording, save_recording
        default_output = PROJECT/DEFAULT_BOUNDARY
        help_text = '场地虚拟围墙：拿着未解锁的飞机沿场地边缘依次走四角，顺/逆时针均可。\n空格记录 XY；X 撤销；Q 保存四角；不记录高度，不发控制指令。\n不要交叉录点；保存不自动开启围墙，在 flight.yaml 中配置启用。'
    parser = argparse.ArgumentParser(description=help_text.splitlines()[0])
    parser.add_argument('--output', type=Path, help='保存文件，默认 '+str(default_output))
    parser.add_argument('--flight-config', type=Path, default=PROJECT/'flight/flight.yaml', help='显示两类点的配置高度和膨胀；执行时读取当时配置')
    parser.add_argument('--resume', nargs='?', const='', metavar='FILE', help='继续录点；不带路径时读取 --output 或默认文件')
    parser.add_argument('--demo', action='store_true', help='离线录点演示，不导入 ROS，不连接硬件')
    parser.add_argument('--stdin', action='store_true', help='配合 --demo 从标准输入读取 xy、space、x、q')
    parser.add_argument('--xy', nargs=2, type=float, default=(0., 0.), metavar=('X', 'Y'), help='离线键盘演示使用的坐标，默认 0 0')
    args = parser.parse_args(argv)
    if args.stdin and not args.demo:
        parser.error('--stdin 只用于 --demo，不能把模拟坐标混入实机录点')
    output = args.output or default_output
    if args.resume is not None:
        source = Path(args.resume) if args.resume else output
        recorder = recorder_class(loader(source))
        if args.output is None:
            output = source
    else:
        recorder = recorder_class()
    profiles = None
    if not delivery and not boundary:
        from airseafly_real400_mission.flight_configuration import base_configuration, apply_flight_configuration
        from airseafly_real400_mission.route_profiles import resolve_profiles
        profiles = resolve_profiles(apply_flight_configuration(base_configuration(), args.flight_config, task='waypoint-route'))
    session = RecorderSession(recorder, delivery=delivery, profiles=profiles, boundary=boundary, single=single)
    print(help_text)
    print('保存位置：'+str(output))
    subscriber = None
    shutdown = lambda: False
    if args.demo:
        session.observe_xy(*args.xy)
        print('离线模式：坐标只是录点输入样例，没有连接 ROS，也不模拟飞行。')
    else:
        import rospy
        from nav_msgs.msg import Odometry
        rospy.init_node('record_single_drop' if single else 'record_drop_points' if delivery else 'record_waypoints',
                        anonymous=True, disable_signals=True)
        subscriber = rospy.Subscriber(TOPIC, Odometry, session.on_odometry, queue_size=1)
        shutdown = rospy.is_shutdown
        print('只读取：'+TOPIC+'；录点到执行任务须保持本次定位连续运行。')
    try:
        try:
            if args.stdin:
                stdin_loop(session, sys.stdin)
            else:
                if not sys.stdin.isatty():
                    raise ValueError('请在交互终端运行；离线脚本演示使用 --demo --stdin')
                terminal_loop(session, shutdown)
        except KeyboardInterrupt:
            print('\n保存当前录点。')
        saver(output, session.document())
        entries = len(recorder.document.get('entry_points', [])) if not delivery and not boundary else 0
        print('已保存 %s%d 个航点：%s；可用 --resume 继续录点。' % (
            '%d 个进场点、' % entries if entries else '', len(recorder.document['points']), output), flush=True)
        if not delivery and not boundary and recorder.document['points']:
            try:
                from airseafly_real400_mission.waypoint_route_cli import write_route_preview
                print('正在生成轨迹预览……', flush=True)
                html = Path(output).with_name(Path(output).stem+'_preview.html')
                print('轨迹预览（浏览器打开）：'+write_route_preview(output, args.flight_config, html), flush=True)
            except Exception as exc:
                # The points are already saved; a preview failure never blocks them.
                print('轨迹预览未能生成：%s（可用 sh flight/fly_points.sh --preview-html 文件.html 再生成）' % exc, flush=True)
        return 0
    finally:
        if subscriber is not None:
            subscriber.unregister()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print('录点未能继续：'+str(exc), file=sys.stderr)
        raise SystemExit(2)
