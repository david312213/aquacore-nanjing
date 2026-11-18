#!/usr/bin/env python3
"""Record the fixed contest order from planning_odometry; never command a UAV."""
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
from airseafly_real400_mission.guided_route import (
    ENTRY_GUIDE, FREE_KINDS, LOOP_SECTION_NAMES, LOOP_SECTIONS, FreeRouteRecorder, GuidedRouteRecorder,
    LoopRouteRecorder, RecorderKeyDecoder, load_route, route_recorder, save_route,
)


PROJECT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = PROJECT/'data/routes/delivery_route.yaml'
TOPIC = '/airseafly/native/planning_odometry'


def flight_order(document, flight_config):
    """The recorded route's flight order, as 4_delivery_route.sh will fly it."""
    import yaml
    from airseafly_real400_mission.native_delivery_route import build_itinerary, describe_laps
    profile = (yaml.safe_load(Path(flight_config).read_text(encoding='utf-8')) or {}).get(
        'mission_profiles', {}).get('delivery-route', {})
    laps = (profile.get('delivery_route_options') or {}).get('laps', 1)
    return ['飞行顺序（flight.yaml laps=%d 为巡航圈数；--set delivery_route_options.laps=N 改圈数）：' % laps,
            *('  '+line for line in describe_laps(build_itinerary(document, laps)))]
LOOP_HELP = '''分段录点：进场点 → 巡航闭环 → 投放环 → 返航点，按飞行顺序边走边记。
h：站在起降点记录 HOME（任何时候都可以，再按一次覆盖）。
1 / 2 / 3：只切换类型，不记点：普通点 / 圆环点（圆环高度和圆环速度）/ 投放点（只能在投放环）。
空格：按当前类型把最新 X/Y 记到当前段；每记一个点打印全部已记点。
① 进场点：从起飞点进入巡航闭环的点，引导录 %d 个，录满自动进入巡航闭环；Enter 可提前结束。
② 巡航闭环：Enter 结束；首点不用重复录：前 N-1 圈自动回到巡航首点闭环，最后一圈从巡航最后一点直接飞到投放环第一点。
③ 投放环：单独录一套点，投放点按记录顺序编号 1～4（MAIN6 一档、MAIN7 一档、MAIN6 二档、MAIN7 二档）；Enter 结束。
④ 返航点：投放环之后回 HOME 途经的点；Enter 结束全部录点。
X：删除最后一个航点并回到它所属的段。Q / Ctrl+C：保存退出，未录完也可以保存，--resume 继续。
flight.yaml laps=N：进场后巡航闭环飞 N 圈，再飞一遍投放环，经返航点回 HOME 降落。
起飞前需要 HOME 和 1～4 个投放点；空的段直接跳过。只读定位，不控制飞机或投放。
旧的共用一套点用 --shared，九步向导用 --wizard。''' % ENTRY_GUIDE
FREE_HELP = '''共用航线录点：绕圈和投放只录一套点，按希望飞行的顺序边走边记。
h：站在起降点记录 HOME（再按一次覆盖）。
1 / 2 / 3：只切换类型，不记点：普通点 / 圆环点（圆环高度和圆环速度）/ 投放点。
空格：按当前类型记录最新 X/Y；每记一个点打印全部已记点。
投放点按记录顺序编号 1～4（MAIN6 一档、MAIN7 一档、MAIN6 二档、MAIN7 二档）。
X：删除最后一个航点。Q / Ctrl+C：保存退出，未录完也可以保存。
flight.yaml laps=N：先满载按录制顺序绕 N 圈（每圈回到第一个点）；第 N+1 圈在投放点投放，同样走完全部航点回到第一个点，再从第一个点回 HOME 降落（laps: 0 为第一圈边飞边投）。
起飞前需要 HOME 和 1～4 个投放点。只读定位，不控制飞机或投放。旧的九步向导用 --wizard。'''
HELP = '''按比赛顺序录点：中央起降点 HOME → 投放1（绕圈起点）→ 绕障碍2 → 投放2
→ 障碍3与下/右边界之间 → 投放3 → 圆环及门后路径 → 投放4
→ 障碍1与上/左边界之间返回投放1。
空格：立即记录最新 X/Y；固定点录完自动进入下一阶段。
Enter：结束当前可变航点段；段内可以反复按空格，不限制点数。
X：删除最后一个记录点，并回到它所属的阶段。
Q / Ctrl+C：保存当前进度并退出，未完成也可以保存。
闭环复用投放点1，返航复用 HOME；高度另设。只读定位，不控制飞机或投放。'''


class RecorderSession:
    def __init__(self, recorder):
        self.recorder = recorder
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
            position = '未收到坐标' if xy is None else 'XY=(%+.3f, %+.3f)米' % tuple(xy)
            return self.recorder.status_text(position)


def point_label(point):
    if point['kind'] == 'drop':
        return '投放点%d' % point['drop_slot']
    return FREE_KINDS.get(point['kind'], point['stage']) if point['stage'] == point['kind'] else point['stage']


def show_event(event):
    kind = event['kind']
    if kind in ('recorded', 'removed'):
        point = event['point']
        label = '已记录' if kind == 'recorded' else '已撤销'
        where = LOOP_SECTION_NAMES[event['section']]+' ' if event.get('section') else ''
        print('\r\033[K%s %s%s XY=(%+.3f, %+.3f)' % (label, where, point_label(point), *point['xy']), flush=True)
        if event.get('next_section'):
            print('\r\033[K进场点已录 %d 个，进入：%s' % (ENTRY_GUIDE, LOOP_SECTION_NAMES[event['next_section']]), flush=True)
    elif kind == 'home_recorded':
        print('\r\033[K%s HOME XY=(%+.3f, %+.3f)' % ('已覆盖' if event['replaced'] else '已记录', *event['xy']), flush=True)
    elif kind == 'kind_selected':
        print('\r\033[K已选择：%s；空格记录' % FREE_KINDS[event['point_kind']], flush=True)
    elif kind == 'stage_finished':
        print('\r\033[K本段完成：'+event['stage'], flush=True)
    elif kind == 'section_finished':
        section = event['section']
        print('\r\033[K'+('各段已录完；Q 保存' if section == 'done' else '进入：'+LOOP_SECTION_NAMES[section]), flush=True)
    elif kind == 'ignored':
        print('\r\033[K'+event['message'], flush=True)


def show_points(document):
    """Print HOME and every recorded point after the list changes."""
    home = document.get('home')
    if document.get('version') == 4:
        total = sum(len(document[section]) for section in LOOP_SECTIONS)
        print('\r\033[K当前共 %d 个点（不含 HOME）：' % total, flush=True)
        print('\r\033[K  HOME   ' + ('未记录' if home is None else 'XY=(%+.3f, %+.3f)' % tuple(home)), flush=True)
        for section in LOOP_SECTIONS:
            print('\r\033[K  %s（%d 个）%s' % (LOOP_SECTION_NAMES[section], len(document[section]),
                  '，前几圈回首点闭环，最后一圈从末点进入投放环' if section == 'cruise_loop' and document[section] else ''), flush=True)
            for number, point in enumerate(document[section], 1):
                print('\r\033[K    %2d %s XY=(%+.3f, %+.3f)' % (number, point_label(point), *point['xy']), flush=True)
        return
    route = document['route']
    print('\r\033[K当前共 %d 个点（不含 HOME）：' % len(route), flush=True)
    print('\r\033[K  HOME   ' + ('未记录' if home is None else 'XY=(%+.3f, %+.3f)' % tuple(home)), flush=True)
    for number, point in enumerate(route, 1):
        print('\r\033[K  %2d %s XY=(%+.3f, %+.3f)' % (number, point_label(point), *point['xy']), flush=True)


def show_key_result(session, event):
    show_event(event)
    if event['kind'] in ('recorded', 'removed', 'home_recorded') and isinstance(
            session.recorder, (FreeRouteRecorder, LoopRouteRecorder)):
        show_points(session.document())


def terminal_loop(session, shutdown=lambda: False):
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
                show_key_result(session, event)
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
    """Offline coordinate samples plus the same four recorder key actions."""
    print('离线输入：xy X Y 更新示例坐标；space / enter / x / q / h / 1 / 2 / 3 执行录点按键。')
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
            key = {'space': ' ', 'enter': '\n', 'x': 'x', 'q': 'q', 'h': 'h',
                   '1': '1', '2': '2', '3': '3'}.get(command.lower())
            if key is None:
                raise ValueError('离线输入仅接受 xy X Y、space、enter、x、q、h、1、2、3')
            event = session.key(key)
            show_key_result(session, event)
            if event['kind'] == 'save_exit':
                break
        print(session.status(), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description='记录配送航线，仅保存 X/Y；无飞行或投放输出')
    parser.add_argument('--output', type=Path, help='保存文件，默认 data/routes/delivery_route.yaml')
    parser.add_argument('--wizard', action='store_true', help='使用旧的九步比赛向导')
    parser.add_argument('--shared', action='store_true', help='使用旧的共用一套点录法；默认分段录进场、巡航闭环、投放环、返航')
    parser.add_argument('--flight-config', type=Path, default=PROJECT/'flight/flight.yaml',
                        help='录完后按其中 laps 显示各圈飞行顺序')
    parser.add_argument('--resume', nargs='?', const='', metavar='FILE', help='继续已有进度；不带路径时读取 --output 或默认文件')
    parser.add_argument('--demo', action='store_true', help='离线录点演示，不导入 ROS，不连接硬件')
    parser.add_argument('--stdin', action='store_true', help='配合 --demo 从标准输入读取 xy、space、enter、x、q')
    parser.add_argument('--xy', nargs=2, type=float, default=(0., 0.), metavar=('X', 'Y'), help='离线键盘演示使用的坐标，默认 0 0')
    args = parser.parse_args(argv)
    if args.stdin and not args.demo:
        parser.error('--stdin 只用于 --demo，不能把模拟坐标混入实机录点')
    output = args.output or DEFAULT_OUTPUT
    if args.resume is not None:
        source = Path(args.resume) if args.resume else output
        # The file decides the mode; a wizard file keeps its nine steps.
        recorder = route_recorder(load_route(source))
        if args.output is None:
            output = source
    else:
        if args.wizard and args.shared:
            parser.error('--wizard 与 --shared 只能选一个')
        recorder = (GuidedRouteRecorder() if args.wizard else FreeRouteRecorder() if args.shared
                    else LoopRouteRecorder())
    session = RecorderSession(recorder)
    loops = isinstance(recorder, LoopRouteRecorder)
    free = loops or isinstance(recorder, FreeRouteRecorder)
    print(LOOP_HELP if loops else FREE_HELP if free else HELP)
    print('保存位置：'+str(output))
    subscriber = None
    shutdown = lambda: False
    if args.demo:
        session.observe_xy(*args.xy)
        print('离线模式：坐标只是录点输入样例，没有连接 ROS，也不模拟飞行。')
    else:
        import rospy
        from nav_msgs.msg import Odometry
        rospy.init_node('record_delivery_route', anonymous=True, disable_signals=True)
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
            print('\n保存当前录点进度。')
        save_route(output, session.document())
        if not free:
            state = '全部阶段已完成' if recorder.complete else '已保存部分进度，可用 --resume 继续'
        elif recorder.complete:
            count = (sum(len(recorder.document[section]) for section in LOOP_SECTIONS) if loops
                     else len(recorder.document['route']))
            state = '已保存 %d 个点、%d 个投放点，可以飞行' % (count, recorder.drops)
        else:
            missing = [text for text, absent in (('HOME（按 h）', recorder.document['home'] is None),
                                                 ('投放点（投放环内按 3 再按空格）' if loops else '投放点（按 3 再按空格）',
                                                  not recorder.drops)) if absent]
            state = '已保存，但还缺 %s，暂不能飞行；可用 --resume 继续' % '、'.join(missing)
        print('%s：%s' % (state, output), flush=True)
        if recorder.complete:
            try:
                print('\n'.join(flight_order(session.document(), args.flight_config)), flush=True)
            except (OSError, ValueError) as exc:
                print('飞行顺序未能显示：'+str(exc), flush=True)
            try:
                from airseafly_real400_mission.delivery_route_html import write_for_route
                print('正在生成轨迹预览……', flush=True)
                print('轨迹预览（浏览器打开）：'+write_for_route(output, args.flight_config), flush=True)
            except Exception as exc:
                # The route is already saved; a preview failure never blocks it.
                print('轨迹预览未能生成：%s（可用 sh flight/4_delivery_route.sh --dry-run 再生成）' % exc, flush=True)
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
