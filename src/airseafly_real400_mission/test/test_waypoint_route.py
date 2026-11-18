"""Ordinary recording, independent file format, and read-only CLI behavior."""
import importlib.util
import io
import math
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
from contextlib import redirect_stdout


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from airseafly_real400_mission.guided_route import GuidedRouteRecorder
from airseafly_real400_mission.waypoint_route import (
    FORMAT, FRAME, WaypointRecorder, load_route, save_route, validate_route,
)

SCRIPT = PACKAGE/'scripts/record_waypoints.py'


def load_script():
    spec = importlib.util.spec_from_file_location('ordinary_waypoint_recorder', SCRIPT)
    entry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(entry)
    return entry



def loop_recorder():
    """A new recording past its entry section, i.e. the original loop-only flow."""
    recorder = WaypointRecorder()
    recorder.key('\n')
    return recorder


class WaypointRecorderTests(unittest.TestCase):
    def test_space_uses_latest_xy_in_order_without_point_count_limit(self):
        recorder = loop_recorder()
        self.assertEqual(recorder.key(' ')['kind'], 'ignored')
        for index in range(1500):
            recorder.observe_xy(index/10., -index/10.)
            event = recorder.key(' ')
            self.assertEqual(event['index'], index+1)
        document = validate_route(recorder.to_dict(), require_points=True)
        self.assertEqual(document['format'], FORMAT)
        self.assertEqual(document['frame'], FRAME)
        self.assertEqual(len(document['points']), 1500)
        self.assertEqual(document['points'][0], [0., 0.])
        self.assertEqual(document['points'][-1], [149.9, -149.9])

    def test_undo_last_point_then_rerecord_uses_same_display_number(self):
        recorder = loop_recorder()
        recorder.observe_xy(1., 2.)
        recorder.key(' ')
        recorder.observe_xy(3., 4.)
        recorder.key(' ')
        removed = recorder.key('x')
        self.assertEqual(removed, dict(kind='removed', index=2, xy=[3., 4.]))
        recorder.observe_xy(5., 6.)
        self.assertEqual(recorder.key(' ')['index'], 2)
        self.assertEqual(recorder.to_dict()['points'], [[1., 2.], [5., 6.]])
        recorder.key('x')
        recorder.key('x')
        self.assertEqual(recorder.key('x')['kind'], 'ignored')
        self.assertEqual(recorder.key('q'), dict(kind='save_exit', count=0))

    def test_duplicate_points_and_enter_leave_no_implicit_closure_or_home(self):
        recorder = loop_recorder()
        recorder.observe_xy(1., 2.)
        recorder.key(' ')
        recorder.key(' ')
        self.assertEqual(recorder.key('\n')['kind'], 'ignored')
        self.assertEqual(recorder.to_dict()['points'], [[1., 2.], [1., 2.]])
        self.assertNotIn('home', recorder.to_dict())

    def test_empty_save_is_allowed_but_flight_requires_a_destination(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'nested/waypoints.yaml'
            save_route(path, loop_recorder().to_dict())
            self.assertEqual(load_route(path)['points'], [])
            with self.assertRaisesRegex(ValueError, '没有航点'):
                load_route(path, require_points=True)
            self.assertFalse(path.with_name(path.name+'.tmp').exists())

    def test_save_resume_preserves_xy_and_never_restores_a_live_position(self):
        recorder = loop_recorder()
        recorder.observe_xy(-1., .25)
        recorder.key(' ')
        recorder.observe_xy(3., -2.)
        recorder.key(' ')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'waypoints.yaml'
            save_route(path, recorder.to_dict())
            restored = WaypointRecorder(load_route(path, require_points=True))
            self.assertEqual(restored.to_dict(), recorder.to_dict())
            self.assertIsNone(restored.latest_xy)
            self.assertEqual(restored.key(' ')['kind'], 'ignored')
            restored.key('x')
            restored.observe_xy(8., 9.)
            restored.key(' ')
            save_route(path, restored.to_dict())
            self.assertEqual(load_route(path)['points'], [[-1., .25], [8., 9.]])

    def test_file_format_rejects_contest_route_and_invalid_coordinates(self):
        document = loop_recorder().to_dict()
        cases = [GuidedRouteRecorder().to_dict(), None, [], dict(document, format='other'),
                 dict(document, version=True), dict(document, frame='map'), dict(document, points={})]
        for point in ([1.], [1., 2., 3.], ['1', 2.], [math.nan, 2.], [1., math.inf], [True, 2.]):
            cases.append(dict(document, points=[point]))
        for case in cases:
            with self.subTest(case=case), self.assertRaises(ValueError):
                validate_route(case)

    def test_route_and_event_copies_cannot_change_the_recording(self):
        recorder = loop_recorder()
        recorder.observe_xy(1., 2.)
        event = recorder.key(' ')
        event['xy'][0] = 99.
        snapshot = recorder.to_dict()
        snapshot['points'][0][1] = 99.
        restored = WaypointRecorder(recorder.to_dict())
        restored.document['points'][0][0] = 50.
        self.assertEqual(recorder.to_dict()['points'], [[1., 2.]])


class WaypointRecorderScriptTests(unittest.TestCase):
    def test_callback_records_xy_ignoring_height_and_timestamp(self):
        entry = load_script()
        session = entry.RecorderSession(loop_recorder())
        message = NS(header=NS(stamp=0), pose=NS(pose=NS(position=NS(x=3., y=-2., z=99.))))
        session.on_odometry(message)
        session.key(' ')
        self.assertEqual(session.document()['points'], [[3., -2.]])
        self.assertIn('闭环 1 点', session.status())
        session.key('x')
        self.assertIn('闭环航点 1', session.status())
        message.pose.pose.position.x = math.nan
        session.on_odometry(message)
        self.assertEqual(session.key(' ')['kind'], 'ignored')

    def test_demo_resume_and_empty_exit_without_ros(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'waypoints.yaml'
            first = subprocess.run([sys.executable, str(SCRIPT), '--demo', '--stdin', '--output', str(output)],
                input='enter\nxy 1 2\nspace\nxy 3 4\nspace\nq\n', capture_output=True, text=True, timeout=10)
            self.assertEqual(first.returncode, 0, first.stderr)
            self.assertEqual(load_route(output)['points'], [[1., 2.], [3., 4.]])
            resumed = subprocess.run([sys.executable, str(SCRIPT), '--demo', '--stdin', '--resume', str(output)],
                input='x\nxy 8 9\nspace\nq\n', capture_output=True, text=True, timeout=10)
            self.assertEqual(resumed.returncode, 0, resumed.stderr)
            self.assertEqual(load_route(output)['points'], [[1., 2.], [8., 9.]])
            empty = subprocess.run([sys.executable, str(SCRIPT), '--demo', '--stdin', '--output', str(output)],
                input='q\n', capture_output=True, text=True, timeout=10)
            self.assertEqual(empty.returncode, 0, empty.stderr)
            self.assertEqual(load_route(output)['points'], [])

    def test_live_main_only_subscribes_and_keyboard_interrupt_saves(self):
        entry = load_script()
        calls = []
        subscriber = NS(unregister=lambda: calls.append(('unregister',)))

        def subscribe(topic, message_type, callback, queue_size):
            calls.append(('subscribe', topic, queue_size))
            callback(NS(pose=NS(pose=NS(position=NS(x=2., y=4., z=9.)))))
            return subscriber

        def interrupt_loop(session, shutdown):
            session.key('\n')  # Skip the entry section: the original loop-only flow.
            session.key(' ')
            raise KeyboardInterrupt()

        rospy = NS(init_node=lambda *a, **kw: calls.append(('init', a, kw)),
                   Subscriber=subscribe, is_shutdown=lambda: False)
        messages = NS(Odometry=object)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'waypoints.yaml'
            with patch.dict(sys.modules, {'rospy': rospy, 'nav_msgs.msg': messages}), \
                    patch.object(sys.stdin, 'isatty', return_value=True), \
                    patch.object(entry, 'terminal_loop', side_effect=interrupt_loop), redirect_stdout(io.StringIO()):
                self.assertEqual(entry.main(['--output', str(output)]), 0)
            self.assertEqual(load_route(output)['points'], [[2., 4.]])
            self.assertEqual(calls[1], ('subscribe', '/airseafly/native/planning_odometry', 1))
            self.assertEqual(calls[-1], ('unregister',))

    def test_raw_terminal_keys_q_and_sigint_save_without_enter(self):
        for interrupt in (False, True):
            with self.subTest(interrupt=interrupt), tempfile.TemporaryDirectory() as directory:
                output = Path(directory)/'waypoints.yaml'
                master, slave = pty.openpty()
                process = subprocess.Popen([sys.executable, str(SCRIPT), '--demo', '--xy', '2', '3', '--output', str(output)],
                    stdin=slave, stdout=slave, stderr=slave, close_fds=True)
                os.close(slave)
                received = b''
                try:
                    deadline = time.monotonic()+5.
                    while b'\x1b[?2004h' not in received and time.monotonic() < deadline:
                        if select.select([master], [], [], .1)[0]:
                            received += os.read(master, 4096)
                    self.assertIn(b'\x1b[?2004h', received)
                    os.write(master, b'\r  x')
                    deadline = time.monotonic()+5.
                    while '已撤销航点 2'.encode() not in received and time.monotonic() < deadline:
                        if select.select([master], [], [], .1)[0]:
                            received += os.read(master, 4096)
                    self.assertIn('已撤销航点 2'.encode(), received)
                    if interrupt:
                        process.send_signal(signal.SIGINT)
                    else:
                        os.write(master, b'q')
                    # A PTY's output must keep draining while the child restores
                    # its terminal with TCSADRAIN (notably on macOS).
                    deadline = time.monotonic()+5.
                    while process.poll() is None and time.monotonic() < deadline:
                        if select.select([master], [], [], .1)[0]:
                            try:
                                received += os.read(master, 4096)
                            except OSError:
                                break
                    self.assertEqual(process.wait(timeout=5), 0)
                    self.assertEqual(load_route(output)['points'], [[2., 3.]])
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.wait()
                    os.close(master)


if __name__ == '__main__':
    unittest.main()
