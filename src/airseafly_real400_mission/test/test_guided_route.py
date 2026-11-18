"""Contest-order recording, undo/resume and read-only keyboard integration."""
from copy import deepcopy
import importlib.util
import math
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from airseafly_real400_mission.guided_route import (
    FRAME, FreeRouteRecorder, GuidedRouteRecorder, RecorderKeyDecoder, load_route, save_route,
    validate_route,
)

SCRIPT = PACKAGE/'scripts/record_delivery_route.py'


def finished_recorder():
    recorder = GuidedRouteRecorder()
    samples = [
        ((0., 0.), ' '), ((3.3, 0.), ' '),
        ((3.4, 1.5), ' '), ((2., 1.), ' '), (None, '\n'),
        ((0., 2.2), ' '), ((-3.4, 2.2), ' '), (None, '\n'),
        ((-3.3, 0.), ' '), ((-3.3, -.8), ' '), ((-3.3, -1.8), ' '), (None, '\n'),
        ((0., -2.2), ' '), ((3.4, -2.2), ' '), (None, '\n'),
    ]
    for xy, key in samples:
        if xy is not None:
            recorder.observe_xy(*xy)
        recorder.key(key)
    return recorder


class RecorderTests(unittest.TestCase):
    def test_complete_contest_order_excludes_home_and_duplicate_closure(self):
        recorder = finished_recorder()
        result = validate_route(recorder.to_dict(), require_complete=True)
        self.assertTrue(recorder.complete)
        self.assertEqual(result['home'], [0., 0.])
        self.assertEqual(result['frame'], FRAME)
        self.assertEqual([p['drop_slot'] for p in result['route'] if p['kind'] == 'drop'], [1, 2, 3, 4])
        self.assertEqual([p['stage'] for p in result['route']], [
            'drop1', 'obstacle2', 'obstacle2', 'drop2', 'obstacle3',
            'drop3', 'gate', 'gate', 'drop4', 'return_drop1'])
        self.assertTrue(all(len(p['xy']) == 2 and 'z' not in p for p in result['route']))

    def test_space_samples_latest_xy_and_does_not_auto_finish_variable_section(self):
        recorder = GuidedRouteRecorder()
        self.assertEqual(recorder.key(' ')['kind'], 'ignored')
        recorder.observe_xy(0., 0.)
        recorder.key(' ')
        recorder.observe_xy(3., 0.)
        recorder.key(' ')
        for i in range(1500):
            recorder.observe_xy(i/100., -i/100.)
            self.assertEqual(recorder.key(' ')['kind'], 'recorded')
        self.assertEqual(recorder.stage_index, 2)
        self.assertEqual(recorder.to_dict()['route'][-1]['xy'], [14.99, -14.99])
        recorder.key('\n')
        self.assertEqual(recorder.current_stage['id'], 'drop2')

    def test_enter_cannot_skip_fixed_point_but_empty_variable_sections_are_allowed(self):
        recorder = GuidedRouteRecorder()
        recorder.observe_xy(1., 2.)
        recorder.key('\n')
        self.assertEqual(recorder.stage_index, 0)
        recorder.key(' ')
        recorder.key(' ')
        recorder.key('\n')
        self.assertEqual(recorder.stage_index, 3)
        self.assertEqual([p['stage'] for p in recorder.to_dict()['route']], ['drop1'])

    def test_undo_across_automatic_and_manual_stage_transitions(self):
        recorder = GuidedRouteRecorder()
        recorder.observe_xy(1., 2.)
        for key in (' ', ' ', '\n'):
            recorder.key(key)
        self.assertEqual(recorder.stage_index, 3)
        self.assertEqual(recorder.key('x')['point']['stage'], 'drop1')
        self.assertEqual(recorder.stage_index, 1)
        recorder.key(' ')
        recorder.key(' ')
        recorder.key('\n')
        self.assertEqual(recorder.key('x')['point']['stage'], 'obstacle2')
        self.assertEqual(recorder.stage_index, 2)
        recorder.key('x')
        recorder.key('x')
        self.assertEqual(recorder.stage_index, 0)
        self.assertIsNone(recorder.to_dict()['home'])
        self.assertEqual(recorder.key('x')['kind'], 'ignored')

    def test_complete_route_can_undo_and_resume_without_repeating_drops(self):
        recorder = finished_recorder()
        recorder.key('x')
        self.assertEqual(recorder.stage_index, 8)
        resumed = GuidedRouteRecorder(recorder.to_dict())
        self.assertIsNone(resumed.latest_xy)
        resumed.observe_xy(3.5, -2.)
        resumed.key(' ')
        resumed.key('\n')
        self.assertEqual([p['drop_slot'] for p in resumed.to_dict()['route'] if p['kind'] == 'drop'], [1, 2, 3, 4])
        validate_route(resumed.to_dict(), require_complete=True)

    def test_save_partial_and_resume_preserves_progress_with_no_xy_fabrication(self):
        recorder = GuidedRouteRecorder()
        recorder.observe_xy(1., 2.)
        recorder.key(' ')
        recorder.key(' ')
        recorder.key(' ')
        self.assertEqual(recorder.key('q')['kind'], 'save_exit')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'nested/route.yaml'
            save_route(path, recorder.to_dict())
            restored = GuidedRouteRecorder(load_route(path))
            self.assertEqual(restored.to_dict(), recorder.to_dict())
            self.assertEqual(restored.key(' ')['kind'], 'ignored')
            with self.assertRaisesRegex(ValueError, '尚未录完'):
                load_route(path, require_complete=True)
            self.assertFalse(path.with_name(path.name+'.tmp').exists())

    def test_structure_rejects_missing_home_drop_wrong_order_or_height(self):
        original = finished_recorder().to_dict()
        cases = []
        missing_home = deepcopy(original)
        missing_home['home'] = None
        cases.append(missing_home)
        missing_drop = deepcopy(original)
        missing_drop['route'] = [p for p in missing_drop['route'] if p.get('drop_slot') != 3]
        cases.append(missing_drop)
        wrong_order = deepcopy(original)
        wrong_order['route'].reverse()
        cases.append(wrong_order)
        wrong_height = deepcopy(original)
        wrong_height['route'][0]['xy'].append(1.)
        cases.append(wrong_height)
        duplicate = deepcopy(original)
        duplicate['route'].append(deepcopy(duplicate['route'][0]))
        cases.append(duplicate)
        for case in cases:
            with self.subTest(case=case), self.assertRaises(ValueError):
                validate_route(case, require_complete=True)
        for bad in (math.nan, math.inf, True):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                GuidedRouteRecorder().observe_xy(bad, 0.)

    def test_key_decoder_preserves_order_enter_and_x_are_recording_keys(self):
        decoder = RecorderKeyDecoder()
        self.assertEqual(decoder.feed(b'  x\r\nq'), [' ', ' ', 'x', '\n', 'q'])
        self.assertEqual(decoder.feed(b'\x1b[A\x1b[200~ xq\n\x1b[201~ '), [' '])
        self.assertEqual(decoder.feed(b'\x03'), ['q'])


class RecorderScriptTests(unittest.TestCase):
    def test_status_counts_current_variable_section_and_follows_undo(self):
        spec = importlib.util.spec_from_file_location('route_recorder_status', SCRIPT)
        entry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(entry)
        session = entry.RecorderSession(GuidedRouteRecorder())
        self.assertIn('中央起降点', session.status())
        session.observe_xy(0., 0.)
        session.key(' ')
        self.assertIn('也是绕圈起点', session.status())
        session.key(' ')
        self.assertIn('本段已记录 0 个航点', session.status())
        session.key(' ')
        session.key(' ')
        self.assertIn('本段已记录 2 个航点', session.status())
        session.key('\n')
        session.key('x')
        self.assertIn('本段已记录 1 个航点', session.status())

    def test_odometry_reads_only_xy_without_flight_node_or_age_checks(self):
        spec = importlib.util.spec_from_file_location('route_recorder_entry', SCRIPT)
        entry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(entry)
        session = entry.RecorderSession(GuidedRouteRecorder())
        session.on_odometry(NS(pose=NS(pose=NS(position=NS(x=3., y=-2., z=99.)))))
        session.key(' ')
        self.assertEqual(session.document()['home'], [3., -2.])
        session.key('x')
        self.assertIsNone(session.document()['home'])

    def test_stdin_demo_and_resume_without_ros(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'route.yaml'
            first = subprocess.run([sys.executable, str(SCRIPT), '--wizard', '--demo', '--stdin', '--output', str(output)],
                input='xy 0 0\nspace\nxy 3.3 0\nspace\nxy 3.4 1\nspace\nq\n',
                capture_output=True, text=True, timeout=10)
            self.assertEqual(first.returncode, 0, first.stderr)
            before = load_route(output)
            self.assertEqual(before['wizard']['stage_index'], 2)
            second = subprocess.run([sys.executable, str(SCRIPT), '--demo', '--stdin', '--resume', str(output)],
                input='x\nxy 3.5 1.2\nspace\nenter\nxy 0 2.2\nspace\nq\n',
                capture_output=True, text=True, timeout=10)
            self.assertEqual(second.returncode, 0, second.stderr)
            result = load_route(output)
            self.assertEqual(result['wizard']['stage_index'], 4)
            self.assertEqual(result['route'][1]['xy'], [3.5, 1.2])
            self.assertEqual(result['route'][-1]['drop_slot'], 2)

    def test_raw_terminal_space_without_enter_x_undo_and_q_save(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'route.yaml'
            master, slave = pty.openpty()
            process = subprocess.Popen([sys.executable, str(SCRIPT), '--wizard', '--demo', '--xy', '2', '3', '--output', str(output)],
                stdin=slave, stdout=slave, stderr=slave, close_fds=True)
            os.close(slave)
            received = b''
            deadline = time.monotonic()+5.
            try:
                while b'\x1b[?2004h' not in received and time.monotonic() < deadline:
                    if select.select([master], [], [], .1)[0]:
                        received += os.read(master, 4096)
                self.assertIn(b'\x1b[?2004h', received)
                os.write(master, b'  x q')
                deadline = time.monotonic()+5.
                while process.poll() is None and time.monotonic() < deadline:
                    if select.select([master], [], [], .1)[0]:
                        try:
                            received += os.read(master, 4096)
                        except OSError:
                            break
                process.wait(timeout=1)
                self.assertEqual(process.returncode, 0)
                result = load_route(output)
                self.assertEqual(result['home'], [2., 3.])
                self.assertEqual(result['wizard']['stage_index'], 2)
                self.assertEqual(len(result['route']), 1)
                self.assertEqual(result['route'][0]['drop_slot'], 1)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
                os.close(master)

    def test_shell_help_needs_no_ros_environment(self):
        result = subprocess.run(['sh', str(ROOT/'flight/record_route.sh'), '--help'],
            env=dict(os.environ, AIRSEAFLY_PYTHON=sys.executable), capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--resume', result.stdout)


def free_recorder(keys):
    recorder = FreeRouteRecorder()
    for xy, key in keys:
        if xy is not None:
            recorder.observe_xy(*xy)
        recorder.key(key)
    return recorder


class FreeRecorderTests(unittest.TestCase):
    def test_types_are_chosen_per_point_and_drops_numbered_in_record_order(self):
        r = free_recorder([((0., 0.), 'h'), ((1., 0.), ' '), (None, '3'), ((2., 0.), ' '), (None, '2'),
                           ((3., 1.), ' '), ((3., 2.), ' '), (None, '1'), ((4., 2.), ' '), (None, '3'),
                           ((5., 2.), ' ')])
        route = r.to_dict()['route']
        self.assertEqual([p['kind'] for p in route], ['transit', 'drop', 'gate', 'gate', 'transit', 'drop'])
        self.assertEqual([p['drop_slot'] for p in route if p['kind'] == 'drop'], [1, 2])
        self.assertEqual(r.to_dict()['home'], [0., 0.])
        self.assertTrue(r.complete)
        self.assertEqual(r.selected_kind, 'drop')

    def test_select_keys_only_switch_type_and_space_records(self):
        r = free_recorder([((0., 0.), 'h'), ((1., 0.), '1'), ((1., 0.), '2'), ((1., 0.), '3')])
        self.assertEqual(r.to_dict()['route'], [])
        for key, kind in (('1', 'transit'), ('2', 'gate'), ('3', 'drop')):
            event = r.key(key)
            self.assertEqual((event['kind'], event['point_kind'], r.selected_kind), ('kind_selected', kind, kind))
        r.observe_xy(1., 1.)
        r.key(' ')
        r.observe_xy(2., 2.)
        r.key(' ')
        self.assertEqual([(p['kind'], p['drop_slot']) for p in r.to_dict()['route']], [('drop', 1), ('drop', 2)])

    def test_fifth_drop_is_refused_and_undo_renumbers_the_next_drop(self):
        r = free_recorder([((0., 0.), 'h'), (None, '3')]+[((float(i), 1.), ' ') for i in range(1, 5)])
        r.observe_xy(9., 9.)
        self.assertEqual(r.key(' ')['kind'], 'ignored')
        self.assertEqual(r.drops, 4)
        self.assertEqual(r.key('x')['point']['drop_slot'], 4)
        r.observe_xy(7., 7.)
        self.assertEqual(r.key(' ')['point']['drop_slot'], 4)

    def test_home_is_replaced_by_h_and_never_undone_by_x(self):
        r = free_recorder([((0., 0.), 'h'), ((1., 1.), 'h')])
        self.assertEqual(r.to_dict()['home'], [1., 1.])
        self.assertEqual(r.key('x')['kind'], 'ignored')
        self.assertEqual(r.to_dict()['home'], [1., 1.])

    def test_flight_needs_home_and_one_to_four_drops_but_partial_saves_are_allowed(self):
        no_home = free_recorder([(None, '3'), ((1., 0.), ' ')]).to_dict()
        no_drop = free_recorder([((0., 0.), 'h'), ((1., 0.), ' ')]).to_dict()
        for document, message in ((no_home, 'HOME'), (no_drop, '投放点')):
            validate_route(document)
            with self.assertRaisesRegex(ValueError, message):
                validate_route(document, require_complete=True)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'route.yaml'
            save_route(path, no_drop)
            self.assertEqual(load_route(path), validate_route(no_drop))

    def test_hand_edited_free_files_are_checked(self):
        base = dict(version=2, frame=FRAME, mode='free', home=[0., 0.])
        bad = [[dict(kind='drop', xy=[1., 0.], drop_slot=2)],
               [dict(kind='drop', xy=[float(i), 0.], drop_slot=i) for i in range(1, 6)],
               [dict(kind='transit', xy=[1., 0.], drop_slot=1)],
               [dict(kind='home', xy=[1., 0.])],
               [dict(kind='transit', xy=[1., 0., 1.])]]
        for route in bad:
            with self.assertRaises(ValueError):
                validate_route(dict(base, route=route))
        with self.assertRaises(ValueError):
            FreeRouteRecorder(finished_recorder().to_dict())

    def test_key_decoder_passes_free_recording_keys(self):
        self.assertEqual(RecorderKeyDecoder().feed(b'h1 2 3xq'), ['h', '1', ' ', '2', ' ', '3', 'x', 'q'])

    def test_stdin_free_demo_saves_and_resumes_in_free_mode(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'route.yaml'
            save_route(output, FreeRouteRecorder().to_dict())
            first = subprocess.run([sys.executable, str(SCRIPT), '--demo', '--stdin', '--resume', str(output)],
                input='xy 0 0\nh\nxy 1 0\nspace\n3\nxy 2 0\nspace\nq\n', capture_output=True, text=True, timeout=10)
            self.assertEqual(first.returncode, 0, first.stderr)
            self.assertIn('可以飞行', first.stdout)
            # A complete route prints its flight order; the return leaves from the first point.
            self.assertIn('第2圈：', first.stdout)
            self.assertIn('2.投放1[投] → 1.普通（闭环）', first.stdout)
            self.assertIn('返航：1.普通 → HOME 降落', first.stdout)
            self.assertIn('当前共 2 个点（不含 HOME）', first.stdout)
            self.assertIn('HOME   XY=(+0.000, +0.000)', first.stdout)
            self.assertIn(' 1 普通点 XY=(+1.000, +0.000)', first.stdout)
            self.assertIn(' 2 投放点1 XY=(+2.000, +0.000)', first.stdout)
            second = subprocess.run([sys.executable, str(SCRIPT), '--demo', '--stdin', '--resume', str(output)],
                input='2\nxy 3 1\nspace\n3\nxy 4 1\nspace\nq\n', capture_output=True, text=True, timeout=10)
            self.assertEqual(second.returncode, 0, second.stderr)
            result = load_route(output, require_complete=True)
            self.assertEqual([p['kind'] for p in result['route']], ['transit', 'drop', 'gate', 'drop'])
            self.assertEqual([p['drop_slot'] for p in result['route'] if p['kind'] == 'drop'], [1, 2])
            partial = Path(directory)/'partial.yaml'
            save_route(partial, FreeRouteRecorder().to_dict())
            missing = subprocess.run([sys.executable, str(SCRIPT), '--demo', '--stdin', '--resume', str(partial)],
                input='xy 1 0\nspace\nq\n', capture_output=True, text=True, timeout=10)
            self.assertEqual(missing.returncode, 0, missing.stderr)
            self.assertIn('暂不能飞行', missing.stdout)


if __name__ == '__main__':
    unittest.main()
