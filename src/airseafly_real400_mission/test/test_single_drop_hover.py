"""Recorded single target: fly there, drop once, recover, then keep hovering.

Offline only: no ROS, services, cameras, servos or aircraft.
"""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from airseafly_real400_mission import fixed_trajectory
from airseafly_real400_mission.drop_points import SingleDropPointRecorder
from airseafly_real400_mission.native_single_drop import (
    SingleDropEngine, build_single_drop_itinerary, validate_drop_test)
from airseafly_real400_mission.single_drop_cli import load_single_point
from airseafly_real400_mission.waypoint_route import save_route
import test_native_fixed_delivery_route as fixed_delivery
import test_single_drop as single


def hover_configuration(fixed=False, slot=2, target=(-3., 6.)):
    config = fixed_delivery.fixed_configuration() if fixed else single.configuration(slot, target)
    config['drop_test'] = dict(target_xy=list(target), slot=slot, finish='hover')
    return config


def fly(fixed):
    config = hover_configuration(fixed)
    h = (fixed_delivery.FixedReplay(config, engine_class=SingleDropEngine, mode='drop_test') if fixed
         else single.SingleDropReplay(config))
    for _ in range(40000):
        if h.engine.done:
            return h
        h.step()
    raise AssertionError((h.engine.state, h.engine.phase))


class HoverItineraryTests(unittest.TestCase):
    def test_finish_defaults_to_land_and_rejects_unknown(self):
        self.assertEqual(validate_drop_test(dict(target_xy=[1, 2], slot=1))['finish'], 'land')
        with self.assertRaisesRegex(ValueError, 'finish'):
            validate_drop_test(dict(target_xy=[1, 2], slot=1, finish='return'))

    def test_hover_itinerary_has_no_return_or_land(self):
        steps = build_single_drop_itinerary([1., 2.], 3, home=[0., 0.], finish='hover')
        self.assertEqual([s['kind'] for s in steps], ['navigation', 'drop', 'hover'])
        land = build_single_drop_itinerary([1., 2.], 3, home=[0., 0.])
        self.assertEqual([s['kind'] for s in land], ['navigation', 'drop', 'navigation', 'land'])


class HoverEngineTests(unittest.TestCase):
    def test_drops_once_then_holds_recovery_point_without_landing(self):
        for fixed in (False, True):
            if fixed:
                try:
                    fixed_trajectory._load_library()
                except RuntimeError as exc:
                    self.skipTest(str(exc))
            with self.subTest(fixed=fixed):
                h = fly(fixed)
                e = h.engine
                self.assertEqual((e.state, e.phase), ('COMPLETE', 'DROP_HOVER'))
                self.assertEqual([v['slot'] for v in h.effects if v['type'] == 'release'], [2])
                self.assertFalse([v for v in h.effects if v['type'] in ('land', 'native_owner')
                                  and v.get('owner') == 'PX4_AUTO_LAND'])
                hover = next(v for v in h.effects if v['type'] == 'drop_hover')['position_xyz']
                self.assertAlmostEqual(hover[0], -3., places=6)
                self.assertAlmostEqual(hover[1], 6., places=6)
                self.assertAlmostEqual(hover[2], h.ground[2]+e.config['flight']['cruise_height_m'], places=6)
                for _ in range(30):
                    output = h.step(dt=1.)
                    self.assertEqual((output['state'], output['phase'], output['desired_group']),
                                     ('COMPLETE', 'DROP_HOVER', 'NONE'))
                    self.assertEqual([float(v) for v in output['native_command']['position']],
                                     [float(v) for v in hover])
                self.assertEqual(output['drop_test']['stage'], 'hover')
                self.assertEqual(sum(v['type'] == 'release' for v in h.effects), 1)


class RecorderAndEntryTests(unittest.TestCase):
    def test_second_space_replaces_the_one_point(self):
        r = SingleDropPointRecorder()
        r.observe_xy(1., 2.)
        r.key(' ')
        r.observe_xy(3., 4.)
        event = r.key(' ')
        self.assertEqual((r.document['points'], event['replaced']), ([[3., 4.]], [1., 2.]))
        r.key('x')
        self.assertEqual(r.document['points'], [])

    def test_point_file_must_hold_exactly_one_point(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'point.yaml'
            save_route(path, dict(format='aquacore_waypoints', version=1, frame='fast_lio_local_enu',
                                  points=[[1., 2.], [3., 4.]]))
            with self.assertRaisesRegex(ValueError, '只有 1 个点'):
                load_single_point(path)
            save_route(path, dict(format='aquacore_waypoints', version=1, frame='fast_lio_local_enu',
                                  points=[[1., 2.]]))
            self.assertEqual(load_single_point(path)[0], [1., 2.])

    def test_record_then_dry_run_shows_hover_and_selected_slot(self):
        try:
            fixed_trajectory._load_library()
        except RuntimeError as exc:
            self.skipTest(str(exc))
        env = dict(__import__('os').environ, AIRSEAFLY_PYTHON=sys.executable)
        with tempfile.TemporaryDirectory() as folder:
            point = Path(folder)/'single.yaml'
            record = subprocess.run(['sh', str(ROOT/'flight/record_single_drop.sh'), '--demo', '--stdin',
                                     '--output', str(point)], env=env, text=True, capture_output=True,
                                    input='xy 1 2\nspace\nxy 1.5 2.5\nspace\nq\n', timeout=30)
            self.assertEqual(record.returncode, 0, record.stderr)
            self.assertIn('已覆盖', record.stdout)
            run = subprocess.run(['sh', str(ROOT/'flight/single_drop.sh'), '--point', str(point),
                                  '--slot', '3', '--dry-run', '--json'], env=env, text=True,
                                 capture_output=True, timeout=60)
            self.assertEqual(run.returncode, 0, run.stderr)
            value = json.loads(run.stdout)
            self.assertEqual((value['target_xy'], value['selected_payload']['slot'], value['finish_action']),
                             ([1.5, 2.5], 3, 'hover_after_recovery'))
            self.assertEqual([s['kind'] for s in value['itinerary']], ['navigation', 'drop', 'hover'])
            text = subprocess.run(['sh', str(ROOT/'flight/single_drop.sh'), '--point', str(point), '--dry-run'],
                                  env=env, text=True, capture_output=True, timeout=60)
            self.assertIn('持续悬停，不返航、不降落', text.stdout)
            self.assertIn('投放编号 1', text.stdout)

    def test_drop_test_dry_run_no_longer_needs_contest_only_options(self):
        env = dict(__import__('os').environ, AIRSEAFLY_PYTHON=sys.executable)
        run = subprocess.run(['sh', str(ROOT/'flight/drop_test.sh'), '--x', '1', '--y', '2', '--slot', '1',
                              '--dry-run'], env=env, text=True, capture_output=True, timeout=60)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertIn('回起飞点降落', run.stdout)


if __name__ == '__main__':
    unittest.main()
