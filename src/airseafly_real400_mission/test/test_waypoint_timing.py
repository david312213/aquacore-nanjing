"""A/B loop entry tests with offline motion feedback and temporary route files."""
from copy import deepcopy
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from airseafly_real400_mission import local_retiming, waypoint_route_cli as cli
from airseafly_real400_mission.fixed_trajectory import build_fixed_trajectory, FixedTrajectory
from airseafly_real400_mission.fixed_trajectory_preview import route_preview
from airseafly_real400_mission.native_waypoint_route import WaypointRouteEngine
from airseafly_real400_mission.route_profiles import point_velocities
from airseafly_real400_mission.waypoint_route import save_route
from test_native_fixed_waypoint_route import configuration, Replay

ROOT = PACKAGE.parents[1]
POINTS = [[0., 0.], [5., 0.], [6., 1.], [6., 4.], [0., 4.]]


def config(mode='original', laps=1, profiles=None, points=None):
    value = configuration(points or POINTS, profiles)
    value['navigation'].update(laps=laps, timing=mode)
    value['planning'].update(max_velocity_mps=2.8, max_acceleration_mps2=3.8, gate_velocity_mps=.6)
    return value


class WaypointTimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original_config = config()
        cls.fast_config = config('optimized')
        cls.ground = (-1., -.5, -.7)
        cls.original = WaypointRouteEngine(cls.original_config)._build_fixed_trajectory(cls.ground)
        cls.fast = WaypointRouteEngine(cls.fast_config)._build_fixed_trajectory(cls.ground)

    def test_default_and_original_are_exact_original_builder(self):
        c = deepcopy(self.original_config)
        c['navigation'].pop('timing')
        engine = WaypointRouteEngine(c)
        points = [(*xy, self.ground[2]+1.) for xy in POINTS]
        original = build_fixed_trajectory((*self.ground[:2], self.ground[2]+1.), points, 1, 2.8, 3.8,
                                         point_velocities(c, ['cruise']*len(points)))
        with patch.object(local_retiming, 'retime_loaded_leg', side_effect=AssertionError('not selected')):
            self.assertEqual(engine._build_fixed_trajectory(self.ground).to_dict(), original.to_dict())
            self.assertEqual(self.original.to_dict(), original.to_dict())

    def test_optimized_reuses_identical_curve_and_caps(self):
        old, fast = self.original, self.fast
        self.assertTrue(fast.summary()['local_retiming']['applied'])
        self.assertLess(fast.duration, old.duration)
        self.assertEqual(fast.nodes, old.nodes)
        self.assertEqual(fast.durations, old.durations)
        np.testing.assert_array_equal(fast.coefficients, old.coefficients)
        for a, b in zip(old.events, fast.events):
            self.assertEqual({k: v for k, v in a.items() if k != 'elapsed_s'},
                             {k: v for k, v in b.items() if k != 'elapsed_s'})
            np.testing.assert_allclose(fast.sample(b['elapsed_s'])['position'], old.nodes[b['node_index']], atol=1e-8)
        samples = [fast.sample(t) for t in np.linspace(0., fast.duration, 2001)]
        self.assertLessEqual(max(np.linalg.norm(s['velocity']) for s in samples), 2.8+1e-7)
        self.assertLessEqual(max(np.linalg.norm(s['acceleration']) for s in samples), 3.8+1e-7)
        for t in (0., fast.duration):
            for key in ('velocity', 'acceleration'):
                np.testing.assert_allclose(fast.sample(t)[key], [0., 0., 0.], atol=1e-7)

    def test_preview_and_actual_preparation_are_identical(self):
        for mode in ('original', 'optimized'):
            c = config(mode)
            before = deepcopy(c)
            preview = route_preview(c, c['waypoint_route'], origin=self.ground, include_samples=True)
            actual = WaypointRouteEngine(c)._build_fixed_trajectory(self.ground)
            self.assertEqual(preview['trajectory'], actual.to_dict())
            self.assertEqual(c, before)

    def test_laps_and_gate_caps_survive_time_mapping(self):
        for laps in (1, 2, 9):
            c = config('optimized', laps, ['cruise', 'cruise', 'gate', 'gate', 'cruise'])
            engine = WaypointRouteEngine(c)
            trajectory = engine._build_fixed_trajectory(self.ground)
            self.assertEqual(trajectory.events[-1]['completed_laps'], laps)
            self.assertEqual(trajectory.end[:2], tuple(POINTS[0]))
            for event in trajectory.events:
                sample = trajectory.sample(event['elapsed_s'])
                np.testing.assert_allclose(sample['position'][:2], POINTS[event['waypoint_index']], atol=1e-7)
                if event['waypoint_index'] in (2, 3):
                    self.assertLessEqual(np.linalg.norm(sample['velocity']), .6+1e-7)
            for window in trajectory.summary()['gate_windows']:
                for t in np.linspace(window['start_s'], window['end_s'], 101):
                    self.assertLessEqual(np.linalg.norm(trajectory.sample(t)['velocity']), .6+1e-7)

    def test_same_location_and_duplicate_points_keep_existing_semantics(self):
        for points in ([[0., 0.]], [[0., 0.], [0., 0.]], POINTS+[POINTS[0]]):
            c = config('optimized', points=points)
            trajectory = WaypointRouteEngine(c)._build_fixed_trajectory((0., 0., 0.))
            self.assertEqual(trajectory.end[:2], tuple(points[0]))
            self.assertEqual(trajectory.events[-1]['completed_laps'], 0 if len(set(map(tuple, points)))==1 else 1)

    def test_optimizer_failure_keeps_original_and_is_visible(self):
        c = config('optimized')
        with patch.object(local_retiming, '_optimize', side_effect=ImportError('test missing dependency')):
            value = cli.preview(c, c['waypoint_route'], origin=self.ground)
            actual = WaypointRouteEngine(c)._build_fixed_trajectory(self.ground)
        detail = value['fixed_trajectory']['local_retiming']
        self.assertFalse(detail['applied'])
        self.assertEqual(actual.duration, self.original.duration)
        for t in np.linspace(0., actual.duration, 101):
            self.assertEqual(actual.sample(t), self.original.sample(t))
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):cli.show_preview(value)
        self.assertIn('局部配时未采用，沿用原参考', stream.getvalue())

    def test_all_ordinary_points_print_success_without_gate_metadata(self):
        value = cli.preview(self.fast_config, self.fast_config['waypoint_route'], origin=self.ground)
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):cli.show_preview(value)
        self.assertIn('局部配时已采用', stream.getvalue())
        self.assertIn('全部为普通点', stream.getvalue())
        self.assertFalse(any(value[k] for k in ('payload', 'visual', 'landing')))

    def test_serialized_reference_replays_without_optimizer(self):
        saved = json.loads(json.dumps(self.fast.to_dict(), allow_nan=False))
        with patch.object(local_retiming, '_optimize', side_effect=AssertionError('runtime must not optimize')):
            loaded = FixedTrajectory.from_dict(saved)
            for t in np.linspace(0., self.fast.duration, 101):
                self.assertEqual(loaded.sample(t), self.fast.sample(t))

    def test_optimized_lifecycle_finishes_one_lap_in_hold_without_payload(self):
        h = Replay(POINTS)
        e = h.engine
        e.config['navigation'].update(timing='optimized', laps=1)
        e.route_laps = 1
        e.start('waypoint_route', h.ground, h.yaw, h.now)
        e.native_origin = h.ground
        h.feedback(); e.begin_native_flight()
        prepared = e.fixed_trajectory
        self.assertTrue(prepared.summary()['local_retiming']['applied'])
        with patch.object(local_retiming, 'retime_loaded_leg', side_effect=AssertionError('already prepared')):
            h.begin_route()
            output = h.progress(prepared.duration)
            self.assertEqual(output['state'], 'COMPLETE')
            self.assertEqual(output['reference_source'], 'HOLD')
            self.assertEqual(output['waypoint_route']['completed_laps'], 1)
            self.assertEqual(output['waypoint_route']['timing_mode'], 'optimized')
            self.assertFalse(any(event['type'] in ('release', 'land', 'goal') for event in h.effects))
            e.hold('test operator')
            self.assertFalse(e.on_fixed_progress(prepared.sample(prepared.duration)))


class LoopEntryTests(unittest.TestCase):
    def test_both_launchers_force_one_lap_keep_parameters_and_only_preview(self):
        with tempfile.TemporaryDirectory(prefix='loop compare ') as temp:
            route = Path(temp)/'ordinary points.yaml'
            save_route(route, config()['waypoint_route'])
            field = ROOT/'flight/flight.yaml'
            before_field, before_route = field.read_bytes(), route.read_bytes()
            previews = {}
            for mode in ('original', 'optimized'):
                run = subprocess.run(['sh', str(ROOT/('flight/test_loop_'+mode+'.sh')),
                    '--route', str(route), '--set', 'navigation.laps=9', '--dry-run', '--json'],
                    env=dict(os.environ, AIRSEAFLY_PYTHON=sys.executable), capture_output=True, text=True)
                self.assertEqual(run.returncode, 0, run.stdout+run.stderr)
                value = json.loads(run.stdout)
                self.assertEqual(value['laps'], 1)
                self.assertEqual(value['timing_mode'], mode)
                self.assertEqual(value['backend'], 'fixed_min_snap')
                self.assertEqual(value['finish_action'], 'HOLD')
                self.assertFalse(any(value[key] for key in ('payload', 'visual', 'landing')))
                previews[mode] = value
            self.assertEqual(previews['original']['parameters'], previews['optimized']['parameters'])
            self.assertEqual(previews['original']['route_source'], previews['optimized']['route_source'])
            self.assertTrue(previews['optimized']['fixed_trajectory']['local_retiming']['applied'])
            self.assertEqual(before_field, field.read_bytes())
            self.assertEqual(before_route, route.read_bytes())


if __name__ == '__main__':
    unittest.main()
