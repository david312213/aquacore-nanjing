"""Thrust-direction budgets for local timing; entirely offline (prototype)."""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from airseafly_real400_mission import local_retiming
from airseafly_real400_mission.fixed_trajectory import FixedTrajectory, build_fixed_leg
from airseafly_real400_mission.native_waypoint_route import WaypointRouteEngine
from test_native_fixed_waypoint_route import configuration

G = local_retiming.GRAVITY
DYN = local_retiming.dynamics_limits({'retiming': {}})


def budgets(trajectory, dynamics, step=.002):
    """Independent check from runtime samples (chain-rule outputs), not the optimizer."""
    samples = [trajectory.sample(t) for t in np.arange(0., trajectory.duration, step)]
    v = np.array([s['velocity'] for s in samples])
    a = np.array([s['acceleration'] for s in samples])
    j = np.array([s['jerk'] for s in samples])
    k = dynamics['drag_per_s']
    f = a+k*v
    f[:, 2] += G
    fdot = j+k*a
    tilt = np.degrees(np.arctan2(np.hypot(f[:, 0], f[:, 1]), f[:, 2]))
    rate = np.degrees(np.linalg.norm(np.cross(f, fdot), axis=1)/np.sum(f*f, axis=1))
    return dict(tilt=tilt.max(), rate=rate.max(), thrust=np.linalg.norm(f, axis=1).max()/G,
                lift=f[:, 2].min()/G, speed=np.linalg.norm(v, axis=1).max(),
                acceleration=np.linalg.norm(a, axis=1).max())


class DynamicsLimitTests(unittest.TestCase):
    def test_absent_or_null_parameters_use_defaults(self):
        for planning in ({}, {'retiming': None}, None):
            self.assertEqual(local_retiming.dynamics_limits(planning), local_retiming.DYNAMICS_DEFAULTS)

    def test_timing_mode_selection(self):
        self.assertEqual(local_retiming.timing_mode({}), 'original')
        self.assertEqual(local_retiming.timing_mode({'navigation': {'timing': None}}, 'optimized'), 'optimized')
        for mode in local_retiming.TIMING_MODES:
            self.assertEqual(local_retiming.timing_mode({'navigation': {'timing': mode}}), mode)
        with self.assertRaises(ValueError):
            local_retiming.timing_mode({'navigation': {'timing': 'fast'}})
        config = {'planning': {'retiming': {'feedforward_tilt_deg': 28}}}
        self.assertIsNone(local_retiming.timing_dynamics(config, 'optimized'))
        self.assertIsNone(local_retiming.timing_dynamics(config, 'original'))
        self.assertEqual(local_retiming.timing_dynamics(config, 'thrust')['feedforward_tilt_deg'], 28.)

    def test_defaults_partial_override_and_null_entries(self):
        self.assertEqual(DYN, local_retiming.DYNAMICS_DEFAULTS)
        value = local_retiming.dynamics_limits({'retiming': {'feedforward_tilt_deg': 25, 'drag_per_s': None}})
        self.assertEqual(value['feedforward_tilt_deg'], 25.)
        self.assertEqual(value['drag_per_s'], local_retiming.DYNAMICS_DEFAULTS['drag_per_s'])

    def test_invalid_values_raise_at_preparation(self):
        for bad in ({'unknown': 1}, {'feedforward_tilt_deg': 70}, {'feedforward_tilt_deg': True},
                    {'tilt_rate_dps': 0}, {'drag_per_s': -.1}, {'max_thrust_g': .9},
                    {'min_thrust_g': 1.2}, {'smoothing': float('nan')}):
            with self.assertRaises(ValueError):
                local_retiming.dynamics_limits({'retiming': bad})
        with self.assertRaises(ValueError):
            local_retiming.dynamics_limits({'retiming': 30})


class DynamicsTimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Tight 1 m loops plus long straights, like the field route.
        cls.points = [(0., 0., 1.4), (0., 3.2, 1.4), (-1.2, 3.8, 1.4), (-2.2, 3.4, 1.4),
                      (-2.1, 1.5, 1.4), (-.5, 1.6, 1.4), (-2.2, -2.9, 1.4), (1.4, -2.9, 1.4),
                      (1.6, -2.9, 1.4), (2.7, -2.0, 1.4), (2.4, 3.5, 1.4), (0., 0., 1.4)]
        cls.caps = [None]*7+[3., 3.]+[None]*3
        cls.groups = [('waypoint', i % 11) for i in range(len(cls.points))]
        cls.base = build_fixed_leg((0., 0., 1.4), cls.points, 6., 8., cls.caps)
        cls.old = local_retiming.retime_loaded_leg(cls.base, cls.caps, cls.groups, 6., 8.)
        cls.new = local_retiming.retime_loaded_leg(cls.base, cls.caps, cls.groups, 6., 8., DYN)

    def test_none_is_exactly_the_deployed_optimizer(self):
        again = local_retiming.retime_loaded_leg(self.base, self.caps, self.groups, 6., 8., None)
        self.assertEqual(again.to_dict(), self.old.to_dict())
        self.assertNotIn('constraint_model', self.old.summary()['local_retiming'])

    def test_all_budgets_hold_between_samples(self):
        self.assertTrue(self.new.summary()['local_retiming']['applied'])
        self.assertEqual(self.new.summary()['local_retiming']['constraint_model'], 'thrust_direction')
        b = budgets(self.new, DYN)
        self.assertLessEqual(b['tilt'], DYN['feedforward_tilt_deg']+.05)
        self.assertLessEqual(b['rate'], DYN['tilt_rate_dps']+.5)
        self.assertLessEqual(b['thrust'], DYN['max_thrust_g']+1e-3)
        self.assertGreaterEqual(b['lift'], DYN['min_thrust_g']-1e-3)
        self.assertLessEqual(b['speed'], 6.+1e-6)
        self.assertLessEqual(b['acceleration'], 8.+1e-6)
        for w in self.new.summary()['gate_windows']:
            peak = max(np.linalg.norm(self.new.sample(t)['velocity'])
                       for t in np.linspace(w['start_s'], w['end_s'], 401))
            self.assertLessEqual(peak, w['cap_mps']+1e-6)

    def test_reduces_the_old_peaks_while_beating_original_timing(self):
        old, new = budgets(self.old, DYN), budgets(self.new, DYN)
        self.assertGreater(old['tilt'], DYN['feedforward_tilt_deg'])
        self.assertGreater(old['rate'], DYN['tilt_rate_dps'])
        self.assertLess(self.new.duration, self.base.duration)
        self.assertGreater(self.new.duration, self.old.duration)

    def test_same_curve_events_rest_ends_and_c3_joins(self):
        base, new = self.base, self.new
        self.assertEqual(new.nodes, base.nodes)
        np.testing.assert_array_equal(new.coefficients, base.coefficients)
        for e in new.events:
            np.testing.assert_allclose(new.sample(e['elapsed_s'])['position'], base.nodes[e['node_index']], atol=1e-9)
        for t in (0., new.duration):
            for field in ('velocity', 'acceleration', 'jerk'):
                np.testing.assert_allclose(new.sample(t)[field], (0.,)*3, atol=1e-9)
        for t in [e['elapsed_s'] for e in new.events[:-1]]:
            before, after = new.sample(t-1e-8), new.sample(t+1e-8)
            for key in ('position', 'velocity', 'acceleration', 'jerk'):
                np.testing.assert_allclose(before[key], after[key], atol=2e-4)

    def test_near_duplicate_recorded_points_stay_local(self):
        # Field delivery routes contain points recorded ~2 mm apart.
        points = [(0., 0., 1.4), (3., 0., 1.4), (3., 3., 1.4), (3.002, 3., 1.4),
                  (3.002, 3.002, 1.4), (0., 3., 1.4), (0., 0., 1.4)]
        caps = [None]*len(points)
        groups = [('shared', i % 6) for i in range(len(points))]
        base = build_fixed_leg((0., 0., 1.4), points, 6., 8., caps)
        result = local_retiming.retime_loaded_leg(base, caps, groups, 6., 8., DYN)
        detail = result.summary()['local_retiming']
        self.assertTrue(detail['solver_converged'])
        self.assertNotIn('solver_fallback', detail)
        self.assertLess(detail['dynamics_normalization'], 1.05)
        self.assertLess(result.duration, 1.5*base.duration)
        b = budgets(result, DYN)
        self.assertLessEqual(b['tilt'], DYN['feedforward_tilt_deg']+.05)
        self.assertLessEqual(b['rate'], DYN['tilt_rate_dps']+.5)

    def test_infeasible_solver_stop_keeps_feasible_uniform_start(self):
        import scipy.optimize
        real = scipy.optimize.minimize

        def broken(fun, x0, **kw):
            value = real(fun, x0, **kw)
            value.x = value.x+3.
            return value
        with patch.object(scipy.optimize, 'minimize', side_effect=broken):
            result = local_retiming.retime_loaded_leg(self.base, self.caps, self.groups, 6., 8., DYN)
        detail = result.summary()['local_retiming']
        self.assertEqual(detail['solver_fallback'], 'uniform_slowdown')
        b = budgets(result, DYN)
        self.assertLessEqual(b['tilt'], DYN['feedforward_tilt_deg']+.05)
        self.assertLessEqual(b['rate'], DYN['tilt_rate_dps']+.5)

    def test_tighter_budgets_are_slower(self):
        slow = local_retiming.retime_loaded_leg(self.base, self.caps, self.groups, 6., 8.,
            dict(DYN, feedforward_tilt_deg=20., tilt_rate_dps=80.))
        self.assertGreater(slow.duration, self.new.duration)
        b = budgets(slow, dict(DYN, feedforward_tilt_deg=20., tilt_rate_dps=80.))
        self.assertLessEqual(b['tilt'], 20.05)
        self.assertLessEqual(b['rate'], 80.5)

    def test_replay_without_scipy_and_failure_fallback(self):
        saved = json.loads(json.dumps(self.new.to_dict(), allow_nan=False))
        with patch.dict(sys.modules, {'scipy': None, 'scipy.interpolate': None, 'scipy.optimize': None}), \
             patch.dict(os.environ, {'AIRSEAFLY_MIN_SNAP_LIBRARY': '/missing'}):
            replay = FixedTrajectory.from_dict(saved)
            for t in np.linspace(0., self.new.duration, 101):
                self.assertEqual(self.new.sample(t), replay.sample(t))
        with patch.object(local_retiming, '_optimize', side_effect=ValueError('numerical failure')):
            result = local_retiming.retime_loaded_leg(self.base, self.caps, self.groups, 6., 8., DYN)
        self.assertFalse(result.summary()['local_retiming']['applied'])
        self.assertEqual(result.to_dict()['events'], self.base.to_dict()['events'])

    def test_report_matches_independent_budget_check(self):
        samples = [self.new.sample(t) for t in np.arange(0., self.new.duration, .002)]
        report = local_retiming.dynamics_report(samples, DYN)
        b = budgets(self.new, DYN)
        self.assertAlmostEqual(report['tilt_max_deg'], b['tilt'], places=6)
        self.assertAlmostEqual(report['tilt_rate_max_dps'], b['rate'], places=6)
        self.assertEqual(report['tilt_over_fraction'], 0.)


class WaypointConfigTests(unittest.TestCase):
    def test_navigation_timing_selects_the_optimizer(self):
        value = configuration([[0., 0.], [4., 0.], [4., 3.], [0., 3.]])
        value['navigation'].update(laps=1)
        value['planning'].update(max_velocity_mps=6., max_acceleration_mps2=8.,
                                 retiming={'feedforward_tilt_deg': 28})
        seen = []
        real = local_retiming.retime_loaded_leg

        def spy(*args):
            seen.append(args[-1])
            return real(*args)
        results = {}
        with patch.object(local_retiming, 'retime_loaded_leg', side_effect=spy):
            for mode in ('original', 'optimized', 'thrust'):
                value['navigation']['timing'] = mode
                results[mode] = WaypointRouteEngine(deepcopy(value))._build_fixed_trajectory((0., 0., 0.))
        self.assertEqual(len(seen), 2)
        self.assertIsNone(seen[0])
        self.assertEqual(seen[1]['feedforward_tilt_deg'], 28.)
        self.assertNotIn('local_retiming', results['original'].summary())
        self.assertNotIn('constraint_model', results['optimized'].summary()['local_retiming'])
        self.assertEqual(results['thrust'].summary()['local_retiming']['constraint_model'], 'thrust_direction')

    def test_cli_accepts_thrust(self):
        from airseafly_real400_mission import waypoint_route_cli as cli
        self.assertEqual(cli.parser().parse_args(['--timing', 'thrust']).timing, 'thrust')
        self.assertIsNone(cli.parser().parse_args([]).timing)


class DeliveryConfigTests(unittest.TestCase):
    def test_delivery_mode_selection_and_legacy_switch(self):
        from airseafly_real400_mission.native_delivery_route import delivery_timing_mode
        from test_split_delivery_route import split_config
        config = split_config(2)
        config.setdefault('navigation', {}).pop('timing', None)
        self.assertEqual(delivery_timing_mode(config), 'optimized')
        for mode in ('original', 'optimized', 'thrust'):
            config['navigation']['timing'] = mode
            self.assertEqual(delivery_timing_mode(config), mode)
        config['delivery_route_options']['local_retiming_enabled'] = False
        self.assertEqual(delivery_timing_mode(config), 'original')

    def test_delivery_loaded_leg_uses_budgets_and_preview_reports_them(self):
        import contextlib
        import io
        from airseafly_real400_mission import delivery_route_cli
        from test_split_delivery_route import split_config
        config = split_config(2)
        config.setdefault('navigation', {})['timing'] = 'thrust'
        config['planning']['retiming'] = {'feedforward_tilt_deg': 30}
        value = delivery_route_cli.preview(config, config['delivery_route'])
        first = value['fixed_legs']['legs'][0]['local_retiming']
        self.assertTrue(first['applied'])
        self.assertEqual(first['constraint_model'], 'thrust_direction')
        self.assertLessEqual(first['dynamics_peak_ratios']['tilt'], 1.)
        for leg in value['fixed_legs']['legs'][1:]:
            self.assertIsNone(leg.get('local_retiming'))
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            delivery_route_cli.show_preview(value)
        self.assertIn('推力方向约束：前馈倾角 ≤ 30°', stream.getvalue())
        self.assertIn('绕圈配时版本：局部配时（推力方向约束）', stream.getvalue())


if __name__ == '__main__':
    unittest.main()
