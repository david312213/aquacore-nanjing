"""Local timing contracts and flight handoff regression, entirely offline."""
from copy import deepcopy
import contextlib
import io
import json
import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from airseafly_real400_mission import local_retiming, native_delivery_route
from airseafly_real400_mission import delivery_route_cli
from airseafly_real400_mission.fixed_trajectory import FixedTrajectory, build_fixed_leg
from test_split_delivery_route import split_config
from test_native_fixed_delivery_route import FixedReplay, fixed_configuration
import test_fixed_trajectory_runtime as runtime_tests


class LocalTimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.points = [(0., 0., 1.), (3., 0., 1.), (3.5, .5, 1.3),
                      (4., 1., 1.3), (4.5, 1.5, 1.3), (4., 4., 1.), (0., 4., 1.)]
        ids = list(range(7))+[0]+list(range(1, 7))+[0]
        cls.caps = [.6 if i in (2, 3, 4) else None for i in ids]
        cls.groups = [('shared', i) for i in ids]
        cls.base = build_fixed_leg((-1., -1., 1.), [cls.points[i] for i in ids], 3.4, 4.1,
                                   cls.caps, [dict(itinerary_index=i) for i in range(len(ids))])
        cls.fast = local_retiming.retime_loaded_leg(cls.base, cls.caps, cls.groups, 3.4, 4.1)

    def test_same_curve_events_and_rest_endpoints(self):
        base, fast = self.base, self.fast
        self.assertTrue(fast.summary()['local_retiming']['applied'])
        self.assertLess(fast.duration, base.duration)
        self.assertEqual(fast.nodes, base.nodes)
        self.assertEqual(fast.durations, base.durations)
        np.testing.assert_array_equal(fast.coefficients, base.coefficients)
        for old, new in zip(base.events, fast.events):
            self.assertEqual({k: v for k, v in old.items() if k != 'elapsed_s'},
                             {k: v for k, v in new.items() if k != 'elapsed_s'})
            np.testing.assert_allclose(fast.sample(new['elapsed_s'])['position'],
                                       base.nodes[new['node_index']], atol=1e-10)
        self.assertTrue(all(a['elapsed_s'] <= b['elapsed_s'] for a, b in zip(fast.events, fast.events[1:])))
        for t in (0., fast.duration):
            for field in ('velocity', 'acceleration', 'jerk'):
                np.testing.assert_allclose(fast.sample(t)[field], (0.,)*3, atol=1e-10)

    def test_speed_acceleration_gate_and_join_continuity(self):
        fast = self.fast
        samples = [fast.sample(t) for t in np.linspace(0., fast.duration, 16001)]
        self.assertLessEqual(max(np.linalg.norm(s['velocity']) for s in samples), 3.4+1e-7)
        self.assertLessEqual(max(np.linalg.norm(s['acceleration']) for s in samples), 4.1+1e-7)
        for w in fast.summary()['gate_windows']:
            maximum = max(np.linalg.norm(fast.sample(t)['velocity'])
                          for t in np.linspace(w['start_s'], w['end_s'], 501))
            self.assertLessEqual(maximum, w['cap_mps']+1e-7)
        # Both waypoint knots and inverse-table cells must have C3 output.
        times = list(fast.retiming.elapsed[1:-1:13])+[e['elapsed_s'] for e in fast.events[:-1]]
        for t in times:
            before, after = fast.sample(t-1e-8), fast.sample(t+1e-8)
            for key in ('position', 'velocity', 'acceleration', 'jerk'):
                np.testing.assert_allclose(before[key], after[key], atol=2e-4)

    def test_time_inverse_and_derivatives_independent_finite_difference(self):
        fast = self.fast
        for tau in np.linspace(0., fast.base_duration, 351):
            t = fast.retiming.time_at_base(tau)
            self.assertAlmostEqual(fast.retiming.state(t)[0], tau, places=10)
        h = 1e-5
        for t in np.linspace(.01, fast.duration-.01, 151):
            before, center, after = fast.sample(t-h), fast.sample(t), fast.sample(t+h)
            for lower, upper in (('position', 'velocity'), ('velocity', 'acceleration'), ('acceleration', 'jerk')):
                estimated = (np.array(after[lower])-np.array(before[lower]))/(2.*h)
                np.testing.assert_allclose(estimated, center[upper], atol=5e-5)

    def test_saved_replay_requires_neither_scipy_nor_native_library(self):
        saved = json.loads(json.dumps(self.fast.to_dict(), allow_nan=False))
        with patch.dict(sys.modules, {'scipy': None, 'scipy.interpolate': None, 'scipy.optimize': None}), \
             patch.dict(os.environ, {'AIRSEAFLY_MIN_SNAP_LIBRARY': '/missing'}):
            replay = FixedTrajectory.from_dict(saved)
            for t in np.linspace(0., self.fast.duration+1., 201):
                self.assertEqual(self.fast.sample(t), replay.sample(t))
        self.assertEqual(self.fast.summary(), replay.summary())
        legacy = FixedTrajectory.from_dict(self.base.to_dict())
        for t in np.linspace(0., self.base.duration, 101):
            self.assertEqual(self.base.sample(t), legacy.sample(t))

    def test_optional_optimizer_failure_keeps_original_reference(self):
        for error in (ImportError('scipy unavailable'), ValueError('numerical failure')):
            with patch.object(local_retiming, '_optimize', side_effect=error):
                result = local_retiming.retime_loaded_leg(self.base, self.caps, self.groups, 3.4, 4.1)
            self.assertFalse(result.summary()['local_retiming']['applied'])
            self.assertEqual(result.events, self.base.events)
            for t in np.linspace(0., self.base.duration, 101):
                self.assertEqual(result.sample(t), self.base.sample(t))

    def test_coincident_short_vertical_and_isolated_gate_nodes(self):
        for offset in (0., 1e-8, .1):
            stops = [(0., 0., 1.), (offset, 0., 1.), (2., 0., 1.),
                     (2., 0., 1.3), (2., 2., 1.), (0., 0., 1.)]
            caps = [None, None, .3, None, None, None]
            base = build_fixed_leg((0., 0., 1.), stops, 2., 2., caps)
            result = local_retiming.retime_loaded_leg(base, caps, [('node', i) for i in range(len(stops))], 2., 2.)
            self.assertLessEqual(result.duration, base.duration)
            for e, cap in zip(result.events, caps):
                np.testing.assert_allclose(result.sample(e['elapsed_s'])['position'],
                                           base.nodes[e['node_index']], atol=1e-9)
                if cap:
                    self.assertLessEqual(np.linalg.norm(result.sample(e['elapsed_s'])['velocity']), cap+1e-7)
        still = build_fixed_leg((0., 0., 1.), [(0., 0., 1.)], 2., 2.)
        self.assertIs(local_retiming.retime_loaded_leg(still, [None], [('x', 0)], 2., 2.), still)


class LocalTimingIntegrationTests(unittest.TestCase):
    def test_preview_reports_local_timing_and_original_arrival_policy(self):
        config = split_config(2)
        config['flight'].update(arrival_radius_m=.2, arrival_dwell_s=.5)
        value = delivery_route_cli.preview(config, config['delivery_route'])
        self.assertTrue(value['fixed_legs']['legs'][0]['local_retiming']['applied'])
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            delivery_route_cli.show_preview(value)
        self.assertIn('局部时间分配：同一空间曲线', stream.getvalue())
        self.assertIn('XYZ 距离 0.20 m 内持续 0.5 s', stream.getvalue())

    def test_only_loaded_legs_change_and_preview_matches_flight(self):
        for laps in (1, 2, 9):
            for equal in (False, True):
                config = split_config(laps)
                if equal:
                    config['delivery_route']['route'][0]['xy'] = config['delivery_route']['loaded_route'][0]['xy'][:]
                before = deepcopy(config)
                steps = native_delivery_route.build_itinerary(config['delivery_route'], laps-1)
                with patch.object(local_retiming, 'retime_loaded_leg', side_effect=lambda trajectory, *a: trajectory):
                    old = native_delivery_route.plan_fixed_legs(config, steps, (0., 0., 0.))
                new = native_delivery_route.plan_fixed_legs(config, steps, (0., 0., 0.))
                self.assertEqual(config, before)
                self.assertEqual(len(old), len(new))
                for i, (a, b) in enumerate(zip(old, new)):
                    if i == 0 and laps > 1:
                        self.assertIn('local_retiming', b['trajectory'].summary())
                        self.assertLessEqual(b['trajectory'].duration, a['trajectory'].duration)
                    else:
                        self.assertEqual(a['trajectory'].to_dict(), b['trajectory'].to_dict())

    def test_takeoff_reuses_prepared_first_leg_without_resolving(self):
        h = FixedReplay(split_config(2))
        prepared = h.engine._prepared_fixed_first_leg['trajectory']
        with patch.object(native_delivery_route, 'build_leg', side_effect=AssertionError('unexpected replan')):
            h.until(lambda e: e.fixed_trajectory is not None, limit=5000)
        self.assertIs(h.engine.fixed_trajectory, prepared)

    def test_ground_registration_refreshes_reference_before_climb(self):
        h = FixedReplay(split_config(2))
        h.engine.native_origin = (h.ground[0]+.03, h.ground[1]-.02, h.ground[2]+.01)
        h.engine.begin_native_flight()
        prepared = h.engine._prepared_fixed_first_leg['trajectory']
        self.assertEqual(prepared.nodes[0], (*h.engine.native_origin[:2],
                         h.engine.native_origin[2]+h.engine.config['flight']['cruise_height_m']))
        with patch.object(native_delivery_route, 'build_leg', side_effect=AssertionError('unexpected replan')):
            h.until(lambda e: e.fixed_trajectory is not None, limit=5000)
        self.assertIs(h.engine.fixed_trajectory, prepared)

    def test_offset_arrival_still_uses_existing_xyz_policy_no_early_drop(self):
        h = FixedReplay(split_config(2))
        h.until(lambda e: e.phase == 'FIXED_TRAJECTORY', limit=5000)
        end = h.engine.fixed_trajectory.end
        h.pin = (end[0]+.5, end[1], end[2])
        h.until(lambda e: e.phase == 'FIXED_ARRIVAL', limit=10000)
        for _ in range(150):
            h.step()
        self.assertEqual(h.engine.phase, 'FIXED_ARRIVAL')
        self.assertFalse([e for e in h.effects if e['type'] == 'release'])
        h.pin = end
        h.until(lambda e: e.state == 'DROP', limit=100)
        self.assertFalse([e for e in h.effects if e['type'] == 'release'])


class LocalTimingRuntimeTests(unittest.TestCase):
    setUp = runtime_tests.FixedRuntimeTests.setUp
    feed_status = runtime_tests.FixedRuntimeTests.feed_status
    feed_px4 = runtime_tests.FixedRuntimeTests.feed_px4
    refresh_inputs = runtime_tests.FixedRuntimeTests.refresh_inputs
    advance = runtime_tests.FixedRuntimeTests.advance
    activate = runtime_tests.FixedRuntimeTests.activate

    def test_real_timer_publishes_retimed_pvaj_without_optimizer_or_hardware_calls(self):
        timer, _ = runtime_tests.FixedRuntimeTests.install(self)
        config = split_config(2)
        steps = native_delivery_route.build_itinerary(config['delivery_route'], 1)
        fast = native_delivery_route.plan_fixed_legs(config, steps, (0., 0., 0.))[0]['trajectory']
        self.runtime.clear_fixed_trajectory()
        self.engine.mode = 'delivery_route'
        self.engine.fixed_trajectory = fast
        fast.leg_sequence = 1
        self.assertTrue(self.runtime.install_fixed_trajectory(self.manager, fast, .7))
        self.assertEqual(timer.period, .02)
        with patch.object(local_retiming, '_optimize', side_effect=AssertionError('runtime must not optimize')):
            for i in range(1, 101):
                self.advance(.02)
                timer.step()
                expected = fast.sample(i*.02)
                for key in ('position', 'velocity', 'acceleration', 'jerk'):
                    np.testing.assert_allclose(self.runtime.last_reference_sent[key], expected[key], atol=1e-10)
        self.assertEqual(self.rpc_calls, [])


if __name__ == '__main__':
    unittest.main()
