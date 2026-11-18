"""Numerical contract of the fixed-route solver and native-free sampler."""
from pathlib import Path
import json
import math
import os
import sys
import unittest
from unittest.mock import patch

import numpy as np
from numpy.polynomial import polynomial as poly

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from airseafly_real400_mission.fixed_trajectory import build_fixed_trajectory, FixedTrajectory


def segment_maximum(coefficient, duration, derivative):
    """Separate companion-root implementation, not a grid or GCOPTER call."""
    rows = [poly.polyder(row, derivative)/duration**derivative for row in coefficient]
    squared = sum((poly.polymul(row, row) for row in rows), np.zeros(2*(7-derivative)+1))
    critical = poly.polyder(squared)
    roots = poly.polyroots(critical) if np.any(critical) else []
    times = [0., 1.] + [root.real for root in roots
                       if abs(root.imag) < 1e-7 and 0. <= root.real <= 1.]
    return max(np.linalg.norm([poly.polyval(unit_time, row) for row in rows]) for unit_time in times)


def independent_maximum(trajectory, derivative):
    return max((segment_maximum(c, d, derivative)
                for c, d in zip(trajectory.coefficients, trajectory.durations)), default=0.)


class FixedTrajectoryTests(unittest.TestCase):
    points = [(1., 0., 1.), (1., 1., 1.), (0., 1., 1.)]

    def build(self, points=None, start=(0., 0., 1.), velocity=.8, acceleration=1., laps=2):
        return build_fixed_trajectory(start, points or self.points, laps, velocity, acceleration)

    def test_whole_route_interpolates_twice_and_has_no_lap_stop(self):
        trajectory = self.build()
        self.assertEqual([e['waypoint_index'] for e in trajectory.events], [0, 1, 2, 0, 1, 2, 0])
        self.assertEqual([e['completed_laps'] for e in trajectory.events], [0, 0, 0, 1, 1, 1, 2])
        for event in trajectory.events:
            sample = trajectory.sample(event['elapsed_s'])
            np.testing.assert_allclose(sample['position'], self.points[event['waypoint_index']], atol=1e-9)
        midpoint = trajectory.events[3]['elapsed_s']
        self.assertGreater(np.linalg.norm(trajectory.sample(midpoint)['velocity']), .01)
        self.assertFalse(trajectory.sample(midpoint)['completed'])
        self.assertEqual(trajectory.sample(midpoint)['completed_laps'], 1)

    def test_pvaj_continuity_and_zero_endpoint_derivatives(self):
        trajectory = self.build(points=[(1.,0.,1.), (2.,1.,.6), (1.,2.,1.3)])
        for order in range(4):
            for index in range(len(trajectory.durations)-1):
                np.testing.assert_allclose(trajectory._sample_segment(index, 1., order),
                                           trajectory._sample_segment(index+1, 0., order), atol=1e-8)
        for order in (1, 2, 3):
            np.testing.assert_allclose(trajectory._sample_segment(0, 0., order), (0.,)*3, atol=1e-8)
            np.testing.assert_allclose(trajectory._sample_segment(len(trajectory.durations)-1, 1., order),
                                       (0.,)*3, atol=1e-8)

    def test_one_segment_matches_closed_form_minimum_snap(self):
        start = np.array((-.3, .2, 1.))
        end = np.array((1., 2., .5))
        trajectory = self.build(start=start, points=[end])
        expected = np.zeros((3, 8))
        expected[:, 0] = start
        expected[:, 4:] = (end-start)[:,None]*np.array([35., -84., 70., -20.])
        np.testing.assert_allclose(trajectory.coefficients[0], expected, atol=1e-9)

    def test_height_is_constant_or_monotone_for_every_segment(self):
        trajectory = self.build(points=[(1.,0.,1.), (2.,1.,.6), (1.,2.,1.3)])
        for index, (a, b) in enumerate(zip(trajectory.nodes, trajectory.nodes[1:])):
            heights = np.array([trajectory._sample_segment(index, u, 0)[2] for u in np.linspace(0,1,101)])
            self.assertGreaterEqual(heights.min(), min(a[2],b[2])-1e-12)
            self.assertLessEqual(heights.max(), max(a[2],b[2])+1e-12)
            if a[2] == b[2]:
                np.testing.assert_array_equal(heights, np.full_like(heights,a[2]))
            else:
                self.assertTrue(np.all(np.diff(heights)*np.sign(b[2]-a[2]) >= -1e-12))

    def test_exact_three_dimensional_velocity_and_acceleration_extrema(self):
        for velocity, acceleration in ((.8,1.), (2.,.2)):
            trajectory = self.build(points=[(.4,.2,.4), (1.,2.,1.5), (-1.,.3,.9)],
                                    velocity=velocity, acceleration=acceleration)
            actual_speed = independent_maximum(trajectory, 1)
            actual_acceleration = independent_maximum(trajectory, 2)
            self.assertLessEqual(actual_speed, velocity*(1.+1e-7))
            self.assertLessEqual(actual_acceleration, acceleration*(1.+1e-7))
            self.assertAlmostEqual(actual_speed, trajectory.summary()['max_velocity_mps'], places=7)
            self.assertAlmostEqual(actual_acceleration, trajectory.summary()['max_acceleration_mps2'], places=7)

    def test_duplicate_nodes_keep_original_events_without_zero_length_segments(self):
        points = [(1.,0.,1.), (1.,0.,1.), (1.,1.,1.), (1.,0.,1.)]
        trajectory = self.build(points=points)
        self.assertEqual(len(trajectory.durations), 5)
        self.assertTrue(all(value>0 for value in trajectory.durations))
        self.assertEqual([event['waypoint_index'] for event in trajectory.events], [0,1,2,3,0,1,2,3,0])
        self.assertEqual(trajectory.events[0]['elapsed_s'], trajectory.events[1]['elapsed_s'])
        self.assertEqual(trajectory.events[3]['elapsed_s'], trajectory.events[4]['elapsed_s'])
        self.assertEqual(trajectory.events[-1]['completed_laps'], 2)

    def test_single_location_does_not_invent_laps_or_movement(self):
        trajectory = self.build(points=[(1.,2.,1.)]*3)
        self.assertEqual(len(trajectory.durations), 1)
        self.assertEqual(trajectory.summary()['effective_laps'], 0)
        self.assertTrue(all(event['completed_laps']==0 for event in trajectory.events))
        with patch.dict(os.environ, {'AIRSEAFLY_MIN_SNAP_LIBRARY': '/does/not/exist'}):
            stationary = self.build(start=(1.,2.,1.), points=[(1.,2.,1.)])
        self.assertEqual(stationary.duration, 0.)
        self.assertEqual(stationary.sample(0.)['position'], (1.,2.,1.))
        self.assertTrue(stationary.sample(0.)['completed'])

    def test_after_completion_holds_first_point_without_third_lap(self):
        trajectory = self.build()
        for time in (trajectory.duration, trajectory.duration+100., 1e12):
            sample = trajectory.sample(time)
            self.assertTrue(sample['completed'])
            self.assertEqual(sample['completed_laps'], 2)
            self.assertEqual(sample['position'], self.points[0])
            for key in ('velocity', 'acceleration', 'jerk'):
                self.assertEqual(sample[key], (0.,)*3)

    def test_serialization_replays_exactly_without_native_library(self):
        trajectory = self.build()
        saved = json.loads(json.dumps(trajectory.to_dict(), allow_nan=False))
        with patch.dict(os.environ, {'AIRSEAFLY_MIN_SNAP_LIBRARY': '/does/not/exist'}):
            replay = FixedTrajectory.from_dict(saved)
            for elapsed in np.linspace(0.,trajectory.duration+10.,173):
                self.assertEqual(trajectory.sample(elapsed),replay.sample(elapsed))
        self.assertEqual(trajectory.summary(),replay.summary())

    def test_missing_library_is_reported_during_preparation(self):
        with patch.dict(os.environ, {'AIRSEAFLY_MIN_SNAP_LIBRARY': '/does/not/exist'}):
            with self.assertRaisesRegex(RuntimeError, 'build_fixed_trajectory.sh'):
                self.build()

    def test_small_spatial_scale_preserves_path_and_finds_extrema(self):
        scale=1e-8
        points=[tuple(scale*v for v in point) for point in self.points]
        trajectory=self.build(start=(0.,0.,scale),points=points)
        for event in trajectory.events:
            np.testing.assert_allclose(trajectory.sample(event['elapsed_s'])['position'],
                                       points[event['waypoint_index']],atol=scale*1e-7)
        self.assertGreater(trajectory.summary()['max_acceleration_mps2'], .99)

    def test_near_coincident_knot_stays_a_real_continuous_node(self):
        for gap in (1e-3, 1e-6, 1e-9):
            points=[(gap,0.,1.), (1.,1.,1.), (0.,1.,1.)]
            trajectory=self.build(points=points)
            self.assertEqual(len(trajectory.durations), 7)
            for index in range(len(trajectory.durations)-1):
                for order in range(4):
                    np.testing.assert_allclose(trajectory._sample_segment(index,1.,order),
                                               trajectory._sample_segment(index+1,0.,order),atol=1e-7)
            self.assertLess(trajectory.duration, 100.)
            self.assertLess(independent_maximum(trajectory,1), .8+1e-7)

    route = [(0.,0.,1.), (3.,0.,1.), (3.5,.5,1.3), (4.,1.,1.3), (4.5,1.5,1.3), (4.,4.,1.), (0.,4.,1.)]
    gates = [None, None, .15, .15, .15, None, None]

    def dense(self, trajectory, step=.005):
        return [trajectory.sample(t) for t in np.arange(0., trajectory.duration+step, step)]

    def test_unset_point_velocities_return_the_original_reference(self):
        plain = self.build(points=self.route, velocity=1., acceleration=.8)
        unset = build_fixed_trajectory((0.,0.,1.), self.route, 2, 1., .8, [None]*len(self.route))
        self.assertIsNone(unset.warp)
        self.assertEqual(plain.summary(), unset.summary())
        for elapsed in np.linspace(0., plain.duration, 97):
            self.assertEqual(plain.sample(elapsed), unset.sample(elapsed))

    def test_gate_cap_slows_only_gate_windows_on_the_same_path(self):
        plain = self.build(points=self.route, velocity=1., acceleration=.8)
        limited = build_fixed_trajectory((0.,0.,1.), self.route, 2, 1., .8, self.gates)
        summary = limited.summary()
        self.assertTrue(summary['time_warp'])
        self.assertEqual(len(summary['gate_windows']), 2)
        self.assertEqual(limited.durations, plain.durations)
        np.testing.assert_array_equal(limited.coefficients, plain.coefficients)
        for event in limited.events:
            sample = limited.sample(event['elapsed_s'])
            np.testing.assert_allclose(sample['position'], self.route[event['waypoint_index']], atol=1e-9)
            if self.gates[event['waypoint_index']] is not None:
                self.assertLessEqual(np.linalg.norm(sample['velocity']), .15*(1.+1e-7))
        samples = self.dense(limited)
        speed = [np.linalg.norm(s['velocity']) for s in samples]
        self.assertLessEqual(max(speed), 1.*(1.+1e-7))
        self.assertLessEqual(max(np.linalg.norm(s['acceleration']) for s in samples), .8*(1.+1e-6))
        self.assertLessEqual(max(speed), summary['max_velocity_mps']*(1.+1e-7))
        inside = [v for s, v in zip(samples, speed) if any(
            w['start_s'] <= s['elapsed_s'] <= w['end_s'] for w in summary['gate_windows'])]
        self.assertTrue(inside)
        self.assertLessEqual(max(inside), .15*(1.+1e-7))
        self.assertGreater(max(inside), .15*.99)
        # The open sections keep the route speed.
        self.assertGreater(max(speed), .9)
        self.assertGreater(limited.duration, plain.duration)

    def test_warped_derivatives_are_consistent_and_continuous(self):
        limited = build_fixed_trajectory((0.,0.,1.), self.route, 1, 1., .8, self.gates)
        h = 1e-4
        for t in np.linspace(h, limited.duration-h, 211):
            before, now, after = (limited.sample(t+d) for d in (-h, 0., h))
            for low, high in (('position', 'velocity'), ('velocity', 'acceleration'),
                              ('acceleration', 'jerk')):
                np.testing.assert_allclose((np.array(after[low])-np.array(before[low]))/(2*h),
                                           now[high], rtol=1e-4, atol=1e-5)
        for piece in limited.warp['pieces']:
            t = limited.warp['scale']*piece['s0']
            if t > h:
                for key in ('position', 'velocity', 'acceleration'):
                    np.testing.assert_allclose(limited.sample(t-1e-9)[key], limited.sample(t+1e-9)[key],
                                               atol=1e-6)

    def test_warped_reference_replays_and_holds_after_completion(self):
        limited = build_fixed_trajectory((0.,0.,1.), self.route, 2, 1., .8, self.gates)
        saved = json.loads(json.dumps(limited.to_dict(), allow_nan=False))
        with patch.dict(os.environ, {'AIRSEAFLY_MIN_SNAP_LIBRARY': '/does/not/exist'}):
            replay = FixedTrajectory.from_dict(saved)
            for elapsed in np.linspace(0., limited.duration+5., 151):
                self.assertEqual(limited.sample(elapsed), replay.sample(elapsed))
        end = limited.sample(limited.duration+100.)
        self.assertTrue(end['completed'])
        self.assertEqual(end['position'], self.route[0])
        self.assertEqual(end['velocity'], (0.,)*3)

    def test_single_gate_point_caps_speed_through_it(self):
        gates = [None, None, None, .15, None, None, None]
        limited = build_fixed_trajectory((0.,0.,1.), self.route, 1, 1., .8, gates)
        self.assertTrue(limited.summary()['time_warp'])
        for event in limited.events:
            if event['waypoint_index'] == 3:
                self.assertLessEqual(np.linalg.norm(limited.sample(event['elapsed_s'])['velocity']),
                                     .15*(1.+1e-7))

    def test_gate_cap_at_or_above_actual_speed_has_no_effect(self):
        plain = self.build(points=self.route, velocity=1., acceleration=.8)
        loose = build_fixed_trajectory((0.,0.,1.), self.route, 2, 1., .8,
                                       [None, None, 1., 1.5, 1., None, None])
        self.assertIsNone(loose.warp)
        self.assertEqual(plain.summary(), loose.summary())

    def test_gate_velocity_configuration_maps_only_gate_points(self):
        from airseafly_real400_mission.route_profiles import gate_velocity, point_velocities
        names = ['cruise', 'gate', 'gate', 'cruise']
        self.assertEqual(point_velocities(dict(planning=dict(gate_velocity_mps=None)), names), [None]*4)
        self.assertEqual(point_velocities(dict(planning={}), names), [None]*4)
        self.assertEqual(point_velocities(dict(planning=dict(gate_velocity_mps=.4)), names),
                         [None, .4, .4, None])
        for value in (0, -1., math.nan, True, '0.4'):
            with self.assertRaises(ValueError):
                gate_velocity(dict(planning=dict(gate_velocity_mps=value)))

    def test_invalid_point_velocities_are_rejected_before_solving(self):
        for values in ([.3], [None, None, 0., None, None, None, None],
                       [None, None, math.nan, None, None, None, None],
                       [None, None, True, None, None, None, None]):
            with self.assertRaises(ValueError):
                build_fixed_trajectory((0.,0.,1.), self.route, 1, 1., .8, values)


if __name__ == '__main__':
    unittest.main()
