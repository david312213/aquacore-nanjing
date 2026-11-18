"""Recorded-route lifecycle with the real fixed polynomial trajectory builder."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
from airseafly_real400_mission.configuration import load_config
from airseafly_real400_mission.native_waypoint_route import WaypointRouteEngine


def configuration(points, profiles=None):
    config = load_config()
    config['navigation'] = dict(backend='fixed_min_snap', laps=2)
    config['flight'].update(controller='px4_native', native_reference_mode='lio_direct',
                           native_task_feedback='lio', cruise_height_m=1.,
                           native_takeoff_speed_mps=.5, native_takeoff_acceleration_mps2=.25)
    config['waypoint_profiles'] = dict(cruise=dict(height_m=1., inflation_m=.2),
                                       gate=dict(height_m=.6, inflation_m=.1))
    config['waypoint_route'] = dict(format='aquacore_waypoints', version=1,
        frame='fast_lio_local_enu', points=deepcopy(points))
    if profiles is not None:
        config['waypoint_route'].update(version=2, point_profiles=profiles)
    return config


class Replay:
    def __init__(self, points, profiles=None):
        self.engine = WaypointRouteEngine(configuration(points, profiles))
        self.ground = (0., 0., -.7)
        self.now = 100.
        self.position = self.ground
        self.yaw = .4
        self.effects = []
        self.engine.start('waypoint_route', self.position, self.yaw, self.now)
        self.engine.native_origin = self.ground
        self.engine.begin_native_flight()
        self.feedback()

    def feedback(self):
        self.engine.on_native_lio_feedback(dict(stamp=self.now,
            position=self.position, velocity=(0., 0., 0.)))
        self.engine.on_native_feedback(dict(stamp=self.now, pose_stamp=self.now,
            position=tuple(v + 40 for v in self.position), velocity=(0., 0., 0.),
            yaw=self.yaw, owner='PX4_NATIVE', armed=True, offboard=True,
            mode='OFFBOARD', landed=False))

    def step(self, dt=.1):
        self.now += dt
        self.feedback()
        output = self.engine.tick(self.now, self.position, self.yaw, 'PX4_NATIVE')
        self.effects.extend(output['effects'])
        return output

    def begin_route(self):
        # Actual vehicle deliberately remains on the ground: takeoff completion
        # and fixed progress are reference based, as in the established flow.
        for _ in range(20):
            output = self.step(dt=1.)
            if self.engine.state == 'APPROACH':
                return output
        raise AssertionError('takeoff reference did not finish')

    def progress(self, elapsed):
        sample = self.engine.fixed_trajectory.sample(elapsed)
        self.engine.on_fixed_progress(sample)
        return self.step()


class FixedWaypointRouteTests(unittest.TestCase):
    def test_unknown_backend_rejected_instead_of_silently_selecting_ego(self):
        config = configuration([[1., 0.]])
        config['navigation']['backend'] = 'fixed_mni_snap'
        with self.assertRaisesRegex(ValueError, 'navigation.backend'):
            WaypointRouteEngine(config)

    def test_trajectory_is_prepared_before_takeoff_with_corrected_ground_rebuild(self):
        h = Replay([[2., 0.], [2., 2.]], ['cruise', 'gate'])
        e = h.engine
        self.assertEqual(e.state, 'TAKEOFF')
        initial = e.fixed_trajectory
        self.assertEqual(initial.sample(0.)['position'], (*h.ground[:2], h.ground[2] + 1.))
        e.begin_native_flight()
        self.assertIs(e.fixed_trajectory, initial)
        e.native_origin = (3., 4., -1.2)
        e.begin_native_flight()
        self.assertIsNot(e.fixed_trajectory, initial)
        self.assertEqual(e.fixed_trajectory.sample(0.)['position'], (3., 4., -1.2 + 1.))
        first_event = next(event for event in e.fixed_trajectory.events if event['waypoint_index'] == 1)
        self.assertAlmostEqual(e.fixed_trajectory.sample(first_event['elapsed_s'])['position'][2], -.6)
        takeoff = [effect for effect in e._effects if effect['type'] == 'takeoff']
        self.assertEqual(len(takeoff), 1)
        self.assertEqual(takeoff[0]['position_xyz'], [3., 4., -1.2 + 1.])

    def test_solver_error_cannot_queue_takeoff(self):
        e = WaypointRouteEngine(configuration([[1., 0.], [1., 1.]]))
        with patch.object(e, '_build_fixed_trajectory', side_effect=RuntimeError('library unavailable')):
            with self.assertRaisesRegex(RuntimeError, 'library unavailable'):
                e.start('waypoint_route', (0., 0., 0.), 0., 10.)
        self.assertEqual(e.state, 'IDLE')
        self.assertFalse(e._effects)

    def test_fixed_references_use_no_ego_or_arrival_dwell(self):
        h = Replay([[2., 0.], [2., 2.], [0., 2.]])
        output = h.begin_route()
        self.assertEqual((output['control'], output['reference_source'], output['desired_group']),
                         ('FIXED_TRAJECTORY', 'FIXED_TRAJECTORY', 'NONE'))
        self.assertEqual(output['native_command']['position'], (0., 0., h.ground[2] + 1.))
        first = h.engine.fixed_trajectory.events[0]
        output = h.progress(first['elapsed_s'])
        self.assertEqual(h.engine.fixed_completed_laps, 0)
        self.assertEqual(h.engine.route_completed_points, 1)
        self.assertEqual(output['telemetry']['arrival_metric'], 'reference_complete')
        self.assertNotEqual(output['waypoint_route']['actual_error_xyz'], [0., 0., 0.])
        self.assertFalse(any(effect['type'] in ('goal', 'waypoint_route_point_arrived', 'release')
                             for effect in h.effects))
        self.assertFalse(output['waypoint_route']['obstacle_avoidance'])
        self.assertFalse(output['waypoint_route']['ego_virtual_walls'])

    def test_all_crossed_events_are_emitted_once_and_two_laps_hold_first(self):
        h = Replay([[2., 0.], [2., 2.], [0., 2.]])
        h.begin_route()
        trajectory = h.engine.fixed_trajectory
        lap_one = next(event for event in trajectory.events if event['completed_laps'] == 1)
        output = h.progress(lap_one['elapsed_s'])
        self.assertEqual(h.engine.fixed_completed_laps, 1)
        self.assertFalse(output['done'])
        self.assertEqual(output['control'], 'FIXED_TRAJECTORY')
        self.assertGreater(sum(v*v for v in output['native_command']['velocity']), 0.)
        h.progress(lap_one['elapsed_s'])
        output = h.progress(trajectory.duration)
        passed = [effect for effect in h.effects if effect['type'] == 'waypoint_route_reference_passed']
        self.assertEqual([event['index'] for event in passed],
                         [event['waypoint_index'] for event in trajectory.events])
        self.assertEqual(output['waypoint_route']['completed_laps'], 2)
        self.assertEqual(output['waypoint_route']['completed_points'], 3)
        self.assertEqual(output['state'], 'COMPLETE')
        self.assertEqual(output['reference_source'], 'HOLD')
        self.assertIn('参考两圈完成', output['reason'])
        self.assertEqual(output['native_command']['position'], (2., 0., h.ground[2] + 1.))
        self.assertEqual(output['native_command']['velocity'], (0., 0., 0.))
        self.assertEqual(output['native_command']['yaw'], h.yaw)
        for _ in range(5):
            output = h.step(dt=100.)
            self.assertEqual(output['reference_source'], 'HOLD')
            self.assertEqual(output['native_command']['position'], tuple(trajectory.end))
            self.assertEqual(output['waypoint_route']['completed_laps'], 2)
        self.assertEqual(sum(effect['type'] == 'waypoint_route_fixed_complete' for effect in h.effects), 1)

    def test_manager_tick_never_resamples_or_restarts_runtime_progress(self):
        h = Replay([[2., 0.], [2., 2.]])
        h.begin_route()
        sample = h.engine.fixed_trajectory.sample(h.engine.fixed_trajectory.duration / 3.)
        h.engine.on_fixed_progress(sample)
        output = h.step()
        previous = deepcopy(output['native_command'])
        progress = output['waypoint_route']['elapsed_s']
        for _ in range(3):
            output = h.step(dt=500.)
            self.assertEqual(output['native_command'], previous)
            self.assertEqual(output['waypoint_route']['elapsed_s'], progress)
            self.assertFalse(output['done'])

    def test_reference_events_use_actual_position_from_same_runtime_sample(self):
        h = Replay([[2., 0.], [2., 2.]])
        h.begin_route()
        e = h.engine
        sample = e.fixed_trajectory.sample(e.fixed_trajectory.duration)
        sample['actual_position_xyz'] = [8., 9., 10.]
        self.assertNotEqual(tuple(sample['actual_position_xyz']), e.position)
        self.assertTrue(e.on_fixed_progress(sample))
        output = h.step()
        events = [event for event in output['effects'] if event['type'] in
                  ('waypoint_route_reference_passed', 'waypoint_route_fixed_complete')]
        self.assertTrue(events)
        for event in events:
            self.assertEqual(event['actual_position_xyz'], sample['actual_position_xyz'])

    def test_ego_acknowledgements_and_arrival_cannot_advance_fixed_route(self):
        h = Replay([[2., 0.], [2., 2.]])
        h.begin_route()
        e = h.engine
        original = e.snapshot()
        e.on_goal_received(1)
        e.on_goal_received()
        self.assertFalse(e._goal_sent)
        self.assertFalse(e._goal_acked)
        self.assertFalse(e._on_custom_arrival())
        self.assertEqual(e.snapshot(), original)

    def test_hold_revokes_trajectory_and_old_progress_cannot_resume(self):
        h = Replay([[2., 0.], [2., 2.]])
        h.begin_route()
        old = h.engine.fixed_trajectory.sample(h.engine.fixed_trajectory.duration)
        h.position = (1.1, 1.2, .7)
        h.feedback()
        h.engine.hold('operator hold')
        self.assertIsNone(h.engine.fixed_trajectory)
        self.assertFalse(h.engine.on_fixed_progress(old))
        for _ in range(5):
            output = h.step(dt=100.)
            self.assertEqual(output['state'], 'HOLD')
            self.assertEqual(output['native_command']['position'], h.position)
            self.assertEqual(output['reference_source'], 'HOLD')
        self.assertEqual(h.engine.fixed_completed_laps, 0)
        self.assertFalse(any(effect['type'] == 'waypoint_route_fixed_complete' for effect in h.effects))

    def test_new_start_clears_progress_and_preserves_takeoff_yaw(self):
        h = Replay([[2., 0.], [2., 2.]])
        h.begin_route()
        old = h.engine.fixed_trajectory
        h.progress(old.duration)
        h.engine.start('waypoint_route', (10., 20., 3.), .9, h.now + 1.)
        self.assertEqual(h.engine.state, 'TAKEOFF')
        self.assertIsNone(h.engine.route_index)
        self.assertEqual(h.engine.route_completed_points, 0)
        self.assertEqual(h.engine.fixed_completed_laps, 0)
        self.assertEqual(h.engine._fixed_event_index, 0)
        self.assertFalse(h.engine.done)
        self.assertIsNot(h.engine.fixed_trajectory, old)
        self.assertEqual(h.engine.fixed_trajectory.sample(0.)['position'], (10., 20., 4.))
        self.assertEqual(h.engine.command_yaw, .9)

    def test_single_point_is_reference_hold_without_fictitious_laps(self):
        h = Replay([[2., 0.]])
        h.begin_route()
        output = h.progress(h.engine.fixed_trajectory.duration)
        self.assertEqual(output['state'], 'COMPLETE')
        self.assertEqual(output['waypoint_route']['completed_laps'], 0)
        self.assertIn('单点参考完成', output['reason'])
        self.assertEqual(output['native_command']['position'], (2., 0., h.ground[2] + 1.))


if __name__ == '__main__':
    unittest.main()
