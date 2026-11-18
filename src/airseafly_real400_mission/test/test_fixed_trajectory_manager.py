"""Real fixed-route engine, manager and runtime, with inert ROS/processes."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np

from ros_fixtures import Bool, Clock, ROS, MANAGER, refresh_inputs
from airseafly_real400_mission.fixed_trajectory import FixedTrajectory
import test_native_waypoint_manager as manager_fixture


class FixedManagerTests(unittest.TestCase):
    setUp = manager_fixture.WaypointManagerTests.setUp
    setup_manager = manager_fixture.WaypointManagerTests.setup_manager
    dispatch_start = manager_fixture.WaypointManagerTests.dispatch_start
    activate = manager_fixture.WaypointManagerTests.activate
    step = manager_fixture.WaypointManagerTests.step
    until = manager_fixture.WaypointManagerTests.until

    def start_fixed(self):
        manager, runtime, replay = self.setup_manager([[13., -8.], [13., -7.], [12., -7.]])
        folder = tempfile.TemporaryDirectory(prefix='fixed-route-manager-')
        self.addCleanup(folder.cleanup)
        manager.log_dir = Path(folder.name)
        config = deepcopy(replay.engine.config)
        config['navigation'] = dict(backend='fixed_min_snap', laps=2)
        self.dispatch_start(manager, runtime, config)
        self.activate(manager, runtime)
        self.until(manager, runtime, lambda: runtime.reference_source == 'FIXED_TRAJECTORY')
        self.assertEqual(manager.engine.state, 'APPROACH')
        return manager, runtime

    def timer_step(self, manager, runtime, dt=.02):
        Clock.now += dt
        manager.odom_stamp = manager.planning_stamp = Clock.now
        refresh_inputs(runtime)
        runtime._full_tick(None)

    def test_no_ego_requests_and_manager_reads_timer_progress_without_resampling(self):
        manager, runtime = self.start_fixed()
        started = runtime.fixed_started_at
        for _ in range(10):
            self.timer_step(manager, runtime)
        progress = runtime.fixed_progress(manager)
        self.assertAlmostEqual(progress['elapsed_s'], .2)
        with mock.patch.object(runtime, '_sample_fixed_trajectory', wraps=runtime._sample_fixed_trajectory) as sample:
            for _ in range(10):
                manager.tick(None)
            sample.assert_not_called()
        self.assertEqual(runtime.fixed_started_at, started)
        self.assertEqual(manager.engine._fixed_progress, progress)
        self.assertEqual(manager.last_output['control'], 'FIXED_TRAJECTORY')
        self.assertEqual(set(manager.runner.requests), {'NONE'})
        self.assertIsNone(manager.ego_goal)
        self.assertEqual(manager.payload_records, [])
        self.assertEqual(manager.land_pub.messages, [])
        ROS.ServiceProxy.assert_not_called()
        runtime.rospy.ServiceProxy.assert_not_called()

    def test_sampler_already_advances_during_slow_start_announcement(self):
        manager, runtime, replay = self.setup_manager([[13., -8.], [13., -7.], [12., -7.]])
        folder = tempfile.TemporaryDirectory(prefix='fixed-start-before-log-')
        self.addCleanup(folder.cleanup)
        manager.log_dir = Path(folder.name)
        config = deepcopy(replay.engine.config)
        config['navigation'] = dict(backend='fixed_min_snap', laps=2)
        self.dispatch_start(manager, runtime, config)
        self.activate(manager, runtime)
        original_event = manager.event
        observed = []

        def slow_announcement(kind, **values):
            if kind == 'effect' and values['effect']['type'] == 'waypoint_route_fixed_started':
                self.assertEqual(runtime.reference_source, 'FIXED_TRAJECTORY')
                started = runtime.fixed_started_at
                self.timer_step(manager, runtime, .2)
                progress = runtime.fixed_progress(manager)
                self.assertAlmostEqual(progress['elapsed_s'], .2)
                self.assertNotEqual(tuple(progress['position']), tuple(manager.engine.fixed_trajectory.nodes[0]))
                self.assertEqual(runtime.fixed_started_at, started)
                observed.append(progress)
            original_event(kind, **values)

        manager.event = slow_announcement
        self.until(manager, runtime, lambda: bool(observed))
        self.assertEqual(len(observed), 1)
        self.assertEqual(manager.engine.state, 'APPROACH')
        self.assertFalse(any(kind == 'manager_error' for kind, _ in manager.events_seen))

    def test_delayed_manager_drains_two_laps_then_keeps_first_point_hold(self):
        manager, runtime = self.start_fixed()
        trajectory = manager.engine.fixed_trajectory
        self.timer_step(manager, runtime, trajectory.duration+1.)
        self.assertFalse(manager.engine.done)
        self.assertTrue(runtime.fixed_progress(manager)['completed'])
        self.assertEqual(runtime.last_reference_sent['position'], trajectory.end)
        self.assertEqual(runtime.last_reference_sent['velocity'], (0., 0., 0.))
        manager.tick(None)
        self.assertTrue(manager.engine.done)
        self.assertEqual(manager.engine.fixed_completed_laps, 2)
        self.assertEqual(manager.engine.phase, 'FIXED_TRAJECTORY_COMPLETE')
        self.assertFalse(manager.active)
        self.assertEqual(runtime.reference_source, 'HOLD')
        self.assertIsNone(runtime.fixed_trajectory)
        self.assertFalse(runtime.install_fixed_trajectory(manager, trajectory, manager.yaw))
        effects = [value['effect'] for kind, value in manager.events_seen if kind == 'effect']
        passed = [item for item in effects if item['type'] == 'waypoint_route_reference_passed']
        self.assertEqual([item['index'] for item in passed],
                         [item['waypoint_index'] for item in trajectory.events])
        self.assertEqual([item['completed_laps'] for item in passed],
                         [item['completed_laps'] for item in trajectory.events])
        self.assertEqual(sum(item['type'] == 'waypoint_route_fixed_complete' for item in effects), 1)
        before = len(passed)
        for _ in range(5):
            self.step(manager, runtime, follow=False, dt=10.)
            self.assertEqual(runtime.last_reference_sent['position'], trajectory.end)
        after = sum(kind == 'effect' and value['effect']['type'] == 'waypoint_route_reference_passed'
                    for kind, value in manager.events_seen)
        self.assertEqual(before, after)
        # Actual position has deliberately not tracked the reference endpoint.
        self.assertTrue(any(abs(value) > .1 for value in
                            manager.last_output['waypoint_route']['actual_error_xyz']))

    def test_actual_accepted_coefficients_and_route_identity_are_saved_once(self):
        manager, runtime = self.start_fixed()
        trajectory = manager.engine.fixed_trajectory
        paths = list(manager.log_dir.glob('fixed_trajectory-*.json'))
        self.assertEqual(len(paths), 1)
        saved = json.loads(paths[0].read_text())
        self.assertEqual(saved['task_id'], manager.task_id)
        self.assertEqual(saved['origin_xyz'], list(manager.engine.origin_xyz))
        self.assertEqual(saved['route'], manager.config['waypoint_route'])
        self.assertEqual(len(saved['route_sha256']), 64)
        restored = FixedTrajectory.from_dict(saved['trajectory'])
        for fraction in (0., .1, .5, .9, 1.):
            for field in ('position', 'velocity', 'acceleration', 'jerk'):
                np.testing.assert_array_equal(restored.sample(trajectory.duration*fraction)[field],
                                              trajectory.sample(trajectory.duration*fraction)[field])
        with mock.patch.object(manager, 'event', wraps=manager.event) as event:
            for _ in range(5):
                manager.tick(None)
            self.assertFalse(any(call.args[0] == 'fixed_trajectory_recorded' for call in event.call_args_list))

    def test_hold_and_priority_landing_revoke_without_any_automatic_resume(self):
        for landing in (False, True):
            with self.subTest(priority_landing=landing):
                manager, runtime = self.start_fixed()
                trajectory, engine = manager.engine.fixed_trajectory, manager.engine
                progress = runtime.fixed_progress(manager)
                if landing:
                    manager.on_priority_landing(Bool(True))
                else:
                    self.assertTrue(manager.dispatch(dict(command='hold'))['ok'])
                sent = len(runtime.publisher.messages)
                self.assertIsNone(runtime.fixed_trajectory)
                self.assertFalse(engine.on_fixed_progress(progress))
                self.assertFalse(runtime.install_fixed_trajectory(manager, trajectory, manager.yaw))
                self.timer_step(manager, runtime)
                if landing:
                    # Priority cancellation is effective before its next owner status.
                    self.assertEqual(len(runtime.publisher.messages), sent)
                    manager.on_priority_landing(Bool(False))
                manager.tick(None)
                self.assertEqual(manager.engine.state, 'HOLD')
                self.assertFalse(manager.active)
                self.assertEqual(manager.requested_group, 'NONE')

    def test_diagnostic_record_failure_does_not_add_a_flight_gate(self):
        with mock.patch.object(MANAGER, 'write_json', side_effect=OSError('read-only test filesystem')):
            manager, runtime = self.start_fixed()
        self.assertEqual(manager.engine.state, 'APPROACH')
        self.assertEqual(runtime.reference_source, 'FIXED_TRAJECTORY')
        self.assertTrue(any(kind == 'fixed_trajectory_record_failed' for kind, _ in manager.events_seen))
        self.assertFalse(any(kind == 'manager_error' for kind, _ in manager.events_seen))

    def test_completed_and_held_status_errors_follow_live_measurement_and_accepted_anchor(self):
        manager, runtime = self.start_fixed()
        trajectory = manager.engine.fixed_trajectory
        self.timer_step(manager, runtime, trajectory.duration+1.)
        manager.tick(None)
        self.assertFalse(manager.active)
        for hold in (False, True):
            measured = (20., 15., 2.) if not hold else (18., 16., 1.4)
            runtime.reference.feed(Clock.now, measured, (0., 0., 0.))
            if hold:
                self.assertTrue(manager.dispatch(dict(command='hold'))['ok'])
                measured = (18.2, 16.4, 1.5)
                runtime.reference.feed(Clock.now, measured, (0., 0., 0.))
            status = manager.status_unlocked()['output']
            target = runtime.reference_command['position']
            error = [a-b for a, b in zip(target, measured)]
            self.assertEqual(status['waypoint_route']['reference_position_xyz'], list(target))
            self.assertEqual(status['waypoint_route']['actual_error_xyz'], error)
            self.assertEqual(status['native']['error_xyz'], error)
            self.assertEqual(status['telemetry']['height_error_m'], error[2])
            self.assertEqual(status['telemetry']['height_target_z'], target[2])

    def test_manual_takeover_during_fixed_stream_stops_task_and_sampler(self):
        manager, runtime = self.start_fixed()
        trajectory = manager.engine.fixed_trajectory
        runtime.packet['manual_paused'] = True
        self.timer_step(manager, runtime)
        self.assertTrue(runtime.manual_takeover)
        self.assertIsNone(runtime.fixed_trajectory)
        self.assertEqual(manager.engine.state, 'HOLD')
        self.assertFalse(manager.active)
        sent = len(runtime.publisher.messages)
        runtime.packet.update(manual_paused=False, offboard=True)
        for _ in range(5):
            self.step(manager, runtime, follow=False)
        self.assertEqual(len(runtime.publisher.messages), sent)
        self.assertFalse(runtime.install_fixed_trajectory(manager, trajectory, manager.yaw))


if __name__ == '__main__':
    unittest.main()
