"""The real 50 Hz adapter with inert ROS: no sockets, RPCs or arm requests."""
from copy import deepcopy
import json
import unittest

import numpy as np

import test_native_full_runtime as runtime_fixture


class AnalyticTrajectory:
    duration = 2.
    end = (2., 2., 3.)
    events = []

    def __init__(self):
        self.calls = []

    def sample(self, elapsed):
        self.calls.append(elapsed)
        s = min(1., max(0., elapsed/self.duration))
        p = 35*s**4-84*s**5+70*s**6-20*s**7
        v = (140*s**3-420*s**4+420*s**5-140*s**6)/self.duration
        a = (420*s**2-1680*s**3+2100*s**4-840*s**5)/self.duration**2
        j = (840*s-5040*s**2+8400*s**3-4200*s**4)/self.duration**3
        return dict(position=(1.+p, 2., 3.), velocity=(v, 0., 0.),
                    acceleration=(a, 0., 0.), jerk=(j, 0., 0.),
                    elapsed_s=elapsed, completed=elapsed >= self.duration,
                    segment_index=0)


class FixedRuntimeTests(unittest.TestCase):
    # Reuse the established inert ROS adapter fixture, not its EGO test cases.
    setUp = runtime_fixture.RuntimeTests.setUp
    feed_status = runtime_fixture.RuntimeTests.feed_status
    feed_px4 = runtime_fixture.RuntimeTests.feed_px4
    refresh_inputs = runtime_fixture.RuntimeTests.refresh_inputs
    advance = runtime_fixture.RuntimeTests.advance
    activate = runtime_fixture.RuntimeTests.activate

    def install(self):
        self.manager.task_id = 'fixed-task'
        timer = self.activate()
        self.engine.state, self.engine.mode = 'APPROACH', 'waypoint_route'
        trajectory = self.engine.fixed_trajectory = AnalyticTrajectory()
        self.assertTrue(self.runtime.install_fixed_trajectory(self.manager, trajectory, .7))
        return timer, trajectory

    def test_timer_analytically_samples_pva_at_50_hz_with_constant_yaw(self):
        timer, trajectory = self.install()
        self.assertEqual(timer.period, .02)
        for i in range(1, 26):
            self.advance(.02)
            timer.step()
            expected = trajectory.sample(i*.02)
            command = self.runtime.last_reference_sent
            for key in ('position', 'velocity', 'acceleration', 'jerk'):
                np.testing.assert_allclose(command[key], expected[key], atol=1e-12)
            self.assertEqual((command['yaw'], command['yaw_rate']), (.7, 0.))
            self.assertEqual(command['reference_source'], 'FIXED_TRAJECTORY')
            self.assertEqual(command['source_stamp'], self.now)
            self.assertEqual(command['trajectory_id'], 'fixed-task:%s' % self.runtime.start_epoch)
            self.assertAlmostEqual(command['trajectory_elapsed_s'], i*.02)
            self.assertEqual((command['completed_laps'], command['lap']), (0, 0))
            self.assertEqual(self.sent[-1].type_mask, 0)
        self.assertEqual(self.rpc_calls, [])
        row = json.loads(self.runtime.trace.getvalue().splitlines()[-1])
        self.assertAlmostEqual(row['fixed_trajectory']['elapsed_s'], .5)
        self.assertEqual(row['fixed_trajectory']['task_id'], 'fixed-task')

    def test_repeated_manager_installs_and_progress_reads_do_not_resample_or_restart(self):
        timer, trajectory = self.install()
        self.advance(.4)
        timer.step()
        progress = self.runtime.fixed_progress(self.manager)
        calls, started = len(trajectory.calls), self.runtime.fixed_started_at
        self.advance(.3)
        for _ in range(20):
            self.assertTrue(self.runtime.install_fixed_trajectory(self.manager, trajectory, 1.2))
            self.assertEqual(self.runtime.fixed_progress(self.manager), progress)
        self.assertEqual(len(trajectory.calls), calls)
        self.assertEqual(self.runtime.fixed_started_at, started)
        timer.step()
        self.assertAlmostEqual(self.runtime.fixed_progress(self.manager)['elapsed_s'], .7)
        self.assertEqual(self.runtime.last_reference_sent['yaw'], .7)

    def test_delayed_manager_and_backward_ros_time_keep_endpoint_without_third_lap(self):
        timer, trajectory = self.install()
        self.advance(.8)
        timer.step()
        self.now -= .4
        timer.step()
        self.assertAlmostEqual(self.runtime.fixed_progress(self.manager)['elapsed_s'], .8)
        self.advance(10.)
        timer.step()
        first = deepcopy(self.runtime.fixed_progress(self.manager))
        for _ in range(10):
            self.advance(1.)
            timer.step()
            command = self.runtime.last_reference_sent
            self.assertEqual(command['position'], trajectory.end)
            self.assertEqual(command['velocity'], (0., 0., 0.))
            self.assertEqual(command['acceleration'], (0., 0., 0.))
            self.assertEqual(command['jerk'], (0., 0., 0.))
        self.assertTrue(first['completed'])
        self.assertEqual(self.runtime.fixed_progress(self.manager)['elapsed_s'], trajectory.duration)

    def test_hold_revokes_identity_even_before_engine_observes_hold(self):
        timer, trajectory = self.install()
        self.advance(.4)
        timer.step()
        self.runtime.hold_current()
        self.assertIsNone(self.runtime.fixed_trajectory)
        self.assertFalse(self.runtime.install_fixed_trajectory(self.manager, trajectory, .7))
        self.advance(.4)
        timer.step()
        self.assertEqual(self.runtime.last_reference_sent['reference_source'], 'HOLD')
        self.assertEqual(self.runtime.last_reference_sent['position'], (1., 2., 3.))

    def test_delivery_legs_install_after_drop_reference_even_with_a_reused_object(self):
        self.manager.task_id = 'delivery-task'
        timer = self.activate()
        self.engine.state, self.engine.mode = 'APPROACH', 'delivery_route'
        leg = self.engine.fixed_trajectory = AnalyticTrajectory()
        leg.leg_sequence = 1
        self.assertTrue(self.runtime.install_fixed_trajectory(self.manager, leg, .7))
        self.advance(.4)
        timer.step()
        self.assertEqual(self.runtime.last_reference_sent['reference_source'], 'FIXED_TRAJECTORY')
        # The drop's native reference revokes leg 1; leg 1 itself stays revoked.
        self.runtime.set_reference(dict(position=(3., 2., 3.), velocity=(0., 0., 0.), yaw=.7), 'ALIGN')
        self.assertFalse(self.runtime.install_fixed_trajectory(self.manager, leg, .7))
        # The next leg may land on the freed object's id; its sequence differs.
        leg.leg_sequence = 2
        self.assertTrue(self.runtime.install_fixed_trajectory(self.manager, leg, .7))
        self.advance(.4)
        timer.step()
        self.assertEqual(self.runtime.last_reference_sent['reference_source'], 'FIXED_TRAJECTORY')
        # Both drop tests fly fixed legs too; other modes cannot install one.
        self.engine.mode = 'drop_points'
        leg.leg_sequence = 3
        self.assertTrue(self.runtime.install_fixed_trajectory(self.manager, leg, .7))
        self.engine.mode = 'basic_takeoff'
        leg.leg_sequence = 4
        self.assertFalse(self.runtime.install_fixed_trajectory(self.manager, leg, .7))

    def test_source_change_and_disabled_or_completed_task_cannot_restore_old_install(self):
        timer, trajectory = self.install()
        command = dict(position=(8., 4., 3.), velocity=(0., 0., 0.), yaw=.4)
        self.runtime.set_reference(command)
        self.assertFalse(self.runtime.install_fixed_trajectory(self.manager, trajectory, .7))
        timer.step()
        self.assertEqual(self.runtime.last_reference_sent['position'], command['position'])
        self.engine.state = 'COMPLETE'
        self.assertFalse(self.runtime.install_fixed_trajectory(self.manager, trajectory, .7))
        self.runtime.disable_full()
        self.engine.state = 'APPROACH'
        self.assertFalse(self.runtime.install_fixed_trajectory(self.manager, trajectory, .7))

    def test_new_task_cannot_accept_previous_engine_or_trajectory(self):
        timer, previous = self.install()
        self.runtime.disable_full()
        self.manager.task_id = 'second-task'
        self.engine.state = 'TAKEOFF'
        self.runtime.enable_full(self.manager)
        replacement = self.engine.fixed_trajectory = AnalyticTrajectory()
        self.engine.state = 'APPROACH'
        self.assertFalse(self.runtime.install_fixed_trajectory(self.manager, previous, .7))
        self.assertTrue(self.runtime.install_fixed_trajectory(self.manager, replacement, .2))
        timer.step()
        self.assertEqual(self.runtime.fixed_progress(self.manager)['task_id'], 'second-task')
        self.manager.task_id = 'late-old-id'
        self.assertIsNone(self.runtime.fixed_progress(self.manager))
        self.assertFalse(self.runtime.install_fixed_trajectory(self.manager, replacement, .2))

    def test_landing_owner_cancels_before_publication_and_never_auto_resumes(self):
        timer, trajectory = self.install()
        timer.step()
        count = len(self.sent)
        self.runtime.packet['owner'] = 'PX4_AUTO_LAND'
        timer.step()
        self.assertIsNone(self.runtime.fixed_trajectory)
        self.assertEqual(len(self.sent), count)
        self.runtime.packet['owner'] = 'PX4_NATIVE'
        self.assertFalse(self.runtime.install_fixed_trajectory(self.manager, trajectory, .7))
        timer.step()
        self.assertEqual(len(self.sent), count)

    def test_manual_takeover_and_gateway_fault_revoke_before_publication(self):
        for reason in ('manual_paused', 'watchdog_fault'):
            with self.subTest(reason=reason):
                # An explicit new task is the existing way to clear a latch.
                self.engine.state = 'TAKEOFF'
                timer, trajectory = self.install()
                timer.step()
                count = len(self.sent)
                self.runtime.packet[reason] = True
                timer.step()
                self.assertIsNone(self.runtime.fixed_trajectory)
                self.assertEqual(len(self.sent), count)
                self.assertEqual(self.engine.state, 'HOLD')
                self.assertFalse(self.manager.active)
                self.assertFalse(self.runtime.install_fixed_trajectory(self.manager, trajectory, .7))
                self.runtime.packet[reason] = False
                self.manager.active = True


if __name__ == '__main__':
    unittest.main()
