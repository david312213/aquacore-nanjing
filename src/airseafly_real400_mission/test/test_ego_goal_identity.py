"""Same-session waypoint replacement with the real manager/native adapter."""
from copy import deepcopy
import unittest
from unittest import mock

import test_native_waypoint_manager as manager_tests
from ros_fixtures import Clock, Stamp, PositionCommand, Vector, EgoGoalCommand, make_ready
from airseafly_real400_mission.ego_identity import goal_key
from airseafly_real400_mission.native_reference import LIO_FRAME


class EgoGoalIdentityTests(unittest.TestCase):
    setUp = manager_tests.WaypointManagerTests.setUp
    setup_manager = manager_tests.WaypointManagerTests.setup_manager
    dispatch_start = manager_tests.WaypointManagerTests.dispatch_start
    activate = manager_tests.WaypointManagerTests.activate
    step = manager_tests.WaypointManagerTests.step
    until = manager_tests.WaypointManagerTests.until

    def navigating(self):
        manager, runtime, h = self.setup_manager()
        self.dispatch_start(manager, runtime, h.engine.config)
        self.activate(manager, runtime)
        self.until(manager, runtime, lambda: manager.engine.state == 'APPROACH')
        e = manager.engine
        e._begin_navigation((4., 3., e.cruise_z), 'target')
        make_ready(manager, 'EGO', 'current')
        e.on_group_ready('EGO')
        manager.tick(None)
        goal = deepcopy(manager.ego_goal)
        self.assertIsNotNone(goal)
        manager.on_goal_received('current', goal)
        runtime.select_ego('current', goal)
        return manager, runtime, h, goal

    def command(self, goal, x, trajectory=1):
        command = PositionCommand()
        command.header.frame_id, command.header.stamp = LIO_FRAME, Stamp(Clock.now)
        command.position = Vector(x, 3., .8)
        command.trajectory_id = trajectory
        return EgoGoalCommand(deepcopy(goal), command)

    def test_new_receipt_cannot_relabel_old_stream_or_complete_new_goal(self):
        m, r, h, first = self.navigating()
        old = self.command(first, 4.)
        m.on_ego_cmd('current', old)
        self.assertEqual(r.ego_command['position'][0], 4.)
        Clock.now += .1
        m.engine._begin_navigation((5., 3., m.engine.cruise_z), 'target')
        m.tick(None)
        second = deepcopy(m.ego_goal)
        self.assertNotEqual(goal_key(first), goal_key(second))
        self.assertIsNone(r.ego_command)
        m.on_goal_received('current', second)
        r.select_ego('current', second)
        m.on_ego_cmd('current', old)
        self.assertIsNone(r.ego_command)
        fresh = self.command(second, 4.2, 2)
        fresh.goal.header.seq = 999  # ROS rewrites this without changing identity.
        m.on_ego_cmd('current', fresh)
        self.assertEqual(r.ego_command['position'][0], 4.2)
        m.on_ego_cmd('current', old)
        with mock.patch.object(m.engine, 'on_navigation_finished', create=True) as finished, \
             mock.patch.object(m.engine, 'on_navigation_failed', create=True) as failed:
            m.on_goal_finished('current', first)
            m.on_goal_failed('current', first)
            finished.assert_not_called()
            failed.assert_not_called()
        self.assertEqual(r.ego_command['goal_key'], goal_key(second))

    def test_all_identity_fields_and_paused_late_samples(self):
        m, r, h, goal = self.navigating()
        for field in ('stamp', 'frame', 'x', 'y', 'z'):
            wrong = deepcopy(goal)
            if field == 'stamp': wrong.header.stamp = Stamp(Clock.now + .01)
            elif field == 'frame': wrong.header.frame_id = 'world'
            else: setattr(wrong.pose.position, field, getattr(wrong.pose.position, field) + .01)
            m.on_ego_cmd('current', self.command(wrong, 99.))
            self.assertIsNone(r.ego_command, field)
        r.manual_takeover = True
        m.engine.hold('manual pause')
        m.on_ego_cmd('current', self.command(goal, 99.))
        m.on_goal_received('current', goal)
        m.on_goal_finished('current', goal)
        self.assertIsNone(r.ego_command)
        self.assertEqual(m.engine.state, 'HOLD')


if __name__ == '__main__':
    unittest.main()
