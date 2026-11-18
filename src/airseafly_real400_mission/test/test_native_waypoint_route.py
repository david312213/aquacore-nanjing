"""Native takeoff/waypoint/hold replays without ROS or hardware actions."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from airseafly_real400_mission.configuration import load_config
from airseafly_real400_mission.native_waypoint_route import NativeTakeoffEngine, WaypointRouteEngine


def configuration(points):
    config = load_config()
    config['flight'].update(controller='px4_native', native_reference_mode='lio_direct',
        native_task_feedback='lio', cruise_height_m=1., native_takeoff_speed_mps=.5,
        native_takeoff_acceleration_mps2=.25)
    config['waypoint_route'] = dict(format='aquacore_waypoints', version=1,
        frame='fast_lio_local_enu', points=deepcopy(points))
    return config


class Replay:
    def __init__(self, points):
        self.engine = WaypointRouteEngine(configuration(points))
        self.ground, self.now = (12., -8., -1.3), 100.
        self.position, self.velocity, self.yaw = self.ground, (0., 0., 0.), .2
        self.effects, self.groups = [], []
        self.engine.start('waypoint_route', self.position, self.yaw, self.now)
        self.engine.native_origin = self.ground
        self.feedback()
        self.engine.begin_native_flight()

    def feedback(self):
        self.engine.on_native_lio_feedback(dict(stamp=self.now,
            position=self.position, velocity=self.velocity))
        self.engine.on_native_feedback(dict(stamp=self.now, pose_stamp=self.now,
            position=tuple(v+40 for v in self.position), velocity=self.velocity,
            yaw=self.yaw, owner='PX4_NATIVE', armed=True, offboard=True,
            mode='OFFBOARD', landed=False))

    def step(self, dt=.1, follow=True, acknowledge=True, arrival_error=(0., 0., 0.)):
        self.now += dt
        e = self.engine
        if follow and e.state == 'TAKEOFF' and e._native_takeoff_motion:
            self.position, self.velocity, _ = e._native_takeoff_motion.sample(self.now)
        elif follow and e.state == 'APPROACH' and e._goal_acked:
            self.position = tuple(v+d for v, d in zip(e.command_position, arrival_error))
            self.velocity = (0., 0., 0.)
        self.feedback()
        if e.group == 'EGO':
            e.on_group_ready('EGO')
        output = e.tick(self.now, self.position, self.yaw, 'PX4_NATIVE')
        self.groups.append(output['desired_group'])
        self.effects.extend(output['effects'])
        if acknowledge:
            for effect in output['effects']:
                if effect['type'] == 'goal':
                    e.on_goal_received(effect['goal_id'])
        return output

    def until(self, predicate, limit=2000, **kwargs):
        for _ in range(limit):
            if predicate(self.engine):
                return
            self.step(**kwargs)
        raise AssertionError((self.engine.state, self.engine.phase, self.engine.snapshot()['waypoint_route']))


class WaypointRouteTests(unittest.TestCase):
    def test_single_point_keeps_exact_endpoint_and_native_stream_after_completion(self):
        h = Replay([[4., 5.]])
        h.until(lambda e: e.done, arrival_error=(.05, -.04, .03))
        e = h.engine
        self.assertNotEqual(e.command_position, h.position)
        self.assertEqual(e.command_position, (4., 5., h.ground[2]+1.))
        for _ in range(10):
            output = h.step(dt=2.)
            self.assertEqual((output['state'], output['phase'], output['desired_group']),
                             ('COMPLETE', 'WAYPOINT_ROUTE_COMPLETE', 'NONE'))
            self.assertEqual(output['control'], 'PX4_NATIVE')
            self.assertEqual(output['reference_source'], 'HOLD')
            self.assertEqual(output['native_command']['position'], e.command_position)
            self.assertEqual(output['native_command']['velocity'], (0., 0., 0.))
        self.assertEqual(sum(v['type'] == 'waypoint_route_complete' for v in h.effects), 1)

    def test_arbitrary_length_route_visits_every_point_once_at_one_height(self):
        for count in (1, 4, 15):
            points = [[index*2., (-1.)**index] for index in range(count)]
            h = Replay(points)
            h.until(lambda e: e.done, arrival_error=(.05, 0., -.04))
            goals = [v for v in h.effects if v['type'] == 'goal']
            self.assertEqual([v['position_xyz'][:2] for v in goals], points)
            self.assertEqual([v['position_xyz'][2] for v in goals], [h.ground[2]+1.]*count)
            self.assertEqual(sum(v['type'] == 'takeoff' for v in h.effects), 1)
            self.assertFalse(any(v['type'] == 'restart_ego' for v in h.effects))
            first_ego = h.groups.index('EGO')
            self.assertEqual(set(h.groups[first_ego:-1]), {'EGO'})
            status = h.engine.snapshot()['waypoint_route']
            self.assertEqual((status['index'], status['completed_points'], status['total_points']),
                             (count-1, count, count))
            self.assertEqual(status['origin_xyz'], list(h.ground))

    def test_takeoff_and_ego_handoffs_do_not_capture_actual_height_error(self):
        h = Replay([[3., 2.], [5., 2.]])
        # Reference completion progresses without requiring actual takeoff height.
        h.until(lambda e: e.state == 'APPROACH', follow=False)
        e = h.engine
        first = e.snapshot()
        self.assertEqual(first['native_command']['position'], (*h.ground[:2], h.ground[2]+1.))
        h.until(lambda e: e.route_index == 1, arrival_error=(0., 0., -.05))
        second = e.snapshot()
        self.assertEqual(second['native_command']['position'][2], h.ground[2]+1.)
        self.assertEqual(e.command_position[2], h.ground[2]+1.)

    def test_route_never_starts_visual_payload_or_landing_states(self):
        h = Replay([[1., 2.], [3., 4.], [1., 2.]])
        h.until(lambda e: e.done)
        self.assertEqual(set(h.groups), {'NONE', 'EGO'})
        forbidden = {'release', 'land', 'native_owner', 'down_takeover'}
        self.assertFalse(forbidden.intersection(v['type'] for v in h.effects))
        self.assertEqual(h.engine.drops_completed, 0)
        self.assertIsNone(h.engine._payload_request)
        self.assertEqual(h.engine.snapshot()['payload_total'], 0)
        self.assertEqual([v['index'] for v in h.effects if v['type'] == 'waypoint_route_point_arrived'],
                         [0, 1, 2])

    def test_hold_prevents_late_ack_or_old_trajectory_from_advancing_route(self):
        h = Replay([[1., 2.], [3., 4.]])
        h.until(lambda e: e.state == 'APPROACH' and e._goal_sent, acknowledge=False)
        e = h.engine
        old_goal = e._goal
        h.position = (1., 2., h.ground[2]+1.)
        h.feedback()
        e.hold('operator pause / manual takeover')
        held = e.snapshot()['native_command']['position']
        e.on_goal_received(old_goal)
        self.assertFalse(e.on_navigation_finished(old_goal))
        for _ in range(10):
            output = h.step(dt=60.)
            self.assertEqual(output['state'], 'HOLD')
            self.assertEqual(output['native_command']['position'], held)
        self.assertEqual(e.route_index, 0)
        self.assertEqual(e.route_completed_points, 0)
        self.assertFalse(any(v['type'] == 'waypoint_route_point_arrived' for v in h.effects))

    def test_old_goal_ack_cannot_advance_the_next_point(self):
        h = Replay([[1., 2.], [3., 4.]])
        h.until(lambda e: e.state == 'APPROACH' and e._goal_acked)
        old_goal = h.engine._goal
        h.until(lambda e: e.route_index == 1)
        h.step(acknowledge=False, follow=False)
        h.engine.on_goal_received(old_goal)
        self.assertFalse(h.engine._goal_acked)
        h.step(dt=400., acknowledge=False, follow=False)
        self.assertEqual(h.engine.route_index, 1)
        self.assertEqual(h.engine.state, 'APPROACH')
        self.assertFalse(h.engine.final_route_deadline_enabled)

    def test_restart_is_a_new_takeoff_and_resets_route_progress(self):
        h = Replay([[1., 2.]])
        h.until(lambda e: e.done)
        e = h.engine
        e.start('waypoint_route', (5., 7., 2.), .5, h.now+1.)
        self.assertEqual(e.state, 'TAKEOFF')
        self.assertIsNone(e.route_index)
        self.assertEqual(e.route_completed_points, 0)
        self.assertFalse(e.done)
        self.assertEqual(e.origin_xyz, (5., 7., 2.))
        self.assertEqual(e.command_position, (5., 7., 3.))

    def test_empty_or_wrong_mode_rejected_before_any_takeoff_effect(self):
        with self.assertRaises(ValueError):
            WaypointRouteEngine(configuration([]))
        e = WaypointRouteEngine(configuration([[1., 2.]]))
        with self.assertRaises(ValueError):
            e.start('full', (0., 0., 0.), 0., 100.)
        self.assertEqual(e.state, 'IDLE')
        self.assertEqual(e._effects, [])

    def test_legacy_controller_configuration_cannot_enter_native_recorded_point_mode(self):
        config = configuration([[1., 2.]])
        config['flight'].update(controller='px4ctrl', native_reference_mode='estimate_compensated')
        # This pairing is valid for the legacy stack, but these dedicated basic
        # entries must not send recorded planning-frame XY through that stack.
        for engine_type in (NativeTakeoffEngine, WaypointRouteEngine):
            with self.subTest(engine=engine_type.__name__):
                with self.assertRaisesRegex(ValueError, 'flight.controller=px4_native'):
                    engine_type(config)


if __name__ == '__main__':
    unittest.main()
