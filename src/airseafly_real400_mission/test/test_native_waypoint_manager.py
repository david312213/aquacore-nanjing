"""Waypoint manager/runtime integration with inert ROS and process stubs only."""
from copy import deepcopy
import json
import os
import sys
from types import SimpleNamespace as NS
import unittest
from unittest import mock

from ros_fixtures import (STUBS, ROS, Bool, Clock, PositionCommand, Stamp, Vector,
                          bound_command, make_manager, make_ready)
from ros_fixtures import refresh_inputs, runtime_for
from test_native_waypoint_route import Replay, configuration
from airseafly_real400_mission.native_reference import LIO_FRAME
from airseafly_real400_mission.native_waypoint_route import WaypointRouteEngine


class WaypointManagerTests(unittest.TestCase):
    def setUp(self):
        patch = mock.patch.dict(sys.modules, STUBS)
        patch.start()
        self.addCleanup(patch.stop)
        clock = mock.patch('airseafly_real400_mission.native_runtime.time.monotonic', lambda: Clock.now)
        clock.start()
        self.addCleanup(clock.stop)
        ROS.ServiceProxy.reset_mock()

    def setup_manager(self, points=None):
        replay = Replay(points or [[1., 2.], [4., 3.]])
        manager = make_manager()
        manager.pose, manager.yaw, manager.odom_stamp = list(replay.ground), replay.yaw, replay.now
        manager.raw_pose, manager.raw_yaw, manager.raw_stamp = [20., -4., 8.], replay.yaw, replay.now
        manager.planning_pose, manager.planning_stamp = list(replay.ground), replay.now
        manager.raw_to_local = object()
        manager.ensure_alignment_trace = mock.Mock()
        Clock.now = replay.now
        runtime = runtime_for(manager, NS(engine=replay.engine, now=replay.now,
                                          p=replay.position, velocity=replay.velocity))
        runtime.packet.update(landed=True, armed=False, offboard=False)
        runtime.start_takeoff = mock.Mock()
        return manager, runtime, replay

    def dispatch_start(self, manager, runtime, config, mode='waypoint_route'):
        config = deepcopy(config)
        config['_flight_configuration'] = {'task': 'takeoff' if mode == 'basic_takeoff' else 'waypoint-route'}
        config.setdefault('recording', {})['enabled'] = False
        with mock.patch.dict(os.environ, ROS_MASTER_URI=config['runtime']['ros_master_uri']), \
             mock.patch.object(runtime, 'acquire_full'), \
             mock.patch.dict(sys.modules, {'airseafly_real400_mission.recording_client': None}):
            result = manager.dispatch(dict(command='start', mode=mode, resolved_config=config))
        self.assertTrue(result['ok'], result)
        return result

    def activate(self, manager, runtime):
        runtime.full_owner_since = Clock.now
        runtime.packet.update(owner='PX4_NATIVE', landed=False, armed=True,
                              offboard=True, mode='OFFBOARD')
        manager.control_state = 'PX4_NATIVE'

    def step(self, manager, runtime, follow=True, dt=.1, acknowledge=True):
        Clock.now += dt
        engine = manager.engine
        position = tuple(manager.pose)
        velocity = (0., 0., 0.)
        if follow and engine.state == 'TAKEOFF' and engine._native_takeoff_motion:
            position, velocity, _ = engine._native_takeoff_motion.sample(Clock.now)
        elif follow and engine.state == 'APPROACH' and engine._goal_acked:
            # Arrival tolerance must not become the next leg's height baseline.
            position = (engine.command_position[0]+.02, engine.command_position[1],
                        engine.command_position[2]-.03)
        manager.pose = manager.planning_pose = position
        manager.odom_stamp = manager.planning_stamp = Clock.now
        runtime.reference.feed(Clock.now, position, velocity)
        refresh_inputs(runtime)
        if manager.requested_group == 'EGO' and not manager.worker_sessions.get('EGO'):
            make_ready(manager, 'EGO', 'waypoint-%d' % engine._navigation_sequence)
        manager.tick(None)
        if acknowledge and manager.ego_goal is not None and not manager.goal_received:
            manager.on_goal_received(manager.worker_sessions['EGO'], manager.ego_goal)
        runtime._full_tick(None)
        errors = [values for kind, values in manager.events_seen if kind == 'manager_error']
        self.assertEqual(errors, [])

    def until(self, manager, runtime, predicate, **kwargs):
        for _ in range(500):
            if predicate():
                return
            self.step(manager, runtime, **kwargs)
        self.fail((manager.engine.state, manager.engine.phase, manager.last_output))

    def test_rpc_registers_first_ground_and_uses_only_native_takeoff(self):
        manager, runtime, replay = self.setup_manager()
        transform = manager.raw_to_local
        self.dispatch_start(manager, runtime, replay.engine.config)
        self.assertIsInstance(manager.engine, WaypointRouteEngine)
        self.assertIs(manager.raw_to_local, transform)
        self.assertEqual(manager.engine.route_points, [(1., 2.), (4., 3.)])
        self.assertEqual(manager.engine.origin_xyz, replay.ground)
        self.assertEqual(manager.engine.native_origin, replay.ground)
        self.assertEqual(runtime.reference_mode, 'lio_direct')
        origin = runtime.parameters[runtime.ORIGIN_PARAM]
        self.assertEqual(origin['radar_origin_xyz'], list(replay.ground))
        self.assertEqual(origin['lio_origin_xyz'], manager.raw_pose)
        manager.tick(None)
        runtime.start_takeoff.assert_called_once()
        ROS.ServiceProxy.assert_not_called()
        runtime.rospy.ServiceProxy.assert_not_called()
        self.assertEqual(manager.position_pub.messages, [])
        self.assertEqual(manager.land_pub.messages, [])
        self.assertEqual(runtime.reference_source, 'TAKEOFF')

    def test_new_ground_start_replaces_previous_origin_without_restarting_localization(self):
        manager, runtime, replay = self.setup_manager()
        self.dispatch_start(manager, runtime, replay.engine.config)
        manager.tick(None)
        self.assertTrue(manager.dispatch(dict(command='hold'))['ok'])
        previous_engine = manager.engine
        transform = manager.raw_to_local
        manager.flight_origin = replay.ground
        ground = (23., 7., 4.6)
        Clock.now += 1.
        manager.pose = manager.planning_pose = ground
        manager.raw_pose = [21., 17., 11.]
        manager.odom_stamp = manager.planning_stamp = manager.raw_stamp = Clock.now
        runtime.reference.feed(Clock.now, ground, (0., 0., 0.))
        refresh_inputs(runtime)
        runtime.packet.update(landed=True, armed=False, offboard=False)
        self.dispatch_start(manager, runtime, replay.engine.config)
        self.assertIsNot(manager.engine, previous_engine)
        self.assertIs(manager.raw_to_local, transform)
        self.assertEqual(manager.engine.origin_xyz, ground)
        self.assertEqual(manager.engine.native_origin, ground)
        self.assertEqual(manager.engine.command_position, (*ground[:2], ground[2]+1.))
        self.assertEqual(runtime.parameters[runtime.ORIGIN_PARAM]['radar_origin_xyz'], list(ground))
        self.assertEqual(runtime.parameters[runtime.ORIGIN_PARAM]['lio_origin_xyz'], manager.raw_pose)
        self.assertIsNone(manager.flight_origin)

    def test_basic_takeoff_hold_and_next_waypoint_flight_share_one_manager(self):
        manager, runtime, replay = self.setup_manager()
        basic_config = deepcopy(replay.engine.config)
        basic_config.pop('waypoint_route')
        takeoff = self.dispatch_start(manager, runtime, basic_config, mode='basic_takeoff')
        self.assertEqual(manager.engine.mode, 'takeoff')
        self.assertTrue(manager.engine.full_native)
        self.assertEqual(manager.engine.state, 'TAKEOFF')
        self.activate(manager, runtime)
        self.until(manager, runtime, lambda: manager.engine.done)
        self.assertEqual(manager.engine.phase, 'TAKEOFF_COMPLETE')
        self.assertFalse(manager.active)
        self.assertTrue(runtime.full_enabled)
        self.assertIs(runtime.manager, manager)
        self.assertEqual(runtime.full_task_id, takeoff['task_id'])
        self.assertEqual(runtime.reference_command['position'], (*replay.ground[:2], replay.ground[2]+1.))
        self.assertEqual(set(manager.runner.requests), {'NONE'})
        self.assertTrue(manager.dispatch(dict(command='hold'))['ok'])
        self.assertEqual(manager.engine.state, 'HOLD')
        self.assertFalse(manager.active)
        self.assertTrue(runtime.full_enabled)

        # Start the ordinary route after landing, using the same persistent
        # manager/runtime but a newly measured launch position.
        ground = (10., 11., 2.)
        Clock.now += 1.
        manager.pose = manager.planning_pose = ground
        manager.odom_stamp = manager.planning_stamp = Clock.now
        runtime.reference.feed(Clock.now, ground, (0., 0., 0.))
        refresh_inputs(runtime)
        runtime.packet.update(landed=True, armed=False, offboard=False)
        route = self.dispatch_start(manager, runtime, replay.engine.config)
        self.assertIsInstance(manager.engine, WaypointRouteEngine)
        self.assertIs(runtime.manager, manager)
        self.assertIs(manager.native_runtime, runtime)
        self.assertEqual(runtime.full_task_id, route['task_id'])
        self.assertNotEqual(route['task_id'], takeoff['task_id'])
        self.assertEqual(manager.engine.origin_xyz, ground)
        self.assertEqual(manager.engine.command_position, (*ground[:2], ground[2]+1.))
        self.activate(manager, runtime)
        self.until(manager, runtime, lambda: manager.engine.done)
        self.assertEqual(manager.engine.phase, 'WAYPOINT_ROUTE_COMPLETE')
        self.assertEqual(manager.engine.route_completed_points, 2)
        self.assertEqual(runtime.reference_command['position'], (4., 3., ground[2]+1.))
        self.assertEqual(manager.land_pub.messages, [])
        self.assertEqual(manager.payload_records, [])
        ROS.ServiceProxy.assert_not_called()

    def test_lio_direct_forwards_recorded_coordinates_without_px4_origin_compensation(self):
        manager, runtime, replay = self.setup_manager()
        self.dispatch_start(manager, runtime, replay.engine.config)
        command = dict(frame=LIO_FRAME, position=(3., 2., replay.ground[2]+1.),
                       velocity=(.2, .1, 0.), yaw=replay.yaw)
        runtime.publish(command, full=True)
        emitted = runtime.publisher.messages[-1]
        self.assertEqual((emitted.position.x, emitted.position.y, emitted.position.z), command['position'])
        self.assertEqual((emitted.velocity.x, emitted.velocity.y, emitted.velocity.z), command['velocity'])
        self.assertNotEqual(runtime.packet['position'], runtime.reference.latest['position'])

    def test_ego_request_carries_completed_takeoff_reference_despite_lio_climb(self):
        manager, runtime, replay = self.setup_manager()
        self.dispatch_start(manager, runtime, replay.engine.config)
        self.activate(manager, runtime)
        self.step(manager, runtime)
        motion = manager.engine._native_takeoff_motion
        Clock.now = motion.started + motion.duration + .01
        cruise = manager.engine.cruise_z
        measured = (replay.ground[0]+.08, replay.ground[1]-.1, cruise-.204)
        manager.pose = manager.planning_pose = measured
        manager.odom_stamp = manager.planning_stamp = Clock.now
        runtime.reference.feed(Clock.now, measured, (.1329, -.0419, .40724))
        refresh_inputs(runtime)
        manager.tick(None)
        make_ready(manager, 'EGO', 'handoff-seed')
        Clock.now += .01
        manager.odom_stamp = manager.planning_stamp = Clock.now
        runtime.reference.feed(Clock.now, measured, (.1329, -.0419, .40724))
        refresh_inputs(runtime)
        manager.tick(None)
        request = manager.ego_goal_start_pub.messages[-1]
        self.assertEqual(request.goal.header.stamp, manager.ego_goal.header.stamp)
        self.assertEqual(request.goal.pose.position.__dict__, manager.ego_goal.pose.position.__dict__)
        start = request.command
        self.assertEqual((start.position.x, start.position.y, start.position.z), (*replay.ground[:2], cruise))
        self.assertEqual((start.velocity.x, start.velocity.y, start.velocity.z), (0., 0., 0.))
        self.assertEqual((start.acceleration.x, start.acceleration.y, start.acceleration.z), (0., 0., 0.))
        self.assertEqual(manager.ego_goal_pub.messages, [])
        frozen = deepcopy((start.position.__dict__, start.velocity.__dict__))
        self.step(manager, runtime, follow=False, acknowledge=False, dt=.3)
        resent = manager.ego_goal_start_pub.messages[-1]
        self.assertEqual((resent.command.position.__dict__, resent.command.velocity.__dict__), frozen)

    def test_whole_route_finishes_with_timer_streaming_last_point_and_no_payload_or_vision(self):
        manager, runtime, replay = self.setup_manager([[1., 2.], [4., 3.], [-2., 6.]])
        self.dispatch_start(manager, runtime, replay.engine.config)
        self.activate(manager, runtime)
        self.until(manager, runtime, lambda: manager.engine.done)
        self.assertFalse(manager.active)
        self.assertTrue(runtime.full_enabled)
        self.assertEqual(runtime.full_timer.duration, .02)
        self.assertEqual(manager.engine.phase, 'WAYPOINT_ROUTE_COMPLETE')
        self.assertEqual(manager.engine.route_completed_points, 3)
        self.assertEqual(manager.requested_group, 'NONE')
        self.assertTrue(set(manager.runner.requests) <= {'NONE', 'EGO'})
        target = (-2., 6., replay.ground[2]+1.)
        self.assertEqual(runtime.reference_command['position'], target)
        sent = len(runtime.publisher.messages)
        for _ in range(3):
            self.step(manager, runtime, follow=False)
            message = runtime.publisher.messages[-1]
            self.assertEqual((message.position.x, message.position.y, message.position.z), target)
            self.assertEqual((message.velocity.x, message.velocity.y, message.velocity.z), (0., 0., 0.))
        self.assertEqual(len(runtime.publisher.messages), sent+3)
        effects = [value['effect'] for kind, value in manager.events_seen if kind == 'effect']
        self.assertEqual([item['position_xyz'] for item in effects if item['type'] == 'goal'],
                         [[1., 2., target[2]], [4., 3., target[2]], list(target)])
        self.assertFalse({'release', 'land', 'native_owner'}.intersection(item['type'] for item in effects))
        self.assertEqual(manager.payload_records, [])
        self.assertEqual(manager.position_pub.messages, [])
        self.assertEqual(manager.land_pub.messages, [])
        ROS.ServiceProxy.assert_not_called()

    def test_hold_cancels_route_but_keeps_native_hold_and_rejects_late_ego_input(self):
        manager, runtime, replay = self.setup_manager()
        self.dispatch_start(manager, runtime, replay.engine.config)
        self.activate(manager, runtime)
        self.until(manager, runtime, lambda: manager.engine.state == 'APPROACH' and manager.goal_received)
        session, goal = manager.worker_sessions['EGO'], deepcopy(manager.ego_goal)
        command = PositionCommand()
        command.header.frame_id, command.header.stamp = LIO_FRAME, Stamp(Clock.now)
        command.position = Vector(1., 2., replay.ground[2]+1.)
        packet = bound_command(manager, command)
        self.assertTrue(manager.dispatch(dict(command='hold'))['ok'])
        held = runtime.reference_command['position']
        manager.on_goal_received(session, goal)
        manager.on_ego_cmd(session, packet)
        for _ in range(5):
            self.step(manager, runtime, follow=False)
        self.assertFalse(manager.active)
        self.assertEqual(manager.engine.state, 'HOLD')
        self.assertEqual(manager.engine.route_index, 0)
        self.assertEqual(manager.engine.route_completed_points, 0)
        self.assertEqual(runtime.reference_command['position'], held)
        self.assertIsNone(runtime.ego_command)
        self.assertTrue(runtime.full_enabled)

    def test_same_worker_handles_closed_route_and_rejects_previous_goal_input(self):
        manager, runtime, replay = self.setup_manager([[1., 2.], [4., 3.], [1., 2.]])
        self.dispatch_start(manager, runtime, replay.engine.config)
        self.activate(manager, runtime)
        self.until(manager, runtime, lambda: manager.engine.state == 'APPROACH' and manager.goal_received)
        self.step(manager, runtime, follow=False)
        session = manager.worker_sessions['EGO']
        publisher, subscribers = manager.ego_goal_pub, list(manager.dynamic_subscribers)
        first_goal = deepcopy(manager.ego_goal)
        command = PositionCommand()
        command.header.frame_id, command.header.stamp = LIO_FRAME, Stamp(Clock.now)
        command.position = Vector(1., 2., replay.ground[2]+1.)
        old_packet = bound_command(manager, command)
        manager.on_ego_cmd(session, old_packet)
        self.assertIsNotNone(runtime.ego_command)

        for index in (1, 2):
            self.until(manager, runtime,
                lambda: manager.engine.route_index == index and manager.engine._goal_sent,
                acknowledge=False)
            self.assertEqual(manager.worker_sessions['EGO'], session)
            self.assertIs(manager.ego_goal_pub, publisher)
            self.assertEqual(manager.dynamic_subscribers, subscribers)
            self.assertTrue(all(not sub.unregistered for sub in subscribers))
            self.assertIsNone(runtime.ego_command)
            self.assertFalse(manager.goal_received)
            # Closing the route repeats the first XYZ, but its goal identity
            # must still reject that point's original ACK and spline.
            self.assertNotEqual(manager.ego_goal.header.stamp, first_goal.header.stamp)
            manager.on_goal_received(session, first_goal)
            self.assertFalse(manager.goal_received)
            self.assertFalse(manager.engine._goal_acked)
            manager.on_goal_received(session, manager.ego_goal)
            self.step(manager, runtime, follow=False)
            old_packet.command.header.stamp = Stamp(Clock.now)
            manager.on_ego_cmd(session, old_packet)
            self.assertIsNone(runtime.ego_command)
            current = deepcopy(old_packet.command)
            current.header.stamp = Stamp(Clock.now)
            manager.on_ego_cmd(session, bound_command(manager, current))
            self.assertIsNotNone(runtime.ego_command)

        self.until(manager, runtime, lambda: manager.engine.done)
        self.assertEqual(manager.runner.requests.count('EGO'), 1)
        self.assertEqual(manager.runner.requests[-1], 'NONE')
        self.assertEqual(manager.engine.route_completed_points, 3)

    def test_safe_land_priority_and_auto_land_owner_stop_route_and_setpoints(self):
        manager, runtime, replay = self.setup_manager()
        self.dispatch_start(manager, runtime, replay.engine.config)
        self.activate(manager, runtime)
        self.until(manager, runtime, lambda: manager.engine.state == 'APPROACH' and manager.goal_received)
        manager.on_priority_landing(Bool(True))
        runtime.packet.update(owner='PX4_AUTO_LAND', mode='AUTO.LAND', offboard=False)
        sent = len(runtime.publisher.messages)
        for _ in range(5):
            self.step(manager, runtime, follow=False)
        self.assertTrue(manager.priority_landing)
        self.assertFalse(manager.active)
        self.assertEqual(manager.engine.state, 'HOLD')
        self.assertEqual(manager.engine.route_completed_points, 0)
        self.assertEqual(manager.requested_group, 'NONE')
        self.assertEqual(len(runtime.publisher.messages), sent)
        self.assertEqual(manager.last_output['control'], 'NONE')
        manager.on_priority_landing(Bool(False))
        manager.tick(None)
        self.assertFalse(manager.active)
        self.assertEqual(manager.engine.route_completed_points, 0)
        runtime.rospy.ServiceProxy.assert_not_called()
        ROS.ServiceProxy.assert_not_called()

    def test_manual_takeover_never_restarts_route_or_reacquires_after_offboard_returns(self):
        for latched in (False, True):
            with self.subTest(latched=latched):
                manager, runtime, replay = self.setup_manager()
                self.dispatch_start(manager, runtime, replay.engine.config)
                self.activate(manager, runtime)
                self.until(manager, runtime, lambda: manager.engine.state == 'APPROACH' and manager.goal_received)
                runtime._full_tick(None)
                runtime.packet.update(manual_paused=latched, mode='OFFBOARD' if latched else 'POSCTL',
                                      offboard=latched)
                sent = len(runtime.publisher.messages)
                runtime._full_tick(None)
                self.assertTrue(runtime.manual_takeover)
                self.assertFalse(manager.active)
                self.assertEqual(manager.engine.state, 'HOLD')
                runtime.packet.update(manual_paused=False, mode='OFFBOARD', offboard=True)
                for _ in range(5):
                    self.step(manager, runtime, follow=False)
                self.assertFalse(manager.active)
                self.assertEqual(manager.engine.route_completed_points, 0)
                self.assertEqual(len(runtime.publisher.messages), sent)
                runtime.rospy.ServiceProxy.assert_not_called()
                ROS.ServiceProxy.assert_not_called()

    def test_trace_records_task_identity_fixed_height_and_active_waypoint(self):
        manager, runtime, replay = self.setup_manager()
        started = self.dispatch_start(manager, runtime, replay.engine.config)
        self.activate(manager, runtime)
        self.until(manager, runtime, lambda: manager.engine.done)
        rows = [json.loads(line) for line in runtime.trace.getvalue().splitlines()]
        self.assertTrue(rows)
        self.assertEqual({row['task_id'] for row in rows}, {started['task_id']})
        self.assertEqual({row['mode'] for row in rows}, {'waypoint_route'})
        self.assertEqual({tuple(row['ground_origin']) for row in rows}, {replay.ground})
        self.assertEqual({tuple(row['takeoff_target_xyz']) for row in rows},
                         {(*replay.ground[:2], replay.ground[2]+1.)})
        navigations = [row for row in rows if row['state'] == 'APPROACH']
        self.assertEqual({row['waypoint_index'] for row in navigations}, {0, 1})
        self.assertEqual({row['waypoint_count'] for row in navigations}, {2})
        self.assertEqual({row['target_xyz'][2] for row in navigations}, {replay.ground[2]+1.})
        self.assertTrue(all(row['control_status']['owner'] == 'PX4_NATIVE' for row in rows))
        self.assertEqual(rows[-1]['phase'], 'WAYPOINT_ROUTE_COMPLETE')
        self.assertEqual(rows[-1]['target_xyz'], [4., 3., replay.ground[2]+1.])

    def test_wait_for_first_ego_reference_keeps_takeoff_target_height_after_goal_ack(self):
        manager, runtime, replay = self.setup_manager()
        self.dispatch_start(manager, runtime, replay.engine.config)
        self.activate(manager, runtime)
        for _ in range(100):
            self.step(manager, runtime, follow=False)
            if manager.engine.state == 'APPROACH' and manager.goal_received:
                break
        self.assertEqual(manager.engine.state, 'APPROACH')
        self.assertTrue(manager.goal_received)
        self.assertEqual(manager.pose, replay.ground)
        target = (*replay.ground[:2], replay.ground[2]+1.)
        self.assertEqual(runtime.reference_command['position'], target)
        # The next task tick selects EGO, whose first spline has not arrived.
        self.step(manager, runtime, follow=False)
        self.assertEqual(runtime.reference_source, 'EGO')
        self.assertIsNone(runtime.ego_command)
        self.assertEqual(runtime.reference_command['position'], target)
        emitted = runtime.publisher.messages[-1]
        self.assertEqual((emitted.position.x, emitted.position.y, emitted.position.z), target)

    def test_trace_keeps_old_task_id_until_runtime_atomically_switches_engine(self):
        manager, runtime, replay = self.setup_manager()
        first = self.dispatch_start(manager, runtime, replay.engine.config)
        self.activate(manager, runtime)
        manager.tick(None)
        self.assertTrue(manager.dispatch(dict(command='hold'))['ok'])
        old_engine = runtime.full_engine
        runtime.packet.update(landed=True, armed=False, offboard=True)
        original_enable = runtime.enable_full
        previous_rows = len(runtime.trace.getvalue().splitlines())
        interleaved = []

        def interleave_before_enable(current_manager):
            # dispatch has installed the new engine/task ID under manager.lock,
            # but its 50 Hz publisher still owns the preceding runtime stream.
            self.assertIs(runtime.full_engine, old_engine)
            self.assertNotEqual(current_manager.task_id, first['task_id'])
            runtime._full_tick(None)
            interleaved.append(json.loads(runtime.trace.getvalue().splitlines()[-1]))
            original_enable(current_manager)

        with mock.patch.object(runtime, 'enable_full', side_effect=interleave_before_enable):
            second = self.dispatch_start(manager, runtime, replay.engine.config)
        self.assertEqual(len(runtime.trace.getvalue().splitlines()), previous_rows+1)
        self.assertEqual(interleaved[0]['task_id'], first['task_id'])
        self.assertEqual(interleaved[0]['state'], 'HOLD')
        self.activate(manager, runtime)
        manager.tick(None)
        runtime._full_tick(None)
        current = json.loads(runtime.trace.getvalue().splitlines()[-1])
        self.assertEqual(current['task_id'], second['task_id'])
        self.assertNotEqual(current['task_id'], first['task_id'])
        self.assertEqual(current['state'], 'TAKEOFF')


if __name__ == '__main__':
    unittest.main()
