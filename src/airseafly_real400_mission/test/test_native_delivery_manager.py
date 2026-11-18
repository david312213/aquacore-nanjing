"""Delivery manager/native-runtime wiring with inert ROS endpoints only."""
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest import mock

from ros_fixtures import (MANAGER, STUBS, ROS, Bool, Clock, PositionCommand, Stamp, String, Vector,
                          bound_command, make_manager, make_ready, refresh_inputs, runtime_for)
from test_native_delivery_route import Replay, configuration
from airseafly_real400_mission.native_delivery_route import DeliveryRouteEngine
from airseafly_real400_mission.native_reference import LIO_FRAME


class RecordingObserver:
    """Inert red-point runner: records starts, never spawns a process."""
    created = []

    def __init__(self, command, directory, on_start, on_error, event, env=None):
        self.session = 'observer-%d' % (len(RecordingObserver.created)+1)
        self.command, self.on_start, self.on_error = command, on_start, on_error
        self.closed = False
        RecordingObserver.created.append(self)

    def request(self, group):
        if group == 'DOWN':
            self.on_start(group, self.session)

    def close(self):
        self.closed = True


class DeliveryManagerTests(unittest.TestCase):
    def setUp(self):
        self.modules = mock.patch.dict(sys.modules, STUBS)
        self.modules.start()
        self.addCleanup(self.modules.stop)
        self.clock = mock.patch('airseafly_real400_mission.native_runtime.time.monotonic', lambda: Clock.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        RecordingObserver.created = []
        self.observer_factory = mock.patch.object(MANAGER, 'ComputeRunner', RecordingObserver)
        self.observer_factory.start()
        self.addCleanup(self.observer_factory.stop)

    def setup_manager(self):
        replay = Replay()
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
        return manager, runtime, replay

    def start_route(self):
        manager, runtime, replay = self.setup_manager()
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        manager.log_dir = Path(folder.name)
        def close_observers():
            manager.stop_red_point_worker()
            for job in getattr(manager, 'red_point_jobs', []):
                self.assertTrue(job['finished'].wait(2))
        self.addCleanup(close_observers)
        config = configuration()
        config['_flight_configuration'] = {'task': 'delivery-route'}
        with mock.patch.dict(os.environ, ROS_MASTER_URI=config['runtime']['ros_master_uri']), \
             mock.patch.object(runtime, 'acquire_full'), \
             mock.patch.dict(sys.modules, {'airseafly_real400_mission.recording_client': None}):
            result = manager.dispatch(dict(command='start', mode='delivery_route', resolved_config=config))
        self.assertTrue(result['ok'], result)
        runtime.start_takeoff = mock.Mock()
        runtime.full_owner_since = Clock.now
        runtime.packet.update(owner='PX4_NATIVE', landed=False, armed=True,
                              offboard=True, mode='OFFBOARD')
        manager.control_state = 'PX4_NATIVE'
        manager.send_payload = mock.Mock(side_effect=lambda slot, action, request_id, **kw:
            manager.engine.on_payload_result(True, request_id))
        return manager, runtime, replay

    def step(self, manager, runtime, follow=True, dt=.1):
        Clock.now += dt
        engine = manager.engine
        position, velocity = tuple(manager.pose), (0., 0., 0.)
        motion = engine._native_takeoff_motion if engine.state == 'TAKEOFF' else engine.native_motion
        if follow and motion:
            position, velocity, _ = motion.sample(Clock.now)
        elif follow and engine.state == 'APPROACH' and engine._goal_acked:
            position = tuple(engine.command_position)
        manager.pose = manager.planning_pose = position
        manager.odom_stamp = manager.planning_stamp = Clock.now
        runtime.reference.feed(Clock.now, position, velocity)
        refresh_inputs(runtime)
        if manager.requested_group == 'EGO' and not manager.worker_sessions.get('EGO'):
            make_ready(manager, 'EGO', 'delivery-%d' % engine._navigation_sequence)
        manager.tick(None)
        if manager.ego_goal is not None and not manager.goal_received:
            manager.on_goal_received(manager.worker_sessions['EGO'], manager.ego_goal)
        runtime._full_tick(None)
        self.assertEqual([v for kind, v in manager.events_seen if kind == 'manager_error'], [])

    def until(self, manager, runtime, predicate):
        for _ in range(5000):
            if predicate():
                return
            self.step(manager, runtime)
        self.fail((manager.engine.state, manager.engine.phase))

    def test_rpc_uses_recorded_frame_and_native_takeoff_without_recorder_or_legacy_publishers(self):
        manager, runtime, replay = self.setup_manager()
        transform = manager.raw_to_local
        config = configuration()
        config['_flight_configuration'] = {'task': 'delivery-route'}
        with mock.patch.dict(os.environ, ROS_MASTER_URI=config['runtime']['ros_master_uri']), \
             mock.patch.object(runtime, 'acquire_full'), \
             mock.patch.dict(sys.modules, {'airseafly_real400_mission.recording_client': None}):
            result = manager.dispatch(dict(command='start', mode='delivery_route', resolved_config=config))
        self.assertTrue(result['ok'], result)
        self.assertIsInstance(manager.engine, DeliveryRouteEngine)
        self.assertIs(manager.raw_to_local, transform)
        self.assertEqual(manager.planning_pose, list(replay.ground))
        self.assertEqual(manager.engine.route_home, tuple(config['delivery_route']['home']))
        self.assertEqual(manager.engine.route_points, config['delivery_route']['route'])
        self.assertEqual(manager.engine.native_origin, replay.ground)
        self.assertEqual(manager.engine.origin_xyz, replay.ground)
        self.assertEqual(runtime.reference_mode, 'lio_direct')
        self.assertEqual(runtime.parameters[runtime.ORIGIN_PARAM]['radar_origin_xyz'], list(replay.ground))
        self.assertEqual(runtime.parameters[runtime.ORIGIN_PARAM]['lio_origin_xyz'], manager.raw_pose)
        runtime.start_takeoff = mock.Mock()
        ROS.ServiceProxy.reset_mock()
        manager.tick(None)
        runtime.start_takeoff.assert_called_once()
        ROS.ServiceProxy.assert_not_called()
        self.assertEqual(manager.position_pub.messages, [])
        self.assertEqual(manager.land_pub.messages, [])
        self.assertEqual(runtime.reference_source, 'TAKEOFF')

    def test_lio_direct_publish_preserves_recorded_xy_despite_px4_origin_difference(self):
        manager, runtime, replay = self.setup_manager()
        manager.engine, manager.config = replay.engine, replay.engine.config
        runtime.raw_origin = manager.raw_pose
        runtime.prepare(manager.engine, 'delivery_route')
        with mock.patch.object(runtime, 'acquire_full'):
            runtime.enable_full(manager)
        command = dict(frame=LIO_FRAME, position=(3., 2., replay.ground[2]+1.4),
                       velocity=(0., 0., 0.), yaw=replay.yaw)
        runtime.publish(command, full=True)
        emitted = runtime.publisher.messages[-1]
        self.assertEqual((emitted.position.x, emitted.position.y, emitted.position.z), command['position'])
        self.assertEqual(runtime.reference.latest['position'], replay.ground)

    def test_one_worker_spans_both_laps_and_native_drop_never_publishes_old_ego_commands(self):
        manager, runtime, replay = self.start_route()
        self.until(manager, runtime, lambda: manager.engine.state == 'APPROACH' and manager.goal_received)
        session = manager.worker_sessions['EGO']
        publisher, subscribers = manager.ego_goal_pub, list(manager.dynamic_subscribers)
        phases, laps, last_packet = set(), set(), None
        for _ in range(5000):
            self.step(manager, runtime)
            engine = manager.engine
            if engine.state == 'LAND':
                break
            self.assertEqual(manager.requested_group, 'EGO', (engine.state, engine.phase))
            self.assertEqual(manager.worker_sessions['EGO'], session)
            self.assertIs(manager.ego_goal_pub, publisher)
            self.assertEqual(manager.dynamic_subscribers, subscribers)
            self.assertTrue(all(not sub.unregistered for sub in subscribers))
            laps.add(engine.route_lap)
            if runtime.reference_source == 'EGO':
                command = PositionCommand()
                command.header.frame_id, command.header.stamp = LIO_FRAME, Stamp(Clock.now)
                command.position = Vector(*engine.command_position)
                command.trajectory_id = engine._navigation_sequence
                last_packet = bound_command(manager, command)
                manager.on_ego_cmd(session, last_packet)
                self.assertIsNotNone(runtime.ego_command)
            if engine.state == 'DROP':
                phases.add(engine.phase)
                self.assertIsNotNone(last_packet)
                self.assertEqual(engine.route_lap, 2)
                self.assertEqual(manager.last_output['control'], 'PX4_NATIVE')
                before = deepcopy(runtime.ego_command)
                delayed = deepcopy(last_packet)
                delayed.command.header.stamp = Stamp(Clock.now)
                delayed.command.position = Vector(999., 999., 999.)
                manager.on_ego_cmd(session, delayed)
                self.assertEqual(runtime.ego_command, before)
                runtime._full_tick(None)
                emitted = runtime.publisher.messages[-1]
                self.assertEqual((emitted.position.x, emitted.position.y, emitted.position.z),
                                 runtime.reference_command['position'])
                self.assertNotEqual((emitted.position.x, emitted.position.y, emitted.position.z),
                                    (999., 999., 999.))
        else:
            self.fail((manager.engine.state, manager.engine.phase))
        self.assertEqual(laps, {1, 2})
        self.assertEqual(phases, {'NATIVE_INTERMEDIATE_DESCEND', 'NATIVE_SECOND_FRAME', 'NATIVE_DESCEND', 'WAIT_ACK', 'SETTLE', 'NATIVE_RECOVER',
                                  'LAND_FIRST_FRAME', 'LAND_DESCEND'})
        self.assertEqual(manager.engine.completed_laps, 2)
        self.assertEqual(manager.engine.route_released_slots, [1, 2, 3, 4])
        self.assertEqual(manager.send_payload.call_count, 4)
        self.assertEqual(manager.runner.requests.count('EGO'), 1)
        self.assertEqual(manager.runner.requests[-1], 'NONE')
        self.assertEqual(manager.requested_group, 'NONE')

    def test_camera_warms_before_navigation_and_h_keeps_original_lifecycle(self):
        manager, runtime, replay = self.start_route()
        warm_job = manager.red_point_job
        self.assertTrue(warm_job['controlled_window'])
        self.assertEqual(manager.engine.state, 'TAKEOFF')
        running_in = []
        for _ in range(8000):
            self.step(manager, runtime)
            engine = manager.engine
            if engine.state == 'LAND':
                break
            job = getattr(manager, 'red_point_job', None)
            if job is not None and not job['stop'].is_set():
                running_in.append((engine.state, engine.phase, engine.route_lap))
                if job['target'] == 'red_point':
                    self.assertIs(job, warm_job)
                    self.assertIn(engine.state, ('TAKEOFF', 'APPROACH', 'DROP'))
                    self.assertFalse(engine.route_returning_home)
                else:
                    self.assertEqual(engine.phase, 'LAND_FIRST_FRAME')
        else:
            self.fail((manager.engine.state, manager.engine.phase))
        # No images are fed: existing deadlines still finish all four drops.
        # One warm red worker spans both laps; the H wait starts its own worker.
        self.assertEqual(len(RecordingObserver.created), 2)
        self.assertEqual({lap for _, _, lap in running_in}, {1, 2})
        self.assertEqual(running_in[-1][1], 'LAND_FIRST_FRAME')
        for job in manager.red_point_jobs:
            self.assertTrue(job['finished'].wait(2))
        self.assertTrue(all(observer.closed for observer in RecordingObserver.created))
        sessions = [observer.session for observer in RecordingObserver.created]
        self.assertEqual(len(set(sessions)), 2)
        commands = [observer.command('DOWN', 'x') for observer in RecordingObserver.created]
        self.assertEqual(len({command[3] for command in commands}), 1)  # One frozen copy for the whole task.
        self.assertEqual(commands[0][-1], '--controlled-window')
        self.assertEqual(commands[1][-2:], ['--target', 'landing_pad'])
        windows = [v for kind, v in manager.events_seen if kind == 'red_point_window']
        self.assertEqual([v['target'] for v in windows], ['red_point']*4 + ['landing_pad'])

    def test_priority_landing_during_drop_exits_worker_and_stops_route(self):
        manager, runtime, replay = self.start_route()
        self.until(manager, runtime, lambda: manager.engine.phase == 'NATIVE_DESCEND')
        manager.on_priority_landing(Bool(True))
        runtime.packet.update(owner='PX4_AUTO_LAND', mode='AUTO.LAND', offboard=False)
        sent = len(runtime.publisher.messages)
        for _ in range(5):
            self.step(manager, runtime, follow=False)
        self.assertFalse(manager.active)
        self.assertEqual(manager.engine.state, 'HOLD')
        self.assertEqual(manager.requested_group, 'NONE')
        self.assertEqual(len(runtime.publisher.messages), sent)
        manager.send_payload.assert_not_called()

    def test_observer_failure_uses_image_timeout_then_releases_without_ego_restart(self):
        manager, runtime, replay = self.start_route()
        self.until(manager, runtime, lambda: manager.engine.phase == 'NATIVE_SECOND_FRAME')
        ego_session = manager.worker_sessions['EGO']
        job = manager.red_point_job
        manager.on_red_point_error(job, 'camera unavailable')
        self.assertEqual(manager.engine.phase, 'NATIVE_SECOND_FRAME')
        self.assertEqual(manager.engine.state, 'DROP')
        self.step(manager, runtime, dt=15.1)
        self.assertEqual(manager.engine.phase, 'NATIVE_DESCEND')
        self.until(manager, runtime, lambda: manager.send_payload.call_count == 1)
        self.assertEqual(manager.worker_sessions['EGO'], ego_session)
        self.assertEqual(manager.runner.requests.count('EGO'), 1)
        self.assertNotEqual(manager.engine.state, 'HOLD')

    def test_operator_hold_during_drop_exits_worker_and_keeps_native_hold(self):
        manager, runtime, replay = self.start_route()
        self.until(manager, runtime, lambda: manager.engine.phase == 'NATIVE_DESCEND')
        session = manager.worker_sessions['EGO']
        command = PositionCommand()
        command.header.frame_id, command.header.stamp = LIO_FRAME, Stamp(Clock.now)
        command.position = Vector(999., 999., 999.)
        delayed = bound_command(manager, command)
        self.assertTrue(manager.dispatch(dict(command='hold'))['ok'])
        held = runtime.reference_command['position']
        manager.on_ego_cmd(session, delayed)
        for _ in range(5):
            self.step(manager, runtime, follow=False)
        self.assertFalse(manager.active)
        self.assertEqual(manager.engine.state, 'HOLD')
        self.assertEqual(manager.requested_group, 'NONE')
        self.assertTrue(runtime.full_enabled)
        self.assertIsNone(runtime.ego_command)
        self.assertEqual(runtime.reference_command['position'], held)
        emitted = runtime.publisher.messages[-1]
        self.assertEqual((emitted.position.x, emitted.position.y, emitted.position.z), held)
        manager.send_payload.assert_not_called()


class OptionalRedPointWorkerTests(unittest.TestCase):
    """No ROS, subprocesses, camera, detector imports or GUI in these tests."""

    def setUp(self):
        self.modules = mock.patch.dict(sys.modules, STUBS)
        self.modules.start()
        self.addCleanup(self.modules.stop)
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.manager = make_manager()
        self.manager.log_dir = Path(self.temporary.name)
        self.manager.engine = DeliveryRouteEngine(configuration())
        self.manager.engine.mode = 'delivery_route'
        self.manager.engine.state = 'DROP'
        self.manager.engine.phase = 'NATIVE_FIRST_FRAME'
        self.manager.engine.on_red_point_observation = mock.Mock(return_value=True)
        self.manager.engine.set_red_point_session = mock.Mock()
        self.manager.active = True
        self.manager.requested_group = 'EGO'
        self.manager.worker_sessions = {'EGO': 'ego-preserved'}
        self.manager.ego_goal_pub = ROS.Publisher()
        self.manager.dynamic_subscribers = [ROS.Subscriber()]
        self.ego_runner = self.manager.runner
        self.ego_pub = self.manager.ego_goal_pub
        self.ego_subscriber = self.manager.dynamic_subscribers[0]
        self.created = []
        self.started = threading.Event()
        self.allow_cleanup = threading.Event()
        self.allow_cleanup.set()
        owner = self

        class ObserverRunner:
            def __init__(self, command, directory, on_start, on_error, event, env=None):
                self.session = 'red-point-%d' % (len(owner.created)+1)
                self.command, self.on_start, self.on_error = command, on_start, on_error
                self.requests, self.closed = [], False
                owner.created.append(self)

            def request(self, group):
                self.requests.append(group)
                if group == 'DOWN':
                    self.argv = self.command(group, self.session)
                    self.on_start(group, self.session)
                    owner.started.set()

            def close(self):
                owner.allow_cleanup.wait()
                self.closed = True

        self.factory = mock.patch.object(MANAGER, 'ComputeRunner', ObserverRunner)
        self.factory.start()
        self.addCleanup(self.factory.stop)
        self.addCleanup(self.stop_observers)

    def stop_observers(self):
        self.allow_cleanup.set()
        with self.manager.lock:
            self.manager.stop_red_point_worker()
        for job in getattr(self.manager, 'red_point_jobs', []):
            self.assertTrue(job['finished'].wait(2), 'observer cleanup did not finish')
            job['thread'].join(2)

    def start(self):
        self.started.clear()
        with self.manager.lock:
            self.manager.start_red_point_worker(self.manager.engine.config)
        self.assertTrue(self.started.wait(2), 'observer did not start')
        return self.manager.red_point_job, self.created[-1]

    @staticmethod
    def observation(session, sequence=1):
        return String(json.dumps(dict(schema='airseafly.red_point.v1', session_id=session,
            image_sequence=sequence, image_stamp_s=Clock.now, candidates=[])))

    def assert_ego_preserved(self):
        self.assertIs(self.manager.runner, self.ego_runner)
        self.assertEqual(self.manager.worker_sessions, {'EGO': 'ego-preserved'})
        self.assertIs(self.manager.ego_goal_pub, self.ego_pub)
        self.assertEqual(self.manager.dynamic_subscribers, [self.ego_subscriber])
        self.assertFalse(self.ego_subscriber.unregistered)
        self.assertEqual(self.manager.requested_group, 'EGO')

    def test_frozen_config_child_command_and_session_callback_preserve_ego(self):
        job, runner = self.start()
        source = self.manager.engine.config['delivery_route_options']['visual_alignment']['red_point_config']
        self.assertEqual(Path(job['config_path']).read_text(), Path(source).read_text())
        self.assertEqual(runner.argv[0], self.manager.engine.config['runtime']['python'])
        self.assertEqual(Path(runner.argv[1]).name, 'red_point_worker.py')
        self.assertEqual(runner.argv[2:], ['--config', job['config_path'], '--session', runner.session])
        self.manager.engine.set_red_point_session.assert_called_with(runner.session)
        self.manager.on_red_point_observation(job, runner.session, self.observation(runner.session))
        self.manager.engine.on_red_point_observation.assert_called_once()
        self.assertEqual(self.manager.engine.on_red_point_observation.call_args.kwargs, {'now': Clock.now})
        self.assertTrue(self.manager.red_point_status['available'])
        self.assert_ego_preserved()

    def test_wrong_session_malformed_or_stopped_callbacks_do_not_reach_engine(self):
        job, runner = self.start()
        self.manager.on_red_point_observation(job, runner.session, self.observation('previous'))
        self.manager.on_red_point_observation(job, runner.session, String('not-json'))
        self.manager.on_red_point_observation(job, runner.session, String('[]'))
        subscriber = self.manager.red_point_subscriber
        with self.manager.lock:
            self.manager.stop_red_point_worker()
        self.manager.on_red_point_observation(job, runner.session, self.observation(runner.session))
        self.manager.engine.on_red_point_observation.assert_not_called()
        self.manager.engine.set_red_point_session.assert_called_with(None)
        self.assertTrue(subscriber.unregistered)
        self.assert_ego_preserved()

    def test_warm_observer_rejects_previous_window_and_does_not_extend_deadline(self):
        self.manager.engine.phase_started_at = Clock.now
        window = self.manager.engine.red_point_window()
        with self.manager.lock:
            self.manager.start_red_point_worker(self.manager.engine.config, controlled_window=True)
        self.assertTrue(self.started.wait(2))
        job, runner = self.manager.red_point_job, self.created[-1]
        self.assertEqual(runner.argv[-1], '--controlled-window')
        publisher = self.manager.red_point_window_pub
        self.assertEqual(publisher.messages[-1].data, window)
        original = self.manager.engine._alignment_deadline()
        packet = json.loads(self.observation(runner.session).data)
        for old in (None, 'NATIVE_FIRST_FRAME@0.000000'):
            packet['image_window'] = old
            self.manager.on_red_point_observation(job, runner.session, String(json.dumps(packet)))
        self.manager.engine.on_red_point_observation.assert_not_called()
        packet['image_window'] = window
        self.manager.on_red_point_observation(job, runner.session, String(json.dumps(packet)))
        self.manager.engine.on_red_point_observation.assert_called_once()
        self.assertEqual(self.manager.engine._alignment_deadline(), original)
        self.manager.engine.phase = 'NATIVE_SECOND_MOVE'
        self.manager.on_red_point_observation(job, runner.session, String(json.dumps(packet)))
        self.assertEqual(self.manager.engine.on_red_point_observation.call_count, 1)
        self.manager.publish_red_point_window()
        self.assertEqual(publisher.messages[-1].data, '')
        self.manager.stop_red_point_worker()
        self.assertTrue(publisher.unregistered)
        self.assert_ego_preserved()

    def test_prewarm_failure_keeps_original_attempt_at_each_new_image_window(self):
        manager = self.manager
        manager.engine.state, manager.engine.phase = 'TAKEOFF', 'WAIT_NATIVE'
        manager.freeze_red_point_config(manager.engine.config)
        before_image = dict(state='TAKEOFF', phase='WAIT_NATIVE', red_point_window=None)
        manager.sync_red_point_window(before_image)
        self.assertTrue(self.started.wait(2))
        self.created[-1].on_error('prewarm failed')
        self.assertTrue(manager.red_point_job['finished'].wait(2))
        for _ in range(3):
            manager.sync_red_point_window(before_image)
        self.assertEqual(len(self.created), 1)
        manager.engine.state, manager.engine.phase = 'DROP', 'NATIVE_SECOND_FRAME'
        for attempt in range(2):
            manager.engine.phase_started_at = Clock.now+attempt
            window = manager.engine.red_point_window()
            output = dict(state='DROP', phase=manager.engine.phase, red_point_window=window)
            original_deadline = manager.engine._alignment_deadline()
            self.started.clear()
            manager.sync_red_point_window(output)
            self.assertTrue(self.started.wait(2))
            self.assertEqual(len(self.created), attempt+2)
            self.created[-1].on_error('image worker failed')
            self.assertTrue(manager.red_point_job['finished'].wait(2))
            for _ in range(3):
                manager.sync_red_point_window(output)
            self.assertEqual(len(self.created), attempt+2)
            self.assertEqual(manager.engine._alignment_deadline(), original_deadline)
        self.assert_ego_preserved()

    def test_detector_failure_never_holds_or_restarts_ego_and_next_task_is_fresh(self):
        old_job, old_runner = self.start()
        old_runner.on_error('camera unavailable')
        self.assertTrue(old_job['finished'].wait(2))
        self.assertEqual(self.manager.engine.state, 'DROP')
        self.assertEqual(self.manager.red_point_status['error'], 'camera unavailable')
        self.assertEqual(len(self.created), 1)  # No automatic retry.
        self.assert_ego_preserved()
        self.manager.task_id = 'next-task'
        new_job, new_runner = self.start()
        self.assertNotEqual(new_runner.session, old_runner.session)
        self.assertNotEqual(new_job['config_path'], old_job['config_path'])
        self.manager.on_red_point_observation(old_job, old_runner.session, self.observation(old_runner.session))
        self.manager.engine.on_red_point_observation.assert_not_called()
        self.assert_ego_preserved()

    def test_replacement_does_not_block_flight_or_reuse_camera_before_cleanup(self):
        old_job, old_runner = self.start()
        self.allow_cleanup.clear()
        self.manager.task_id = 'replacement'
        self.started.clear()
        with self.manager.lock:
            self.manager.start_red_point_worker(self.manager.engine.config)
        replacement = self.manager.red_point_job
        self.assertIsNot(replacement, old_job)
        self.assertEqual(self.manager.engine.state, 'DROP')
        self.assertFalse(self.started.wait(.05))
        self.assertEqual(len(self.created), 1)
        self.assert_ego_preserved()
        self.allow_cleanup.set()
        self.assertTrue(self.started.wait(2))
        self.assertTrue(old_runner.closed)
        self.assertEqual(len(self.created), 2)

    def test_missing_config_only_marks_observer_unavailable(self):
        config = deepcopy(self.manager.engine.config)
        config['delivery_route_options']['visual_alignment']['red_point_config'] = '/missing/red_point.yaml'
        with self.manager.lock:
            self.manager.start_red_point_worker(config)
        self.assertEqual(self.created, [])
        self.assertEqual(self.manager.engine.state, 'DROP')
        self.assertTrue(self.manager.active)
        self.assertIn('red_point_unavailable', [kind for kind, _ in self.manager.events_seen])
        self.assert_ego_preserved()


if __name__ == '__main__':
    unittest.main()
