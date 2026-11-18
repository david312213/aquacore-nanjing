"""Single-drop manager integration with inert flight and observer endpoints."""
import json
import os
from pathlib import Path
import tempfile
import threading
import unittest
from unittest import mock

import test_native_delivery_manager as delivery_tests
from ros_fixtures import MANAGER, Clock, String
from test_native_delivery_route import configuration
from airseafly_real400_mission.native_single_drop import SingleDropEngine
from airseafly_real400_mission.native_reference import LIO_FRAME


class SingleDropManagerTests(unittest.TestCase):
    setup_manager = delivery_tests.DeliveryManagerTests.setup_manager
    step = delivery_tests.DeliveryManagerTests.step
    until = delivery_tests.DeliveryManagerTests.until

    def setUp(self):
        delivery_tests.DeliveryManagerTests.setUp(self)
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.observer_started = threading.Event()
        self.observers, self.managers = [], []
        owner = self

        class ObserverRunner:
            def __init__(self, command, directory, on_start, on_error, event, env=None):
                self.command, self.on_start, self.on_error = command, on_start, on_error
                self.session = 'single-drop-%d' % (len(owner.observers)+1)
                self.closed = False
                owner.observers.append(self)

            def request(self, group):
                if group == 'DOWN':
                    self.argv = self.command(group, self.session)
                    self.on_start(group, self.session)
                    owner.observer_started.set()

            def close(self):
                self.closed = True

        factory = mock.patch.object(MANAGER, 'ComputeRunner', ObserverRunner)
        factory.start()
        self.addCleanup(factory.stop)
        self.addCleanup(self.stop_observers)

    def stop_observers(self):
        for manager in self.managers:
            with manager.lock:
                manager.stop_red_point_worker()
            for job in getattr(manager, 'red_point_jobs', []):
                self.assertTrue(job['finished'].wait(2))
                job['thread'].join(2)

    def start_drop(self, slot=3):
        manager, runtime, replay = self.setup_manager()
        self.managers.append(manager)
        manager.log_dir = Path(self.temporary.name)
        config = configuration()
        config.pop('delivery_route')
        config['drop_test'] = dict(target_xy=[-6., 7.], slot=slot)
        config['_flight_configuration'] = {'task': 'delivery-route'}
        self.observer_started.clear()
        with mock.patch.dict(os.environ, ROS_MASTER_URI=config['runtime']['ros_master_uri']), \
                mock.patch.object(runtime, 'acquire_full'):
            result = manager.dispatch(dict(command='start', mode='drop_test', resolved_config=config))
        self.assertTrue(result['ok'], result)
        # Camera startup overlaps takeoff; detection is disabled until its window.
        self.assertTrue(manager.red_point_job['controlled_window'])
        self.assertTrue((manager.log_dir/('red_point_%s.yaml' % manager.task_id)).exists())
        runtime.start_takeoff, runtime.request = mock.Mock(), mock.Mock()
        runtime.full_owner_since = Clock.now
        runtime.packet.update(owner='PX4_NATIVE', landed=False, armed=True,
                              offboard=True, mode='OFFBOARD')
        manager.control_state = 'PX4_NATIVE'
        manager.send_payload = mock.Mock(side_effect=lambda chosen, action, request_id, **kw:
            manager.engine.on_payload_result(True, request_id))
        return manager, runtime, replay

    def wait_observer(self):
        self.assertTrue(self.observer_started.wait(2), 'image-phase observer did not start')
        self.observer_started.clear()

    def feed_target(self, manager, sequence, xy):
        session = manager.red_point_session
        packet = dict(schema='airseafly.red_point.v1', session_id=session,
            image_window=manager.engine.red_point_window(),
            image_sequence=sequence, image_stamp_s=Clock.now, frame_id=LIO_FRAME,
            green_candidates=[dict(class_name='green_circle', target_confirmed=True, projection_status='ok_near_level', xy_m=list(xy))],
            candidates=[dict(target_confirmed=True, projection_status='ok_near_level', xy_m=list(xy))])
        manager.on_red_point_observation(manager.red_point_job, session, String(json.dumps(packet)))

    def test_start_selects_requested_port_and_observer_then_returns_to_actual_origin(self):
        manager, runtime, replay = self.start_drop(slot=3)
        engine = manager.engine
        self.assertIsInstance(engine, SingleDropEngine)
        self.assertEqual(engine.mode, 'drop_test')
        self.assertEqual(engine.native_origin, replay.ground)
        self.assertIsNone(manager.flight_origin)
        self.assertEqual(runtime.parameters[runtime.ORIGIN_PARAM]['radar_origin_xyz'], list(replay.ground))
        status = engine.snapshot()
        self.assertEqual(status['payload_total'], 1)
        self.assertEqual(status['drop_test']['slot'], 3)
        self.assertEqual(status['drop_test']['target_xy'], [-6., 7.])
        self.assertEqual(list(status['drop_test']['home_xy']), list(replay.ground[:2]))
        self.assertNotIn('delivery_route', status)
        self.until(manager, runtime, lambda: engine.phase == 'NATIVE_INTERMEDIATE_DESCEND')
        self.wait_observer()
        self.assertEqual(manager.red_point_window_pub.messages[-1].data, '')
        self.until(manager, runtime, lambda: engine.phase == 'NATIVE_SECOND_FRAME')
        self.assertEqual(len(self.observers), 1)
        self.assertEqual(self.observers[0].argv[0], engine.config['runtime']['python'])
        ego_session = manager.worker_sessions['EGO']
        self.feed_target(manager, 1, [-5.85, 7.05])
        self.assertEqual(engine.phase, 'NATIVE_SECOND_MOVE')
        self.step(manager, runtime)
        self.assertFalse(manager.red_point_job['stop'].is_set())  # Camera stays warm.
        self.assertEqual(manager.red_point_window_pub.messages[-1].data, '')
        self.until(manager, runtime, lambda: engine.state == 'APPROACH' and engine.drops_completed == 1)
        self.assertEqual(manager.worker_sessions['EGO'], ego_session)
        self.assertEqual(tuple(engine.command_position[:2]), replay.ground[:2])
        self.until(manager, runtime, lambda: engine.native_pending == 'PX4_AUTO_LAND')
        self.assertEqual(manager.send_payload.call_count, 1)
        self.assertEqual(manager.send_payload.call_args.args[:2], (3, 'release'))
        self.assertEqual(engine.drops_completed, 1)
        self.assertEqual(tuple(engine.position[:2]), replay.ground[:2])
        self.assertEqual(manager.runner.requests.count('EGO'), 1)
        self.assertEqual(manager.runner.requests[-1], 'NONE')
        self.assertEqual(engine._navigation_sequence, 2)
        self.assertFalse(engine.done)
        self.assertTrue(manager.red_point_job['stop'].is_set())
        self.assertTrue(manager.red_point_job['finished'].wait(2))
        self.assertTrue(all(observer.closed for observer in self.observers))
        self.assertEqual(len(self.observers), 1)  # One image phase only.
        runtime.request.assert_called_once()
        engine.on_native_owner_result('PX4_AUTO_LAND', True, 'PX4_AUTO_LAND', 'queued')
        self.step(manager, runtime)
        self.assertFalse(engine.done)  # Service acknowledgement is not landing.
        runtime.packet.update(owner='PX4_AUTO_LAND', mode='AUTO.LAND', offboard=False)
        self.step(manager, runtime)
        self.assertEqual(engine.phase, 'LANDING')
        self.assertFalse(engine.done)
        runtime.packet.update(landed=True, armed=False)
        self.step(manager, runtime)
        self.assertEqual(engine.phase, 'LANDED')
        self.assertTrue(engine.done)
        self.assertFalse(manager.active)
        self.assertEqual(manager.requested_group, 'NONE')

    def test_observer_failure_keeps_single_drop_timeout_fallback_and_selected_port(self):
        manager, runtime, replay = self.start_drop(slot=4)
        self.until(manager, runtime, lambda: manager.engine.phase == 'NATIVE_SECOND_FRAME')
        self.wait_observer()
        ego_session = manager.worker_sessions['EGO']
        self.observers[0].on_error('camera unavailable')
        self.assertTrue(manager.red_point_job['finished'].wait(2))
        self.assertEqual(manager.engine.phase, 'NATIVE_SECOND_FRAME')
        self.step(manager, runtime, dt=15.1)
        self.assertEqual(manager.engine.phase, 'NATIVE_DESCEND')
        self.assertEqual(manager.engine.native_goal[:2], (-6., 7.))
        self.until(manager, runtime, lambda: manager.send_payload.call_count == 1)
        self.assertEqual(manager.send_payload.call_args.args[:2], (4, 'release'))
        self.assertEqual(manager.worker_sessions['EGO'], ego_session)
        self.assertEqual(len(self.observers), 1)
        self.assertNotEqual(manager.engine.state, 'HOLD')


if __name__ == '__main__':
    unittest.main()
