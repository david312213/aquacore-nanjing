"""Exercise manager, native preparation, observer and EGO reuse together."""
import os
from pathlib import Path
import unittest
from unittest import mock

import test_native_single_drop_manager as single_tests
from test_native_delivery_route import configuration
from test_drop_points import document
from ros_fixtures import Clock
from airseafly_real400_mission.native_drop_points import DropPointsEngine


class DropPointsManagerTests(unittest.TestCase):
    setup_manager = single_tests.SingleDropManagerTests.setup_manager
    setUp = single_tests.SingleDropManagerTests.setUp
    stop_observers = single_tests.SingleDropManagerTests.stop_observers
    step = single_tests.SingleDropManagerTests.step
    until = single_tests.SingleDropManagerTests.until
    feed_target = single_tests.SingleDropManagerTests.feed_target
    wait_observer = single_tests.SingleDropManagerTests.wait_observer

    def test_recorded_four_points_share_warm_camera_then_land_at_actual_home(self):
        manager, runtime, replay = self.setup_manager()
        self.managers.append(manager)
        manager.log_dir = Path(self.temporary.name)
        config = configuration()
        config.pop('delivery_route')
        config['drop_points'] = document()
        config['_flight_configuration'] = {'task': 'delivery-route'}
        with mock.patch.dict(os.environ, ROS_MASTER_URI=config['runtime']['ros_master_uri']), \
                mock.patch.object(runtime, 'acquire_full'):
            result = manager.dispatch(dict(command='start', mode='drop_points', resolved_config=config))
        self.assertTrue(result['ok'], result)
        self.assertTrue(manager.red_point_job['controlled_window'])
        self.wait_observer()
        self.assertEqual(manager.red_point_window_pub.messages[-1].data, '')
        engine = manager.engine
        self.assertIsInstance(engine, DropPointsEngine)
        self.assertEqual(engine.native_origin, replay.ground)
        self.assertEqual(engine.route_home, replay.ground[:2])
        self.assertEqual(runtime.parameters[runtime.ORIGIN_PARAM]['radar_origin_xyz'], list(replay.ground))
        runtime.start_takeoff, runtime.request = mock.Mock(), mock.Mock()
        runtime.full_owner_since = Clock.now
        runtime.packet.update(owner='PX4_NATIVE', landed=False, armed=True, offboard=True, mode='OFFBOARD')
        manager.control_state = 'PX4_NATIVE'
        manager.send_payload = mock.Mock(side_effect=lambda chosen, action, request_id, **kw:
            engine.on_payload_result(True, request_id))
        for index in range(4):
            self.until(manager, runtime, lambda: engine.route_index == index and engine.phase == 'NATIVE_SECOND_FRAME')
            self.feed_target(manager, index+1, document()['points'][index])
            self.assertEqual(engine.phase, 'NATIVE_SECOND_MOVE')
        self.until(manager, runtime, lambda: engine.native_pending == 'PX4_AUTO_LAND')
        self.assertEqual([call.args[:2] for call in manager.send_payload.call_args_list],
                         [(1, 'release'), (2, 'release'), (3, 'release'), (4, 'release')])
        self.assertEqual(engine.route_released_points, {0, 1, 2, 3})
        self.assertEqual(tuple(engine.position[:2]), replay.ground[:2])
        self.assertEqual(manager.runner.requests.count('EGO'), 1)
        self.assertEqual(len(self.observers), 1)
        self.assertEqual(len({observer.session for observer in self.observers}), 1)
        self.assertEqual([r['alignment']['samples'] for r in engine.native_results], [1]*4)
        self.assertTrue(manager.red_point_job['stop'].is_set())
        self.assertFalse(engine.done)


if __name__ == '__main__':
    unittest.main()
