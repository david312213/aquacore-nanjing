"""Priority landing observations and mission cancellation, no live ROS calls."""
import ast
from pathlib import Path
import threading
import sys
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

from airseafly_real400_mission.landing import LandingObservation
from ros_fixtures import Bool, make_manager, packet, PositionCommand, ROS, STUBS


class LandingTests(unittest.TestCase):
    def test_completion_requires_actual_landing_and_fresh_fcu_disarm(self):
        state = LandingObservation()
        for key, value in [('connected', True), ('armed', False), ('on_ground', True), ('landed', True)]:
            state.update(key, value, 10.)
        self.assertFalse(state.complete(10.))
        state.update('control', 'AUTO_LAND', 10.)
        self.assertTrue(state.complete(10.))
        state.update('armed', True, 10.)
        self.assertFalse(state.complete(10.))
        state.update('armed', False, 10.)
        self.assertFalse(state.complete(13.))

    def test_landing_detector_alone_is_not_complete(self):
        state = LandingObservation()
        state.update('control', 'AUTO_LAND', 10.)
        state.update('landed', True, 10.)
        self.assertFalse(state.complete(10.))

    def test_rc_handoff_report_and_late_disarm(self):
        state = LandingObservation()
        state.update('control', 'AUTO_LAND', 10.)
        state.update('priority', True, 10.)
        state.update('control', 'MANUAL_CTRL', 11.)
        state.update('priority', False, 11.)
        self.assertTrue(state.interrupted(11.))
        for key, value in [('connected', True), ('armed', False), ('on_ground', True)]:
            state.update(key, value, 11.1)
        self.assertTrue(state.complete(11.1))
        self.assertFalse(state.interrupted(11.1))

    def test_manager_stops_without_another_position_or_payload_effect(self):
        m = make_manager('takeoff')
        m.requested_group = 'EGO'
        m.worker_sessions['EGO'] = 'current'
        m.handoff_pending = True
        m.ego_cmd = PositionCommand()
        m.on_priority_landing(Bool(data=True))
        self.assertFalse(m.active)
        self.assertFalse(m.handoff_pending)
        self.assertEqual(m.requested_group, 'NONE')
        self.assertIsNone(m.expected_session)
        self.assertIsNone(m.ego_cmd)
        m.tick(None)
        self.assertEqual(m.position_pub.messages, [])
        self.assertEqual(m.land_pub.messages, [])
        self.assertFalse(m.dispatch(dict(command='start', mode='full'))['ok'])
        self.assertFalse(m.dispatch(dict(command='payload', action='release', slot=2))['ok'])
        self.assertTrue(m.dispatch(dict(command='hold'))['ok'])
        self.assertFalse(m.handoff_pending)
        self.assertEqual(m.engine.state, 'HOLD')
        m.on_priority_landing(Bool(data=False))
        m.tick(None)
        self.assertFalse(m.active)
        self.assertEqual(m.position_pub.messages, [])

    def test_duplicate_priority_message_does_not_restart_cleanup(self):
        m = make_manager('takeoff')
        m.on_priority_landing(Bool(data=True))
        events = len(m.events_seen)
        m.on_priority_landing(Bool(data=True))
        self.assertEqual(len(m.events_seen), events)

    def test_landing_cancels_a_payload_still_waiting_for_service(self):
        m = make_manager('takeoff')
        entered, resume, finished = threading.Event(), threading.Event(), threading.Event()
        def wait(*args, **kwargs):
            entered.set()
            self.assertTrue(resume.wait(1.))
        with mock.patch.dict(sys.modules, STUBS), \
                mock.patch.object(ROS, 'wait_for_service', side_effect=wait), \
                mock.patch.object(ROS, 'ServiceProxy') as service:
            m.send_payload(2, 'release', 12, for_mission=True, done=lambda r: finished.set())
            self.assertTrue(entered.wait(1.))
            m.on_priority_landing(Bool(data=True))
            resume.set()
            self.assertTrue(finished.wait(1.))
            service.assert_not_called()
        self.assertEqual(m.engine.drops_completed, 0)

    def test_launch_is_request_only_and_client_has_no_output_publisher(self):
        package = Path(__file__).resolve().parents[1]
        launch = ET.parse(package / 'launch/safe_land.launch').getroot()
        self.assertEqual(launch.findall('include'), [])
        self.assertEqual([n.get('type') for n in launch.findall('node')], ['safe_land.py'])
        tree = ast.parse((package / 'scripts/safe_land.py').read_text())
        calls = [n.func.attr for n in ast.walk(tree)
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
        self.assertNotIn('Publisher', calls)
        self.assertNotIn('Popen', calls)


if __name__ == '__main__':
    unittest.main()
