"""Native payload dispatch with fake services and delayed real acknowledgements."""
from pathlib import Path
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros_fixtures import STUBS, ROS, make_manager


class NativePayloadTransportTests(unittest.TestCase):
    def setUp(self):
        patch = mock.patch.dict(sys.modules, STUBS)
        patch.start()
        self.addCleanup(patch.stop)
        self.manager = make_manager()
        self.manager.engine.native_active = True
        self.manager.engine.state = 'DROP'
        self.manager.engine.phase = 'WAIT_ACK'
        self.manager.engine.native_feedback = dict(healthy=False, armed=False,
                                                  offboard=False, owner='PX4CTRL')
        self.manager.engine.on_payload_result = mock.Mock()
        self.manager.config['payload']['ack_timeout_s'] = .05
        self.manager.config['payload']['pd_profiles_enabled'] = False

    def test_legacy_dispatch_times_out_once_and_late_ack_is_audit_only(self):
        self.manager.engine.native_active = False
        entered, release_ack, finished, late = (threading.Event() for _ in range(4))
        requests, results = [], []
        self.manager.event = mock.Mock(side_effect=lambda kind, **kw: late.set() if kind == 'payload_late_result' else None)

        def actuator(request):
            requests.append(request)
            entered.set()
            release_ack.wait(1.)
            return SimpleNamespace(success=True, result=0)

        with mock.patch.object(ROS, 'wait_for_service') as wait_service, \
                mock.patch.object(ROS, 'ServiceProxy', return_value=actuator):
            self.manager.send_payload(2, 'release', 91, for_mission=True,
                                      done=lambda result: (results.append(result), finished.set()))
            self.assertTrue(entered.wait(1.))
            try:
                self.assertTrue(finished.wait(1.))
                self.assertFalse(results[0]['success'])
                self.assertEqual(results[0]['outcome'], 'unknown')
                self.assertIn(2, self.manager.payload_blocked_slots)
                self.assertEqual(self.manager.engine.on_payload_result.call_args.kwargs['outcome'], 'unknown')
                # A new mission request cannot reuse a slot with an unknown result.
                for request_id in (91, 92):
                    blocked = threading.Event()
                    self.manager.send_payload(2, 'release', request_id, for_mission=True,
                                              done=lambda result: blocked.set())
                    self.assertTrue(blocked.wait(1.))
                    self.assertEqual(len(requests), 1)
                    self.assertIn(2, self.manager.payload_blocked_slots)
            finally:
                release_ack.set()
                self.assertTrue(late.wait(1.))
            wait_service.assert_called_once()
            self.assertGreater(wait_service.call_args.kwargs['timeout'], 0.)
        self.assertEqual(len(requests), 1)
        self.assertEqual(requests[0].param2, 1.)
        self.assertEqual(len(results), 1)
        self.assertIn(2, self.manager.payload_blocked_slots)

    def test_service_appearing_after_deadline_cannot_dispatch(self):
        self.manager.engine.native_active = False
        entered, service_ready, finished, late = (threading.Event() for _ in range(4))
        results = []
        def wait_service(*args, **kwargs):
            entered.set(); service_ready.wait(1.)
        self.manager.event = mock.Mock(side_effect=lambda kind, **kw: late.set() if kind == 'payload_late_result' else None)
        with mock.patch.object(ROS, 'wait_for_service', side_effect=wait_service), \
                mock.patch.object(ROS, 'ServiceProxy') as proxy:
            self.manager.send_payload(2, 'release', 93, for_mission=True,
                                      done=lambda result: (results.append(result), finished.set()))
            self.assertTrue(entered.wait(1.)); self.assertTrue(finished.wait(1.))
            self.assertEqual(results[0]['outcome'], 'not_sent')
            service_ready.set(); self.assertTrue(late.wait(1.))
        proxy.assert_not_called()

    def test_native_request_waits_past_configured_timeout_for_real_ack(self):
        entered, reply, finished = (threading.Event() for _ in range(3))
        results = []
        def actuator(request):
            entered.set(); reply.wait(2.)
            return SimpleNamespace(success=True, result=0)
        with mock.patch.object(ROS, 'wait_for_service') as wait_service, \
                mock.patch.object(ROS, 'ServiceProxy', return_value=actuator) as proxy:
            self.manager.send_payload(2, 'release', 101, for_mission=True,
                                      done=lambda result: (results.append(result), finished.set()))
            try:
                self.assertTrue(entered.wait(1.))
                self.assertFalse(finished.wait(.15))
                self.manager.engine.on_payload_result.assert_not_called()
                self.assertIn(2, self.manager.payload_blocked_slots)
            finally:
                reply.set()
            self.assertTrue(finished.wait(1.))
        wait_service.assert_called_once_with('/mavros/cmd/command')
        proxy.assert_called_once()
        self.assertEqual(results[0]['outcome'], 'accepted')
        self.assertNotIn(2, self.manager.payload_blocked_slots)

    def test_native_service_can_become_ready_after_old_timeout(self):
        entered, ready, finished = (threading.Event() for _ in range(3))
        results = []
        def wait_service(*args, **kwargs):
            entered.set(); ready.wait(2.)
        actuator = mock.Mock(return_value=SimpleNamespace(success=True, result=0))
        with mock.patch.object(ROS, 'wait_for_service', side_effect=wait_service), \
                mock.patch.object(ROS, 'ServiceProxy', return_value=actuator):
            self.manager.send_payload(2, 'release', 102, for_mission=True,
                                      done=lambda result: (results.append(result), finished.set()))
            try:
                self.assertTrue(entered.wait(1.))
                self.assertFalse(finished.wait(.15))
            finally:
                ready.set()
            self.assertTrue(finished.wait(1.))
        actuator.assert_called_once()
        self.assertEqual(results[0]['outcome'], 'accepted')

    def test_native_delayed_discovery_cannot_release_after_task_replacement(self):
        entered, ready, finished = (threading.Event() for _ in range(3))
        results = []
        def wait_service(*args, **kwargs):
            entered.set(); ready.wait(2.)
        actuator = mock.Mock(return_value=SimpleNamespace(success=True, result=0))
        with mock.patch.object(ROS, 'wait_for_service', side_effect=wait_service), \
                mock.patch.object(ROS, 'ServiceProxy', return_value=actuator):
            old_engine = self.manager.engine
            self.manager.send_payload(2, 'release', 103, for_mission=True,
                                      done=lambda result: (results.append(result), finished.set()))
            try:
                self.assertTrue(entered.wait(1.))
                self.manager.engine = SimpleNamespace(state='DROP')
            finally:
                ready.set()
            self.assertTrue(finished.wait(1.))
        actuator.assert_not_called()
        old_engine.on_payload_result.assert_not_called()
        self.assertEqual(results[0]['outcome'], 'not_sent')

    def test_rejected_release_reports_failure_without_retry(self):
        finished, results = threading.Event(), []
        with mock.patch.object(ROS, 'wait_for_service'), \
                mock.patch.object(ROS, 'ServiceProxy', return_value=mock.Mock(return_value=SimpleNamespace(success=False, result=1))) as proxy:
            self.manager.send_payload(2, 'release', 94, for_mission=True,
                                      done=lambda result: (results.append(result), finished.set()))
            self.assertTrue(finished.wait(1.))
        proxy.assert_called_once()
        self.assertEqual(results[0]['outcome'], 'rejected')

    def test_late_rejection_keeps_slot_locked_until_explicit_lock(self):
        self.manager.engine.native_active = False
        entered, reply, finished, late = (threading.Event() for _ in range(4))
        self.manager.event = mock.Mock(side_effect=lambda kind, **kw: late.set() if kind == 'payload_late_result' else None)
        def actuator(request):
            entered.set(); reply.wait(1.)
            return SimpleNamespace(success=False, result=1)
        with mock.patch.object(ROS, 'wait_for_service'), \
                mock.patch.object(ROS, 'ServiceProxy', return_value=actuator):
            self.manager.send_payload(2, 'release', 95, for_mission=True, done=lambda result: finished.set())
            self.assertTrue(entered.wait(1.)); self.assertTrue(finished.wait(1.))
            reply.set(); self.assertTrue(late.wait(1.))
            self.assertIn(2, self.manager.payload_blocked_slots)
        locked = threading.Event()
        with mock.patch.object(ROS, 'wait_for_service'), \
                mock.patch.object(ROS, 'ServiceProxy', return_value=lambda req: SimpleNamespace(success=True, result=0)):
            self.manager.send_payload(2, 'lock', 96, done=lambda result: locked.set())
            self.assertTrue(locked.wait(1.))
        self.assertNotIn(2, self.manager.payload_blocked_slots)

    def test_explicit_user_stop_cancels_a_not_yet_sent_release(self):
        self.manager.engine.state = 'HOLD'
        finished = threading.Event()
        actuator = mock.Mock()
        with mock.patch.object(ROS, 'wait_for_service'), \
                mock.patch.object(ROS, 'ServiceProxy', return_value=actuator):
            self.manager.send_payload(2, 'release', 92, for_mission=True,
                                      done=lambda result: finished.set())
            self.assertTrue(finished.wait(1.))
        actuator.assert_not_called()


if __name__ == '__main__':
    unittest.main()
