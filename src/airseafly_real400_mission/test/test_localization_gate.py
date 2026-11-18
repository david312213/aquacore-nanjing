"""Localization gate: trip rule on recorded-style samples, and the node with all aircraft IO mocked."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src/airseafly_real400_mission/src'))
from airseafly_real400_mission.flight_configuration import apply_flight_configuration, base_configuration
from airseafly_real400_mission.localization_gate import DivergenceGate

SCRIPT = ROOT/'src/airseafly_real400_mission/scripts/localization_gate_node.py'
spec = importlib.util.spec_from_file_location('localization_gate_node', SCRIPT)
node_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(node_module)


def feed(gate, samples, active=True):
    """samples: (stamp, error); returns stamps that tripped."""
    return [t for t, e in samples if gate.update(t, e, active)]


class DivergenceGateTests(unittest.TestCase):
    def test_normal_flight_errors_never_trip(self):
        # Worst normal flight on record peaked at 0.37 m.
        gate = DivergenceGate(0.5, 0.5)
        samples = [(i * .02, .37 if i % 50 == 0 else .05) for i in range(5000)]
        self.assertEqual(feed(gate, samples), [])
        self.assertFalse(gate.tripped)

    def test_threshold_is_strictly_above_and_hold_is_inclusive(self):
        gate = DivergenceGate(0.5, 0.5)
        self.assertEqual(feed(gate, [(i * .02, .5) for i in range(100)]), [])
        gate = DivergenceGate(0.5, 0.5)
        tripped = feed(gate, [(10 + i * .02, .51) for i in range(40)])
        self.assertEqual(len(tripped), 1)
        self.assertGreaterEqual(tripped[0] - 10, .5 - 1e-9)
        self.assertLess(tripped[0] - 10, .5 + .021)  # first 50 Hz sample at or past the hold
        self.assertAlmostEqual(gate.trip_error_m, .51)

    def test_short_spike_below_hold_does_not_trip(self):
        gate = DivergenceGate(0.5, 0.5)
        spike = [(i * .02, 1.0 if 100 <= i < 124 else .05) for i in range(300)]  # 0.46 s above
        self.assertEqual(feed(gate, spike), [])

    def test_a_dip_or_missing_sample_restarts_the_hold(self):
        for gap in (.3, None):
            with self.subTest(gap=gap):
                gate = DivergenceGate(0.5, 0.5)
                samples = [(i * .02, gap if i == 20 else 1.0) for i in range(60)]
                tripped = feed(gate, samples)
                self.assertEqual(len(tripped), 1)
                # The run before the dip (0.38 s) is discarded; a new run starts at 0.42 s.
                self.assertGreaterEqual(tripped[0], .42 + .5 - 1e-9)
                self.assertLess(tripped[0], .42 + .5 + .021)

    def test_inactive_samples_never_count(self):
        gate = DivergenceGate(0.5, 0.5)
        self.assertEqual(feed(gate, [(i * .02, 5.) for i in range(200)], active=False), [])
        tripped = feed(gate, [(4 + i * .02, 5.) for i in range(30)])
        self.assertEqual(len(tripped), 1)
        self.assertLess(abs(tripped[0] - 4.5), .021)

    def test_latched_until_reset(self):
        gate = DivergenceGate(0.5, 0.5)
        self.assertEqual(len(feed(gate, [(i * .1, 2.) for i in range(20)])), 1)
        self.assertEqual(feed(gate, [(5 + i * .1, .01) for i in range(20)]), [])
        self.assertTrue(gate.tripped)
        gate.reset()
        self.assertFalse(gate.tripped)
        self.assertIsNone(gate.trip_stamp)

    def test_sparse_replay_rate_needs_two_samples_spanning_hold(self):
        gate = DivergenceGate(0.5, 0.5)
        self.assertEqual(feed(gate, [(0., .9), (.49, .9)]), [])
        self.assertEqual(feed(gate, [(.5, .9)]), [.5])

    def test_invalid_settings_rejected(self):
        for args in ((0, .5), (-1, .5), (float('nan'), .5), (.5, -.1), (.5, float('inf'))):
            with self.subTest(args=args), self.assertRaises(ValueError):
                DivergenceGate(*args)


class FakeRos:
    def __init__(self):
        self.published, self.subscribers, self.logs = [], {}, []
        self.land_results, self.land_calls = [], 0
        self.called = threading.Event()

    def Publisher(self, topic, _type, **_kw):
        return SimpleNamespace(publish=lambda m: self.published.append(json.loads(m.data)))

    def Subscriber(self, topic, _type, callback, **_kw):
        self.subscribers[topic] = callback

    def ServiceProxy(self, name, _type):
        assert name == '/px4ctrl/force_land'
        def call():
            self.land_calls += 1
            self.called.set()
            ok, text = self.land_results.pop(0) if self.land_results else (True, 'accepted')
            if ok is None:
                raise RuntimeError(text)
            return SimpleNamespace(success=ok, message=text)
        return call

    def on_shutdown(self, _callback):
        pass

    def loginfo(self, *a): self.logs.append(('info', a))
    def logwarn(self, *a): self.logs.append(('warn', a))
    def logerr(self, *a): self.logs.append(('err', a))


class Text(SimpleNamespace):
    def __init__(self, data=''):
        super().__init__(data=data)


class GateNodeTests(unittest.TestCase):
    def setUp(self):
        self.saved_retry = node_module.RETRY_S
        node_module.RETRY_S = .05
        self.ros = FakeRos()
        self.config = apply_flight_configuration(base_configuration(), task='delivery-route')
        self.node = node_module.LocalizationGateNode(self.ros, object, Text, object, self.config)

    def tearDown(self):
        self.node.close()
        node_module.RETRY_S = self.saved_retry

    def state(self, armed, mode):
        self.ros.subscribers['/mavros/state'](SimpleNamespace(armed=armed, mode=mode))

    def errors(self, start, error, seconds=1.0, rate=50.):
        for i in range(int(seconds * rate) + 1):
            packet = dict(stamp=start + i / rate, px4_error_m=error, healthy=True)
            self.ros.subscribers['/airseafly/native/localization_status'](Text(json.dumps(packet)))

    def wait_calls(self, count, timeout=2.):
        deadline = time.monotonic() + timeout
        while self.ros.land_calls < count and time.monotonic() < deadline:
            time.sleep(.01)
        return self.ros.land_calls

    def test_trip_in_offboard_requests_landing_once(self):
        self.state(True, 'OFFBOARD')
        self.errors(100., .9)
        self.assertEqual(self.wait_calls(1), 1)
        time.sleep(.2)
        self.assertEqual(self.ros.land_calls, 1)
        last = self.ros.published[-1]
        self.assertTrue(last['tripped'] and last['land_accepted'])

    def test_no_request_outside_armed_offboard(self):
        for armed, mode in ((False, 'OFFBOARD'), (True, 'POSCTL'), (True, 'AUTO.LAND'), (False, 'MANUAL')):
            with self.subTest(armed=armed, mode=mode):
                self.state(armed, mode)
                self.errors(200., 3.)
                time.sleep(.1)
                self.assertEqual(self.ros.land_calls, 0)
                self.assertFalse(self.node.gate.tripped)

    def test_rejected_request_retries_while_still_offboard(self):
        self.ros.land_results = [(False, 'busy'), (None, 'no service'), (True, 'ok')]
        self.state(True, 'OFFBOARD')
        self.errors(300., 1.2)
        self.assertEqual(self.wait_calls(3), 3)
        time.sleep(.2)
        self.assertEqual(self.ros.land_calls, 3)
        self.assertTrue(self.node.land_accepted)

    def test_pilot_leaving_offboard_stops_retries(self):
        self.ros.land_results = [(False, 'busy')] * 50
        self.state(True, 'OFFBOARD')
        self.errors(400., 1.2)
        self.wait_calls(1)
        self.state(True, 'POSCTL')
        time.sleep(.1)
        calls = self.ros.land_calls
        time.sleep(.3)
        self.assertEqual(self.ros.land_calls, calls)
        self.assertTrue(self.node.gate.tripped)

    def test_disarm_clears_trip_for_the_next_flight(self):
        self.state(True, 'OFFBOARD')
        self.errors(500., .9)
        self.wait_calls(1)
        self.state(False, 'AUTO.LAND')
        self.assertFalse(self.node.gate.tripped)
        self.assertEqual(self.node.land_attempts, 0)
        self.state(True, 'OFFBOARD')
        self.errors(600., .05)
        time.sleep(.1)
        self.assertEqual(self.ros.land_calls, 1)

    def test_disabled_gate_never_requests(self):
        self.node.close()
        self.config['localization_gate']['enabled'] = False
        self.ros = FakeRos()
        self.node = node_module.LocalizationGateNode(self.ros, object, Text, object, self.config)
        self.state(True, 'OFFBOARD')
        self.errors(700., 5.)
        time.sleep(.1)
        self.assertEqual(self.ros.land_calls, 0)

    def test_bad_status_packets_are_ignored(self):
        self.state(True, 'OFFBOARD')
        callback = self.ros.subscribers['/airseafly/native/localization_status']
        for data in ('', 'not json', '[1, 2]', json.dumps(dict(stamp=None, px4_error_m=9.))):
            callback(Text(data))
        time.sleep(.05)
        self.assertFalse(self.node.gate.tripped)


class GateConfigurationTests(unittest.TestCase):
    def test_flight_yaml_values_are_the_confirmed_rule(self):
        for task in (None, 'takeoff', 'waypoint-route', 'delivery-route'):
            with self.subTest(task=task):
                settings = apply_flight_configuration(base_configuration(), task=task)['localization_gate']
                self.assertEqual(settings, dict(enabled=True, max_error_m=.5, hold_s=.5))

    def test_sensor_launch_starts_gate_without_making_it_required(self):
        nodes = {n.get('name'): n for n in ET.parse(ROOT/'config/aircraft/sensors.launch').findall('node')}
        gate = nodes['localization_gate']
        self.assertEqual(gate.get('type'), 'localization_gate_node.py')
        self.assertEqual(gate.get('required'), 'false')
        self.assertEqual(gate.find('param').get('value'), '$(arg flight_config)')

    def test_node_is_installed(self):
        self.assertIn('scripts/localization_gate_node.py',
                      (ROOT/'src/airseafly_real400_mission/CMakeLists.txt').read_text())


if __name__ == '__main__':
    unittest.main()
