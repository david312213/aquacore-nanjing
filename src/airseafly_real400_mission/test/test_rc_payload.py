"""RC9 frames through the real callback/worker, with all aircraft IO mocked."""
from copy import deepcopy
import importlib.util
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src/airseafly_real400_mission/src'))
from airseafly_real400_mission.flight_configuration import apply_flight_configuration, base_configuration
from airseafly_real400_mission.payload import actuator_command
from airseafly_real400_mission.rc_payload import paired_command, switch_position

SCRIPT = ROOT/'src/airseafly_real400_mission/scripts/rc_payload_control.py'
spec = importlib.util.spec_from_file_location('rc_payload_node', SCRIPT)
node_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(node_module)


def rc(pwm):
    # Channels 1..8 deliberately have different values to catch indexing errors.
    return SimpleNamespace(channels=[2000, 1000, 1500, 2000, 1000, 1500, 2000, 1000, pwm])


class RCPayloadTests(unittest.TestCase):
    def setUp(self):
        self.config = apply_flight_configuration(base_configuration(), task='delivery-route')
        self.payload, self.settings = self.config['payload'], self.config['rc_payload']

    def test_three_positions_and_boundaries_use_channel_nine(self):
        for pwm, expected in ((1000, 'low'), (1300, 'low'), (1301, 'middle'),
                              (1500, 'middle'), (1699, 'middle'), (1700, 'high'), (2000, 'high')):
            with self.subTest(pwm=pwm):
                self.assertEqual(switch_position(rc(pwm).channels, self.settings), expected)
        for channels in ([], [1000]*8, rc(0).channels, rc(65535).channels):
            self.assertIsNone(switch_position(channels, self.settings))

    def test_open_and_close_change_both_outputs_in_one_command(self):
        before = deepcopy(self.payload)
        for position, pwm in (('low', 1350), ('middle', 900)):
            command = paired_command(position, self.payload, self.settings)
            self.assertEqual(command['command'], 187)
            self.assertEqual(command['pwm_us'], pwm)
            self.assertEqual([i for i, value in enumerate(command['params'], 1)
                              if math.isfinite(value)], [1, 2])
            for value in command['params'][:2]:
                self.assertAlmostEqual(900+(value+1)*550, pwm)
        self.assertIsNone(paired_command('high', self.payload, self.settings))
        self.assertEqual(self.payload, before)
        self.assertEqual([actuator_command(slot, 'release', self.payload)['pwm_us']
                          for slot in range(1, 5)], [1190, 1190, 1350, 1350])

    def test_physical_output_calibration_and_reverse_are_preserved(self):
        parameters = dict(PWM_MAIN_MIN6=800, PWM_MAIN_MAX6=1950,
                          PWM_MAIN_MIN7=850, PWM_MAIN_MAX7=2000, PWM_MAIN_REV=1 << 5)
        for position, pwm in (('low', 1350), ('middle', 900)):
            command = paired_command(position, self.payload, self.settings, parameters)
            for index, output in enumerate((6, 7)):
                value = command['params'][index] * (-1 if output == 6 else 1)
                low, high = (parameters['PWM_MAIN_'+key+str(output)] for key in ('MIN', 'MAX'))
                self.assertAlmostEqual(low+(value+1)*(high-low)/2, pwm)

    def make_node(self, transport=None, parameters=None):
        self.receipt = threading.Event()
        self.logged = []

        def log(*args):
            self.logged.append(args)
            if '回执' in args[0] or '未确认' in args[0]:
                self.receipt.set()

        self.transport = transport or mock.Mock(return_value=SimpleNamespace(success=True, result=0))
        self.ros = SimpleNamespace(ServiceProxy=mock.Mock(return_value=self.transport),
            Subscriber=mock.Mock(), on_shutdown=mock.Mock(),
            get_param=parameters or mock.Mock(return_value={}), loginfo=log, logerr=log)
        node = node_module.RCPayloadControl(self.ros, object, object, self.config)

        def cleanup():
            node.close()
            node.worker.join(2)
            self.assertFalse(node.worker.is_alive())

        self.addCleanup(cleanup)
        return node

    def send(self, node, pwm):
        self.receipt.clear()
        node.on_rc(rc(pwm))
        self.assertTrue(self.receipt.wait(2), 'worker did not report its mocked receipt')

    def assert_pwm(self, request, pwm):
        self.assertEqual(request['command'], 187)
        self.assertFalse(request['broadcast'])
        self.assertEqual(request['confirmation'], 0)
        self.assertEqual(request['param7'], 0.)
        for name in ('param1', 'param2'):
            self.assertAlmostEqual(900+(request[name]+1)*550, pwm)
        self.assertTrue(all(math.isnan(request['param%d' % i]) for i in range(3, 7)))

    def test_first_frame_then_edges_only_do_not_override_automatic_release(self):
        node = self.make_node()
        self.transport.assert_not_called()
        node.on_rc(rc(2000))
        self.transport.assert_not_called()
        self.send(node, 1500)
        for _ in range(100):
            node.on_rc(rc(1490))
        self.assertEqual(self.transport.call_count, 1)
        self.assertIsNone(node.pending)
        self.send(node, 1000)
        node.on_rc(rc(2000))
        self.assertIsNone(node.pending)
        self.assertEqual(self.transport.call_count, 2)
        self.send(node, 1000)
        self.send(node, 1500)
        self.assertEqual(self.transport.call_count, 4)
        for call, pwm in zip(self.transport.call_args_list, (900, 1350, 1350, 900)):
            self.assert_pwm(call.kwargs, pwm)
        self.ros.Subscriber.assert_called_once_with('/mavros/rc/in', object, node.on_rc, queue_size=1)
        node.close()
        node.on_rc(rc(1000))
        self.assertEqual(self.transport.call_count, 4)

    def test_first_low_frame_opens_without_mode_or_arm_state(self):
        node = self.make_node()
        self.send(node, 1000)
        self.assert_pwm(self.transport.call_args.kwargs, 1350)
        self.ros.get_param.assert_called_once_with('/mavros/param', {})
        self.assertEqual(self.ros.Subscriber.call_count, 1)

    def test_missing_channels_do_not_move_servos_or_retrigger_same_position(self):
        node = self.make_node()
        for channels in ([], [1000]*8, rc(0).channels, rc(65535).channels):
            node.on_rc(SimpleNamespace(channels=channels))
        self.transport.assert_not_called()
        self.send(node, 1500)
        node.on_rc(SimpleNamespace(channels=[]))
        node.on_rc(rc(1500))
        self.assertIsNone(node.pending)
        self.assertEqual(self.transport.call_count, 1)

    def test_real_rejection_or_exception_does_not_generate_repeated_commands(self):
        transport = mock.Mock(side_effect=[SimpleNamespace(success=False, result=2),
                                          RuntimeError('no receipt'), SimpleNamespace(success=True, result=0)])
        node = self.make_node(transport)
        self.send(node, 1000)
        for _ in range(20):
            node.on_rc(rc(1000))
        self.assertEqual(transport.call_count, 1)
        self.send(node, 1500)
        node.on_rc(rc(1500))
        self.assertIsNone(node.pending)
        self.assertEqual(transport.call_count, 2)
        self.send(node, 1000)
        self.assertEqual(transport.call_count, 3)
        self.assertTrue(any('未确认' in row[0] for row in self.logged))

    def test_high_cancels_unsent_action_while_parameters_are_being_read(self):
        entered, resume = threading.Event(), threading.Event()

        def parameters(*_args):
            entered.set()
            self.assertTrue(resume.wait(2))
            return {}

        node = self.make_node(parameters=mock.Mock(side_effect=parameters))
        self.addCleanup(resume.set)
        node.on_rc(rc(1000))
        self.assertTrue(entered.wait(2))
        node.on_rc(rc(2000))
        self.assertIsNone(node.pending)
        resume.set()
        # A later middle command confirms the older low command was discarded.
        self.send(node, 1500)
        self.assertEqual(self.transport.call_count, 1)
        self.assert_pwm(self.transport.call_args.kwargs, 900)

    def test_slow_receipt_does_not_block_rc_or_replay_obsolete_switch_positions(self):
        entered, resume = threading.Event(), threading.Event()

        def transport(**_fields):
            entered.set()
            self.assertTrue(resume.wait(2))
            return SimpleNamespace(success=True, result=0)

        node = self.make_node(mock.Mock(side_effect=transport))
        self.addCleanup(resume.set)
        node.on_rc(rc(1500))
        self.assertTrue(entered.wait(2))
        node.on_rc(rc(1000))
        self.assertEqual(node.pending[1], 'low')
        node.on_rc(rc(2000))
        self.assertIsNone(node.pending)
        resume.set()
        self.assertTrue(self.receipt.wait(2))
        self.assertEqual(self.transport.call_count, 1)
        self.assert_pwm(self.transport.call_args.kwargs, 900)

    def test_sensor_launch_wires_node_and_install_entry(self):
        root = ET.parse(ROOT/'config/aircraft/sensors.launch').getroot()
        node = root.find("node[@name='rc_payload_control']")
        self.assertEqual(node.get('pkg'), 'airseafly_real400_mission')
        self.assertEqual(node.get('type'), SCRIPT.name)
        self.assertEqual(node.get('required'), 'false')
        self.assertEqual(node.find("param[@name='flight_config']").get('value'), '$(arg flight_config)')
        self.assertIn('scripts/'+SCRIPT.name,
                      (ROOT/'src/airseafly_real400_mission/CMakeLists.txt').read_text())


if __name__ == '__main__':
    unittest.main()
