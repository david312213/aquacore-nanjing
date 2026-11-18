"""Actuator-test mapping and exactly-once command behavior, without hardware."""
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

PROJECT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT/'src/airseafly_real400_mission/src'))
from airseafly_real400_mission.main_servo_test import inspect_mapping, run_menu, send_once, actuator_request


class MainServoTests(unittest.TestCase):
    def setUp(self):
        self.payload = dict(actuator_indices={'1': 1, '2': 2, '3': 3},
            pwm_min_us=1100, pwm_center_us=1500, pwm_max_us=2200,
            locked_pwm_us=1260, released_pwm_us=2200)
        self.params = {'COM_MOT_TEST_EN': 1}
        for slot in (1, 2, 3):
            self.params.update({'PWM_MAIN_FUNC'+str(slot): 300+slot,
                                'PWM_MAIN_MIN'+str(slot): 1100,
                                'PWM_MAIN_MAX'+str(slot): 2200})
        for slot in (5, 6, 7, 8):
            self.params['PWM_MAIN_FUNC'+str(slot)] = 101+slot-5
        self.state = SimpleNamespace(connected=True, armed=False)

    def test_ground_test_addressing_preserves_flight_values(self):
        for slot in (1, 2, 3):
            for action, value in (('lock', -.6), ('release', 1.)):
                command = actuator_request(slot, action, self.payload)
                transport = mock.Mock()
                send_once(command, self.state, self.params, self.payload, transport)
                transport.assert_called_once()
                fields = transport.call_args.kwargs
                self.assertEqual(fields['command'], 310)
                self.assertEqual(fields['param5'], 1300+slot)
                self.assertEqual(fields['param1'], value)
                self.assertEqual(fields['param2'], 2.)
                self.assertEqual(fields['param3'], 0.)
                self.assertFalse(fields['broadcast'])

    def test_main_remap_is_required_and_duplicate_aux_is_rejected(self):
        for updates in ({'PWM_MAIN_FUNC1': 101}, {'PWM_MAIN_FUNC1': 0},
                        {'PWM_AUX_FUNC2': 302},
                        {'COM_MOT_TEST_EN': 0}):
            parameters = dict(self.params, **updates)
            transport = mock.Mock()
            with self.assertRaises(ValueError):
                send_once(actuator_request(2, 'lock', self.payload), self.state,
                          parameters, self.payload, transport)
            transport.assert_not_called()

    def test_different_main_ranges_allow_menu_and_use_physical_slot_override(self):
        payload = dict(deepcopy(self.payload), released_pwm_us_by_slot={'2': 2020})
        parameters = dict(self.params, PWM_MAIN_MAX2=2020, PWM_MAIN_MAX3=1900)
        before_payload, before_parameters = deepcopy(payload), deepcopy(parameters)
        rows, errors = inspect_mapping(parameters, payload)
        self.assertEqual(errors, [])
        self.assertEqual([row['maximum_us'] for row in rows], [2200, 2020, 1900])
        command = actuator_request(2, 'release', payload, parameters=parameters)
        transport = mock.Mock()
        send_once(command, self.state, parameters, payload, transport)
        transport.assert_called_once()
        fields = transport.call_args.kwargs
        self.assertEqual(fields['param5'], 1302.)
        self.assertEqual(fields['param1'], 1.)
        self.assertEqual(1100+(fields['param1']+1)*460, 2020)
        self.assertEqual(actuator_request(2, 'lock', payload, parameters=parameters)['param1'], -.6)
        self.assertEqual(actuator_request(1, 'release', payload, parameters=parameters)['param1'], 1.)
        self.assertEqual(payload, before_payload)
        self.assertEqual(parameters, before_parameters)

    def test_physical_slot_range_is_used_after_function_remap(self):
        payload = dict(deepcopy(self.payload), released_pwm_us_by_slot={2: 2020})
        payload['actuator_indices']['2'] = 5
        parameters = dict(self.params, PWM_MAIN_FUNC2=305, PWM_MAIN_MAX2=2020)
        command = actuator_request(2, 'release', payload, parameters=parameters)
        transport = mock.Mock()
        send_once(command, self.state, parameters, payload, transport)
        self.assertEqual(transport.call_args.kwargs['param5'], 1305.)
        self.assertEqual(transport.call_args.kwargs['param1'], 1.)

    def test_armed_or_disconnected_never_sends(self):
        for state in (SimpleNamespace(connected=True, armed=True),
                      SimpleNamespace(connected=False, armed=False)):
            transport = mock.Mock()
            with self.assertRaises(ValueError):
                send_once(actuator_request(1, 'release', self.payload), state,
                          self.params, self.payload, transport)
            transport.assert_not_called()

    def test_uncertain_ack_not_retried_and_no_exit_command(self):
        transport = mock.Mock(side_effect=TimeoutError('lost ACK'))
        command = actuator_request(3, 'release', self.payload)
        def send(slot, action):
            send_once(command, self.state, self.params, self.payload, transport)
        lines = []
        run_menu(send, read=mock.Mock(side_effect=['3', '1', 'q']), emit=lines.append)
        transport.assert_called_once()
        self.assertIn('未自动重发', '\n'.join(lines))

    def test_menu_needs_explicit_action_and_no_automatic_close(self):
        send = mock.Mock()
        run_menu(send, read=mock.Mock(side_effect=['x', '1', 'b', '2', '2', 'q']), emit=lambda x: None)
        send.assert_called_once_with(2, 'lock')

    def test_invalid_duration_slot_action_never_construct_command(self):
        for seconds in (0, -1, 3.1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                actuator_request(1, 'release', self.payload, seconds)
        with self.assertRaises(ValueError):
            actuator_request(4, 'release', self.payload)

    def test_dry_run_has_no_ros_and_incomplete_command_is_error(self):
        env = dict(__import__('os').environ, AIRSEAFLY_PYTHON=sys.executable)
        result = subprocess.run(['sh', str(PROJECT/'tools/payload/5_servo_test.sh'), 'release', '3', '--dry-run'],
                                env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        preview = json.loads(result.stdout[result.stdout.index('{'):])
        self.assertEqual(preview['commands'][0]['main_output'], 6)
        self.assertEqual(preview['commands'][0]['configured_pwm_us'], 1350)
        self.assertEqual(preview['commands'][0]['command'], 187)
        self.assertIn('"connected": false', result.stdout)
        result = subprocess.run(['sh', str(PROJECT/'tools/payload/5_servo_test.sh'), 'release'], env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        self.assertEqual(result.returncode, 2)


if __name__ == '__main__':
    unittest.main()
