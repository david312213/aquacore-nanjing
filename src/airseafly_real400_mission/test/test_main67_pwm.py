"""MAIN6/7 bench wiring: physical ports are distinct from actuator channels."""
from copy import deepcopy
import json
import os
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import test_main1_pwm as helpers
from airseafly_real400_mission.main_servo_test import (
    inspect_mapping, main_pwm_request, run_port_pwm_menu, send_once,
)


class Main67PWMTests(unittest.TestCase):
    def setUp(self):
        self.h = helpers.Main1PWMTests()
        self.h.setUp()
        self.h.parameters = dict(PWM_MAIN_FUNC1=0, PWM_MAIN_FUNC6=301,
            PWM_MAIN_FUNC7=302, PWM_MAIN_MIN6=900, PWM_MAIN_MAX6=2000,
            PWM_MAIN_MIN7=1000, PWM_MAIN_MAX7=2200, PWM_MAIN_REV=0,
            PWM_AUX_FUNC1=101, PWM_AUX_FUNC2=102, COM_PREARM_MODE=2)
        self.cli = self.h.load_cli()

    def test_per_port_endpoints_midpoints_and_physical_reverse_bits(self):
        before = deepcopy(self.h.payload)
        for slot, low, high in ((6, 900, 2000), (7, 1000, 2200)):
            for reverse in (0, 1, 1 << 5, 1 << 6, (1 << 5) | (1 << 6)):
                params = dict(self.h.parameters, PWM_MAIN_REV=reverse)
                for pwm in (low, (low+high)/2, 1500, min(high, 2000)):
                    with self.subTest(slot=slot, reverse=reverse, pwm=pwm):
                        payload = self.cli.port_payload(self.h.payload, slot)
                        command = main_pwm_request(slot, pwm, payload, params)
                        self.h.assert_only_channel(command, slot-5)
                        physical = command['param'+str(slot-5)]
                        if reverse & (1 << (slot-1)):
                            physical *= -1
                        self.assertAlmostEqual(low+(physical+1)*(high-low)/2, pwm)
        self.assertEqual(self.h.payload, before)

    def test_mapping_checks_only_selected_port_and_shared_function(self):
        payload = self.cli.port_payload(self.h.payload, 6)
        params = dict(self.h.parameters, PWM_MAIN_FUNC7=0)
        self.assertEqual(inspect_mapping(params, payload, slots=(6,))[1], [])
        for update in ({'PWM_MAIN_FUNC6': 0}, {'PWM_MAIN_FUNC1': 301},
                       {'PWM_AUX_FUNC4': 301}, {'PWM_MAIN_FUNC7': 301}):
            changed = dict(params, **update)
            transport = mock.Mock()
            with self.assertRaises(ValueError):
                send_once(main_pwm_request(6, 1500, payload, changed), self.h.state,
                          changed, payload, transport, slots=(6,))
            transport.assert_not_called()

    def test_dual_cli_routes_once_and_continues_after_invalid_input_and_failed_ack(self):
        code, transport, events, output, _ = self.h.run_mocked_cli(['--main67'],
            inputs=['', '1500', '8 1500', '6 nan', '6 899', '6 1500', '7 1600', 'q'],
            replies=[SimpleNamespace(success=False, result=2), SimpleNamespace(success=True, result=0)])
        self.assertEqual(code, 0)
        self.assertEqual(transport.call_count, 2)
        for call, channel in zip(transport.call_args_list, (1, 2)):
            self.h.assert_only_channel(call.kwargs, channel)
        self.assertEqual([e['event'] for e in events], ['error', 'request', 'ack', 'request', 'ack'])
        self.assertEqual([e['slot'] for e in events if e['event'] == 'ack'], [6, 7])
        self.assertIn('机构实际位置', output)

    def test_bad_other_port_does_not_block_valid_port(self):
        self.h.parameters['PWM_MAIN_FUNC7'] = 0
        code, transport, events, _, _ = self.h.run_mocked_cli(['--main67'],
            inputs=['7 1500', '6 1500', 'q'])
        self.assertEqual(code, 0)
        transport.assert_called_once()
        self.h.assert_only_channel(transport.call_args.kwargs, 1)
        self.assertEqual([e['slot'] for e in events if e['event'] == 'ack'], [6])

    def test_fixed_port_single_command_and_interactive(self):
        for argv, inputs in ((['--main67', '--port', '7', '1600'], ()),
                             (['--main67', '--port', '7'], ['1600', 'q'])):
            code, transport, events, _, _ = self.h.run_mocked_cli(argv, inputs=inputs)
            self.assertEqual(code, 0)
            transport.assert_called_once()
            self.h.assert_only_channel(transport.call_args.kwargs, 2)
            self.assertAlmostEqual(transport.call_args.kwargs['param2'], 0.)
            self.assertEqual([e['event'] for e in events], ['request', 'ack'])

    def test_startup_exit_does_not_send(self):
        for ending in ('q', EOFError(), KeyboardInterrupt()):
            code, transport, events, _, _ = self.h.run_mocked_cli(['--main67'], inputs=[ending])
            self.assertEqual(code, 0)
            transport.assert_not_called()
            self.assertEqual(events, [])

    def test_transport_exception_and_interrupt_append_no_reset_or_retry(self):
        send = mock.Mock(side_effect=[RuntimeError('lost ACK'), True])
        run_port_pwm_menu(send, read=mock.Mock(side_effect=['6 1500', '7 1500', EOFError()]), emit=lambda _: None)
        self.assertEqual(send.call_args_list, [mock.call(6, 1500.), mock.call(7, 1500.)])
        send = mock.Mock(side_effect=KeyboardInterrupt())
        run_port_pwm_menu(send, read=mock.Mock(return_value='7 1500'), emit=lambda _: None)
        send.assert_called_once_with(7, 1500.)

    def test_shell_offline_preview_has_no_ros_and_targets_correct_channels(self):
        env = dict(os.environ, AIRSEAFLY_PYTHON=sys.executable, PYTHONDONTWRITEBYTECODE='1')
        with tempfile.TemporaryDirectory(prefix='main67 pwm ') as directory:
            for args, channel in (([], None), (['--port', '6', '900'], 1), (['--port', '7', '2000'], 2)):
                result = subprocess.run(['sh', str(helpers.PROJECT/'tools/payload/main67_pwm.sh'), '--dry-run', *args],
                    cwd=directory, env=env, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                value = json.loads(result.stdout[result.stdout.index('{'):])
                self.assertFalse(value['connected'])
                self.assertIn('2026-10-25 读回值', result.stdout)
                if channel is None:
                    self.assertEqual(value['commands'], [])
                    self.assertEqual([row['output'] for row in value['outputs']], ['MAIN6', 'MAIN7'])
                else:
                    self.h.assert_only_channel(value['commands'][0], channel)


if __name__ == '__main__':
    unittest.main()
