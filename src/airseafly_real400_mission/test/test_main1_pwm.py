"""MAIN1 raw PWM calibration: real CLI parsing with mocked ROS/actuators."""
from contextlib import redirect_stdout, redirect_stderr
from copy import deepcopy
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

PROJECT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT/'src/airseafly_real400_mission/src'))
from airseafly_real400_mission.main_servo_test import (
    inspect_mapping, main1_payload, main1_pwm_request, run_pwm_menu, run_port_pwm_menu, send_once,
)


class Main1PWMTests(unittest.TestCase):
    def setUp(self):
        self.payload = dict(actuator_indices={'1': 1, '2': 2, '3': 3, '4': 4},
            pwm_min_us=1100, pwm_center_us=1500, pwm_max_us=2200,
            locked_pwm_us=2200, released_pwm_us=1100)
        # Unrelated MAIN outputs deliberately do not match mission config.
        self.parameters = dict(PWM_MAIN_FUNC1=301, PWM_MAIN_MIN1=1000,
            PWM_MAIN_MAX1=2400, PWM_MAIN_REV=0, PWM_MAIN_FUNC2=101,
            PWM_MAIN_FUNC3=0, PWM_MAIN_FUNC4=0, COM_MOT_TEST_EN=0)
        self.state = SimpleNamespace(connected=True, armed=False)

    def assert_only_channel(self, fields, channel):
        self.assertEqual([i for i in range(1, 7) if math.isfinite(fields['param'+str(i)])], [channel])
        self.assertEqual(fields['command'], 187)
        self.assertEqual(fields['param7'], 0.)
        self.assertFalse(fields['broadcast'])
        self.assertEqual(fields['confirmation'], 0)

    def test_physical_limits_endpoints_middle_and_reversal(self):
        for reverse in (0, 1, 2):
            params = dict(self.parameters, PWM_MAIN_REV=reverse)
            for pwm in (1000, 1500, 1700, 2250, 2400):
                with self.subTest(reverse=reverse, pwm=pwm):
                    command = main1_pwm_request(pwm, self.payload, params)
                    physical = command['param1'] * (-1 if reverse & 1 else 1)
                    self.assertAlmostEqual(1000+(physical+1)*700, pwm)
                    self.assertEqual(command['configured_pwm_us'], pwm)
                    self.assertTrue(command['persistent'])
                    self.assert_only_channel(command, 1)

    def test_configured_function_remap_targets_main1_only(self):
        payload = deepcopy(self.payload)
        payload['actuator_indices']['1'] = 6
        params = dict(self.parameters, PWM_MAIN_FUNC1=306, PWM_MAIN_REV=1)
        command = main1_pwm_request(1800, payload, params)
        transport = mock.Mock()
        send_once(command, self.state, params, main1_payload(payload), transport)
        transport.assert_called_once()
        self.assert_only_channel(transport.call_args.kwargs, 6)
        self.assertAlmostEqual(1000+(-command['param6']+1)*700, 1800)

    def test_main1_scope_leaves_config_unchanged_and_ignores_unrelated_outputs(self):
        before = deepcopy(self.payload)
        params_before = deepcopy(self.parameters)
        scoped = main1_payload(self.payload)
        rows, errors = inspect_mapping(self.parameters, scoped)
        self.assertEqual(errors, [])
        self.assertEqual([row['slot'] for row in rows], [1])
        send_once(main1_pwm_request(1500, scoped, self.parameters), self.state,
                  self.parameters, scoped, mock.Mock())
        self.assertEqual(self.payload, before)
        self.assertEqual(self.parameters, params_before)

    def test_main1_wrong_or_shared_function_never_sends(self):
        for update in ({'PWM_MAIN_FUNC1': 101}, {'PWM_MAIN_FUNC1': 0},
                       {'PWM_MAIN_FUNC2': 301}, {'PWM_AUX_FUNC4': 301}):
            params = dict(self.parameters, **update)
            transport = mock.Mock()
            with self.assertRaises(ValueError):
                send_once(main1_pwm_request(1500, self.payload, params), self.state,
                          params, main1_payload(self.payload), transport)
            transport.assert_not_called()

    def test_existing_connected_and_disarmed_checks_remain(self):
        for state in (SimpleNamespace(connected=False, armed=False),
                      SimpleNamespace(connected=True, armed=True)):
            transport = mock.Mock()
            with self.assertRaises(ValueError):
                send_once(main1_pwm_request(1500, self.payload, self.parameters), state,
                          self.parameters, main1_payload(self.payload), transport)
            transport.assert_not_called()

    def test_invalid_values_and_channel_are_rejected(self):
        for value in (999, 2401, float('nan'), float('inf'), -float('inf')):
            with self.assertRaises(ValueError):
                main1_pwm_request(value, self.payload, self.parameters)
        for channel in (0, 7):
            with self.assertRaises(ValueError):
                main1_pwm_request(1500, dict(self.payload, actuator_indices={'1': channel}))
        with self.assertRaises(ValueError):
            main1_payload(dict(self.payload, actuator_indices={'2': 2}))

    def test_live_scale_does_not_depend_on_mission_center_or_saved_range(self):
        params = dict(self.parameters, PWM_MAIN_MIN1=1600, PWM_MAIN_MAX1=2600)
        for pwm in (1600, 2100, 2600):
            command = main1_pwm_request(pwm, self.payload, params)
            self.assertAlmostEqual(1600+(command['param1']+1)*500, pwm)

    def test_offline_scale_uses_configured_main1_override(self):
        payload = dict(self.payload, pwm_max_us_by_slot={'1': 2020, '2': 1800})
        command = main1_pwm_request(2020, payload)
        self.assertEqual(command['param1'], 1.)
        self.assert_only_channel(command, 1)

    def test_interactive_input_does_not_send_on_start_or_exit(self):
        send = mock.Mock(return_value=True)
        run_pwm_menu(send, read=mock.Mock(side_effect=['q']), emit=lambda _: None)
        send.assert_not_called()
        run_pwm_menu(send, read=mock.Mock(side_effect=['', 'abc', 'nan', 'inf', '1500', '1550.5', 'q']),
                     emit=lambda _: None)
        self.assertEqual(send.call_args_list, [mock.call(1500.), mock.call(1550.5)])

    def test_menu_continues_after_rejection_invalid_range_and_transport_exception(self):
        send = mock.Mock(side_effect=[False, ValueError('范围错误'), RuntimeError('ACK lost'), True])
        messages = []
        run_pwm_menu(send, read=mock.Mock(side_effect=['1500', '999', '1600', '1700', 'q']),
                     emit=messages.append)
        self.assertEqual(send.call_args_list, [mock.call(x) for x in (1500., 999., 1600., 1700.)])
        self.assertIn('范围错误', '\n'.join(messages))
        self.assertIn('ACK lost', '\n'.join(messages))

    def test_menu_eof_and_interrupt_do_not_append_a_command(self):
        for error in (EOFError(), KeyboardInterrupt()):
            send = mock.Mock(return_value=True)
            run_pwm_menu(send, read=mock.Mock(side_effect=['1500', error]), emit=lambda _: None)
            send.assert_called_once_with(1500.)
        send = mock.Mock(side_effect=KeyboardInterrupt())
        run_pwm_menu(send, read=mock.Mock(return_value='1500'), emit=lambda _: None)
        send.assert_called_once_with(1500.)

    def load_cli(self):
        spec = importlib.util.spec_from_file_location('main1_pwm_cli', PROJECT/'tools/payload/main1_pwm.py')
        cli = importlib.util.module_from_spec(spec)
        with mock.patch.object(sys, 'path', [str(PROJECT/'tools/payload'), *sys.path]):
            spec.loader.exec_module(cli)
        return cli

    def run_mocked_cli(self, argv, inputs=(), replies=None, parameters=None):
        cli = self.load_cli()
        output = io.StringIO()
        transport = mock.Mock(side_effect=replies,
                              return_value=SimpleNamespace(success=True, result=0))
        services = {'/mavros/cmd/command': transport,
                    '/mavros/param/pull': mock.Mock(return_value=SimpleNamespace(success=True))}
        get_param = mock.Mock(side_effect=parameters) if parameters else mock.Mock(return_value=self.parameters)
        ros = SimpleNamespace(init_node=mock.Mock(), wait_for_message=mock.Mock(return_value=self.state),
            wait_for_service=mock.Mock(), get_param=get_param,
            ServiceProxy=mock.Mock(side_effect=lambda name, _type: services[name]))
        modules = {'rospy': ros, 'mavros_msgs': SimpleNamespace(),
                   'mavros_msgs.msg': SimpleNamespace(State=object),
                   'mavros_msgs.srv': SimpleNamespace(CommandLong=object, ParamPull=object)}
        config = dict(payload=self.payload, _config_path='machine.yaml',
                      _flight_configuration=dict(path='flight.yaml'))
        with tempfile.TemporaryDirectory() as temp, \
             mock.patch.object(cli, 'PROJECT', Path(temp)), \
             mock.patch.object(cli, 'base_configuration', return_value=config), \
             mock.patch.object(cli, 'apply_flight_configuration', return_value=config), \
             mock.patch.object(cli, 'Connection') as connection, \
             mock.patch.object(cli.time, 'sleep') as sleep, \
             mock.patch.object(cli, 'run_pwm_menu', side_effect=lambda send, **kwargs: run_pwm_menu(
                 send, read=mock.Mock(side_effect=inputs), **kwargs)), \
             mock.patch.object(cli, 'run_port_pwm_menu', side_effect=lambda send, **kwargs: run_port_pwm_menu(
                 send, read=mock.Mock(side_effect=inputs), **kwargs)), \
             mock.patch.dict(sys.modules, modules), \
             mock.patch.dict(os.environ, AIRSEAFLY_SERVO_ROS_READY='1'), \
             redirect_stdout(output):
            code = cli.main(argv)
            events = [json.loads(line) for path in Path(temp).glob('data/*/*/actions.jsonl')
                      for line in path.read_text().splitlines()]
            sleep.assert_not_called()
            connection.return_value.__exit__.assert_called_once()
            selected_url = connection.call_args.args[1]
        return code, transport, events, output.getvalue(), selected_url

    def test_live_cli_single_value_uses_actual_range_fcu_url_and_no_reset(self):
        code, transport, events, output, selected_url = self.run_mocked_cli(
            ['2250', '--fcu-url', 'udp://:14540@127.0.0.1:14557'])
        self.assertEqual(code, 0)
        self.assertEqual(selected_url, 'udp://:14540@127.0.0.1:14557')
        transport.assert_called_once()
        self.assert_only_channel(transport.call_args.kwargs, 1)
        self.assertAlmostEqual(1000+(transport.call_args.kwargs['param1']+1)*700, 2250)
        self.assertEqual([e['event'] for e in events], ['request', 'ack'])
        self.assertTrue(events[-1]['success'])
        self.assertIn('机构实际位置', output)

    def test_live_cli_interactive_refreshes_range_reversal_and_logs_ack_errors(self):
        changed = dict(self.parameters, PWM_MAIN_MAX1=2200, PWM_MAIN_REV=1)
        code, transport, events, output, _ = self.run_mocked_cli([], inputs=['1500', '1600', '1700', 'q'],
            replies=[SimpleNamespace(success=False, result=2), RuntimeError('lost ACK'),
                     SimpleNamespace(success=True, result=0)],
            parameters=[self.parameters, self.parameters, self.parameters, changed])
        self.assertEqual(code, 0)
        self.assertEqual(transport.call_count, 3)
        self.assertAlmostEqual(1000+(-transport.call_args.kwargs['param1']+1)*600, 1700)
        self.assertEqual([e['event'] for e in events], ['request', 'ack', 'request', 'error', 'request', 'ack'])
        self.assertIn('lost ACK', output)
        self.assertIn('未自动重发', output)

    def test_live_cli_out_of_range_is_logged_and_menu_continues(self):
        code, transport, events, _, _ = self.run_mocked_cli([], inputs=['999', '1500', 'q'])
        self.assertEqual(code, 0)
        transport.assert_called_once()
        self.assertEqual([e['event'] for e in events], ['error', 'request', 'ack'])

    def test_live_cli_exit_without_input_has_no_actuator_requests(self):
        code, transport, events, _, _ = self.run_mocked_cli([], inputs=['q'])
        self.assertEqual(code, 0)
        transport.assert_not_called()
        self.assertEqual(events, [])

    def test_live_cli_rejected_single_command_returns_nonzero(self):
        code, transport, _, _, _ = self.run_mocked_cli(['1500'],
            replies=[SimpleNamespace(success=False, result=2)])
        self.assertEqual(code, 2)
        transport.assert_called_once()

    def test_dry_run_and_invalid_cli_never_create_connection(self):
        cli = self.load_cli()
        for argv in (['1500', '--dry-run'], ['--dry-run']):
            with mock.patch.object(cli, 'Connection') as connection, \
                 mock.patch.object(cli.os, 'execve') as execve, redirect_stdout(io.StringIO()) as output:
                self.assertEqual(cli.main(argv), 0)
                value = json.loads(output.getvalue()[output.getvalue().index('{'):])
                self.assertFalse(value['connected'])
                self.assertEqual(len(value['commands']), 0 if len(argv) == 1 else 1)
                connection.assert_not_called()
                execve.assert_not_called()
        for value in ('abc', 'nan', 'inf'):
            with mock.patch.object(cli, 'Connection') as connection, redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as exc:
                    cli.main([value])
                self.assertEqual(exc.exception.code, 2)
                connection.assert_not_called()

    def test_shell_entry_dry_run_from_another_working_directory(self):
        env = dict(os.environ, AIRSEAFLY_PYTHON=sys.executable, PYTHONDONTWRITEBYTECODE='1')
        with tempfile.TemporaryDirectory(prefix='main1 pwm ') as directory:
            for pwm in ('900', '1500', '2000'):
                result = subprocess.run(['sh', str(PROJECT/'tools/payload/main1_pwm.sh'), pwm, '--dry-run'],
                    cwd=directory, env=env, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                preview = json.loads(result.stdout[result.stdout.index('{'):])
                self.assertFalse(preview['connected'])
                self.assertEqual(len(preview['commands']), 1)
                self.assert_only_channel(preview['commands'][0], 1)


if __name__ == '__main__':
    unittest.main()
