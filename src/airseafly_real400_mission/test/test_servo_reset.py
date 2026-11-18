"""Three-servo centering through the existing ground interface, without IO."""
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
from airseafly_real400_mission.main_servo_test import actuator_request, run_reset, send_once


class ServoResetTests(unittest.TestCase):
    def setUp(self):
        self.payload = dict(actuator_indices={'1': 1, '2': 2, '3': 3},
                            pwm_min_us=1100, pwm_center_us=1500, pwm_max_us=2200,
                            locked_pwm_us=1260, released_pwm_us=2200)
        self.params = {'COM_MOT_TEST_EN': 1, 'PWM_MAIN_REV': 0}
        for slot in (1, 2, 3):
            self.params.update({'PWM_MAIN_FUNC'+str(slot): 300+slot,
                                'PWM_MAIN_MIN'+str(slot): 1100,
                                'PWM_MAIN_MAX'+str(slot): 2200})
        self.state = SimpleNamespace(connected=True, armed=False)

    def test_center_is_actual_configured_pwm_not_zero_or_lock(self):
        for slot in (1, 2, 3):
            command = actuator_request(slot, 'center', self.payload)
            self.assertEqual(command['configured_pwm_us'], 1500)
            self.assertNotEqual(command['normalized'], 0.)
            self.assertNotEqual(command['normalized'], -.6)
            self.assertAlmostEqual(command['normalized']*550+1650, 1500)
            transport = mock.Mock()
            send_once(command, self.state, self.params, self.payload, transport)
            transport.assert_called_once()
            fields = transport.call_args.kwargs
            self.assertEqual(fields['command'], 187)
            self.assertFalse(fields['broadcast'])
            self.assertEqual(fields['confirmation'], 0)
            self.assertEqual(fields['param7'], 0.)
            self.assertEqual(fields['param'+str(slot)], command['normalized'])
            self.assertEqual([i for i in range(1, 7) if math.isfinite(fields['param'+str(i)])], [slot])
            self.assertTrue(command['persistent'])

    def test_changed_center_mapping_and_reversal_use_physical_output(self):
        payload = deepcopy(self.payload)
        payload['pwm_center_us'] = 1540
        payload['actuator_indices']['3'] = 6
        params = dict(self.params, PWM_MAIN_FUNC3=306, PWM_MAIN_REV=4)
        command = actuator_request(3, 'center', payload, 1.5, params)
        self.assertEqual(command['channel'], 6)
        self.assertTrue(math.isnan(command['param2']))
        self.assertAlmostEqual(-command['param6']*550+1650, 1540)
        transport = mock.Mock()
        send_once(command, self.state, params, payload, transport)
        transport.assert_called_once()
        self.assertEqual(self.payload['pwm_center_us'], 1500)

    def test_center_has_no_test_duration_or_motor_test_enable_dependency(self):
        params = dict(self.params, COM_MOT_TEST_EN=0)
        for seconds in (0., 2., 10., float('nan')):
            command = actuator_request(3, 'center', self.payload, seconds, params)
            transport = mock.Mock()
            send_once(command, self.state, params, self.payload, transport)
            self.assertEqual(transport.call_args.kwargs['command'], 187)
            self.assertTrue(math.isnan(transport.call_args.kwargs['param2']))

    def test_center_uses_each_outputs_range_with_reversal(self):
        for reverse in (0, 2):
            params = dict(self.params, PWM_MAIN_MAX2=2020, PWM_MAIN_REV=reverse)
            command = actuator_request(2, 'center', self.payload, parameters=params)
            value = command['param2'] * (-1 if reverse else 1)
            self.assertAlmostEqual(1100+(value+1)*460, 1500)
            transport = mock.Mock()
            send_once(command, self.state, params, self.payload, transport)
            transport.assert_called_once()

    def test_each_invocation_sends_each_slot_once_in_order(self):
        send = mock.Mock(return_value=True)
        lines = []
        for _ in range(2):
            self.assertEqual(run_reset(send, lines.append), 0)
        self.assertEqual(send.call_args_list,
                         [mock.call(s, 'center') for s in (1, 2, 3, 1, 2, 3)])
        self.assertEqual(sum('均获飞控接受' in line for line in lines), 2)

    def test_rejection_or_exception_does_not_repeat_or_skip_other_slots(self):
        for result in (False, RuntimeError('no receipt'), TimeoutError('lost ACK')):
            send = mock.Mock(side_effect=[result, True, True])
            lines = []
            self.assertEqual(run_reset(send, lines.append), 2)
            self.assertEqual(send.call_args_list, [mock.call(s, 'center') for s in (1, 2, 3)])
            self.assertIn('回位未全部确认：1号口。', lines)
            self.assertNotIn('均获飞控接受', '\n'.join(lines))

    def test_interrupt_does_not_send_remaining_slots(self):
        send = mock.Mock(side_effect=[True, KeyboardInterrupt()])
        with self.assertRaises(KeyboardInterrupt):
            run_reset(send, lambda _: None)
        self.assertEqual(send.call_args_list, [mock.call(1, 'center'), mock.call(2, 'center')])

    def test_reset_shell_dry_run_closes_each_physical_servo_once_without_ros(self):
        env = dict(os.environ, AIRSEAFLY_PYTHON=sys.executable)
        result = subprocess.run(['sh', str(PROJECT/'tools/payload/servo_reset.sh'), '--dry-run', '--seconds', '1.5'],
                                env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        preview = json.loads(result.stdout[result.stdout.index('{'):])
        self.assertFalse(preview['connected'])
        commands = preview['commands']
        self.assertEqual([c['slot'] for c in commands], [1, 2])
        self.assertEqual([c['main_output'] for c in commands], [6, 7])
        self.assertEqual([c['action'] for c in commands], ['center']*2)
        self.assertEqual([c['configured_pwm_us'] for c in commands], [900]*2)
        self.assertEqual([c['command'] for c in commands], [187]*2)
        self.assertTrue(all(c['persistent'] for c in commands))
        self.assertIn('退出脚本不恢复旧位置', result.stdout)
        self.assertNotIn('秒后飞控恢复', result.stdout)

    def load_cli(self):
        spec = importlib.util.spec_from_file_location('servo_reset_cli', PROJECT/'tools/payload/servo_test.py')
        cli = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cli)
        return cli

    def test_invalid_reset_arguments_stop_before_connecting(self):
        cli = self.load_cli()
        for argv in (['reset', '2'], ['reset', '--check'], ['center']):
            with mock.patch.object(cli, 'Connection') as connect, redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    cli.main(argv)
                self.assertEqual(error.exception.code, 2)
                connect.assert_not_called()

    def test_full_cli_uses_current_mapping_and_sends_only_three_requests(self):
        cli = self.load_cli()
        params = dict(self.params, PWM_MAIN_REV=2, PWM_MAIN_MAX2=2020)
        transport = mock.Mock(return_value=SimpleNamespace(success=True, result=0))
        pull = mock.Mock(return_value=SimpleNamespace(success=True))
        services = {'/mavros/cmd/command': transport, '/mavros/param/pull': pull}
        ros = SimpleNamespace(init_node=mock.Mock(), wait_for_message=mock.Mock(return_value=self.state),
            wait_for_service=mock.Mock(), get_param=mock.Mock(return_value=params),
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
             mock.patch.object(cli, 'Connection'), \
             mock.patch.object(cli.time, 'sleep') as sleep, \
             mock.patch.dict(sys.modules, modules), \
             mock.patch.dict(os.environ, AIRSEAFLY_SERVO_ROS_READY='1'), \
             redirect_stdout(io.StringIO()):
            self.assertEqual(cli.main(['reset']), 0)
            events = [json.loads(line) for path in Path(temp).glob('data/servo_test/*/actions.jsonl')
                      for line in path.read_text().splitlines()]
        self.assertEqual([e['command']['slot'] for e in events if e['event'] == 'request'], [1, 2, 3])
        self.assertEqual(transport.call_count, 3)
        for slot, call in enumerate(transport.call_args_list, 1):
            value = call.kwargs['param'+str(slot)] * (-1 if slot == 2 else 1)
            low, high = params['PWM_MAIN_MIN'+str(slot)], params['PWM_MAIN_MAX'+str(slot)]
            self.assertAlmostEqual(low+(value+1)*(high-low)/2., 1500)
            self.assertEqual(call.kwargs['command'], 187)
            self.assertEqual([i for i in range(1, 7) if math.isfinite(call.kwargs['param'+str(i)])], [slot])
        sleep.assert_not_called()


if __name__ == '__main__':
    unittest.main()
