"""Takeoff/manual-release entry; transport and all aircraft actions stay mocked."""
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from airseafly_real400_mission import hover_drop_cli as cli
from airseafly_real400_mission.configuration import project_root
from airseafly_real400_mission.payload import actuator_command
from airseafly_real400_mission.rpc import CommandNotSent


class HoverDropEntryTests(unittest.TestCase):
    def invoke(self, *arguments):
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            code = cli.main(list(arguments))
        return code, stream.getvalue()

    @staticmethod
    def completed_watch(*args, **kwargs):
        request_id = kwargs.get('expected_height_id')
        output = dict(done=True, state='COMPLETE', mode='takeoff',
                      phase='MANUAL_HEIGHT_COMPLETE' if request_id else 'TAKEOFF_COMPLETE')
        if request_id:
            output['manual_height'] = dict(request_id=request_id, state='complete')
        kwargs['completed_status'].update(task_id='hover-7', output=output)
        return 0

    @staticmethod
    def successful_rpc(_path, body, *args, **kwargs):
        if body['command'] == 'start':
            return dict(ok=True, task_id='hover-7', event_start=91)
        if body['command'] == 'manual_height':
            return dict(ok=True, task_id='hover-7', request_id=body['request_id'], event_start=98)
        if body['command'] == 'payload':
            return dict(ok=True, result=dict(success=True))
        raise AssertionError('Unexpected RPC: '+repr(body))

    def live(self, choices, watch=None, rpc=None, landing=0, extra=()):
        client = mock.Mock()
        client.watch.side_effect = watch or self.completed_watch
        with mock.patch.object(cli, 'manager_client', return_value=client), \
                mock.patch.object(cli, 'request', side_effect=rpc or self.successful_rpc) as transport, \
                mock.patch.object(cli, 'land_current', return_value=landing,
                                  side_effect=landing if isinstance(landing, BaseException) else None) as land, \
                mock.patch('builtins.input', side_effect=choices):
            code, text = self.invoke(*extra)
        return code, text, client, transport, land

    def test_dry_run_is_offline_and_preserves_competition_payload_mapping(self):
        with mock.patch.object(cli, 'manager_client', side_effect=AssertionError('no manager')), \
                mock.patch.object(cli, 'request', side_effect=AssertionError('no RPC')), \
                mock.patch.object(cli, 'run_hover_drop_console', side_effect=AssertionError('no menu')), \
                mock.patch.object(cli, 'land_current', side_effect=AssertionError('no landing')):
            code, text = self.invoke('--dry-run', '--json')
        self.assertEqual(code, 0)
        value = json.loads(text)
        self.assertEqual(value['takeoff_mode'], 'basic_takeoff')
        self.assertEqual(value['height_m'], 1.4)
        self.assertFalse(value['automatic_release'])
        self.assertFalse(value['automatic_reset'])
        self.assertFalse(value['visual'])
        self.assertFalse(value['navigation'])
        self.assertEqual([item['main_output'] for item in value['payload_options']], [6, 7, 6, 7])
        self.assertEqual([item['pwm_us'] for item in value['payload_options']], [1190, 1190, 1350, 1350])
        config = cli.resolve_config(cli.parser().parse_args([]))
        self.assertEqual(config['flight']['cruise_height_m'], 1.4)
        commands = [actuator_command(slot, 'release', config['payload']) for slot in range(1, 5)]
        self.assertEqual([item['main_output'] for item in commands], [6, 7, 6, 7])
        self.assertEqual([item['pwm_us'] for item in commands], [1190, 1190, 1350, 1350])

    def test_cli_overrides_reach_takeoff_and_height_control_configuration(self):
        args = cli.parser().parse_args(['--height', '.8', '--takeoff-speed', '.3',
            '--takeoff-acceleration', '.2', '--drop-speed', '.12', '--recover-speed', '.25',
            '--delivery-setpoint-mode', 'position_yaw', '--set', 'payload.settle_s=.6'])
        config = cli.resolve_config(args)
        self.assertEqual(config['flight']['cruise_height_m'], .8)
        self.assertEqual(config['flight']['native_takeoff_speed_mps'], .3)
        self.assertEqual(config['flight']['native_takeoff_acceleration_mps2'], .2)
        self.assertEqual(config['flight']['drop_speed_mps'], .12)
        self.assertEqual(config['flight']['recover_speed_mps'], .25)
        self.assertEqual(config['motion']['native_setpoint_mode'], 'position_yaw')
        self.assertEqual(config['payload']['settle_s'], .6)

    def test_menu_follows_completed_takeoff_and_each_number_sends_once_without_reset(self):
        code, text, client, transport, land = self.live(['1', '2', '3', '4', '2', 'q'])
        self.assertEqual(code, 0)
        bodies = [call.args[1] for call in transport.call_args_list]
        self.assertEqual([body['command'] for body in bodies], ['start'] + ['payload']*5)
        self.assertEqual(bodies[0]['mode'], 'basic_takeoff')
        self.assertEqual([body['slot'] for body in bodies[1:]], [1, 2, 3, 4, 2])
        self.assertTrue(all(body['action'] == 'release' for body in bodies[1:]))
        self.assertTrue(all(body['resolved_config'] == bodies[0]['resolved_config'] for body in bodies[1:]))
        commands = [actuator_command(body['slot'], body['action'], body['resolved_config']['payload'])
                    for body in bodies[1:]]
        self.assertEqual([item['command'] for item in commands], [187]*5)
        self.assertEqual([item['main_output'] for item in commands], [6, 7, 6, 7, 7])
        self.assertEqual([item['pwm_us'] for item in commands], [1190, 1190, 1350, 1350, 1190])
        self.assertEqual(client.watch.call_count, 1)
        self.assertEqual(client.watch.call_args.kwargs['expected_task_id'], 'hover-7')
        self.assertEqual(client.watch.call_args.kwargs['after'], 91)
        self.assertTrue(client.watch.call_args.kwargs['finish_task'])
        self.assertIn('completed_status', client.watch.call_args.kwargs)
        land.assert_called_once()

    def test_zero_watch_result_without_completion_does_not_open_menu_after_interrupt(self):
        client = mock.Mock()
        client.watch.return_value = 0  # Existing watch returns zero after Ctrl-C.
        with mock.patch.object(cli, 'manager_client', return_value=client), \
                mock.patch.object(cli, 'request', side_effect=self.successful_rpc) as rpc, \
                mock.patch.object(cli, 'run_hover_drop_console') as menu, \
                mock.patch.object(cli, 'land_current') as land:
            self.invoke()
        menu.assert_not_called()
        land.assert_not_called()
        self.assertEqual(rpc.call_count, 1)

    def test_hold_completion_does_not_open_menu(self):
        def held(*args, **kwargs):
            kwargs['completed_status'].update(task_id='hover-7', output=dict(done=False, state='HOLD'))
            return 0
        with mock.patch.object(cli, 'run_hover_drop_console') as menu:
            code, _, _, transport, land = self.live([], watch=held)
        menu.assert_not_called()
        land.assert_not_called()
        self.assertEqual(transport.call_count, 1)

    def test_rejected_start_never_watches_opens_menu_or_retries(self):
        client = mock.Mock()
        with mock.patch.object(cli, 'manager_client', return_value=client), \
                mock.patch.object(cli, 'request', return_value=dict(ok=False, error='existing task')) as rpc, \
                mock.patch.object(cli, 'run_hover_drop_console') as menu:
            code, text = self.invoke()
        self.assertNotEqual(code, 0)
        self.assertIn('existing task', text)
        self.assertEqual(rpc.call_count, 1)
        client.watch.assert_not_called()
        menu.assert_not_called()

    def test_unconfirmed_payload_is_never_retried(self):
        for failure in (dict(ok=False, error='FCU rejected'), OSError('receipt lost'),
                        CommandNotSent('not connected')):
            with self.subTest(failure=failure):
                def response(path, body, *args, **kwargs):
                    if body['command'] == 'payload':
                        if isinstance(failure, Exception):
                            raise failure
                        return failure
                    return self.successful_rpc(path, body, *args, **kwargs)
                code, text, _, transport, land = self.live(['4', 'q'], rpc=response)
                self.assertEqual([call.args[1]['command'] for call in transport.call_args_list],
                                 ['start', 'payload'])
                land.assert_called_once()

    def test_menu_interrupt_or_eof_only_exits_menu_without_land_or_additional_commands(self):
        for interrupt in (KeyboardInterrupt(), EOFError()):
            with self.subTest(interrupt=interrupt):
                code, _, _, transport, land = self.live([interrupt])
                self.assertEqual(code, 0)
                self.assertEqual(transport.call_count, 1)
                land.assert_not_called()

    def test_failed_landing_still_closes_menu_without_further_release(self):
        code, _, _, transport, land = self.live(['q', '1'], landing=1)
        self.assertEqual(code, 1)
        self.assertEqual(transport.call_count, 1)
        land.assert_called_once()

    def test_interrupted_landing_observation_does_not_return_to_payload_input(self):
        code, _, _, transport, land = self.live(['q', '4'], landing=KeyboardInterrupt())
        self.assertEqual(code, 0)
        self.assertEqual(transport.call_count, 1)
        land.assert_called_once()

    def test_height_change_reuses_original_task_then_returns_to_manual_release(self):
        code, _, client, transport, land = self.live(['h .8', '3', 'q'])
        self.assertEqual(code, 0)
        bodies = [call.args[1] for call in transport.call_args_list]
        self.assertEqual([body['command'] for body in bodies], ['start', 'manual_height', 'payload'])
        height = bodies[1]
        self.assertEqual(height['height_m'], .8)
        self.assertEqual(height['task_id'], 'hover-7')
        self.assertTrue(height['request_id'])
        self.assertEqual(client.watch.call_count, 2)
        kwargs = client.watch.call_args.kwargs
        self.assertEqual(kwargs['expected_task_id'], 'hover-7')
        self.assertEqual(kwargs['expected_height_id'], height['request_id'])
        self.assertEqual(kwargs['after'], 98)
        self.assertTrue(kwargs['finish_task'])
        self.assertIn('completed_status', kwargs)
        self.assertEqual(bodies[2]['slot'], 3)
        land.assert_called_once()

    def test_interrupted_height_observation_does_not_resume_payload_menu(self):
        calls = []
        def watch(*args, **kwargs):
            calls.append(kwargs)
            if len(calls) == 1:
                return self.completed_watch(*args, **kwargs)
            return 0  # Interrupted watch: no terminal completion snapshot.
        code, _, _, transport, land = self.live(['h .8', '4', 'q'], watch=watch)
        self.assertEqual([call.args[1]['command'] for call in transport.call_args_list],
                         ['start', 'manual_height'])
        land.assert_not_called()

    def test_rejected_height_change_exits_menu_without_retry_or_release(self):
        def reject_height(path, body, *args, **kwargs):
            if body['command'] == 'manual_height':
                return dict(ok=False, error='original task replaced')
            return self.successful_rpc(path, body, *args, **kwargs)
        code, text, client, transport, land = self.live(['h .8', '4', 'q'], rpc=reject_height)
        self.assertNotEqual(code, 0)
        self.assertIn('original task replaced', text)
        self.assertEqual([call.args[1]['command'] for call in transport.call_args_list],
                         ['start', 'manual_height'])
        self.assertEqual(client.watch.call_count, 1)
        land.assert_not_called()

    def test_shell_help_and_show_config_work_without_ros_from_another_directory(self):
        script = project_root()/'flight/hover_drop.sh'
        env = dict(os.environ, AIRSEAFLY_PYTHON=sys.executable, PYTHONDONTWRITEBYTECODE='1')
        for args in (['--help'], ['--height', '.9', '--show-config', '--json']):
            with self.subTest(arguments=args):
                result = subprocess.run(['sh', str(script), *args], cwd='/tmp', env=env,
                                        capture_output=True, text=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stderr)
                if '--json' in args:
                    self.assertEqual(json.loads(result.stdout)['height_m'], .9)
                else:
                    self.assertIn('--height', result.stdout)


if __name__ == '__main__':
    unittest.main()
