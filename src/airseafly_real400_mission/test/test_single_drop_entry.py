"""Public single-drop command; all flight/process/actuator interfaces are mocked."""
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from airseafly_real400_mission import single_drop_cli as cli
from airseafly_real400_mission.configuration import project_root
from airseafly_real400_mission.telemetry import TerminalReporter


class SingleDropEntryTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.flight_config = Path(folder.name)/'flight.yaml'
        self.flight_config.write_text(json.dumps(dict(common={}, mission_profiles={'delivery-route':
            dict(flight=dict(cruise_height_m=1.4, drop_height_m=.7))})))

    def invoke(self, *extra):
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            result = cli.main(['--x', '2.5', '--y', '-1.2', '--slot', '3',
                               '--flight-config', str(self.flight_config), *extra])
        return result, stream.getvalue()

    def test_preview_needs_no_route_ros_camera_or_payload_connection(self):
        with mock.patch.object(cli, 'manager_client', side_effect=AssertionError('no manager')), \
                mock.patch.object(cli, 'request', side_effect=AssertionError('no RPC')):
            code, text = self.invoke('--dry-run', '--json')
        value = json.loads(text)
        self.assertEqual(code, 0)
        self.assertEqual(value['mode'], 'drop_test')
        self.assertEqual(value['target_xy'], [2.5, -1.2])
        self.assertEqual(value['payload_total'], 1)
        self.assertEqual(value['selected_payload'], dict(slot=3, main_output=6, channel=1,
                                                      pwm_us=1350, servo_stage=2))
        self.assertEqual(value['heights_relative_to_takeoff_m'], dict(cruise=1.4, drop=.7))
        self.assertEqual(value['home_source'], 'takeoff_origin_xy')
        self.assertIsNone(value['home_xy'])
        self.assertEqual(value['finish_action'], 'return_home_and_land')
        self.assertEqual(value['mount_offset_body_m'], [0., 0.])
        self.assertEqual(value['visual_alignment']['frame_timeout_s'], 15.)

    def test_selected_slot_maps_directly_without_reset_or_other_releases(self):
        for slot, main, pwm in ((1, 6, 1190), (2, 7, 1190), (3, 6, 1350), (4, 7, 1350)):
            with self.subTest(slot=slot):
                _, text = self.invoke('--slot', str(slot), '--dry-run', '--json')
                value = json.loads(text)
                self.assertEqual(value['selected_payload']['main_output'], main)
                self.assertEqual(value['selected_payload']['pwm_us'], pwm)
                drops = [step for step in value['itinerary'] if step['kind'] == 'drop']
                self.assertEqual([step['drop_slot'] for step in drops], [slot])

    def test_custom_height_speed_and_setpoint_mode_reach_resolved_config(self):
        args = cli.parser().parse_args(['--x', '0', '--y', '0', '--slot', '2',
            '--takeoff-height', '1.1', '--drop-height', '.4', '--align-speed', '.2',
            '--delivery-setpoint-mode', 'position_yaw', '--set',
            'delivery_route_options.visual_alignment.frame_timeout_s=12'])
        config = cli.resolve_config(args)
        self.assertEqual(config['flight']['cruise_height_m'], 1.1)
        self.assertEqual(config['flight']['drop_height_m'], .4)
        self.assertEqual(config['motion']['native_horizontal_speed_mps'], .2)
        self.assertEqual(config['motion']['native_setpoint_mode'], 'position_yaw')
        self.assertEqual(config['delivery_route_options']['visual_alignment']['frame_timeout_s'], 12)

    def test_start_submits_only_requested_target_and_scopes_watch_to_task(self):
        client = mock.Mock()
        client.watch.return_value = 0
        with mock.patch.object(cli, 'manager_client', return_value=client), \
                mock.patch.object(cli, 'request', return_value=dict(ok=True, task_id='single-7', event_start=91)) as rpc:
            result, _ = self.invoke()
        self.assertEqual(result, 0)
        body = rpc.call_args.args[1]
        self.assertEqual((body['command'], body['mode']), ('start', 'drop_test'))
        self.assertEqual(body['resolved_config']['drop_test'], dict(target_xy=[2.5, -1.2], slot=3, finish='land'))
        self.assertNotIn('delivery_route', body['resolved_config'])
        self.assertEqual(client.watch.call_args.kwargs['expected_task_id'], 'single-7')
        self.assertEqual(client.watch.call_args.kwargs['after'], 91)
        self.assertTrue(client.watch.call_args.kwargs['finish_task'])

    def test_rejected_start_is_reported_without_watching_or_retrying(self):
        client = mock.Mock()
        with mock.patch.object(cli, 'manager_client', return_value=client), \
                mock.patch.object(cli, 'request', return_value=dict(ok=False, error='existing task')) as rpc:
            result, text = self.invoke()
        self.assertEqual(result, 1)
        self.assertIn('existing task', text)
        self.assertEqual(rpc.call_count, 1)
        client.watch.assert_not_called()

    def test_nonfinite_coordinate_is_not_sent_as_a_flight_command(self):
        with mock.patch.object(cli, 'manager_client', side_effect=AssertionError('no manager')):
            with self.assertRaises(ValueError):
                self.invoke('--x', 'nan', '--dry-run')

    def test_shell_is_relocatable_and_help_and_show_config_need_no_ros(self):
        script = project_root()/'flight/drop_test.sh'
        env = dict(os.environ, AIRSEAFLY_PYTHON=sys.executable, PYTHONDONTWRITEBYTECODE='1')
        for args in (['--help'], ['--x', '0', '--y', '-.5', '--slot', '4', '--show-config', '--json']):
            run = subprocess.run(['sh', str(script), *args], cwd='/tmp', env=env,
                                 capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)
            if '--json' in args:
                self.assertEqual(json.loads(run.stdout)['selected_payload']['slot'], 4)
            else:
                self.assertIn('--slot', run.stdout)

    def test_terminal_exposes_single_target_alignment_and_chosen_slot(self):
        lines = []
        TerminalReporter(emit=lines.append).status(dict(pose=[2., 3., 1.4], output=dict(
            state='DROP', phase='NATIVE_FIRST_FRAME', position_xyz=[2., 3., 1.4],
            drop_test=dict(slot=4, current_point=dict(drop_slot=4), visual_alignment=dict(
                stage=1, recorded_xy=[2.5, 3.2], fixes=[], fallback=dict(
                    remaining_s=12., retakes=0, retake_limit=1, waiting_reason='等待有效红点识别'))))))
        self.assertIn('投放点 4', lines[1])
        self.assertIn('锁定XY (2.50, 3.20)', lines[1])
        self.assertIn('剩余 12.0s', lines[1])


if __name__ == '__main__':
    unittest.main()
