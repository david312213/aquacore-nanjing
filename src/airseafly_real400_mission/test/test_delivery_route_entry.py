"""Offline user-entry integration; no ROS, flight or actuator operations."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from airseafly_real400_mission import delivery_route_cli as cli
from airseafly_real400_mission.configuration import project_root
from airseafly_real400_mission.guided_route import GuidedRouteRecorder, STAGES, save_route
from airseafly_real400_mission.telemetry import TerminalReporter

ROOT = project_root()


def recorded_document():
    recorder = GuidedRouteRecorder()
    for index, stage in enumerate(STAGES):
        recorder.observe_xy(index+.1, -index-.2)
        recorder.key(' ')
        if stage['mode'] == 'variable':
            recorder.observe_xy(index+.4, -index-.5)
            recorder.key(' ')
            recorder.key('\n')
    return recorder.to_dict()


class DeliveryEntryTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory(prefix='配送 录点 ')
        self.addCleanup(folder.cleanup)
        self.path = Path(folder.name)/'航线.yaml'
        self.document = recorded_document()
        save_route(self.path, self.document)
        # These assertions describe the shipped base defaults, not mutable field tuning.
        self.flight_config = self.path.parent/'test-flight.yaml'
        self.flight_config.write_text(json.dumps(dict(common={}, mission_profiles={'delivery-route': {}})))

    def invoke(self, args):
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            status = cli.main(args+['--flight-config', str(self.flight_config)])
        self.assertEqual(status, 0)
        return stream.getvalue()

    def test_dry_run_is_no_connection_and_shared_four_release_itinerary(self):
        with mock.patch.object(cli, 'manager_client', side_effect=AssertionError('no manager')), \
                mock.patch.object(cli, 'request', side_effect=AssertionError('no RPC')):
            value = json.loads(self.invoke(['--route', str(self.path), '--dry-run', '--json']))
        steps = value['itinerary']
        drops = [step for step in steps if step['kind'] == 'drop']
        self.assertEqual([step['drop_slot'] for step in drops], [1, 2, 3, 4])
        self.assertEqual([step['main_output'] for step in drops], [6, 7, 6, 7])
        self.assertEqual([step['pwm_us'] for step in drops], [1190, 1190, 1350, 1350])
        self.assertTrue(all(step['alignment_samples'] == 1 for step in drops))
        self.assertTrue(all(step['mount_offset_body_m'] == [0., 0.] for step in drops))
        self.assertEqual(value['visual_alignment']['frame_timeout_s'], 15.)
        self.assertEqual(value['visual_alignment']['motion_timeout_margin_s'], 15.)
        self.assertEqual(value['visual_alignment']['retake_limit'], 1)
        self.assertEqual(value['recognition_fallback_xy'], 'last_visual_target_or_recorded_waypoint')
        self.assertEqual(value['success_identity'], 'route_index')
        self.assertEqual({step['lap'] for step in drops}, {2})
        self.assertEqual(steps[-2]['xy'], self.document['home'])
        self.assertEqual(steps[-1]['kind'], 'land')
        # Both laps close at the first point; HOME is reached from there.
        self.assertEqual([step['xy'] for step in steps if step.get('closing')],
                         [self.document['route'][0]['xy']]*2)
        self.assertTrue(steps[-3]['closing'])

    def test_configured_laps_change_preview_itinerary_and_text(self):
        default = json.loads(self.invoke(['--route', str(self.path), '--dry-run', '--json']))
        self.assertEqual(default['laps'], 1)  # cruise laps; one more lap drops
        points = [p['xy'] for p in self.document['route']]
        last = max(i for i, p in enumerate(self.document['route']) if p['kind'] == 'drop')
        for laps in (1, 2, 5):
            args = ['--route', str(self.path), '--dry-run', '--set', 'delivery_route_options.laps=%d' % (laps-1)]
            value = json.loads(self.invoke(args+['--json']))
            steps = value['itinerary']
            self.assertEqual(value['laps'], laps-1)
            navigation = [s['xy'] for s in steps if s['kind'] == 'navigation']
            self.assertEqual(navigation, [points[0]]+(points[1:]+[points[0]])*laps+[self.document['home']])
            self.assertEqual([s['completes_lap'] for s in steps if s.get('closing')], list(range(1, laps+1)))
            drops = [s for s in steps if s['kind'] == 'drop']
            self.assertEqual([s['drop_slot'] for s in drops], [1, 2, 3, 4])
            self.assertEqual({s['lap'] for s in drops}, {laps})
            self.assertEqual(steps[-1]['kind'], 'land')
            text = self.invoke(args)
            self.assertNotIn('两圈', text)
            if laps == 1:
                self.assertIn('一圈，第一圈边飞边按 1→2→3→4 释放', text)
                self.assertIn('只飞一圈', text)
            else:
                self.assertIn('巡航 %d 圈后第 %d 圈按 1→2→3→4 释放' % (laps-1, laps), text)
                self.assertIn('满载绕圈 %d 圈' % (laps-1), text)
        with self.assertRaisesRegex(ValueError, '正整数'):
            self.invoke(['--route', str(self.path), '--dry-run', '--set', 'delivery_route_options.laps=-1'])

    def test_submit_resolved_route_to_existing_manager(self):
        client = mock.Mock()
        client.watch.return_value = 0
        with mock.patch.object(cli, 'manager_client', return_value=client), \
                mock.patch.object(cli, 'request', return_value=dict(ok=True, event_start=17)) as rpc:
            self.invoke(['--route', str(self.path)])
        body = rpc.call_args.args[1]
        self.assertEqual((body['command'], body['mode']), ('start', 'delivery_route'))
        config = body['resolved_config']
        self.assertEqual(config['delivery_route'], self.document)
        self.assertFalse(config['recording']['enabled'])
        self.assertFalse(config['sensors']['cameras_enabled'])
        self.assertFalse(config['payload']['pd_profiles_enabled'])
        self.assertEqual(config['flight']['native_reference_mode'], 'lio_direct')
        self.assertEqual(client.watch.call_args.kwargs['after'], 17)

    def test_partial_save_does_not_submit_unrecorded_course(self):
        save_route(self.path, GuidedRouteRecorder().to_dict())
        with mock.patch.object(cli, 'manager_client', side_effect=AssertionError('no manager')):
            with self.assertRaisesRegex(ValueError, '尚未录完'):
                self.invoke(['--route', str(self.path)])

    def test_independent_height_overrides(self):
        value = json.loads(self.invoke(['--route', str(self.path), '--dry-run', '--json',
                                       '--takeoff-height', '1.5', '--gate-height', '1.8', '--drop-height', '.6']))
        self.assertEqual(value['heights_relative_to_takeoff_m'], dict(cruise=1.5, gate=1.8, drop=.6))
        for step in value['itinerary']:
            if step['kind'] != 'land':
                self.assertEqual(step['height_m'], value['heights_relative_to_takeoff_m'][step['height_role']])

    def test_alignment_override_and_text_preview_explain_fallback(self):
        args = ['--route', str(self.path), '--dry-run', '--set',
                'delivery_route_options.visual_alignment.frame_timeout_s=10']
        value = json.loads(self.invoke(args+['--json']))
        self.assertEqual(value['visual_alignment']['frame_timeout_s'], 10)
        text = self.invoke(args)
        self.assertIn('取图每次最多 10s', text)
        self.assertIn('下降至识别高度 → 红点对准', text)
        self.assertIn('定位 Z=0.50m', text)
        self.assertIn('实测位置和高度投放', text)

    def test_terminal_reports_point_deadline_retake_and_forced_release(self):
        lines = []
        TerminalReporter(emit=lines.append).status(dict(pose=[1., 2., .8], output=dict(
            state='DROP', phase='WAIT_ACK', position_xyz=[1., 2., .8], delivery_route=dict(
                current_point=dict(drop_slot=2), visual_alignment=dict(stage=2,
                    fixes=[dict(target_xyz=[1.1, 2.2, 1.4])], fallback=dict(
                        remaining_s=None, retakes=1, retake_limit=1, active=True,
                        reason='投放下降到位超时', release_actual_height_m=.8))))))
        self.assertIn('投放点 2', lines[1])
        self.assertIn('锁定XY (1.10, 2.20)', lines[1])
        self.assertIn('重拍 1/1', lines[1])
        self.assertIn('兜底：投放下降到位超时', lines[1])
        self.assertIn('实际释放高度 0.800m', lines[1])

    def test_magnet_uses_persistent_command_without_route(self):
        with mock.patch.object(cli, 'manager_client', side_effect=AssertionError('no manager')):
            value = json.loads(self.invoke(['payload', 'release', '4', '--dry-run', '--route', '/missing.yaml']))
        self.assertEqual(value['payload']['command'], 187)
        self.assertEqual(value['payload']['main_output'], 7)
        self.assertEqual(value['payload']['pwm_us'], 1350)
        self.assertAlmostEqual(900+(value['payload']['params'][1]+1)*550, 1350)
        self.assertTrue(all(v is None for i, v in enumerate(value['payload']['params']) if i != 1))

    def test_shell_recorder_to_flight_preview_round_trip(self):
        # Exercise both actual user-facing wrappers, with a path containing spaces.
        inputs = []
        for index, stage in enumerate(STAGES):
            inputs.extend(['xy %d %d' % (index, -index), 'space'])
            if stage['mode'] == 'variable':
                inputs.extend(['xy %s %s' % (index+.25, -index-.25), 'space', 'enter'])
        inputs.append('q')
        env = dict(os.environ, AIRSEAFLY_PYTHON=sys.executable)
        record = subprocess.run(['sh', str(ROOT/'flight/record_route.sh'), '--wizard', '--demo', '--stdin', '--output', str(self.path)],
                                input='\n'.join(inputs)+'\n', text=True, capture_output=True, env=env)
        self.assertEqual(record.returncode, 0, record.stderr)
        run = subprocess.run(['sh', str(ROOT/'flight/4_delivery_route.sh'), '--route', str(self.path), '--dry-run', '--json'],
                             text=True, capture_output=True, env=env)
        self.assertEqual(run.returncode, 0, run.stderr)
        result = json.loads(run.stdout)
        self.assertEqual(result['route_points'], 12)
        self.assertEqual(result['home'], [0., 0.])

    def test_shell_free_recorder_to_flight_preview_round_trip(self):
        from airseafly_real400_mission.guided_route import FreeRouteRecorder
        # Existing v2 files resume in the original one-route mode.
        save_route(self.path, FreeRouteRecorder().to_dict())
        inputs = ['xy -4 -5', 'h', 'xy 0 0', 'space', 'xy 3 0', '3', 'space', '2', 'xy 4 1', 'space',
                  '1', 'xy 6 0', 'space', 'xy 6 3', '3', 'space', 'q']
        env = dict(os.environ, AIRSEAFLY_PYTHON=sys.executable)
        record = subprocess.run(['sh', str(ROOT/'flight/record_route.sh'), '--demo', '--stdin', '--resume', str(self.path)],
                                input='\n'.join(inputs)+'\n', text=True, capture_output=True, env=env)
        self.assertEqual(record.returncode, 0, record.stderr)
        run = subprocess.run(['sh', str(ROOT/'flight/4_delivery_route.sh'), '--route', str(self.path), '--dry-run', '--json'],
                             text=True, capture_output=True, env=env)
        self.assertEqual(run.returncode, 0, run.stderr)
        result = json.loads(run.stdout)
        self.assertEqual((result['recording_mode'], result['route_points'], result['drop_points']), ('free', 5, 2))
        self.assertEqual([s['drop_slot'] for s in result['itinerary'] if s['kind'] == 'drop'], [1, 2])
        self.assertEqual(result['home'], [-4., -5.])

    def test_camera_disabled_startup_keeps_both_camera_includes_conditional(self):
        spec = importlib.util.spec_from_file_location('delivery_startup', ROOT/'scripts/startup_tuning.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        args, extra = module.parse_arguments(['sensors', '--flight-config', str(ROOT/'flight/flight.yaml'), '--dry-run'])
        _effective, _report, launch_args = module.resolve_startup(args, extra)
        self.assertIn('cameras_enabled:=false', launch_args)
        tree = ET.parse(ROOT/'config/aircraft/sensors.launch')
        includes = [element for element in tree.findall('include') if 'camera' in element.get('file', '')]
        self.assertEqual(includes, [])
        self.assertTrue(all(element.get('if') == '$(arg cameras_enabled)' for element in includes))

    def test_new_manager_is_started_without_default_camera_recording(self):
        client = cli.manager_client()
        config = cli.resolve_config(cli.parser().parse_args([]))
        with mock.patch.object(client, 'require_ros_master'), \
                mock.patch.object(client, 'project_root', return_value=self.path.parent), \
                mock.patch.object(client, 'request', side_effect=[OSError('not running'), dict(ok=True)]), \
                mock.patch.object(client, 'manager_registered', return_value=True), \
                mock.patch.object(client, 'manager_lock_held', return_value=False), \
                mock.patch.object(client, 'socket_identity', return_value=None), \
                mock.patch.object(client, 'ProcessLock'), \
                mock.patch.object(client.subprocess, 'Popen') as popen:
            client.connect_or_start(dict(socket='mock.sock', manager_lock='mock.lock'), config)
        self.assertIn('--no-recording', popen.call_args.args[0])

    def test_controller_entry_disables_recording_from_delivery_config(self):
        spec = importlib.util.spec_from_file_location('delivery_flight_entry', ROOT/'scripts/flight_entry.py')
        entry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(entry)
        with mock.patch.dict(os.environ), mock.patch.object(entry.os, 'execv', side_effect=RuntimeError('exec intercepted')):
            with self.assertRaisesRegex(RuntimeError, 'exec intercepted'):
                entry.main(['controller', '--flight-config', str(ROOT/'flight/flight.yaml')])


if __name__ == '__main__':
    unittest.main()
