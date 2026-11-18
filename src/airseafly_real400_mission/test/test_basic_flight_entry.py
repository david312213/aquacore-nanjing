"""Basic user commands exercised offline, including shell/cwd boundaries."""
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

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from airseafly_real400_mission import waypoint_route_cli as cli
from airseafly_real400_mission.waypoint_route import FRAME, FORMAT, load_route, save_route


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class BasicFlightEntryTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory(prefix='基础 试飞 ')
        self.addCleanup(folder.cleanup)
        self.directory = Path(folder.name).resolve()
        self.route = self.directory/'普通 航点.yaml'
        self.document = dict(format=FORMAT, version=1, frame=FRAME, points=[[1., 2.], [5., -3.]])
        save_route(self.route, self.document)
        # Tests of base defaults must not bind mutable field height/lap tuning.
        self.base_flight_config = self.directory/'base-flight.yaml'
        self.base_flight_config.write_text(yaml.safe_dump(dict(common={}, mission_profiles={
            'takeoff': {}, 'waypoint-route': {'navigation': {'backend': 'fixed_min_snap', 'laps': 2}}})))

    def shell(self, script, *args, input=None):
        result = subprocess.run(['sh', str(ROOT/'flight'/script), *map(str, args)],
            cwd=str(self.directory), env=dict(os.environ, AIRSEAFLY_PYTHON=sys.executable),
            input=input, text=True, capture_output=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stderr)
        return result.stdout

    def custom_config(self):
        value = yaml.safe_load((ROOT/'flight/flight.yaml').read_text())
        value['common']['flight']['cruise_height_m'] = 1.2
        value['common']['flight']['native_takeoff_speed_mps'] = .3
        value['common']['flight']['native_takeoff_acceleration_mps2'] = .2
        value['common']['planning']['max_velocity_mps'] = .4
        value['common']['planning']['max_acceleration_mps2'] = .15
        value['common']['planning']['map']['max_height_m'] = 1.8
        path = self.directory/'自定义 试飞.yaml'
        path.write_text(yaml.safe_dump(value), encoding='utf-8')
        return path

    def test_record_undo_save_then_shell_preview_in_unrelated_directory(self):
        self.shell('record_points.sh', '--demo', '--stdin', '--output', self.route,
            input='enter\nxy 1 2\nspace\nxy 99 99\nspace\nx\nxy 5 -3\nspace\nq\n')
        recorded = load_route(self.route)
        self.assertEqual((recorded['points'], recorded['entry_points']), (self.document['points'], []))
        value = json.loads(self.shell('fly_points.sh', '--route', self.route,
                                     '--flight-config', self.base_flight_config, '--dry-run', '--json'))
        self.assertEqual(value['mode'], 'waypoint_route')
        self.assertEqual(value['height_reference'], 'takeoff_origin_z')
        self.assertEqual([step['kind'] for step in value['itinerary']], ['takeoff', 'navigation', 'navigation', 'hold'])
        self.assertEqual([step['xy'] for step in value['itinerary'] if step['kind'] == 'navigation'], self.document['points'])
        self.assertEqual({step['height_m'] for step in value['itinerary']}, {1.})
        self.assertEqual(value['itinerary'][-1]['xy'], self.document['points'][0])
        self.assertEqual(value['backend'], 'fixed_min_snap')
        self.assertEqual(value['laps'], 2)
        self.assertEqual(value['arrival']['policy'], 'trajectory_time')
        self.assertEqual(value['finish_action'], 'HOLD')
        for key in ('payload', 'visual', 'landing'):
            self.assertFalse(value[key])
        self.assertEqual(value['parameters']['map']['max_height_m'], 1.4)

    def test_takeoff_default_and_explicit_configuration(self):
        default = json.loads(self.shell('takeoff.sh', '--flight-config', self.base_flight_config,
                                       '--dry-run', '--json'))
        self.assertTrue(default['dry_run'])
        self.assertEqual(default['controller'], 'px4_native')
        self.assertEqual(default['parameters']['native_reference_mode'], 'lio_direct')
        self.assertEqual(default['stages'], ['takeoff', 'hold'])
        self.assertEqual(default['finish_action'], 'HOLD')
        self.assertTrue(default['reference_stream_after_completion'])
        parameters = default['parameters']
        self.assertEqual(parameters['height_m'], 1.)
        self.assertEqual(parameters['takeoff_speed_mps'], .5)
        self.assertEqual(parameters['takeoff_acceleration_mps2'], .25)
        self.assertEqual(parameters['ego_speed_mps'], .5)
        self.assertEqual(parameters['ego_acceleration_mps2'], .25)
        self.assertEqual(parameters['map']['max_height_m'], 1.4)
        custom = json.loads(self.shell('takeoff.sh', '--flight-config', self.custom_config(), '--dry-run', '--json'))
        self.assertEqual(custom['parameters']['height_m'], 1.2)
        self.assertEqual(custom['parameters']['takeoff_speed_mps'], .3)
        self.assertEqual(custom['parameters']['map']['max_height_m'], 1.8)

    def test_waypoint_configuration_and_cli_overrides_keep_one_height(self):
        custom = self.custom_config()
        value = json.loads(self.shell('fly_points.sh', '--route', self.route, '--flight-config', custom,
            '--set', 'flight.cruise_height_m=0.8', '--height', '.9',
            '--takeoff-speed', '.35', '--takeoff-acceleration', '.18',
            '--speed', '.25', '--acceleration', '.12', '--dry-run', '--json'))
        self.assertEqual(value['height_m'], .9)
        self.assertEqual({step['height_m'] for step in value['itinerary']}, {.9})
        self.assertEqual(value['parameters']['takeoff_speed_mps'], .35)
        self.assertEqual(value['parameters']['takeoff_acceleration_mps2'], .18)
        self.assertEqual(value['parameters']['ego_speed_mps'], .25)
        self.assertEqual(value['parameters']['ego_acceleration_mps2'], .12)
        self.assertEqual(value['parameters']['map']['max_height_m'], 1.8)
        self.assertEqual(yaml.safe_load(custom.read_text())['common']['flight']['cruise_height_m'], 1.2)

    def test_hold_and_safe_land_preview_do_not_need_route_or_execute_commands(self):
        hold = json.loads(self.shell('hold.sh', '--dry-run'))
        landing = json.loads(self.shell('safe_land.sh', '--dry-run'))
        self.assertEqual(hold, dict(command='hold', dry_run=True))
        self.assertTrue(landing['dry_run'])
        self.assertEqual(landing['service'], '/px4ctrl/force_land')
        self.assertEqual(landing['native_mode'], 'AUTO.LAND')
        self.assertFalse(landing['starts_controller'])
        with mock.patch.object(cli, 'request', side_effect=AssertionError('preview must not send RPC')), \
                mock.patch.object(cli, 'manager_client', side_effect=AssertionError('preview must not create a manager')), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(cli.main(['hold', '--route', '/missing-route.yaml', '--dry-run']), 0)
        entry = load_module('basic_test_environment', ROOT/'scripts/flight_entry.py')
        with mock.patch.dict(sys.modules, {'flight_entry': entry}):
            basic = load_module('basic_test_dispatch', ROOT/'scripts/basic_flight_entry.py')
        with mock.patch.object(basic, 'execute_command', side_effect=AssertionError('preview must not launch ROS')), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(basic.main(['safe-land', '--dry-run']), 0)

    def test_takeoff_run_and_hold_use_the_same_manager_socket(self):
        paths = dict(socket=str(self.directory/'shared-manager.sock'))
        client = mock.Mock()
        client.watch.return_value = 0
        with mock.patch.object(cli, 'runtime_paths', return_value=paths), \
                mock.patch.object(cli, 'manager_client', return_value=client), \
                mock.patch.object(cli, 'request', return_value=dict(ok=True, task_id=42, event_start=7)) as rpc, \
                contextlib.redirect_stdout(io.StringIO()):
            with mock.patch.object(cli, 'load_route_snapshot', side_effect=AssertionError('takeoff does not require recorded points')):
                self.assertEqual(cli.main(['takeoff', '--route', '/missing-waypoints.yaml']), 0)
            self.assertEqual(cli.main(['run', '--route', str(self.route)]), 0)
            with mock.patch.object(cli, 'load_route_snapshot', side_effect=AssertionError('hold does not require recorded points')):
                self.assertEqual(cli.main(['hold', '--route', '/missing-waypoints.yaml']), 0)
        self.assertEqual([call.args[0] for call in rpc.call_args_list], [paths['socket']]*3)
        commands = [call.args[1] for call in rpc.call_args_list]
        self.assertEqual([body['command'] for body in commands], ['start', 'start', 'hold'])
        self.assertEqual(commands[0]['mode'], 'basic_takeoff')
        self.assertNotIn('waypoint_route', commands[0]['resolved_config'])
        self.assertEqual(commands[1]['mode'], 'waypoint_route')
        self.assertEqual(commands[1]['resolved_config']['waypoint_route'], self.document)
        for body in commands[:2]:
            self.assertFalse(body['resolved_config']['payload']['pd_profiles_enabled'])
            self.assertFalse(body['resolved_config']['sensors']['cameras_enabled'])
            self.assertFalse(body['resolved_config']['recording']['enabled'])
        self.assertEqual(client.connect_or_start.call_count, 2)
        self.assertEqual([call.args[0] for call in client.connect_or_start.call_args_list], [paths]*2)
        self.assertEqual([call.args[0] for call in client.watch.call_args_list], [paths]*2)

    def test_basic_shell_dispatcher_sends_takeoff_and_route_to_shared_cli(self):
        entry = load_module('basic_test_shared_environment', ROOT/'scripts/flight_entry.py')
        with mock.patch.object(entry, 'execute_command', return_value=0) as execute, \
                mock.patch.dict(sys.modules, {'flight_entry': entry}):
            basic = load_module('basic_test_shared_dispatch', ROOT/'scripts/basic_flight_entry.py')
            for action, command in (('takeoff', 'takeoff'), ('fly-points', 'run'), ('hold', 'hold')):
                with self.subTest(action=action):
                    self.assertEqual(basic.main([action, '--dry-run']), 0)
                    invocation = execute.call_args.args[0]
                    self.assertEqual(Path(invocation[1]), PACKAGE/'scripts/fly_waypoints.py')
                    self.assertEqual(invocation[2], command)
                    self.assertTrue(execute.call_args.kwargs['offline'])

    def test_takeoff_preview_requires_no_route_and_has_no_payload_or_visual_stage(self):
        with mock.patch.object(cli, 'load_route_snapshot', side_effect=AssertionError('takeoff does not load a route')), \
                mock.patch.object(cli, 'request', side_effect=AssertionError('preview must not send RPC')), \
                mock.patch.object(cli, 'manager_client', side_effect=AssertionError('preview must not create a manager')):
            stream = io.StringIO()
            with contextlib.redirect_stdout(stream):
                self.assertEqual(cli.main(['takeoff', '--route', '/missing-waypoints.yaml', '--dry-run', '--json']), 0)
        preview = json.loads(stream.getvalue())
        self.assertEqual(preview['stages'], ['takeoff', 'hold'])
        self.assertEqual(preview['finish_action'], 'HOLD')
        self.assertFalse(preview['payload'])
        self.assertFalse(preview['visual'])
        self.assertFalse(preview['landing'])

    def test_default_first_two_terminal_previews_use_camera_free_test_profile(self):
        sensor = json.loads(self.shell('1_sensors.sh', '--dry-run'))
        localization = json.loads(self.shell('2_localization.sh', '--dry-run'))
        for result in (sensor, localization):
            self.assertEqual(Path(result['flight_config']), ROOT/'flight/flight.yaml')
            self.assertTrue(result['dry_run'])
            self.assertFalse(result['takeoff'])
            self.assertFalse(result['payload'])
            self.assertFalse(result['landing'])
        self.assertIn('cameras_enabled:=false', sensor['launch_arguments'])
        self.assertEqual(sensor['rc_payload'], dict(channel=9, low_max_us=1300, high_min_us=1700,
            closed_pwm_us=900, open_pwm_us=1350))
        self.assertIn('flight_config:='+str(ROOT/'flight/flight.yaml'), sensor['launch_arguments'])
        self.assertEqual(sensor['mapping']['planning.map.max_height_m']['value'], 1.4)
        selected = json.loads(self.shell('1_sensors.sh', '--flight-config', self.custom_config(), '--dry-run'))
        self.assertEqual(Path(selected['flight_config']), self.directory/'自定义 试飞.yaml')
        self.assertIn('cameras_enabled:=false', selected['launch_arguments'])
        self.assertIn('flight_config:='+str(self.directory/'自定义 试飞.yaml'), selected['launch_arguments'])

    def test_controller_default_uses_test_profile_and_disables_camera_recording(self):
        entry = load_module('basic_test_controller_entry', ROOT/'scripts/flight_entry.py')
        with mock.patch.dict(os.environ), mock.patch.object(entry.os, 'execv', side_effect=RuntimeError('exec intercepted')) as execute:
            with self.assertRaisesRegex(RuntimeError, 'exec intercepted'):
                entry.main(['controller'])
            self.assertEqual(Path(os.environ['AIRSEAFLY_FLIGHT_CONFIG']), ROOT/'flight/flight.yaml')
            self.assertIn(str(ROOT/'scripts/start_ego_px4_controller.sh'), execute.call_args.args[1])
        custom = self.custom_config()
        with mock.patch.dict(os.environ), mock.patch.object(entry.os, 'execv', side_effect=RuntimeError('exec intercepted')):
            with self.assertRaisesRegex(RuntimeError, 'exec intercepted'):
                entry.main(['controller', '--flight-config', str(custom)])
            self.assertEqual(Path(os.environ['AIRSEAFLY_FLIGHT_CONFIG']), custom)

    def test_controller_shell_preview_is_offline_and_disables_cameras_and_video(self):
        preview = json.loads(self.shell('3_controller.sh', '--dry-run'))
        self.assertTrue(preview['dry_run'])
        self.assertEqual(preview['start_action'], 'controller_only')
        self.assertEqual(Path(preview['flight_config']), ROOT/'flight/flight.yaml')
        for key in ('recording', 'cameras_enabled', 'takeoff', 'payload', 'landing'):
            self.assertFalse(preview[key])

    def test_ordinary_default_recording_does_not_replace_contest_route(self):
        recorder = load_module('basic_test_recorder', PACKAGE/'scripts/record_waypoints.py')
        self.assertEqual(recorder.DEFAULT_OUTPUT, ROOT/'data/routes/waypoints.yaml')
        self.assertNotEqual(recorder.DEFAULT_OUTPUT, ROOT/'data/routes/delivery_route.yaml')
        contest = self.directory/'delivery_route.yaml'
        contest.write_text('contest route sentinel\n', encoding='utf-8')
        output = self.directory/'waypoints.yaml'
        with mock.patch.object(recorder, 'DEFAULT_OUTPUT', output), \
                mock.patch.object(recorder.sys, 'stdin', io.StringIO('enter\nxy 1 2\nspace\nq\n')), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(recorder.main(['--demo', '--stdin']), 0)
        self.assertEqual(load_route(output)['points'], [[1., 2.]])
        self.assertEqual(contest.read_text(), 'contest route sentinel\n')

    def test_help_works_without_ros_or_route_in_any_directory(self):
        for script in ('takeoff.sh', 'record_points.sh', 'fly_points.sh', 'hold.sh', 'safe_land.sh'):
            with self.subTest(script=script):
                self.assertTrue(self.shell(script, '--help').strip())
        self.assertIn('--flight-config', self.shell('takeoff.sh', '--help'))
        self.assertIn('--flight-config', self.shell('fly_points.sh', '--help'))


if __name__ == '__main__':
    unittest.main()
