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

from test_drop_points import document
from airseafly_real400_mission import drop_points_cli as cli
from airseafly_real400_mission.configuration import project_root
from airseafly_real400_mission.drop_points import load_points, save_points
from airseafly_real400_mission.waypoint_route import DEFAULT_ROUTE, load_route, save_route
from airseafly_real400_mission.telemetry import TerminalReporter


class DropPointsEntryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.route = Path(self.temp.name)/'投放 points.yaml'
        save_points(self.route, document())
        self.flight_config = Path(self.temp.name)/'flight.yaml'
        self.flight_config.write_text(json.dumps(dict(common={}, mission_profiles={'delivery-route': {}})))

    def invoke(self, *args):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = cli.main(['--route', str(self.route), '--flight-config', str(self.flight_config), *args])
        return result, output.getvalue()

    def test_preview_selection_maps_all_four_stages_without_hardware(self):
        for count in range(1, 5):
            with self.subTest(count=count), mock.patch.object(cli, 'manager_client', side_effect=AssertionError('no hardware')):
                code, text = self.invoke('--count', str(count), '--takeoff-height', '1.0', '--drop-height', '.7', '--dry-run', '--json')
                value = json.loads(text)
                self.assertEqual(code, 0)
                self.assertEqual(value['payload_total'], count)
                self.assertEqual([(v['main_output'], v['pwm_us']) for v in value['targets']],
                                 [(6, 1190), (7, 1190), (6, 1350), (7, 1350)][:count])
                self.assertEqual(value['heights_relative_to_takeoff_m'], dict(cruise=1., drop=.7))
                self.assertIsNone(value['home_xy'])
                self.assertEqual(len(value['itinerary']), 2*count+2)

    def test_launch_passes_selected_copy_and_watches_only_new_task(self):
        client = mock.Mock(); client.watch.return_value = 0
        with mock.patch.object(cli, 'manager_client', return_value=client), \
                mock.patch.object(cli, 'request', return_value=dict(ok=True, task_id='recorded-1', event_start=5)) as rpc:
            code, _ = self.invoke('--count', '2')
        self.assertEqual(code, 0)
        body = rpc.call_args.args[1]
        self.assertEqual(body['mode'], 'drop_points')
        self.assertEqual(body['resolved_config']['drop_points']['points'], document(2)['points'])
        self.assertEqual(len(load_points(self.route)['points']), 4)
        self.assertEqual(client.watch.call_args.kwargs['expected_task_id'], 'recorded-1')
        self.assertTrue(client.watch.call_args.kwargs['finish_task'])

    def test_record_undo_resume_and_shell_preview_from_another_directory(self):
        env = dict(os.environ, AIRSEAFLY_PYTHON=sys.executable, PYTHONDONTWRITEBYTECODE='1')
        recorder = project_root()/'flight/record_drop_points.sh'
        run = subprocess.run(['sh', str(recorder), '--demo', '--stdin', '--output', str(self.route)],
            input='xy 1 2\nspace\nxy 9 9\nspace\nx\nxy 3 4\nspace\nq\n',
            cwd='/tmp', env=env, capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(load_points(self.route)['points'], [[1., 2.], [3., 4.]])
        run = subprocess.run(['sh', str(recorder), '--demo', '--stdin', '--resume', str(self.route)],
            input='xy 5 6\nspace\nq\n', cwd='/tmp', env=env, capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(len(load_points(self.route)['points']), 3)
        run = subprocess.run(['sh', str(project_root()/'flight/drop_points.sh'), '--route', str(self.route),
            '--count', '1', '--show-config', '--json'], cwd='/tmp', env=env, capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(json.loads(run.stdout)['targets'][0]['xy'], [1., 2.])

    def test_terminal_displays_current_point_alignment(self):
        lines = []
        TerminalReporter(emit=lines.append).status(dict(output=dict(state='DROP',
            drop_points=dict(current_point=dict(drop_slot=2), visual_alignment=dict(stage=1,
                recorded_xy=[2., 3.], fallback=dict(remaining_s=12., retakes=0, retake_limit=1))))))
        self.assertIn('投放点 2', lines[1])
        self.assertIn('剩余 12.0s', lines[1])

    def test_default_execution_uses_common_recording_even_when_old_drop_file_exists(self):
        import hashlib
        root = Path(self.temp.name)
        common = root/DEFAULT_ROUTE
        new = document(1); new['points'] = [[1.7985, 1.3144]]
        old = document(1); old['points'] = [[-2.3041, .5544]]
        save_route(common, new)
        save_points(root/'data/routes/drop_points.yaml', old)
        with mock.patch.object(cli, 'project_root', return_value=root), contextlib.redirect_stdout(io.StringIO()) as output:
            cli.main(['--dry-run', '--json', '--flight-config', str(project_root()/'flight/flight.yaml')])
        value = json.loads(output.getvalue())
        self.assertEqual(value['targets'][0]['xy'], new['points'][0])
        self.assertEqual(value['route_source']['path'], str(common.resolve()))
        self.assertEqual(value['route_source']['sha256'], hashlib.sha256(common.read_bytes()).hexdigest())

    def test_recorders_share_file_format_and_count_selects_before_four_slot_validation(self):
        # A drop recording is also flyable by the ordinary entry.
        self.assertEqual(load_route(self.route)['points'], document()['points'])
        ordinary = document(); ordinary['points'] += [[5., 6.], [7., 8.]]
        save_route(self.route, ordinary)
        code, text = self.invoke('--count', '2', '--dry-run', '--json')
        self.assertEqual(code, 0)
        self.assertEqual([v['xy'] for v in json.loads(text)['targets']], ordinary['points'][:2])
        # Explicit legacy files still load, without becoming the default again.
        self.route.write_text('format: aquacore_drop_points\nversion: 1\nframe: fast_lio_local_enu\npoints: [[-1, 2]]\n')
        self.assertEqual(load_points(self.route)['points'], [[-1., 2.]])


if __name__ == '__main__':
    unittest.main()
