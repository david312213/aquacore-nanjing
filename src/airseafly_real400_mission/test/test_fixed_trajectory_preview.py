"""User-facing route selection and offline preview exercise the actual solver."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from airseafly_real400_mission import waypoint_route_cli as cli
from airseafly_real400_mission.flight_configuration import apply_flight_configuration, base_configuration
from airseafly_real400_mission.fixed_trajectory_preview import route_preview, write_preview_html
from airseafly_real400_mission.waypoint_route import save_route


class FixedPreviewTests(unittest.TestCase):
    def setUp(self):
        self.document = dict(format='aquacore_waypoints', version=2, frame='fast_lio_local_enu',
            points=[[0.,0.],[2.,0.],[2.,2.],[0.,2.]],
            point_profiles=['cruise','cruise','gate','cruise'])
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        fixture = Path(folder.name)/'flight.yaml'
        fixture.write_text(json.dumps(dict(common={}, mission_profiles={'waypoint-route': dict(
            navigation=dict(backend='fixed_min_snap', laps=2),
            waypoint_profiles=dict(cruise=dict(height_m=1.), gate=dict(height_m=1.))) })))
        self.config = apply_flight_configuration(base_configuration(), path=fixture, task='waypoint-route')

    def test_default_fixed_laps_and_ego_rollback_have_distinct_finish_semantics(self):
        fixed = cli.preview(self.config, self.document)
        self.assertEqual(fixed['backend'], 'fixed_min_snap')
        self.assertEqual(fixed['laps'], 2)
        self.assertEqual(fixed['arrival']['policy'], 'trajectory_time')
        self.assertEqual(fixed['itinerary'][-1]['xy'], self.document['points'][0])
        self.config['navigation']['backend'] = 'ego'
        ego = cli.preview(self.config, self.document)
        self.assertEqual(ego['arrival']['policy'], 'position')
        self.assertEqual(ego['itinerary'][-1]['xy'], self.document['points'][-1])
        self.assertNotIn('fixed_trajectory', ego)

    def test_preview_origin_changes_only_preview_and_reports_actual_limits(self):
        self.config['waypoint_profiles']['gate']['height_m'] = 1.5
        before = json.dumps(self.config, sort_keys=True)
        value = route_preview(self.config, self.document, (5., -2., .3), True)
        self.assertFalse(value['preview_origin_assumed'])
        self.assertEqual(value['start_xyz'], [5., -2., 1.3])
        self.assertEqual(value['end_xyz'], [0., 0., 1.3])
        self.assertLessEqual(value['max_velocity_mps'], self.config['planning']['max_velocity_mps']+1e-8)
        self.assertLessEqual(value['max_acceleration_mps2'], self.config['planning']['max_acceleration_mps2']+1e-8)
        self.assertEqual(json.dumps(self.config, sort_keys=True), before)
        self.assertGreater(len(value['samples']), 200)

    def test_boundary_is_diagnostic_in_fixed_mode_even_when_file_missing(self):
        config = apply_flight_configuration(base_configuration(), task='waypoint-route', overrides=[
            'planning.virtual_boundary.enabled=true',
            'planning.virtual_boundary.file=/missing-fixed-preview-boundary.yaml'])
        self.assertFalse(config['_virtual_boundary']['enabled'])
        self.assertTrue(config['_virtual_boundary']['configured_enabled'])
        self.assertIn('diagnostic', config['_virtual_boundary'])
        config['_virtual_boundary']['corners'] = [[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]]
        value = route_preview(config, self.document)
        self.assertGreater(value['outside_boundary_xy_samples'], 0)
        self.assertGreater(value['sampled_max_outside_distance_m'], 2.)
        self.assertFalse(value['ego_boundary_effective'])

    def test_html_option_is_offline_without_dry_run_and_does_not_send_rpc(self):
        with tempfile.TemporaryDirectory() as folder:
            route = Path(folder)/'route.yaml'; output = Path(folder)/'preview.html'
            save_route(route, self.document)
            with patch.object(cli, 'manager_client', side_effect=AssertionError('no manager')), \
                    patch.object(cli, 'request', side_effect=AssertionError('no RPC')), \
                    contextlib.redirect_stdout(io.StringIO()) as stream:
                self.assertEqual(cli.main(['--route', str(route), '--preview-html', str(output), '--json']), 0)
            result = json.loads(stream.getvalue())
            self.assertEqual(result['preview_html'], str(output.resolve()))
            page = output.read_text()
            self.assertIn('type="range"', page)
            self.assertIn('不生效', page)
            self.assertNotIn('<script src=', page)
            self.assertNotIn('__DATA__', page)

    def test_single_point_and_delivery_scope(self):
        doc = dict(self.document, version=1, points=[[1.,2.]])
        doc.pop('point_profiles')
        value = cli.preview(self.config, doc)
        self.assertEqual(value['laps'], 0)
        # The contest entry flies fixed legs between stops, but never takes the
        # ordinary route's lap count.
        delivery = apply_flight_configuration(base_configuration(), task='delivery-route')
        self.assertEqual(delivery['navigation']['backend'], 'fixed_min_snap')
        self.assertNotIn('laps', delivery['navigation'])


if __name__ == '__main__':
    unittest.main()
