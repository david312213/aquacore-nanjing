"""The two handheld entries only display their requested observations."""
import json
from pathlib import Path
import subprocess
import unittest

from test_red_point_worker import image, odometry, SESSION, NS
from airseafly_real400_mission.red_point_worker import MissionObservations
from airseafly_real400_mission.red_point_runtime import load_config, DEFAULT_CONFIG
from airseafly_real400_mission.red_point_preview import select_preview_targets, ConsoleReporter

ROOT = Path(__file__).resolve().parents[3]


class HandheldTests(unittest.TestCase):
    def status(self):
        obs = MissionObservations(load_config(DEFAULT_CONFIG), SESSION)
        obs.inputs.on_odometry(odometry())
        return obs.process(image(), NS(imgmsg_to_cv2=lambda m, **kw: m.pixels), 'reused', 1.)[1]

    def test_green_shows_only_green_without_mutating_original(self):
        original = self.status()
        selected = select_preview_targets(original, 'green')
        self.assertFalse(selected['candidates'])
        self.assertTrue(original['candidates'])
        report = ConsoleReporter(1., True, 'green', True)
        text = report.observe(selected, 1.)
        self.assertIn('绿色估算圆心', text)
        self.assertIn('像素=', text)
        self.assertIn('飞机XYZ=', text)
        self.assertNotIn('红点', text)
        self.assertIsNone(report.observe(selected, 1.1))
        self.assertIsNone(report.observe(dict(selected, green_candidates=[]), 2.))

    def test_red_does_not_print_green_when_red_is_missing(self):
        selected = select_preview_targets(self.status(), 'red')
        self.assertFalse(selected['green_candidates'])
        reporter = ConsoleReporter(1., True, 'red', True)
        self.assertIn('红点', reporter.observe(selected, 1.))
        self.assertIsNone(reporter.observe(dict(selected, candidates=[]), 2.))

    def test_missing_pose_keeps_pixels(self):
        selected = select_preview_targets(self.status(), 'green')
        selected['pose'] = None
        selected['green_candidates'] = [dict(c, xy_m=None, offset_xy_m=None) for c in selected['green_candidates']]
        text = ConsoleReporter(1., True, 'green', True).observe(selected, 1.)
        self.assertIn('不可计算', text)
        self.assertIn('像素=', text)

    def test_entry_dry_runs_have_no_flight_or_payload_commands(self):
        for target in ('green', 'red'):
            result = subprocess.run(['sh', str(ROOT/'flight'/('test_align_'+target+'.sh')), '--dry-run'],
                                    capture_output=True, text=True, check=True)
            data = json.loads(result.stdout)
            self.assertEqual(data['target'], target)
            self.assertTrue(data['confirmed_only'])
            self.assertEqual(data['config']['console']['rate_hz'], 1.)
            self.assertFalse(data['flight_commands'])
            self.assertFalse(data['payload_commands'])
