"""Independent color centers and the approved intermediate-descent behavior."""
import unittest

import cv2
import numpy as np

from test_red_point import CONFIG, target
from test_delivery_alignment import descending, waiting, deadline, releases
from airseafly_real400_mission.green_circle import green_candidates
from airseafly_real400_mission.red_point import detect_targets, RedPointSession
from airseafly_real400_mission.red_point_runtime import load_config


class GreenDetectorTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(CONFIG)

    def test_green_ring_without_red_or_white_has_hough_center(self):
        image = np.full((480, 640, 3), 30, np.uint8)
        cv2.circle(image, (320, 240), 110, (0, 170, 0), -1)
        cv2.circle(image, (320, 240), 55, (30, 30, 30), -1)
        candidates, ms = detect_targets(image, self.config)
        status = RedPointSession(self.config).describe(candidates, (640, 480), 1, ms, offline=True)
        self.assertFalse(status['candidates'])
        self.assertEqual(len(status['green_candidates']), 1)
        np.testing.assert_allclose(status['green_candidates'][0]['pixel'], (320, 240), atol=4)

    def test_solid_green_and_rectangle_and_empty_image_are_not_rings(self):
        for shape in ('disk', 'rectangle', 'empty'):
            image = np.zeros((480, 640, 3), np.uint8)
            if shape == 'disk': cv2.circle(image, (320, 240), 100, (0, 170, 0), -1)
            if shape == 'rectangle': cv2.rectangle(image, (100, 100), (500, 350), (0, 170, 0), -1)
            self.assertFalse(green_candidates(image, self.config['detector']), shape)

    def test_multiple_and_partially_cropped_rings(self):
        image = np.zeros((480, 800, 3), np.uint8)
        for center in ((65, 240), (570, 240)):
            target(image, center, 110)
        found = green_candidates(image, self.config['detector'])
        self.assertEqual(len(found), 2)
        for expected in ((65, 240), (570, 240)):
            self.assertLess(min(np.linalg.norm(np.asarray(c['pixel'])-expected) for c in found), 6)

    def test_projection_preserves_red_packet_and_adds_green_separately(self):
        image = target(np.zeros((480, 640, 3), np.uint8), (320, 240), 110)
        candidates, ms = detect_targets(image, self.config)
        session = RedPointSession(self.config)
        session.observe_pose(1., [2., 3., 1.], [0., 0., 0., 1.], 'fast_lio_local_enu')
        status = session.describe(candidates, (640, 480), 1., ms)
        self.assertEqual(len(status['candidates']), 1)
        self.assertEqual(status['confirmed_count'], 1)
        self.assertEqual(len(status['green_candidates']), 1)
        for c in status['candidates']+status['green_candidates']:
            self.assertEqual(c['projection_status'], 'ok_near_level')
            self.assertEqual(len(c['xy_m']), 2)
        self.assertEqual(status['candidates'][0]['class_name'], 'red_point')


class IntermediateDescentTests(unittest.TestCase):
    def packet(self, h, **values):
        h.image_sequence += 1
        return dict(schema='airseafly.red_point.v1', session_id='offline-red-point',
            image_sequence=h.image_sequence, frame_id='fast_lio_local_enu', **values)

    def test_drop_ignores_green_and_aligns_once_on_red(self):
        h = waiting(); e = h.engine
        red = dict(class_name='red_point', target_confirmed=True,
                   projection_status='ok_near_level', xy_m=[3.1, 2.1])
        green = dict(red, class_name='green_circle')
        e.on_red_point_observation(self.packet(h, green_candidates=[green], candidates=[green]))
        self.assertEqual(e.phase, 'NATIVE_SECOND_FRAME')
        e.on_red_point_observation(self.packet(h, candidates=[red]))
        self.assertEqual(e.phase, 'NATIVE_SECOND_MOVE')
        self.assertEqual(e.native_goal, (3.1, 2.1, .5))

    def test_intermediate_deadline_keeps_actual_xyz_and_starts_frame_wait(self):
        h = descending(); e = h.engine
        self.assertEqual(e.native_goal, (3., 2., .5))
        actual = (3.6, 2.4, .82)
        h.position = actual
        self.assertFalse(h.frame((9., 9.)))
        deadline(h, before=.01)
        self.assertEqual(e.phase, 'NATIVE_INTERMEDIATE_DESCEND')
        deadline(h)
        self.assertEqual(e.phase, 'NATIVE_SECOND_FRAME')
        self.assertEqual(e.native_goal, actual)
        self.assertTrue(e._intermediate_descent_timed_out)
        self.assertFalse(e._delivery_fallback_active)
        self.assertEqual(e._delivery_retakes, 0)
        self.assertAlmostEqual(e._alignment_deadline()-h.now, 15.)
        self.assertFalse(releases(h))
        h.frame((3.25, 2.15))
        self.assertEqual(e.native_goal, (3.25, 2.15, .82))

    def test_intermediate_arrival_at_deadline_wins_and_retake_does_not_descend_again(self):
        h = descending(); e = h.engine
        h.position = e.native_goal; deadline(h)
        self.assertEqual(e.phase, 'NATIVE_SECOND_FRAME')
        self.assertFalse(e._intermediate_descent_timed_out)
        h.frame((4., 3.)); deadline(h)
        self.assertEqual(e.phase, 'NATIVE_SECOND_FRAME')
        h.frame((3.5, 2.5)); h.until(lambda e: e.phase == 'NATIVE_DESCEND')
        phases = [x['phase'] for x in h.effects if x['type'] == 'native_target']
        self.assertEqual(phases.count('NATIVE_INTERMEDIATE_DESCEND'), 1)
        self.assertFalse(releases(h))

    def test_intermediate_timeout_then_red_timeout_uses_recorded_xy(self):
        h = descending(); e = h.engine
        h.position = (4., 3., .8); deadline(h); deadline(h)
        self.assertEqual(e.phase, 'NATIVE_DESCEND')
        self.assertEqual(e.native_goal[:2], (3., 2.))
        self.assertFalse(releases(h))

if __name__ == '__main__':
    unittest.main()
