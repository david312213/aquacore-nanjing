"""Real-photo regressions and independent metric projection checks; no ROS."""
import math
from pathlib import Path
import sys
import unittest
from unittest import mock

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
for package in ('airseafly_real400_mission',):
    sys.path.insert(0, str(ROOT/'src'/package/'src'))
from airseafly_real400_mission.red_point import PoseHistory, RedPointSession, annotate, detect
from airseafly_real400_mission.red_point_preview import load_config
from airseafly_real400_mission.df200_geometry import CAMERA_MATRIX, DISTORTION


CONFIG = ROOT/'flight/red_point.yaml'
PHOTO = Path(__file__).parent/'fixtures/red_point_target.jpg'


def df200_config():
    """Flight config with the DF200 as the reference camera for the geometry expectations."""
    config = load_config(CONFIG)
    config['projection'].pop('intrinsics', None)  # down_projection falls back to df200_geometry
    return config
FRAME = 'fast_lio_local_enu'


def yaw_q(angle):
    return [0., 0., math.sin(angle/2), math.cos(angle/2)]


def candidate(pixel):
    return dict(class_name='red_point', pixel=list(pixel), bbox_xywh=[0, 0, 10, 10],
                area_px=70, circularity=1., solidity=1., score=.4,
                target_confirmed=True, confirmation_reason='confirmed')


def target(image, center, radius=120, white_offset=(0, 0), red_offset=(0, 0), white_colour=(230, 230, 230)):
    cv2.circle(image, center, radius, (0, 160, 0), -1)
    cv2.circle(image, tuple(np.asarray(center)+white_offset), radius//2, white_colour, -1)
    cv2.circle(image, tuple(np.asarray(center)+red_offset), max(3, radius//24), (0, 0, 230), -1)
    return image


class DetectorTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(CONFIG)
        self.image = cv2.imread(str(PHOTO))
        self.assertIsNotNone(self.image)

    def one(self, image, expected, tolerance=1.5, confirmed=True):
        detections, elapsed = detect(image, self.config)
        self.assertEqual(len(detections), 1, detections)
        np.testing.assert_allclose(detections[0]['pixel'], expected, atol=tolerance, rtol=0)
        self.assertEqual(detections[0]['class_name'], 'red_point')
        self.assertEqual(detections[0]['target_confirmed'], confirmed, detections[0])
        self.assertGreaterEqual(elapsed, 0.)
        return detections[0]

    def test_attached_photo_centroid_is_the_red_dot_not_ring(self):
        result = self.one(self.image, [458, 682])
        self.assertLess(result['center_error_px'], result['center_tolerance_px'])
        self.assertAlmostEqual(result['circles']['green_outer']['radius_px'], 413., delta=2.)
        self.assertAlmostEqual(result['circles']['white_inner']['radius_px'], 205., delta=2.)
        self.assertLess(abs(result['pixel'][1]-682), .2)

    def test_scaled_blurred_and_different_brightness(self):
        for scale in (1., .5, .25):
            for brightness in (.55, 1., 1.25):
                for blur in (0, 5):
                    with self.subTest(scale=scale, brightness=brightness, blur=blur):
                        image = cv2.resize(self.image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                        image = np.clip(image.astype(float)*brightness, 0, 255).astype(np.uint8)
                        if blur:
                            image = cv2.GaussianBlur(image, (blur, blur), 0)
                        self.one(image, np.asarray([458., 682.])*scale, tolerance=2.)

    def test_green_only_and_disappearance_publish_empty_current_results(self):
        without_red = self.image.copy()
        cv2.circle(without_red, (458, 682), 18, (230, 230, 230), -1)
        session = RedPointSession(self.config)
        for image, count in ((self.image, 1), (without_red, 0), (np.zeros_like(self.image), 0), (self.image, 1)):
            detections, elapsed = detect(image, self.config)
            status = session.describe(detections, image.shape[1::-1], 1., elapsed)
            self.assertEqual(len(status['candidates']), count)
            self.assertEqual(status['projection_status'], 'missing_odometry')

    def test_partial_outer_circle_can_confirm_but_missing_outer_cannot(self):
        result = self.one(self.image[:, 250:], [208, 682])
        self.assertLess(result['circles']['green_outer']['arc_degrees'], 300.)
        # Inner green hole is still a complete circle; it must never stand in
        # for the absent green outer edge.
        cropped = self.image[450:920, 235:685]
        result = self.one(cropped, [223, 232], confirmed=False)
        self.assertEqual(result['confirmation_reason'], 'missing_green_outer')

    def test_multiple_targets_and_unconfirmed_red_candidates_are_all_kept(self):
        image = np.full((720, 1280, 3), 35, np.uint8)
        target(image, (900, 350))
        target(image, (500, 350), radius=150)
        cv2.circle(image, (120, 360), 8, (0, 0, 230), -1)
        detections, _ = detect(image, self.config)
        self.assertEqual(len(detections), 3)
        confirmed = sorted(c['pixel'] for c in detections if c['target_confirmed'])
        np.testing.assert_allclose(confirmed, [[500, 350], [900, 350]], atol=.1)
        self.assertFalse(detections[-1]['target_confirmed'])
        self.assertEqual(detections[-1]['confirmation_reason'], 'missing_green_outer')

    def test_nonconcentric_red_or_white_and_missing_white_are_unconfirmed(self):
        for changes in (dict(red_offset=(12, 0)), dict(white_offset=(12, 0)),
                        dict(white_colour=(40, 40, 40))):
            with self.subTest(changes=changes):
                image = target(np.full((480, 640, 3), 30, np.uint8), (320, 240), **changes)
                result = self.one(image, np.asarray([320, 240])+changes.get('red_offset', (0, 0)),
                                  confirmed=False)
                self.assertEqual(result['confirmation_reason'],
                                 'missing_white_inner' if 'white_colour' in changes else 'not_concentric')

    def test_outer_white_sheet_and_small_white_patch_cannot_replace_center_disc(self):
        for white_radius in (0, 12):
            image = np.full((480, 640, 3), 30, np.uint8)
            cv2.circle(image, (320, 240), 200, (230, 230, 230), -1)
            cv2.circle(image, (320, 240), 120, (0, 160, 0), -1)
            if white_radius:
                cv2.circle(image, (320, 240), white_radius, (230, 230, 230), -1)
            cv2.circle(image, (320, 240), 5, (0, 0, 230), -1)
            result = self.one(image, [320, 240], confirmed=False)
            self.assertEqual(result['confirmation_reason'], 'invalid_nesting')

    def test_partial_and_irregular_arcs_do_not_become_an_invented_outer_circle(self):
        image = np.full((720, 1280, 3), 230, np.uint8)
        cv2.ellipse(image, (850, 360), (180, 180), 0, 0, 170, (0, 150, 0), 55)
        cv2.circle(image, (850, 360), 8, (0, 0, 230), -1)
        self.one(image, [850, 360], confirmed=False)
        # A frame full of green containing a white hole is not an outer circle.
        image[:] = (0, 160, 0)
        cv2.circle(image, (850, 360), 90, (230, 230, 230), -1)
        cv2.circle(image, (850, 360), 8, (0, 0, 230), -1)
        self.one(image, [850, 360], confirmed=False)

    def test_mild_foreshortening_keeps_the_red_centroid(self):
        image = target(np.full((480, 640, 3), 30, np.uint8), (320, 240))
        for angle in (0., 28.):
            matrix = cv2.getRotationMatrix2D((320, 240), angle, 1.)
            matrix[1, :] *= .85
            warped = cv2.warpAffine(image, matrix, (640, 480))
            self.one(warped, [320, 204], tolerance=.5)

    def test_red_centroid_not_bbox_centre_and_hue_wrap(self):
        image = np.full((200, 300, 3), 220, np.uint8)
        hsv_colour = np.uint8([[[179, 230, 230]]])
        colour = cv2.cvtColor(hsv_colour, cv2.COLOR_HSV2BGR)[0, 0].tolist()
        cv2.circle(image, (150, 100), 20, colour, -1)
        cv2.rectangle(image, (157, 102), (170, 120), (220, 220, 220), -1)
        mask = np.any(image != (220, 220, 220), axis=2)
        y, x = np.nonzero(mask)
        result = self.one(image, [np.mean(x), np.mean(y)], .01, confirmed=False)
        self.assertGreater(np.linalg.norm(np.asarray(result['pixel'])-[150, 100]), 1.)

    def test_annotation_does_not_modify_pixels_or_invent_metric_photo_data(self):
        original = self.image.copy()
        detections, elapsed = detect(self.image, self.config)
        status = RedPointSession(self.config).describe(detections, (960, 1280), 0, elapsed, offline=True)
        canvas = annotate(self.image, status)
        np.testing.assert_array_equal(self.image, original)
        self.assertEqual(canvas.shape, self.image.shape)
        self.assertIsNone(status['camera_aim'])
        self.assertIsNone(status['ground_reference'])
        self.assertIsNone(status['candidates'][0]['xy_m'])
        self.assertIsNone(status['candidates'][0]['offset_xy_m'])
        self.assertEqual(status['projection_status'], 'offline_pixels_only')

    def test_overlay_only_shows_detection_status_with_fixed_frame_size(self):
        detections, elapsed = detect(self.image, self.config)
        session = RedPointSession(self.config)
        unconfirmed = [dict(detections[0], target_confirmed=False)]
        for points, error, label in ((detections, '', 'TARGET CONFIRMED'),
                                     (unconfirmed, '', 'UNCONFIRMED'),
                                     ([], '', 'NO RED POINT'), ([], 'bad image', 'IMAGE ERROR')):
            status = session.describe(points, self.image.shape[1::-1], 0., elapsed, offline=True)
            status['image_error'] = error
            with mock.patch.object(cv2, 'putText', wraps=cv2.putText) as text, \
                    mock.patch.object(cv2, 'drawMarker', wraps=cv2.drawMarker) as marker:
                canvas = annotate(self.image, status)
            self.assertEqual(canvas.shape, self.image.shape)
            self.assertEqual([call.args[1] for call in text.call_args_list],
                             [label]+(['RED CENTER'] if label == 'TARGET CONFIRMED' else []))
            self.assertEqual(marker.call_count, int(label == 'TARGET CONFIRMED'))

    def test_changing_circle_settings_never_changes_full_resolution_red_centroids(self):
        image = self.image.copy()
        cv2.circle(image, (90, 120), 7, (0, 0, 220), -1)
        baseline = None
        confirmations = []
        for tolerance in (.005, .05, .2):
            config = dict(self.config, detector=dict(self.config['detector'],
                                                     concentric_tolerance_ratio=tolerance))
            results, _ = detect(image, config)
            confirmations.append(sum(c['target_confirmed'] for c in results))
            red_geometry = sorted((r['pixel'], r['bbox_xywh'], r['area_px']) for r in results)
            if baseline is None:
                baseline = red_geometry
                self.assertEqual(len(baseline), 2)
            else:
                self.assertEqual(red_geometry, baseline)
        self.assertEqual(confirmations, [0, 1, 1])


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.session = RedPointSession(df200_config())
        self.session.observe_pose(1., [2., -3., 0.], yaw_q(0), FRAME)

    def report(self, pixel, size=(1280, 720), stamp=2.):
        return self.session.describe([candidate(pixel)], size, stamp, 0.)

    def test_only_current_confirmed_candidates_get_xy_and_never_reuse_previous_xy(self):
        self.session.observe_pose(2., [2., -3., 1.], yaw_q(0), FRAME)
        confirmed = candidate([600., 360.])
        rejected = dict(confirmed, target_confirmed=False, confirmation_reason='not_concentric')
        for points, expected in (([confirmed], 1), ([rejected], 0), ([], 0),
                                 ([confirmed, rejected], 1), ([confirmed], 1)):
            status = self.session.describe(points, (1280, 720), 2., 0.)
            self.assertEqual(status['confirmed_count'], expected)
            self.assertEqual(sum(c['xy_m'] is not None for c in status['candidates']), expected)
            for item in status['candidates']:
                if not item['target_confirmed']:
                    self.assertEqual(item['projection_status'], 'target_unconfirmed')
                    self.assertIsNone(item['xy_m'])
                    self.assertIsNone(item['offset_xy_m'])
        unknown = dict(confirmed)
        del unknown['target_confirmed']
        self.assertIsNone(self.session.describe([unknown], (1280, 720), 2., 0.)['candidates'][0]['xy_m'])

    def test_principal_point_and_fixed_localization_origin(self):
        principal = [CAMERA_MATRIX[0][2], CAMERA_MATRIX[1][2]]
        self.session.observe_pose(2., [7., 8., 1.], yaw_q(.8), FRAME)
        status = self.report(principal)
        np.testing.assert_allclose(status['candidates'][0]['xy_m'], [7, 8], atol=1e-10)
        np.testing.assert_allclose(status['candidates'][0]['offset_xy_m'], [0, 0], atol=1e-10)
        self.assertAlmostEqual(status['camera_height_m'], 1.015)
        self.session.observe_pose(3., [8., 9., 2.], yaw_q(0), FRAME)
        status = self.report(principal, stamp=3.)
        self.assertAlmostEqual(status['camera_height_m'], 2.015)
        self.assertEqual(status['ground_reference']['z_m'], 0.)
        self.assertEqual(status['ground_reference']['source'], 'localization_origin')

    def test_starting_or_restarting_preview_aloft_never_rebases_height(self):
        original = None
        for _ in range(2):
            session = RedPointSession(self.session.config)
            session.observe_pose(10., [7., 8., 1.], yaw_q(.4), FRAME)
            # First latched transform may arrive after the first pose.
            session.observe_frame(('same_localization_session', 1.))
            status = session.describe([candidate([650., 400.])], (1280, 720), 10., 0.)
            self.assertAlmostEqual(status['camera_height_m'], 1.015)
            self.assertEqual(status['projection_status'], 'ok_near_level')
            if original is None:
                original = status['candidates'][0]['xy_m']
            else:
                np.testing.assert_allclose(status['candidates'][0]['xy_m'], original)

    def test_fixed_target_stays_fixed_as_vehicle_moves_and_changes_height(self):
        target = np.asarray([2., -1.3])
        for stamp, (x, y, z, yaw) in enumerate(((1.8, -1.1, .75, 0.),
                (2.1, -1.5, 1.2, math.pi/2), (2.3, -1.3, 1.5, -.3)), 2):
            dx, dy = target-[x, y]
            forward = math.cos(yaw)*dx+math.sin(yaw)*dy
            left = -math.sin(yaw)*dx+math.cos(yaw)*dy
            optical = np.asarray([[[-left, -forward, z+.015]]])
            pixel, _ = cv2.projectPoints(optical, np.zeros(3), np.zeros(3),
                                        np.asarray(CAMERA_MATRIX), np.asarray(DISTORTION))
            self.session.observe_pose(float(stamp), [x, y, z], yaw_q(yaw), FRAME)
            result = self.report(pixel.reshape(2), stamp=float(stamp))
            np.testing.assert_allclose(result['candidates'][0]['xy_m'], target, atol=1e-6)
            np.testing.assert_allclose(result['candidates'][0]['offset_xy_m'], [dx, dy], atol=1e-6)

    def test_distorted_four_directions_heights_yaw_and_resolution(self):
        # Forward projection via OpenCV is independent of our inverse formula.
        for width, height in ((1280, 720), (640, 360), (2560, 1440)):
            K = np.diag([width/1280., height/720., 1.]) @ np.asarray(CAMERA_MATRIX)
            for altitude in (.25, 1., 2.):
                for yaw in (0., math.pi/2):
                    self.session.observe_pose(2., [2., -3., altitude-.015], yaw_q(yaw), FRAME)
                    for ox, oy in ((.1, 0.), (-.1, 0.), (0., .1), (0., -.1)):
                        with self.subTest(size=(width, height), altitude=altitude, yaw=yaw, optical=(ox, oy)):
                            px, _ = cv2.projectPoints(np.asarray([[[ox, oy, altitude]]]),
                                np.zeros(3), np.zeros(3), K, np.asarray(DISTORTION))
                            status = self.report(px.reshape(2), (width, height))
                            forward, left = -oy, -ox
                            expected = [2+math.cos(yaw)*forward-math.sin(yaw)*left,
                                        -3+math.sin(yaw)*forward+math.cos(yaw)*left]
                            np.testing.assert_allclose(status['candidates'][0]['xy_m'], expected, atol=1e-6)

    def test_image_time_interpolation_and_unlimited_nearest_fallback(self):
        self.session.observe_pose(3., [4., -1., 2.], yaw_q(math.pi/2), FRAME)
        status = self.report([600, 360], stamp=2.)
        pose = status['pose']
        np.testing.assert_allclose(pose['position'], [3., -2., 1.])
        self.assertAlmostEqual(status['camera_height_m'], 1.015)
        self.assertAlmostEqual(pose['rpy_deg'][2], 45.)
        self.assertEqual(pose['method'], 'interpolated')
        self.assertEqual(pose['source_stamps_s'], [1., 3.])
        self.assertEqual(pose['dt_s'], 0.)
        for image_time, expected_dt in ((-50., 51.), (200., -197.)):
            result = self.report([600, 360], stamp=image_time)
            self.assertEqual(result['pose']['method'], 'nearest')
            self.assertEqual(result['pose']['dt_s'], expected_dt)
            self.assertIsNotNone(result['candidates'][0]['xy_m'])

    def test_quaternion_wrap_and_pose_history_capacity_are_not_age_limits(self):
        h = PoseHistory(2)
        h.add(1., [0, 0, 0], yaw_q(math.radians(179)), FRAME)
        h.add(3., [0, 0, 0], yaw_q(math.radians(-179)), FRAME)
        self.assertAlmostEqual(abs(h.at(2.)['rpy_deg'][2]), 180.)
        self.assertEqual(h.at(1.)['method'], 'exact')
        h.add(100., [1, 0, 0], yaw_q(0), FRAME)
        self.assertEqual(len(h.samples), 2)
        self.assertEqual(h.at(-100.)['stamp_s'], 3.)

    def test_missing_pose_keeps_pixels_and_receiving_pose_enables_xy_automatically(self):
        new = RedPointSession(self.session.config)
        status = new.describe([candidate([1, 2])], (1280, 720), 1, 0)
        self.assertEqual(status['projection_status'], 'missing_odometry')
        self.assertEqual(status['candidates'][0]['pixel'], [1, 2])
        self.assertIsNone(status['candidates'][0]['xy_m'])
        self.assertIsNone(status['candidates'][0]['offset_xy_m'])
        new.observe_pose(1., [0, 0, 0], yaw_q(0), FRAME)
        result = new.describe([candidate([600, 360])], (1280, 720), 1, 0)
        self.assertEqual(result['projection_status'], 'ok_near_level')
        self.assertAlmostEqual(result['camera_height_m'], .015)
        self.assertIsNotNone(result['candidates'][0]['xy_m'])

    def test_localization_restart_clears_old_history_and_automatically_uses_new_zero(self):
        self.session.observe_frame(('first_frame', 1.))
        self.session.observe_pose(2., [0, 0, 3], yaw_q(0), FRAME)
        self.session.observe_frame(('first_frame', 1.))
        self.assertAlmostEqual(self.report([600, 360])['camera_height_m'], 3.015)
        self.session.observe_frame(('second_frame', 20.))
        self.assertEqual(self.session.history.samples, [])
        self.assertEqual(self.report([600, 360])['projection_status'], 'missing_odometry')
        self.session.observe_pose(20., [0, 0, .7], yaw_q(0), FRAME)
        result = self.report([600, 360], stamp=20.)
        self.assertAlmostEqual(result['camera_height_m'], .715)
        self.assertEqual(result['ground_reference'], dict(z_m=0., source='localization_origin', epoch=1))
        self.session.observe_pose(1., [0, 0, .25], yaw_q(0), FRAME)
        result = self.report([600, 360], stamp=1.)
        self.assertAlmostEqual(result['camera_height_m'], .265)
        self.assertEqual(result['ground_reference'], dict(z_m=0., source='localization_origin', epoch=2))
        self.assertEqual(len(self.session.history.samples), 1)
        self.assertEqual(self.session.reference_event, 'odometry_time_restarted')

    def test_wrong_frame_and_nonpositive_height_are_explicit_not_fake_xy(self):
        self.session.observe_pose(2., [0, 0, -.1], yaw_q(0), FRAME)
        self.assertEqual(self.report([600, 360])['projection_status'], 'nonpositive_camera_height')
        self.session.observe_pose(3., [0, 0, 0], yaw_q(0), 'other_frame')
        status = self.report([600, 360], stamp=3.)
        self.assertEqual(status['projection_status'], 'frame_mismatch')
        self.assertEqual(len(status['candidates']), 1)
        self.assertIsNone(status['candidates'][0]['xy_m'])

    def test_roll_and_pitch_are_reported_without_an_attitude_gate(self):
        roll, pitch = math.radians(30), math.radians(15)
        q = [math.sin(roll/2)*math.cos(pitch/2), math.cos(roll/2)*math.sin(pitch/2),
             -math.sin(roll/2)*math.sin(pitch/2), math.cos(roll/2)*math.cos(pitch/2)]
        self.session.observe_pose(2., [2., -3., 5.], q, FRAME)
        result = self.report([CAMERA_MATRIX[0][2], CAMERA_MATRIX[1][2]])
        np.testing.assert_allclose(result['pose']['rpy_deg'], [30, 15, 0], atol=1e-8)
        self.assertEqual(result['projection_status'], 'ok_near_level')
        # Deliberately yaw-only, never silently claim full-attitude projection.
        np.testing.assert_allclose(result['candidates'][0]['xy_m'], [2, -3], atol=1e-8)


if __name__ == '__main__':
    unittest.main()
