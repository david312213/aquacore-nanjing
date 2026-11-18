import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import calibrate  # noqa: E402

COLS, ROWS, SQUARE = 11, 8, 0.03
TRUE_K = np.array([[400., 0., 320.], [0., 400., 240.], [0., 0., 1.]])


def flat_board(px=20, margin=40):
    """Checkerboard image and the map from board metres to its pixels."""
    squares = np.indices((ROWS + 1, COLS + 1)).sum(axis=0) % 2
    board = np.kron(squares, np.ones((px, px))) * 255
    board = np.pad(board, margin, constant_values=255).astype(np.uint8)
    scale = px / SQUARE
    to_pixels = np.array([[scale, 0., margin + px], [0., scale, margin + px], [0., 0., 1.]])
    return board, to_pixels


def render(rvec, tvec):
    board, to_pixels = flat_board()
    plane = calibrate.board_points(COLS, ROWS, SQUARE)
    image_pts, _ = cv2.projectPoints(plane, np.float64(rvec), np.float64(tvec), TRUE_K, None)
    homography, _ = cv2.findHomography(plane[:, :2], image_pts.reshape(-1, 2))
    warp = homography @ np.linalg.inv(to_pixels)
    return cv2.warpPerspective(board, warp, (640, 480), borderValue=255)


class CalibrateTest(unittest.TestCase):
    def test_recovers_focal_length_from_synthetic_views(self):
        views = []
        for rx in (-0.3, 0., 0.3):
            for ry in (-0.3, 0.3):
                for tx in (-0.08, 0.):
                    corners = calibrate.find_corners(render([rx, ry, 0.05], [tx, -0.1, 0.55]), COLS, ROWS)
                    if corners is not None:
                        views.append(corners)
        self.assertGreaterEqual(len(views), 10)
        result = calibrate.calibrate(views, (640, 480), COLS, ROWS, SQUARE)
        self.assertLess(result['rms'], 1.0)
        matrix = result['matrix']
        np.testing.assert_allclose([matrix[0, 0], matrix[1, 1]], [400., 400.], rtol=0.03)
        self.assertEqual(len(result['per_view']), len(views))
        self.assertGreater(result['sd'][0], 0)

    def test_tilt_direction_names_the_nearer_side(self):
        pose = lambda r: calibrate.find_corners(render(r, [-0.15, -0.1, 0.55]), COLS, ROWS)
        self.assertIsNone(calibrate.tilt_direction(pose([0., 0., 0.]), COLS))
        vertical = {calibrate.tilt_direction(pose([s * 0.45, 0., 0.]), COLS) for s in (1, -1)}
        horizontal = {calibrate.tilt_direction(pose([0., s * 0.45, 0.]), COLS) for s in (1, -1)}
        self.assertEqual(vertical, {'上边近', '下边近'})
        self.assertEqual(horizontal, {'左边近', '右边近'})

    def test_corner_motion_ignores_point_order(self):
        corners = calibrate.find_corners(render([0., 0., 0.], [-0.15, -0.1, 0.55]), COLS, ROWS)
        self.assertIsNone(calibrate.corner_motion(None, corners))
        self.assertAlmostEqual(calibrate.corner_motion(corners, corners[::-1]), 0.)
        self.assertAlmostEqual(calibrate.corner_motion(corners, corners + 2.), 2. * np.sqrt(2), places=4)

    def test_missing_reports_unvisited_edges_and_tilts(self):
        centred = calibrate.find_corners(render([0., 0., 0.], [-0.15, -0.1, 0.55]), COLS, ROWS)
        regions, tilts, counts = calibrate.missing([centred] * 3, (640, 480), COLS)
        self.assertEqual(set(regions), set(calibrate.REGIONS))
        self.assertEqual(set(tilts), set(calibrate.TILTS))
        self.assertEqual(sum(counts.values()), 0)

    def test_duplicate_pose_is_not_kept_twice(self):
        corners = calibrate.find_corners(render([0.2, 0., 0.], [0., -0.1, 0.55]), COLS, ROWS)
        signature = calibrate.view_signature(corners, (640, 480), COLS, ROWS)
        self.assertFalse(calibrate.is_new_view(signature, [signature]))
        moved = calibrate.find_corners(render([0.2, 0., 0.], [0.1, -0.1, 0.55]), COLS, ROWS)
        self.assertTrue(calibrate.is_new_view(
            calibrate.view_signature(moved, (640, 480), COLS, ROWS), [signature]))

    def test_camera_info_loads_in_red_point_runtime(self):
        root = Path(calibrate.PROJECT)
        sys.path.insert(0, str(root / 'src/airseafly_real400_mission/src'))
        from airseafly_real400_mission.red_point_runtime import load_calibration
        info = calibrate.camera_info(TRUE_K, np.zeros(5), (640, 480), 'down_test')
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'camera_info.yaml'
            path.write_text(yaml.safe_dump(info), encoding='utf-8')
            loaded = load_calibration(path)
        self.assertEqual(loaded['calibration_size'], [640., 480.])
        self.assertEqual(loaded['camera_name'], 'down_test')
        np.testing.assert_allclose(loaded['camera_matrix'], TRUE_K)


if __name__ == '__main__':
    unittest.main()
