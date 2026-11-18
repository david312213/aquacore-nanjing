#!/usr/bin/env python3
"""Down-camera intrinsic calibration with a checkerboard; no ROS, no flight.

Live: reads the camera directly (stop any usb_cam using it first), keeps frames whose
board pose differs from those already kept, then writes a ROS camera_info YAML that
flight/red_point.yaml camera.calibration can point to. --images re-runs on saved frames.
"""
import argparse
import glob
import os
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

PROJECT = Path(__file__).resolve().parents[3]
DEVICE = '/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0'
FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)


def board_points(cols, rows, square_m):
    grid = np.zeros((rows * cols, 3), np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    return grid


def find_corners(gray, cols, rows):
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), FLAGS)
    if not found:
        return None
    return cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), CRITERIA)


def view_signature(corners, size, cols, rows):
    """Normalized centre, apparent size and two tilt ratios of the board in the image."""
    pts = corners.reshape(-1, 2)
    width, height = size
    centre = pts.mean(axis=0) / (width, height)
    span = np.ptp(pts, axis=0) / (width, height)
    tl, tr, bl, br = pts[0], pts[cols - 1], pts[-cols], pts[-1]
    tilt_x = np.linalg.norm(tr - tl) / max(1e-6, np.linalg.norm(br - bl))
    tilt_y = np.linalg.norm(bl - tl) / max(1e-6, np.linalg.norm(br - tr))
    return np.array([centre[0], centre[1], float(np.sqrt(span[0] * span[1])),
                     np.log(tilt_x), np.log(tilt_y)])


def is_new_view(signature, kept, min_distance=0.08):
    return all(np.linalg.norm(signature - other) >= min_distance for other in kept)


# Outer bands of the frame where lens distortion is largest and only edge views constrain it.
REGIONS = dict(左上=((0, .2), (0, .2)), 右上=((.8, 1), (0, .2)),
               左下=((0, .2), (.8, 1)), 右下=((.8, 1), (.8, 1)),
               左边=((0, .1), (.2, .8)), 右边=((.9, 1), (.2, .8)),
               上边=((.2, .8), (0, .1)), 下边=((.2, .8), (.9, 1)))
TILT_LOG = 0.15  # ~20 deg at 0.6 m; only for hints
TILTS = ('上边近', '下边近', '左边近', '右边近')


def regions_hit(corners, size):
    pts = corners.reshape(-1, 2) / size
    return {name for name, ((x0, x1), (y0, y1)) in REGIONS.items()
            if np.any((pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (pts[:, 1] >= y0) & (pts[:, 1] <= y1))}


def tilt_direction(corners, cols):
    """Image side of the board nearer the camera (it looks longer), or None if near-flat."""
    pts = corners.reshape(-1, 2)
    c0, c1, c2, c3 = pts[0], pts[cols - 1], pts[-cols], pts[-1]
    sides = [(c0, c1), (c1, c3), (c3, c2), (c2, c0)]  # outer grid lines; curved edges don't matter
    centre = pts.mean(axis=0)
    def side(pick):
        return max(sides, key=lambda s: pick((s[0] + s[1]) / 2 - centre))
    length = lambda s: np.linalg.norm(s[0] - s[1])
    up, down = side(lambda d: -d[1]), side(lambda d: d[1])
    left, right = side(lambda d: -d[0]), side(lambda d: d[0])
    vertical = np.log(length(up) / max(1e-6, length(down)))
    horizontal = np.log(length(left) / max(1e-6, length(right)))
    if max(abs(vertical), abs(horizontal)) < TILT_LOG:
        return None
    if abs(vertical) >= abs(horizontal):
        return '上边近' if vertical > 0 else '下边近'
    return '左边近' if horizontal > 0 else '右边近'


def corner_motion(previous, corners):
    """Mean pixel shift of corners between consecutive detections (either point order)."""
    if previous is None or previous.shape != corners.shape:
        return None
    a, b = previous.reshape(-1, 2), corners.reshape(-1, 2)
    return float(min(np.linalg.norm(a - b, axis=1).mean(), np.linalg.norm(a - b[::-1], axis=1).mean()))


def missing(views, size, cols):
    hits = {name: 0 for name in REGIONS}
    tilts = {name: 0 for name in TILTS}
    for corners in views:
        for name in regions_hit(corners, size):
            hits[name] += 1
        direction = tilt_direction(corners, cols)
        if direction:
            tilts[direction] += 1
    return [n for n, c in hits.items() if c < 2], [n for n, c in tilts.items() if c < 2], tilts


def calibrate(views, size, cols, rows, square_m):
    objects = [board_points(cols, rows, square_m)] * len(views)
    rms, matrix, distortion, _, _, sd, _, per_view = cv2.calibrateCameraExtended(
        objects, views, tuple(size), None, None)
    return dict(rms=rms, matrix=matrix, distortion=distortion.reshape(-1)[:5],
                sd=sd.reshape(-1), per_view=per_view.reshape(-1))


def camera_info(matrix, distortion, size, name):
    width, height = size
    projection, _ = cv2.getOptimalNewCameraMatrix(matrix, distortion, (width, height), 0)
    projection = np.hstack([projection, np.zeros((3, 1))])
    flat = lambda array: [round(float(v), 6) for v in np.asarray(array).reshape(-1)]
    return dict(image_width=int(width), image_height=int(height), camera_name=name,
                camera_matrix=dict(rows=3, cols=3, data=flat(matrix)),
                distortion_model='plumb_bob',
                distortion_coefficients=dict(rows=1, cols=5, data=flat(distortion)),
                rectification_matrix=dict(rows=3, cols=3, data=flat(np.eye(3))),
                projection_matrix=dict(rows=3, cols=4, data=flat(projection)))


TILT_LABELS = {'上边近': 'top', '下边近': 'bottom', '左边近': 'left', '右边近': 'right'}  # putText has no CJK


def draw_preview(frame, corners, found, state, views, size, cols, target, elapsed, timeout):
    """Camera view with detected corners, edge-region coverage and save status."""
    view = frame.copy()
    width, height = size
    hits = {name: 0 for name in REGIONS}
    for saved in views:
        for name in regions_hit(saved, size):
            hits[name] += 1
        x, y = saved.reshape(-1, 2).mean(axis=0)
        cv2.circle(view, (int(x), int(y)), 6, (255, 200, 0), -1)
    for name, ((x0, x1), (y0, y1)) in REGIONS.items():
        colour = (0, 200, 0) if hits[name] >= 2 else (0, 0, 255)
        cv2.rectangle(view, (int(x0 * width), int(y0 * height)),
                      (int(x1 * width) - 1, int(y1 * height) - 1), colour, 2)
    if corners is not None:
        cv2.drawChessboardCorners(view, (cols, len(corners) // cols), corners, found)
    _, _, tilts = missing(views, size, cols)
    lines = ['saved %d/%d   %.0f/%.0f s   q = finish' % (len(views), target, elapsed, timeout),
             state,
             'tilt  ' + '  '.join('%s %d' % (TILT_LABELS[k], v) for k, v in tilts.items())]
    for i, text in enumerate(lines):
        y = 36 + 34 * i
        cv2.putText(view, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(view, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
    return view


def capture_live(args, out_dir):
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # detection is slower than 60 fps; show the newest frame
    if not cap.isOpened():
        raise SystemExit('cannot open %s (is usb_cam still running?)' % args.device)
    show = args.show and bool(os.environ.get('DISPLAY'))
    if args.show and not show:
        print('没有 DISPLAY，不显示预览窗口；在机载桌面终端运行可看到画面。', flush=True)
    if show:
        cv2.namedWindow('calibrate', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('calibrate', 1280, 720)
    views, kept, size, started = [], [], None, time.time()
    previous, detected, last_note = None, 0, started
    print('移动后停稳约 1 秒再换下一个姿态，只保存静止且姿态不同的帧。目标 %d 张，最长 %d s。'
          % (args.frames, args.timeout), flush=True)
    if show:
        print('窗口：彩色角点=检测到棋盘，红框=该边角区域还不够 2 张，绿框=已够，蓝点=已存位置；按 q 提前结束并标定。',
              flush=True)
    while len(views) < args.frames and time.time() - started < args.timeout:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])
        found, raw = cv2.findChessboardCorners(gray, (args.cols, args.rows), FLAGS)
        corners = cv2.cornerSubPix(gray, raw, (5, 5), (-1, -1), CRITERIA) if found else None
        if time.time() - last_note >= 5:
            last_note = time.time()
            regions, tilts, _ = missing(views, size, args.cols)
            print('  … %.0f s，已存 %d 张；还缺 区域：%s  倾斜：%s' % (
                last_note - started, len(views), ' '.join(regions) or '无', ' '.join(tilts) or '无'), flush=True)
        state = 'no board: all %dx%d inner corners must be in view' % (args.cols, args.rows)
        if corners is None:
            previous = None
        else:
            detected += 1
            motion, previous = corner_motion(previous, corners), corners
            signature = view_signature(corners, size, args.cols, args.rows)
            if motion is None or motion > args.steady_px:
                state = 'board found, hold still (moved %s px, need <= %.1f)' % (
                    '-' if motion is None else '%.1f' % motion, args.steady_px)
            elif not is_new_view(signature, kept):
                state = 'same pose as a saved one: move or tilt the board'
            else:
                kept.append(signature)
                views.append(corners)
                cv2.imwrite(str(out_dir / ('view_%03d.png' % len(views))), frame)
                state = 'SAVED #%d' % len(views)
                print('存 %2d  位置 (%.2f, %.2f)  大小 %.2f  倾斜 %s' % (
                    len(views), signature[0], signature[1], signature[2],
                    tilt_direction(corners, args.cols) or '平'), flush=True)
        if show:
            preview = draw_preview(frame, raw if found else None, found, state, views, size,
                                   args.cols, args.frames, time.time() - started, args.timeout)
            cv2.imshow('calibrate', preview)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print('按 q 提前结束，用已存 %d 张标定。' % len(views), flush=True)
                break
    cap.release()
    if show:
        cv2.destroyAllWindows()
    if not detected:
        print('全程没有检测到棋盘格：检查内角点数 --cols/--rows、光线和反光。', flush=True)
    return views, size


def load_images(args):
    views, size = [], None
    for path in sorted(glob.glob(str(Path(args.images) / '*.png')) + glob.glob(str(Path(args.images) / '*.jpg'))):
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if size is None:
            size = (gray.shape[1], gray.shape[0])
        if (gray.shape[1], gray.shape[0]) != size:
            raise SystemExit('mixed image sizes in %s' % args.images)
        corners = find_corners(gray, args.cols, args.rows)
        if corners is not None:
            views.append(corners)
    return views, size


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--device', default=DEVICE)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fourcc', default='YUYV')
    parser.add_argument('--cols', type=int, default=11, help='inner corners per row')
    parser.add_argument('--rows', type=int, default=8, help='inner corners per column')
    parser.add_argument('--square', type=float, default=0.03, help='square size, m')
    parser.add_argument('--frames', type=int, default=40)
    parser.add_argument('--timeout', type=float, default=300)
    parser.add_argument('--steady-px', type=float, default=3.0,
                        help='max mean corner shift between consecutive frames to save one')
    parser.add_argument('--no-show', dest='show', action='store_false',
                        help='do not open the live preview window')
    parser.add_argument('--images', help='calibrate saved frames instead of the live camera')
    parser.add_argument('--camera-name', default='down_hd_1280x720_yuyv')
    parser.add_argument('--out', type=Path, help='default: --images folder, or a new data/calibration run')
    args = parser.parse_args()
    if args.out is None:
        args.out = (Path(args.images) if args.images else
                    PROJECT / 'data/calibration/down_camera' / time.strftime('%Y%m%d_%H%M%S'))
    args.out.mkdir(parents=True, exist_ok=True)
    views, size = load_images(args) if args.images else capture_live(args, args.out)
    if len(views) < 10:
        raise SystemExit('only %d usable views; need at least 10' % len(views))
    result = calibrate(views, size, args.cols, args.rows, args.square)
    matrix, distortion, sd = result['matrix'], result['distortion'], result['sd']
    info = camera_info(matrix, distortion, size, args.camera_name)
    target = args.out / 'camera_info.yaml'
    target.write_text(yaml.safe_dump(info, sort_keys=False), encoding='utf-8')
    regions, tilt_gaps, tilts = missing(views, size, args.cols)
    print('\n%d 张，RMS %.3f px（单张最大 %.2f）' % (len(views), result['rms'], result['per_view'].max()))
    print('fx %.1f±%.1f fy %.1f±%.1f cx %.1f±%.1f cy %.1f±%.1f' % (
        matrix[0, 0], sd[0], matrix[1, 1], sd[1], matrix[0, 2], sd[2], matrix[1, 2], sd[3]))
    print('畸变 %s  ±%s' % (np.round(distortion, 3).tolist(), np.round(sd[4:9], 3).tolist()))
    print('倾斜张数 %s；未覆盖区域 %s' % (
        ' '.join('%s %d' % kv for kv in tilts.items()), ' '.join(regions) or '无'))
    print('写入 %s' % target)
    print('参考：RMS < 0.5 px、fx 误差 < 0.5%%（现在 %.1f%%）、四个倾斜方向各 ≥ 2 张、边角区域都有，较可靠。'
          % (100 * sd[0] / matrix[0, 0]))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
