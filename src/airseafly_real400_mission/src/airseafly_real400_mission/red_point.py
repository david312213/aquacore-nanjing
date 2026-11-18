"""ROS-free red-centroid detection and near-level DF200 coordinate preview.

Observations only: no target association, drop numbering or flight interface.
"""
from bisect import bisect_left
import math
import time

import cv2
import numpy as np

from .green_circle import green_candidates
from .down_projection import camera_aim, pixel_displacement
from .quaternion import normalized_quaternion, quaternion_slerp


def _ellipse_coordinates(points, ellipse):
    angle = math.radians(ellipse['angle_deg'])
    dx, dy = (np.asarray(points)-ellipse['center_px']).T
    return np.column_stack((dx*math.cos(angle)+dy*math.sin(angle),
                            -dx*math.sin(angle)+dy*math.cos(angle))) / (np.asarray(ellipse['axes_px'])/2.)


def _outer_ellipses(mask, settings):
    """Fit foreground outer boundaries, never a ring's hole or a crop edge."""
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2:]
    if hierarchy is None:
        return []
    height, width = mask.shape
    results = []
    for index, contour in enumerate(contours):
        if len(contour) < settings['min_contour_points'] or cv2.contourArea(contour) < settings['min_area_px']:
            continue
        depth, parent = 0, hierarchy[0, index, 3]
        while parent >= 0:
            depth += 1
            parent = hierarchy[0, parent, 3]
        if depth % 2:
            continue  # Odd depth is a hole, e.g. the inside of the green ring.
        points = contour.reshape(-1, 2)
        points = points[(points[:, 0] > 1) & (points[:, 0] < width-2) &
                        (points[:, 1] > 1) & (points[:, 1] < height-2)]
        if len(points) < settings['min_contour_points']:
            continue
        try:
            center, axes, angle = cv2.fitEllipse(points)
        except cv2.error:
            continue
        if not np.all(np.isfinite((*center, *axes, angle))) or min(axes) <= 0:
            continue
        ellipse = dict(center_px=list(center), axes_px=list(axes), angle_deg=float(angle),
                       radius_px=math.sqrt(axes[0]*axes[1])/2.)
        normalized = _ellipse_coordinates(points, ellipse)
        if not np.all(np.isfinite(normalized)):
            continue
        residual = float(np.mean(np.abs(np.linalg.norm(normalized, axis=1)-1.)))
        angles = np.sort(np.arctan2(normalized[:, 1], normalized[:, 0]))
        coverage = float(360.-math.degrees(np.max(np.diff(np.r_[angles, angles[0]+2*math.pi]))))
        if residual > settings['max_radial_residual'] or coverage < settings['min_arc_degrees']:
            continue
        ellipse.update(radial_residual=residual, arc_degrees=coverage, _contour=contour)
        results.append(ellipse)
    return results


def _nested_ellipses(white, green, settings):
    ratio = white['radius_px']/green['radius_px']
    low, high = settings['white_green_radius_ratio']
    if not low <= ratio <= high:
        return False
    # Check the fitted white outline, not its bounding box or the outer sheet.
    angles = np.linspace(0., 2*math.pi, 48, endpoint=False)
    points = np.column_stack((np.cos(angles), np.sin(angles))) * (np.asarray(white['axes_px'])/2.)
    angle = math.radians(white['angle_deg'])
    x, y = points.T
    points = np.column_stack((x*math.cos(angle)-y*math.sin(angle),
                              x*math.sin(angle)+y*math.cos(angle))) + white['center_px']
    return bool(np.all(np.linalg.norm(_ellipse_coordinates(points, green), axis=1) <= 1.))


def _confirm_target(candidate, greens, whites, settings):
    """Single-frame appearance validation only; no temporal or flight rules."""
    pixel = tuple(candidate['pixel'])
    greens = [e for e in greens if cv2.pointPolygonTest(e['_contour'], pixel, False) >= 0]
    whites = [e for e in whites if cv2.pointPolygonTest(e['_contour'], pixel, False) >= 0]
    candidate.update(target_confirmed=False, confirmation_reason='missing_green_outer',
                     circles=dict(green_outer=None, white_inner=None),
                     center_error_px=None, center_tolerance_px=None)
    if not greens:
        return
    candidate['confirmation_reason'] = 'missing_white_inner'
    if not whites:
        return
    pairs = []
    for white in whites:
        for green in greens:
            if not _nested_ellipses(white, green, settings):
                continue
            centers = np.asarray([pixel, white['center_px'], green['center_px']])
            error = float(np.max(np.linalg.norm(centers[:, None]-centers[None, :], axis=2)))
            tolerance = max(settings['concentric_tolerance_px'],
                            settings['concentric_tolerance_ratio']*white['radius_px'])
            pairs.append((error/tolerance, error, tolerance, green, white))
    candidate['confirmation_reason'] = 'invalid_nesting'
    if not pairs:
        return
    _, error, tolerance, green, white = min(pairs, key=lambda pair: pair[0])
    candidate.update(
        target_confirmed=error <= tolerance,
        confirmation_reason='confirmed' if error <= tolerance else 'not_concentric',
        circles={name: {key: value for key, value in ellipse.items() if not key.startswith('_')}
                 for name, ellipse in (('green_outer', green), ('white_inner', white))},
        center_error_px=error, center_tolerance_px=tolerance)


def red_candidates(image, settings):
    """Keep every red candidate; only a nested, concentric target is confirmed."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red = np.zeros(image.shape[:2], dtype=np.uint8)
    for low, high in settings['red_ranges']:
        red |= cv2.inRange(hsv, np.asarray(low, np.uint8), np.asarray(high, np.uint8))
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(red, connectivity=8)
    candidates = []
    for label in range(1, count):
        x, y, width, height, area = (int(v) for v in stats[label])
        if area < settings['min_area_px']:
            continue
        aspect = max(width / height, height / width)
        if aspect > settings['max_aspect_ratio']:
            continue
        component = np.uint8(labels[y:y+height, x:x+width] == label) * 255
        contours = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        contour = max(contours, key=cv2.contourArea)
        contour_area, perimeter = cv2.contourArea(contour), cv2.arcLength(contour, True)
        circularity = min(1., 4 * math.pi * contour_area / perimeter**2) if perimeter else 0.
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        solidity = contour_area / hull_area if hull_area else 0.
        if circularity < settings['min_circularity'] or solidity < settings['min_solidity']:
            continue
        u, v = (float(value) for value in centroids[label])
        candidates.append(dict(class_name='red_point', pixel=[u, v],
            bbox_xywh=[x, y, width, height], area_px=area,
            circularity=circularity, solidity=solidity,
            score=.25*circularity + .15*solidity))
    if candidates:
        low, high = settings['green_range']
        green = cv2.inRange(hsv, np.asarray(low, np.uint8), np.asarray(high, np.uint8))
        low, high = settings['white_range']
        white = cv2.inRange(hsv, np.asarray(low, np.uint8), np.asarray(high, np.uint8))
        white[(green != 0) | (red != 0)] = 0
        # Extract both colour boundaries once per image, shared by all red dots.
        greens, whites = _outer_ellipses(green, settings), _outer_ellipses(white, settings)
        for candidate in candidates:
            _confirm_target(candidate, greens, whites, settings)
    # Labels are per-image display ranks, never stable IDs or drop numbers.
    return sorted(candidates, key=lambda c: (not c['target_confirmed'], -c['score'],
                                             c['pixel'][0], c['pixel'][1]))


def attitude_degrees(q):
    x, y, z, w = normalized_quaternion(q)
    return [math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))),
            math.degrees(math.asin(float(np.clip(2*(w*y-z*x), -1., 1.)))),
            math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))]


class PoseHistory:
    """Bounded storage, unrestricted time separation, nearest fallback."""
    def __init__(self, capacity):
        self.capacity = int(capacity)
        if self.capacity < 2:
            raise ValueError('pose_history_samples must be at least 2')
        self.samples = []

    def add(self, stamp, position, orientation, frame_id):
        p = np.asarray(position, dtype=float)
        if p.shape != (3,) or not np.all(np.isfinite(p)) or not math.isfinite(stamp):
            raise ValueError('non-finite odometry')
        sample = dict(stamp_s=float(stamp), position=p.tolist(),
                      quaternion=normalized_quaternion(orientation).tolist(), frame_id=frame_id)
        rolled_back = bool(self.samples and stamp < self.samples[-1]['stamp_s'])
        if rolled_back:
            self.samples.clear()
        if self.samples and stamp == self.samples[-1]['stamp_s']:
            self.samples[-1] = sample
        else:
            self.samples.append(sample)
            self.samples = self.samples[-self.capacity:]
        return rolled_back

    def at(self, stamp):
        if not self.samples:
            return None
        index = bisect_left([s['stamp_s'] for s in self.samples], stamp)
        nearest = min(self.samples[max(0, index-1):index+1], key=lambda s: abs(s['stamp_s']-stamp))
        result = dict(nearest, method='exact' if nearest['stamp_s'] == stamp else 'nearest',
                      source_stamps_s=[nearest['stamp_s']],
                      nearest_dt_s=nearest['stamp_s']-stamp)
        if 0 < index < len(self.samples) and nearest['stamp_s'] != stamp:
            before, after = self.samples[index-1:index+1]
            if before['frame_id'] == after['frame_id']:
                fraction = (stamp-before['stamp_s']) / (after['stamp_s']-before['stamp_s'])
                result.update(stamp_s=float(stamp), method='interpolated',
                    source_stamps_s=[before['stamp_s'], after['stamp_s']],
                    position=((1-fraction)*np.asarray(before['position'])+
                              fraction*np.asarray(after['position'])).tolist(),
                    quaternion=quaternion_slerp(before['quaternion'], after['quaternion'], fraction).tolist())
        result['dt_s'] = result['stamp_s']-stamp
        result['rpy_deg'] = attitude_degrees(result['quaternion'])
        return result


class RedPointSession:
    """Use the ground-initialized localization zero, never the first preview pose.

    The bridge initializes the vehicle at local Z=0. The user establishes that
    frame on the target's ground plane; starting/restarting this viewer aloft
    must not rebase height. Caller serializes access to pose history.
    """
    def __init__(self, config):
        self.config = config
        self.history = PoseHistory(config['pose_history_samples'])
        self.frame_token = None
        self.reference_event = 'localization_origin'
        self.epoch = 0

    def reset_localization(self, reason):
        self.history.samples.clear()
        self.reference_event = reason
        self.epoch += 1

    def observe_frame(self, token):
        if token != self.frame_token:
            if self.frame_token is not None:
                self.reset_localization('localization_frame_changed')
            self.frame_token = token

    def observe_pose(self, stamp, position, orientation, frame_id):
        if self.history.samples and frame_id != self.history.samples[-1]['frame_id']:
            self.reset_localization('odometry_frame_changed')
        if self.history.add(stamp, position, orientation, frame_id):
            # History.add already discarded old samples; retain the new one.
            self.epoch += 1
            self.reference_event = 'odometry_time_restarted'

    def describe(self, candidates, image_size, stamp, detection_ms, offline=False):
        width, height = image_size
        projection = self.config['projection']
        down = dict(projection, reference_width=width, reference_height=height)
        aim = list(camera_aim(down)) if not offline else None
        pose = self.history.at(stamp) if not offline else None
        status = ('offline_pixels_only' if offline else 'missing_odometry' if pose is None else
                  'frame_mismatch' if pose['frame_id'] != projection['frame_id'] else
                  'ok_near_level')
        camera_height = None
        if status == 'ok_near_level':
            camera_height = pose['position'][2]+projection['camera_height_at_reference_m']
            if camera_height <= 0:
                status = 'nonpositive_camera_height'
        output = []
        for candidate in candidates:
            item = dict(candidate, xy_m=None, offset_xy_m=None, projection_status=status,
                        pixel_error=None if aim is None else [candidate['pixel'][i]-aim[i] for i in (0, 1)])
            if not candidate.get('target_confirmed', False):
                item['projection_status'] = 'target_unconfirmed'
            elif status == 'ok_near_level':
                displacement = pixel_displacement(candidate['pixel'], camera_height,
                                                  math.radians(pose['rpy_deg'][2]), down)
                item['xy_m'] = [pose['position'][i]+displacement[i] for i in (0, 1)]
                item['offset_xy_m'] = list(displacement)
            output.append(item)
        green_output = [c for c in output if c.get('class_name') == 'green_circle']
        landing_output = [c for c in output if c.get('class_name') == 'landing_pad']
        output = [c for c in output if c.get('class_name') not in ('green_circle', 'landing_pad')]
        return dict(schema='airseafly.red_point.v1', class_name='red_point',
                    frame_id=projection['frame_id'], image_stamp_s=float(stamp),
                    image_size=[width, height], camera_aim=aim, pose=pose,
                    ground_reference=None if offline else dict(z_m=0., source='localization_origin', epoch=self.epoch),
                    reference_event=self.reference_event, localization_epoch=self.epoch,
                    camera_height_m=camera_height, projection_status=status,
                    projection_model='near_level_yaw_only',
                    detection_ms=float(detection_ms), candidates=output,
                    green_candidates=green_output, green_count=len(green_output),
                    landing_candidates=landing_output, landing_count=len(landing_output),
                    confirmed_count=sum(bool(c.get('target_confirmed', False)) for c in output))


def detect(image, config):
    start = time.perf_counter()
    candidates = red_candidates(image, config['detector'])
    return candidates, (time.perf_counter()-start)*1000


def detect_targets(image, config):
    """Both centers for the shared preview/worker; detect() remains red-only."""
    start = time.perf_counter()
    candidates = red_candidates(image, config['detector'])
    candidates.extend(green_candidates(image, config['detector']))
    return candidates, (time.perf_counter()-start)*1000


def annotate(image, status):
    """A fixed-size camera frame with detection markers; metrics stay in text."""
    candidates = [c for c in status['candidates'] if c.get('target_confirmed', False)]
    canvas = image.copy()
    if status.get('image_error'):
        label, colour = 'IMAGE ERROR', (0, 160, 255)
    elif status.get('landing_candidates'):
        label, colour = 'H PAD FOUND', (0, 220, 0)
    elif candidates:
        label, colour = 'TARGET CONFIRMED', (0, 220, 0)
    elif status.get('green_candidates'):
        label, colour = 'GREEN CENTER FOUND', (0, 220, 0)
    elif status['candidates']:
        label, colour = 'UNCONFIRMED', (0, 160, 255)
    else:
        label, colour = 'NO RED POINT', (0, 160, 255)
    cv2.rectangle(canvas, (0, 0), (230, 34), (24, 24, 24), -1)
    cv2.putText(canvas, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, .65, colour, 2, cv2.LINE_AA)
    for candidate in status.get('green_candidates', []):
        u, v = (round(value) for value in candidate['pixel'])
        cv2.circle(canvas, (u, v), round(candidate['radius_px']), (0, 220, 0), 2)
        cv2.drawMarker(canvas, (u, v), (255, 255, 0), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(canvas, 'GREEN EST CENTER', (max(0, u-80), max(50, v-18)),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 0), 1, cv2.LINE_AA)
    for candidate in status.get('landing_candidates', []):
        x, y, w, h = candidate['bbox_xywh']
        u, v = (round(value) for value in candidate['pixel'])
        cv2.rectangle(canvas, (x, y), (x+w, y+h), (255, 0, 255), 2)
        cv2.drawMarker(canvas, (u, v), (255, 0, 255), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(canvas, 'H %.2f' % candidate['score'], (x, max(50, y-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 0, 255), 2, cv2.LINE_AA)
    for candidate in candidates:
        u, v = (round(value) for value in candidate['pixel'])
        cv2.putText(canvas, 'RED CENTER', (max(0, u-50), min(canvas.shape[0]-10, v+24)),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 1, cv2.LINE_AA)
        x, y, w, h = candidate['bbox_xywh']
        cv2.rectangle(canvas, (x-3, y-3), (x+w+3, y+h+3), (0, 220, 0), 2)
        cv2.drawMarker(canvas, (u, v), (0, 220, 0), cv2.MARKER_CROSS, 12, 1)
    return canvas
