"""Single-image green-ring Hough centers; independent of white/red detection."""
import math

import cv2
import numpy as np


DEFAULTS = dict(scale=.5, dp=1., min_dist_px=60., param1=100., param2=18.,
                min_radius_px=24, max_radius_px=700, blur_kernel=5,
                min_visible_degrees=120., min_ring_support=.55,
                max_center_green=.25, dedup_radius_ratio=.5)


def green_candidates(image, settings):
    cfg = dict(DEFAULTS, **settings.get('green_hough', {}))
    scale = cfg['scale']
    small = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    low, high = settings['green_range']
    mask = cv2.inRange(hsv, np.asarray(low, np.uint8), np.asarray(high, np.uint8))
    # Fill enclosed holes only for Hough voting, so the inner ring does not
    # suppress the outer circle at the same center. Appearance uses raw mask.
    outer = np.zeros_like(mask)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cv2.drawContours(outer, contours, -1, 255, cv2.FILLED)
    smooth = cv2.GaussianBlur(outer, (cfg['blur_kernel'], cfg['blur_kernel']), 0)
    circles = cv2.HoughCircles(smooth, cv2.HOUGH_GRADIENT, cfg['dp'],
        minDist=cfg['min_dist_px']*scale, param1=cfg['param1'], param2=cfg['param2'],
        minRadius=max(1, round(cfg['min_radius_px']*scale)),
        maxRadius=max(1, round(cfg['max_radius_px']*scale)))
    if circles is None:
        return []
    height, width = mask.shape
    angles = np.linspace(0., 2*math.pi, 180, endpoint=False)
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    results = []
    for u, v, radius in circles[0]:
        if not (0 <= u < width and 0 <= v < height):
            continue
        # An outer green boundary has green just inside and non-green outside.
        # Sample only visible arcs: a clipped ring need not fit wholly in frame.
        points = np.rint(np.asarray([u, v])+directions[None, :, :]*radius*
                         np.asarray([.85, .95, 1.05, 1.15])[:, None, None]).astype(int)
        valid = np.all((points[:, :, 0] >= 0) & (points[:, :, 0] < width) &
                       (points[:, :, 1] >= 0) & (points[:, :, 1] < height), axis=0)
        if valid.sum()*2 < cfg['min_visible_degrees']:
            continue
        samples = mask[points[:, valid, 1], points[:, valid, 0]] > 0
        support = float(np.mean(np.any(samples[:2], axis=0) & ~np.any(samples[2:], axis=0)))
        center_points = np.rint(np.asarray([u, v])+directions*radius*.2).astype(int)
        inside = ((center_points[:, 0] >= 0) & (center_points[:, 0] < width) &
                  (center_points[:, 1] >= 0) & (center_points[:, 1] < height))
        center_green = float(np.mean(mask[center_points[inside, 1], center_points[inside, 0]] > 0))
        if support < cfg['min_ring_support'] or center_green > cfg['max_center_green']:
            continue
        u, v, radius = float(u/scale), float(v/scale), float(radius/scale)
        results.append(dict(class_name='green_circle', pixel=[u, v], radius_px=radius,
            bbox_xywh=[round(u-radius), round(v-radius), round(2*radius), round(2*radius)],
            score=support, target_confirmed=True, confirmation_reason='green_hough',
            visible_degrees=float(valid.sum()*2)))
    chosen = []
    for item in sorted(results, key=lambda c: -c['score']):
        if not any(math.dist(item['pixel'], other['pixel']) <
                   cfg['dedup_radius_ratio']*max(item['radius_px'], other['radius_px']) for other in chosen):
            chosen.append(item)
    return chosen
