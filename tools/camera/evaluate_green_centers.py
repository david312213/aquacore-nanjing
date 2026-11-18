#!/usr/bin/env python3
"""Offline green-circle evaluation. No ROS, camera, controller or actuator I/O."""
import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/airseafly_real400_mission/src'))
from airseafly_real400_mission.green_circle import green_candidates
from airseafly_real400_mission.red_point import red_candidates
from airseafly_real400_mission.red_point_runtime import load_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('images', type=Path)
    parser.add_argument('--output', type=Path, default=ROOT/'data/checks/green_circle')
    parser.add_argument('--config', type=Path, default=ROOT/'flight/red_point.yaml')
    parser.add_argument('--sweep', action='store_true')
    args = parser.parse_args()
    config = load_config(args.config)
    files = sorted(args.images.glob('*.jpg'))
    if not files:
        parser.error('no JPG images')
    args.output.mkdir(parents=True, exist_ok=True)
    rows, errors, previews = [], [], []
    for index, path in enumerate(files):
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError('cannot read '+str(path))
        start = time.perf_counter()
        found = green_candidates(image, config['detector'])
        elapsed = (time.perf_counter()-start)*1000
        red = [c for c in red_candidates(image, config['detector']) if c['target_confirmed']]
        # This is a detector-to-detector comparison, NOT hand-labeled accuracy.
        distance = (min(float(np.linalg.norm(np.asarray(g['pixel'])-r['pixel']))
                        for g in found for r in red) if found and red else None)
        if distance is not None:
            errors.append(distance)
        rows.append(dict(file=path.name, candidates=found, red_confirmed=len(red),
                         red_center_distance_px=distance, green_ms=elapsed))
        for c in found:
            u, v = map(round, c['pixel'])
            cv2.circle(image, (u, v), round(c['radius_px']), (0, 255, 255), 3)
            cv2.drawMarker(image, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 22, 3)
        tile = np.zeros((128, 192, 3), np.uint8)
        tile[:108] = cv2.resize(image, (192, 108))
        cv2.putText(tile, '%d: %s' % (index+1, 'OK' if found else 'MISS'), (3, 123),
                    cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 1)
        previews.append(tile)
    summary = dict(images=len(rows), detected=sum(bool(r['candidates']) for r in rows),
        multiple=sum(len(r['candidates'])>1 for r in rows), red_detected=sum(r['red_confirmed']>0 for r in rows),
        green_ms_median=float(np.median([r['green_ms'] for r in rows])),
        green_ms_p95=float(np.percentile([r['green_ms'] for r in rows], 95)),
        compared_with_red=len(errors),
        red_center_distance_px_median=float(np.median(errors)) if errors else None,
        red_center_distance_px_p95=float(np.percentile(errors, 95)) if errors else None,
        settings=config['detector']['green_hough'], opencv=cv2.__version__,
        note='Same-video tuning sample, not independent validation or hand-labeled accuracy; timing on this host only.')
    if args.sweep:
        summary['sweep'] = []
        for param in (24., 18., 12.):
            for support in (.55, .45):
                detector = dict(config['detector'], green_hough=dict(config['detector']['green_hough'],
                                param2=param, min_ring_support=support))
                results = [green_candidates(cv2.imread(str(path)), detector) for path in files]
                summary['sweep'].append(dict(param2=param, min_ring_support=support,
                    detected=sum(bool(r) for r in results), multiple=sum(len(r)>1 for r in results)))
    (args.output/'results.json').write_text(json.dumps(rows, indent=2)+'\n')
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    for start in range(0, len(previews), 90):
        canvas = np.zeros((1152, 1920, 3), np.uint8)
        for i, tile in enumerate(previews[start:start+90]):
            y, x = i//10*128, i%10*192
            canvas[y:y+128, x:x+192] = tile
        cv2.imwrite(str(args.output/('preview_%d.jpg' % (start//90+1))), canvas)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
