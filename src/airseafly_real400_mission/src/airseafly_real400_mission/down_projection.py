"""Shared down-camera pixel-to-world projection for fixed visual corrections.

Same near-level, yaw-only geometry as the copy delivery experiment. Inputs use
reference pixels; the raw camera calibration includes lens distortion. The
calibration comes from the red_point camera section (red_point_runtime.load_config);
a projection dict without it keeps the original DF200 intrinsics.
"""
import math

import cv2
import numpy as np

from .df200_geometry import CAMERA_MATRIX, DISTORTION, CALIBRATION_SIZE


def intrinsics(down):
    values = down.get('intrinsics')
    if values is None:
        return np.asarray(CAMERA_MATRIX, dtype=float), np.asarray(DISTORTION, dtype=float), CALIBRATION_SIZE
    return (np.asarray(values['camera_matrix'], dtype=float),
            np.asarray(values['distortion'], dtype=float), tuple(values['calibration_size']))


def reference_intrinsics(down):
    matrix, _, size = intrinsics(down)
    scaling = np.diag([down['reference_width'] / size[0],
                       down['reference_height'] / size[1], 1.])
    return scaling @ matrix


def camera_aim(down):
    matrix = reference_intrinsics(down)
    return float(matrix[0, 2]), float(matrix[1, 2])


def body_to_world(displacement, yaw_rad, down):
    forward, left = displacement
    angle = float(yaw_rad) + float(down['body_yaw_offset_rad'])
    return (math.cos(angle) * forward - math.sin(angle) * left,
            math.sin(angle) * forward + math.cos(angle) * left)


def pixel_displacement(pixel, height_m, yaw_rad, down):
    """Camera-centering correction; callers apply their own mechanical offset."""
    ray = cv2.undistortPoints(np.asarray(pixel, dtype=float).reshape(1, 1, 2),
                              reference_intrinsics(down), intrinsics(down)[1]).reshape(2)
    camera_to_target = np.asarray(down['pixel_to_body'], dtype=float) @ (ray * height_m)
    return body_to_world(camera_to_target, yaw_rad, down)
