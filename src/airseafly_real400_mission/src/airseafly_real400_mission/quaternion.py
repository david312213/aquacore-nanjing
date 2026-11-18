"""Quaternion math retained for timestamp-aligned DF200 observations."""
import math
import numpy as np
from typing import Sequence

def _vector(values: Sequence[float], size: int, name: str) -> np.ndarray:
    vector = np.asarray(values, dtype=np.float64)
    if vector.shape != (size,) or not np.all(np.isfinite(vector)):
        raise ValueError("{} must contain {} finite values".format(name, size))
    return vector


def normalized_quaternion(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    quaternion = _vector(quaternion_xyzw, 4, "quaternion_xyzw")
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("quaternion must have a finite nonzero norm")
    return quaternion / norm


def quaternion_slerp(first_xyzw, second_xyzw, fraction: float) -> np.ndarray:
    """Shortest-arc SLERP, including opposite-sign equivalent quaternions."""
    fraction = float(fraction)
    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        raise ValueError("interpolation fraction must be in [0, 1]")
    first = normalized_quaternion(first_xyzw)
    second = normalized_quaternion(second_xyzw)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return normalized_quaternion(first + fraction * (second - first))
    angle = math.acos(dot)
    sine = math.sin(angle)
    return normalized_quaternion(
        math.sin((1.0 - fraction) * angle) / sine * first
        + math.sin(fraction * angle) / sine * second
    )


