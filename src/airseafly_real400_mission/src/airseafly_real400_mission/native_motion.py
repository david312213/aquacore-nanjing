"""PX4-native geometry and bounded straight-line references (no ROS).

Positions are ENU, rotations map body FLU into ENU. Raw FAST-LIO twist is
world linear velocity and IMU angular velocity, unlike REP-147 odometry.
"""
import math
import numpy as np


def vector(value, size=3):
    out = np.asarray(value, dtype=float)
    if out.shape != (size,) or not np.isfinite(out).all():
        raise ValueError('expected a finite %d-vector' % size)
    return out


def rotation(xyzw):
    q = vector(xyzw, 4)
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        raise ValueError('zero quaternion')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quaternion(matrix):
    # Symmetric eigenproblem also handles rotations near 180 degrees.
    m = np.asarray(matrix, dtype=float)
    K = np.array([[m[0,0]-m[1,1]-m[2,2], m[0,1]+m[1,0], m[0,2]+m[2,0], m[2,1]-m[1,2]],
                  [m[0,1]+m[1,0], m[1,1]-m[0,0]-m[2,2], m[1,2]+m[2,1], m[0,2]-m[2,0]],
                  [m[0,2]+m[2,0], m[1,2]+m[2,1], m[2,2]-m[0,0]-m[1,1], m[1,0]-m[0,1]],
                  [m[2,1]-m[1,2], m[0,2]-m[2,0], m[1,0]-m[0,1], np.trace(m)]]) / 3.
    values, vectors = np.linalg.eigh(K)
    q = vectors[:, np.argmax(values)]
    return q if q[3] >= 0 else -q


class FrozenLioFrame:
    """One fixed initialized frame plus the IMU-to-body lever arm."""
    def __init__(self, mount_xyzw, mount_xyz):
        self.r_bi, self.t_bi = rotation(mount_xyzw), vector(mount_xyz)
        self.r_nw = self.t_nw = None

    def initialize(self, p_wi, q_wi, p_nb, q_nb, armed=None):
        # Keep the old optional argument for callers; arming is not a gate.
        if self.r_nw is not None:
            raise ValueError('frame is already initialized')
        r_wb = rotation(q_wi) @ self.r_bi.T
        p_wb = vector(p_wi) - r_wb @ self.t_bi
        # Full rotation is essential: the raw LIO world may be tilted.
        self.r_nw = rotation(q_nb) @ r_wb.T
        self.t_nw = vector(p_nb) - self.r_nw @ p_wb

    def transform(self, p_wi, q_wi, v_wi, omega_i):
        if self.r_nw is None:
            raise ValueError('frame is not initialized')
        r_wi = rotation(q_wi)
        r_wb = r_wi @ self.r_bi.T
        lever = r_wb @ self.t_bi
        r_nb = self.r_nw @ r_wb
        p_nb = self.r_nw @ (vector(p_wi)-lever) + self.t_nw
        v_wb = vector(v_wi) - np.cross(r_wi @ vector(omega_i), lever)
        v_b = r_nb.T @ self.r_nw @ v_wb
        return p_nb, quaternion(r_nb), v_b, self.r_bi @ vector(omega_i)


class StraightLine:
    """Analytic trapezoidal reference to an immutable target; no integration drift."""
    def __init__(self, start, goal, started, horizontal=.10, vertical=.15, acceleration=.50):
        self.start, self.goal = vector(start), vector(goal)
        self.started = float(started)
        if (not math.isfinite(self.started) or
                not all(math.isfinite(v) and v > 0 for v in (horizontal, vertical, acceleration))):
            raise ValueError('invalid motion limits')
        delta = self.goal-self.start
        self.length = float(np.linalg.norm(delta))
        self.direction = delta/self.length if self.length else np.zeros(3)
        h, z = np.linalg.norm(self.direction[:2]), abs(self.direction[2])
        limit = min(horizontal/h if h else float('inf'), vertical/z if z else float('inf'))
        self.acceleration = float(acceleration)
        self.speed = min(limit, math.sqrt(self.length*acceleration)) if self.length else 0.
        self.ramp = self.speed/acceleration
        self.cruise = max(0., self.length/self.speed-self.ramp) if self.speed else 0.
        self.duration = 2*self.ramp+self.cruise

    def sample(self, now):
        t = max(0., float(now)-self.started)
        if t >= self.duration:
            return tuple(self.goal), (0., 0., 0.), True
        if t < self.ramp:
            speed, distance = self.acceleration*t, .5*self.acceleration*t*t
        elif t < self.ramp+self.cruise:
            speed = self.speed
            distance = .5*self.speed*self.ramp+self.speed*(t-self.ramp)
        else:
            remaining = self.duration-t
            speed = self.acceleration*remaining
            distance = self.length-.5*self.acceleration*remaining*remaining
        return tuple(self.start+self.direction*distance), tuple(self.direction*speed), False
