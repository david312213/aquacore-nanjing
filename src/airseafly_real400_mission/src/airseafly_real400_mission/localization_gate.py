"""PX4/FAST-LIO divergence gate; ROS-free so logs can be replayed through it.

User-confirmed rule (2026-10-31): trip when the bridge's px4_error_m stays above
max_error_m for at least hold_s of source time, while armed in OFFBOARD. Once
tripped it stays tripped until disarm. A missing error sample is not evidence of
divergence and ends the current run without tripping.
"""
import math


class DivergenceGate:
    def __init__(self, max_error_m=0.5, hold_s=0.5):
        self.max_error_m, self.hold_s = float(max_error_m), float(hold_s)
        if not (math.isfinite(self.max_error_m) and self.max_error_m > 0.):
            raise ValueError('localization_gate.max_error_m 必须为正数')
        if not (math.isfinite(self.hold_s) and self.hold_s >= 0.):
            raise ValueError('localization_gate.hold_s 必须为非负数')
        self.reset()

    def reset(self):
        self.tripped = False
        self.trip_stamp = self.trip_error_m = None
        self.run_start = None
        self.run_peak_m = 0.

    def update(self, stamp, error_m, active):
        """Feed one bridge status sample. Returns True only on the sample that trips."""
        if self.tripped:
            return False
        valid = (active and error_m is not None and stamp is not None and
                 math.isfinite(float(error_m)) and math.isfinite(float(stamp)))
        if not valid or float(error_m) <= self.max_error_m:
            self.run_start, self.run_peak_m = None, 0.
            return False
        stamp, error_m = float(stamp), float(error_m)
        if self.run_start is None or stamp < self.run_start:
            self.run_start = stamp
        self.run_peak_m = max(self.run_peak_m, error_m)
        if stamp - self.run_start >= self.hold_s:
            self.tripped, self.trip_stamp, self.trip_error_m = True, stamp, error_m
            return True
        return False
