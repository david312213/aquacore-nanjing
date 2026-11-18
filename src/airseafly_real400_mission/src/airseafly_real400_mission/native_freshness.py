"""Freshness depends on source progress and both clocks, not republishing."""
import math

MAX_AGE_S = .30
FUTURE_TOLERANCE_S = .05


def fresh_stamp(stamp, now, max_age=MAX_AGE_S):
    try:
        age = float(now) - float(stamp)
        return (math.isfinite(age) and float(stamp) > 0. and
                -FUTURE_TOLERANCE_S <= age <= max_age)
    except (TypeError, ValueError):
        return False


class SourceProgress:
    def __init__(self):
        self.stamp = self.received = None

    def record(self, stamp, monotonic_now):
        """Record arrival without source-age or timestamp-progress admission."""
        try:
            stamp, received = float(stamp), float(monotonic_now)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(stamp) or not math.isfinite(received):
            return False
        self.stamp, self.received = stamp, received
        return True

    def observe(self, stamp, monotonic_now):
        try:
            stamp, received = float(stamp), float(monotonic_now)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(stamp) or stamp <= 0. or not math.isfinite(received):
            return False
        if self.stamp is None or stamp > self.stamp:
            self.stamp, self.received = stamp, received
            return True
        return False

    def fresh(self, now, monotonic_now, max_age=MAX_AGE_S):
        if self.received is None or not fresh_stamp(self.stamp, now, max_age):
            return False
        age = float(monotonic_now)-self.received
        return math.isfinite(age) and 0. <= age <= max_age
