#pragma once

#include <algorithm>

namespace airseafly_landing {

struct VerticalReference {
  double position;
  double velocity;
  double acceleration;
};

// Start with zero vertical velocity/acceleration, ramp to the existing landing
// speed in one second. The polynomial's integral supplies matching position.
inline VerticalReference vertical_reference(double start_z, double speed,
                                             double elapsed, double actual_z) {
  const double ramp_s = 1.0;
  const double t = std::max(0.0, elapsed);
  const double u = std::min(1.0, t / ramp_s);
  const double factor = u * u * (3.0 - 2.0 * u);
  const double distance_time = t < ramp_s ? ramp_s * (u*u*u - 0.5*u*u*u*u)
                                          : t - 0.5 * ramp_s;
  const double acceleration = t < ramp_s ? speed * 6.0 * u * (1.0-u) / ramp_s : 0.0;
  // Keep the reference from descending indefinitely below a stopped vehicle.
  // 0.6 m retains the existing land detector's >0.5 m / 3 s condition.
  return {std::max(start_z + speed * distance_time, actual_z - 0.6),
          speed * factor, acceleration};
}

}  // namespace airseafly_landing
