#pragma once

#include <bspline_opt/uniform_bspline.h>
#include <cmath>
#include <limits>

namespace ego_planner {

// Velocity lies in the convex hull of the derivative control points. Use that
// upper bound to limit travel between samples to half a map cell, including
// curves whose endpoints are close together or identical.
inline double trajectoryCollisionStep(UniformBspline &trajectory, double resolution) {
  const auto points = trajectory.getControlPoint();
  if (!std::isfinite(resolution) || resolution <= 0. || points.rows() != 3 ||
      points.cols() < 4 || !points.allFinite())
    return 0.;
  const auto velocities = trajectory.getDerivative().getControlPoint();
  if (!velocities.allFinite()) return 0.;
  double maximum_speed = 0.;
  for (int i = 0; i < velocities.cols(); ++i)
    maximum_speed = std::max(maximum_speed, velocities.col(i).norm());
  if (!std::isfinite(maximum_speed)) return 0.;
  return maximum_speed > 0. ? .5 * resolution / maximum_speed
                           : std::numeric_limits<double>::infinity();
}

// Query the complete remaining trajectory, including its final position.
// Invalid geometry is rejected instead of being treated as a clear path.
template <class Occupied>
bool firstTrajectoryCollision(UniformBspline &trajectory, double resolution,
                              const Occupied &occupied, double &collision_time,
                              double start_time = 0.) {
  const double step = trajectoryCollisionStep(trajectory, resolution);
  collision_time = std::isfinite(start_time) ? std::max(0., start_time) : 0.;
  if (!(step > 0.) || !std::isfinite(start_time)) return true;
  const double duration = trajectory.getTimeSum();
  if (!std::isfinite(duration) || duration < 0.) return true;
  double t = std::min(collision_time, duration);
  // Reject a numerically broken/extreme curve instead of scanning indefinitely.
  if (std::isfinite(step) && (duration - t) / step > 100000.) return true;
  while (true) {
    const Eigen::Vector3d position = trajectory.evaluateDeBoorT(t);
    if (!position.allFinite() || occupied(position)) {
      collision_time = t;
      return true;
    }
    if (t >= duration) return false;
    const double next = std::min(duration, t + step);
    if (!(next > t)) {
      collision_time = t;
      return true;
    }
    t = next;
  }
}

} // namespace ego_planner
