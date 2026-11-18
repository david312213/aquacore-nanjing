#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace airseafly_control {

inline void validate_tilt_limit(double radians) {
  if (!std::isfinite(radians) || radians >= std::acos(-1.0) / 2.0)
    throw std::invalid_argument("max_angle must be finite and below 90 degrees; negative disables tilt limiting");
}

// Limit the thrust vector about FCU gravity, not FAST-LIO's possibly tilted Z.
// Preserve positive vertical force and horizontal direction, reduce only the
// excessive horizontal component. A multirotor cannot command downward thrust.
inline Eigen::Vector3d limit_tilt(const Eigen::Vector3d &world_force,
                                const Eigen::Quaterniond &world_to_fcu,
                                double radians) {
  if (radians < 0.0) return world_force;
  Eigen::Vector3d force = world_to_fcu * world_force;
  const double horizontal = std::hypot(force.x(), force.y());
  const double vertical = std::max(0.0, force.z());
  const double allowed = vertical * std::tan(radians);
  if (force.z() >= 0.0 && horizontal <= allowed) return world_force;
  force.z() = vertical;
  if (horizontal > 0.0) force.head<2>() *= allowed / horizontal;
  return world_to_fcu.conjugate() * force;
}

inline double normalized_thrust(double requested) {
  // Match MAVROS setpoint_raw's ordering, including its non-finite behavior.
  return std::min(1.0, std::max(0.0, requested));
}

}  // namespace airseafly_control
