#pragma once
#include <cstdint>

namespace native_control {
// MAVLink POSITION_TARGET_TYPEMASK values. ROS fields remain ENU; MAVROS is
// responsible for the ENU/NED conversion. Position/yaw-only ignores v/a/yawrate.
constexpr std::uint16_t kPositionVelocityYawMask = 2496;
constexpr std::uint16_t kPositionYawMask = 2552;

inline bool supported_local_mask(std::uint16_t mask) {
  return mask == 0 || mask == kPositionVelocityYawMask || mask == kPositionYawMask;
}
}  // namespace native_control
