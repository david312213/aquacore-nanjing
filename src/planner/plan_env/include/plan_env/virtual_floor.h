#pragma once

namespace ego_planner {

// Occupied lower half-space, relative to the same takeoff reference as the
// ceiling. This is the vehicle-reference boundary, without extra inflation or
// voxel rounding. It constrains planning; it does not clamp flight commands.
struct VirtualFloor {
  double height = -1.0;  // metres above origin_z; negative disables the floor
  double origin_z = 0.0;

  bool enabled() const { return height >= 0.0; }
  double worldZ() const { return origin_z + height; }
  bool occupied(double world_z) const {
    return enabled() && world_z <= worldZ();
  }
};

} // namespace ego_planner
