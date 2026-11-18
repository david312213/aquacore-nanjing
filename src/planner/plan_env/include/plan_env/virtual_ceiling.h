#pragma once

namespace ego_planner {

// A virtual occupied half-space for the existing 3D planner. The boundary is
// continuous (not rounded down to a voxel) and already denotes the permitted
// vehicle-reference height: do not apply obstacle inflation a second time.
// No trajectory coordinate is clamped or rewritten here.
struct VirtualCeiling {
  double height = -1.0;  // metres above origin_z; negative disables the ceiling
  double origin_z = 0.0; // world Z of takeoff reference; 0 for legacy world mode

  bool enabled() const { return height >= 0.0; }
  double worldZ() const { return origin_z + height; }
  bool occupied(double world_z) const {
    return enabled() && world_z >= worldZ();
  }
};

} // namespace ego_planner
