#pragma once
#include <tuple>

namespace ego_planner {
template <class Goal>
auto goalIdentity(const Goal &goal) {
  const auto &p = goal.pose.position;
  return std::make_tuple(goal.header.stamp.toNSec(), goal.header.frame_id, p.x, p.y, p.z);
}
template <class Goal>
bool sameGoal(const Goal &a, const Goal &b) { return goalIdentity(a) == goalIdentity(b); }
} // namespace ego_planner
