#pragma once
#include <Eigen/Eigen>
#include <bspline_opt/uniform_bspline.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace ego_planner {

// Cubic uniform-spline boundary equations at t=0 and t=end. The endpoint
// and its zero velocity/acceleration remain exact after changing the interval.
inline void fixShortBoundary(Eigen::MatrixXd &points, double dt,
                             const Eigen::Vector3d &start, const Eigen::Vector3d &velocity,
                             const Eigen::Vector3d &acceleration, const Eigen::Vector3d &end) {
  points.col(0) = start - velocity * dt + acceleration * (dt * dt / 3.);
  points.col(1) = start - acceleration * (dt * dt / 6.);
  points.col(2) = start + velocity * dt + acceleration * (dt * dt / 3.);
  points.rightCols(3).colwise() = end;
}

inline Eigen::MatrixXd shortControlPoints(const Eigen::Vector3d &start,
                                         const Eigen::Vector3d &velocity,
                                         const Eigen::Vector3d &acceleration,
                                         const Eigen::Vector3d &end, double dt) {
  Eigen::MatrixXd points(3, 9);
  fixShortBoundary(points, dt, start, velocity, acceleration, end);
  for (int i = 3; i < 6; ++i)
    points.col(i) = points.col(2) + (end - points.col(2)) * ((i - 2) / 4.);
  return points;
}

inline bool retimeShortControlPoints(Eigen::MatrixXd &points, double &dt,
    const Eigen::Vector3d &start, const Eigen::Vector3d &velocity,
    const Eigen::Vector3d &acceleration, const Eigen::Vector3d &end,
    double max_velocity, double max_acceleration, double tolerance) {
  // Fixed initial derivatives must themselves satisfy the existing physical
  // limits. Retiming cannot reduce an immutable measured starting velocity.
  if (velocity.cwiseAbs().maxCoeff() > max_velocity * (1. + tolerance) + 1e-4 ||
      acceleration.cwiseAbs().maxCoeff() > max_acceleration * (1. + tolerance) + 1e-4) return false;
  while (true) {
    fixShortBoundary(points, dt, start, velocity, acceleration, end);
    if (!points.allFinite()) return false;
    UniformBspline spline(points, 3, dt);
    spline.setPhysicalLimits(max_velocity, max_acceleration, tolerance);
    double ratio;
    if (spline.checkFeasibility(ratio, false)) return true;
    const double next = dt * std::max(1.01, ratio);
    if (!std::isfinite(next) || !(next > dt)) return false;
    dt = next;
  }
}

// Exactly N+1 samples, even for stationary or repeated source points. Neither
// shrinking spacing nor floating point accumulation controls termination.
inline std::vector<Eigen::Vector3d> resamplePolyline(
    const std::vector<Eigen::Vector3d> &input, double spacing) {
  if (input.empty()) return {};
  std::vector<Eigen::Vector3d> points{input.front()};
  std::vector<double> length{0.};
  for (size_t i = 1; i < input.size(); ++i) {
    const double d = (input[i] - points.back()).norm();
    if (d > 1e-9) { points.push_back(input[i]); length.push_back(length.back() + d); }
  }
  if (points.size() == 1) return std::vector<Eigen::Vector3d>(7, input.back());
  const size_t segments = std::max<size_t>(6, static_cast<size_t>(std::ceil(length.back() / spacing)));
  std::vector<Eigen::Vector3d> samples;
  size_t j = 0;
  for (size_t i = 0; i < segments; ++i) {
    const double s = length.back() * double(i) / double(segments);
    while (j + 2 < points.size() && length[j + 1] <= s) ++j;
    const double fraction = (s - length[j]) / (length[j + 1] - length[j]);
    samples.push_back(points[j] + fraction * (points[j + 1] - points[j]));
  }
  samples.push_back(input.back());
  return samples;
}
} // namespace ego_planner
