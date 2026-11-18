#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace airseafly_control {

using IntegralVector = std::array<double, 3>;

struct PositionIntegralConfig {
  bool enabled = false;  // Old YAML files retain the original PD behavior.
  IntegralVector ki{{0.0, 0.0, 0.0}};             // 1/s^3, integrates position error
  IntegralVector max_acceleration{{0.4, 0.4, 0.4}};  // m/s^2, per world axis

  void validate() const {
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(ki[i]) || ki[i] < 0.0 ||
          !std::isfinite(max_acceleration[i]) || max_acceleration[i] < 0.0 ||
          (ki[i] > 0.0 && max_acceleration[i] == 0.0))
        throw std::invalid_argument("position_integral: finite nonnegative gains and limits required; active axes need a positive limit");
    }
  }
};

// Stores the acceleration contribution directly (Ki * integral(error * dt)).
// Integrates every elapsed control interval, with only a per-axis I limit.
// The controller resets history when it relinquishes position control.
class PositionIntegral {
 public:
  explicit PositionIntegral(const PositionIntegralConfig &config) : config_(config) {
    config_.validate();
  }

  void reset() {
    acceleration_.fill(0.0);
    have_time_ = false;
  }

  const IntegralVector &acceleration() const { return acceleration_; }

  IntegralVector update(const IntegralVector &error, double now) {
    if (!config_.enabled || !std::isfinite(now) || !finite(error)) {
      reset();
      return acceleration_;
    }
    if (!have_time_) {
      last_time_ = now;
      have_time_ = true;
      return acceleration_;
    }
    const double dt = now - last_time_;
    last_time_ = now;
    if (dt < 0.0) {
      reset();
      return acceleration_;
    }
    // A paused simulation clock must not accumulate (or lose) integral.
    if (dt == 0.0) return acceleration_;
    for (int i = 0; i < 3; ++i) {
      const double limit = config_.max_acceleration[i];
      acceleration_[i] = std::max(-limit, std::min(limit, acceleration_[i] + config_.ki[i] * error[i] * dt));
    }
    return acceleration_;
  }

 private:
  static bool finite(const IntegralVector &values) {
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
  }
  PositionIntegralConfig config_;
  IntegralVector acceleration_{{0.0, 0.0, 0.0}};
  double last_time_ = 0.0;
  bool have_time_ = false;
};

}  // namespace airseafly_control
