#ifndef PX4CTRL_PAYLOAD_PD_H
#define PX4CTRL_PAYLOAD_PD_H

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

// No ROS dependencies: flight state gates live at the service boundary.
namespace payload_pd {
struct Gains {
  std::array<double, 3> kp;
  std::array<double, 3> kv;
};

class Schedule {
public:
  Schedule(const Gains &base, const std::array<Gains, 4> &profiles,
           bool enabled, double transition_s)
      : profiles_(profiles), from_(base), target_(base), enabled_(enabled),
        transition_s_(transition_s) {
    if (enabled_) {
      if (!std::isfinite(transition_s_) || transition_s_ <= 0.)
        throw std::invalid_argument("payload_pd/transition_s must be positive and finite");
      for (const Gains &g : profiles_)
        for (int axis = 0; axis < 3; ++axis)
          if (!std::isfinite(g.kp[axis]) || g.kp[axis] <= 0. ||
              !std::isfinite(g.kv[axis]) || g.kv[axis] <= 0.)
            throw std::invalid_argument("payload PD gains must be positive and finite");
    }
  }

  bool reset(unsigned mask, bool ground_reset_allowed, double now, std::string &message) {
    if (!enabled_) { message = "payload PD scheduling disabled"; return false; }
    if (!ground_reset_allowed) { message = "reset requires fresh connected, disarmed MANUAL_CTRL state"; return false; }
    if (mask > 7) { message = "loaded_mask must be 0..7 (slots 1,2,3)"; return false; }
    loaded_mask_ = initial_mask_ = mask;
    ++generation_;
    ready_ = true;
    from_ = target_ = profiles_[remaining()];
    started_at_ = now;
    message = "ground payload inventory reset; selected PD profile";
    return true;
  }

  bool release(unsigned slot, double now, std::string &message) {
    if (!enabled_ || !ready_) { message = "payload PD scheduling disabled or inventory not initialized"; return false; }
    if (slot < 1 || slot > 3 || !(initial_mask_ & (1u << (slot - 1)))) {
      message = "slot was not loaded at ground reset"; return false;
    }
    const unsigned bit = 1u << (slot - 1);
    if (!(loaded_mask_ & bit)) {
      message = "release already recorded; profile unchanged";
      return true;
    }
    from_ = gains(now);  // Continuous even if another release arrives during a ramp.
    loaded_mask_ &= ~bit;
    target_ = profiles_[remaining()];
    started_at_ = now;
    message = "release ACK recorded; transitioning PD profile (physical release unverified)";
    return true;
  }

  Gains gains(double now) const {
    if (!enabled_ || !ready_) return from_;
    const double x = std::max(0., std::min(1., (now - started_at_) / transition_s_));
    const double weight = x * x * (3. - 2. * x);
    Gains result;
    for (int axis = 0; axis < 3; ++axis) {
      result.kp[axis] = from_.kp[axis] + weight * (target_.kp[axis] - from_.kp[axis]);
      result.kv[axis] = from_.kv[axis] + weight * (target_.kv[axis] - from_.kv[axis]);
    }
    return result;
  }

  unsigned remaining() const {
    return (loaded_mask_ & 1u) + ((loaded_mask_ >> 1) & 1u) + ((loaded_mask_ >> 2) & 1u);
  }
  unsigned loadedMask() const { return loaded_mask_; }
  bool ready() const { return ready_; }
  bool enabled() const { return enabled_; }
  unsigned generation() const { return generation_; }

private:
  std::array<Gains, 4> profiles_;
  Gains from_, target_;
  bool enabled_, ready_ = false;
  unsigned loaded_mask_ = 0, initial_mask_ = 0;
  unsigned generation_ = 0;
  double transition_s_, started_at_ = 0.;
};
}  // namespace payload_pd
#endif
