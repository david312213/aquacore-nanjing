#pragma once

#include "native_owner.h"
#include <cstdint>
#include <string>

namespace native_control {

// Pure state machine. The caller supplies WallTime seconds and owns the one
// asynchronous RPC. A sent/accepted SET_MODE is never actual mode feedback.
class LandingRequest {
 public:
  enum class State { Idle, Requesting, Confirmed, Failed, Cancelled };
  static constexpr unsigned kMaxAttempts = 3;
  static constexpr double kConfirmationSeconds = 2.;
  static constexpr double kRetrySeconds = .5;
  static constexpr double kTotalSeconds = 8.;

 private:
  State state_ = State::Idle;
  std::uint64_t generation_ = 0;
  unsigned attempts_ = 0;
  double started_at_ = 0., confirmation_until_ = 0., next_attempt_at_ = 0.;
  bool awaiting_confirmation_ = false;
  std::string reason_;

  void fail(const char* message) {
    state_ = State::Failed;
    awaiting_confirmation_ = false;
    reason_ = message;
  }

 public:
  State state() const { return state_; }
  std::uint64_t generation() const { return generation_; }
  unsigned attempts() const { return attempts_; }
  const std::string& reason() const { return reason_; }
  bool requesting() const { return state_ == State::Requesting; }
  bool failed() const { return state_ == State::Failed; }
  const char* name() const {
    switch (state_) {
      case State::Idle: return "idle";
      case State::Requesting: return "requesting";
      case State::Confirmed: return "confirmed";
      case State::Failed: return "failed";
      case State::Cancelled: return "cancelled";
    }
    return "idle";
  }

  void reset() {
    ++generation_;
    state_ = State::Idle;
    attempts_ = 0;
    awaiting_confirmation_ = false;
    reason_.clear();
  }

  void start(double now) {
    reset();
    state_ = State::Requesting;
    started_at_ = next_attempt_at_ = now;
    reason_ = "AUTO.LAND queued; waiting for actual mode feedback";
  }

  void cancel(const char* message) {
    if (state_ != State::Requesting && state_ != State::Confirmed) return;
    state_ = State::Cancelled;
    awaiting_confirmation_ = false;
    reason_ = message;
  }

  void update(double now, const Health& h, bool landed_and_disarmed = false) {
    if (state_ != State::Requesting && state_ != State::Confirmed) return;
    // PX4 may return to MANUAL after disarming before AUTO.LAND is sampled.
    if (landed_and_disarmed && !h.armed) {
      state_ = State::Confirmed;
      awaiting_confirmation_ = false;
      reason_ = "landing completed: on ground and disarmed";
      return;
    }
    if (!h.connected) {
      fail("FCU connection unavailable during landing request");
      return;
    }
    if (state_ == State::Confirmed) {
      if (!h.auto_land) cancel("pilot left confirmed AUTO.LAND; automatic requests cancelled");
      return;
    }
    // A real mode observation wins over an RPC timeout at the same instant.
    if (h.auto_land) {
      state_ = State::Confirmed;
      awaiting_confirmation_ = false;
      reason_ = "AUTO.LAND confirmed by FCU mode feedback";
      return;
    }
    if (!h.offboard) {
      cancel("mode left OFFBOARD before AUTO.LAND confirmation; automatic requests cancelled");
      return;
    }
    if (now - started_at_ >= kTotalSeconds) {
      fail("AUTO.LAND not confirmed within 8 seconds; holding current position when valid");
      return;
    }
    if (awaiting_confirmation_ && now >= confirmation_until_) {
      awaiting_confirmation_ = false;
      if (attempts_ >= kMaxAttempts) {
        fail("AUTO.LAND not confirmed after 3 attempts; holding current position when valid");
      } else {
        next_attempt_at_ = now + kRetrySeconds;
        reason_ = "AUTO.LAND mode confirmation timed out; retry pending";
      }
    }
  }

  // The gateway must consume a ready future before calling this. Even after
  // this attempt's two-second deadline, a pending RPC prevents another call.
  bool take_attempt(double now, bool rpc_pending) {
    if (!requesting() || rpc_pending || awaiting_confirmation_ ||
        attempts_ >= kMaxAttempts || now < next_attempt_at_ ||
        now - started_at_ >= kTotalSeconds) return false;
    ++attempts_;
    awaiting_confirmation_ = true;
    confirmation_until_ = now + kConfirmationSeconds;
    reason_ = "AUTO.LAND mode request sent; waiting for actual mode feedback";
    return true;
  }

  void rpc_result(std::uint64_t generation, bool accepted, double now) {
    if (generation != generation_ || !requesting()) return;
    // A late RPC must not extend the transaction's absolute deadline.
    if (now - started_at_ >= kTotalSeconds) {
      fail("AUTO.LAND not confirmed within 8 seconds; holding current position when valid");
      return;
    }
    if (accepted) {
      reason_ = "AUTO.LAND request accepted by MAVROS; waiting for actual mode feedback";
    } else {
      awaiting_confirmation_ = false;
      if (attempts_ >= kMaxAttempts) {
        fail("AUTO.LAND request rejected after 3 attempts; holding current position when valid");
      } else {
        next_attempt_at_ = now + kRetrySeconds;
        reason_ = "AUTO.LAND request rejected; retry pending";
      }
    }
  }

  bool hold_output_allowed(const Health& h) const {
    return (requesting() || failed()) && h.connected && h.offboard && h.pose && h.estimator;
  }
};

}  // namespace native_control
