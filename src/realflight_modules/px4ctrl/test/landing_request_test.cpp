#include "../src/landing_request.h"
#include <cassert>
#include <cstring>

using native_control::Health;
using native_control::LandingRequest;

static Health airborne() {
  Health h;
  h.connected = h.armed = h.offboard = h.pose = h.estimator = true;
  return h;
}

static void rejected_then_confirmed() {
  LandingRequest land;
  auto h = airborne();
  assert(!std::strcmp(land.name(), "idle"));
  land.start(100.);
  assert(!std::strcmp(land.name(), "requesting"));
  assert(land.take_attempt(100., false));
  const auto generation = land.generation();
  land.rpc_result(generation, false, 100.01);
  assert(land.requesting() && land.attempts() == 1);
  assert(!land.take_attempt(100.50, false));
  assert(land.take_attempt(100.51, false));
  land.rpc_result(generation, true, 100.52);
  assert(land.requesting());  // mode_sent never confirms the flight mode.
  assert(land.hold_output_allowed(h));
  h.offboard = false; h.auto_land = true;
  land.update(100.6, h);
  assert(!std::strcmp(land.name(), "confirmed") && land.attempts() == 2);
  assert(!land.hold_output_allowed(h));
  assert(!land.take_attempt(110., false));
  land.rpc_result(generation, false, 111.);  // Actual mode beats a late rejection.
  assert(!std::strcmp(land.name(), "confirmed"));
  land.update(112., h, true);  // An on-ground flag while armed is not completion.
  assert(land.reason().find("completed") == std::string::npos);
  h.armed = false;
  land.update(113., h, true);
  assert(land.reason().find("on ground and disarmed") != std::string::npos);
  h.auto_land = false;
  land.update(114., h, true);  // Normal post-disarm mode change is not a takeover.
  assert(!std::strcmp(land.name(), "confirmed"));
}

static void no_mode_feedback_is_bounded() {
  LandingRequest land;
  auto h = airborne();
  land.start(0.);
  for (unsigned i = 0; i < 3; ++i) {
    const double started = i * 2.5;
    assert(land.take_attempt(started, false));
    land.rpc_result(land.generation(), true, started + .01);
    land.update(started + 1.999, h);
    assert(land.requesting() && land.hold_output_allowed(h));
    assert(!land.take_attempt(started + 1.999, false));
    land.update(started + 2., h);
    if (i < 2) {
      assert(land.requesting());
      assert(!land.take_attempt(started + 2.499, false));
    }
  }
  assert(land.failed() && land.attempts() == 3);
  assert(!land.take_attempt(100., false));
  assert(land.hold_output_allowed(h));  // Failure retains a valid position hold.
  h.pose = false;
  assert(!land.hold_output_allowed(h));
  h.pose = true; h.estimator = false;
  assert(!land.hold_output_allowed(h));
  h.estimator = true; h.offboard = false;
  assert(!land.hold_output_allowed(h));
}

static void three_rejections_stop() {
  LandingRequest land;
  const auto h = airborne();
  land.start(0.);
  for (unsigned i = 0; i < 3; ++i) {
    const double now = static_cast<double>(i);
    land.update(now, h);
    assert(land.take_attempt(now, false));
    land.rpc_result(land.generation(), false, now + .1);
  }
  assert(land.failed() && land.attempts() == 3);
  assert(land.reason().find("rejected after 3 attempts") != std::string::npos);
  for (double now : {3., 8., 100.}) {
    land.update(now, h);
    assert(!land.take_attempt(now, false));
    assert(land.hold_output_allowed(h));
  }
}

static void hanging_rpc_obeys_absolute_deadline() {
  LandingRequest land;
  const auto h = airborne();
  land.start(20.);
  assert(land.take_attempt(20., false));
  const auto generation = land.generation();
  for (double now : {22., 22.5, 25., 27.999}) {
    land.update(now, h);
    assert(land.requesting() && land.hold_output_allowed(h));
    assert(!land.take_attempt(now, true));  // Never overlap a blocked RPC.
  }
  land.update(28., h);
  assert(land.failed() && land.attempts() == 1);
  assert(land.reason().find("8 seconds") != std::string::npos);
  const auto reason = land.reason();
  land.rpc_result(generation, true, 29.);
  assert(land.failed() && land.reason() == reason);
  auto late = h; late.auto_land = true; late.offboard = false;
  land.update(29., late);  // Late feedback cannot revive a terminal transaction.
  assert(land.failed() && !land.hold_output_allowed(late));
}

static void total_deadline_includes_retry_delays() {
  LandingRequest land;
  const auto h = airborne();
  land.start(0.);
  assert(land.take_attempt(0., false));
  land.update(2., h);
  land.rpc_result(land.generation(), false, 2.5);
  assert(land.take_attempt(3., false));
  land.update(5., h);
  land.rpc_result(land.generation(), false, 6.);
  assert(land.take_attempt(6.5, false));
  land.rpc_result(land.generation(), true, 6.6);
  land.update(7.999, h);
  assert(land.requesting());
  land.update(8., h);  // Do not extend this to the last attempt's 8.5 s deadline.
  assert(land.failed() && land.attempts() == 3);
}

static void pilot_cancel_and_old_results_never_restart() {
  LandingRequest land;
  auto h = airborne();
  land.start(0.);
  assert(land.take_attempt(0., false));
  const auto old_generation = land.generation();
  h.offboard = false;
  land.update(.1, h);
  assert(!std::strcmp(land.name(), "cancelled"));
  h.offboard = true;
  land.rpc_result(old_generation, true, .2);
  land.update(1., h);
  assert(!std::strcmp(land.name(), "cancelled"));
  assert(!land.take_attempt(1., false) && !land.hold_output_allowed(h));
  land.start(10.);  // Only a new explicit request opens another generation.
  assert(land.generation() != old_generation && land.attempts() == 0);
  assert(!land.take_attempt(10., true));
  const auto reason = land.reason();
  land.rpc_result(old_generation, true, 10.1);
  assert(land.reason() == reason && land.attempts() == 0);
  assert(land.take_attempt(10.1, false));
  land.cancel("explicit hold");
  land.rpc_result(land.generation(), false, 10.2);
  assert(!std::strcmp(land.name(), "cancelled"));
  land.reset();
  assert(!std::strcmp(land.name(), "idle") && land.attempts() == 0);
}

static void mode_confirmation_and_connection_health() {
  LandingRequest land;
  auto h = airborne();
  h.auto_land = true; h.offboard = false;
  land.start(0.); land.update(0., h);
  assert(!std::strcmp(land.name(), "confirmed") && land.attempts() == 0);
  h.auto_land = false;
  land.update(.1, h);
  assert(!std::strcmp(land.name(), "cancelled"));
  h = airborne();
  land.start(10.); assert(land.take_attempt(10., false));
  h.connected = false; h.auto_land = true;
  land.update(10.1, h);
  assert(land.failed());  // A stale cached AUTO.LAND cannot confirm the request.
  assert(!land.hold_output_allowed(h));
  h = airborne();
  land.start(20.); assert(land.take_attempt(20., false));
  h.auto_land = true; h.offboard = false;
  land.update(28., h);  // Observed mode at the deadline beats an RPC timeout.
  assert(!std::strcmp(land.name(), "confirmed"));
}

static void ground_disarmed_completes_without_intermediate_mode_sample() {
  for (bool connected : {true, false}) {
    LandingRequest land;
    auto h = airborne();
    land.start(0.); assert(land.take_attempt(0., false));
    h.armed = h.offboard = false;
    h.connected = connected;
    land.update(.1, h, true);
    assert(!std::strcmp(land.name(), "confirmed"));
    assert(land.reason().find("on ground and disarmed") != std::string::npos);
    assert(!land.take_attempt(1., false));
  }
}

int main() {
  rejected_then_confirmed();
  no_mode_feedback_is_bounded();
  three_rejections_stop();
  hanging_rpc_obeys_absolute_deadline();
  total_deadline_includes_retry_delays();
  pilot_cancel_and_old_results_never_restart();
  mode_confirmation_and_connection_health();
  ground_disarmed_completes_without_intermediate_mode_sample();
}
