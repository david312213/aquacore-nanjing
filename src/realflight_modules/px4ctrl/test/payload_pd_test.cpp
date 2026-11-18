#include "../src/payload_pd.h"
#include <iostream>
#include <limits>

int main() {
  int failures = 0;
  auto check = [&](bool condition, const char *name) {
    if (!condition) { ++failures; std::cerr << "FAIL: " << name << '\n'; }
  };
  const payload_pd::Gains base{{{2., 2., 2.}}, {{1.5, 1.5, 1.5}}};
  std::array<payload_pd::Gains, 4> profiles;
  for (unsigned n = 0; n <= 3; ++n)
    profiles[n] = {{{1. + n, 2. + n, 3. + n}}, {{.5 + n, 1. + n, 1.5 + n}}};
  payload_pd::Schedule schedule(base, profiles, true, 1.);
  std::string message;
  check(!schedule.ready(), "restart requires declared inventory");
  check(!schedule.release(2, 0., message), "uninitialized release rejected");
  check(schedule.gains(0.).kp == base.kp, "uninitialized retains base PD");
  check(!schedule.reset(7, false, 0., message), "airborne reset rejected");
  check(!schedule.reset(8, true, 0., message), "invalid inventory rejected");
  check(schedule.reset(7, true, 0., message), "ground reset accepted");
  const unsigned flight_generation = schedule.generation();
  check(schedule.remaining() == 3 && schedule.gains(0.).kp == profiles[3].kp, "full profile selected");
  check(!schedule.release(0, 0., message) && !schedule.release(4, 0., message), "invalid slots rejected");
  check(schedule.release(2, 10., message), "first release");
  check(schedule.remaining() == 2 && schedule.loadedMask() == 5, "physical slot two counted once");
  check(schedule.gains(10.).kp == profiles[3].kp, "no immediate gain jump");
  check(std::abs(schedule.gains(10.5).kp[0] - 3.5) < 1e-12, "smooth midpoint");
  check(schedule.release(2, 10.5, message), "duplicate ACK idempotent");
  check(schedule.gains(11.).kp == profiles[2].kp, "duplicate does not restart ramp");
  check(schedule.release(1, 11., message), "second release");
  const auto mid = schedule.gains(11.25);
  check(schedule.release(3, 11.25, message), "third release during ramp");
  check(schedule.gains(11.25).kp == mid.kp, "interrupted ramp is continuous");
  check(schedule.remaining() == 0 && schedule.loadedMask() == 0, "empty inventory");
  check(schedule.gains(12.25).kp == profiles[0].kp &&
        schedule.gains(12.25).kv == profiles[0].kv, "all axes reach empty profile");
  check(!schedule.reset(7, false, 20., message) && schedule.remaining() == 0, "cannot reload in flight");
  check(schedule.reset(4, true, 20., message), "partial load reset");
  check(schedule.generation() != flight_generation, "reload invalidates old inventory generation");
  check(!schedule.release(2, 21., message) && schedule.remaining() == 1, "unloaded slot rejected");
  check(schedule.release(3, 21., message) && schedule.remaining() == 0, "single loaded slot release");

  std::array<payload_pd::Gains, 4> same{{base, base, base, base}};
  payload_pd::Schedule inherited(base, same, true, 1.);
  inherited.reset(7, true, 0., message);
  for (unsigned slot : {2u, 1u, 3u}) {
    inherited.release(slot, slot, message);
    check(inherited.gains(slot + .3).kp == base.kp && inherited.gains(slot + .3).kv == base.kv,
          "inherited profiles preserve original gains");
  }
  payload_pd::Schedule disabled(base, profiles, false, 1.);
  check(!disabled.reset(7, true, 0., message) && !disabled.release(2, 1., message), "disabled cannot mutate");
  check(disabled.gains(100.).kp == base.kp, "disabled retains legacy gains");
  bool rejected = false;
  profiles[1].kv[2] = std::numeric_limits<double>::quiet_NaN();
  try { payload_pd::Schedule invalid(base, profiles, true, 1.); }
  catch (const std::invalid_argument &) { rejected = true; }
  check(rejected, "NaN gains rejected before flight");
  rejected = false;
  try { payload_pd::Schedule invalid(base, same, true, 0.); }
  catch (const std::invalid_argument &) { rejected = true; }
  check(rejected, "zero transition rejected");
  std::cout << "payload_pd_test: " << failures << " failures\n";
  return failures ? 1 : 0;
}
