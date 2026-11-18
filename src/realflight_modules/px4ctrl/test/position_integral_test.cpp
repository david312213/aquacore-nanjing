#include "../src/position_integral.h"
#include <iostream>
#include <limits>
#include <string>

using airseafly_control::IntegralVector;
using airseafly_control::PositionIntegral;
using airseafly_control::PositionIntegralConfig;

namespace {
int cases = 0, failures = 0;
const IntegralVector zero{{0., 0., 0.}}, gravity{{0., 0., 9.81}};
void check(bool ok, const std::string &why) {
  ++cases;
  if (!ok) { ++failures; std::cerr << "FAIL: " << why << '\n'; }
}
bool near(double a, double b, double tolerance = 1e-10) { return std::abs(a-b) < tolerance; }
PositionIntegralConfig config() {
  PositionIntegralConfig c;
  c.enabled = true;
  c.ki = {{.15, .15, 0.}};
  c.max_acceleration = {{.4, .4, 0.}};
  return c;
}
IntegralVector tilt_limit(IntegralVector force) {
  force[2] = std::max(0., force[2]);
  const double horizontal = std::hypot(force[0], force[1]);
  const double bound = force[2]*std::tan(std::acos(-1.)/6.);
  if (horizontal > bound) {
    force[0] *= bound/horizontal;
    force[1] *= bound/horizontal;
  }
  return force;
}
double rate_trial(double dt) {
  PositionIntegral integral(config());
  integral.update({{.2, -.1, 2.}}, 1.);
  for (int n = 1; n <= int(std::round(1./dt)); ++n)
    integral.update({{.2, -.1, 2.}}, 1.+n*dt);
  check(near(integral.acceleration()[2], 0.), "disabled Z cannot accumulate even with a large Z error");
  return integral.acceleration()[0];
}

// Closed-loop double-integrator plant with 80 ms actuator lag, constant force
// disturbance and a setpoint change. This exercises the actual I helper; it
// is a synthetic dynamics regression, not an aircraft identification/flight.
IntegralVector simulate(bool use_integral, double dt, double start = 0., bool move_target = false) {
  auto cfg = config(); cfg.enabled = use_integral;
  PositionIntegral integral(cfg);
  IntegralVector p{{start, 0., 0.}}, v=zero, applied=zero, target=zero;
  const IntegralVector disturbance{{.30, -.20, .10}};
  const int steps = int(std::round(120./dt));
  for (int n = 0; n < steps; ++n) {
    if (move_target && n*dt >= 40.) target = {{.8, -.4, 0.}};
    IntegralVector error, base, command;
    for (int i = 0; i < 3; ++i) {
      error[i] = target[i]-p[i];
      base[i] = 2.*error[i]-1.5*v[i]+gravity[i];
    }
    const auto bias = integral.update(error, n*dt);
    for (int i = 0; i < 3; ++i) {
      command[i] = base[i]+bias[i];
      if (std::abs(bias[i]) > cfg.max_acceleration[i]+1e-12)
        check(false, "closed-loop integral exceeded its configured limit");
    }
    command = tilt_limit(command);
    for (int i = 0; i < 3; ++i) {
      applied[i] += (command[i]-gravity[i]-applied[i])*dt/.08;
      v[i] += (applied[i]+disturbance[i])*dt;
      p[i] += v[i]*dt;
    }
  }
  for (int i = 0; i < 3; ++i) p[i] -= target[i];
  return p;
}
}

int main() {
  check(near(rate_trial(.01), .03), "100 Hz integrates actual seconds");
  check(near(rate_trial(.02), .03), "50 Hz gives the same integral");
  auto cfg = config();
  PositionIntegral integral(cfg);
  double t = 1.;
  const IntegralVector error{{.2, -.1, 0.}};
  check(integral.update(error, t) == zero,
        "first control cycle primes the clock without a kick");
  for (int n = 0; n < 100; ++n) {
    const double dt = n%2 ? .017 : .003;
    integral.update(error, t+=dt);
  }
  check(near(integral.acceleration()[0], .03) && near(integral.acceleration()[1], -.015),
        "jittered loop integrates elapsed time independently per axis");
  const auto learned = integral.acceleration();
  check(integral.update(error, t) == learned,
        "paused simulation does not accumulate or clear bias");
  integral.update({{.6, .0, .0}}, t+=.01);
  check(near(integral.acceleration()[0], learned[0]+.0009),
        "large setpoint change continues integrating");
  integral.update({{.4, .4, .0}}, t+=.01);
  check(near(integral.acceleration()[0], learned[0]+.0015) &&
        near(integral.acceleration()[1], learned[1]+.0006),
        "diagonal error above the former norm threshold continues integrating");
  const auto before_gap = integral.acceleration();
  integral.update(error, t+=.5);
  check(near(integral.acceleration()[0], before_gap[0]+.015) &&
        near(integral.acceleration()[1], before_gap[1]-.0075),
        "long control interval contributes its full elapsed time");
  check(integral.update(error, t-=1.) == zero,
        "backwards clock clears history");
  for (int n = 0; n < 3000; ++n)
    integral.update(error, t+=.01);
  check(near(integral.acceleration()[0], .4) && near(integral.acceleration()[1], -.4),
        "persistent unreachable error is clamped on both signs");
  integral.update({{-.2, .1, 0.}}, t+=.01);
  check(integral.acceleration()[0] < .4 && integral.acceleration()[1] > -.4,
        "reversed error immediately unwinds a full integral");
  integral.reset();
  check(integral.acceleration() == zero, "explicit ownership/thrust-map reset clears all axes");

  check(integral.update(error, t+=.01) == zero,
        "explicit reset primes a new control interval");
  for (int field = 0; field < 2; ++field) {
    PositionIntegral bad(cfg);
    bad.update(error, 1.);
    bad.update(error, 1.01);
    IntegralVector e=error;
    double now=1.02, nan=std::numeric_limits<double>::quiet_NaN();
    if (field==0) e[0]=nan;
    if (field==1) now=nan;
    check(bad.update(e, now) == zero, "nonfinite arithmetic input clears integral");
  }
  PositionIntegral legacy(PositionIntegralConfig{});
  for (int n=0; n<100; ++n) legacy.update(error, n*.01);
  check(legacy.acceleration() == zero, "missing configuration preserves legacy PD exactly");
  cfg.enabled=false;
  PositionIntegral disabled(cfg);
  for (int n=0; n<100; ++n) disabled.update(error, n*.01);
  check(disabled.acceleration() == zero, "explicit disable restores PD despite nonzero Ki");
  for (int field=0; field<4; ++field) {
    auto invalid=config();
    if(field==0) invalid.ki[0]=-1.;
    if(field==1) invalid.ki[0]=std::numeric_limits<double>::quiet_NaN();
    if(field==2) invalid.max_acceleration[0]=0.;
    if(field==3) invalid.max_acceleration[1]=-1.;
    bool rejected=false;
    try { PositionIntegral invalid_integral(invalid); }
    catch(const std::invalid_argument &) { rejected=true; }
    check(rejected, "invalid integral settings rejected before control starts");
  }

  const auto pd = simulate(false, .01), pid = simulate(true, .01);
  check(near(pd[0], .15, .001) && near(pd[1], -.10, .001), "PD leaves expected constant-disturbance offset");
  check(std::hypot(pid[0], pid[1]) < .002, "position I rejects a constant XY disturbance");
  check(near(pid[2], pd[2], .000001), "disabled Z retains the previous altitude response");
  const auto slow = simulate(true, .02), far = simulate(true, .01, 2.0), moved = simulate(true, .01, 0., true);
  check(std::hypot(slow[0], slow[1]) < .002, "disturbance rejection survives a 50 Hz loop");
  check(std::hypot(far[0], far[1]) < .002, "large initial error integrates continuously and settles");
  check(std::hypot(moved[0], moved[1]) < .002, "learned bias survives a target move and settles at the new target");
  std::cout << "{\"cases\":" << cases << ",\"failures\":" << failures
            << ",\"synthetic_pd_xy_error_m\":" << std::hypot(pd[0],pd[1])
            << ",\"synthetic_pid_xy_error_m\":" << std::hypot(pid[0],pid[1])
            << ",\"real_flight_validated\":false}\n";
  return failures ? 1 : 0;
}
