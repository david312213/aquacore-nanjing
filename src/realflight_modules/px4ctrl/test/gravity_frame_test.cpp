#include "controller.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

// Function-level tests only. No ros::init, publishers, services or vehicle.
namespace {
const double pi = std::acos(-1.0);
Eigen::Quaterniond attitude(double yaw, double pitch = 0., double roll = 0.) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}
Parameter_t parameters() {
  Parameter_t p{};
  p.gra = 9.81;
  p.thr_map.hover_percentage = 0.58;
  p.gain.Kp0 = p.gain.Kp1 = p.gain.Kp2 = 2.;
  p.gain.Kv0 = p.gain.Kv1 = p.gain.Kv2 = 1.5;
  return p;
}
Eigen::Vector3d force(const Controller_Output_t &u, const Parameter_t &p) {
  return u.q * Eigen::Vector3d::UnitZ() *
    (u.thrust * p.gra / p.thr_map.hover_percentage);
}
}

int main(int argc, char **argv) {
  ros::Time::init();
  const bool baseline = argc > 1 && std::string(argv[1]) == "--baseline";
  const char *fixture = argc > (baseline ? 2 : 1) ? argv[baseline ? 2 : 1] : nullptr;
  Parameter_t p = parameters();
  LinearControl controller(p);
  int cases = 0, failures = 0, recorded = 0;
  double max_force_error = 0., max_yaw_error = 0., max_recorded_horizontal = 0.;

  auto exercise = [&](const Eigen::Quaterniond &odom_q,
                      const Eigen::Quaterniond &imu_q,
                      const Eigen::Vector3d &dp,
                      const Eigen::Vector3d &dv,
                      const Eigen::Vector3d &acceleration, double desired_yaw,
                      bool is_recorded) {
    Odom_Data_t odom;
    odom.p = Eigen::Vector3d(1.2, -.8, .65);
    odom.v = Eigen::Vector3d(.1, -.2, .03);
    odom.q = odom_q;
    Imu_Data_t imu;
    imu.q = imu_q;
    Desired_State_t des(odom);
    des.p += dp;
    des.v = odom.v + dv;
    des.a = acceleration;
    des.yaw = desired_yaw;
    Controller_Output_t u;
    controller.calculateControl(des, odom, imu, u);

    // Independent physical oracle: inertial acceleration equals thrust minus
    // vertical gravity. Rotate only the requested translational acceleration.
    const Eigen::Quaterniond frame = (imu_q * odom_q.inverse()).normalized();
    const Eigen::Vector3d expected_acc = frame * (2. * dp + 1.5 * dv + acceleration);
    const Eigen::Vector3d actual_acc = force(u, p) - Eigen::Vector3d(0., 0., p.gra);
    const double error = (actual_acc - expected_acc).norm();
    const Eigen::Matrix3d raw_rotation = (frame.inverse() * u.q).toRotationMatrix();
    const double yaw_error = std::abs(std::atan2(
      std::sin(std::atan2(raw_rotation(1, 0), raw_rotation(0, 0)) - desired_yaw),
      std::cos(std::atan2(raw_rotation(1, 0), raw_rotation(0, 0)) - desired_yaw)));
    ++cases;
    if (!u.q.coeffs().allFinite() || !std::isfinite(u.thrust) ||
        std::abs(u.q.norm() - 1.) > 1e-10 || error > 1e-8 || yaw_error > 1e-8)
      ++failures;
    max_force_error = std::max(max_force_error, error);
    max_yaw_error = std::max(max_yaw_error, yaw_error);
    if (is_recorded) {
      ++recorded;
      max_recorded_horizontal = std::max(max_recorded_horizontal, actual_acc.head<2>().norm());
    }
  };

  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  // Level/yaw-only frames retain the previous behavior, including axis signs.
  for (int actual : {-179, -90, 0, 90, 179})
    for (int target : {-179, -90, 0, 90, 179})
      for (int frame_yaw : {-120, 0, 120})
        for (const Eigen::Vector3d &a : {zero, Eigen::Vector3d(.4, 0., 0.),
            Eigen::Vector3d(0., .4, 0.), Eigen::Vector3d(0., -.4, 0.),
            Eigen::Vector3d(-.4, 0., 0.), Eigen::Vector3d(0., 0., -.4)}) {
          const Eigen::Quaterniond q = attitude(actual * pi / 180., .1, -.12);
          exercise(q, attitude(frame_yaw * pi / 180.) * q, zero, zero, a,
                   target * pi / 180., false);
        }

  std::mt19937 rng(20261005);
  std::uniform_real_distribution<double> yaw(-pi, pi), tilt(-.2, .2), error(-.2, .2);
  for (int i = 0; i < 2000; ++i) {
    const Eigen::Quaterniond q = attitude(yaw(rng), tilt(rng), tilt(rng));
    const Eigen::Quaterniond frame = attitude(yaw(rng), tilt(rng), tilt(rng));
    exercise(q, frame * q, Eigen::Vector3d(error(rng), error(rng), error(rng)),
             Eigen::Vector3d(error(rng), error(rng), error(rng)),
             Eigen::Vector3d(error(rng), error(rng), error(rng)), yaw(rng), false);
  }

  if (fixture) {
    std::ifstream input(fixture);
    if (!input) { std::cerr << "Cannot open attitude fixture\n"; return 2; }
    double ox, oy, oz, ow, ix, iy, iz, iw;
    while (input >> ox >> oy >> oz >> ow >> ix >> iy >> iz >> iw)
      exercise(Eigen::Quaterniond(ow, ox, oy, oz), Eigen::Quaterniond(iw, ix, iy, iz),
               zero, zero, zero, 0., true);
    if (!input.eof()) { std::cerr << "Invalid attitude fixture\n"; return 2; }
  }

  // Ideal translational plant: constant tilted map, perfect attitude tracking.
  // This checks closed-loop equilibrium, not real-aircraft flight performance.
  double max_final_error = 0.;
  for (double roll : {-.08, .08}) {
    const Eigen::Quaterniond frame = attitude(.4, -.06, roll);
    LinearControl c(p);
    Odom_Data_t odom;
    odom.p = Eigen::Vector3d(.1, -.2, .7);
    odom.v.setZero();
    odom.q = frame.inverse();
    Imu_Data_t imu;
    imu.q.setIdentity();
    const Eigen::Vector3d goal(0., 0., .65);
    const double dt = .005;
    for (int i = 0; i < 6000; ++i) {
      Desired_State_t des(odom);
      des.p = goal;
      des.v.setZero();
      des.a.setZero();
      des.yaw = 0.;
      Controller_Output_t u;
      c.calculateControl(des, odom, imu, u);
      const Eigen::Vector3d acc = frame.inverse() *
        (force(u, p) - Eigen::Vector3d(0., 0., p.gra));
      odom.p += odom.v * dt + .5 * acc * dt * dt;
      odom.v += acc * dt;
      imu.q = u.q;
      odom.q = frame.inverse() * u.q;
    }
    const double final_error = (odom.p - goal).norm();
    ++cases;
    if (!std::isfinite(final_error) || final_error > 1e-5) ++failures;
    max_final_error = std::max(max_final_error, final_error);
  }

  std::cout << std::setprecision(12)
    << "{\"variant\":\"" << (baseline ? "before" : "after")
    << "\",\"cases\":" << cases << ",\"failures\":" << failures
    << ",\"recorded_ground_attitude_pairs\":" << recorded
    << ",\"max_force_error_mps2\":" << max_force_error
    << ",\"max_yaw_error_rad\":" << max_yaw_error
    << ",\"max_zero_error_ground_lateral_mps2\":" << max_recorded_horizontal
    << ",\"ideal_plant_final_position_error_m\":" << max_final_error
    << ",\"real_flight_validated\":false}\n";
  return baseline ? (failures > 0 ? 0 : 2) : (failures == 0 ? 0 : 1);
}
