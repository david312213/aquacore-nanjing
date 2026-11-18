#pragma once
// This adapter is not a second flight controller. px4ctrl retains its upstream
// control law. All quaternion products below map child vectors into parent.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

namespace airseafly_real400 {
using V3 = Eigen::Vector3d;
using Q = Eigen::Quaterniond;
using M3 = Eigen::Matrix3d;
inline void require(bool ok, const char* reason) {
  if (!ok) throw std::runtime_error(reason);
}
inline Q normalized(Q q) {
  require(q.coeffs().allFinite() && q.norm() > 1e-6, "invalid quaternion");
  return q.normalized();
}
inline Q expQ(const V3& r) {
  const double a = r.norm();
  return a < 1e-12 ? Q::Identity() : Q(Eigen::AngleAxisd(a, r / a));
}
inline double angle(Q a, Q b) {
  return 2 * std::acos(std::min(1., std::abs(normalized(a).dot(normalized(b)))));
}
inline bool fresh(double now, double stamp, double age = .08) {
  return std::isfinite(now) && std::isfinite(stamp) && stamp > 0 &&
         now - stamp >= -.02 && now - stamp <= age;
}

struct State {
  double t = 0;
  V3 p = V3::Zero(), v = V3::Zero(), bg = V3::Zero(), ba = V3::Zero();
  V3 gravity = V3(0, 0, -9.81);
  Q q = Q::Identity();
  double accel_scale = 1.;
  void validate() const {
    require(t > 0 && std::isfinite(t), "invalid state timestamp");
    require(p.allFinite() && v.allFinite() && bg.allFinite() && ba.allFinite(),
            "invalid state vector");
    require(gravity.allFinite() && gravity.norm() > 8 && gravity.norm() < 11,
            "invalid gravity");
    require(std::isfinite(accel_scale) && accel_scale > .1 && accel_scale < 20,
            "invalid accelerometer scale");
    normalized(q);
  }
};
struct Imu { double t; V3 acc, gyro; };
struct Prediction { State state; V3 omega = V3::Zero(); };

// Owned by one mutex in the ROS wrapper. Samples remain in DRIVER units;
// the exact FAST-LIO initialization scale is applied once, in propagate().
class Predictor {
 public:
  double horizon = .25, max_gap = .03;
  std::string fault;
  bool initialized() const { return have_anchor_; }
  double latestStamp() const { return imu_.empty() ? 0 : imu_.back().t; }
  double anchorStamp() const { return have_anchor_ ? anchor_.t : 0; }
  void add(const Imu& s) {
    require(fault.empty(), fault.c_str());
    require(std::isfinite(s.t) && s.t > 0 && s.acc.allFinite() && s.gyro.allFinite(),
            "invalid IMU sample");
    if (!imu_.empty() && s.t <= imu_.back().t) {
      fault = "duplicate/reversed IMU timestamp; new session required";
      throw std::runtime_error(fault);
    }
    imu_.push_back(s);
    while (imu_.size() > 2 && s.t - imu_[1].t > 2) imu_.pop_front();
  }
  void correct(const State& state) {
    require(fault.empty(), fault.c_str()); state.validate();
    if (have_anchor_) {
      if (state.t <= anchor_.t || state.t - anchor_.t > horizon) {
        fault = "LIO time reset/correction gap; new session required";
        throw std::runtime_error(fault);
      }
      if (latestStamp() >= state.t) {
        const auto expected = predict(state.t).state;
        if ((expected.p - state.p).norm() > .5 || angle(expected.q, state.q) > .35) {
          fault = "LIO correction jump; new session required";
          throw std::runtime_error(fault);
        }
      }
    }
    anchor_ = state; anchor_.q = normalized(state.q); have_anchor_ = true;
  }
  Prediction predict(double t) const {
    require(fault.empty(), fault.c_str());
    require(have_anchor_, "waiting for LIO posterior");
    require(t >= anchor_.t && t - anchor_.t <= horizon, "prediction horizon exceeded");
    require(imu_.size() >= 2 && imu_.front().t <= anchor_.t && imu_.back().t >= t,
            "IMU buffer does not bracket prediction");
    Prediction result; result.state = anchor_;
    for (size_t i = 1; i < imu_.size(); ++i) {
      const Imu& a = imu_[i-1]; const Imu& b = imu_[i];
      const double left = std::max(a.t, result.state.t), right = std::min(b.t, t);
      if (right <= left) continue;
      require(b.t - a.t <= max_gap, "IMU gap exceeded");
      const double f = ((left + right) * .5 - a.t) / (b.t - a.t);
      const V3 omega = (1-f)*a.gyro + f*b.gyro - anchor_.bg;
      const V3 acc = ((1-f)*a.acc + f*b.acc)*anchor_.accel_scale - anchor_.ba;
      const double dt = right-left;
      const Q qmid = normalized(result.state.q * expQ(omega * dt * .5));
      const V3 aw = qmid * acc + anchor_.gravity;
      result.state.p += result.state.v * dt + aw * (.5*dt*dt);
      result.state.v += aw * dt;
      result.state.q = normalized(result.state.q * expQ(omega*dt));
      result.state.t = right; result.omega = omega;
      if (right >= t) break;
    }
    require(std::abs(result.state.t-t) < 1e-8, "incomplete propagation");
    if (t == anchor_.t) {
      for (const auto& s : imu_) if (s.t >= t) { result.omega = s.gyro-anchor_.bg; break; }
    }
    return result;
  }
 private:
  bool have_anchor_ = false;
  State anchor_;
  std::deque<Imu> imu_;
};

struct BodyState {
  double t = 0;
  V3 p = V3::Zero(), v_world = V3::Zero(), v_body = V3::Zero();
  V3 omega_world = V3::Zero(), omega_body = V3::Zero();
  Q q = Q::Identity();
};
class Frames {
 public:
  // T_BI: Livox IMU origin/axes expressed in PX4-reference FLU body frame.
  Q q_bi = Q::Identity(); V3 t_bi = V3::Zero();
  Q q_ow = Q::Identity(); V3 t_ow = V3::Zero();
  bool initialized = false;
  void initialize(const State& s) {
    require(!initialized, "frame anchor already frozen"); s.validate();
    q_bi = normalized(q_bi); require(t_bi.allFinite(), "invalid mount");
    const Q q_wb = normalized(s.q * q_bi.conjugate());
    const V3 up = -s.gravity.normalized();
    V3 forward = q_wb * V3::UnitX(); forward -= up * forward.dot(up);
    require(forward.norm() > .5, "initial aircraft heading is near vertical");
    forward.normalize(); const V3 left = up.cross(forward).normalized();
    M3 r; r.row(0) = forward.transpose(); r.row(1) = left.transpose(); r.row(2) = up.transpose();
    q_ow = normalized(Q(r));
    t_ow = -(q_ow * (s.p - q_wb*t_bi)); initialized = true;
  }
  BodyState transform(const State& s, const V3& omega_i) const {
    require(initialized, "frame anchor not initialized"); s.validate();
    require(omega_i.allFinite(), "invalid angular velocity");
    const Q q_wb = normalized(s.q * q_bi.conjugate());
    const V3 lever = q_wb * t_bi, omega_w = s.q * omega_i;
    BodyState o; o.t = s.t;
    o.q = normalized(q_ow*q_wb);
    o.p = q_ow*(s.p-lever)+t_ow;
    o.v_world = q_ow*(s.v-omega_w.cross(lever));
    o.v_body = o.q.conjugate()*o.v_world;
    o.omega_body = q_bi*omega_i; o.omega_world = o.q*o.omega_body;
    return o;
  }
  V3 scanPoint(const State& scan, const V3& point_i) const {
    require(initialized, "frame anchor not initialized");
    return q_ow*(scan.q*point_i+scan.p)+t_ow;
  }
};

struct AttitudeSample { double t; Q q; V3 acceleration; };
class AttitudeHistory {
 public:
  double max_span = .04;
  void add(AttitudeSample s) {
    s.q = normalized(s.q);
    require(s.t > 0 && std::isfinite(s.t) && s.acceleration.allFinite(), "bad PX4 attitude");
    require(samples_.empty() || s.t > samples_.back().t, "PX4 clock reversed/duplicate");
    samples_.push_back(s);
    while (samples_.size() > 2 && s.t-samples_[1].t > 2) samples_.pop_front();
  }
  double latestStamp() const { return samples_.empty() ? 0 : samples_.back().t; }
  AttitudeSample sample(double t) const {
    for (const auto& s : samples_) if (std::abs(s.t-t)<1e-8) return s;
    for (size_t i=1; i<samples_.size(); ++i) {
      const auto& a=samples_[i-1]; const auto& b=samples_[i];
      if (a.t<t && t<b.t) {
        require(b.t-a.t<=max_span, "PX4 attitude interpolation gap");
        require(angle(a.q,b.q)<.2617993878, "cannot interpolate across PX4 reference reset");
        const double f=(t-a.t)/(b.t-a.t);
        return {t, normalized(a.q.slerp(f,b.q)), (1-f)*a.acceleration+f*b.acceleration};
      }
    }
    throw std::runtime_error("no bracketed PX4 attitude");
  }
 private:
  std::deque<AttitudeSample> samples_;
};
class BodyHistory {
 public:
  void add(const BodyState& s) {
    require(s.t>0 && std::isfinite(s.t) && s.p.allFinite() && s.v_world.allFinite(), "bad body state");
    normalized(s.q);
    require(samples_.empty() || s.t>samples_.back().t, "body odometry time reset");
    samples_.push_back(s);
    while(samples_.size()>2 && s.t-samples_[1].t>2) samples_.pop_front();
  }
  double latestStamp() const { return samples_.empty()?0:samples_.back().t; }
  BodyState sample(double t) const {
    for(const auto& s:samples_) if(std::abs(s.t-t)<1e-8) return s;
    for(size_t i=1;i<samples_.size();++i) {
      const auto& a=samples_[i-1]; const auto& b=samples_[i];
      if(a.t<t && t<b.t) {
        require(b.t-a.t<=.03, "body interpolation gap");
        require((b.p-a.p).norm()<.5 && angle(a.q,b.q)<.35, "body interpolation discontinuity");
        double f=(t-a.t)/(b.t-a.t); BodyState o;
        o.t=t; o.p=(1-f)*a.p+f*b.p; o.v_world=(1-f)*a.v_world+f*b.v_world;
        o.q=normalized(a.q.slerp(f,b.q)); o.v_body=o.q.conjugate()*o.v_world;
        return o;
      }
    }
    throw std::runtime_error("no bracketed body odometry");
  }
 private:
  std::deque<BodyState> samples_;
};
struct PairedState { BodyState body; AttitudeSample px4; };
inline PairedState pairStates(const BodyHistory& lio, const AttitudeHistory& px4, double now) {
  double t=std::min(lio.latestStamp(),px4.latestStamp());
  require(fresh(now,t), "no fresh common LIO/PX4 time");
  return {lio.sample(t),px4.sample(t)};
}

struct RcDecision {
  bool hover=false, command=false, hover_rising=false, command_rising=false, centered=false;
};
class RcGate {
 public:
  RcDecision update(const std::vector<uint16_t>& channels) {
    require(channels.size()>=8, "RC message has fewer than eight channels");
    for(size_t i=0;i<8;++i) require(channels[i]>=800 && channels[i]<=2200,"invalid RC PWM");
    bool h=channels[4]>1700, c=channels[5]>1700;
    if(!h) seen_hover_low_=true;
    if(!c) seen_command_low_=true;
    RcDecision d;
    d.hover=h && seen_hover_low_; d.command=d.hover && c && seen_command_low_;
    d.hover_rising=d.hover && !last_h_; d.command_rising=d.command && !last_c_;
    d.centered=true;
    for(size_t i=0;i<4;++i) d.centered=d.centered && std::abs(int(channels[i])-1500)<=50;
    last_h_=d.hover; last_c_=d.command;
    return d; // Channel 8 deliberately has no command, reboot, or arm action.
  }
 private:
  bool seen_hover_low_=false, seen_command_low_=false, last_h_=false, last_c_=false;
};
}  // namespace airseafly_real400
