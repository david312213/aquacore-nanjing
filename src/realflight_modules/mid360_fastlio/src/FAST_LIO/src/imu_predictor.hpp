#pragma once

// Opt-in IMU re-propagation; derived from the separately reviewed Oct 8 trial.
// A short-horizon propagation model, not another EKF or proof of flight accuracy.
// Anchor: corrected IMU-body pose in world at the LiDAR scan-end timestamp.
// Samples: the exact time-adjusted and callback-scaled measurements passed to ImuProcess.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>
#include "prediction_covariance.hpp"

namespace lio_prediction {

struct Vec3 {
  double x = 0, y = 0, z = 0;
  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  Vec3 operator+(const Vec3 &b) const { return {x+b.x, y+b.y, z+b.z}; }
  Vec3 operator-(const Vec3 &b) const { return {x-b.x, y-b.y, z-b.z}; }
  Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
  double squaredNorm() const { return x*x+y*y+z*z; }
  bool finite() const { return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z); }
};
inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
  return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}

struct Quaternion {
  double w = 1, x = 0, y = 0, z = 0;
  Quaternion() = default;
  Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}
  double squaredNorm() const { return w*w+x*x+y*y+z*z; }
  bool finite() const {
    return std::isfinite(w)&&std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z);
  }
  Quaternion normalized() const {
    const double n = std::sqrt(squaredNorm());
    return {w/n, x/n, y/n, z/n};
  }
  Quaternion operator*(const Quaternion &b) const {
    return {w*b.w-x*b.x-y*b.y-z*b.z, w*b.x+x*b.w+y*b.z-z*b.y,
            w*b.y-x*b.z+y*b.w+z*b.x, w*b.z+x*b.y-y*b.x+z*b.w};
  }
  Vec3 rotate(const Vec3 &v) const {
    const Vec3 qv{x,y,z};
    const Vec3 t = cross(qv,v)*2;
    return v+t*w+cross(qv,t);
  }
  Vec3 inverseRotate(const Vec3 &v) const { return Quaternion{w,-x,-y,-z}.rotate(v); }
  Rotation3 rotationMatrix() const {
    Rotation3 r;
    const Vec3 columns[3]={rotate({1,0,0}),rotate({0,1,0}),rotate({0,0,1})};
    for (int j=0; j<3; ++j) { r(0,j)=columns[j].x; r(1,j)=columns[j].y; r(2,j)=columns[j].z; }
    return r;
  }
  static Quaternion exp(const Vec3 &rotation_vector) {
    const double angle = std::sqrt(rotation_vector.squaredNorm());
    const double scale = angle < 1e-8 ? 0.5-angle*angle/48.0 : std::sin(angle*0.5)/angle;
    return Quaternion{std::cos(angle*0.5), rotation_vector.x*scale,
                      rotation_vector.y*scale, rotation_vector.z*scale}.normalized();
  }
};

struct Sample {
  double t = 0;
  Vec3 accel, gyro;
};
struct State {
  double t = 0;
  Vec3 position, velocity;
  Quaternion orientation;  // IMU body -> world
  Vec3 accel_bias, gyro_bias, gravity{0,0,-9.81};
  double accel_scale = 1;  // Exactly ImuProcess's G_m_s2 / mean_acc.norm().
  bool covariance_valid = false;
  Covariance18 covariance;
  ProcessNoise12 process_noise;
};
struct Prediction : State {
  Vec3 angular_velocity_body;
  Vec3 velocityBody() const { return orientation.inverseRotate(velocity); }
  PoseCovariance poseCovariance() const { return poseCovarianceWorld(covariance,orientation.rotationMatrix()); }
};
struct Options {
  double history_seconds = 3.0;
  double max_imu_gap = 0.05;
  double max_prediction_horizon = 1.0;
  std::size_t max_samples = 4096;
};
enum class Status {
  Queued, Ready, Duplicate, TooOld, Invalid, OldCorrection, MissingBracket,
  ImuGap, CorrectionExpired
};
inline const char *statusName(Status s) {
  switch(s) {
    case Status::Queued: return "queued";
    case Status::Ready: return "ready";
    case Status::Duplicate: return "duplicate";
    case Status::TooOld: return "too_old";
    case Status::Invalid: return "invalid";
    case Status::OldCorrection: return "old_correction";
    case Status::MissingBracket: return "missing_bracket";
    case Status::ImuGap: return "imu_gap";
    case Status::CorrectionExpired: return "correction_expired";
  }
  return "unknown";
}

// Matches the supplied FAST-LIO callback, including its optional software-sync override.
// No assumption that either offset is zero. Call once and use the result on BOTH paths.
inline double adjustedImuTime(double raw, double configured_offset,
                             bool software_sync, double software_offset) {
  return software_sync && std::abs(software_offset)>0.1
           ? raw+software_offset : raw-configured_offset;
}

class Predictor {
 public:
  explicit Predictor(const Options &options = Options{}) : options_(options) {
    if (!(options.history_seconds>0 && options.max_imu_gap>0 &&
          options.max_prediction_horizon>0 && options.max_samples>=2 &&
          std::isfinite(options.history_seconds) && std::isfinite(options.max_imu_gap) &&
          std::isfinite(options.max_prediction_horizon))) {
      throw std::invalid_argument("invalid predictor bounds");
    }
  }

  Status addImu(const Sample &sample) {
    if (!std::isfinite(sample.t) || !sample.accel.finite() || !sample.gyro.finite())
      return Status::Invalid;
    if (!samples_.empty() && sample.t<samples_.back().t-options_.history_seconds)
      return Status::TooOld;
    const auto it = std::lower_bound(samples_.begin(),samples_.end(),sample.t,
      [](const Sample &s,double t) { return s.t<t; });
    if (it!=samples_.end() && it->t==sample.t) return Status::Duplicate;
    const bool append_ready=ready_ && it==samples_.end() && sample.t>predicted_.t;
    samples_.insert(it,sample);
    trim();
    // Normal in-order IMU arrivals cost one step. Only a delayed measurement or
    // a new LiDAR posterior requires replay of the buffered interval.
    if (append_ready) return advance(sample);
    return has_anchor_ ? rebuild() : Status::Queued;
  }

  Status correct(const State &anchor) {
    if (!valid(anchor)) { has_anchor_=false; ready_=false; return Status::Invalid; }
    // Reprocessing a repeated/older LiDAR correction must not roll the anchor back.
    if (has_anchor_ && anchor.t<=anchor_.t) return Status::OldCorrection;
    anchor_=anchor;
    anchor_.orientation=anchor.orientation.normalized();
    has_anchor_=true;
    return rebuild();
  }

  // Only the newest integrated state is available, never a replay burst. A correction
  // may rebuild the same timestamp internally; it becomes visible at the NEXT new IMU.
  bool takePublication(Prediction &output) {
    if (!ready_ || predicted_.t<=last_published_) return false;
    output=predicted_;
    last_published_=predicted_.t;
    return true;
  }
  bool latest(Prediction &output) const {
    if (!ready_) return false;
    output=predicted_;
    return true;
  }
  std::size_t bufferedSamples() const { return samples_.size(); }
  double lastGapFrom() const { return gap_from_; }
  double lastGapTo() const { return gap_to_; }
  std::size_t gapCount() const { return gap_count_; }
  void reset() {
    samples_.clear(); has_anchor_=false; ready_=false;
    // Keep the publication watermark. For a bag-clock restart, restart the node.
  }

 private:
  Status recordGap(double from, double to) {
    if (from != gap_from_ || to != gap_to_) ++gap_count_;
    gap_from_ = from; gap_to_ = to;
    return Status::ImuGap;
  }
  static bool exceedsInterval(double newer, double older, double bound) {
    // Subtracting epoch-sized doubles loses sub-microsecond precision. Do not
    // reject an exactly-on-limit 50 ms sample merely because of representation.
    const double rounding = std::max(1e-12, 8*std::numeric_limits<double>::epsilon()*
                                     std::max(std::abs(newer),std::abs(older)));
    return newer-older > bound+rounding;
  }
  static bool valid(const State &s) {
    return std::isfinite(s.t) && s.position.finite() && s.velocity.finite() &&
      s.orientation.finite() && std::isfinite(s.orientation.squaredNorm()) &&
      s.orientation.squaredNorm()>1e-16 && s.accel_bias.finite() &&
      s.gyro_bias.finite() && s.gravity.finite() &&
      std::isfinite(s.accel_scale) && s.accel_scale>0 &&
      (!s.covariance_valid || (validCovariance(s.covariance) && validCovariance(s.process_noise)));
  }
  void trim() {
    if (samples_.empty()) return;
    const double cutoff=samples_.back().t-options_.history_seconds;
    // Retain one sample before the cutoff for interpolation.
    while (samples_.size()>2 && samples_[1].t<cutoff) samples_.erase(samples_.begin());
    if (samples_.size()>options_.max_samples)
      samples_.erase(samples_.begin(),samples_.end()-options_.max_samples);
  }
  Status rebuild() {
    ready_=false;
    if (samples_.empty()) return Status::MissingBracket;
    auto hi=std::lower_bound(samples_.begin(),samples_.end(),anchor_.t,
      [](const Sample &s,double t) { return s.t<t; });
    if (hi==samples_.end()) return Status::MissingBracket;
    Sample previous;
    if (hi->t==anchor_.t) previous=*hi;
    else {
      if (hi==samples_.begin()) return Status::MissingBracket;
      const auto &lo=*(hi-1);
      const double dt=hi->t-lo.t;
      if (exceedsInterval(hi->t,lo.t,options_.max_imu_gap)) return recordGap(lo.t, hi->t);
      const double alpha=(anchor_.t-lo.t)/dt;
      previous={anchor_.t,lo.accel+(hi->accel-lo.accel)*alpha,
                         lo.gyro+(hi->gyro-lo.gyro)*alpha};
    }
    if (exceedsInterval(samples_.back().t,anchor_.t,options_.max_prediction_horizon))
      return Status::CorrectionExpired;
    static_cast<State &>(predicted_)=anchor_;
    predicted_.angular_velocity_body=previous.gyro-anchor_.gyro_bias;
    previous_sample_=previous;
    for (const auto &sample : samples_) {
      if (sample.t<=anchor_.t) continue;
      const auto status=advance(sample);
      if (status!=Status::Ready) return status;
    }
    ready_=valid(predicted_) && predicted_.angular_velocity_body.finite();
    return ready_ ? Status::Ready : Status::Invalid;
  }
  Status advance(const Sample &sample) {
    ready_=false;
    if (exceedsInterval(sample.t,anchor_.t,options_.max_prediction_horizon)) return Status::CorrectionExpired;
    const double dt=sample.t-previous_sample_.t;
    if (dt<=0 || exceedsInterval(sample.t,previous_sample_.t,options_.max_imu_gap))
      return recordGap(previous_sample_.t, sample.t);
    const Vec3 omega=(previous_sample_.gyro+sample.gyro)*.5-anchor_.gyro_bias;
    const Vec3 phi=omega*dt;
    const auto delta_q=Quaternion::exp(phi);
    const auto next_q=(predicted_.orientation*delta_q).normalized();
    const Vec3 body_a0=previous_sample_.accel*anchor_.accel_scale-anchor_.accel_bias;
    const Vec3 body_a1=sample.accel*anchor_.accel_scale-anchor_.accel_bias;
    const Vec3 a0=predicted_.orientation.rotate(body_a0)+anchor_.gravity;
    const Vec3 a1=next_q.rotate(body_a1)+anchor_.gravity;
    if (predicted_.covariance_valid) {
      const auto step=covarianceStep(predicted_.orientation.rotationMatrix(),next_q.rotationMatrix(),
        delta_q.rotationMatrix(),rightJacobian(phi.x,phi.y,phi.z),
        skew(body_a0.x,body_a0.y,body_a0.z),skew(body_a1.x,body_a1.y,body_a1.z),dt);
      predicted_.covariance=propagateCovariance(predicted_.covariance,anchor_.process_noise,step);
    }
    // Integrate linearly varying world acceleration over the segment.
    predicted_.position=predicted_.position+predicted_.velocity*dt+(a0*2+a1)*(dt*dt/6.);
    predicted_.velocity=predicted_.velocity+(a0+a1)*(.5*dt);
    predicted_.orientation=next_q;
    predicted_.t=sample.t;
    predicted_.angular_velocity_body=sample.gyro-anchor_.gyro_bias;
    previous_sample_=sample;
    ready_=valid(predicted_) && predicted_.angular_velocity_body.finite();
    return ready_ ? Status::Ready : Status::Invalid;
  }
  Options options_;
  std::vector<Sample> samples_;
  State anchor_;
  Prediction predicted_;
  Sample previous_sample_;
  bool has_anchor_=false, ready_=false;
  double gap_from_=std::numeric_limits<double>::quiet_NaN();
  double gap_to_=std::numeric_limits<double>::quiet_NaN();
  std::size_t gap_count_=0;
  double last_published_=-std::numeric_limits<double>::infinity();
};

}  // namespace lio_prediction
