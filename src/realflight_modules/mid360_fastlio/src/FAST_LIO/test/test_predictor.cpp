#include "imu_predictor.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

using namespace lio_prediction;
int checks=0;
void check(bool yes,const std::string &message) {
  ++checks;
  if (!yes) { std::cerr<<"FAIL: "<<message<<"\n"; std::exit(1); }
}
void near(double a,double b,double tolerance,const std::string &message) {
  check(std::abs(a-b)<=tolerance,message+" actual="+std::to_string(a)+" expected="+std::to_string(b));
}
void nearVec(Vec3 a,Vec3 b,double tolerance,const std::string &message) {
  near(a.x,b.x,tolerance,message+" x"); near(a.y,b.y,tolerance,message+" y");
  near(a.z,b.z,tolerance,message+" z");
}
Options longTestOptions() { Options o; o.max_prediction_horizon=3; return o; }
Sample sample(double t, Vec3 accel={0,0,9.81}, Vec3 gyro={}) { return {t,accel,gyro}; }

void stationaryAndUnits() {
  Predictor p(longTestOptions());
  State a; a.orientation=Quaternion::exp({0.5,-0.3,0.2});
  a.position={3,-2,1}; a.accel_bias={0.13,-0.06,0.2}; a.gyro_bias={0.01,0.02,-0.03};
  // Input scale deliberately not 1: catches dropping FAST-LIO's normalization.
  a.accel_scale=9.81/10.07;
  const Vec3 raw=(a.orientation.inverseRotate(a.gravity*-1)+a.accel_bias)*(1/a.accel_scale);
  for(int i=0;i<=200;++i) p.addImu(sample(i*0.005,raw,a.gyro_bias));
  check(p.correct(a)==Status::Ready,"stationary correction accepted");
  Prediction result; check(p.takePublication(result),"stationary result");
  nearVec(result.position,a.position,1e-11,"stationary position");
  nearVec(result.velocity,{},1e-11,"stationary velocity");
  nearVec(result.angular_velocity_body,{},1e-12,"gyro bias removed");
}

void lateAnchorReplayAndInterpolation() {
  Predictor p(longTestOptions());
  // Analytic world acceleration a_x(t)=2*t, v_x=t^2, x=t^3/3.
  for(int i=0;i<=40;++i) p.addImu(sample(i*0.005,{2*i*0.005,0,9.81}));
  State a; a.t=0.1025; a.position={std::pow(a.t,3)/3,0,0}; a.velocity={a.t*a.t,0,0};
  check(p.correct(a)==Status::Ready,"interpolated late anchor accepted");
  Prediction result; check(p.takePublication(result),"late anchor replay output");
  near(result.t,0.2,1e-14,"replayed all post-anchor samples");
  near(result.position.x,std::pow(0.2,3)/3,1e-12,"interpolation position analytic");
  near(result.velocity.x,0.04,1e-12,"interpolation velocity analytic");
  // A correction arrives at 0.1525 after 0.2 has already been published.
  a.t=0.1525; a.position={std::pow(a.t,3)/3+0.4,0,0}; a.velocity={a.t*a.t,0,0};
  check(p.correct(a)==Status::Ready,"second delayed correction accepted");
  check(!p.takePublication(result),"correction cannot republish same timestamp");
  p.addImu(sample(0.205,{0.41,0,9.81}));
  check(p.takePublication(result),"next IMU publishes corrected state");
  near(result.position.x,std::pow(0.205,3)/3+0.4,1e-12,"late correction persisted");
  near(result.velocity.x,0.205*0.205,1e-12,"late correction replay velocity");
}

void rotationAndFrame() {
  Predictor p(longTestOptions());
  State a; a.velocity={1,0,0}; a.gyro_bias={0,0,0.03};
  for(int i=0;i<=200;++i) p.addImu(sample(i*0.005,{0,0,9.81},{0,0,1.03}));
  check(p.correct(a)==Status::Ready,"rotation anchor");
  Prediction r; check(p.takePublication(r),"rotation result");
  near(r.orientation.w,std::cos(0.5),1e-12,"rotation quaternion w");
  near(r.orientation.z,std::sin(0.5),1e-12,"rotation quaternion z");
  nearVec(r.position,{1,0,0},1e-11,"world constant velocity");
  nearVec(r.velocityBody(),{std::cos(1.),-std::sin(1.),0},1e-11,"ROS child-frame velocity");
}

void sortingAndDuplicates() {
  Predictor sorted(longTestOptions()), shuffled(longTestOptions());
  for(int i=0;i<=20;++i) sorted.addImu(sample(i*0.01,{1,0,9.81}));
  for(int i=20;i>=0;--i) shuffled.addImu(sample(i*0.01,{1,0,9.81}));
  State a; a.t=0.025; a.position={0.5*a.t*a.t,0,0}; a.velocity={a.t,0,0};
  sorted.correct(a); shuffled.correct(a);
  Prediction s,r; sorted.takePublication(s); shuffled.takePublication(r);
  nearVec(s.position,r.position,1e-12,"sorted and reverse arrivals agree");
  check(shuffled.addImu(sample(0.2,{100,0,9.81}))==Status::Duplicate,"duplicate first value wins");
  check(!shuffled.takePublication(r),"duplicate no output");
  check(shuffled.correct(a)==Status::OldCorrection,"repeated anchor rejected");
  a.t=0.01; check(shuffled.correct(a)==Status::OldCorrection,"backward anchor rejected");
  // Insert a delayed IMU between the anchor and the newest IMU. Rebuild internally.
  shuffled.addImu(sample(0.115,{1,0,9.81}));
  check(!shuffled.takePublication(r),"late IMU cannot regress publication");
  shuffled.addImu(sample(0.21,{1,0,9.81}));
  check(shuffled.takePublication(r),"late IMU recovery on next sample");
  near(r.position.x,0.5*0.21*0.21,1e-12,"late IMU replay matches analytic");
}

void boundsAndClock() {
  Predictor p;
  State a; a.t=1;
  check(p.correct(a)==Status::MissingBracket,"no endpoint rejected");
  p.addImu(sample(1.01));
  Prediction r; check(!p.takePublication(r),"cannot invent missing lower endpoint");
  p.addImu(sample(0.99)); check(p.takePublication(r),"late lower bracket repairs anchor");
  p.addImu(sample(1.2)); check(!p.takePublication(r),"IMU gap suppresses prediction");
  a.t=1.2; check(p.correct(a)==Status::Ready,"new exact anchor repairs old gap");
  check(p.takePublication(r),"repair publication moves forward");
  check(p.addImu(sample(std::numeric_limits<double>::quiet_NaN()))==Status::Invalid,"NaN timestamp");
  check(p.addImu(sample(1.21,{0,0,std::numeric_limits<double>::infinity()}))==Status::Invalid,"Inf acceleration");
  a.t=1.3; a.orientation={0,0,0,0}; check(p.correct(a)==Status::Invalid,"zero quaternion");
  Predictor expired; for(int i=0;i<=220;++i) expired.addImu(sample(i*0.005));
  check(expired.correct(State{})==Status::CorrectionExpired,"LiDAR loss horizon bounded");
  check(!expired.takePublication(r),"no stale-anchor odometry");
  Options o; o.max_samples=20; Predictor limited(o);
  for(int i=0;i<1000;++i) limited.addImu(sample(i*0.005));
  check(limited.bufferedSamples()==20,"bounded sample buffer");
  check(limited.addImu(sample(0))==Status::TooOld,"ancient sample rejected");
  near(adjustedImuTime(10,0.027,false,0),9.973,1e-12,"nonzero configured offset");
  near(adjustedImuTime(10,0.027,true,0.25),10.25,1e-12,"software override matches baseline");
  near(adjustedImuTime(10,0.027,true,0.05),9.973,1e-12,"small software offset uses configured offset");
  for(double epoch : {10.0,1800000000.0}) {
    Predictor boundary;
    boundary.addImu(sample(epoch)); boundary.addImu(sample(epoch+0.05));
    State exact; exact.t=epoch;
    check(boundary.correct(exact)==Status::Ready,"50 ms boundary tolerates epoch rounding");
    check(boundary.takePublication(r),"boundary output available");
    boundary.reset();
    boundary.addImu(sample(epoch-1)); boundary.addImu(sample(epoch-0.99));
    exact.t=epoch-1; boundary.correct(exact);
    check(!boundary.takePublication(r),"reset retains monotonic publication watermark");
  }
}

void delayedAnchorLegacyRegression() {
  // Exact horizontal acceleration a(t)=20*t. The correction is 52.5 ms old,
  // but the legacy endpoint still belongs to the last IMU at 200 ms.
  const double anchor_time=.1525, previous_imu=.2, now=.205;
  State a; a.t=anchor_time; a.position={10.*std::pow(a.t,3)/3.,0,0};
  a.velocity={10.*a.t*a.t,0,0};
  Predictor fixed;
  for (int i=0;i<=40;++i) fixed.addImu(sample(i*.005,{20*i*.005,0,9.81}));
  fixed.correct(a); fixed.addImu(sample(now,{20*now,0,9.81}));
  Prediction result; check(fixed.takePublication(result),"regression predictor output");
  const double dt=now-anchor_time;
  const double legacy_acc=.5*(20*previous_imu+20*now);
  const double legacy_v=a.velocity.x+dt*legacy_acc;
  const double legacy_p=a.position.x+dt*a.velocity.x+.5*dt*dt*legacy_acc;
  const double exact_v=10*now*now, exact_p=10*std::pow(now,3)/3.;
  near(result.velocity.x,exact_v,1e-12,"replayed velocity matches analytic ramp");
  near(result.position.x,exact_p,1e-12,"replayed position matches analytic ramp");
  check(std::abs(legacy_v-exact_v)>.02,"legacy future endpoint reproduces velocity mismatch");
  std::cout << "Synthetic 52.5 ms delayed posterior: legacy velocity error "
            << legacy_v-exact_v << " m/s, position error " << legacy_p-exact_p
            << " m; replayed errors < 1e-12 (not a flight accuracy result).\n";
}

int main() {
  stationaryAndUnits(); lateAnchorReplayAndInterpolation(); rotationAndFrame();
  sortingAndDuplicates(); boundsAndClock(); delayedAnchorLegacyRegression();
  std::cout<<"PASS: "<<checks<<" numerical and rejection checks; synthetic input only (no flight bag).\n";
}
