#include "imu_predictor.hpp"
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>

using namespace lio_prediction;
int checks=0;
void check(bool ok,const std::string &message) {
  ++checks;
  if (!ok) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
void near(double a,double b,double tolerance,const std::string &message) {
  check(std::abs(a-b)<=tolerance,message+" actual="+std::to_string(a)+" expected="+std::to_string(b));
}
void perturb(Vec3 &v,int axis,double amount) {
  if (axis==0) v.x+=amount;
  if (axis==1) v.y+=amount;
  if (axis==2) v.z+=amount;
}
Prediction oneStep(const State &s,const Sample &start,const Sample &end) {
  Predictor predictor;
  predictor.addImu(start); predictor.addImu(end);
  check(predictor.correct(s)==Status::Ready,"one step prediction ready");
  Prediction p; check(predictor.takePublication(p),"one step publication"); return p;
}
std::array<double,18> difference(const Prediction &a,const Prediction &b) {
  std::array<double,18> d{};
  const Vec3 vectors[5]={a.position-b.position,a.velocity-b.velocity,a.gyro_bias-b.gyro_bias,
                         a.accel_bias-b.accel_bias,a.gravity-b.gravity};
  const int offsets[5]={0,6,9,12,15};
  for (int i=0;i<5;++i) { d[offsets[i]]=vectors[i].x; d[offsets[i]+1]=vectors[i].y; d[offsets[i]+2]=vectors[i].z; }
  const auto q=Quaternion{b.orientation.w,-b.orientation.x,-b.orientation.y,-b.orientation.z}*a.orientation;
  d[3]=2*q.x; d[4]=2*q.y; d[5]=2*q.z;
  return d;
}
void checkAnalyticJacobian() {
  State s;
  s.t=7.; s.orientation=Quaternion::exp({.4,-.3,.7}); s.velocity={.7,-.2,.4};
  s.accel_bias={.03,-.07,.1}; s.gyro_bias={.05,-.03,.1}; s.accel_scale=1.03;
  Sample start{7.,{.2,-.7,9.7},{.5,.4,-.2}}, end{7.03,{1.1,-.5,9.9},{.6,.7,-.1}};
  const auto nominal=oneStep(s,start,end);
  const double dt=end.t-start.t;
  const auto omega=((start.gyro+end.gyro)*.5-s.gyro_bias)*dt;
  const auto increment=Quaternion::exp(omega);
  const auto a0=start.accel*s.accel_scale-s.accel_bias;
  const auto a1=end.accel*s.accel_scale-s.accel_bias;
  const auto linearization=covarianceStep(s.orientation.rotationMatrix(),nominal.orientation.rotationMatrix(),
    increment.rotationMatrix(),rightJacobian(omega.x,omega.y,omega.z),
    skew(a0.x,a0.y,a0.z),skew(a1.x,a1.y,a1.z),dt);
  const double epsilon=1e-6;
  for (int column=0;column<18;++column) {
    State shifted=s;
    if (column<3) perturb(shifted.position,column,epsilon);
    else if (column<6) { Vec3 rotation; perturb(rotation,column-3,epsilon); shifted.orientation=s.orientation*Quaternion::exp(rotation); }
    else if (column<9) perturb(shifted.velocity,column-6,epsilon);
    else if (column<12) perturb(shifted.gyro_bias,column-9,epsilon);
    else if (column<15) perturb(shifted.accel_bias,column-12,epsilon);
    else perturb(shifted.gravity,column-15,epsilon);
    const auto delta=difference(oneStep(shifted,start,end),nominal);
    for (int row=0;row<18;++row)
      near(delta[row]/epsilon,linearization.transition(row,column),2e-7,"analytic derivative "+std::to_string(row)+","+std::to_string(column));
  }
}
State uncertainState() {
  State s; s.covariance_valid=true;
  Matrix<18,18> factor;
  for (int i=0;i<18;++i) { factor(i,i)=.03+.002*i; if (i>0) factor(i,i-1)=.01; }
  s.covariance=multiply(factor,transpose(factor));
  for (int i=0;i<12;++i) s.process_noise(i,i)=i<6 ? .1 : .0001;
  return s;
}
void delayedCovarianceReplay() {
  auto s=uncertainState(); s.t=.1025;
  Predictor buffered,incremental;
  for (int i=0;i<=40;++i) buffered.addImu({i*.005,{.3*i*.005,.2,9.81},{.1,-.2,.4}});
  for (int i=0;i<=21;++i) incremental.addImu({i*.005,{.3*i*.005,.2,9.81},{.1,-.2,.4}});
  check(buffered.correct(s)==Status::Ready,"late covariance anchor ready");
  check(incremental.correct(s)==Status::Ready,"incremental covariance anchor ready");
  for (int i=22;i<=40;++i) incremental.addImu({i*.005,{.3*i*.005,.2,9.81},{.1,-.2,.4}});
  Prediction a,b; check(buffered.latest(a)&&incremental.latest(b),"both covariance results ready");
  near(a.t,.2,1e-12,"covariance accompanies newest state epoch");
  for (int i=0;i<18;++i) for (int j=0;j<18;++j)
    near(a.covariance(i,j),b.covariance(i,j),1e-13,"batch replay equals online covariance");
  check(a.covariance(0,0)>s.covariance(0,0),"position uncertainty grows with velocity/bias/gravity uncertainty");
  std::mt19937 generator(42); std::normal_distribution<double> normal;
  for (int trial=0;trial<100;++trial) {
    std::array<double,18> x; for (auto &value:x) value=normal(generator);
    double quadratic=0;
    for (int i=0;i<18;++i) for (int j=0;j<18;++j) quadratic+=x[i]*a.covariance(i,j)*x[j];
    check(quadratic>=-1e-12,"positive semidefinite covariance along sampled directions");
  }
  auto replacement=uncertainState(); replacement.t=.1525;
  replacement.covariance(0,0)=.09;
  check(buffered.correct(replacement)==Status::Ready,"new posterior covariance replaces old anchor");
  buffered.latest(a); check(a.covariance(0,0)>.09,"new posterior covariance propagated to latest time");
}
void covarianceConventions() {
  Covariance18 p;
  p(0,0)=4; p(3,3)=1; p(4,4)=2; p(5,5)=3; p(0,3)=p(3,0)=.25;
  const auto rotation=Quaternion::exp({0,0,1.5707963267948966}).rotationMatrix();
  const auto world=poseCovarianceWorld(p,rotation);
  near(world(3,3),2,1e-12,"world attitude x rotates from local y");
  near(world(4,4),1,1e-12,"world attitude y rotates from local x");
  near(world(0,4),.25,1e-12,"position attitude cross covariance rotated");
  near(world(4,0),.25,1e-12,"cross covariance remains symmetric");
  auto s=uncertainState(); s.covariance(0,0)=-1;
  Predictor invalid; invalid.addImu({0,{0,0,9.81},{}});
  check(invalid.correct(s)==Status::Invalid,"negative source variance rejected");
}
int main() {
  checkAnalyticJacobian(); delayedCovarianceReplay(); covarianceConventions();
  std::cout << "PASS: " << checks << " covariance/Jacobian checks; synthetic input only.\n";
}
