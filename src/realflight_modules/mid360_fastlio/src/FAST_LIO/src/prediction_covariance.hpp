#pragma once

// Short-horizon error covariance for the opt-in IMU predictor. No ROS/Eigen
// dependency so the same numerical code can be checked on a workstation.
// Error order: p_world, theta_right_IMU, v_world, bg_IMU, ba_IMU, gravity_world.
// The 23-D IKFoM posterior is mapped into these 18 coordinates in the adapter.
// Extrinsic uncertainty already present in its marginal is retained; extrinsics
// are constant during this propagation. This is a first-order uncertainty model,
// not an independently tuned estimator or a calibrated accuracy guarantee.
#include <array>
#include <cmath>
#include <cstddef>

namespace lio_prediction {
template<std::size_t Rows, std::size_t Cols> struct Matrix {
  std::array<double, Rows*Cols> values{};
  double &operator()(std::size_t r, std::size_t c) { return values[r*Cols+c]; }
  double operator()(std::size_t r, std::size_t c) const { return values[r*Cols+c]; }
};
template<std::size_t N> Matrix<N,N> identity() {
  Matrix<N,N> result;
  for (std::size_t i=0; i<N; ++i) result(i,i)=1;
  return result;
}
template<std::size_t R, std::size_t C> Matrix<C,R> transpose(const Matrix<R,C> &a) {
  Matrix<C,R> result;
  for (std::size_t i=0; i<R; ++i) for (std::size_t j=0; j<C; ++j) result(j,i)=a(i,j);
  return result;
}
template<std::size_t R, std::size_t K, std::size_t C>
Matrix<R,C> multiply(const Matrix<R,K> &a, const Matrix<K,C> &b) {
  Matrix<R,C> result;
  for (std::size_t i=0; i<R; ++i) for (std::size_t k=0; k<K; ++k) {
    if (a(i,k)==0) continue;
    for (std::size_t j=0; j<C; ++j) result(i,j)+=a(i,k)*b(k,j);
  }
  return result;
}
template<std::size_t R, std::size_t K>
Matrix<R,R> sandwich(const Matrix<R,K> &j, const Matrix<K,K> &p) {
  return multiply(multiply(j,p),transpose(j));
}
using Covariance18=Matrix<18,18>;
using ProcessNoise12=Matrix<12,12>;
using Rotation3=Matrix<3,3>;
using PoseCovariance=Matrix<6,6>;

inline Rotation3 skew(double x, double y, double z) {
  Rotation3 r;
  r(0,1)=-z; r(0,2)=y; r(1,0)=z; r(1,2)=-x; r(2,0)=-y; r(2,1)=x;
  return r;
}
inline Rotation3 rightJacobian(double x, double y, double z) {
  const double angle2=x*x+y*y+z*z;
  const double angle=std::sqrt(angle2);
  const double a=angle<1e-5 ? .5-angle2/24. : (1-std::cos(angle))/angle2;
  const double b=angle<1e-5 ? 1./6.-angle2/120. : (angle-std::sin(angle))/(angle2*angle);
  const auto k=skew(x,y,z), kk=multiply(k,k);
  auto result=identity<3>();
  for (std::size_t i=0; i<9; ++i) result.values[i]+=-a*k.values[i]+b*kk.values[i];
  return result;
}
template<std::size_t N> bool validCovariance(const Matrix<N,N> &p) {
  // Input comes from the EKF. Validate finite/symmetric/nonnegative variances;
  // downstream bridge additionally checks positive semidefiniteness before use.
  for (std::size_t i=0; i<N; ++i) for (std::size_t j=0; j<N; ++j) {
    if (!std::isfinite(p(i,j))) return false;
    if (std::abs(p(i,j)-p(j,i))>1e-8*(1+std::abs(p(i,j))+std::abs(p(j,i)))) return false;
  }
  for (std::size_t i=0; i<N; ++i) if (p(i,i)<-1e-12) return false;
  return true;
}

struct CovarianceStep {
  Covariance18 transition;
  Matrix<18,12> noise_jacobian;
};
inline CovarianceStep covarianceStep(const Rotation3 &r0, const Rotation3 &r1,
                                     const Rotation3 &delta_r, const Rotation3 &jr,
                                     const Rotation3 &acc0_skew, const Rotation3 &acc1_skew,
                                     double dt) {
  CovarianceStep out;
  out.transition=identity<18>();
  const auto theta=multiply(r1,acc1_skew);
  const auto a0=multiply(r0,acc0_skew);
  const auto a1=multiply(theta,transpose(delta_r));
  const auto bg=multiply(theta,jr);
  for (std::size_t i=0; i<3; ++i) for (std::size_t j=0; j<3; ++j) {
    const double ident=i==j ? 1. : 0.;
    out.transition(i,6+j)=ident*dt;
    out.transition(i,3+j)=-(2*a0(i,j)+a1(i,j))*dt*dt/6.;
    out.transition(i,9+j)=bg(i,j)*dt*dt*dt/6.;
    out.transition(i,12+j)=-(2*r0(i,j)+r1(i,j))*dt*dt/6.;
    out.transition(i,15+j)=ident*dt*dt*.5;
    out.transition(3+i,3+j)=delta_r(j,i);
    out.transition(3+i,9+j)=-jr(i,j)*dt;
    out.transition(6+i,3+j)=-(a0(i,j)+a1(i,j))*dt*.5;
    out.transition(6+i,9+j)=bg(i,j)*dt*dt*.5;
    out.transition(6+i,12+j)=-(r0(i,j)+r1(i,j))*dt*.5;
    out.transition(6+i,15+j)=ident*dt;
    // Same discrete Q convention as IKFoM predict: dt*G Q (dt*G)^T.
    // Noise order is gyro, acceleration, gyro bias walk, acceleration bias walk.
    out.noise_jacobian(i,j)=out.transition(i,9+j);
    out.noise_jacobian(3+i,j)=out.transition(3+i,9+j);
    out.noise_jacobian(6+i,j)=out.transition(6+i,9+j);
    out.noise_jacobian(i,3+j)=out.transition(i,12+j);
    out.noise_jacobian(6+i,3+j)=out.transition(6+i,12+j);
    out.noise_jacobian(9+i,6+j)=ident*dt;
    out.noise_jacobian(12+i,9+j)=ident*dt;
  }
  return out;
}
inline Covariance18 propagateCovariance(const Covariance18 &prior, const ProcessNoise12 &noise,
                                       const CovarianceStep &step) {
  auto next=sandwich(step.transition,prior);
  const auto added=sandwich(step.noise_jacobian,noise);
  for (std::size_t i=0; i<18; ++i) for (std::size_t j=i; j<18; ++j) {
    const double value=.5*(next(i,j)+next(j,i)+added(i,j)+added(j,i));
    next(i,j)=next(j,i)=value;
  }
  return next;
}
inline PoseCovariance poseCovarianceWorld(const Covariance18 &p, const Rotation3 &rotation) {
  Matrix<6,18> j;
  for (std::size_t i=0; i<3; ++i) {
    j(i,i)=1;
    for (std::size_t k=0; k<3; ++k) j(3+i,3+k)=rotation(i,k);
  }
  return sandwich(j,p);
}
} // namespace lio_prediction
