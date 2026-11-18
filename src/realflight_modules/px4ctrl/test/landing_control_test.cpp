#include "controller.h"
#include "control_limits.h"
#include "landing_reference.h"
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <string>

namespace {
const double pi = std::acos(-1.0);
int cases = 0, failures = 0;
void check(bool ok, const char *why) {
  ++cases;
  if (!ok) { ++failures; if (failures <= 8) std::cerr << why << '\n'; }
}
Parameter_t parameters() {
  Parameter_t p{};
  p.gra = 9.81; p.thr_map.hover_percentage = .68; p.max_angle = pi/6.;
  p.gain.Kp0=p.gain.Kp1=p.gain.Kp2=2.;
  p.gain.Kv0=p.gain.Kv1=p.gain.Kv2=1.8;
  return p;
}
Eigen::Quaterniond rotation(double yaw, double pitch, double roll) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX()));
}
}

int main(int argc, char **argv) {
  const char *master=std::getenv("ROS_MASTER_URI");
  if (!master || std::string(master).find("http://127.0.0.1:")!=0 ||
      std::string(master).find(":11341")!=std::string::npos ||
      std::string(master).find(":11311")!=std::string::npos) return 2;
  ros::init(argc,argv,"landing_control_offline",ros::init_options::AnonymousName);
  Parameter_t p=parameters();
  LinearControl controller(p);
  double max_tilt=0., max_thrust=0., max_force_error=0.;
  int recorded=0;
  auto exercise = [&](Odom_Data_t &odom, Imu_Data_t &imu, Desired_State_t &des) {
    Controller_Output_t out;
    controller.calculateControl(des,odom,imu,out);
    const Eigen::Vector3d z = out.q * Eigen::Vector3d::UnitZ();
    const double tilt = std::acos(std::max(-1.,std::min(1.,z.z())));
    max_tilt=std::max(max_tilt,tilt); max_thrust=std::max(max_thrust,out.thrust);
    check(out.q.coeffs().allFinite() && std::abs(out.q.norm()-1.)<1e-9,
          "invalid output quaternion");
    check(tilt <= p.max_angle+1e-9,"tilt exceeded configured FCU-vertical limit");
    check(std::isfinite(out.thrust) && out.thrust>=0. && out.thrust<=1.,
          "normalized thrust outside [0,1]");
    // Independent force-domain oracle (the emitted attitude and thrust must
    // describe the same force, including both forms of saturation).
    const Eigen::Quaterniond frame=(imu.q*odom.q.inverse()).normalized();
    Eigen::Vector3d requested = frame*(des.a+2.*(des.p-odom.p)+1.8*(des.v-odom.v));
    requested.z() += p.gra;
    Eigen::Vector3d expected=requested;
    if (requested.z()<=0.) expected.setZero();
    else {
      const double xy=std::hypot(requested.x(),requested.y());
      const double bound=requested.z()*std::tan(p.max_angle);
      if(xy>bound) expected.head<2>() *= bound/xy;
    }
    const double mapping=p.gra/p.thr_map.hover_percentage;
    if (expected.norm()>mapping) expected*=mapping/expected.norm();
    const double error=(z*out.thrust*mapping-expected).norm();
    max_force_error=std::max(max_force_error,error);
    check(error<1e-8,"attitude/thrust do not preserve the bounded requested force");
  };
  std::mt19937 rng(20261006);
  std::uniform_real_distribution<double> angle(-pi,pi), error(-8.,8.);
  for(int n=0;n<2000;++n) {
    Odom_Data_t odom; Imu_Data_t imu;
    odom.p.setZero(); odom.v.setZero();
    odom.q=rotation(angle(rng),angle(rng),angle(rng));
    imu.q=rotation(angle(rng),angle(rng),angle(rng));
    Desired_State_t des(odom); des.p.setZero();des.v.setZero();
    des.a=Eigen::Vector3d(error(rng)*4.,error(rng)*4.,error(rng)*2.);
    des.yaw=angle(rng); exercise(odom,imu,des);
  }
  // Include zero-force and downward requests in an explicitly tilted map.
  for(double vertical : {0.,-5.,9.81}) {
    Odom_Data_t odom; Imu_Data_t imu;
    odom.p.setZero();odom.v.setZero();odom.q=rotation(.7,.8,-.4);imu.q.setIdentity();
    Desired_State_t des(odom);des.p.setZero();des.v.setZero();des.yaw=.5;
    des.a=odom.q*Eigen::Vector3d(0.,0.,vertical-9.81);
    exercise(odom,imu,des);
  }
  if(argc>1) {
    std::ifstream input(argv[1]);
    double t,ox,oy,oz,ow,ix,iy,iz,iw;
    Odom_Data_t odom; Imu_Data_t imu; Desired_State_t des;
    while(input>>t>>odom.p.x()>>odom.p.y()>>odom.p.z()
          >>odom.v.x()>>odom.v.y()>>odom.v.z()
          >>ox>>oy>>oz>>ow>>ix>>iy>>iz>>iw
          >>des.p.x()>>des.p.y()>>des.p.z()
          >>des.v.x()>>des.v.y()>>des.v.z()
          >>des.a.x()>>des.a.y()>>des.a.z()>>des.yaw) {
      odom.q=Eigen::Quaterniond(ow,ox,oy,oz);imu.q=Eigen::Quaterniond(iw,ix,iy,iz);
      exercise(odom,imu,des);++recorded;
    }
    check(input.eof() && recorded>0,"recorded fixture missing/invalid");
  }
  // The pre-existing small-error command must still give the same force.
  {
    Odom_Data_t odom;Imu_Data_t imu;
    odom.p.setZero();odom.v.setZero();odom.q.setIdentity();imu.q.setIdentity();
    Desired_State_t des(odom);des.a=Eigen::Vector3d(.4,-.2,.1);des.yaw=.8;
    exercise(odom,imu,des);
  }
  // Smooth speed entry, derivative consistency and bounded reference at rest.
  const double start=.65, speed=-.3;
  auto ref=[&](double t) {return airseafly_landing::vertical_reference(start,speed,t,0.);};
  check(ref(0).velocity==0. && ref(0).acceleration==0. && ref(0).position==start,
        "landing reference has an entry step");
  for(double t : {.01,.25,.5,.75,.99,1.01,2.}) {
    const double dt=1e-6;
    check(std::abs((ref(t+dt).position-ref(t-dt).position)/(2.*dt)-ref(t).velocity)<1e-7,
          "landing position/velocity discontinuity");
    check(std::abs((ref(t+dt).velocity-ref(t-dt).velocity)/(2.*dt)-ref(t).acceleration)<1e-7,
          "landing velocity/acceleration discontinuity");
  }
  check(std::abs(ref(1.).velocity-speed)<1e-12 && ref(1.).acceleration==0.,"landing ramp end");
  check(ref(600.).position>=-.6 && ref(600.).position<-.5,
        "landing reference unbounded or existing land detector unreachable");
  // Do not learn from a saturated command, or accept an invalid model update.
  for(bool saturated : {false,true}) {
    LinearControl c(p);Odom_Data_t odom;Imu_Data_t imu;
    odom.p.setZero();odom.v.setZero();odom.q.setIdentity();imu.q.setIdentity();
    Desired_State_t des(odom);des.a=Eigen::Vector3d(0.,0.,saturated?100.:0.);des.yaw=0.;
    Controller_Output_t u;ros::Time::setNow(ros::Time(100.));c.calculateControl(des,odom,imu,u);
    ros::Time::setNow(ros::Time(100.04));
    check(!c.estimateThrustModel(Eigen::Vector3d(0.,0.,-100.),p),
          "thrust estimator accepted a saturated/invalid observation");
    des.a.setZero();c.calculateControl(des,odom,imu,u);
    check(std::abs(u.thrust-.68)<1e-9,"invalid estimate changed hover thrust mapping");
  }
  std::cout<<"{\"cases\":"<<cases<<",\"failures\":"<<failures
           <<",\"recorded_async_input_combinations\":"<<recorded
           <<",\"max_output_tilt_deg\":"<<max_tilt*180./pi
           <<",\"max_output_thrust\":"<<max_thrust
           <<",\"max_force_error\":"<<max_force_error
           <<",\"real_flight_validated\":false}\n";
  ros::shutdown();return failures?1:0;
}
