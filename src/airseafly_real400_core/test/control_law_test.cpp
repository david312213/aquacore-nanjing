// Compile the actual upstream controller.cpp with minimal ROS transport types.
// This tests the real control math, NOT ROS connectivity or PX4 hardware.
#include <airseafly_real400/core.hpp>
#include <queue>
#include <iostream>
#define __CONTROLLER_H
#include <ros/ros.h>
#include "payload_pd.h"
namespace px4ctrl { struct PayloadPD {
  struct Request {
    enum { STATUS=0, RESET=1, RELEASE=2 };
    unsigned operation=STATUS, loaded_mask=0, slot=0;
    std::string inventory_id;
  };
  struct Response {
    bool success=false, enabled=false, ready=false;
    unsigned loaded_mask=0, remaining=0;
    std::string message, inventory_id;
    std::array<double,3> kp{},kv{};
  };
}; }
struct StampedMessage { struct Header { ros::Time stamp; } header; };
struct Parameter_t {
  struct {double Kp0=2,Kp1=2,Kp2=2,Kv0=1.5,Kv1=1.5,Kv2=1.5;} gain;
  struct {double hover_percentage=.35;} thr_map;
  double gra=9.81,max_angle=.2617993878;
  bool payload_pd_enabled=false;
  double payload_pd_transition_s=1.;
  std::array<payload_pd::Gains,4> payload_pd_profiles{};
};
namespace quadrotor_msgs {struct Px4ctrlDebug {
  double des_v_x,des_v_y,des_v_z,des_a_x,des_a_y,des_a_z,des_q_x,des_q_y,des_q_z,des_q_w,des_thr;
};}
using namespace airseafly_real400;
struct Odom_Data_t {StampedMessage msg;ros::Time rcv_stamp;V3 p=V3::Zero(),v=V3::Zero();Q q=Q::Identity();};
struct Imu_Data_t {StampedMessage msg;ros::Time rcv_stamp;V3 w=V3::Zero(),a=V3::Zero();Q q=Q::Identity();};
struct Desired_State_t {V3 p=V3::Zero(),v=V3::Zero(),a=V3::Zero();double yaw=0;};
struct Controller_Output_t {Q q;double thrust;};
class LinearControl {
 public:
  LinearControl(Parameter_t&);
  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t&,const Odom_Data_t&,const Imu_Data_t&,Controller_Output_t&);
  bool estimateThrustModel(const V3&,const Parameter_t&);void resetThrustMapping(double measured_hover=0.);
  bool payloadPDRequest(px4ctrl::PayloadPD::Request&,px4ctrl::PayloadPD::Response&,bool);
 private:
  Parameter_t param_;quadrotor_msgs::Px4ctrlDebug debug_msg_;
  payload_pd::Schedule payload_pd_;std::string payload_pd_instance_id_;
  ros::Publisher pd_trace_pub_;ros::WallTime pd_trace_last_wall_;
  std::queue<std::pair<ros::Time,double>> timed_thrust_;
  double thr2acc_,P_,rho2_=.998;
  double computeDesiredCollectiveThrustSignal(const V3&);double fromQuaternion2yaw(Q);
};
#include "controller.cpp"
int main() {
  try {
    Parameter_t param;LinearControl control(param);Odom_Data_t odom;Imu_Data_t imu;
    Desired_State_t desired;Controller_Output_t out;
    control.calculateControl(desired,odom,imu,out);
    require(std::abs(out.thrust-.35)<1e-12,"hover seed mapping changed");
    for(int axis=0;axis<3;++axis) for(int sign:{-1,1}) {
      desired.p=V3::Zero();desired.p[axis]=sign*.1;
      auto debug=control.calculateControl(desired,odom,imu,out);
      V3 a(debug.des_a_x,debug.des_a_y,debug.des_a_z-9.81);
      require(sign*a[axis]>0,"position correction direction wrong");
      desired.p.setZero();odom.v=V3::Zero();odom.v[axis]=sign*.1;
      debug=control.calculateControl(desired,odom,imu,out);
      a=V3(debug.des_a_x,debug.des_a_y,debug.des_a_z-9.81);
      require(sign*a[axis]<0,"velocity damping direction wrong");odom.v.setZero();
    }
    odom.q=expQ(V3(.03,-.02,1.5707963267948966));desired.p=V3(.1,.2,.1);desired.yaw=.4;
    Q original_error;
    for(double reset:{0.,.9,-1.2}) {
      imu.q=expQ(V3(0,0,reset))*odom.q;
      control.calculateControl(desired,odom,imu,out);
      Q error=imu.q.conjugate()*out.q;
      if(reset==0) original_error=error;
      else require(angle(original_error,error)<1e-7,"PX4 yaw reference reset changed body correction");
    }
    std::cout<<"PASS actual upstream C++ control law: hover, six-axis corrections, velocity damping, 90-degree yaw and PX4 reference reset\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
