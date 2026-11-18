// Actual controller/FSM integration on an isolated ROS master with synthetic
// feedback. No MAVROS node, physical actuator, or real arming service is used.
#include "PX4CtrlFSM.h"
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <atomic>
#include <chrono>
#include <thread>

namespace {
int cases = 0;
void check(bool ok, const char *message) {
  ++cases;
  if (!ok) throw std::runtime_error(message);
}
bool mock_mode(mavros_msgs::SetMode::Request &, mavros_msgs::SetMode::Response &r) {
  r.mode_sent = true; return true;
}
void feedback(PX4CtrlFSM &fsm, double t) {
  ros::Time::setNow(ros::Time(t));
  fsm.odom_data.p = Eigen::Vector3d(0., 0., 1.);
  fsm.odom_data.v.setZero(); fsm.odom_data.q.setIdentity();
  fsm.odom_data.rcv_stamp = fsm.odom_data.msg.header.stamp = ros::Time(t);
  fsm.imu_data.q.setIdentity(); fsm.imu_data.a = Eigen::Vector3d(0., 0., 9.81);
  fsm.imu_data.w.setZero();
  fsm.imu_data.rcv_stamp = fsm.imu_data.msg.header.stamp = ros::Time(t);
  fsm.state_data.current_state.connected = true;
  fsm.state_data.current_state.armed = true;
  fsm.state_data.current_state.mode = "OFFBOARD";
  fsm.extended_state_data.current_extended_state.landed_state = mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
}
}

int main(int argc, char **argv) {
  const char *master = std::getenv("ROS_MASTER_URI");
  const char *isolated = std::getenv("PX4CTRL_ISOLATED_TEST");
  if (!master || !isolated || std::string(isolated) != "1" ||
      std::string(master).find("http://127.0.0.1:") != 0 ||
      std::string(master).find(":11311") != std::string::npos ||
      std::string(master).find(":11341") != std::string::npos ||
      std::string(master).find(":11345") != std::string::npos) return 2;
  ros::init(argc, argv, "position_integral_offline", ros::init_options::AnonymousName);
  ros::NodeHandle node("/offline_integral"), params("/offline_integral_config");
  auto service = node.advertiseService("mode", mock_mode);
  ros::AsyncSpinner spinner(1); spinner.start();
  try {
    Parameter_t p;
    p.config_from_ros_handle(params); // Real ROS parameter loader and candidate YAML.
    check(p.position_integral.enabled && p.position_integral.ki[0] == .15 &&
          p.position_integral.ki[1] == .15 && p.position_integral.ki[2] == 0., "integral YAML not loaded");
    p.max_angle = std::acos(-1.)/6.;
    Odom_Data_t odom; Imu_Data_t imu;
    odom.p.setZero(); odom.v.setZero(); odom.q.setIdentity();
    imu.q.setIdentity(); imu.a=Eigen::Vector3d(0.,0.,p.gra); imu.w.setZero();
    Desired_State_t des(odom); des.p=Eigen::Vector3d(.2,-.1,0.);
    Controller_Output_t out;
    LinearControl c(p);
    quadrotor_msgs::Px4ctrlDebug debug;
    for (int n=0;n<=100;++n) {
      ros::Time::setNow(ros::Time(100.+n*.01));
      debug=c.calculateControl(des,odom,imu,out,true);
    }
    check(std::abs(debug.des_a_x-p.gain.Kp0*.2-.03)<1e-9, "actual controller did not apply X integral");
    check(std::abs(debug.des_a_y+p.gain.Kp1*.1+.015)<1e-9, "actual controller did not apply Y integral");
    check(std::abs(debug.des_a_z-p.gra)<1e-9, "disabled Z integral changed altitude force");
    // Recover the world force from the actual attitude and collective output.
    const Eigen::Vector3d recovered=out.q*Eigen::Vector3d::UnitZ()*out.thrust*p.gra/p.thr_map.hover_percentage;
    check((recovered-Eigen::Vector3d(debug.des_a_x,debug.des_a_y,debug.des_a_z)).norm()<1e-9,
          "attitude and thrust disagree with the PID acceleration");
    c.resetPositionIntegral();
    des.a.x()=100.;
    for(int n=0;n<=100;++n) {
      ros::Time::setNow(ros::Time(102.+n*.01)); c.calculateControl(des,odom,imu,out,true);
    }
    des.a.setZero();
    ros::Time::setNow(ros::Time(103.01)); debug=c.calculateControl(des,odom,imu,out,true);
    check(std::abs(debug.des_a_x-p.gain.Kp0*.2-.0303)<1e-9, "output saturation still gated integration");
    c.resetPositionIntegral();
    des.p=Eigen::Vector3d(1.,-.8,0.);
    odom.v=Eigen::Vector3d(.5,-.6,0.);
    des.v=Eigen::Vector3d(.7,.4,0.);
    ros::Time::setNow(ros::Time(103.1)); c.calculateControl(des,odom,imu,out);
    ros::Time::setNow(ros::Time(103.6)); debug=c.calculateControl(des,odom,imu,out);
    check(std::abs(debug.des_a_x-p.gain.Kp0-p.gain.Kv0*.2-.075)<1e-9 &&
          std::abs(debug.des_a_y+p.gain.Kp1*.8-p.gain.Kv1+.06)<1e-9,
          "large error, measured/desired speed or long interval still gated integration");
    des.p=Eigen::Vector3d(.2,-.1,0.); des.v.setZero(); odom.v.setZero();
    // Tilted odometry world: gravity and I must share the same transform.
    c.resetPositionIntegral();
    const Eigen::Quaterniond frame(Eigen::AngleAxisd(.08,Eigen::Vector3d::UnitY()));
    imu.q=frame;
    for(int n=0;n<=100;++n) {
      ros::Time::setNow(ros::Time(104.+n*.01)); debug=c.calculateControl(des,odom,imu,out,true);
    }
    const Eigen::Vector3d expect=frame*Eigen::Vector3d(p.gain.Kp0*.2+.03,-p.gain.Kp1*.1-.015,0.)+
                               Eigen::Vector3d(0.,0.,p.gra);
    const Eigen::Vector3d actual=out.q*Eigen::Vector3d::UnitZ()*out.thrust*p.gra/p.thr_map.hover_percentage;
    check((actual-expect).norm()<1e-9, "world-frame integral was rotated twice or mixed with gravity");

    LinearControl controller(p); PX4CtrlFSM fsm(p,controller);
    fsm.ctrl_FCU_pub=node.advertise<mavros_msgs::AttitudeTarget>("attitude",10);
    fsm.debug_pub=node.advertise<quadrotor_msgs::Px4ctrlDebug>("debug",10);
    fsm.traj_start_trigger_pub=node.advertise<geometry_msgs::PoseStamped>("trigger",10);
    fsm.set_FCU_mode_srv=node.serviceClient<mavros_msgs::SetMode>("mode");
    check(fsm.set_FCU_mode_srv.waitForExistence(ros::Duration(3)), "mock mode service missing");
    double t=200.;
    feedback(fsm,t); fsm.rc_data.is_command_mode=false;
    fsm.rc_data.enter_hover_mode=true; fsm.process();
    check(fsm.get_state()==PX4CtrlFSM::AUTO_HOVER, "hover handoff failed");
    fsm.hover_pose.x()=.2;
    const auto learn=[&]() {
      for(int n=0;n<30;++n) { feedback(fsm,t+=.01); fsm.process(); }
      check(fsm.debug_msg.des_a_x>p.gain.Kp0*.2+.005, "FSM did not integrate hover error");
    };
    learn();
    for(int fault=0;fault<7;++fault) {
      const double previous=fsm.debug_msg.des_a_x;
      feedback(fsm,t+=.01);
      if(fault==0) fsm.imu_data.rcv_stamp=ros::Time(t-1.);
      if(fault==1) fsm.imu_data.msg.header.stamp=ros::Time(t-1.);
      if(fault==2) fsm.odom_data.msg.header.stamp=ros::Time(t-1.);
      if(fault==3) fsm.state_data.current_state.armed=false;
      if(fault==4) fsm.state_data.current_state.connected=false;
      if(fault==5) fsm.state_data.current_state.mode="POSCTL";
      if(fault==6) fsm.extended_state_data.current_extended_state.landed_state=mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
      fsm.process();
      check(std::abs(fsm.debug_msg.des_a_x-previous-.0003)<1e-9,
            "removed flight-feedback gate still interrupted integration");
    }
    // Direct fixed-position commands used by the original open-loop entry.
    feedback(fsm,t+=.01); fsm.rc_data.is_command_mode=true;
    quadrotor_msgs::PositionCommand::Ptr cmd(new quadrotor_msgs::PositionCommand);
    cmd->position.x=.2; cmd->position.z=1.; cmd->header.stamp=ros::Time(t);
    fsm.cmd_data.feed(cmd); fsm.process();
    check(fsm.get_state()==PX4CtrlFSM::CMD_CTRL && fsm.debug_msg.des_a_x>p.gain.Kp0*.2+.005,
          "fixed-position command handoff lost learned compensation");
    feedback(fsm,t+=.01); fsm.rc_data.is_hover_mode=false; fsm.process();
    check(fsm.get_state()==PX4CtrlFSM::MANUAL_CTRL && std::abs(fsm.debug_msg.des_a_x)<1e-9,
          "pilot takeover did not clear integral");
    feedback(fsm,t+=.01); fsm.cmd_data.rcv_stamp=ros::Time(0);
    fsm.rc_data.is_hover_mode=true; fsm.rc_data.is_command_mode=false;
    fsm.rc_data.enter_hover_mode=true; fsm.process(); fsm.hover_pose.x()=.2;
    learn();
    const double learned_before_land=fsm.debug_msg.des_a_x-p.gain.Kp0*.2;
    feedback(fsm,t+=.01); fsm.rc_data.is_command_mode=true;
    std_srvs::Trigger::Request request; std_srvs::Trigger::Response response;
    fsm.force_land(request,response); check(response.success,"synthetic landing handoff rejected");
    fsm.process(); feedback(fsm,t+=.01); fsm.process();
    check(fsm.get_state()==PX4CtrlFSM::AUTO_LAND &&
          std::abs(fsm.debug_msg.des_a_x-learned_before_land)<1e-9,
          "landing cleared accumulated integral");
    const double landing_force=fsm.debug_msg.des_a_x;
    feedback(fsm,t+=.01); fsm.odom_data.p.x()=-.2; fsm.process();
    check(std::abs(fsm.debug_msg.des_a_x-landing_force-p.gain.Kp0*.2-.0003)<1e-9,
          "landing did not continue integrating horizontal error");
    feedback(fsm,t+=.01); fsm.odom_data.rcv_stamp=ros::Time(t-1.); fsm.process();
    check(fsm.get_state()==PX4CtrlFSM::MANUAL_CTRL,
          "original odometry-loss handoff changed");
    feedback(fsm,t+=.01); fsm.state_data.current_state.armed=false;
    fsm.cmd_data.rcv_stamp=ros::Time(0); fsm.process();
    p.takeoff_land.enable_auto_arm=false;
    p.takeoff_land.enable=true;
    airseafly_real400_msgs::MissionTakeoff::Request takeoff;
    airseafly_real400_msgs::MissionTakeoff::Response takeoff_response;
    takeoff.target_world_z=2.;
    fsm.mission_takeoff(takeoff,takeoff_response);
    check(takeoff_response.accepted,"synthetic takeoff request rejected");
    // Advance only the private simulation clock during the FSM's existing
    // short OFFBOARD handoff delay. The arming path is disabled above.
    std::atomic<bool> advance{true};
    std::thread clock([&]() {
      double clock_time=t;
      while(advance) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        ros::Time::setNow(ros::Time(clock_time+=.01));
      }
    });
    fsm.process(); advance=false; clock.join();
    check(fsm.get_state()==PX4CtrlFSM::AUTO_TAKEOFF,"takeoff handoff failed");
    t=ros::Time::now().toSec();
    for(int n=0;n<30;++n) {
      feedback(fsm,t+=.01); fsm.odom_data.p.x()=-.2; fsm.process();
    }
    check(fsm.debug_msg.des_a_x>p.gain.Kp0*.2+.005,"takeoff still gated horizontal integral");
    feedback(fsm,t+=3.2); fsm.odom_data.p.x()=-.2; fsm.process();
    check(fsm.get_state()==PX4CtrlFSM::AUTO_TAKEOFF && fsm.debug_msg.des_a_x>p.gain.Kp0*.2+.09,
          "climbing or long control interval interrupted horizontal integral");
    std::cout<<"{\"cases\":"<<cases<<",\"failures\":0,\"real_controller_and_fsm\":true,\"hardware_used\":false}\n";
  } catch(const std::exception &e) {
    std::cerr<<"FAIL after "<<cases<<" checks: "<<e.what()<<'\n'; ros::shutdown(); return 1;
  }
  ros::shutdown(); return 0;
}
