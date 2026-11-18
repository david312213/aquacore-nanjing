// Actual FSM and controller functions, synthetic inputs and PRIVATE master only.
#include "PX4CtrlFSM.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

static void check(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}
static bool mode(mavros_msgs::SetMode::Request &, mavros_msgs::SetMode::Response &r)
{ r.mode_sent = true; return true; }
static bool arm(mavros_msgs::CommandBool::Request &q, mavros_msgs::CommandBool::Response &r)
{ check(!q.value, "test must never arm"); r.success = true; return true; }

static void pose(PX4CtrlFSM &fsm, double z=.65)
{
  nav_msgs::Odometry::Ptr m(new nav_msgs::Odometry);
  m->pose.pose.position.x=2.; m->pose.pose.position.y=-3.; m->pose.pose.position.z=z;
  m->pose.pose.orientation.w=std::cos(.2); m->pose.pose.orientation.z=std::sin(.2);
  fsm.odom_data.feed(m);
  fsm.imu_data.q=fsm.odom_data.q;
  fsm.imu_data.a=Eigen::Vector3d(0,0,9.81);
  fsm.imu_data.w.setZero();
  fsm.imu_data.rcv_stamp=ros::Time::now();
}
static void moving_goal(PX4CtrlFSM &fsm)
{
  quadrotor_msgs::PositionCommand::Ptr m(new quadrotor_msgs::PositionCommand);
  m->position.x=30.; m->position.y=-20.; m->position.z=10.; m->yaw=1.8;
  fsm.cmd_data.feed(m);
}
static void request(PX4CtrlFSM &fsm, bool expected=true)
{
  std_srvs::Trigger::Request q; std_srvs::Trigger::Response r;
  fsm.force_land(q,r);
  check(r.success==expected,"unexpected force_land response");
}

int main(int argc, char **argv)
{
  const std::string scenario=argc>1 ? argv[1] : "cmd";
  const char *master=std::getenv("ROS_MASTER_URI");
  if (!master || std::string(master).find("http://127.0.0.1:")!=0 ||
      std::string(master).find(":11341")!=std::string::npos ||
      std::string(master).find(":11311")!=std::string::npos) return 2;
  ros::init(argc,argv,"force_land_offline",ros::init_options::AnonymousName);
  ros::NodeHandle n("/offline_force_land");
  auto mode_service=n.advertiseService("mode",mode);
  auto arm_service=n.advertiseService("arm",arm);
  ros::AsyncSpinner spinner(1); spinner.start();
  try
  {
    Parameter_t p;
    p.gain=Parameter_t::Gain{};
    p.gain.Kp0=p.gain.Kp1=p.gain.Kp2=2.;
    p.gain.Kv0=p.gain.Kv1=p.gain.Kv2=1.5;
    p.gra=9.81; p.thr_map=Parameter_t::ThrustMapping{}; p.thr_map.hover_percentage=.58;
    p.takeoff_land=Parameter_t::AutoTakeoffLand{true,false,false,.65,.3};
    p.msg_timeout=Parameter_t::MsgTimeout{.5,.5,.5,.5,.5};
    p.rc_reverse=Parameter_t::RCReverse{false,false,false,false};
    p.use_bodyrate_ctrl=false; p.max_manual_vel=1.; p.max_angle=std::acos(-1.0)/6.;
    LinearControl controller(p); PX4CtrlFSM fsm(p,controller);
    fsm.ctrl_FCU_pub=n.advertise<mavros_msgs::AttitudeTarget>("attitude",10);
    fsm.debug_pub=n.advertise<quadrotor_msgs::Px4ctrlDebug>("debug",10);
    fsm.traj_start_trigger_pub=n.advertise<geometry_msgs::PoseStamped>("trigger",10);
    fsm.set_FCU_mode_srv=n.serviceClient<mavros_msgs::SetMode>("mode");
    fsm.arming_client_srv=n.serviceClient<mavros_msgs::CommandBool>("arm");
    check(fsm.set_FCU_mode_srv.waitForExistence(ros::Duration(3)),"mock mode missing");
    pose(fsm);
    fsm.state_data.current_state.mode="OFFBOARD";
    fsm.state_data.current_state.armed=true;
    request(fsm,false); // Never seize MANUAL control or arm the aircraft.
    if (scenario=="takeoff")
    {
      pose(fsm,0.);
      quadrotor_msgs::TakeoffLand::Ptr m(new quadrotor_msgs::TakeoffLand);
      m->takeoff_land_cmd=quadrotor_msgs::TakeoffLand::TAKEOFF;
      fsm.feed_takeoff_land(m); fsm.process();
      check(fsm.get_state()==PX4CtrlFSM::AUTO_TAKEOFF,"takeoff setup failed");
      pose(fsm,.3);
    }
    else
    {
      fsm.rc_data.enter_hover_mode=true; fsm.process();
      check(fsm.get_state()==PX4CtrlFSM::AUTO_HOVER,"hover setup failed");
      if (scenario!="hover")
      {
        moving_goal(fsm); fsm.process();
        check(fsm.get_state()==PX4CtrlFSM::CMD_CTRL,"cmd setup failed");
      }
    }
    request(fsm); request(fsm); // Duplicate before processing.
    moving_goal(fsm); fsm.process();
    check(fsm.get_state()==PX4CtrlFSM::AUTO_HOVER,"missing explicit hover handoff");
    check(fsm.force_land_active(),"landing ownership not retained");
    const Eigen::Vector4d anchor=fsm.hover_pose;
    const double t=ros::Time::now().toSec();
    check(std::abs(anchor.x()-2.)<1e-9 && std::abs(anchor.y()+3.)<1e-9,"wrong XY anchor");
    request(fsm); // Must not reset the hover stage indefinitely.
    ros::Time::setNow(ros::Time(t+.01));
    pose(fsm,anchor.z()); moving_goal(fsm); fsm.process();
    check(fsm.get_state()==PX4CtrlFSM::AUTO_LAND,"fresh command stole landing handoff");
    check(std::abs(fsm.debug_msg.des_v_z)<1e-6,"landing starts with a vertical velocity step");
    for (int i=1;i<=10;++i)
    {
      ros::Time::setNow(ros::Time(t+.01+i*.1));
      pose(fsm,anchor.z()-.3*i*.1); moving_goal(fsm); request(fsm); fsm.process();
      check(fsm.get_state()==PX4CtrlFSM::AUTO_LAND,"command/duplicate restarted landing");
      check(std::abs(fsm.debug_msg.des_a_x)<1e-6 && std::abs(fsm.debug_msg.des_a_y)<1e-6,"XY followed old waypoint");
    }
    if (scenario=="rc" || scenario=="rc_command")
    {
      if (scenario=="rc") fsm.rc_data.is_hover_mode=false;
      else fsm.rc_data.is_command_mode=false;
      fsm.process();
      check(!fsm.force_land_active(),"RC takeover ignored");
      check(fsm.get_state()==(scenario=="rc" ? PX4CtrlFSM::MANUAL_CTRL : PX4CtrlFSM::AUTO_HOVER),"RC fallback changed");
      request(fsm,false);
    }
    else if (scenario=="odom")
    {
      ros::Time::setNow(ros::Time(t+10.)); moving_goal(fsm); fsm.process();
      check(fsm.get_state()==PX4CtrlFSM::MANUAL_CTRL && !fsm.force_land_active(),"odom fallback changed");
    }
    else
    {
      fsm.extended_state_data.current_extended_state.landed_state=mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
      for (int i=0;i<50 && fsm.get_state()!=PX4CtrlFSM::MANUAL_CTRL;++i)
      {
        ros::Time::setNow(ros::Time(t+10.+i*.1)); pose(fsm,0.); fsm.process();
      }
      check(fsm.get_state()==PX4CtrlFSM::MANUAL_CTRL && !fsm.force_land_active(),"landing did not complete with mock disarm");
    }
    std::cout << "PASS " << scenario << ": real FSM, no hardware; priority, anchor and existing fallback verified\n";
  }
  catch(const std::exception &e) { std::cerr<<"FAIL "<<scenario<<": "<<e.what()<<'\n'; ros::shutdown(); return 1; }
  ros::shutdown(); return 0;
}
