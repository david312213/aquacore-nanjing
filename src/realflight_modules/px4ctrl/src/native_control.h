#pragma once
#include "native_owner.h"
#include "landing_request.h"
#include "native_setpoint.h"
#include "PX4CtrlFSM.h"
#include <px4ctrl/ControlOwner.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/EstimatorStatus.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iomanip>
#include <future>
#include <memory>
#include <stdexcept>
#include <thread>

static_assert(native_control::kPositionYawMask ==
    (mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
     mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX |
     mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
     mavros_msgs::PositionTarget::IGNORE_YAW_RATE), "position/yaw mask must match MAVROS");

// Lives in the same spinOnce thread as PX4CtrlFSM. This node is the single
// publisher for both MAVROS control inputs; native ownership skips the FSM.
class NativeControl {
  PX4CtrlFSM& fsm;
  native_control::Gate gate;
  native_control::FreshnessWatchdog watchdog;
  ros::Publisher output, status;
  ros::Subscriber pose_sub, velocity_sub, estimator_sub, bridge_sub, command_sub, thrust_sub;
  ros::ServiceServer owner_service;
  geometry_msgs::PoseStamped pose;
  geometry_msgs::TwistStamped velocity;
  mavros_msgs::PositionTarget command, anchor;
  ros::WallTime pose_at, velocity_at, estimator_at, bridge_at, command_at;
  ros::WallTime last_publish, last_status;
  ros::WallTime next_publish;
  native_control::LandingRequest landing;
  std::string watchdog_reason;
  double output_rate_hz = 30.;
  bool stop_on_mode_exit = false;
  native_control::ModeExitLatch mode_exit;
  ros::Time legacy_after;
  std::future<bool> land_call;
  std::uint64_t land_call_generation = 0;
  bool estimator_ok = false, bridge_ok = false, anchor_valid = false, pose_received = false;
  bool thrust_received = false;
  bool priority_land = false;
  double thrust = 0.;
  std::string reason = "legacy control";
  static bool fresh(ros::WallTime t, double seconds) {
    const double age = (ros::WallTime::now()-t).toSec();
    return !t.isZero() && age >= 0. && age <= seconds;
  }
  static bool stamp_fresh(ros::Time t, double seconds) {
    const double age = (ros::Time::now()-t).toSec();
    return !t.isZero() && age >= -0.05 && age <= seconds;
  }
  static bool finite(double x) { return std::isfinite(x); }
  native_control::Health health() const {
    // Received finite inputs remain usable across source-time gaps.
    // They do not cancel a running task or request AUTO.LAND after 0.30 s.
    native_control::Health h;
    h.connected = fsm.state_data.current_state.connected && stamp_fresh(fsm.state_data.rcv_stamp, 2.5);
    h.armed = fsm.state_data.current_state.armed;
    h.offboard = fsm.state_data.current_state.mode == "OFFBOARD";
    h.auto_land = fsm.state_data.current_state.mode == "AUTO.LAND";
    h.pose = pose_received && !velocity_at.isZero();
    h.estimator = estimator_ok && fresh(estimator_at,2.);
    h.bridge = bridge_ok;
    h.rc = fsm.param.takeoff_land.no_RC ||
           (fsm.rc_is_received(ros::Time::now()) && fsm.rc_data.is_hover_mode && fsm.rc_data.is_command_mode);
    h.command = !command_at.isZero();
    return h;
  }
  void capture() {
    anchor = mavros_msgs::PositionTarget();
    anchor.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    anchor.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY |
                       mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    anchor.position = pose.pose.position;
    const auto& q = pose.pose.orientation;
    anchor.yaw = std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    anchor_valid = true;
  }
  bool request(const std::string& owner) {
    const auto h = health();
    if (owner == "STATUS") return true;
    if (owner == "PX4_NATIVE") {
      if (!h.connected || !h.pose || !h.bridge) {
        reason = "native acquisition requires valid PX4 pose and localization"; return false;
      }
      if (land_call.valid()) { reason="previous mode request still pending"; return false; }
      landing.reset();
      capture();
      gate.acquire(h);
      watchdog.reset(); watchdog_reason.clear();
      // Initialize native output with the measured pose on each acquisition.
      command = anchor;
      command.header.stamp = ros::Time::now();
      command_at = ros::WallTime::now();
      fsm.enter_native();
      mode_exit.reset();
      next_publish = ros::WallTime();
      reason = "native acquired at current pose";
      return true;
    }
    if (owner == "PX4_HOLD" && gate.owner != native_control::Owner::Legacy) {
      if (watchdog.action==native_control::WatchdogAction::Land || watchdog.action==native_control::WatchdogAction::Stop) {
        reason="watchdog latched; explicit native acquisition required"; return false;
      }
      if (!h.pose || !h.estimator) { reason = "valid PX4 pose required for hold"; return false; }
      landing.cancel("landing cancelled by explicit hold request");
      capture(); gate.owner = native_control::Owner::Hold; reason = "explicit native hold"; return true;
    }
    if (owner == "PX4CTRL") {
      if (gate.owner == native_control::Owner::Legacy) return true;
      if (!h.armed) {
        landing.cancel("landing cancelled by explicit legacy control request");
        legacy_after=ros::Time::now();
        gate.owner = native_control::Owner::Legacy;
        fsm.leave_native(false,thrust_received ? thrust : fsm.param.thr_map.hover_percentage);
        reason = "legacy controller restored while disarmed"; return true;
      }
      if (fsm.odom_data.rcv_stamp.isZero()) {
        reason = "FAST-LIO pose has not been received for legacy hover"; return false;
      }
      landing.cancel("landing cancelled by explicit legacy control request");
      fsm.leave_native(true,thrust_received ? thrust : fsm.param.thr_map.hover_percentage);
      legacy_after=ros::Time::now();
      gate.owner = native_control::Owner::Legacy;
      command_at = ros::WallTime();
      reason = "legacy hover initialized from current pose and thrust"; return true;
    }
    if (owner == "PX4_AUTO_LAND" && gate.owner != native_control::Owner::Legacy) {
      if (gate.owner == native_control::Owner::AutoLand) return true;
      if (land_call.valid()) { reason="previous mode request still pending"; return false; }
      if (mode_exit.paused) { reason="manual mode exit latched; landing request not sent"; return false; }
      // Request PX4's landing mode even if localization has failed. Only a
      // fresh valid estimate permits a temporary hold while the mode changes.
      if (h.pose && h.estimator) capture();
      else anchor_valid=false;
      gate.owner=native_control::Owner::AutoLand;
      landing.start(ros::WallTime::now().toSec());
      mode_exit.start_landing();
      // accepted acknowledges the transaction only. process() dispatches its
      // bounded attempts and observes the actual FCU mode without blocking.
      reason=landing.reason(); return true;
    }
    reason="invalid owner transition"; return false;
  }
 public:
  NativeControl(ros::NodeHandle& nh, PX4CtrlFSM& f): fsm(f) {
    nh.param("native_output_rate_hz", output_rate_hz, 30.);
    nh.param("native_stop_on_mode_exit", stop_on_mode_exit, false);
    if (!finite(output_rate_hz) || output_rate_hz<=0.)
      throw std::invalid_argument("native_output_rate_hz must be positive");
    output=nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",1);
    status=nh.advertise<std_msgs::String>("native_status",1,true);
    pose_sub=nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",1,
      [this](const geometry_msgs::PoseStamped::ConstPtr& m) {
        const auto& p=m->pose.position; const auto& q=m->pose.orientation;
        const double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
        if (finite(p.x)&&finite(p.y)&&finite(p.z)&&finite(n)&&n>0.) {
          pose=*m; pose_at=ros::WallTime::now();
          const double scale=1./std::sqrt(n);
          pose.pose.orientation.x*=scale; pose.pose.orientation.y*=scale;
          pose.pose.orientation.z*=scale; pose.pose.orientation.w*=scale;
          pose_received=true;
        }
      });
    velocity_sub=nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",1,
      [this](const geometry_msgs::TwistStamped::ConstPtr& m) {
        const auto& v=m->twist.linear;
        if (finite(v.x)&&finite(v.y)&&finite(v.z)) {
          velocity=*m; velocity_at=ros::WallTime::now();
        }
      });
    estimator_sub=nh.subscribe<mavros_msgs::EstimatorStatus>("/mavros/estimator_status",1,
      [this](const mavros_msgs::EstimatorStatus::ConstPtr& m) {
        // Diagnostic during tracking, required for a synthesized hold.
        // PX4 1.17 CONST_POS also means vehicle_at_rest.
        // PRED_POS_HORIZ_REL reports horizontal aiding without identifying
        // the EV source or constraining control ownership.
        estimator_ok=m->attitude_status_flag && m->velocity_horiz_status_flag && m->velocity_vert_status_flag &&
          (m->pos_horiz_rel_status_flag || m->pos_horiz_abs_status_flag) && m->pos_vert_abs_status_flag &&
          m->pred_pos_horiz_rel_status_flag && !m->accel_error_status_flag;
        estimator_at=ros::WallTime::now();
      });
    bridge_sub=nh.subscribe<std_msgs::Bool>("/airseafly/native/localization_healthy",1,
      [this](const std_msgs::Bool::ConstPtr& m) {bridge_ok=m->data; bridge_at=ros::WallTime::now();});
    command_sub=nh.subscribe<mavros_msgs::PositionTarget>("native_setpoint",1,
      [this](const mavros_msgs::PositionTarget::ConstPtr& m) {
        const bool full = m->type_mask==0;
        const bool position_only = m->type_mask==native_control::kPositionYawMask;
        if (m->coordinate_frame!=mavros_msgs::PositionTarget::FRAME_LOCAL_NED ||
            !native_control::supported_local_mask(m->type_mask) ||
            !finite(m->position.x)||!finite(m->position.y)||!finite(m->position.z)||!finite(m->yaw)||
            (!position_only && (!finite(m->velocity.x)||!finite(m->velocity.y)||!finite(m->velocity.z)))||
            (full && (!finite(m->acceleration_or_force.x)||!finite(m->acceleration_or_force.y)||
                      !finite(m->acceleration_or_force.z)||!finite(m->yaw_rate)))) return;
        command=*m; command_at=ros::WallTime::now();
      });
    thrust_sub=nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude",1,
      [this](const mavros_msgs::AttitudeTarget::ConstPtr& m) {
        if (gate.owner!=native_control::Owner::Legacy && finite(m->thrust)) {
          thrust=m->thrust; thrust_received=true;
        }
      });
    owner_service=nh.advertiseService<px4ctrl::ControlOwner::Request,px4ctrl::ControlOwner::Response>(
      "control_owner", [this](px4ctrl::ControlOwner::Request& req,px4ctrl::ControlOwner::Response& res) {
        res.accepted=request(req.owner); res.owner=name(); res.message=reason;
        res.position_enu[0]=pose.pose.position.x;
        res.position_enu[1]=pose.pose.position.y;
        res.position_enu[2]=pose.pose.position.z;
        const auto& q=pose.pose.orientation;
        res.yaw_enu=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        res.pose_stamp=pose.header.stamp.toSec();
        return true;
      });
  }
  const char* name() const {
    switch(gate.owner) {
      case native_control::Owner::Legacy:return "PX4CTRL";
      case native_control::Owner::Native:return "PX4_NATIVE";
      case native_control::Owner::Hold:return "PX4_HOLD";
      case native_control::Owner::AutoLand:return "PX4_AUTO_LAND";
    }
    return "PX4CTRL";
  }
  bool owns() const {return gate.owner!=native_control::Owner::Legacy;}
  void legacy_command(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    if (owns()) return;
    if (!legacy_after.isZero() && msg->header.stamp<=legacy_after) return;
    fsm.cmd_data.feed(msg);
  }
  bool priority_landing() const {
    return priority_land && gate.owner==native_control::Owner::AutoLand &&
      fsm.state_data.current_state.armed &&
      (fsm.state_data.current_state.mode=="AUTO.LAND" || fsm.state_data.current_state.mode=="OFFBOARD");
  }
  bool landed() const {
    return !fsm.state_data.current_state.armed &&
      fsm.extended_state_data.current_extended_state.landed_state==mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
  }
  bool force_land(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res) {
    if (!owns()) return fsm.force_land(req,res);
    res.success=request("PX4_AUTO_LAND"); priority_land=res.success; res.message=reason; return true;
  }
  void process() {
    const auto h=health();
    const auto wall_now=ros::WallTime::now();
    const double landing_now=wall_now.toSec();
    if (gate.owner!=native_control::Owner::AutoLand || !h.armed) priority_land=false;
    // Process authoritative mode feedback before a possibly late RPC result.
    landing.update(landing_now,h,landed());
    if (land_call.valid() && land_call.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      const bool accepted=land_call.get();
      landing.rpc_result(land_call_generation,accepted,landing_now);
    }
    if (landing.take_attempt(landing_now,land_call.valid())) {
      land_call_generation=landing.generation();
      auto completion=std::make_shared<std::promise<bool>>();
      land_call=completion->get_future();
      auto client=fsm.set_FCU_mode_srv;
      // Unlike an async future, destroying this promise-backed future does not
      // wait on a stuck RPC during shutdown. The thread owns no NativeControl.
      try {
        std::thread([client,completion]() mutable {
          bool accepted=false;
          try {
            mavros_msgs::SetMode mode; mode.request.custom_mode="AUTO.LAND";
            accepted=client.call(mode) && mode.response.mode_sent;
          } catch (...) { accepted=false; }
          completion->set_value(accepted);
        }).detach();
      } catch (...) {
        completion->set_value(false);
      }
    }
    if (gate.owner==native_control::Owner::AutoLand) {
      reason=landing.reason();
      if (landing.failed()) {
        if (h.pose && h.estimator) capture();
        else anchor_valid=false;
        gate.owner=native_control::Owner::Hold;
        // The failed landing transaction is already terminal. Do not let an
        // earlier hold watchdog immediately create another automatic round.
        watchdog.reset(); watchdog_reason.clear();
        priority_land=false;
      } else if (landing.state()==native_control::LandingRequest::State::Cancelled) {
        gate.owner=native_control::Owner::Hold;
        mode_exit.paused=true;
        priority_land=false;
      }
    }
    if ((landing.requesting() || landing.failed()) && !anchor_valid &&
        h.pose && h.estimator && h.offboard && !mode_exit.paused) {
      capture();
    }
    if ((stop_on_mode_exit || landing.failed()) &&
        (gate.owner==native_control::Owner::Native || gate.owner==native_control::Owner::Hold ||
         (gate.owner==native_control::Owner::AutoLand && !landed())) &&
        mode_exit.update(gate.owner,h))
      reason=gate.owner==native_control::Owner::AutoLand
        ? "mode left AUTO.LAND; waiting for explicit native acquisition"
        : "mode left OFFBOARD; waiting for explicit native acquisition";
    if (!mode_exit.paused &&
        (gate.owner==native_control::Owner::Native || gate.owner==native_control::Owner::Hold)) {
      const auto previous=watchdog.action;
      const auto action=watchdog.update(h, gate.owner==native_control::Owner::Hold && !landing.failed());
      if (action!=previous) {
        if (action==native_control::WatchdogAction::Land) {
          watchdog_reason="valid estimator unavailable during explicit hold; AUTO.LAND requested";
          if (!request("PX4_AUTO_LAND")) reason="watchdog could not request AUTO.LAND: "+reason;
        } else if (action==native_control::WatchdogAction::Stop) {
          watchdog_reason="FCU connection unavailable; native output stopped until explicit acquisition";
          reason=watchdog_reason;
        }
        ROS_ERROR_STREAM("[native watchdog] " << watchdog_reason);
      }
    }
    const auto selected=gate.update(h);
    const bool output_valid=(gate.owner==native_control::Owner::AutoLand ||
                             (gate.owner==native_control::Owner::Hold && landing.failed()))
      ? landing.hold_output_allowed(h) &&
        (gate.owner==native_control::Owner::AutoLand || watchdog.action!=native_control::WatchdogAction::Stop)
      : native_control::output_allowed(gate.owner,watchdog.action,h,false);
    if (selected==native_control::Output::Local && output_valid && anchor_valid && !mode_exit.paused &&
        (next_publish.isZero() || wall_now>=next_publish)) {
      auto target=gate.owner==native_control::Owner::Native ? command : anchor;
      if (gate.owner==native_control::Owner::Native && !h.command) {
        // If no command exists, keep the acquired position/yaw anchor.
        target.type_mask=native_control::kPositionYawMask;
        target.velocity.x=target.velocity.y=target.velocity.z=0.;
        target.acceleration_or_force.x=target.acceleration_or_force.y=target.acceleration_or_force.z=0.;
        target.yaw_rate=0.;
      }
      // The wait point is a current hold stream, not a newly accepted command.
      if (gate.owner!=native_control::Owner::Native || !h.command)
        target.header.stamp=ros::Time::now();
      target.header.frame_id="map";
      // MAVROS converts ROS ENU to MAVLink LOCAL_NED; never swap axes here.
      output.publish(target); last_publish=ros::WallTime::now();
      const ros::WallDuration period(1./output_rate_hz);
      if (next_publish.isZero()) next_publish=wall_now+period;
      else do {next_publish+=period;} while(next_publish<=wall_now);
    }
    if (!fresh(last_status,.05)) {
      const auto& p=pose.pose.position; const auto& v=velocity.twist.linear; const auto& q=pose.pose.orientation;
      const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
      std::ostringstream s; s<<std::setprecision(12)<<std::boolalpha;
      s<<"{\"stamp\":"<<ros::Time::now().toSec()<<",\"pose_stamp\":"<<pose.header.stamp.toSec()
       <<",\"owner\":\""<<name()<<"\",\"enabled\":true"
       <<",\"healthy\":"<<(h.connected&&h.pose&&h.estimator&&h.bridge)
       <<",\"connected\":"<<h.connected<<",\"pose_received\":"<<pose_received
       <<",\"state_stamp\":"<<fsm.state_data.rcv_stamp.toSec()
       <<",\"extended_state_stamp\":"<<fsm.extended_state_data.rcv_stamp.toSec()
       <<",\"velocity_stamp\":"<<velocity.header.stamp.toSec()
       <<",\"manual_paused\":"<<mode_exit.paused<<",\"output_rate_hz\":"<<output_rate_hz
       <<",\"pose_fresh\":"<<(fresh(pose_at,.30) && stamp_fresh(pose.header.stamp,.30) && fresh(velocity_at,.30) && stamp_fresh(velocity.header.stamp,.30))<<",\"estimator_healthy\":"<<h.estimator
       <<",\"bridge_healthy\":"<<h.bridge<<",\"command_fresh\":"<<(fresh(command_at,.30) && stamp_fresh(command.header.stamp,.30))
       <<",\"watchdog_fault\":"<<(watchdog.action!=native_control::WatchdogAction::Track)
       <<",\"watchdog_reason\":\""<<watchdog_reason<<"\""
       <<",\"landing_state\":\""<<landing.name()<<"\",\"landing_attempts\":"<<landing.attempts()
       <<",\"landing_reason\":\""<<landing.reason()<<"\""
       <<",\"armed\":"<<h.armed<<",\"offboard\":"<<h.offboard
       <<",\"mode\":\""<<fsm.state_data.current_state.mode<<"\""
       <<",\"landed\":"<<landed()
       <<",\"position\":["<<p.x<<","<<p.y<<","<<p.z<<"],\"velocity\":["<<v.x<<","<<v.y<<","<<v.z
       <<"],\"yaw\":"<<yaw<<",\"reason\":\""<<reason<<"\"}";
      std_msgs::String msg; msg.data=s.str(); status.publish(msg); last_status=ros::WallTime::now();
    }
  }
};
