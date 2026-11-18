#include "controller.h"
#include "control_limits.h"
#include <std_msgs/Float64MultiArray.h>
#include <chrono>

namespace {
double monotonicSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
payload_pd::Gains basePD(const Parameter_t &p) {
  return {{{p.gain.Kp0, p.gain.Kp1, p.gain.Kp2}},
          {{p.gain.Kv0, p.gain.Kv1, p.gain.Kv2}}};
}
std::array<payload_pd::Gains, 4> profiles(const Parameter_t &p) {
  // Disabled scheduling does not inspect optional profiles (including tests).
  return p.payload_pd_enabled ? p.payload_pd_profiles :
      std::array<payload_pd::Gains, 4>{{basePD(p), basePD(p), basePD(p), basePD(p)}};
}
}

using namespace std;



double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param),
  payload_pd_(basePD(param), profiles(param), param.payload_pd_enabled, param.payload_pd_transition_s)
{
  payload_pd_instance_id_ = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  airseafly_control::validate_tilt_limit(param_.max_angle);
  resetThrustMapping();
  ros::NodeHandle diagnostics_nh;
  pd_trace_pub_ = diagnostics_nh.advertise<std_msgs::Float64MultiArray>("/px4ctrl/pd_trace", 2);
}

bool LinearControl::payloadPDRequest(px4ctrl::PayloadPD::Request &request,
    px4ctrl::PayloadPD::Response &response, bool ground_reset_allowed)
{
  const double now = monotonicSeconds();
  if (request.operation == px4ctrl::PayloadPD::Request::STATUS) {
    response.success = true;
    response.message = "payload inventory is ACK-based; physical release unverified";
  } else if (request.operation == px4ctrl::PayloadPD::Request::RESET) {
    response.success = payload_pd_.reset(request.loaded_mask, ground_reset_allowed, now, response.message);
  } else if (request.operation == px4ctrl::PayloadPD::Request::RELEASE) {
    if (request.inventory_id != payload_pd_instance_id_ + ":" + std::to_string(payload_pd_.generation())) {
      response.success = false;
      response.message = "stale inventory_id; controller restarted or payload inventory reset";
    } else {
      response.success = payload_pd_.release(request.slot, now, response.message);
    }
  } else {
    response.success = false;
    response.message = "unknown payload PD operation";
  }
  response.enabled = payload_pd_.enabled();
  response.ready = payload_pd_.ready();
  response.loaded_mask = payload_pd_.loadedMask();
  response.remaining = payload_pd_.remaining();
  response.inventory_id = payload_pd_instance_id_ + ":" + std::to_string(payload_pd_.generation());
  const auto current = payload_pd_.gains(now);
  std::copy(current.kp.begin(), current.kp.end(), response.kp.begin());
  std::copy(current.kv.begin(), current.kv.end(), response.kv.begin());
  if (request.operation != px4ctrl::PayloadPD::Request::STATUS)
    ROS_INFO_STREAM("[PX4Ctrl] payload PD: " << response.message << "; remaining=" << int(response.remaining));
  return true;  // Application rejection is response.success, not a transport failure.
}

/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/
quadrotor_msgs::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
  /* WRITE YOUR CODE HERE */
      //compute disired acceleration
      Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
      Eigen::Vector3d Kp,Kv;
      const auto gains = payload_pd_.gains(monotonicSeconds());
      Kp << gains.kp[0], gains.kp[1], gains.kp[2];
      Kv << gains.kv[0], gains.kv[1], gains.kv[2];
      des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);

      // FAST-LIO's initial world need not be gravity-aligned. Feedback is in
      // that world, while gravity is vertical in the FCU inertial frame.
      // Use the SAME frame transform here and below; adding raw-world +Z
      // before a tilted frame conversion produces a constant lateral force
      // even when position and velocity errors are zero.
      const Eigen::Quaterniond world_to_fcu = (imu.q * odom.q.inverse()).normalized();
      const Eigen::Vector3d gravity_in_world =
        world_to_fcu.conjugate() * Eigen::Vector3d(0, 0, param_.gra);
      des_acc += gravity_in_world;

      des_acc = airseafly_control::limit_tilt(des_acc, world_to_fcu, param_.max_angle);

      u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
      // ROS world/body convention: positive body Z is the thrust direction.
      // Construct the desired attitude from one WORLD acceleration vector and
      // the DESIRED yaw. Actual yaw must not rotate that vector a second time.
      const double accel_norm = des_acc.norm();
      Eigen::Vector3d thrust_axis = world_to_fcu.conjugate() * Eigen::Vector3d::UnitZ();
      if (accel_norm > 1e-9)
        thrust_axis = des_acc / accel_norm;

      const Eigen::Vector3d yaw_lateral(-std::sin(des.yaw), std::cos(des.yaw), 0.0);
      Eigen::Vector3d forward_axis = yaw_lateral.cross(thrust_axis);
      if (thrust_axis.z() < 0.0)
        forward_axis = -forward_axis;
      if (forward_axis.squaredNorm() < 1e-12)
      {
        // At this horizontal-thrust singularity yaw is not independently
        // defined. Choose an orthogonal axis solely to avoid division by zero.
        forward_axis = Eigen::Vector3d::UnitZ() - thrust_axis.z() * thrust_axis;
      }
      forward_axis.normalize();
      const Eigen::Vector3d lateral_axis = thrust_axis.cross(forward_axis).normalized();
      Eigen::Matrix3d desired_rotation;
      desired_rotation.col(0) = forward_axis;
      desired_rotation.col(1) = lateral_axis;
      desired_rotation.col(2) = thrust_axis;
      const Eigen::Quaterniond q(desired_rotation);
      // Preserve the existing odometry-world to FCU-world conversion.
      u.q = (world_to_fcu * q).normalized();


  /* WRITE YOUR CODE HERE */

  //used for debug
  // debug_msg_.des_p_x = des.p(0);
  // debug_msg_.des_p_y = des.p(1);
  // debug_msg_.des_p_z = des.p(2);
  
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;

  // Schema v1, 62 doubles. See hover_pid.py for the named layout.
  // Publish at most 25 Hz, only with subscribers; never log or render here.
  const ros::WallTime diagnostics_now = ros::WallTime::now();
  if (pd_trace_pub_.getNumSubscribers() > 0 &&
      (diagnostics_now - pd_trace_last_wall_).toSec() >= 0.04)
  {
    pd_trace_last_wall_ = diagnostics_now;
    try
    {
      std_msgs::Float64MultiArray trace;
      trace.data.reserve(62);
      trace.data = {1.0, ros::Time::now().toSec(),
                    odom.msg.header.stamp.toSec(), odom.rcv_stamp.toSec(),
                    imu.msg.header.stamp.toSec(), imu.rcv_stamp.toSec()};
      const auto add3 = [&trace](const Eigen::Vector3d &v) {
        for (int i = 0; i < 3; ++i) trace.data.push_back(v(i));
      };
      const auto addq = [&trace](const Eigen::Quaterniond &q) {
        trace.data.push_back(q.x()); trace.data.push_back(q.y());
        trace.data.push_back(q.z()); trace.data.push_back(q.w());
      };
      add3(des.p); add3(odom.p); add3(des.v); add3(odom.v); add3(des.a);
      add3(Kp); add3(Kv);
      add3(Kp.asDiagonal() * (des.p - odom.p));
      add3(Kv.asDiagonal() * (des.v - odom.v));
      add3(gravity_in_world); add3(des_acc);
      addq(u.q); addq(imu.q);
      trace.data.push_back(u.thrust);
      trace.data.push_back(param_.gra);
      trace.data.push_back(thr2acc_);
      trace.data.push_back(param_.thr_map.hover_percentage);
      trace.data.push_back(des.yaw);
      addq(odom.q); add3(imu.w); add3(imu.a);
      pd_trace_pub_.publish(trace);
    }
    catch (const std::exception &e)
    {
      ROS_WARN_THROTTLE(5.0, "PD diagnostics unavailable: %s", e.what());
    }
  }
  
  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage 
*/
double 
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  // Use the same vector for direction AND magnitude. The mapping/estimator
  // remains unchanged; at level hover this equals the previous Z-only value.
  throttle_percentage = airseafly_control::normalized_thrust(des_acc.norm() / thr2acc_);

  return throttle_percentage;
}

bool 
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    // Saturated output is not a reliable linear thrust-model observation.
    if (thr <= 0.0 || thr >= 1.0 || !std::isfinite(est_a(2)))
      continue;
    
    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    const double updated_mapping = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    const double updated_covariance = (1 - K * thr) * P_ / rho2_;
    if (!std::isfinite(updated_mapping) || updated_mapping <= 0.0 ||
        !std::isfinite(updated_covariance) || updated_covariance <= 0.0)
      return false;
    thr2acc_ = updated_mapping;
    P_ = updated_covariance;
    //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    //fflush(stdout);

    // debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void 
LinearControl::resetThrustMapping(double measured_hover)
{
  while (!timed_thrust_.empty()) timed_thrust_.pop();
  const double hover = std::isfinite(measured_hover) && measured_hover > .05 && measured_hover < .95
      ? measured_hover : param_.thr_map.hover_percentage;
  thr2acc_ = param_.gra / hover;
  P_ = 1e6;
}



