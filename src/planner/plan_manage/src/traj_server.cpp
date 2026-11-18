#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "traj_utils/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "plan_manage/goal_identity.h"
#include <airseafly_real400_msgs/EgoGoalTrajectory.h>
#include <airseafly_real400_msgs/EgoGoalCommand.h>
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <boost/make_shared.hpp>

ros::Publisher pos_cmd_pub;
ros::Publisher goal_cmd_pub;
ros::Publisher goal_finished_pub;
bool mission_goal_binding_ = false;
geometry_msgs::PoseStamped trajectory_goal_, pending_goal_;
bool trajectory_final_ = false, pending_final_ = false, completion_reported_ = false;

quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
std::int64_t traj_id_;
std::string world_frame_ = "world";

// Only the sender queues a future message; EGO planning itself is unchanged.
// Keeping the active spline prevents a future start_time from publishing zeros.
bool have_pending_traj_ = false;
vector<UniformBspline> pending_traj_;
ros::Time pending_start_time_;
std::int64_t pending_traj_id_;
double pending_duration_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;
bool mission_fixed_yaw_ = false;
double mission_yaw_ = 0.0;
bool hold_at_trajectory_end_ = false;

void bsplineCallback(traj_utils::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  pending_start_time_ = msg->start_time;
  pending_traj_id_ = msg->traj_id;
  pending_traj_.clear();
  pending_traj_.push_back(pos_traj);
  pending_traj_.push_back(pending_traj_[0].getDerivative());
  pending_traj_.push_back(pending_traj_[1].getDerivative());
  pending_duration_ = pending_traj_[0].getTimeSum();
  have_pending_traj_ = true;
  ROS_INFO_STREAM("[EGO trajectory queued] id=" << pending_traj_id_
      << " start=" << std::setprecision(17) << pending_start_time_.toSec()
      << " wait_s=" << (pending_start_time_ - ros::Time::now()).toSec());
}

void goalTrajectoryCallback(const airseafly_real400_msgs::EgoGoalTrajectoryConstPtr &msg)
{
  // Order timed splines, not waypoints. Start time also lets a restarted
  // planner's fresh spline replace one whose process-local ID was larger.
  const auto incoming = std::make_pair(msg->trajectory.start_time, msg->trajectory.traj_id);
  if ((receive_traj_ && incoming <= std::make_pair(start_time_, traj_id_)) ||
      (have_pending_traj_ && incoming <= std::make_pair(pending_start_time_, pending_traj_id_))) return;
  bsplineCallback(boost::make_shared<traj_utils::Bspline>(msg->trajectory));
  pending_goal_ = msg->goal;
  pending_final_ = msg->final_segment;
}

// Diagnostics compare both references at the same execution timestamp, not
// different adjacent samples. They report existing discontinuities, not repair them.
void activatePendingTrajectory(const ros::Time &now)
{
  if (!have_pending_traj_ || now < pending_start_time_) return;
  if (receive_traj_)
  {
    const double old_t = std::max(0.0, std::min(traj_duration_, (now - start_time_).toSec()));
    const double new_t = std::max(0.0, std::min(pending_duration_, (now - pending_start_time_).toSec()));
    const bool old_end = (now - start_time_).toSec() >= traj_duration_;
    const bool new_end = (now - pending_start_time_).toSec() >= pending_duration_;
    const Eigen::Vector3d dp = pending_traj_[0].evaluateDeBoorT(new_t) - traj_[0].evaluateDeBoorT(old_t);
    Eigen::Vector3d old_v = traj_[1].evaluateDeBoorT(old_t);
    if (old_end) old_v.setZero();
    Eigen::Vector3d old_a = traj_[2].evaluateDeBoorT(old_t);
    if (old_end) old_a.setZero();
    Eigen::Vector3d new_v = pending_traj_[1].evaluateDeBoorT(new_t);
    if (new_end) new_v.setZero();
    Eigen::Vector3d new_a = pending_traj_[2].evaluateDeBoorT(new_t);
    if (new_end) new_a.setZero();
    ROS_INFO_STREAM("[EGO trajectory switch] previous=" << traj_id_ << " next=" << pending_traj_id_
        << " sample_stamp=" << std::setprecision(17) << now.toSec()
        << " start_stamp=" << pending_start_time_.toSec()
        << " elapsed_s=" << (now - pending_start_time_).toSec()
        << " dp=" << dp.transpose() << " dv=" << (new_v - old_v).transpose()
        << " da=" << (new_a - old_a).transpose());
  }
  else
    ROS_INFO_STREAM("[EGO trajectory first] id=" << pending_traj_id_
        << " start_stamp=" << std::setprecision(17) << pending_start_time_.toSec()
        << " elapsed_s=" << (now - pending_start_time_).toSec());

  if (mission_goal_binding_ && !mission_fixed_yaw_ &&
      (!receive_traj_ || !ego_planner::sameGoal(trajectory_goal_, pending_goal_)))
  {
    // A persistent executor may have advanced the old spline's yaw while
    // native delivery held the aircraft. Each new goal carries the measured
    // heading at dispatch; only its first spline starts from that heading.
    // Replans for the same goal retain their continuous preview yaw.
    const auto &q = pending_goal_.pose.orientation;
    last_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    last_yaw_dot_ = 0.0;
  }

  traj_.swap(pending_traj_);
  traj_duration_ = pending_duration_;
  start_time_ = pending_start_time_;
  traj_id_ = pending_traj_id_;
  trajectory_goal_ = pending_goal_;
  trajectory_final_ = pending_final_;
  completion_reported_ = false;
  if (mission_goal_binding_)
    ROS_INFO_STREAM("[EGO trajectory executing] goal=" << trajectory_goal_.header.stamp << " id=" << traj_id_);
  receive_traj_ = true;
  have_pending_traj_ = false;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  // Navigation yaw is capped at 15 degrees/s, independently of search turns.
  constexpr double YAW_DOT_MAX_PER_SEC = 15.0 * PI / 180.0;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;
  // The first timer sample and a repeated ROS timestamp have zero elapsed time.
  const double dt = std::max(1e-6, (time_now - time_last).toSec());

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * dt;
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback(const ros::TimerEvent &e)
{
  ros::Time time_now = ros::Time::now();
  activatePendingTrajectory(time_now);
  // Before the first candidate starts, the upstream task retains its hold.
  if (!receive_traj_)
    return;
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = mission_fixed_yaw_
        ? std::make_pair(mission_yaw_, 0.0)
        : calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    // Task mode keeps the planned endpoint active until the mission manager
    // confirms arrival from odometry and stops this worker. The standalone
    // entry retains its existing stop-publishing behavior.
    if (!hold_at_trajectory_end_)
      return;
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = mission_fixed_yaw_ ? mission_yaw_ : last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    ROS_WARN_THROTTLE(1.0, "[Traj server] clock precedes active trajectory; retaining upstream hold");
    return;
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = world_frame_;
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd);
  if (mission_goal_binding_)
  {
    airseafly_real400_msgs::EgoGoalCommand message;
    message.goal = trajectory_goal_;
    message.command = cmd;
    goal_cmd_pub.publish(message);
    if (trajectory_final_ && t_cur >= traj_duration_ && !completion_reported_)
    {
      goal_finished_pub.publish(trajectory_goal_);
      completion_reported_ = true;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  nh.param("traj_server/mission_goal_binding", mission_goal_binding_, false);
  ros::Subscriber bspline_sub;
  if (mission_goal_binding_)
    bspline_sub = nh.subscribe("/airseafly/mission/goal_trajectory", 10, goalTrajectoryCallback);
  else
    bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback);

  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  if (mission_goal_binding_)
  {
    goal_cmd_pub = nh.advertise<airseafly_real400_msgs::EgoGoalCommand>("/airseafly/mission/goal_position_cmd", 50);
    goal_finished_pub = nh.advertise<geometry_msgs::PoseStamped>("/airseafly/front/goal_finished", 1, true);
  }

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("world_frame", world_frame_, std::string("world"));
  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/mission_fixed_yaw", mission_fixed_yaw_, false);
  nh.param("traj_server/mission_yaw", mission_yaw_, 0.0);
  nh.param("traj_server/hold_at_trajectory_end", hold_at_trajectory_end_, false);
  nh.param("traj_server/initial_yaw", last_yaw_, 0.0);
  if (mission_fixed_yaw_) last_yaw_ = mission_yaw_;
  last_yaw_dot_ = 0.0;

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}
