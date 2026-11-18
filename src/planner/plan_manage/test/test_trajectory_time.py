#!/usr/bin/env python3
"""Compile the actual trajectory callbacks with deterministic clocks/transport.

The spline stand-in isolates switching, frame and timestamp behavior; these tests
are not a ROS build, trajectory optimization test, or flight simulation.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[1] / 'src/traj_server.cpp'


def function(source, signature):
    begin = source.index(signature)
    opening = source.index('{', begin)
    depth = 1
    index = opening + 1
    while depth:
        depth += (source[index] == '{') - (source[index] == '}')
        index += 1
    return source[begin:index]


@unittest.skipUnless(shutil.which('c++'), 'C++ compiler unavailable')
class TrajectoryTimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        text = SOURCE.read_text()
        callbacks = '\n'.join(function(text, signature) for signature in
                              ('void bsplineCallback(', 'void goalTrajectoryCallback(', 'void activatePendingTrajectory(', 'std::pair<double, double> calculate_yaw(', 'void cmdCallback('))
        code = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include "plan_manage/goal_identity.h"
using namespace std;
#define ROS_INFO_STREAM(x) do { std::ostringstream stream; stream << x; } while(0)
#define ROS_WARN_THROTTLE(...) do {} while(0)
namespace ros {
double clock_now = 10.;
struct Duration { double s; double toSec() const { return s; } };
struct Time {
 double s=0.; static Time now() { return {clock_now}; }
 double toSec() const {return s;}
 uint64_t toNSec() const {return static_cast<uint64_t>(llround(s*1e9));}
};
Duration operator-(Time a, Time b) { return {a.s-b.s}; }
bool operator<(Time a,Time b) { return a.s<b.s; }
struct TimerEvent {};
ostream& operator<<(ostream& s, Time t) {return s << t.s;}
}
namespace boost {using std::make_shared;}
namespace Eigen {
struct Vector3d {
 double d[3]={0.,0.,0.};
 static Vector3d Zero() { return {}; }
 void setZero() { d[0]=d[1]=d[2]=0.; }
 double operator()(int i) const { return d[i]; }
 Vector3d operator-(const Vector3d& v) const {return {{d[0]-v.d[0],d[1]-v.d[1],d[2]-v.d[2]}};}
 double norm() const {return std::sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);}
 std::string transpose() const {return "diagnostic-vector";}
};
struct VectorXd {vector<double> values; explicit VectorXd(size_t n):values(n){} double& operator()(size_t i){return values[i];}};
struct MatrixXd {
 size_t n; vector<double> values;
 MatrixXd(size_t rows,size_t cols):n(cols),values(rows*cols){}
 double& operator()(size_t row,size_t col){return values[row*n+col];}
 double operator()(size_t row,size_t col) const{return values[row*n+col];}
};
}
namespace quadrotor_msgs {
struct PositionCommand {
 enum { TRAJECTORY_STATUS_READY=1 };
 struct Header { ros::Time stamp; string frame_id; } header;
 struct Vector { double x=0.,y=0.,z=0.; } position,velocity,acceleration;
 int trajectory_flag=0;
 int64_t trajectory_id=0;
 double yaw=0.,yaw_dot=0.;
};
}
namespace traj_utils {
struct Bspline {
 struct Point {double x,y,z;};
 vector<Point> pos_pts; vector<double> knots;
 int order=3; ros::Time start_time; int64_t traj_id=0;
};
using BsplineConstPtr=shared_ptr<const Bspline>;
}
namespace geometry_msgs {
struct PoseStamped {
 struct {ros::Time stamp; string frame_id; int seq=0;} header;
 struct {
  struct {double x=0.,y=0.,z=0.;} position;
  struct {double x=0.,y=0.,z=0.,w=1.;} orientation;
 } pose;
};
}
namespace airseafly_real400_msgs {
struct EgoGoalCommand {geometry_msgs::PoseStamped goal; quadrotor_msgs::PositionCommand command;};
struct EgoGoalTrajectory {geometry_msgs::PoseStamped goal; traj_utils::Bspline trajectory; bool final_segment=false;};
using EgoGoalTrajectoryConstPtr=shared_ptr<const EgoGoalTrajectory>;
}
struct UniformBspline {
 int derivative=0;
 double offset=0.;
 UniformBspline()=default;
 UniformBspline(Eigen::MatrixXd p,int,double):offset(p(0,0)){}
 void setKnot(const Eigen::VectorXd&){}
 UniformBspline getDerivative() const {auto copy=*this;++copy.derivative;return copy;}
 double getTimeSum() const{return 1.;}
 Eigen::Vector3d evaluateDeBoorT(double t) const {
  if(derivative==0) return {{offset+t,2.,.7}};
  if(derivative==1) return {{.3,.2,.1}};
  return {{.1,.2,.3}};
 }
};
struct Publisher {
 vector<quadrotor_msgs::PositionCommand> messages;
 void publish(const quadrotor_msgs::PositionCommand &c) {messages.push_back(c);}
} pos_cmd_pub;
template<class T> struct TypedPublisher {vector<T> messages; void publish(const T& m) {messages.push_back(m);}};
TypedPublisher<airseafly_real400_msgs::EgoGoalCommand> goal_cmd_pub;
TypedPublisher<geometry_msgs::PoseStamped> goal_finished_pub;
bool mission_goal_binding_=false, trajectory_final_=false, pending_final_=false, completion_reported_=false;
geometry_msgs::PoseStamped trajectory_goal_, pending_goal_;
quadrotor_msgs::PositionCommand cmd;
bool receive_traj_=false, mission_fixed_yaw_=true, hold_at_trajectory_end_=true;
double traj_duration_=1., mission_yaw_=.73, last_yaw_=.1, last_yaw_dot_=0., time_forward_=.3;
ros::Time start_time_;
int64_t traj_id_=0;
string world_frame_="world";
vector<UniformBspline> traj_,pending_traj_;
bool have_pending_traj_=false;
ros::Time pending_start_time_;
int64_t pending_traj_id_=0;
double pending_duration_=0;

''' + callbacks + r'''
void queue_curve(double stamp,int64_t id,double offset) {
 auto msg=make_shared<traj_utils::Bspline>();
 msg->start_time={stamp};msg->traj_id=id;
 msg->pos_pts={{offset,2.,.7},{offset+1.,2.,.7}};
 msg->knots={0.,1.};bsplineCallback(msg);
}
void tick(double stamp) {ros::clock_now=stamp;cmdCallback({});}
shared_ptr<airseafly_real400_msgs::EgoGoalTrajectory> goal_curve(
    double goal_stamp,double yaw,double start,int64_t id) {
 auto message=make_shared<airseafly_real400_msgs::EgoGoalTrajectory>();
 message->goal.header.stamp={goal_stamp};message->goal.header.frame_id="world";
 message->goal.pose.position.x=5.;message->goal.pose.position.y=2.;message->goal.pose.position.z=.7;
 message->goal.pose.orientation.z=sin(yaw/2.);message->goal.pose.orientation.w=cos(yaw/2.);
 message->trajectory.start_time={start};message->trajectory.traj_id=id;
 message->trajectory.pos_pts={{4.,2.,.7},{5.,2.,.7}};message->trajectory.knots={0.,1.};
 return message;
}
int main(int argc,char** argv) {
 string check=argv[1];
 if(check=="first_future") {
  queue_curve(11.,4,4.);tick(10.);tick(10.99);
  assert(pos_cmd_pub.messages.empty() && !receive_traj_);
  tick(11.);assert(pos_cmd_pub.messages.size()==1 && cmd.position.x==4.);
  assert(cmd.trajectory_id==4 && start_time_.s==11. && cmd.header.stamp.s==11.);
 } else if(check=="active_until_due") {
  queue_curve(10.,4,4.);tick(10.);queue_curve(10.5,5,8.);
  tick(10.2);assert(cmd.trajectory_id==4 && fabs(cmd.position.x-4.2)<1e-9);
  tick(10.49);assert(cmd.trajectory_id==4);
  tick(10.51);assert(cmd.trajectory_id==5 && fabs(cmd.position.x-8.01)<1e-9);
  assert(start_time_.s==10.5); // Reception and timer latency never restart curve time.
 } else if(check=="end_and_future") {
  queue_curve(10.,4,4.);tick(10.);queue_curve(12.,5,8.);
  for(double stamp:{11.,11.5,11.99}) {
   tick(stamp);assert(cmd.trajectory_id==4 && cmd.position.x==5.);
   assert(cmd.velocity.x==0. && cmd.acceleration.x==0. && cmd.yaw_dot==0.);
  }
  tick(12.);assert(cmd.trajectory_id==5 && cmd.position.x==8.);
 } else if(check=="replacement_and_clock") {
  queue_curve(10.,4,4.);tick(10.);queue_curve(12.,5,8.);
  queue_curve(10.1,6,7.);tick(10.1);assert(cmd.trajectory_id==6);
  tick(12.5);assert(cmd.trajectory_id==6 && cmd.position.x==8.);
  auto size=pos_cmd_pub.messages.size();tick(9.);
  assert(pos_cmd_pub.messages.size()==size); // Backward clock cannot command zero.
 } else if(check=="frame_pva_yaw") {
  world_frame_="fast_lio_local_enu";
  queue_curve(9.8,2147483649LL,4.);tick(10.);
  assert(fabs(cmd.position.x-4.2)<1e-9 && cmd.position.y==2. && cmd.position.z==.7);
  assert(cmd.velocity.x==.3 && cmd.velocity.y==.2 && cmd.velocity.z==.1);
  assert(cmd.acceleration.x==.1 && cmd.acceleration.y==.2 && cmd.acceleration.z==.3);
  assert(cmd.header.frame_id==world_frame_ && cmd.header.stamp.s==10. && cmd.trajectory_id==2147483649LL);
  assert(cmd.yaw==.73 && cmd.yaw_dot==0.);
 } else if(check=="first_yaw") {
  mission_fixed_yaw_=false;last_yaw_=0.;queue_curve(10.,4,4.);tick(10.);
  assert(std::isfinite(cmd.yaw) && std::isfinite(cmd.yaw_dot));
  assert(cmd.yaw==0. && cmd.yaw_dot==0.);
 } else if(check=="yaw_rate_limit") {
  // Exercise the production yaw calculation in both directions and across
  // +/-180 degrees; both angle steps and rate feedforward must stay bounded.
  const double pi=acos(-1.), limit=15.*pi/180.;
  traj_={UniformBspline()};
  for(const auto& headings:vector<pair<double,double>>{{0.,120.},{0.,-120.},
       {179.,-179.},{-179.,179.},{90.,-90.},{-90.,90.}}) {
   last_yaw_=headings.first*pi/180.;last_yaw_dot_=0.;
   const double target=headings.second*pi/180.;
   auto pos=traj_[0].evaluateDeBoorT(time_forward_);
   pos.d[0]-=cos(target);pos.d[1]-=sin(target);
   ros::Time previous{10.};
   for(int sample=0;sample<240;++sample) {
    const double dt=vector<double>{.01,.04,.15,.002}[sample%4];
    ros::Time now{previous.s+dt};
    const double before=last_yaw_;
    const auto result=calculate_yaw(0.,pos,now,previous);
    assert(isfinite(result.first) && isfinite(result.second));
    assert(fabs(result.second)<=limit+1e-9);
    assert(fabs(remainder(result.first-before,2.*pi))<=limit*dt+2e-7);
    previous=now;
   }
   assert(fabs(remainder(last_yaw_-target,2.*pi))<fabs(remainder(headings.first*pi/180.-target,2.*pi)));
  }
 } else if(check=="legacy_endpoint") {
  hold_at_trajectory_end_=false;queue_curve(10.,4,4.);tick(10.5);
  assert(cmd.header.frame_id=="world");
  auto size=pos_cmd_pub.messages.size();tick(11.);
  assert(pos_cmd_pub.messages.size()==size);
 } else if(check=="goal_binding") {
  mission_goal_binding_=true;
  auto message=make_shared<airseafly_real400_msgs::EgoGoalTrajectory>();
  message->goal.header.stamp={100.}; message->goal.header.frame_id="world";
  message->trajectory.start_time={10.}; message->trajectory.traj_id=4;
  message->trajectory.pos_pts={{4.,2.,.7},{5.,2.,.7}}; message->trajectory.knots={0.,1.};
  message->final_segment=true;
  goalTrajectoryCallback(message); tick(10.1);
  assert(goal_cmd_pub.messages.back().goal.header.stamp.s==100.);
  assert(goal_finished_pub.messages.empty());
  tick(11.); tick(11.2);
  assert(goal_finished_pub.messages.size()==1 && goal_finished_pub.messages.back().header.stamp.s==100.);
  auto old=make_shared<airseafly_real400_msgs::EgoGoalTrajectory>(*message);
  message->goal.header.stamp={101.}; message->goal.header.seq=999;
  message->trajectory.start_time={11.2}; message->trajectory.traj_id=5;
  goalTrajectoryCallback(message); tick(11.3); goalTrajectoryCallback(old); tick(11.4);
  assert(goal_cmd_pub.messages.back().goal.header.stamp.s==101. && cmd.trajectory_id==5);
  assert(goal_finished_pub.messages.size()==1);
  tick(12.3); assert(goal_finished_pub.messages.size()==2 && goal_finished_pub.messages.back().header.stamp.s==101.);
  // A planner restart resets only its local trajectory counter. The fresh
  // start time must still replace the old command without another waypoint.
  message->trajectory.start_time={13.}; message->trajectory.traj_id=1;
  goalTrajectoryCallback(message); tick(13.1);
  assert(cmd.trajectory_id==1 && goal_cmd_pub.messages.back().goal.header.stamp.s==101.);
  goalTrajectoryCallback(old); tick(13.2); assert(cmd.trajectory_id==1);
 } else if(check=="new_goal_yaw") {
  mission_goal_binding_=true;mission_fixed_yaw_=false;
  last_yaw_=2.;last_yaw_dot_=.2;
  auto first=goal_curve(100.,.6,10.,4);
  goalTrajectoryCallback(first);activatePendingTrajectory({10.});
  assert(fabs(last_yaw_-.6)<1e-12 && last_yaw_dot_==0.);
  tick(10.); // Start the executor's production yaw timer.
  // Native flight can hold a different heading while the old spline keeps
  // running in this process. A queued goal must rebase only when it activates.
  last_yaw_=1.4;last_yaw_dot_=.2;
  auto next=goal_curve(101.,-.8,10.2,5);
  goalTrajectoryCallback(next);activatePendingTrajectory({10.19});
  assert(last_yaw_==1.4 && last_yaw_dot_==.2);
  activatePendingTrajectory({10.2});
  assert(fabs(last_yaw_+.8)<1e-12 && last_yaw_dot_==0.);
  tick(10.2);
  assert(fabs(cmd.yaw+.8)<=15.*acos(-1.)/180.*.2+1e-9);
  assert(cmd.yaw<0. && isfinite(cmd.yaw_dot));
 } else if(check=="same_goal_yaw") {
  mission_goal_binding_=true;mission_fixed_yaw_=false;
  auto first=goal_curve(100.,.6,10.,4);
  goalTrajectoryCallback(first);activatePendingTrajectory({10.});
  last_yaw_=-1.2;last_yaw_dot_=-.1;
  auto replan=goal_curve(100.,.6,10.2,5);
  replan->goal.header.seq=999; // ROS sequence and orientation are not identity.
  replan->goal.pose.orientation.z=sin(1.1);replan->goal.pose.orientation.w=cos(1.1);
  goalTrajectoryCallback(replan);activatePendingTrajectory({10.2});
  assert(last_yaw_==-1.2 && last_yaw_dot_==-.1);
  // Position participates in the existing identity even if stamp is equal.
  replan->goal.pose.position.x+=1.;replan->trajectory.start_time={10.4};
  replan->trajectory.traj_id=6;
  goalTrajectoryCallback(replan);activatePendingTrajectory({10.4});
  assert(fabs(last_yaw_-2.2)<1e-12 && last_yaw_dot_==0.);
 } else if(check=="new_goal_yaw_wrap") {
  mission_goal_binding_=true;mission_fixed_yaw_=false;
  const double pi=acos(-1.),limit=15.*pi/180.;
  int64_t id=4;
  for(double heading:{179.,-179.}) {
   last_yaw_=0.;last_yaw_dot_=.2;
   const double measured=heading*pi/180.,target=-measured;
   auto message=goal_curve(100.+id,measured,10.+id,id);
   // q and -q represent the same physical heading.
   message->goal.pose.orientation.z*=-1.;message->goal.pose.orientation.w*=-1.;
   goalTrajectoryCallback(message);activatePendingTrajectory({10.+id});
   assert(fabs(remainder(last_yaw_-measured,2.*pi))<1e-12 && last_yaw_dot_==0.);
   auto pos=traj_[0].evaluateDeBoorT(time_forward_);
   pos.d[0]-=cos(target);pos.d[1]-=sin(target);
   ros::Time previous{10.+id},now{10.02+id};
   const auto result=calculate_yaw(0.,pos,now,previous);
   const double step=remainder(result.first-measured,2.*pi);
   assert(isfinite(result.first) && isfinite(result.second));
   assert(fabs(step)<=limit*.02+2e-7 && fabs(result.second)<=limit+1e-9);
   assert(step*heading>0.); // Take the short turn through +/-pi.
   ++id;
  }
 } else if(check=="yaw_mode_compatibility") {
  mission_goal_binding_=true;mission_fixed_yaw_=true;
  last_yaw_=.3;last_yaw_dot_=.1;
  auto message=goal_curve(100.,-2.,10.,4);
  goalTrajectoryCallback(message);activatePendingTrajectory({10.});
  assert(last_yaw_==.3 && last_yaw_dot_==.1);
  tick(10.);assert(cmd.yaw==mission_yaw_ && cmd.yaw_dot==0.);
  mission_goal_binding_=false;mission_fixed_yaw_=false;
  last_yaw_=.4;last_yaw_dot_=.2;
  queue_curve(10.2,5,4.);activatePendingTrajectory({10.2});
  assert(last_yaw_==.4 && last_yaw_dot_==.2);
 } else return 2;
}
'''
        cls.temp = tempfile.TemporaryDirectory(prefix='ego-reference-time-')
        source, cls.binary = Path(cls.temp.name)/'test.cpp', Path(cls.temp.name)/'test'
        source.write_text(code)
        subprocess.run(['c++','-std=c++14','-Wall','-Wextra','-Werror',
                        '-Wno-unused-parameter','-I',str(SOURCE.parents[1]/'include'),
                        str(source),'-o',str(cls.binary)],check=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def run_case(self, case):
        subprocess.run([str(self.binary),case],check=True,capture_output=True)

    def test_future_first_trajectory_publishes_nothing_until_due(self): self.run_case('first_future')
    def test_active_curve_keeps_running_until_future_curve_starts(self): self.run_case('active_until_due')
    def test_old_endpoint_holds_while_new_curve_waits(self): self.run_case('end_and_future')
    def test_immediate_replacement_cancels_pending_and_clock_rewind_has_no_zero(self): self.run_case('replacement_and_clock')
    def test_frame_pva_timestamp_id_and_initial_yaw_pass_through(self): self.run_case('frame_pva_yaw')
    def test_first_yaw_sample_has_finite_rate_at_zero_elapsed_time(self): self.run_case('first_yaw')
    def test_yaw_angle_and_rate_are_limited_to_fifteen_degrees_per_second(self): self.run_case('yaw_rate_limit')
    def test_legacy_endpoint_behavior_is_preserved(self): self.run_case('legacy_endpoint')
    def test_immutable_goal_binding_and_executor_completion_ignore_old_spline(self): self.run_case('goal_binding')
    def test_new_goal_rebases_preview_yaw_when_its_first_spline_activates(self): self.run_case('new_goal_yaw')
    def test_same_goal_replanning_preserves_preview_yaw_and_rate(self): self.run_case('same_goal_yaw')
    def test_new_goal_yaw_handles_quaternion_sign_and_pi_wrap(self): self.run_case('new_goal_yaw_wrap')
    def test_fixed_yaw_and_unbound_trajectory_modes_do_not_rebase(self): self.run_case('yaw_mode_compatibility')


if __name__ == '__main__': unittest.main(verbosity=2)
