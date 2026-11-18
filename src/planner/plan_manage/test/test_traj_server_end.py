#!/usr/bin/env python3
"""Compile the real trajectory timer callback with fake transport and splines.

No ROS master, aircraft, subprocess planner or flight controller is started.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_mission_interfaces import extract

SOURCE = Path(__file__).resolve().parents[1] / 'src/traj_server.cpp'


class TrajectoryEndTests(unittest.TestCase):
    def test_real_callback_endpoint_hold_and_replacement(self):
        callback = extract(SOURCE.read_text(), 'void cmdCallback(')
        code = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
using namespace std;
#define ROS_WARN_THROTTLE(...) do {} while(0)
namespace ros {
double clock_now = 10.;
struct Duration { double s; double toSec() const { return s; } };
struct Time { double s=0.; static Time now() { return {clock_now}; } };
Duration operator-(Time a,Time b) { return {a.s-b.s}; }
struct TimerEvent {};
}
namespace Eigen {
struct Vector3d {
 double d[3]={0.,0.,0.};
 static Vector3d Zero() { return {}; }
 void setZero() { d[0]=d[1]=d[2]=0.; }
 double operator()(int i) const { return d[i]; }
};
}
namespace quadrotor_msgs {
struct PositionCommand {
 enum { TRAJECTORY_STATUS_READY=1 };
 struct Header { ros::Time stamp; string frame_id; } header;
 struct Vector { double x=0.,y=0.,z=0.; } position,velocity,acceleration;
 int trajectory_flag=0,trajectory_id=0;
 double yaw=0.,yaw_dot=0.;
};
}
struct Spline {
 int derivative;
 double offset;
 Eigen::Vector3d evaluateDeBoorT(double t) const {
  if(derivative==0) return {{offset+t,2.,.7}};
  if(derivative==1) return {{.3,.2,.1}};
  return {{.1,.2,.3}};
 }
};
struct Publisher {
 vector<quadrotor_msgs::PositionCommand> messages;
 void publish(const quadrotor_msgs::PositionCommand &c) { messages.push_back(c); }
} pos_cmd_pub;
namespace geometry_msgs {struct PoseStamped {};}
namespace airseafly_real400_msgs {struct EgoGoalCommand {geometry_msgs::PoseStamped goal; quadrotor_msgs::PositionCommand command;};}
struct ExtraPublisher {template<class T> void publish(const T&) {}} goal_cmd_pub, goal_finished_pub;
bool mission_goal_binding_=false, trajectory_final_=false, completion_reported_=false;
geometry_msgs::PoseStamped trajectory_goal_;
quadrotor_msgs::PositionCommand cmd;
bool receive_traj_=false, mission_fixed_yaw_=true, hold_at_trajectory_end_=false;
double traj_duration_=1., mission_yaw_=.73, last_yaw_=.1;
ros::Time start_time_{10.};
int traj_id_=4;
std::string world_frame_="world";
void activatePendingTrajectory(const ros::Time&) {}
vector<Spline> traj_={{0,4.},{1,0.},{2,0.}};
pair<double,double> calculate_yaw(double,Eigen::Vector3d &,ros::Time &,ros::Time &) {
 return {.2,.1};
}
''' + callback + r'''
int main() {
 cmdCallback({}); assert(pos_cmd_pub.messages.empty());
 receive_traj_=true; ros::clock_now=10.5; cmdCallback({});
 assert(pos_cmd_pub.messages.size()==1 && cmd.position.x==4.5);
 assert(cmd.velocity.x==.3 && cmd.yaw==.73);
 ros::clock_now=11.; cmdCallback({});
 assert(pos_cmd_pub.messages.size()==1); // Legacy entry keeps its prior behavior.
 hold_at_trajectory_end_=true;
 for(double now : {11.,11.5,12.}) {
  ros::clock_now=now; const auto before=pos_cmd_pub.messages.size(); cmdCallback({});
  assert(pos_cmd_pub.messages.size()==before+1 && cmd.header.stamp.s==now);
  assert(cmd.position.x==5. && cmd.position.y==2. && cmd.position.z==.7);
  assert(cmd.velocity.x==0. && cmd.velocity.y==0. && cmd.velocity.z==0.);
  assert(cmd.acceleration.x==0. && cmd.acceleration.y==0. && cmd.acceleration.z==0.);
  assert(cmd.yaw==.73 && cmd.yaw_dot==0. && cmd.header.frame_id=="world");
  assert(cmd.trajectory_id==4);
 }
 // A new spline immediately replaces the held endpoint.
 start_time_={12.}; traj_[0].offset=7.; traj_id_=5;
 ros::clock_now=12.1; cmdCallback({});
 assert(fabs(cmd.position.x-7.1)<1e-9 && cmd.velocity.x==.3 && cmd.trajectory_id==5);
 mission_fixed_yaw_=false; last_yaw_=-.22;
 ros::clock_now=13.1; cmdCallback({});
 assert(cmd.position.x==8. && cmd.yaw==-.22 && cmd.yaw_dot==0.);
 hold_at_trajectory_end_=false; auto count=pos_cmd_pub.messages.size();
 ros::clock_now=13.5; cmdCallback({}); assert(pos_cmd_pub.messages.size()==count);
}
'''
        with tempfile.TemporaryDirectory(prefix='real400-traj-end-') as temp:
            source, binary = Path(temp) / 'test.cpp', Path(temp) / 'test'
            source.write_text(code)
            subprocess.run(['c++', '-std=c++14', '-Wall', '-Wextra', '-Werror',
                            '-Wno-unused-parameter', str(source), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)

    def test_hold_is_an_explicit_node_parameter(self):
        source = SOURCE.read_text()
        self.assertIn('bool hold_at_trajectory_end_ = false;', source)
        self.assertIn('nh.param("traj_server/hold_at_trajectory_end", hold_at_trajectory_end_, false);', source)


if __name__ == '__main__':
    unittest.main(verbosity=2)
