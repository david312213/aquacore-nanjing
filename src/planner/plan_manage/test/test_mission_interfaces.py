#!/usr/bin/env python3
"""Compile actual task callback bodies against test doubles; no ROS or hardware."""
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[4]
CTRL = ROOT / 'src/realflight_modules/px4ctrl/src'
PLANNER = ROOT / 'src/planner/plan_manage'


def extract(source, signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == '{':
            depth += 1
        elif source[index] == '}':
            depth -= 1
            if depth == 0:
                return source[start:index + 1]
    raise AssertionError('unbalanced function')


def compile_run(code):
    with tempfile.TemporaryDirectory(prefix='real400-task-callback-') as temp:
        source = Path(temp) / 'test.cpp'
        binary = Path(temp) / 'test'
        source.write_text(code)
        subprocess.run(['c++', '-std=c++14', '-Wall', '-Wextra', '-Werror',
                        '-I', str(CTRL), '-I', str(PLANNER/'include'), str(source), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


class MissionInterfaces(unittest.TestCase):
    def test_actual_takeoff_service_and_legacy_override(self):
        source = (CTRL / 'PX4CtrlFSM.cpp').read_text()
        functions = '\n'.join(extract(source, signature) for signature in (
            'bool PX4CtrlFSM::mission_takeoff(', 'void PX4CtrlFSM::feed_takeoff_land(',
            'const char *PX4CtrlFSM::control_state_name('))
        compile_run(r'''
#include <cassert>
#include <string>
#include <memory>
#include <limits>
#include "takeoff_z.h"
namespace ros { struct Time { static Time now() { return {}; } }; }
namespace airseafly_real400_msgs { struct MissionTakeoff {
 struct Request { double target_world_z; };
 struct Response { bool accepted=false; std::string message; };
}; }
namespace quadrotor_msgs {
struct TakeoffLand { enum { TAKEOFF=1, LAND=2 }; int takeoff_land_cmd=TAKEOFF; };
using TakeoffLandConstPtr=std::shared_ptr<const TakeoffLand>;
}
struct Vec { double value=0.; double norm() const { return value; } double operator()(int) const { return value; } };
class PX4CtrlFSM {
public:
 enum State { MANUAL_CTRL=1, AUTO_HOVER, CMD_CTRL, AUTO_TAKEOFF, AUTO_LAND, PX4_NATIVE };
 State state=MANUAL_CTRL;
 struct Parameters { struct Takeoff { bool enable=true; double height=.7, speed=.3; } takeoff_land; } param;
 struct Odom { Vec p, v; } odom_data;
 struct Rc { bool is_hover_mode=true, is_command_mode=true, centered=true; bool check_centered() { return centered; } } rc_data;
 struct Trigger { int takeoff_land_cmd=0; bool triggered=false; void feed(quadrotor_msgs::TakeoffLandConstPtr m) { takeoff_land_cmd=m->takeoff_land_cmd; triggered=true; } } takeoff_land_data;
 bool mission_takeoff_pending_=false, odom=true, command=false, landed=true, rc=true;
 double mission_takeoff_target_z_=0.;
 bool odom_is_received(ros::Time) { return odom; }
 bool cmd_is_received(ros::Time) { return command; }
 bool rc_is_received(ros::Time) { return rc; }
 bool get_landed() { return landed; }
 bool mission_takeoff(airseafly_real400_msgs::MissionTakeoff::Request &, airseafly_real400_msgs::MissionTakeoff::Response &);
 void feed_takeoff_land(quadrotor_msgs::TakeoffLandConstPtr);
 const char *control_state_name() const;
};
''' + functions + r'''
int main() {
 using Service=airseafly_real400_msgs::MissionTakeoff;
 PX4CtrlFSM f; f.odom_data.p.value=2.; Service::Request req{2.8}; Service::Response res;
 assert(f.mission_takeoff(req,res)); assert(res.accepted); assert(f.mission_takeoff_pending_);
 assert(f.mission_takeoff_target_z_==2.8 && f.param.takeoff_land.height==.7);
 f.feed_takeoff_land(std::make_shared<quadrotor_msgs::TakeoffLand>());
 assert(!f.mission_takeoff_pending_ && f.param.takeoff_land.height==.7);
 for (int failure=0;failure<9;++failure) {
  PX4CtrlFSM x; x.odom_data.p.value=2.; auto r=req;
  if(failure==0) x.param.takeoff_land.enable=false;
  if(failure==1) x.state=PX4CtrlFSM::AUTO_HOVER;
  if(failure==2) x.odom=false;
  if(failure==3) x.command=true;
  if(failure==4) x.odom_data.v.value=.11;
  if(failure==5) x.landed=false;
  if(failure==6) r.target_world_z=1.9;
  if(failure==7) x.rc_data.centered=false;
  if(failure==8) r.target_world_z=std::numeric_limits<double>::quiet_NaN();
  assert(x.mission_takeoff(r,res) && !res.accepted && !res.message.empty());
  assert(!x.mission_takeoff_pending_ && !x.takeoff_land_data.triggered);
 }
 f.rc=false; f.rc_data.centered=false;
 assert(f.mission_takeoff(req,res) && res.accepted); // preserves no-RC semantics
 f.state=PX4CtrlFSM::AUTO_LAND; assert(std::string(f.control_state_name())=="AUTO_LAND");
 auto ref=airseafly_takeoff::world_z_reference(2.,2.8,.3,9.);
 assert(ref.position==2.8 && ref.velocity==0.);
}
''')
        self.assertIn('mission_takeoff_target_z_ : param.takeoff_land.height', source)
        self.assertIn('active_takeoff_target_z_ = target_z;', source)
        self.assertIn('hover_pose(2) = active_takeoff_target_z_;', source)
        self.assertNotIn('param.takeoff_land.height =', source)

    def test_actual_front_goal_callbacks_wait_for_map_and_start_fresh_worker(self):
        source = (PLANNER / 'src/ego_replan_fsm.cpp').read_text()
        functions = '\n'.join(extract(source, '  void EGOReplanFSM::' + name + '(')
                             for name in ('frontGoalCallback', 'frontGoalStartCallback', 'receiveFrontGoal',
                                          'setPlanningStart', 'processFrontGoal', 'executorConnected',
                                          'publishGoalTrajectory', 'updatePlannerReady'))
        compile_run(r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <memory>
#include <set>
#include <tuple>
#include <string>
#define ROS_WARN(...) do {} while(0)
#define ROS_INFO(...) do {} while(0)
#define ROS_INFO_STREAM(...) do {} while(0)
#include <plan_manage/goal_identity.h>
using ego_planner::goalIdentity;
namespace ros { struct SingleSubscriberPublisher {}; }
namespace Eigen { struct Vector3d { double x,y,z; Vector3d(double a=0,double b=0,double c=0):x(a),y(b),z(c){} void setZero(){x=y=z=0.;} }; }
namespace std_msgs { struct Bool { bool data=false; }; }
namespace geometry_msgs {
struct PoseStamped {
 struct Header { std::string frame_id="world"; struct Stamp { std::uint64_t value=0; std::uint64_t toNSec() const { return value; } } stamp; } header;
 struct Pose { struct Point { double x=1.,y=2.,z=2.8; } position; } pose;
}; using PoseStampedPtr=std::shared_ptr<PoseStamped>;
}
namespace quadrotor_msgs { struct PositionCommand {
 geometry_msgs::PoseStamped::Header header;
 Eigen::Vector3d position, velocity, acceleration;
}; }
namespace traj_utils { struct Bspline {}; }
namespace airseafly_real400_msgs { struct EgoGoalCommand {
 geometry_msgs::PoseStamped goal;
 quadrotor_msgs::PositionCommand command;
 bool apply_map_inflation=false; double map_inflation_m=0.;
}; using EgoGoalCommandConstPtr=std::shared_ptr<const EgoGoalCommand>;
struct EgoGoalTrajectory { geometry_msgs::PoseStamped goal; traj_utils::Bspline trajectory; bool final_segment=false; }; }
struct Publisher { int calls=0; template<class T> void publish(const T &) { ++calls; } int getNumSubscribers() const { return 1; } };
struct Map { bool ready=false; double inflation=.25; int changes=0; void setObstaclesInflation(double v){inflation=v;++changes;} bool initialCloudMapReady() const { return ready; } };
struct Manager { std::shared_ptr<Map> grid_map_{new Map}; };
class EGOReplanFSM {
public:
 enum State { INIT, WAIT_TARGET, EXEC_TRAJ, EMERGENCY_STOP };
 State exec_state_=INIT;
 std::string world_frame_="world";
 bool have_active_front_goal_=false;
 bool have_front_start_=false, use_front_start_=false;
 bool executor_connection_seen_=false, have_trajectory_front_goal_=false;
 quadrotor_msgs::PositionCommand front_start_reference_;
 bool have_latest_front_goal_=false, front_goal_failure_reported_=false;
 bool have_pending_front_goal_=false,have_odom_=false,require_initial_map_=true,ego_ready_=false,have_trigger_=false;
 geometry_msgs::PoseStamped pending_front_goal_request_,active_front_goal_request_,latest_front_goal_request_;
 geometry_msgs::PoseStamped trajectory_front_goal_request_;
 std::set<std::tuple<std::uint64_t,std::string,double,double,double>> received_front_goals_;
 Publisher front_goal_received_pub_,ego_ready_pub_,goal_trajectory_pub_;
 std::shared_ptr<Manager> planner_manager_{new Manager};
 Eigen::Vector3d odom_pos_,odom_vel_,init_pt_,goal,start_pt_,start_vel_,start_acc_;
 int planned=0;
 Eigen::Vector3d planned_start;
 double now=0., planned_time=0.;
 bool planNextWaypoint(Eigen::Vector3d p, bool replace) { assert(replace); setPlanningStart(); goal=p; planned_start=start_pt_; planned_time=now; ++planned; return true; }
 void reportFrontGoalFailure() {}
 void executorConnected(const ros::SingleSubscriberPublisher &);
 void frontGoalCallback(const geometry_msgs::PoseStampedPtr &);
 void frontGoalStartCallback(const airseafly_real400_msgs::EgoGoalCommandConstPtr &);
 void receiveFrontGoal(const geometry_msgs::PoseStamped &, const quadrotor_msgs::PositionCommand *start=nullptr, const double *inflation=nullptr);
 void setPlanningStart();
 void publishGoalTrajectory(const traj_utils::Bspline &, bool);
 void processFrontGoal(); void updatePlannerReady();
};
''' + functions + r'''
int main() {
 EGOReplanFSM f; auto m=std::make_shared<geometry_msgs::PoseStamped>();
 f.frontGoalCallback(m); assert(f.have_pending_front_goal_ && f.front_goal_received_pub_.calls==1);
 f.processFrontGoal(); assert(f.planned==0);
 f.have_odom_=true;f.exec_state_=EGOReplanFSM::WAIT_TARGET;
 f.updatePlannerReady();f.processFrontGoal();assert(!f.ego_ready_ && f.planned==0);
 f.planner_manager_->grid_map_->ready=true;f.updatePlannerReady();
 assert(f.ego_ready_ && f.ego_ready_pub_.calls==1);f.processFrontGoal();
 assert(f.planned==1 && f.goal.z==2.8 && f.have_trigger_);
 f.frontGoalCallback(m);f.processFrontGoal();assert(f.planned==1 && f.front_goal_received_pub_.calls==2);
 m->header.stamp.value=2;m->pose.position.x=3;f.frontGoalCallback(m);
 m->header.stamp.value=3;m->pose.position.x=4;f.frontGoalCallback(m);
 f.processFrontGoal();assert(f.planned==2 && f.goal.x==4);
 m->header.frame_id="camera";f.frontGoalCallback(m);assert(!f.have_pending_front_goal_);
 EGOReplanFSM legacy;legacy.require_initial_map_=false;legacy.have_odom_=true;legacy.exec_state_=EGOReplanFSM::WAIT_TARGET;
 m->header.frame_id="world";legacy.frontGoalCallback(m);legacy.processFrontGoal();assert(legacy.planned==1);
 EGOReplanFSM native;native.world_frame_="fast_lio_local_enu";
 native.frontGoalCallback(m);assert(!native.have_pending_front_goal_);
 m->header.frame_id="fast_lio_local_enu";native.frontGoalCallback(m);
 assert(native.have_pending_front_goal_ && native.front_goal_received_pub_.calls==1);
 auto bound=std::make_shared<airseafly_real400_msgs::EgoGoalCommand>();
 bound->goal=*m; bound->goal.header.stamp.value=10;
 bound->command.header.frame_id="fast_lio_local_enu";
 bound->command.position={.079,.496,1.419};
 native.have_odom_=true; native.require_initial_map_=false;
 native.exec_state_=EGOReplanFSM::WAIT_TARGET;
 native.odom_pos_={-.011,.565,1.215}; native.odom_vel_={.133,-.042,.407};
 bound->apply_map_inflation=true; bound->map_inflation_m=.1;
 native.frontGoalStartCallback(bound); native.processFrontGoal();
 assert(native.planner_manager_->grid_map_->inflation==.1 && native.planner_manager_->grid_map_->changes==1);
 assert(native.planned_start.z==1.419 && native.start_vel_.z==0.);
 native.publishGoalTrajectory({},false); // TCP subscriber registered; callback still queued.
 assert(native.have_front_start_);
 native.executorConnected({}); native.processFrontGoal();
 assert(native.planned==2 && native.planned_start.z==1.419 && native.start_vel_.z==0.);
 native.publishGoalTrajectory({},false); // First connection handled; seed can now be consumed.
 assert(!native.have_front_start_);
 native.frontGoalStartCallback(bound); native.processFrontGoal();
 assert(!native.have_front_start_ && native.planned==2); // Retry cannot reset seed.
 assert(native.planner_manager_->grid_map_->changes==1); // Nor reapply inflation.
 native.executorConnected({}); native.processFrontGoal();
 assert(native.planned==3 && native.planned_start.z==1.215 && native.start_vel_.z==.407);
 auto next=std::make_shared<airseafly_real400_msgs::EgoGoalCommand>(*bound);
 next->goal.header.stamp.value=11; next->map_inflation_m=.35;
 native.frontGoalStartCallback(next);
 assert(native.planner_manager_->grid_map_->inflation==.35);
 native.frontGoalStartCallback(bound); // Older retry must leave latest radius intact.
 assert(native.planner_manager_->grid_map_->inflation==.35 && native.planner_manager_->grid_map_->changes==2);
 next->goal.header.stamp.value=12; next->map_inflation_m=-1;
 native.frontGoalStartCallback(next);
 assert(native.planner_manager_->grid_map_->changes==2);
 for(double delay : {0., .6, 2.}) {
  EGOReplanFSM connected; connected.require_initial_map_=false;
  connected.have_odom_=true; connected.exec_state_=EGOReplanFSM::WAIT_TARGET;
  m->header.frame_id="world"; connected.frontGoalCallback(m); connected.processFrontGoal();
  connected.now=delay; connected.odom_pos_={delay,0.,1.};
  connected.executorConnected({}); connected.processFrontGoal();
  assert(connected.planned==2 && connected.planned_time==delay);
  assert(connected.planned_start.x==delay);
  connected.now=delay+3.; connected.odom_pos_={delay+1.,0.,1.};
  connected.executorConnected({}); connected.processFrontGoal();
  assert(connected.planned==3 && connected.planned_start.x==delay+1.);
 }
}
''')
        waypoint = extract(source, '  void EGOReplanFSM::waypointCallback(')
        self.assertIn('Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, rviz_goal_height_);', waypoint)
        self.assertIn('nh.param("fsm/require_initial_map", require_initial_map_, false)', source)
        self.assertIn('nh.param("fsm/rviz_goal_height", rviz_goal_height_, 0.7)', source)

    def test_actual_replan_uses_measured_residual_without_losing_existing_retries(self):
        source = (PLANNER / 'src/ego_replan_fsm.cpp').read_text()
        body = extract(source, '  bool EGOReplanFSM::planFromCurrentTraj(')
        compile_run(r'''
#include <cassert>
#include <cmath>
#include <vector>
struct Vec {
 double x=0., y=0., z=0.;
 Vec operator-(const Vec &v) const {return {x-v.x,y-v.y,z-v.z};}
 double norm() const {return std::sqrt(x*x+y*y+z*z);}
 void setZero() {x=y=z=0.;}
};
namespace ros {
struct Duration {double value; double toSec() const {return value;}};
struct Time {double value; static Time now() {return {12.};}};
Duration operator-(Time a, Time b) {return {a.value-b.value};}
}
struct Curve {Vec value; Vec evaluateDeBoorT(double) const {return value;}};
struct LocalTrajData {ros::Time start_time_{10.}; Curve position_traj_,velocity_traj_,acceleration_traj_;};
struct Planner {LocalTrajData local_data_; double shortReplanDistance() const {return .2;}};
class EGOReplanFSM {
public:
 Planner planner;
 Planner *planner_manager_=&planner;
 Vec start_pt_,start_vel_,start_acc_,end_pt_{0.,0.,.8},odom_pos_{-.25,0.,.8},odom_vel_{.12,0.,0.};
 struct Attempt {bool polynomial,random; Vec start,velocity,acceleration;};
 std::vector<Attempt> attempts;
 unsigned succeed_after=1;
 bool callReboundReplan(bool polynomial,bool random) {
  attempts.push_back({polynomial,random,start_pt_,start_vel_,start_acc_});
  return attempts.size()>=succeed_after;
 }
 bool planFromCurrentTraj(int trial_times=1);
};
''' + body + r'''
int main() {
 for(double reference_residual : {0.,.18,.20,.22,.25}) {
  EGOReplanFSM f;
  f.planner.local_data_.position_traj_.value={-reference_residual,0.,.8};
  f.planner.local_data_.velocity_traj_.value={.01,0.,0.};
  f.planner.local_data_.acceleration_traj_.value={.02,0.,0.};
  assert(f.planFromCurrentTraj() && f.attempts.size()==1);
  auto attempt=f.attempts.front();
  if(reference_residual<=.2) {
   assert(attempt.polynomial && !attempt.random);
   assert(attempt.start.x==-.25 && attempt.velocity.x==.12 && attempt.acceleration.norm()==0.);
  } else {
   assert(!attempt.polynomial && !attempt.random);
   assert(attempt.start.x==-reference_residual && attempt.velocity.x==.01 && attempt.acceleration.x==.02);
  }
 }
 EGOReplanFSM retry;
 retry.planner.local_data_.position_traj_.value=retry.end_pt_;
 retry.succeed_after=3;
 assert(retry.planFromCurrentTraj(1) && retry.attempts.size()==3);
 assert(retry.attempts.back().polynomial && retry.attempts.back().random);
 for(auto attempt:retry.attempts) assert(attempt.start.x==-.25 && attempt.velocity.x==.12);
 EGOReplanFSM failed;
 failed.planner.local_data_.position_traj_.value=failed.end_pt_;
 failed.succeed_after=99;
 assert(!failed.planFromCurrentTraj(2) && failed.attempts.size()==4);
}
''')

    def test_worker_launch_and_map_readiness_contract(self):
        launch = ET.parse(PLANNER / 'launch/mission_worker.launch').getroot()
        arguments = {arg.get('name'):arg.get('value') for arg in launch.find('include').findall('arg')}
        self.assertNotIn('planar_mode', arguments)
        self.assertEqual(arguments['height_origin_z'], '$(arg height_origin_z)')
        self.assertEqual(arguments['require_initial_map'], 'true')
        self.assertEqual(arguments['mission_fixed_yaw'], 'true')
        single = ET.parse(PLANNER / 'launch/single_run_in_exp.launch').getroot()
        defaults = {arg.get('name'):arg.get('default') for arg in single.findall('arg')}
        self.assertEqual(defaults['position_cmd_topic'], '/position_cmd')
        self.assertNotIn('planar_mode', defaults)
        self.assertEqual(defaults['height_origin_z'], '0.0')
        self.assertEqual(defaults['mission_fixed_yaw'], 'false')
        grid = (ROOT / 'src/planner/plan_env/src/grid_map.cpp').read_text()
        cloud = extract(grid, 'void GridMap::cloudCallback(')
        self.assertEqual(grid.count('built_from_valid_point = true;'), 1)
        self.assertIn('if (!md_.has_odom_)', cloud)
        self.assertIn('if (latest_cloud.points.size() == 0)', cloud)
        self.assertIn('if (!p3d.allFinite()) continue;', cloud)
        self.assertLess(cloud.index('md_.occupancy_buffer_inflate_[idx_inf] = 1;'), cloud.index('built_from_valid_point = true;'))
        self.assertLess(cloud.index('boundIndex(md_.local_bound_max_);'), cloud.index('initial_cloud_map_ready_ ='))
        trajectory = (PLANNER / 'src/traj_server.cpp').read_text()
        self.assertIn('? std::make_pair(mission_yaw_, 0.0)', trajectory)
        self.assertIn('nh.param("traj_server/initial_yaw", last_yaw_, 0.0)', trajectory)
        self.assertNotIn('ros::Duration(1.0).sleep()', trajectory)
        worker = ET.parse(ROOT / 'src/airseafly_real400_mission/launch/ego_worker.launch').getroot()
        group = worker.find('group')
        params = {param.get('name'): param.get('value') for param in group.findall('param')}
        self.assertEqual(params['/drone_0_traj_server/traj_server/mission_goal_binding'], 'true')
        self.assertEqual(params['/drone_0_ego_planner_node/fsm/mission_goal_binding'], 'true')
        remaps = {remap.get('from'): remap.get('to') for remap in group.findall('remap')}
        for topic in ('goal_trajectory', 'goal_position_cmd'):
            self.assertEqual(remaps['/airseafly/mission/'+topic],
                             '/airseafly/mission/session_$(arg session)/'+topic)


if __name__ == '__main__':
    unittest.main(verbosity=2)
