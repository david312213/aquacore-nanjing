"""Compile the real EGO execution/completion branch with offline ROS stubs.

This checks goal identity and timing. It is not a full ROS build or a flight model.
EGO_FSM_SOURCE can select an onboard-version candidate for the same checks.
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


@unittest.skipUnless(shutil.which('c++'), 'C++ compiler unavailable')
class GoalCompletionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = os.environ.get('EGO_FSM_SOURCE') or Path(__file__).resolve().parents[1]/'src/ego_replan_fsm.cpp'
        source = Path(path).read_text()
        begin = source.index('  void EGOReplanFSM::processFrontGoal()')
        failure_begin = source.index('  void EGOReplanFSM::reportFrontGoalFailure()', begin)
        process = source[begin:source.index('  void EGOReplanFSM::executorConnected', begin)]
        failure = source[failure_begin:source.index('  void EGOReplanFSM::odometryCallback', failure_begin)]
        begin = source.index('    case EXEC_TRAJ:')
        execution = source[begin:source.index('    case EMERGENCY_STOP:', begin)]
        code = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <string>
#include <vector>
using std::min;
namespace Eigen {
struct Vector3d {
 double x=0,y=0,z=0;
 Vector3d()=default;
 Vector3d(double a,double b,double c):x(a),y(b),z(c){}
 Vector3d operator-(Vector3d v) const {return {x-v.x,y-v.y,z-v.z};}
 double norm() const {return std::sqrt(x*x+y*y+z*z);}
};
}
namespace ros {
struct Time {
 double t=0;
 static Time now(){return {10.0};}
 Time operator-(Time other) const {return {t-other.t};}
 double toSec() const {return t;}
};
}
struct Request { int id=0; struct {Eigen::Vector3d position;} pose; };
bool sameGoal(const Request &a, const Request &b) {return a.id==b.id;}
int goalIdentity(const Request &a) {return a.id;}
struct Publisher {std::vector<Request> messages; void publish(Request r){messages.push_back(r);}};
struct Trajectory {Eigen::Vector3d evaluateDeBoorT(double) {return {1,0,0};}};
struct LocalTrajData {ros::Time start_time_; double duration_=10; Trajectory position_traj_;};
struct Planner {LocalTrajData local_data_;};
class EGOReplanFSM {
public:
 enum State {INIT, WAIT_TARGET, EXEC_TRAJ, GEN_NEW_TRAJ, REPLAN_TRAJ, EMERGENCY_STOP};
 enum TARGET_TYPE {MANUAL_TARGET, PRESET_TARGET};
 State exec_state_=EXEC_TRAJ;
 TARGET_TYPE target_type_=MANUAL_TARGET;
 bool have_pending_front_goal_=false, have_odom_=true, have_active_front_goal_=true;
 bool require_initial_map_=true, ego_ready_=true, have_target_=true, have_trigger_=true;
 bool front_goal_failure_reported_=false;
 bool have_front_start_=false, use_front_start_=false;
 bool have_trajectory_front_goal_=true, mission_goal_binding_=false;
 bool plan_success=true;
 int wp_id_=0, waypoint_num_=1;
 double no_replan_thresh_=.2, replan_thresh_=1;
 Eigen::Vector3d end_pt_{1,0,0}, local_target_pt_{1,0,0}, init_pt_, odom_pos_;
 std::vector<Eigen::Vector3d> wps_{{1,0,0}};
 Request pending_front_goal_request_, active_front_goal_request_, trajectory_front_goal_request_, latest_front_goal_request_;
 Publisher front_goal_finished_pub_, front_goal_failed_pub_;
 Planner planner; Planner* planner_manager_=&planner;
 bool planNextWaypoint(Eigen::Vector3d goal, bool=false) {if(plan_success) end_pt_=goal; return plan_success;}
 void changeFSMExecState(State state,const char*) {exec_state_=state;}
 void processFrontGoal();
 void reportFrontGoalFailure();
 void execute() {
  switch(exec_state_) {
   EXECUTION
   default: break;
  }
  force_return: return;
 }
};
PROCESS
FAILURE
int main(int argc,char** argv) {
 EGOReplanFSM f;
 f.active_front_goal_request_.id=17;
 f.trajectory_front_goal_request_.id=17;
 std::string check=argv[1];
 if(check=="completion") {
  f.execute();
  assert(f.front_goal_finished_pub_.messages.size()==1);
  assert(f.front_goal_finished_pub_.messages[0].id==17);
  assert(f.exec_state_==EGOReplanFSM::WAIT_TARGET);
  f.execute();
  assert(f.front_goal_finished_pub_.messages.size()==1);
 } else if(check=="timing") {
  f.planner.local_data_.start_time_.t=.005; // Five milliseconds before the end.
  f.execute();
  assert(f.front_goal_finished_pub_.messages.empty());
  f.planner.local_data_.start_time_.t=0;
  f.execute();
  assert(f.front_goal_finished_pub_.messages.size()==1);
 } else if(check=="intermediate") {
  f.local_target_pt_={.5,0,0};
  f.execute();
  assert(f.front_goal_finished_pub_.messages.empty());
  assert(f.have_active_front_goal_);
  f.exec_state_=EGOReplanFSM::EMERGENCY_STOP;
  f.execute();
  assert(f.front_goal_finished_pub_.messages.empty());
 } else if(check=="failed") {
  f.have_pending_front_goal_=true;
  f.plan_success=false;
  f.pending_front_goal_request_.id=18;
  f.processFrontGoal();
  assert(f.front_goal_failed_pub_.messages.size()==1);
  assert(f.front_goal_failed_pub_.messages[0].id==18);
  assert(!f.have_active_front_goal_);
  f.execute(); // An old trajectory reaching its end cannot complete request 18.
  assert(f.front_goal_finished_pub_.messages.empty());
 } else if(check=="identity") {
  f.have_pending_front_goal_=true;
  f.pending_front_goal_request_.id=18;
  f.pending_front_goal_request_.pose.position={1,0,0};
  f.processFrontGoal();
  assert(f.front_goal_finished_pub_.messages.empty());
  f.execute();
  assert(f.front_goal_finished_pub_.messages.empty()); // Only request 18 received; spline 17 is still old.
  f.trajectory_front_goal_request_=f.active_front_goal_request_;
  f.exec_state_=EGOReplanFSM::EXEC_TRAJ;
  f.execute();
  assert(f.front_goal_finished_pub_.messages.size()==1);
  assert(f.front_goal_finished_pub_.messages[0].id==18);
 } else if(check=="executor") {
  f.mission_goal_binding_=true;
  f.execute();
  assert(f.front_goal_finished_pub_.messages.empty()); // Executor owns the completion event.
 }
}
'''.replace('EXECUTION', execution).replace('PROCESS', process).replace('FAILURE', failure)
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        directory = Path(cls.temp.name)
        (directory/'test.cpp').write_text(code)
        cls.binary = directory/'test'
        result = subprocess.run(['c++', '-std=c++14', '-O2', str(directory/'test.cpp'), '-o', str(cls.binary)],
                                capture_output=True, text=True)
        if result.returncode:
            raise AssertionError(result.stderr)

    def run_case(self, name):
        subprocess.run([str(self.binary), name], check=True, capture_output=True)

    def test_global_goal_finishes_exactly_once(self):
        self.run_case('completion')

    def test_completion_waits_until_full_duration(self):
        self.run_case('timing')

    def test_intermediate_spline_and_emergency_stop_do_not_complete_goal(self):
        self.run_case('intermediate')

    def test_failed_request_cannot_be_completed_by_previous_trajectory(self):
        self.run_case('failed')

    def test_completed_echo_is_for_the_request_actually_planned(self):
        self.run_case('identity')

    def test_bound_mission_completion_is_not_emitted_by_planning_clock(self):
        self.run_case('executor')


if __name__ == '__main__':
    unittest.main()
