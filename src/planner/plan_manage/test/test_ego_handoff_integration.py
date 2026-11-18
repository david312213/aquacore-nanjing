#!/usr/bin/env python3
"""Real EGO/ROS replay on a private loopback master, with synthetic sensors.

No controller, MAVROS, flight mode or payload service is started. Requires a
built ROS Noetic workspace. Executor delays exist only in this test harness.
"""
import json
import argparse
import math
import os
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
import xmlrpc.client

from test_ego_restart_integration import ROOT, wait_for, stop_group
from airseafly_real400_mission.configuration import runtime_paths
from airseafly_real400_mission.resources import ProcessLock
from airseafly_real400_mission.ego_identity import goal_key


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native-only', action='store_true', help='run the recorded native handoff cases only')
    args = parser.parse_args()
    if not shutil.which('roscore') or not shutil.which('roslaunch'):
        raise RuntimeError('requires ROS Noetic and build_ws/devel/setup.bash; integration NOT run')
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    logs = Path(tempfile.mkdtemp(prefix='ego-handoff-'))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port,
                      ROS_IP='127.0.0.1', ROS_LOG_DIR=str(logs/'ros'))
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ.pop('ROS_NAMESPACE', None)
    master = planner = executor = None
    streams, timers, subscribers = [], [], []
    ros = None
    slot = ProcessLock(runtime_paths()['compute_lock'], 'EGO交接与短距离隔离回放')

    def start(argv, name):
        stream = (logs/(name+'.log')).open('wb')
        streams.append(stream)
        return subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=stream,
                                stderr=subprocess.STDOUT, start_new_session=True)

    def start_executor(name):
        return start(['rosrun', 'ego_planner', 'traj_server', '__name:=handoff_executor',
                      '_world_frame:=world', '_traj_server/mission_goal_binding:=true',
                      '_traj_server/hold_at_trajectory_end:=true',
                      '_traj_server/mission_fixed_yaw:=true', '_traj_server/mission_yaw:=0'], name)

    try:
        master = start(['roscore', '-p', str(port)], 'master')
        def master_ready():
            try:
                return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/handoff_test')[0] == 1
            except (OSError, xmlrpc.client.Error):
                return False
        wait_for(master_ready, 'private master', 20., (master,))
        import rospy
        from airseafly_real400_msgs.msg import EgoGoalCommand
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs.point_cloud2 import create_cloud_xyz32
        from std_msgs.msg import Bool, Header
        from traj_utils.msg import Bspline
        ros = rospy
        ros.init_node('handoff_replay', disable_signals=True)
        odom = ros.Publisher('/synthetic/odom', Odometry, queue_size=1)
        cloud = ros.Publisher('/synthetic/cloud', PointCloud2, queue_size=1)
        goals = ros.Publisher('/airseafly/front/goal', PoseStamped, queue_size=1)
        starts = ros.Publisher('/airseafly/front/goal_with_start', EgoGoalCommand, queue_size=1)
        sim = dict(position=(0., 0., .8))
        commands, curves, finished, ready = [], [], [], []
        # Observe generic Bspline only; a diagnostic subscriber must not itself
        # stand in for the delayed executor's connection to goal_trajectory.
        subscribers = [ros.Subscriber('/airseafly/mission/goal_position_cmd', EgoGoalCommand, commands.append),
                       ros.Subscriber('/drone_0_planning/bspline', Bspline, curves.append),
                       ros.Subscriber('/airseafly/front/goal_finished', PoseStamped, finished.append),
                       ros.Subscriber('/airseafly/mission/ego_ready', Bool, lambda m: ready.append(m.data))]
        def odom_tick(_):
            m = Odometry()
            m.header.stamp, m.header.frame_id = ros.Time.now(), 'world'
            m.child_frame_id = 'synthetic_body'
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = sim['position']
            m.pose.pose.orientation.w = 1.
            m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = sim.get('velocity', (0.,0.,0.))
            odom.publish(m)
        def cloud_tick(_):
            cloud.publish(create_cloud_xyz32(Header(stamp=ros.Time.now(), frame_id='world'),
                [(x, y, .02) for x in (-2., -1., 0., 1., 2.) for y in (-2., -1., 0., 1., 2.)]))
        timers = [ros.Timer(ros.Duration(.02), odom_tick), ros.Timer(ros.Duration(.1), cloud_tick)]
        tree = ET.parse(ROOT/'src/planner/plan_manage/launch/single_run_in_exp.launch')
        launch = tree.getroot()
        for node in list(launch.findall('node')):
            if node.get('type') == 'traj_server': launch.remove(node)
        ET.SubElement(launch, 'param', name='/drone_0_ego_planner_node/fsm/mission_goal_binding', value='true', type='bool')
        launch_path = logs/'planner_only.launch'
        tree.write(launch_path, encoding='unicode')
        results = []

        def matching(request):
            return [m for m in commands if goal_key(m.goal) == goal_key(request)]

        def request_at(distance):
            m = PoseStamped()
            m.header.stamp, m.header.frame_id = ros.Time.now(), 'world'
            m.pose.position.x, m.pose.position.y, m.pose.position.z = sim['position']
            m.pose.position.x += distance
            m.pose.orientation.w = 1.
            return m

        def check_endpoint(request):
            p = request.pose.position
            def endpoint_received():
                samples = matching(request)
                if not samples or not any(goal_key(m) == goal_key(request) for m in finished):
                    return False
                end = samples[-1].command
                return (math.dist((end.position.x,end.position.y,end.position.z), (p.x,p.y,p.z)) < 1e-6
                        and max(abs(v) for v in (end.velocity.x,end.velocity.y,end.velocity.z)) < 1e-8)
            # Completion and commands use different TCP connections: either
            # callback can arrive first even when published in the right order.
            wait_for(endpoint_received, 'executor completion and endpoint sample',
                     30., (master, planner, executor))
            samples = matching(request)
            assert all(max(abs(v) for v in (m.command.velocity.x,m.command.velocity.y,m.command.velocity.z)) <= .3152 for m in samples)
            assert all(max(abs(v) for v in (m.command.acceleration.x,m.command.acceleration.y,m.command.acceleration.z)) <= .5252 for m in samples)

        def generated_curve(trajectory_id):
            wait_for(lambda: any(m.traj_id == trajectory_id for m in curves),
                     'matching generated spline', 30., (master, planner, executor))
            return next(m for m in curves if m.traj_id == trajectory_id)

        for delay in (() if args.native_only else (0., .6, 2.)):
            sim['position'] = (0.,0.,.8)
            commands.clear(); curves.clear(); finished.clear(); ready.clear()
            planner = start(['roslaunch', str(launch_path), 'require_initial_map:=true',
                             'odometry_topic:=/synthetic/odom', 'cloud_topic:=/synthetic/cloud'], 'planner-'+str(delay))
            wait_for(lambda: ready and ready[-1] and goals.get_num_connections(), 'planner ready', 30., (planner,))
            request = request_at(.25)
            connected_at = ros.get_time()
            if delay == 0.:
                executor = start_executor('executor-'+str(delay))
            goals.publish(request)
            if delay:
                wait_for(lambda: curves, 'first planned spline before executor', 30., (planner,))
                first_id = curves[-1].traj_id
                sim['position'] = (.03,0.,.8)
                time.sleep(delay)
                assert not matching(request) and not finished
                connected_at = ros.get_time()
                executor = start_executor('executor-'+str(delay))
            wait_for(lambda: matching(request), 'first bound command', 30., (planner,executor))
            sample = matching(request)[0].command
            if delay:
                assert sample.trajectory_id > first_id
            generated = generated_curve(sample.trajectory_id)
            assert generated.start_time.to_sec() >= connected_at
            assert math.dist((sample.position.x,sample.position.y,sample.position.z),sim['position']) < .05
            check_endpoint(request)
            # Actual odometry stays at .03/0 while the old reference reaches .25.
            # New short goals must start from actual state, including 20 cm +/-.
            for distance in (.18,.20,.22,.25):
                request = request_at(distance)
                goals.publish(request)
                wait_for(lambda: matching(request), 'short replacement', 30., (planner,executor))
                sample = matching(request)[0].command
                assert math.dist((sample.position.x,sample.position.y,sample.position.z),sim['position']) < .05
                check_endpoint(request)
            old_id = matching(request)[-1].command.trajectory_id
            stop_group(executor); executor = None
            connected_at = ros.get_time()
            executor = start_executor('reconnect-'+str(delay))
            wait_for(lambda: any(m.command.trajectory_id > old_id for m in matching(request)),
                     'fresh trajectory on reconnect without another goal request', 30., (planner,executor))
            sample = next(m for m in matching(request) if m.command.trajectory_id > old_id)
            generated = generated_curve(sample.command.trajectory_id)
            assert generated.start_time.to_sec() >= connected_at
            results.append(dict(executor_delay_s=delay, short_distances_m=[.18,.20,.22,.25], reconnect=True))
            stop_group(executor); executor = None
            stop_group(planner); planner = None
        # Recorded Oct 29 climb estimate: the held reference has already stopped
        # at cruise Z while LIO still estimates +0.407 m/s. Exercise the real ROS
        # input, FSM, optimizer and trajectory server, including late connection.
        for delay in (0., .8):
            cruise = 1.4193235
            sim.update(position=(-.01099237,.56532998,1.21550198),
                       velocity=(.13290505,-.04188809,.40724242))
            commands.clear(); curves.clear(); finished.clear(); ready.clear()
            planner = start(['roslaunch', str(launch_path), 'require_initial_map:=true',
                             'max_vel:=0.5', 'max_acc:=0.25',
                             'map_max_height:=2.4', 'height_origin_z:=0.0193235',
                             'odometry_topic:=/synthetic/odom', 'cloud_topic:=/synthetic/cloud'], 'native-planner-'+str(delay))
            wait_for(lambda: ready and ready[-1] and starts.get_num_connections(), 'native planner ready', 30., (planner,))
            request = request_at(0.)
            request.pose.position.x, request.pose.position.y, request.pose.position.z = (-2.304086,.554427,cruise)
            bound = EgoGoalCommand()
            bound.goal = request
            bound.command.header = request.header
            held = (.0790548, .496317, cruise)
            bound.command.position.x, bound.command.position.y, bound.command.position.z = held
            if not delay: executor = start_executor('native-executor-'+str(delay))
            starts.publish(bound)
            if delay:
                wait_for(lambda: curves, 'native curve before executor', 30., (planner,))
                time.sleep(delay)
                executor = start_executor('native-executor-'+str(delay))
            wait_for(lambda: matching(request), 'native bound command', 30., (planner,executor))
            sample = matching(request)[0].command
            assert math.dist((sample.position.x,sample.position.y,sample.position.z),held)<.03, sample
            assert abs(sample.velocity.z)<.01, sample
            # Retry cannot reset an executing goal to its original start reference.
            starts.publish(bound)
            wait_for(lambda: matching(request)[-1].command.header.stamp.to_sec() - sample.header.stamp.to_sec() >= 3.,
                     'native horizontal trajectory samples', 10., (planner,executor))
            samples = matching(request)
            max_z_error = max(abs(m.command.position.z-cruise) for m in samples)
            max_vz = max(abs(m.command.velocity.z) for m in samples)
            assert max_z_error<.015 and max_vz<.015, (max_z_error,max_vz)
            assert all(max(abs(v) for v in (m.command.velocity.x,m.command.velocity.y,m.command.velocity.z))<=.527 for m in samples)
            assert all(max(abs(v) for v in (m.command.acceleration.x,m.command.acceleration.y,m.command.acceleration.z))<=.265 for m in samples)
            old_id = samples[-1].command.trajectory_id
            stop_group(executor); executor = None
            executor = start_executor('native-reconnect-'+str(delay))
            wait_for(lambda: any(m.command.trajectory_id>old_id for m in matching(request)),
                     'native reconnect from current measured state', 30., (planner,executor))
            sample = next(m.command for m in matching(request) if m.command.trajectory_id>old_id)
            assert abs(sample.position.z-sim['position'][2])<.05, sample
            results.append(dict(native_reference=True, executor_delay_s=delay,
                                measured_vz=.40724242, max_z_error_m=max_z_error,
                                max_vz_mps=max_vz, reconnect=True))
            stop_group(executor); executor = None
            stop_group(planner); planner = None
        report = dict(passed=True, cases=results, logs=str(logs), hardware=False)
        (logs/'result.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report), flush=True)
    finally:
        for timer in timers: timer.shutdown()
        for sub in subscribers: sub.unregister()
        stop_group(executor); stop_group(planner)
        if ros is not None: ros.signal_shutdown('isolated replay complete')
        stop_group(master)
        slot.close()
        for stream in streams: stream.close()


if __name__ == '__main__':
    main()
