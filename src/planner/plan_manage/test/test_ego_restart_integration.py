#!/usr/bin/env python3
"""Run ONLY EGO on a private ROS master with synthetic odom/cloud inputs.

Source /opt/ros/noetic/setup.bash and build_ws/devel/setup.bash first.
No controller, MAVROS, camera, lidar, payload or existing ROS master is used.
"""
import argparse
import json
import math
import os
import re
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'src/airseafly_real400_mission/src'))
from airseafly_real400_mission.configuration import runtime_paths
from airseafly_real400_mission.flight_core import FlightCore
from airseafly_real400_mission.resources import ProcessLock


class ArrivalFixture(FlightCore):
    """Use the common recorded-route arrival rule; never dispatch its effects."""
    def _on_custom_arrival(self):
        self._state('COMPLETE', 'APPROACH_COMPLETE', 'NONE')
        self.done = True
        return True


def wait_for(predicate, description, timeout, processes=()):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for process in processes:
            if process.poll() is not None:
                raise AssertionError('test child exited: %s' % process.returncode)
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError('timed out: ' + description)


def stop_group(process):
    if process is None:
        return
    for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            if sig == signal.SIGINT and process.poll() is None:
                process.send_signal(sig)
            else:
                os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=40.)
    args = parser.parse_args()
    if not shutil.which('roscore') or not shutil.which('roslaunch'):
        raise RuntimeError('source ROS Noetic and build_ws/devel/setup.bash first')
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    assert port not in (11311, 11341)
    logs = Path(tempfile.mkdtemp(prefix='real400-ego-restart-'))
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ['ROS_LOG_DIR'] = str(logs / 'ros')
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ.pop('ROS_NAMESPACE', None)
    master = worker = None
    rospy = None
    timers = []
    streams = []
    # Use the same machine-wide compute slot as task/debug launchers.
    # An active manager or algorithm must finish before this offline test runs.
    compute_lock = ProcessLock(runtime_paths()['compute_lock'], 'EGO隔离到点回归测试')

    def start(argv, name):
        output = (logs / (name + '.log')).open('wb')
        streams.append(output)
        return subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=output,
                                stderr=subprocess.STDOUT, start_new_session=True,
                                env=os.environ.copy())

    try:
        master = start(['roscore', '-p', str(port)], 'master')
        def master_ready():
            try:
                return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/ego_restart_test')[0] == 1
            except (OSError, xmlrpc.client.Error):
                return False
        wait_for(master_ready, 'isolated ROS master', args.timeout, (master,))
        import rospy as ros_python
        rospy = ros_python
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from quadrotor_msgs.msg import PositionCommand
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs import point_cloud2
        from std_msgs.msg import Bool, Header
        rospy.init_node('ego_restart_synthetic_test', anonymous=True, disable_signals=True)
        odom_pub = rospy.Publisher('/Odom_high_freq', Odometry, queue_size=1)
        cloud_pub = rospy.Publisher('/cloud_registered', PointCloud2, queue_size=1)
        lock = threading.Lock()
        yaw = .73
        sim = dict(position=(1.2, -.7, .95), velocity=(0., 0., 0.),
                   odom=False, cloud='none', active=None)
        epochs = {}

        def odom_tick(_event):
            with lock:
                enabled, position, velocity = sim['odom'], sim['position'], sim['velocity']
            if not enabled:
                return
            message = Odometry()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = 'world'
            message.child_frame_id = 'synthetic_body'
            message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = position
            message.pose.pose.orientation.z = math.sin(yaw / 2.)
            message.pose.pose.orientation.w = math.cos(yaw / 2.)
            message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z = velocity
            odom_pub.publish(message)

        def cloud_tick(_event):
            with lock:
                mode, position = sim['cloud'], sim['position']
            if mode == 'none':
                return
            points = []
            if mode == 'valid':
                # Synthetic floor leaves the .95 m cruise plane unobstructed.
                points = [(position[0] + dx, position[1] + dy, .03)
                          for dx in (-2., -1., 0., 1., 2.)
                          for dy in (-2., -1., 0., 1., 2.)]
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=rospy.Time.now(), frame_id='world'), points))

        timers.extend((rospy.Timer(rospy.Duration(.02), odom_tick),
                       rospy.Timer(rospy.Duration(.1), cloud_tick)))
        summaries = []
        for index, goal_xy in enumerate(((1.8, -.7), (1.8, -.1)), 1):
            goal_z = .95 if index == 1 else .85
            name = 'session_%d' % index
            session = 'integration_' + name
            prefix = '/airseafly/mission/session_' + session
            epoch = dict(ready=[], ready_times=[], receipts=[], commands=[], last_command_wall=None,
                         first_command_wall=None, endpoint_stamps=[], endpoint_hold_started=None,
                         release_tracking_lag=False, arrival_wall=None)
            epochs[name] = epoch
            with lock:
                start_position = sim['position']
                sim.update(odom=False, cloud='none', active=name, velocity=(0., 0., 0.))
            arrival = ArrivalFixture()
            arrival.start('takeoff', start_position, yaw, rospy.get_time())
            arrival._begin_navigation((goal_xy[0], goal_xy[1], goal_z), 'target')
            # The planner deliberately receives a goal before its first map in
            # this test; all flight effects stay in memory, never reach hardware.
            arrival.on_group_ready('EGO')
            arrival.tick(rospy.get_time(), start_position, yaw, 'CMD_CTRL')
            dx, dy = goal_xy[0]-start_position[0], goal_xy[1]-start_position[1]
            norm = math.hypot(dx, dy)
            direction = (dx/norm, dy/norm)
            def ready_callback(message, record=epoch):
                with lock:
                    record['ready'].append(message.data)
                    record['ready_times'].append((message.data, time.monotonic()))
            def receipt_callback(message, record=epoch, mission=arrival):
                with lock:
                    record['receipts'].append((message.header.stamp.to_nsec(),
                        (message.pose.position.x, message.pose.position.y, message.pose.position.z)))
                    mission.on_goal_received(mission._goal)
            def command_callback(message, record=epoch, active_session=name, mission=arrival,
                                 initial=start_position, axis=direction, goal=(goal_xy[0],goal_xy[1],goal_z)):
                position = (message.position.x, message.position.y, message.position.z)
                velocity = (message.velocity.x, message.velocity.y, message.velocity.z)
                with lock:
                    record['last_command_wall'] = time.monotonic()
                    if record['first_command_wall'] is None:
                        record['first_command_wall'] = record['last_command_wall']
                    record['commands'].append((position, velocity,
                        (message.acceleration.x, message.acceleration.y, message.acceleration.z),
                        message.yaw, message.header.frame_id))
                    if sim['active'] == active_session:
                        # Keep the simulated aircraft 45 cm behind the command
                        # until the endpoint stream has been verified. Position
                        # then catches up; the real engine must still dwell .5 s.
                        progress=max(0.,sum((position[i]-initial[i])*axis[i] for i in (0,1)))
                        lag=0. if record['release_tracking_lag'] else min(.45,progress)
                        sim['position']=(position[0]-axis[0]*lag,position[1]-axis[1]*lag,position[2])
                        sim['velocity']=velocity
                        acceleration=(message.acceleration.x,message.acceleration.y,message.acceleration.z)
                        # The optimized spline endpoint can differ from the
                        # requested waypoint. Use the task's arrival radius;
                        # exact final-spline evaluation is checked separately
                        # by test_traj_server_end.py.
                        endpoint=(math.dist(position,goal)<=mission.navigation_arrival_radius_m and
                                  max(abs(v) for v in velocity+acceleration)<1e-9)
                        if endpoint:
                            if record['endpoint_hold_started'] is None:
                                record['endpoint_hold_started']=time.monotonic()
                            record['endpoint_stamps'].append(message.header.stamp.to_sec())
                        mission.tick(rospy.get_time(),sim['position'],yaw,'CMD_CTRL')
                        if mission.done and record['arrival_wall'] is None:
                            record['arrival_wall']=time.monotonic()
            subscribers = [rospy.Subscriber(prefix + '/ready', Bool, ready_callback, queue_size=20),
                rospy.Subscriber(prefix + '/goal_received', PoseStamped, receipt_callback, queue_size=20),
                rospy.Subscriber(prefix + '/position_cmd', PositionCommand, command_callback, queue_size=100)]
            goal_pub = rospy.Publisher(prefix + '/goal', PoseStamped, queue_size=1)
            launch = logs / (name + '.launch')
            launch.write_text('''<launch>
<include file="$(find airseafly_real400_mission)/launch/ego_worker.launch">
<arg name="session" value="{session}"/>
<arg name="height_origin_z" value="0.15"/>
<arg name="yaw" value="{yaw}"/>
<arg name="fixed_yaw" value="true"/>
</include></launch>'''.format(session=session, yaw=yaw))
            epoch['worker_start_wall'] = time.monotonic()
            worker = start(['roslaunch', str(launch)], name)
            wait_for(lambda: epoch['ready'] and goal_pub.get_num_connections() > 0,
                     'fresh worker readiness and goal subscription', args.timeout, (master, worker))
            assert epoch['ready'] == [False], epoch['ready']
            goal = PoseStamped()
            goal.header.frame_id = 'world'
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x, goal.pose.position.y = goal_xy
            goal.pose.position.z = goal_z
            goal.pose.orientation.w = 1.
            goal_pub.publish(goal)
            wait_for(lambda: epoch['receipts'], 'goal receipt before initial map', args.timeout, (worker,))
            assert epoch['receipts'][0] == (goal.header.stamp.to_nsec(), (goal_xy[0], goal_xy[1], goal_z))
            assert not epoch['commands']
            with lock:
                sim['odom'] = True
            time.sleep(.4)
            assert True not in epoch['ready'] and not epoch['commands']
            with lock:
                sim['cloud'] = 'empty'
            time.sleep(.4)
            assert True not in epoch['ready'] and not epoch['commands'], 'empty cloud incorrectly became ready'
            with lock:
                epoch['valid_cloud_enabled_wall'] = time.monotonic()
                sim['cloud'] = 'valid'
            wait_for(lambda: True in epoch['ready'], 'first populated map', args.timeout, (worker,))
            wait_for(lambda: len(epoch['commands']) >= 10, 'actual EGO trajectory output', args.timeout, (worker,))
            wait_for(lambda: len(epoch['endpoint_stamps']) >= 60 and
                     epoch['endpoint_stamps'][-1]-epoch['endpoint_stamps'][0] >= .7,
                     'continuous fresh endpoint hold commands', args.timeout, (worker,))
            with lock:
                lag_error=math.dist(sim['position'],(goal_xy[0],goal_xy[1],goal_z))
                assert lag_error > arrival.navigation_arrival_radius_m and not arrival.done
                assert all(b>a for a,b in zip(epoch['endpoint_stamps'],epoch['endpoint_stamps'][1:]))
                epoch['release_tracking_lag']=True
                epoch['tracking_release_wall']=time.monotonic()
            wait_for(lambda: arrival.done, 'actual odometry arrival and dwell', args.timeout, (worker,))
            assert epoch['arrival_wall']-epoch['tracking_release_wall'] >= .49
            assert arrival.phase=='APPROACH_COMPLETE' and arrival.group=='NONE'
            with lock:
                samples = list(epoch['commands'])
                end_position = sim['position']
            endpoint_error = math.hypot(end_position[0] - goal_xy[0], end_position[1] - goal_xy[1])
            assert endpoint_error <= arrival.navigation_arrival_radius_m, ('mission arrival radius', end_position, goal_xy, endpoint_error)
            assert math.dist(samples[0][0], start_position) < .12, (start_position, samples[0][0])
            for position, velocity, acceleration, heading, frame in samples:
                assert position[2] < 1.05, ('ceiling world Z=.15+.9', position)
                assert all(math.isfinite(v) for v in position + velocity + acceleration)
                assert abs(heading - yaw) < 1e-6 and frame == 'world'
            if index == 2:
                assert max(p[0][2] for p in samples)-min(p[0][2] for p in samples) > .04, 'unexpected fixed Z'
            graph = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getSystemState('/ego_restart_test')
            nodes = {node for group in graph[2] for _topic, owners in group for node in owners}
            assert not any(any(label in node.lower() for label in ('mavros', 'px4ctrl', 'camera', 'livox', 'fast_lio')) for node in nodes), nodes
            summaries.append(dict(session=name, start=start_position, goal=(goal_xy[0], goal_xy[1], goal_z),
                                  end=end_position, min_world_z=min(p[0][2] for p in samples), max_world_z=max(p[0][2] for p in samples), endpoint_error_m=endpoint_error, commands=len(samples), ready=epoch['ready'],
                                  startup_false_s=epoch['ready_times'][0][1]-epoch['worker_start_wall'],
                                  first_map_ready_s=next(stamp for value,stamp in epoch['ready_times'] if value)-epoch['valid_cloud_enabled_wall'],
                                  first_command_after_map_enable_s=epoch['first_command_wall']-epoch['valid_cloud_enabled_wall'],
                                  endpoint_hold_frames=len(epoch['endpoint_stamps']),
                                  tracking_lag_before_release_m=lag_error,
                                  arrival_radius_m=arrival.navigation_arrival_radius_m,
                                  actual_arrival_dwell_s=epoch['arrival_wall']-epoch['tracking_release_wall']))
            with lock:
                sim.update(active=None, cloud='none', velocity=(0., 0., 0.))
            stop_group(worker)
            worker = None
            for subscriber in subscribers:
                subscriber.unregister()
            goal_pub.unregister()
        result = dict(result='PASS', isolated_master=os.environ['ROS_MASTER_URI'],
                      hardware_nodes_started=False, ceiling_origin_z=.15, ceiling_relative_m=.9, sessions=summaries, logs=str(logs))
        (logs / 'result.json').write_text(json.dumps(result, indent=2))
    finally:
        if 'epochs' in locals():
            (logs / 'observations.json').write_text(json.dumps(epochs, indent=2))
        stop_group(worker)
        for timer in timers:
            timer.shutdown()
        if rospy is not None:
            rospy.signal_shutdown('isolated EGO restart integration complete')
        stop_group(master)
        compute_lock.close()
        for stream in streams:
            stream.close()
        remaining = []
        for logfile in logs.glob('session_*.log'):
            for pid in re.findall(r'started with pid \[(\d+)\]', logfile.read_text(errors='replace')):
                process = Path('/proc') / pid / 'cmdline'
                if process.exists() and process.read_bytes():
                    remaining.append(int(pid))
        if 'result' in locals():
            result['remaining_worker_pids'] = remaining
            result['private_master_exited'] = master is None or master.poll() is not None
            (logs / 'result.json').write_text(json.dumps(result, indent=2))
            print(json.dumps(result, indent=2))
            assert not remaining and result['private_master_exited'], 'isolated test left child processes'
        print('Integration logs:', logs)


if __name__ == '__main__':
    main()
