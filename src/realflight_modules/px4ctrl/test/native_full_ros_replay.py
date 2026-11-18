#!/usr/bin/env python3
"""Real px4ctrl process + fake MAVROS, confined to loopback ROS master 11349.

Start an EMPTY roscore on port 11349 and source the candidate overlay first.
No MAVROS/hardware driver, real FCU, arming command or takeoff request is started.
Tests routing/interface behavior, NOT PX4 dynamics, EKF fusion or flight accuracy.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import tempfile
import threading
import time
import traceback

MASTER = 'http://127.0.0.1:11349'
FULL = dict(position=(2.12, -1.34, .67), velocity=(.21, -.12, .08),
            acceleration=(.08, -.04, .02), yaw=.43, yaw_rate=-.17)
ANCHOR = (2., -1., .65)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', required=True, help='candidate devel/lib/px4ctrl/px4ctrl_node')
    parser.add_argument('--config', required=True, help='original working controller YAML; loaded only on private master')
    parser.add_argument('--output-dir', help='JSON result and controller log directory')
    parser.add_argument('--rate-hz', type=float, default=50.)
    parser.add_argument('--sample-s', type=float, default=2.)
    args = parser.parse_args()
    if os.environ.get('ROS_MASTER_URI') != MASTER:
        raise RuntimeError('set ROS_MASTER_URI exactly to ' + MASTER)
    if args.rate_hz <= 0 or args.sample_s < 1.:
        parser.error('rate must be positive and sample-s must be at least one second')
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ.pop('ROS_NAMESPACE', None)
    import yaml
    import rospy
    import rosnode
    import rosgraph
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Bool, String
    from mavros_msgs.msg import State, ExtendedState, EstimatorStatus, AttitudeTarget, PositionTarget
    from mavros_msgs.srv import SetMode, SetModeResponse, CommandBool, CommandBoolResponse
    from quadrotor_msgs.msg import PositionCommand
    from px4ctrl.srv import ControlOwner

    output_dir = Path(args.output_dir or tempfile.mkdtemp(prefix='native-full-replay-')).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    rospy.init_node('native_full_ros_replay', disable_signals=True)
    unexpected = set(rosnode.get_node_names()) - {'/rosout', rospy.get_name()}
    if unexpected:
        raise RuntimeError('dedicated master must be empty; existing nodes: ' + ', '.join(sorted(unexpected)))
    # No real ROS graph is contacted by this script. A duplicate controller or
    # hardware process on this dedicated graph is a test setup error.
    config = yaml.safe_load(Path(args.config).read_text())
    config.pop('native_control', None)
    config['native_output_rate_hz'] = args.rate_hz
    config['native_stop_on_mode_exit'] = True
    # The existing no_RC parser requires these two enable flags. No takeoff
    # request is published; the fake arm service rejects and records any call.
    config['auto_takeoff_land'].update(no_RC=True, enable=True, enable_auto_arm=True)
    rospy.set_param('/px4ctrl', config)
    lock = threading.Lock()
    sim = dict(mode='POSCTL', armed=False, native_stream=False, native_mask=0, old_stream=True)
    status, outputs, mode_calls, arm_calls = {}, {'local': [], 'attitude': []}, [], []
    stop = threading.Event()
    worker_error = []
    publishers, subscribers, services = {}, [], []
    process = None
    checks, intervals = [], []
    result = dict(passed=False, master=MASTER, hardware_nodes_started=False,
                  real_fcu_connected=False, requested_rate_hz=args.rate_hz,
                  config=str(Path(args.config).resolve()), binary=str(Path(args.binary).resolve()))

    def wait_for(predicate, name, timeout=6.):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            if worker_error:
                raise AssertionError('fake stream failed: ' + worker_error[0])
            if process is not None and process.poll() is not None:
                raise AssertionError('px4ctrl exited (%s); see controller.log' % process.returncode)
            if predicate():
                return
            time.sleep(.01)
        raise AssertionError('timed out: ' + name)

    def feedback(message):
        with lock:
            status.update(json.loads(message.data))

    def state_copy():
        with lock:
            return dict(status)

    def messages(kind, start=0., finish=float('inf')):
        with lock:
            return [(stamp, message) for stamp, message in outputs[kind] if start <= stamp <= finish]

    def record(kind, message):
        with lock:
            outputs[kind].append((time.monotonic(), message))

    def change(**values):
        with lock:
            sim.update(values)

    def mode_service(request):
        called = time.monotonic()
        with lock:
            mode_calls.append(dict(mode=request.custom_mode, received_monotonic=called))
        # Test that the controller continues its hold while the fake FCU RPC
        # is slow. This service only changes an in-memory fake state.
        if request.custom_mode == 'AUTO.LAND':
            time.sleep(2.8)
        change(mode=request.custom_mode)
        return SetModeResponse(mode_sent=True)

    def arm_service(request):
        with lock:
            arm_calls.append(bool(request.value))
        # Any call is a test failure; do not let it simulate a successful arm.
        return CommandBoolResponse(success=False, result=1)

    for topic, message_type in (
        ('/mavros/state', State), ('/mavros/extended_state', ExtendedState),
        ('/Odom_high_freq', Odometry), ('/mavros/imu/data', Imu),
        ('/mavros/local_position/pose', PoseStamped),
        ('/mavros/local_position/velocity_local', TwistStamped),
        ('/mavros/estimator_status', EstimatorStatus),
        ('/airseafly/native/localization_healthy', Bool),
        ('/px4ctrl/native_setpoint', PositionTarget), ('/position_cmd', PositionCommand)):
        publishers[topic] = rospy.Publisher(topic, message_type, queue_size=1)
    subscribers.append(rospy.Subscriber('/px4ctrl/native_status', String, feedback, queue_size=50))
    for key, topic, message_type in (
        ('local', '/mavros/setpoint_raw/local', PositionTarget),
        ('attitude', '/mavros/setpoint_raw/attitude', AttitudeTarget)):
        subscribers.append(rospy.Subscriber(topic, message_type,
            lambda message, kind=key: record(kind, message), queue_size=1000, tcp_nodelay=True))
    services.extend((rospy.Service('/mavros/set_mode', SetMode, mode_service),
                     rospy.Service('/mavros/cmd/arming', CommandBool, arm_service)))

    def fake_stream():
        try:
            while not stop.is_set() and not rospy.is_shutdown():
                with lock:
                    current = dict(sim)
                now = rospy.Time.now()
                state = State()
                state.header.stamp, state.connected = now, True
                state.armed, state.mode = current['armed'], current['mode']
                publishers['/mavros/state'].publish(state)
                extended = ExtendedState()
                extended.landed_state = (ExtendedState.LANDED_STATE_IN_AIR if current['armed']
                                         else ExtendedState.LANDED_STATE_ON_GROUND)
                publishers['/mavros/extended_state'].publish(extended)
                odom = Odometry()
                odom.header.stamp, odom.header.frame_id = now, 'world'
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = ANCHOR
                odom.pose.pose.orientation.w = 1.
                publishers['/Odom_high_freq'].publish(odom)
                imu = Imu()
                imu.header.stamp, imu.orientation.w, imu.linear_acceleration.z = now, 1., 9.81
                publishers['/mavros/imu/data'].publish(imu)
                pose = PoseStamped()
                pose.header.stamp, pose.header.frame_id = now, 'map'
                pose.pose = odom.pose.pose
                publishers['/mavros/local_position/pose'].publish(pose)
                velocity = TwistStamped()
                velocity.header.stamp, velocity.header.frame_id = now, 'map'
                publishers['/mavros/local_position/velocity_local'].publish(velocity)
                estimator = EstimatorStatus()
                estimator.header.stamp = now
                for field in ('attitude', 'velocity_horiz', 'velocity_vert', 'pos_horiz_rel',
                              'pos_vert_abs', 'pred_pos_horiz_rel'):
                    setattr(estimator, field + '_status_flag', True)
                publishers['/mavros/estimator_status'].publish(estimator)
                publishers['/airseafly/native/localization_healthy'].publish(Bool(data=True))
                if current['native_stream']:
                    target = PositionTarget()
                    target.header.stamp, target.header.frame_id = now, 'map'
                    target.coordinate_frame, target.type_mask = PositionTarget.FRAME_LOCAL_NED, current['native_mask']
                    target.position.x, target.position.y, target.position.z = FULL['position']
                    target.velocity.x, target.velocity.y, target.velocity.z = FULL['velocity']
                    target.acceleration_or_force.x, target.acceleration_or_force.y, target.acceleration_or_force.z = FULL['acceleration']
                    target.yaw, target.yaw_rate = FULL['yaw'], FULL['yaw_rate']
                    if current['native_mask'] == 2552:
                        target.velocity.x = target.velocity.y = target.velocity.z = 0.
                        target.acceleration_or_force.x = target.acceleration_or_force.y = target.acceleration_or_force.z = 0.
                        target.yaw_rate = 0.
                    publishers['/px4ctrl/native_setpoint'].publish(target)
                if current['old_stream']:
                    old = PositionCommand()
                    old.header.stamp, old.header.frame_id = now, 'world'
                    old.position.x, old.position.z = 50., .65
                    old.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
                    publishers['/position_cmd'].publish(old)
                stop.wait(.01)
        except Exception:
            worker_error.append(traceback.format_exc())

    def is_full(message):
        if message.type_mask != 0 or message.coordinate_frame != PositionTarget.FRAME_LOCAL_NED:
            return False
        for field, expected in (('position', FULL['position']), ('velocity', FULL['velocity']),
                                ('acceleration_or_force', FULL['acceleration'])):
            vector = getattr(message, field)
            if any(abs(value-ref) > 1e-6 for value, ref in zip((vector.x, vector.y, vector.z), expected)):
                return False
        return abs(message.yaw-FULL['yaw']) < 1e-6 and abs(message.yaw_rate-FULL['yaw_rate']) < 1e-6

    def assert_no_output(start, label):
        time.sleep(.30)
        assert not messages('local', start), label + ': native stream continued'
        assert not messages('attitude', start), label + ': legacy output unexpectedly resumed'

    worker = threading.Thread(target=fake_stream, daemon=True)
    worker.start()
    controller_log = (output_dir/'controller.log').open('w')
    try:
        process = subprocess.Popen([args.binary, '__name:=px4ctrl', '~odom:=/Odom_high_freq',
                                    '~cmd:=/position_cmd'], stdout=controller_log,
                                   stderr=subprocess.STDOUT, start_new_session=True,
                                   env=os.environ.copy())
        rospy.wait_for_service('/px4ctrl/control_owner', timeout=12.)
        owner = rospy.ServiceProxy('/px4ctrl/control_owner', ControlOwner)
        wait_for(lambda: state_copy().get('pose_received'), 'fake pose consumed')
        response = owner('PX4_NATIVE')
        assert response.accepted, response.message
        wait_for(lambda: state_copy().get('owner') == 'PX4_NATIVE', 'native acquired')
        # Merely update the fake FCU state; no arm or set-mode request is made.
        change(mode='OFFBOARD', armed=True)
        wait_for(lambda: state_copy().get('offboard') is True, 'fake OFFBOARD received')
        mark = time.monotonic()
        change(native_stream=True)
        wait_for(lambda: any(is_full(message) for _, message in messages('local', mark)), 'full p/v/a/yaw-rate accepted')
        time.sleep(.15)
        sample_start = time.monotonic()
        time.sleep(args.sample_s)
        sample_end = time.monotonic()
        samples = messages('local', sample_start, sample_end)
        assert len(samples) >= args.rate_hz*args.sample_s*.85, len(samples)
        assert all(is_full(message) for _, message in samples), 'full reference mutated or replaced'
        assert not messages('attitude', sample_start, sample_end), 'two command families active'
        stamps = [message.header.stamp.to_sec() for _, message in samples]
        # Cached forwarding keeps its source timestamp; a duplicate must never
        # be made younger by the gateway. Fresh source packets still advance it.
        assert all(right >= left for left, right in zip(stamps, stamps[1:])), 'reordered output stamps'
        intervals = [right-left for left, right in zip(stamps, stamps[1:])]
        rate = (len(stamps)-1)/(stamps[-1]-stamps[0])
        assert args.rate_hz*.90 <= rate <= args.rate_hz*1.10, rate
        assert max(intervals) <= .12, ('stream gap', max(intervals))
        status_sample = state_copy()
        assert status_sample['output_rate_hz'] == args.rate_hz
        system_publishers = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
        for topic in ('/mavros/setpoint_raw/local', '/mavros/setpoint_raw/attitude'):
            assert system_publishers.get(topic) == ['/px4ctrl'], (topic, system_publishers.get(topic))
        checks += ['type_mask_0_pva_yaw_rate_preserved', 'single_publisher_single_active_output', 'configured_50hz_reference']
        result['frequency'] = dict(samples=len(samples), measured_hz=rate,
            median_interval_s=statistics.median(intervals),
            p95_interval_s=sorted(intervals)[int(.95*(len(intervals)-1))], max_interval_s=max(intervals))

        # H correction waypoints use this same binary, with velocity and
        # acceleration ignored rather than enabled as zero feedforward.
        mark = time.monotonic()
        change(native_mask=2552)
        wait_for(lambda: any(message.type_mask == 2552 for _, message in messages('local',mark)),
                 'H position/yaw-only mask accepted')
        time.sleep(.1)
        mark = time.monotonic(); time.sleep(.4)
        positions = messages('local',mark)
        assert len(positions) >= args.rate_hz*.3, len(positions)
        for _, message in positions:
            assert message.type_mask == 2552
            assert all(abs(getattr(message.position,k)-v) < 1e-6 for k,v in zip(('x','y','z'),FULL['position']))
            assert abs(message.yaw-FULL['yaw']) < 1e-6
            for field in ('velocity','acceleration_or_force'):
                assert all(getattr(getattr(message,field),k) == 0. for k in ('x','y','z'))
        assert not messages('attitude',mark), 'legacy output during H position-only ownership'
        checks.append('h_position_yaw_mask_2552_no_velocity_acceleration_feedforward')
        change(native_mask=0)
        mark = time.monotonic()
        wait_for(lambda: any(is_full(message) for _,message in messages('local',mark)), 'full reference restored after H')

        mode_count = len(mode_calls)
        change(mode='ALTCTL')
        wait_for(lambda: state_copy().get('manual_paused') is True, 'manual exit latched')
        time.sleep(.06)
        assert_no_output(time.monotonic(), 'manual ALTCTL')
        assert len(mode_calls) == mode_count, 'automatic mode intervention after pilot exit'
        change(mode='OFFBOARD')
        wait_for(lambda: state_copy().get('offboard') is True, 'fake OFFBOARD restored')
        assert state_copy()['manual_paused'] is True
        time.sleep(.06)
        assert_no_output(time.monotonic(), 'OFFBOARD alone must not resume')
        checks += ['manual_mode_exit_stops_both_outputs', 'no_auto_mode_intervention', 'offboard_return_stays_paused']

        change(native_stream=False)
        time.sleep(.04)
        assert owner('PX4_NATIVE').accepted
        wait_for(lambda: state_copy().get('manual_paused') is False, 'explicit owner clears pause')
        mark = time.monotonic()
        wait_for(lambda: any(abs(message.position.x-ANCHOR[0]) < 1e-6 and message.type_mask == 2496
                            for _, message in messages('local', mark)), 'reacquisition captures current pose')
        change(native_stream=True)
        wait_for(lambda: any(is_full(message) for _, message in messages('local', mark)), 'explicit resume accepts full commands')
        checks.append('explicit_owner_reacquisition_resumes_at_current_pose')

        # Exercise the same gateway's opposite route, with no stale EGO producer.
        change(old_stream=False, native_stream=False)
        time.sleep(.04)
        assert owner('PX4CTRL').accepted
        wait_for(lambda: state_copy().get('owner') == 'PX4CTRL', 'legacy handback')
        time.sleep(.06)
        mark = time.monotonic()
        time.sleep(.20)
        assert messages('attitude', mark), 'legacy hover did not produce attitude'
        assert not messages('local', mark), 'local setpoints continued under legacy owner'
        checks.append('explicit_legacy_handoff_is_exclusive')
        assert owner('PX4_NATIVE').accepted
        wait_for(lambda: state_copy().get('owner') == 'PX4_NATIVE', 'native reacquired for landing')
        change(native_stream=True)
        time.sleep(.10)

        requested = time.monotonic()
        response = owner('PX4_AUTO_LAND')
        service_elapsed = time.monotonic()-requested
        assert response.accepted and service_elapsed < .30, (response.message, service_elapsed)
        wait_for(lambda: any(call['mode'] == 'AUTO.LAND' for call in mode_calls), 'fake land RPC started')
        time.sleep(2.2)
        holding = messages('local', requested+.15, time.monotonic())
        assert len(holding) >= 45, 'slow mode RPC blocked hold output'
        assert messages('local', requested+2.05), 'valid landing hold stopped at the old two-second cutoff'
        assert state_copy().get('landing_state') == 'requesting', 'queued RPC was mistaken for mode confirmation'
        assert state_copy().get('landing_attempts') == 1, 'another RPC overlapped the pending landing request'
        assert state_copy().get('mode') == 'OFFBOARD'
        assert all(abs(message.position.x-ANCHOR[0]) < 1e-6 and
                   abs(message.velocity.x) < 1e-6 and message.type_mask == 2496
                   for _, message in holding), 'landing wait did not capture a zero-speed hold'
        assert not messages('attitude', requested+.06), 'legacy output during landing handoff'
        wait_for(lambda: state_copy().get('mode') == 'AUTO.LAND' and
                 state_copy().get('landing_state') == 'confirmed', 'fake AUTO.LAND feedback confirms handoff')
        assert state_copy()['manual_paused'] is False, 'entering AUTO.LAND was mistaken for pilot takeover'
        time.sleep(.06)
        assert_no_output(time.monotonic(), 'PX4 AUTO.LAND ownership')
        change(mode='POSCTL')
        wait_for(lambda: state_copy().get('manual_paused') is True, 'manual exit from confirmed AUTO.LAND latched')
        assert state_copy().get('landing_state') == 'cancelled'
        time.sleep(.06)
        assert_no_output(time.monotonic(), 'pilot cancelled AUTO.LAND')
        change(mode='OFFBOARD')
        wait_for(lambda: state_copy().get('offboard') is True, 'fake OFFBOARD after landing cancellation')
        assert state_copy()['manual_paused'] is True
        time.sleep(.06)
        assert_no_output(time.monotonic(), 'OFFBOARD must not revive the landing hold')
        change(native_stream=False)
        time.sleep(.04)
        assert owner('PX4_NATIVE').accepted
        wait_for(lambda: state_copy().get('manual_paused') is False, 'explicit acquisition after landing cancellation')
        mark = time.monotonic()
        wait_for(lambda: any(abs(message.position.x-ANCHOR[0]) < 1e-6 and message.type_mask == 2496
                            for _, message in messages('local', mark)), 'landing cancellation resumes at captured pose')
        assert [call['mode'] for call in mode_calls] == ['AUTO.LAND'], mode_calls
        assert not arm_calls, 'test unexpectedly sent an arming request'
        checks += ['auto_land_request_nonblocking', 'valid_land_hold_beyond_two_seconds',
                   'one_pending_land_rpc', 'hold_until_auto_land_feedback', 'auto_land_feedback_stops_offboard_outputs',
                   'auto_land_manual_exit_latches_and_stops_outputs', 'offboard_after_land_exit_stays_paused',
                   'explicit_owner_after_land_exit_captures_current_pose', 'no_arming_or_takeoff_requests']
        result.update(passed=True, checks=checks, owner=state_copy().get('owner'),
                      mode_calls=mode_calls, arm_calls=arm_calls,
                      auto_land_service_elapsed_s=service_elapsed,
                      output_counts={key:len(messages(key)) for key in outputs})
    except Exception as error:
        result.update(error=str(error), checks=checks, status=state_copy(), traceback=traceback.format_exc())
        raise
    finally:
        stop.set()
        worker.join(timeout=2.)
        if process is not None:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2.)
            result['controller_returncode'] = process.returncode
        controller_log.close()
        for subscriber in subscribers:
            subscriber.unregister()
        for server in services:
            server.shutdown('isolated replay finished')
        result['controller_log'] = str(output_dir/'controller.log')
        (output_dir/'result.json').write_text(json.dumps(result, indent=2, sort_keys=True)+'\n')
        print(json.dumps(result, sort_keys=True))
        rospy.signal_shutdown('isolated replay complete')


if __name__ == '__main__':
    main()
