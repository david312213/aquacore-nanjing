#!/usr/bin/env python3
"""Exercise the actual request/observer node with a fake controller, private ROS."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client


def main():
    package = Path(__file__).resolve().parents[1]
    out = Path(os.environ['REAL400_LANDING_TEST_OUTPUT']).resolve()
    out.mkdir(parents=True, exist_ok=True)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
    assert port not in (11311, 11341)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port, ROS_IP='127.0.0.1',
                      ROS_HOME=str(out / 'ros_home'))
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ.pop('ROS_NAMESPACE', None)
    master_log = (out / 'master.log').open('w')
    master = subprocess.Popen(['roscore', '-p', str(port)], stdout=master_log,
                              stderr=subprocess.STDOUT, start_new_session=True)
    client = None
    try:
        for _ in range(100):
            try:
                if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/test')[0] == 1:
                    break
            except OSError:
                pass
            time.sleep(.05)
        else:
            raise RuntimeError('private master not ready')
        import rospy
        from mavros_msgs.msg import State, ExtendedState
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Bool, String
        from std_srvs.srv import Trigger, TriggerResponse
        rospy.init_node('offline_landing_fixture', anonymous=True, disable_signals=True)
        publishers = {
            'control': rospy.Publisher('/px4ctrl/control_state', String, queue_size=1, latch=True),
            'priority': rospy.Publisher('/px4ctrl/force_land_active', Bool, queue_size=1, latch=True),
            'landed': rospy.Publisher('/px4ctrl/landed', Bool, queue_size=1, latch=True),
            'state': rospy.Publisher('/mavros/state', State, queue_size=1),
            'extended': rospy.Publisher('/mavros/extended_state', ExtendedState, queue_size=1),
            'odom': rospy.Publisher('/Odom_high_freq', Odometry, queue_size=1)}
        requests, requested_at = [], []
        def request(_):
            requests.append(True)
            requested_at.append(time.monotonic())
            return TriggerResponse(success=True, message='OFFLINE controller accepted')
        service = rospy.Service('/px4ctrl/force_land', Trigger, request)
        env = dict(os.environ)
        env['PYTHONPATH'] = str(package / 'src') + os.pathsep + env.get('PYTHONPATH', '')
        with (out / 'client.log').open('w') as client_log:
            client = subprocess.Popen(['/usr/bin/python3', str(package / 'scripts/safe_land.py')],
                                      env=env, stdout=client_log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 15.
            while client.poll() is None and time.monotonic() < deadline:
                elapsed = time.monotonic() - requested_at[0] if requested_at else -1.
                descending = .25 < elapsed < 1.2
                finished = elapsed >= 1.2
                control = 'MANUAL_CTRL' if finished else ('AUTO_LAND' if descending else 'AUTO_HOVER')
                publishers['control'].publish(String(data=control))
                publishers['priority'].publish(Bool(data=bool(requested_at) and not finished))
                publishers['landed'].publish(Bool(data=elapsed > .8))
                state = State(connected=True, armed=not finished, mode='OFFBOARD')
                publishers['state'].publish(state)
                extended = ExtendedState()
                extended.landed_state = ExtendedState.LANDED_STATE_ON_GROUND if finished else ExtendedState.LANDED_STATE_IN_AIR
                publishers['extended'].publish(extended)
                odom = Odometry(); odom.pose.pose.orientation.w = 1.
                odom.pose.pose.position.z = 0. if finished else .65
                publishers['odom'].publish(odom)
                time.sleep(.03)
            assert client.poll() == 0, 'client did not confirm fake landing'
            assert len(requests) == 1, 'landing request repeated'
        output = (out / 'client.log').read_text()
        assert '降落完成' in output and 'AUTO_LAND' in output
        print('PASS real request/observer node: one request, transitions, PX4 ground + disarm confirmation; no hardware')
        rospy.signal_shutdown('offline check done')
    finally:
        if client is not None and client.poll() is None:
            client.send_signal(signal.SIGINT)
            try: client.wait(timeout=4)
            except subprocess.TimeoutExpired: client.kill(); client.wait()
        os.killpg(master.pid, signal.SIGINT)
        try: master.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(master.pid, signal.SIGTERM); master.wait(timeout=5)
        master_log.close()


if __name__ == '__main__':
    main()
