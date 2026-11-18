#!/usr/bin/env python3
"""Isolated-master test; synthetic messages only, no hardware drivers."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client

PORT = 11329
with socket.socket() as probe:
    probe.bind(('127.0.0.1', PORT))  # Refuse to touch an existing ROS master.
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % PORT
os.environ['ROS_HOSTNAME'] = '127.0.0.1'
os.environ.pop('ROS_IP', None)

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray
from live_octomap_monitor import sensor_transform

root = Path(__file__).resolve().parent
log = (root/'integration_test.log').open('w')
children = []
results = {}
try:
    core = subprocess.Popen(['roscore', '-p', str(PORT)], stdout=log, stderr=log, start_new_session=True)
    children.append(core)
    master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
    for _ in range(60):
        try:
            if master.getPid('/test')[0] == 1: break
        except OSError: pass
        time.sleep(.2)
    else: raise RuntimeError('test master unavailable')
    rospy.init_node('octomap_synthetic_test')
    assert np.allclose(sensor_transform([10, 0, 0], [0, 0, np.sqrt(.5), np.sqrt(.5)], [1, 0, 0]), [10, 1, 0])
    runtime = subprocess.Popen(['python3', str(root/'live_octomap_monitor.py'), '--headless', '--config',
        str(root.parents[1]/'config/fastlio.yaml')], stdout=log, stderr=log, start_new_session=True)
    children.append(runtime)
    states, clouds, transforms, markers = [], [], [], []
    subscriptions = [
        rospy.Subscriber('/airseafly/octomap_preview/status', DiagnosticArray, lambda m: states.append(m), queue_size=10),
        rospy.Subscriber('/airseafly/octomap_preview/cloud', PointCloud2, lambda m: clouds.append(m), queue_size=10),
        rospy.Subscriber('/airseafly/octomap_preview/tf', TFMessage, lambda m: transforms.append(m), queue_size=10),
        rospy.Subscriber('/airseafly/octomap_preview/occupied_cells_vis_array', MarkerArray, lambda m: markers.append(m), queue_size=1)]
    cp = rospy.Publisher('/airseafly/lio/cloud_body', PointCloud2, queue_size=1)
    op = rospy.Publisher('/airseafly/lio/raw_odom', Odometry, queue_size=5)
    def send(x=10., age=0.):
        stamp = rospy.Time.now()-rospy.Duration(age)
        o = Odometry(header=Header(stamp=stamp, frame_id='lio_init'), child_frame_id='livox_imu')
        o.pose.pose.position.x = x
        o.pose.pose.orientation.w = 1.
        points = [(2., y, z) for y in np.linspace(-.5, .5, 15) for z in np.linspace(-.5, .5, 15)]
        op.publish(o)
        cp.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id='livox_imu'), points))
    for _ in range(75):
        assert runtime.poll() is None, 'runtime exited; inspect integration_test.log'
        send(); time.sleep(.1)
    assert markers and any(len(m.points) for m in markers[-1].markers), 'no occupied voxels'
    assert clouds and transforms
    mapped_x = [p.x for m in markers[-1].markers for p in m.points]
    assert min(mapped_x) > 11.8 and max(mapped_x) < 12.2, 'world transform incorrect'
    results['occupied_map_updates'] = True
    results['world_x_approximately_12m'] = [min(mapped_x), max(mapped_x)]
    results['output_scans_during_7_5s'] = len(clouds)
    assert len(clouds) <= 17, 'rate limiter exceeded'
    count = len(clouds)
    for _ in range(15): send(age=3.); time.sleep(.1)
    assert len(clouds) == count, 'stale input was published'
    results['stale_scan_rejection'] = True
    time.sleep(2)
    assert states[-1].status[0].level != 0 and 'NO SIGNAL' in states[-1].status[0].message
    results['disconnect_warning'] = True
    # A pose discontinuity prevents mixing coordinate epochs, but cannot command landing.
    for _ in range(12): send(x=20.); time.sleep(.1)
    assert len(clouds) == count
    assert 'discontinuity' in states[-1].status[0].message.lower()
    results['discontinuity_freezes_display_only'] = True
    pubs = master.getSystemState('/test')[2][0]
    forbidden = [t for t, _ in pubs if t.startswith(('/mavros/', '/move_base', '/planning/')) or t in ('/tf', '/tf_static')]
    assert not forbidden, forbidden
    results['no_control_or_global_tf_publishers'] = True
    os.killpg(runtime.pid, signal.SIGINT)
    runtime.wait(timeout=15)
    assert core.poll() is None and master.getPid('/test')[0] == 1
    pubs_after = master.getSystemState('/test')[2][0]
    assert any(t == '/airseafly/lio/cloud_body' for t, _ in pubs_after)
    assert not any(t.startswith('/airseafly/octomap_preview/') for t, _ in pubs_after)
    results['monitor_stop_preserves_source_and_master'] = True
    (root/'integration_test_result.json').write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))
finally:
    rospy.signal_shutdown('test finished')
    for child in reversed(children):
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try: child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)
    log.close()
