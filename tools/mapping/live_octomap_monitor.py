#!/usr/bin/env python3
"""Display-only OctoMap sidecar. No MAVROS publishers or flight service clients."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import message_filters
import numpy as np
import rospy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from tf.transformations import quaternion_matrix
from tf2_msgs.msg import TFMessage

NS = '/airseafly/octomap_preview'
SENSOR = 'octomap_preview_sensor'


def sensor_transform(position, quaternion, offset):
    position, quaternion = np.asarray(position), np.asarray(quaternion)
    if not np.isfinite(position).all() or not np.isfinite(quaternion).all():
        raise ValueError('non-finite pose')
    if abs(np.linalg.norm(quaternion)-1) > .01:
        raise ValueError('invalid quaternion')
    return quaternion_matrix(quaternion)[:3, :3].dot(offset)+position


def filtered_xyz(cloud, offset):
    fields = {f.name: f for f in cloud.fields}
    if any(k not in fields or fields[k].datatype != 7 for k in ('x', 'y', 'z')):
        raise ValueError('expected float32 xyz')
    dtype = np.dtype(dict(names=['x', 'y', 'z'],
                         formats=[('>' if cloud.is_bigendian else '<')+'f4']*3,
                         offsets=[fields[k].offset for k in ('x', 'y', 'z')],
                         itemsize=cloud.point_step))
    array = np.ndarray((cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
                       strides=(cloud.row_step, cloud.point_step))
    points = np.column_stack([array[k].ravel() for k in ('x', 'y', 'z')])
    # cloud_body already has the lidar->IMU transform. Only shift the origin;
    # the private sensor axes deliberately remain parallel to livox_imu axes.
    points = points[np.isfinite(points).all(axis=1)] - offset
    points = points[np.linalg.norm(points, axis=1) >= .5]
    _, idx = np.unique(np.floor(points/.08).astype(np.int64), axis=0, return_index=True)
    return points[idx]


class Adapter:
    def __init__(self, offset):
        self.offset = offset
        self.lock = threading.Lock()
        self.latest = None
        self.last_stamp = None
        self.last_position = None
        self.last_received = time.monotonic()
        self.blocked = False
        self.count = 0
        self.status = 'Waiting for synchronized FAST-LIO cloud and odometry'
        self.cloud_pub = rospy.Publisher(NS+'/cloud', PointCloud2, queue_size=1)
        self.tf_pub = rospy.Publisher(NS+'/tf', TFMessage, queue_size=5)
        self.static_pub = rospy.Publisher(NS+'/tf_static', TFMessage, queue_size=1, latch=True)
        anchor = TransformStamped()
        anchor.header.stamp = rospy.Time.now()
        anchor.header.frame_id = 'lio_init'
        anchor.child_frame_id = 'octomap_preview_anchor'
        anchor.transform.rotation.w = 1.
        self.static_pub.publish(TFMessage([anchor]))
        self.diag_pub = rospy.Publisher(NS+'/status', DiagnosticArray, queue_size=1)
        self.cloud_sub = message_filters.Subscriber('/airseafly/lio/cloud_body', PointCloud2, queue_size=1)
        self.odom_sub = message_filters.Subscriber('/airseafly/lio/raw_odom', Odometry, queue_size=20)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.cloud_sub, self.odom_sub], 20, .01)
        self.sync.registerCallback(self.receive)
        self.timer = rospy.Timer(rospy.Duration(.5), self.tick)

    def receive(self, cloud, odom):
        with self.lock:
            self.latest = (cloud, odom)
            self.last_received = time.monotonic()

    def tick(self, _event):
        with self.lock:
            pair, self.latest = self.latest, None
        try:
            if self.blocked:
                raise ValueError('Pose discontinuity: stop and restart MONITOR to create a new map')
            if pair is None:
                if time.monotonic()-self.last_received > 1.5:
                    raise ValueError('NO SIGNAL: old map retained; it is not a current obstacle map')
            else:
                self.update(*pair)
        except Exception as exc:
            self.status = str(exc)
            rospy.logwarn_throttle(3, self.status)
        level = DiagnosticStatus.OK if self.status == 'Updating display-only OctoMap' else DiagnosticStatus.WARN
        self.diag_pub.publish(DiagnosticArray(header=Header(stamp=rospy.Time.now()), status=[
            DiagnosticStatus(level=level, name='octomap_preview', message=self.status,
                             hardware_id='display_only', values=[KeyValue('inserted_scans', str(self.count))])]))

    def update(self, cloud, odom):
        if cloud.header.frame_id != 'livox_imu' or odom.header.frame_id != 'lio_init' or odom.child_frame_id != 'livox_imu':
            raise ValueError('Unexpected input coordinate frames')
        stamp = cloud.header.stamp.to_sec()
        age = (rospy.Time.now()-cloud.header.stamp).to_sec()
        if stamp <= 0 or age < -.1 or age > 1.:
            raise ValueError('Stale or future-dated scan rejected')
        if abs((cloud.header.stamp-odom.header.stamp).to_sec()) > .01:
            raise ValueError('Unsynchronized scan rejected')
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        position = np.array([p.x, p.y, p.z])
        origin = sensor_transform(position, [q.x, q.y, q.z, q.w], self.offset)
        if self.last_stamp is not None:
            dt = stamp-self.last_stamp
            if dt <= 0 or dt > 2 or np.linalg.norm(position-self.last_position) > max(1., dt*4.):
                self.blocked = True
                raise ValueError('Pose discontinuity detected; map updates paused')
        points = filtered_xyz(cloud, self.offset)
        if not len(points): raise ValueError('Empty filtered scan')
        tf = TransformStamped()
        tf.header.stamp = cloud.header.stamp
        tf.header.frame_id = 'lio_init'
        tf.child_frame_id = SENSOR
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = origin.tolist()
        tf.transform.rotation = q
        self.tf_pub.publish(TFMessage([tf]))
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=cloud.header.stamp, frame_id=SENSOR), points))
        self.last_stamp, self.last_position = stamp, position
        self.count += 1
        self.status = 'Updating display-only OctoMap'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])
    if not args.headless and not os.environ.get('DISPLAY'):
        parser.error('Run this in the NUC NoMachine desktop terminal (DISPLAY missing)')
    config = yaml.safe_load(Path(args.config).read_text())
    if config['mapping'].get('extrinsic_est_en', False):
        parser.error('Online-changing LiDAR extrinsics unsupported by this display adapter')
    offset = np.asarray(config['mapping']['extrinsic_T'], dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all(): parser.error('Invalid LiDAR origin offset')
    rospy.init_node('airseafly_octomap_preview_adapter', anonymous=True)
    master = rospy.get_master()
    _, _, state = master.getSystemState()
    if any(topic.startswith(NS+'/') and publishers for topic, publishers in state[0]):
        raise RuntimeError('An OctoMap preview is already running; close it first')
    children = []
    common_tf = ['/tf:='+NS+'/tf', '/tf_static:='+NS+'/tf_static']
    try:
        commands = [['rosrun', 'octomap_server', 'octomap_server_node', '__name:=octomap_server', '__ns:='+NS,
                     'cloud_in:='+NS+'/cloud', '_frame_id:=lio_init', '_base_frame_id:='+SENSOR,
                     '_resolution:=0.10', '_sensor_model/max_range:=15.0', '_filter_ground:=false',
                     '_filter_speckles:=true', '_latch:=false']+common_tf]
        if not args.headless:
            commands += [['rosrun', 'rviz', 'rviz', '-d', str(Path(__file__).with_name('live_octomap.rviz')),
                          '__name:=octomap_preview_rviz']+common_tf]
        for cmd in commands:
            children.append(subprocess.Popen(cmd, start_new_session=True))
        adapter = Adapter(offset)
        print('DISPLAY ONLY: 2 Hz / 10 cm / 15 m. Ctrl+C closes these viewers, NOT sensors or recording.', flush=True)
        print('Use this run\'s configured LiDAR-IMU offset; no camera extrinsics or PX4 pose injection.', flush=True)
        while not rospy.is_shutdown():
            if children[0].poll() is not None:
                raise RuntimeError('OctoMap server exited; sensors and recorder remain independent')
            time.sleep(.2)
    finally:
        rospy.signal_shutdown('display closed')
        # Only terminate child process groups owned by this invocation.
        for child in children:
            if child.poll() is None: os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try: child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()


if __name__ == '__main__':
    main()
