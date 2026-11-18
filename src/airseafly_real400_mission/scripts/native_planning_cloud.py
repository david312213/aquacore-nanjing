#!/usr/bin/env python3
"""Convert registered radar clouds in a process separate from localization.

/airseafly/native/planning_cloud is the converted single scan, as before.
/airseafly/native/planning_cloud_memory additionally keeps obstacles already
seen (see planning_memory.py). Scans are merged/published only while subscribed;
the cached obstacles survive an EGO worker reconnect within the same task and
still expire by age/radius while idle. The manager selects this topic for enabled
gate routes. A new task resets memory even with a persistent viewer.
"""
import threading

import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

from airseafly_real400_mission.native_planning_geometry import RawToLocal, transform_registered_cloud
from airseafly_real400_mission.point_cloud import cloud_xyz
from airseafly_real400_mission.planning_memory import VoxelMemory, pointcloud2_fields, xyz_bytes
from airseafly_real400_mission.planning_memory_config import memory_options

MEMORY_PARAM = '/airseafly/native/planning_memory'


class PlanningCloud:
    def __init__(self):
        self.condition = threading.Condition()
        self.transform = self.pending = None
        self.closed = False
        self.memory = None
        self.memory_options = None
        self.center = None
        self.publisher = rospy.Publisher('/airseafly/native/planning_cloud', PointCloud2, queue_size=1)
        self.memory_publisher = rospy.Publisher('/airseafly/native/planning_cloud_memory', PointCloud2, queue_size=1)
        self.worker = threading.Thread(target=self.run, name='native-planning-cloud', daemon=True)
        rospy.on_shutdown(self.shutdown)
        self.worker.start()
        rospy.Subscriber('/airseafly/native/raw_to_local', TransformStamped,
                         self.on_transform, queue_size=1)
        rospy.Subscriber('/airseafly/native/planning_odometry', Odometry, self.on_odometry, queue_size=1)
        rospy.Subscriber('/cloud_registered', PointCloud2, self.on_cloud,
                         queue_size=1, buff_size=16*1024*1024, tcp_nodelay=True)

    def on_transform(self, message):
        try:
            transform = RawToLocal.from_message(message)
        except (ValueError, TypeError) as exc:
            rospy.logerr('Planning frame cannot be read: %s', exc)
            return
        with self.condition:
            if self.transform is not None:
                if (not np.allclose(self.transform.rotation, transform.rotation, atol=1e-12, rtol=0.) or
                        not np.allclose(self.transform.translation, transform.translation, atol=1e-12, rtol=0.) or
                        self.transform.source_frame != transform.source_frame):
                    rospy.logerr('The frozen planning frame changed; restart the localization session together.')
                return
            self.transform = transform
            self.condition.notify()

    def on_odometry(self, message):
        p = message.pose.pose.position
        if np.isfinite(p.x) and np.isfinite(p.y):
            self.center = (p.x, p.y)

    def on_cloud(self, message):
        with self.condition:
            # Latest scan wins while transforming the preceding cloud. This
            # worker never queues a backlog or takes the localization lock.
            self.pending = message
            self.condition.notify()

    def run(self):
        while True:
            with self.condition:
                while not self.closed and (self.pending is None or self.transform is None):
                    self.condition.wait()
                if self.closed:
                    return
                message, transform = self.pending, self.transform
                self.pending = None
            try:
                out = transform_registered_cloud(message, transform)
                if not self.closed and not rospy.is_shutdown():
                    self.publisher.publish(out)
                    self.publish_memory(out)
            except (ValueError, TypeError, np.linalg.LinAlgError) as exc:
                rospy.logwarn_throttle(1., 'Planning point cloud skipped: %s', exc)
            except rospy.ROSException:
                if not self.closed and not rospy.is_shutdown():
                    raise

    def publish_memory(self, scan):
        if self.memory_publisher.get_num_connections() == 0:
            # EGO is restarted between the two gate waypoints. A temporary
            # subscriber gap must not discard the door frame already inside
            # the radar blind radius. Keep the cache, without learning idle
            # scans or refreshing their last-seen times; discard stale points.
            if self.memory is not None and self.center is not None:
                self.memory.prune(scan.header.stamp.to_sec(), self.center)
                if not len(self.memory):
                    rospy.loginfo('Planning memory released after age/radius pruning')
                    self.memory = None
                    self.memory_options = None
            return
        if self.center is None:
            return
        # Cached parameters update on task changes without a master RPC per scan.
        options = memory_options(rospy.get_param_cached(MEMORY_PARAM, {}) or {})
        if self.memory is None or options != self.memory_options:
            origin_z = options.get('origin_z')
            z_max = options.get('z_max_m')
            self.memory = VoxelMemory(options.get('voxel_m', .10), options.get('radius_m', 4.),
                                      options.get('max_age_s', 60.), options.get('min_hits', 2),
                                      None if z_max is None or origin_z is None else origin_z+z_max)
            self.memory_options = dict(options)
            rospy.loginfo('Planning memory started: %s', options)
        stamp = scan.header.stamp.to_sec()
        points = self.memory.crop(cloud_xyz(scan), self.center)
        cloud = self.memory.add(points, stamp, self.center)
        out = PointCloud2()
        out.header = scan.header
        out.height, out.width = 1, int(len(cloud))
        out.fields = pointcloud2_fields(PointField)
        out.is_bigendian, out.point_step = False, 12
        out.row_step, out.is_dense = 12*out.width, True
        out.data = xyz_bytes(cloud)
        self.memory_publisher.publish(out)

    def shutdown(self):
        with self.condition:
            self.closed = True
            self.pending = None
            self.condition.notify_all()


if __name__ == '__main__':
    rospy.init_node('native_planning_cloud')
    node = PlanningCloud()
    rospy.spin()
