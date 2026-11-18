#!/usr/bin/env python3
"""FAST-LIO -> PX4 external odometry with a fixed frame; no flight commands."""
import json
import threading
import time
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import EstimatorStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from airseafly_real400_mission.native_motion import FrozenLioFrame, quaternion
from airseafly_real400_mission.native_planning_geometry import planning_odometry
from airseafly_real400_mission.native_reference import LIO_FRAME
from airseafly_real400_mission.native_freshness import SourceProgress, fresh_stamp
from airseafly_real400_mission.native_covariance import covariances, validate_mode, validate_settings


def xyz(v):
    return (v.x, v.y, v.z)


def xyzw(q):
    return (q.x, q.y, q.z, q.w)


class Bridge:
    # Optional outputs are absent for the existing alignment-only entry.
    planning_output = frame_output = None
    covariance_mode = 'fixed'
    covariance_report = dict(pose='fixed', twist='fixed', source_pose_valid=None, fallback_reason='')
    covariance_fallback_count = 0
    pose_covariance_scale = 1.
    pose_covariance_floor_enabled = True
    angular_velocity_variance = None  # Compatibility for ROS-free legacy fixtures.

    @property
    def covariance_settings(self):
        return validate_settings(self.variances, pose_covariance_scale=self.pose_covariance_scale,
            pose_covariance_floor_enabled=self.pose_covariance_floor_enabled,
            angular_velocity_variance=self.angular_velocity_variance)

    def __init__(self):
        self.lock = threading.RLock()
        self.shutting_down = False
        self.timer = None
        self.frame = FrozenLioFrame(rospy.get_param('~mount/quaternion_xyzw'),
                                    rospy.get_param('~mount/translation_xyz_m'))
        self.imu = self.latest = None
        self.px4_pose = self.estimator = None
        self.estimator_receipt = 0.
        self.sent_stamp = None
        self.input_progress = SourceProgress()
        self.input_count = self.accepted_input_count = self.output_count = 0
        self.rate_at = time.monotonic()
        self.rate_counts = (0, 0, 0)
        self.input_hz = self.accepted_input_hz = self.output_hz = None
        self.last_input_error = ''
        self.variances = tuple(rospy.get_param('~'+key, default) for key, default in
                               (('position_variance', .0025), ('orientation_variance', .0025), ('velocity_variance', .01)))
        self.covariance_mode = validate_mode(rospy.get_param('~covariance_mode', 'fixed'))
        self.pose_covariance_scale = rospy.get_param('~pose_covariance_scale', 1.)
        self.pose_covariance_floor_enabled = rospy.get_param('~pose_covariance_floor_enabled', True)
        self.angular_velocity_variance = rospy.get_param('~angular_velocity_variance', .0025)
        if self.angular_velocity_variance is None:
            raise ValueError('angular_velocity_variance must be a finite positive number')
        # Validate every configured field before advertising or subscribing.
        settings = self.covariance_settings
        self.variances = tuple(settings[key] for key in
                               ('position_variance', 'orientation_variance', 'velocity_variance'))
        self.pose_covariance_scale = settings['pose_covariance_scale']
        self.angular_velocity_variance = settings['angular_velocity_variance']
        self.covariance_fallback_count = 0
        self.covariance_report = dict(pose='waiting_for_sample', twist='fixed',
                                      source_pose_valid=None, fallback_reason='',
                                      pose_scale_applied=None, pose_floor_applied=False)
        self.output = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=1)
        # The mission reads this pre-EKF measurement as its position reference.
        self.lio_output = rospy.Publisher('/airseafly/native/lio_odometry', Odometry, queue_size=1)
        self.planning_output = self.frame_output = None
        if rospy.get_param('~publish_planning_interfaces', False):
            self.planning_output = rospy.Publisher('/airseafly/native/planning_odometry', Odometry, queue_size=1)
            self.frame_output = rospy.Publisher('/airseafly/native/raw_to_local', TransformStamped,
                                                queue_size=1, latch=True)
        self.health = rospy.Publisher('/airseafly/native/localization_healthy', Bool, queue_size=1)
        self.status = rospy.Publisher('/airseafly/native/localization_status', String, queue_size=1)
        self.tf = tf2_ros.StaticTransformBroadcaster()
        # Unique child names avoid replacing any existing MAVROS/FAST-LIO TF.
        parent = rospy.get_param('/mavros/odometry/fcu/odom_parent_id_des', 'odom')+'_ned'
        child = rospy.get_param('/mavros/odometry/fcu/odom_child_id_des', 'base_link')+'_frd'
        transforms = []
        for p, c, q in ((parent, 'native_odom', (.5**.5,.5**.5,0.,0.)),
                        (child, 'native_base', (1.,0.,0.,0.))):
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = rospy.Time.now(), p, c
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
            transforms.append(t)
        self.tf.sendTransform(transforms)
        # Avoid Nagle/delayed-ACK batching of small high-frequency TCPROS
        # packets: queue_size=1 then keeps only the last message of each batch.
        # One odometry sample fits comfortably in this receive buffer.
        rospy.Subscriber('/mavros/imu/data', Imu, self.on_imu, queue_size=1,
                         tcp_nodelay=True, buff_size=65536)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.on_pose, queue_size=1,
                         tcp_nodelay=True, buff_size=65536)
        rospy.Subscriber('/mavros/estimator_status', EstimatorStatus, self.on_estimator, queue_size=1)
        rospy.Subscriber('/Odom_high_freq', Odometry, self.on_odom, queue_size=1,
                         tcp_nodelay=True, buff_size=65536)
        rospy.on_shutdown(self.shutdown)
        with self.lock:
            if not self.shutting_down and not rospy.is_shutdown():
                self.timer = rospy.Timer(rospy.Duration(.02), self.tick)

    def shutdown(self):
        # Serialize with an in-progress tick before ROS tears publishers down.
        with self.lock:
            self.shutting_down = True
            if self.timer is not None:
                self.timer.shutdown()

    def on_imu(self, msg):
        with self.lock: self.imu = msg

    def on_pose(self, msg):
        with self.lock: self.px4_pose = msg

    def on_estimator(self, msg):
        with self.lock: self.estimator, self.estimator_receipt = msg, time.monotonic()

    def on_odom(self, msg):
        with self.lock:
            self.input_count += 1
            try:
                stamp = msg.header.stamp.to_sec()
                if not fresh_stamp(stamp, rospy.get_time()):
                    raise ValueError('FAST-LIO sample timestamp is stale or in the future')
                if self.input_progress.stamp is not None and stamp <= self.input_progress.stamp:
                    return  # Repeated timestamps cannot renew localization validity.
                if self.frame.r_nw is None:
                    if self.imu is None:
                        self.last_input_error = 'waiting for PX4 attitude to initialize the fixed frame'
                        return
                    # Capture the first calculable LIO/PX4-attitude pair. The
                    # body origin is zero at this instant; no arming, settling
                    # or timestamp-alignment condition delays initialization.
                    # Never reinitialize this transform during the process.
                    self.frame.initialize(xyz(msg.pose.pose.position), xyzw(msg.pose.pose.orientation),
                                          (0.,0.,0.), xyzw(self.imu.orientation))
                    if self.frame_output is not None:
                        frozen = TransformStamped()
                        frozen.header.stamp = msg.header.stamp
                        frozen.header.frame_id, frozen.child_frame_id = LIO_FRAME, msg.header.frame_id
                        (frozen.transform.translation.x, frozen.transform.translation.y,
                         frozen.transform.translation.z) = self.frame.t_nw
                        (frozen.transform.rotation.x, frozen.transform.rotation.y,
                         frozen.transform.rotation.z, frozen.transform.rotation.w) = quaternion(self.frame.r_nw)
                        # This message describes raw world in local radar ENU;
                        # it is not broadcast into the pre-existing TF tree.
                        self.frame_output.publish(frozen)
                p, q, v, w = self.frame.transform(xyz(msg.pose.pose.position), xyzw(msg.pose.pose.orientation),
                                                xyz(msg.twist.twist.linear), xyz(msg.twist.twist.angular))
                out = Odometry()
                out.header.stamp = msg.header.stamp  # Never freshen an old estimate.
                out.header.frame_id, out.child_frame_id = 'native_odom', 'native_base'
                out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = p
                (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
                 out.pose.pose.orientation.z, out.pose.pose.orientation.w) = q
                out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = v
                out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = w
                pose_cov, twist_cov, covariance_report = covariances(
                    self.covariance_mode, msg.pose.covariance, self.frame,
                    xyzw(msg.pose.pose.orientation), self.variances,
                    pose_covariance_scale=self.pose_covariance_scale,
                    pose_covariance_floor_enabled=self.pose_covariance_floor_enabled,
                    angular_velocity_variance=self.angular_velocity_variance)
                out.pose.covariance = pose_cov.reshape(-1).tolist()
                out.twist.covariance = twist_cov.reshape(-1).tolist()
                self.covariance_report = covariance_report
                if covariance_report['source_pose_valid'] is False:
                    self.covariance_fallback_count += 1
                    rospy.logwarn_throttle(5., 'Source pose covariance unusable; using fixed variances: %s',
                                          covariance_report['fallback_reason'])
                self.latest = out
                self.input_progress.observe(stamp, time.monotonic())
                self.accepted_input_count += 1
                self.last_input_error = ''
            except (ValueError, TypeError, np.linalg.LinAlgError) as exc:
                # Skip an uncomputable sample. A following usable sample can
                # always resume; input errors never latch or reset the frame.
                self.last_input_error = str(exc)
                rospy.logwarn_throttle(1., 'Native localization sample skipped: %s', self.last_input_error)

    def tick(self, _event):
        try:
            self._tick(_event)
        except rospy.ROSException:
            # ROS may close a publisher between the check and publish when a
            # shutdown has already begun. Do not hide publish failures in flight.
            if not self.shutting_down and not rospy.is_shutdown():
                raise

    def _tick(self, _event):
        with self.lock:
            if self.shutting_down or rospy.is_shutdown():
                return
            now = rospy.get_time()
            ready = self.latest is not None and self.input_progress.fresh(now, time.monotonic())
            if ready and self.latest.header.stamp != self.sent_stamp:
                self.lio_output.publish(self.latest)
                self.output.publish(self.latest)
                if self.planning_output is not None:
                    self.planning_output.publish(planning_odometry(self.latest))
                self.sent_stamp = self.latest.header.stamp
                self.output_count += 1
            wall_now = time.monotonic()
            rate_elapsed = wall_now-self.rate_at
            if rate_elapsed >= 1.:
                counts = (self.input_count, self.accepted_input_count, self.output_count)
                self.input_hz, self.accepted_input_hz, self.output_hz = (
                    (count-previous)/rate_elapsed for count, previous in zip(counts, self.rate_counts))
                self.rate_counts, self.rate_at = counts, wall_now
            estimator=self.estimator
            error = (float(np.linalg.norm(np.array(xyz(self.px4_pose.pose.position)) -
                        np.array(xyz(self.latest.pose.pose.position))))
                     if self.px4_pose is not None and self.latest is not None else None)
            estimator_healthy = bool(estimator is not None and 0. <= time.monotonic()-self.estimator_receipt <= 2. and
                estimator.attitude_status_flag and estimator.velocity_horiz_status_flag and estimator.velocity_vert_status_flag and
                (estimator.pos_horiz_rel_status_flag or estimator.pos_horiz_abs_status_flag) and estimator.pos_vert_abs_status_flag and
                estimator.pred_pos_horiz_rel_status_flag and not estimator.accel_error_status_flag)
            # PX4 v1.17 MAVLink ESTIMATOR_STATUS carries solution_status_flags,
            # not the EKF source-control flags or per-source last-fuse times.
            # SYS_STATUS describes sensor health/presence, not active fusion.
            # In PX4 v1.17 CONST_POS also includes vehicle_at_rest, even with
            # valid external aiding. PRED_POS_HORIZ_REL is instead based on
            # isHorizontalAidingActive(), which excludes fake-position aiding.
            # It proves horizontal aiding exists, not which source is active.
            # Estimator flags do not prove EV fusion. Source freshness is a
            # separate property, used by the command gateway's watchdog.
            # Bad packets are skipped. Only exhaustion of the last good sample's
            # freshness budget stops localization; one reordered packet is not
            # a localization outage.
            healthy = ready
            self.health.publish(Bool(data=bool(healthy)))
            self.status.publish(String(data=json.dumps(dict(enabled=True, healthy=bool(healthy), output_healthy=bool(ready),
                estimator_healthy=estimator_healthy, health_is_gate=True,
                health_basis='source_timestamp_and_monotonic_progress',
                sample_age_s=now-self.latest.header.stamp.to_sec() if self.latest is not None else None,
                fusion_healthy=None, fusion_source_verified=False,
                fusion_evidence='unknown: MAVROS ESTIMATOR_STATUS does not expose active EKF aiding sources',
                input_count=self.input_count, accepted_input_count=self.accepted_input_count,
                output_count=self.output_count, input_hz=self.input_hz,
                accepted_input_hz=self.accepted_input_hz, output_hz=self.output_hz,
                covariance_mode=self.covariance_mode, covariance=self.covariance_report,
                covariance_settings=self.covariance_settings,
                covariance_fallback_count=self.covariance_fallback_count,
                covariance_calibration_verified=False,
                px4_error_m=error if error is not None and np.isfinite(error) else None,
                initialized=self.frame.r_nw is not None, last_input_error=self.last_input_error,
                fault='' if ready else 'FAST-LIO source missing, stale, or timestamp not progressing', stamp=now))))


if __name__ == '__main__':
    rospy.init_node('native_lio_bridge')
    bridge = Bridge()
    rospy.spin()
