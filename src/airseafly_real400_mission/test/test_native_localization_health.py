"""ROS-free replay of source freshness, forwarding and diagnostic-only EKF flags."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from airseafly_real400_mission.native_motion import FrozenLioFrame
from airseafly_real400_mission.native_freshness import SourceProgress


def load_script(name):
    stubs = {
        'rospy': SimpleNamespace(is_shutdown=lambda: False, logwarn_throttle=lambda *args: None,
                                 ROSException=type('ROSException', (Exception,), {})),
        'tf2_ros': SimpleNamespace(),
        'geometry_msgs.msg': SimpleNamespace(TransformStamped=object, PoseStamped=object),
        'sensor_msgs.msg': SimpleNamespace(Imu=object),
        'mavros_msgs.msg': SimpleNamespace(State=object, EstimatorStatus=object),
        'nav_msgs.msg': SimpleNamespace(Odometry=object),
        'std_msgs.msg': SimpleNamespace(Bool=SimpleNamespace, String=SimpleNamespace),
    }
    spec = importlib.util.spec_from_file_location(name, PACKAGE/'scripts'/(name+'.py'))
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


bridge_module = load_script('native_lio_bridge')
ground_module = load_script('native_ground_check')


class Stamp:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds

    def __eq__(self, other):
        return isinstance(other, Stamp) and self.seconds == other.seconds


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def vector(x=0., y=0., z=0.):
    return SimpleNamespace(x=x, y=y, z=z)


def odometry(stamp=0., x=0., y=0., z=0.):
    return SimpleNamespace(header=SimpleNamespace(stamp=Stamp(stamp), frame_id='world'), child_frame_id='odom_imu',
        pose=SimpleNamespace(pose=SimpleNamespace(position=vector(x,y,z),
            orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)), covariance=[0.]*36),
        twist=SimpleNamespace(twist=SimpleNamespace(linear=vector(),angular=vector()),covariance=[0.]*36))


class LiveHealthTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        b = self.bridge = bridge_module.Bridge.__new__(bridge_module.Bridge)
        b.lock = threading.RLock()
        b.shutting_down, b.timer = False, None
        b.frame = FrozenLioFrame((0.,0.,0.,1.),(0.,0.,0.))
        b.frame.initialize((0.,0.,0.),(0.,0.,0.,1.),(0.,0.,0.),(0.,0.,0.,1.))
        b.last_input_error = ''
        b.variances = (.0025,.0025,.01)
        b.imu = SimpleNamespace(header=SimpleNamespace(stamp=Stamp(0.)),
                                orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
        b.input_count = b.accepted_input_count = b.output_count = 0
        b.rate_at, b.rate_counts = self.now, (0, 0, 0)
        b.input_hz = b.accepted_input_hz = b.output_hz = None
        b.estimator = SimpleNamespace(attitude_status_flag=True, velocity_horiz_status_flag=True,
            velocity_vert_status_flag=True, pos_horiz_rel_status_flag=True, pos_horiz_abs_status_flag=False,
            pos_vert_abs_status_flag=True, pred_pos_horiz_rel_status_flag=True,
            const_pos_mode_status_flag=False, accel_error_status_flag=False)
        b.sent_stamp = None
        b.input_progress = SourceProgress()
        b.output, b.health, b.status = Publisher(), Publisher(), Publisher()
        b.lio_output = Publisher()
        self.refresh()

    def refresh(self):
        b = self.bridge
        b.estimator_receipt = self.now
        b.latest = odometry(self.now)
        b.input_progress.observe(self.now, self.now)
        b.px4_pose = SimpleNamespace(header=SimpleNamespace(stamp=Stamp(self.now)),
                                    pose=SimpleNamespace(position=vector()))

    def tick(self):
        with patch.object(bridge_module.rospy, 'get_time', lambda: self.now, create=True), \
                patch.object(bridge_module.time, 'monotonic', lambda: self.now):
            self.bridge.tick(None)
        return json.loads(self.bridge.status.messages[-1].data)

    def feed(self, message):
        with patch.object(bridge_module, 'Odometry', odometry), \
             patch.object(bridge_module.rospy, 'get_time', lambda:self.now, create=True), \
             patch.object(bridge_module.time, 'monotonic', lambda:self.now):
            self.bridge.on_odom(message)

    def test_valid_estimate_and_agreement_do_not_prove_external_fusion(self):
        status = self.tick()
        self.assertTrue(status['healthy'])
        self.assertTrue(status['estimator_healthy'])
        self.assertTrue(status['health_is_gate'])
        self.assertIsNone(status['fusion_healthy'])
        self.assertIs(status['fusion_source_verified'], False)
        self.assertIn('unknown', status['fusion_evidence'])
        self.assertTrue(self.bridge.health.messages[-1].data)

    def test_radar_task_topic_is_the_pre_ekf_measurement(self):
        self.now = 12.
        self.bridge.px4_pose.pose.position.x=100.
        self.feed(odometry(12.,x=2.,y=3.,z=4.))
        self.tick()
        radar=self.bridge.lio_output.messages[-1]
        self.assertIs(radar,self.bridge.output.messages[-1])
        self.assertEqual(radar.pose.pose.position.x,2.)
        self.assertEqual(radar.header.stamp.to_sec(),12.)

    def test_stale_estimator_pose_and_large_disagreement_do_not_gate_output(self):
        self.bridge.estimator_receipt = self.now-2.01
        self.bridge.estimator.accel_error_status_flag = True
        self.bridge.px4_pose.pose.position.x = 100.
        self.bridge.px4_pose.header.stamp = Stamp(0.)
        status = self.tick()
        self.assertFalse(status['estimator_healthy'])
        self.assertTrue(status['healthy'])
        self.assertEqual(status['px4_error_m'], 100.)
        self.assertEqual(len(self.bridge.output.messages), 1)

    def test_input_silence_reports_unhealthy_without_repeating_old_sample(self):
        self.tick()
        sent = len(self.bridge.output.messages)
        self.now += 5.
        status = self.tick()
        self.assertFalse(status['healthy'])
        self.assertEqual(status['sample_age_s'], 5.)
        self.assertIn('stale', status['fault'])
        self.assertEqual(len(self.bridge.output.messages), sent)
        self.feed(odometry(self.now, x=2.))
        self.tick()
        self.assertEqual(len(self.bridge.output.messages), sent+1)

    def test_old_future_duplicate_and_reordered_samples_cannot_replace_fresh_sample(self):
        self.tick()
        for stamp, x in ((1.,0.), (10.,8.), (20.,2.), (9.99,5.)):
            self.feed(odometry(stamp,x=x))
            self.tick()
        self.assertEqual([m.header.stamp.to_sec() for m in self.bridge.output.messages], [10.])
        self.assertEqual(self.bridge.latest.pose.pose.position.x, 0.)
        self.now += .32
        self.feed(odometry(10.,x=2.))
        self.assertFalse(self.tick()['healthy'])
        self.feed(odometry(self.now,x=2.))
        self.assertTrue(self.tick()['healthy'])
        self.assertEqual(len(self.bridge.output.messages), 2)
        self.assertEqual(self.bridge.last_input_error, '')

    def test_frame_aliases_do_not_block_configured_lio_transform(self):
        self.now = 12.
        sample = odometry(12., x=2.)
        sample.header.frame_id = 'lidar_world_alias'
        sample.child_frame_id = 'lidar_imu_alias'
        self.feed(sample)
        self.tick()
        output = self.bridge.output.messages[-1]
        self.assertEqual(output.header.stamp, sample.header.stamp)
        self.assertEqual(output.pose.pose.position.x, 2.)
        self.assertEqual(output.header.frame_id, 'native_odom')
        self.assertEqual(output.child_frame_id, 'native_base')
        self.assertEqual(self.bridge.last_input_error, '')

    def test_first_lio_and_attitude_pair_initializes_without_settling(self):
        b = self.bridge
        b.frame = FrozenLioFrame((0.,0.,0.,1.),(0.,0.,0.))
        b.latest, b.imu = None, None
        b.input_progress = SourceProgress()
        self.now = 1.
        self.feed(odometry(1.,x=5.))
        self.assertIsNone(b.frame.r_nw)
        self.assertFalse(self.tick()['healthy'])
        b.imu = SimpleNamespace(header=SimpleNamespace(stamp=Stamp(100.)),
                                orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
        moving = odometry(2.,x=5.)
        self.now = 2.
        moving.twist.twist.linear.x = 2.
        self.feed(moving)
        self.assertIsNotNone(b.frame.r_nw)
        self.assertTrue(self.tick()['healthy'])
        self.assertAlmostEqual(b.output.messages[-1].pose.pose.position.x, 0.)
        self.now = 3.
        self.feed(odometry(self.now,x=8.))
        self.tick()
        self.assertAlmostEqual(b.output.messages[-1].pose.pose.position.x, 3.)

    def test_nonfinite_sample_is_skipped_and_next_usable_sample_resumes(self):
        self.tick()
        sent = len(self.bridge.output.messages)
        self.now = 11.
        self.feed(odometry(11.,x=float('nan')))
        status = self.tick()
        self.assertIn('finite', status['last_input_error'])
        self.assertEqual(len(self.bridge.output.messages), sent)
        self.now = 12.
        self.feed(odometry(self.now,x=1.))
        self.assertTrue(self.tick()['healthy'])
        self.assertEqual(len(self.bridge.output.messages), sent+1)

    def test_shutdown_stops_timer_and_drops_late_tick(self):
        stopped = []
        self.bridge.timer = SimpleNamespace(shutdown=lambda: stopped.append(True))
        self.bridge.shutdown()
        self.bridge.tick(None)
        self.assertEqual(stopped, [True])
        self.assertTrue(self.bridge.shutting_down)
        self.assertEqual(self.bridge.output.messages, [])
        self.assertEqual(self.bridge.health.messages, [])
        self.assertEqual(self.bridge.status.messages, [])

    def test_publisher_closing_during_shutdown_is_ignored(self):
        closing = [False]
        def publish(_message):
            closing[0] = True
            raise bridge_module.rospy.ROSException('publish() to a closed topic')
        self.bridge.output.publish = publish
        with patch.object(bridge_module.rospy, 'get_time', lambda: self.now, create=True), \
                patch.object(bridge_module.rospy, 'is_shutdown', lambda: closing[0]), \
                patch.object(bridge_module.time, 'monotonic', lambda: self.now):
            self.bridge.tick(None)
        self.assertEqual(self.bridge.health.messages, [])
        self.assertEqual(self.bridge.status.messages, [])

    def test_runtime_publish_failure_is_not_swallowed(self):
        def publish(_message):
            raise bridge_module.rospy.ROSException('unexpected runtime failure')
        self.bridge.output.publish = publish
        with patch.object(bridge_module.rospy, 'get_time', lambda: self.now, create=True), \
                patch.object(bridge_module.time, 'monotonic', lambda: self.now):
            with self.assertRaisesRegex(bridge_module.rospy.ROSException, 'runtime failure'):
                self.bridge.tick(None)


class GroundReportTests(unittest.TestCase):
    def setUp(self):
        self.stamps = [10.+i*.02 for i in range(500)]
        self.ages = [.005]*500
        self.states = [dict(connected=True, armed=False, mode='POSCTL')]
        self.statuses = [dict(healthy=True, px4_error_m=.016, fusion_healthy=None, health_is_gate=True,
                              fusion_source_verified=False)]*500

    def report(self):
        return ground_module.summarize_samples(self.stamps, self.ages, self.statuses, self.states, 10.)

    def test_reports_observations_without_acceptance_verdict(self):
        report = self.report()
        self.assertEqual(report['record_kind'], 'native_localization_observations')
        for name in ('passed', 'passed_means', 'stationary_consistency_passed'):
            self.assertNotIn(name, report)
        self.assertAlmostEqual(report['output_hz'], 50.)
        self.assertEqual(report['localization_healthy_fraction'], 1.)
        self.assertTrue(report['health_is_gate'])
        self.assertIsNone(report['fusion_healthy'])
        self.assertFalse(report['fusion_source_verified'])
        self.assertNotIn('fusion_healthy_fraction', report)

    def test_legacy_fusion_flag_is_not_source_level_evidence_or_live_health(self):
        self.statuses = [dict(fusion_healthy=True, px4_error_m=0.)]*500
        report = self.report()
        self.assertIsNone(report['localization_healthy_fraction'])
        self.assertIsNone(report['fusion_healthy'])
        self.assertFalse(report['fusion_source_verified'])
        self.statuses = [dict(healthy=True, fusion_healthy=True, px4_error_m=0.)]*500
        self.assertEqual(self.report()['localization_healthy_fraction'], 1.)
        self.assertIsNone(self.report()['fusion_healthy'])

    def test_unusual_timestamps_are_recorded_without_verdict(self):
        self.ages[0] = -.01
        self.stamps[1] = self.stamps[0]
        report = self.report()
        self.assertEqual(report['min_sample_age_s'], -.01)
        self.assertFalse(report['unique_monotonic'])
        self.assertNotIn('passed', report)

    def test_armed_or_disconnected_state_is_recorded_without_verdict(self):
        self.states[0].update(armed=True, connected=False)
        report = self.report()
        self.assertEqual(report['final_state'], self.states[0])
        self.assertNotIn('passed', report)

    def test_empty_recording_completes_normally(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'observations.json'
            with patch.object(sys, 'argv', ['native_ground_check', '--output', str(output), '--seconds', '1']), \
                    patch.object(ground_module.rospy, 'init_node', lambda *a, **kw: None, create=True), \
                    patch.object(ground_module.rospy, 'Subscriber',
                                 lambda *a, **kw: SimpleNamespace(unregister=lambda: None), create=True), \
                    patch.object(ground_module.rospy, 'is_shutdown', lambda: True), patch('builtins.print'):
                self.assertEqual(ground_module.main(), 0)
            report = json.loads(output.read_text())
            self.assertEqual(report['output_samples'], 0)
            self.assertIsNone(report['localization_healthy_fraction'])
            self.assertNotIn('passed', report)


if __name__ == '__main__':
    unittest.main()
