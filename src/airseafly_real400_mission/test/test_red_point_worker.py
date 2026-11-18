"""Mission observations remain headless, session-bound and source-image-bound."""
from copy import deepcopy
import io
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS
import unittest
from unittest import mock

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src/airseafly_real400_mission/src'))
from airseafly_real400_mission import red_point_worker as worker
from airseafly_real400_mission import red_point_runtime as runtime
from airseafly_real400_mission import red_point_preview as preview

SESSION = '052594d7-c3a8-4a42-80f9-2f759dbab62c'
STATUS = '/airseafly/mission/red_point_052594d7_c3a8_4a42_80f9_2f759dbab62c/status'
ANNOTATED = STATUS.rsplit('/', 1)[0]+'/image_annotated'


def header(stamp=1., seq=1, frame='down_camera_optical'):
    return NS(stamp=NS(to_sec=lambda: stamp), seq=seq, frame_id=frame)


def image(stamp=1., seq=1):
    pixels = np.full((720, 1280, 3), 230, np.uint8)
    cv2.circle(pixels, (603, 364), 140, (0, 160, 0), -1)
    cv2.circle(pixels, (603, 364), 70, (230, 230, 230), -1)
    cv2.circle(pixels, (603, 364), 8, (0, 0, 220), -1)
    return NS(header=header(stamp, seq), width=1280, height=720, pixels=pixels)


def odometry(stamp=1., z=1.):
    return NS(header=header(stamp, frame='fast_lio_local_enu'), pose=NS(pose=NS(
        position=NS(x=0., y=0., z=z), orientation=NS(x=0., y=0., z=0., w=1.))))


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.config = runtime.load_config(runtime.DEFAULT_CONFIG)
        self.observations = worker.MissionObservations(self.config, SESSION)
        self.observations.inputs.on_odometry(odometry())
        self.bridge = NS(imgmsg_to_cv2=lambda message, **kw: message.pixels)

    def process(self, message):
        return self.observations.process(message, self.bridge, 'reused', 10.)

    def test_shared_runtime_keeps_existing_preview_interfaces(self):
        for name in ('load_config', 'CameraOwner', 'ProducerIdentity', 'PreviewInputs', 'attach_ros'):
            self.assertIs(getattr(preview, name), getattr(runtime, name))

    def test_monotonic_sequence_and_retransmitted_sources_never_get_new_identity(self):
        first, second = image(), image(2., 2)
        status1 = self.process(first)[1]
        self.assertEqual(status1['schema'], 'airseafly.red_point.v1')
        self.assertEqual(status1['session_id'], SESSION)
        self.assertEqual(status1['image_sequence'], 1)
        self.assertEqual(status1['confirmed_count'], 1)
        self.assertIsNotNone(status1['candidates'][0]['xy_m'])
        self.assertEqual(status1['green_count'], 1)
        self.assertIsNotNone(status1['green_candidates'][0]['xy_m'])
        self.assertIsNone(self.process(deepcopy(first)))
        self.assertEqual(self.process(second)[1]['image_sequence'], 2)
        self.assertIsNone(self.process(deepcopy(first)))
        self.assertEqual(self.process(image(2., 3))[1]['image_sequence'], 3)
        self.assertEqual(self.observations.image_sequence, 3)

    def test_worker_publishes_green_center_when_red_is_absent(self):
        source = image()
        cv2.circle(source.pixels, (603, 364), 15, (230, 230, 230), -1)
        status = self.process(source)[1]
        self.assertEqual(status['candidates'], [])
        self.assertEqual(status['green_count'], 1)
        self.assertIsNotNone(status['green_candidates'][0]['xy_m'])
        self.assertEqual(status['session_id'], SESSION)
        self.assertEqual(status['image_sequence'], 1)

    def test_pose_epoch_reset_does_not_allow_an_old_image_to_reenter_alignment(self):
        source = image()
        self.process(source)
        self.observations.inputs.session.reset_localization('localization_producer_restarted')
        self.observations.inputs.on_odometry(odometry(2., 2.))
        self.assertIsNone(self.process(source))
        self.assertIsNone(self.observations.inputs.take_image())
        status = self.process(image(2., 2))[1]
        self.assertEqual(status['image_sequence'], 2)
        self.assertEqual(status['localization_epoch'], 1)
        self.assertAlmostEqual(status['camera_height_m'], 2.015)

    def test_latest_mailbox_discards_backlog_without_detecting_older_frames(self):
        for seq in range(1, 4):
            self.observations.inputs.on_image(image(float(seq), seq))
        source = self.observations.inputs.take_image()
        with mock.patch.object(worker, 'detect', wraps=worker.detect) as detection:
            status = self.process(source)[1]
        self.assertEqual(status['image_stamp_s'], 3.)
        self.assertEqual(status['image_sequence'], 1)
        self.assertEqual(detection.call_count, 1)
        self.assertIsNone(self.observations.inputs.take_image())

    def test_detection_failure_clears_candidates_and_next_source_can_recover(self):
        with mock.patch.object(worker, 'detect', side_effect=RuntimeError('bad detector frame')):
            status = self.process(image())[1]
        self.assertEqual(status['candidates'], [])
        self.assertEqual(status['confirmed_count'], 0)
        self.assertEqual(status['image_sequence'], 1)
        self.assertEqual(status['image_error'], 'bad detector frame')
        self.assertEqual(status['green_candidates'], [])
        self.assertEqual(self.process(image(2., 2))[1]['confirmed_count'], 1)

    def test_projection_error_is_an_empty_observation_with_source_sequence(self):
        session = self.observations.inputs.session
        describe = session.describe
        def fail_with_candidates(candidates, *args, **kwargs):
            if candidates:
                raise ValueError('bad projection')
            return describe(candidates, *args, **kwargs)
        with mock.patch.object(session, 'describe', side_effect=fail_with_candidates):
            canvas, status = self.process(image())
        self.assertIsNone(canvas)
        self.assertEqual(status['image_sequence'], 1)
        self.assertEqual(status['candidates'], [])
        self.assertEqual(status['worker_error'], 'bad projection')

    def test_no_pose_age_gate_is_added(self):
        status = self.process(image(1000., 9))[1]
        self.assertEqual(status['confirmed_count'], 1)
        self.assertIsNotNone(status['candidates'][0]['xy_m'])
        self.assertEqual(status['pose']['nearest_dt_s'], -999.)


class ImageWindowTests(unittest.TestCase):
    def setUp(self):
        self.inputs = runtime.PreviewInputs(mock.Mock())
        self.window = worker.ImageWindow(self.inputs)

    def test_no_recognition_during_prewarm_and_no_cached_high_image(self):
        self.inputs.on_image(image(1., 1))
        self.assertIsNone(self.window.take())
        self.inputs.on_image(image(1.9, 2))
        self.window.update(NS(data='NATIVE_SECOND_FRAME@2.000000'))
        self.assertIsNone(self.window.take())  # Mailbox cleared on phase entry.
        self.inputs.on_image(image(1.99, 3))  # Late ROS delivery from descent.
        self.assertIsNone(self.window.take())
        self.inputs.on_image(image(2., 4))
        selected = self.window.take()
        self.assertEqual(selected[0], 'NATIVE_SECOND_FRAME@2.000000')
        self.assertEqual(selected[1].header.seq, 4)

    def test_each_window_is_distinct_without_age_wait_or_minimum_frames(self):
        self.window.update(NS(data='NATIVE_SECOND_FRAME@2.000000'))
        self.inputs.on_image(image(999., 1))
        self.assertIsNotNone(self.window.take())  # No maximum age/extra delay.
        self.window.update(NS(data=''))
        self.inputs.on_image(image(1000., 2))
        self.assertIsNone(self.window.take())
        self.window.update(NS(data='NATIVE_SECOND_FRAME@1001.000000'))
        self.inputs.on_image(image(1001., 3))
        self.assertEqual(self.window.take()[0], 'NATIVE_SECOND_FRAME@1001.000000')


class LiveWorkerTests(unittest.TestCase):
    def run_worker(self, subscribed=False, startup_error=False, target='red_point', controlled=False):
        config = runtime.load_config(runtime.DEFAULT_CONFIG)
        # The H child is never spawned here; record how the worker drives it.
        self.landing = mock.Mock()
        self.landing.detect.return_value = []
        self.landing.status.return_value = dict(enabled=True, state='ready', error='')
        callbacks, publications, subscribers = {}, {}, []
        loops = [0]
        source = image(2., 2)

        def subscribe(topic, kind, callback, **kwargs):
            callbacks[topic] = callback
            subscriber = mock.Mock()
            subscribers.append(subscriber)
            if topic == config['topics']['odometry']:
                callback(odometry())
            if topic == config['topics']['image']:
                callback(image())
                callback(source)
            return subscriber

        def publisher(topic, kind, **kwargs):
            messages = publications.setdefault(topic, [])
            def publish(message):
                messages.append(message)
                if topic == STATUS and len(messages) == 1 and not startup_error:
                    callbacks[config['topics']['image']](deepcopy(source))
            return NS(publish=publish, get_num_connections=lambda: int(subscribed))

        def is_shutdown():
            loops[0] += 1
            if controlled:
                window_callback = callbacks[STATUS.rsplit('/', 1)[0]+'/window']
                if loops[0] == 2:
                    window_callback(NS(data='NATIVE_SECOND_FRAME@2.000000'))
                    callbacks[config['topics']['image']](image(1.9, 3))
                if loops[0] == 3:
                    callbacks[config['topics']['image']](image(3., 4))
                if loops[0] == 4:
                    window_callback(NS(data=''))
                    callbacks[config['topics']['image']](image(4., 5))
                if loops[0] == 5:
                    window_callback(NS(data='NATIVE_SECOND_FRAME@5.000000'))
                    callbacks[config['topics']['image']](image(5., 6))
                return loops[0] >= 6
            if loops[0] == 3:
                # A new pose/frame epoch without a new source must not publish.
                callbacks[config['topics']['odometry']](odometry(.5, 2.))
            if loops[0] == 4:
                callbacks[config['topics']['image']](image(3., 3))
            return loops[0] >= 5

        ros = NS(init_node=mock.Mock(), get_name=lambda: '/red_point_worker_123',
                 get_time=lambda: 10., Publisher=publisher, Subscriber=subscribe,
                 is_shutdown=is_shutdown, logwarn_throttle=mock.Mock())
        master = mock.Mock()
        master.getSystemState.return_value = ([('/down_camera/image_raw', ['/camera'])], [], [])
        bridge = NS(imgmsg_to_cv2=lambda message, **kw: message.pixels,
                    cv2_to_imgmsg=mock.Mock(side_effect=lambda pixels, **kw: NS(pixels=pixels)))
        modules = {'rospy': ros,
            'rosgraph': NS(Master=lambda name: master, MasterError=ValueError, MasterFailure=RuntimeError),
            'roslaunch': NS(), 'cv_bridge': NS(CvBridge=lambda: bridge),
            'geometry_msgs.msg': NS(TransformStamped=object), 'nav_msgs.msg': NS(Odometry=object),
            'sensor_msgs.msg': NS(Image=object), 'std_msgs.msg': NS(String=lambda **kw: NS(**kw))}
        owner = runtime.CameraOwner(config, master, mock.Mock(), '/red_point_worker_123/camera')
        if startup_error:
            owner.start = mock.Mock(side_effect=RuntimeError('camera start failed'))
        stdin = mock.Mock()
        stdin.fileno.side_effect = AssertionError('worker must not read stdin')
        with mock.patch.dict(sys.modules, modules), mock.patch.object(sys, 'stdin', stdin), \
             mock.patch.object(worker, 'CameraOwner', return_value=owner), \
             mock.patch.object(worker, 'LandingPadDetector', return_value=self.landing) as landing_class, \
             mock.patch.object(owner, 'close', wraps=owner.close) as close, \
             mock.patch.object(cv2, 'namedWindow') as create_window, \
             mock.patch.object(cv2, 'imshow') as show, mock.patch.object(cv2, 'waitKey') as key, \
             mock.patch.object(worker, 'annotate', wraps=worker.annotate) as annotate, \
             mock.patch.object(worker.time, 'monotonic', return_value=100.), \
             mock.patch.object(worker.time, 'sleep'):
            if startup_error:
                with self.assertRaisesRegex(RuntimeError, 'camera start failed'):
                    worker.live(config, SESSION, target)
            else:
                self.assertEqual(worker.live(config, SESSION, target, controlled), 0)
            create_window.assert_not_called()
            show.assert_not_called()
            key.assert_not_called()
            stdin.fileno.assert_not_called()
            stdin.read.assert_not_called()
            close.assert_called_once()
            if target == 'landing_pad':
                landing_class.assert_called_once_with(config['landing_pad'])
                self.landing.start.assert_called_once()
                self.landing.close.assert_called_once()
            else:
                landing_class.assert_not_called()  # A red-point window never starts YOLO.
        self.assertEqual(set(publications), {STATUS, ANNOTATED})
        self.assertEqual(config['topics']['status'], '/airseafly/red_point/status')
        for subscriber in subscribers:
            subscriber.unregister.assert_called_once()
        statuses = [json.loads(message.data) for message in publications[STATUS]]
        return statuses, publications, bridge, annotate

    def test_headless_session_topics_latest_images_and_no_epoch_replay(self):
        statuses, publications, bridge, annotate = self.run_worker()
        self.assertEqual([s['image_stamp_s'] for s in statuses], [2., 3.])
        self.assertEqual([s['image_sequence'] for s in statuses], [1, 2])
        self.assertEqual([s['localization_epoch'] for s in statuses], [0, 1])
        self.assertTrue(all(s['session_id'] == SESSION for s in statuses))
        self.assertTrue(all(s['confirmed_count'] == 1 for s in statuses))
        self.assertEqual(publications[ANNOTATED], [])
        bridge.cv2_to_imgmsg.assert_not_called()
        annotate.assert_not_called()

    def test_h_window_uses_only_the_h_detector_and_closes_it(self):
        statuses, _, _, _ = self.run_worker(target='landing_pad')
        self.assertEqual([s['image_sequence'] for s in statuses], [1, 2])
        self.assertEqual(self.landing.detect.call_count, 2)
        self.assertTrue(all(s['confirmed_count'] == 0 and s['green_count'] == 0 for s in statuses))
        self.assertTrue(all(s['landing_detector']['state'] == 'ready' for s in statuses))

    def test_warm_worker_detects_only_images_from_enabled_windows(self):
        statuses, _, _, annotate = self.run_worker(controlled=True)
        self.assertEqual([s['image_stamp_s'] for s in statuses], [3., 5.])
        self.assertEqual([s['image_sequence'] for s in statuses], [1, 2])
        self.assertEqual([s['image_window'] for s in statuses],
                         ['NATIVE_SECOND_FRAME@2.000000', 'NATIVE_SECOND_FRAME@5.000000'])
        self.assertTrue(all(s['confirmed_count'] == 1 for s in statuses))
        annotate.assert_not_called()

    def test_annotation_is_only_rendered_when_subscribed_and_preserves_source_stamp(self):
        statuses, publications, bridge, annotate = self.run_worker(subscribed=True)
        self.assertEqual(annotate.call_count, 2)
        self.assertEqual(bridge.cv2_to_imgmsg.call_count, 2)
        self.assertEqual([m.header.stamp.to_sec() for m in publications[ANNOTATED]], [2., 3.])

    def test_camera_start_failure_clears_observations_and_cleans_owned_resources(self):
        statuses, publications, bridge, annotate = self.run_worker(startup_error=True)
        self.assertEqual(len(statuses), 1)
        self.assertEqual(statuses[0]['session_id'], SESSION)
        self.assertEqual(statuses[0]['candidates'], [])
        self.assertEqual(statuses[0]['worker_error'], 'camera start failed')
        self.assertEqual(statuses[0]['image_sequence'], 0)
        annotate.assert_not_called()

    def test_cli_help_needs_no_ros_and_reports_startup_failure_as_exit_one(self):
        entry = ROOT/'src/airseafly_real400_mission/scripts/red_point_worker.py'
        result = subprocess.run([sys.executable, str(entry), '--help'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--session', result.stdout)
        with mock.patch.object(worker, 'live', side_effect=RuntimeError('camera unavailable')), \
             mock.patch.object(sys, 'stderr', io.StringIO()) as output:
            self.assertEqual(worker.main(['--config', str(runtime.DEFAULT_CONFIG), '--session', SESSION]), 1)
        self.assertIn('camera unavailable', output.getvalue())


if __name__ == '__main__':
    unittest.main()
