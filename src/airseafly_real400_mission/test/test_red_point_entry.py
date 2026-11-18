"""Observation-only ROS boundary, camera ownership and offline shell entry."""
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
for package in ('airseafly_real400_mission',):
    sys.path.insert(0, str(ROOT/'src'/package/'src'))
from airseafly_real400_mission import red_point_preview as preview
from airseafly_real400_mission.red_point import RedPointSession


def header(stamp, frame='fast_lio_local_enu'):
    return NS(stamp=NS(to_sec=lambda: stamp), frame_id=frame)


def odometry(stamp, z):
    return NS(header=header(stamp), pose=NS(pose=NS(position=NS(x=0., y=0., z=z),
        orientation=NS(x=0., y=0., z=0., w=1.))))


class CameraOwnershipTests(unittest.TestCase):
    def setUp(self):
        self.config = preview.load_config(preview.DEFAULT_CONFIG)
        self.master, self.factory = mock.Mock(), mock.Mock()
        self.master.getSystemState.return_value = ([], [], [])
        self.owner = preview.CameraOwner(self.config, self.master, self.factory, '/preview_123/camera')

    def test_existing_publisher_is_never_stopped_or_replaced(self):
        self.master.getSystemState.return_value = ([('/down_camera/image_raw', ['/down_camera'])], [], [])
        self.assertEqual(self.owner.start(), 'reused')
        self.owner.close()
        self.factory.assert_not_called()
        self.assertEqual(self.master.method_calls, [mock.call.getSystemState()])

    def test_owned_camera_has_unique_namespace_and_only_its_parent_is_shutdown(self):
        self.assertEqual(self.owner.start(), 'started_here')
        path, arguments = self.factory.call_args.args
        self.assertEqual(Path(path), preview.CAMERA_LAUNCH)
        self.assertIn('owner_ns:=/preview_123/camera', arguments)
        self.assertIn('image_topic:=/down_camera/image_raw', arguments)
        self.factory.return_value.start.assert_called_once_with()
        self.owner.close()
        self.owner.close()
        self.factory.return_value.shutdown.assert_called_once_with()

    def test_partial_launch_failure_can_clean_up_only_owned_parent(self):
        self.factory.return_value.start.side_effect = RuntimeError('failed')
        with self.assertRaises(RuntimeError):
            self.owner.start()
        self.owner.close()
        self.factory.return_value.shutdown.assert_called_once_with()

    def test_subscribe_only_config_does_not_start_hardware(self):
        self.config['camera']['auto_start'] = False
        self.assertEqual(self.owner.start(), 'subscribe_only')
        self.owner.close()
        self.factory.assert_not_called()

    def test_camera_launch_only_includes_existing_df200_capture(self):
        tree = ET.parse(preview.CAMERA_LAUNCH)
        includes = tree.findall('.//include')
        self.assertEqual([node.get('file') for node in includes],
                         ['$(find airseafly_real400_mission)/launch/df200_camera.launch'])
        self.assertTrue((ROOT/'src/airseafly_real400_mission/launch/df200_camera.launch').samefile(ROOT/'config/cameras/down.launch'))
        camera = ET.parse(ROOT/'config/cameras/down.launch').find('node')
        self.assertEqual((camera.get('pkg'), camera.get('type')), ('usb_cam', 'usb_cam_node'))
        self.assertEqual(tree.find('group').get('ns'), '$(arg owner_ns)')
        remaps = {r.get('from'): r.get('to') for r in tree.findall('.//remap')}
        self.assertEqual(remaps['down_camera/image_raw'], '$(arg image_topic)')

    def test_owned_camera_uses_configured_format_and_calibration(self):
        c = self.config['camera']
        c.update(image_width=640, image_height=480, pixel_format='yuyv', framerate=25)
        self.owner.start()
        arguments = self.factory.call_args.args[1]
        for expected in ('image_width:=640', 'image_height:=480', 'pixel_format:=yuyv', 'framerate:=25',
                         'camera_info_url:=file://'+c['calibration'],
                         'camera_name:='+self.config['projection']['intrinsics']['camera_name']):
            self.assertIn(expected, arguments)

    def test_launch_defaults_stay_df200_for_callers_without_format(self):
        args = {a.get('name'): a.get('default') for a in
                ET.parse(ROOT/'config/cameras/down.launch').findall('arg')}
        self.assertEqual((args['image_width'], args['image_height'], args['pixel_format'], args['framerate']),
                         ('1280', '720', 'yuyv', '9'))
        self.assertTrue(args['camera_info_url'].endswith('config/aircraft/down_hd_1280x720_yuyv_calib.yaml'))
        passed = {a.get('name') for a in ET.parse(preview.CAMERA_LAUNCH).findall('.//include/arg')}
        self.assertLessEqual({'image_width', 'image_height', 'pixel_format', 'framerate',
                              'camera_name', 'camera_info_url'}, passed)


class CalibrationTests(unittest.TestCase):
    def write_calibration(self, folder, fx=400., size=(640, 480)):
        path = Path(folder)/'cam.yaml'
        path.write_text(yaml.safe_dump(dict(
            image_width=size[0], image_height=size[1], camera_name='new_cam',
            camera_matrix=dict(rows=3, cols=3, data=[fx, 0., 320., 0., fx, 240., 0., 0., 1.]),
            distortion_coefficients=dict(rows=1, cols=5, data=[0.]*5))), encoding='utf-8')
        return path

    def config_with(self, calibration):
        data = yaml.safe_load(preview.DEFAULT_CONFIG.read_text(encoding='utf-8'))
        data['camera']['calibration'] = str(calibration)
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False, encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, allow_unicode=True)
        self.addCleanup(os.unlink, stream.name)
        return preview.load_config(stream.name)

    def test_configured_calibration_drives_aim_and_projection(self):
        from airseafly_real400_mission.down_projection import camera_aim, pixel_displacement
        with tempfile.TemporaryDirectory() as folder:
            config = self.config_with(self.write_calibration(folder))
        down = dict(config['projection'], reference_width=640, reference_height=480)
        self.assertEqual(camera_aim(down), (320., 240.))
        self.assertEqual(config['projection']['intrinsics']['camera_name'], 'new_cam')
        # 40 px right of centre at fx=400 and 1 m height is 0.1 m in the camera plane.
        offset = np.asarray(down['pixel_to_body']) @ [0.1, 0.]
        dx, dy = pixel_displacement([360., 240.], 1., -float(down['body_yaw_offset_rad']), down)
        np.testing.assert_allclose([dx, dy], offset, atol=1e-9)

    def test_projection_without_intrinsics_keeps_df200(self):
        from airseafly_real400_mission.down_projection import camera_aim
        from airseafly_real400_mission.df200_geometry import CAMERA_MATRIX
        down = dict(reference_width=1280, reference_height=720)
        self.assertEqual(camera_aim(down), (CAMERA_MATRIX[0][2], CAMERA_MATRIX[1][2]))

    def test_older_config_without_format_keys_loads_df200(self):
        data = yaml.safe_load(preview.DEFAULT_CONFIG.read_text(encoding='utf-8'))
        for key in ('image_width', 'image_height', 'pixel_format', 'framerate', 'calibration'):
            data['camera'].pop(key, None)
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False, encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, allow_unicode=True)
        self.addCleanup(os.unlink, stream.name)
        config = preview.load_config(stream.name)
        self.assertEqual((config['camera']['image_width'], config['camera']['pixel_format']), (1280, 'yuyv'))
        self.assertEqual(config['projection']['intrinsics']['calibration_size'], [1280., 720.])

    def test_relative_calibration_resolves_from_workspace_root(self):
        config = preview.load_config(preview.DEFAULT_CONFIG)
        self.assertTrue(Path(config['camera']['calibration']).is_absolute())
        self.assertTrue(Path(config['camera']['calibration']).exists())

    def test_flight_config_calibration_matches_its_stream(self):
        # A camera swap without recalibration would scale another lens's intrinsics.
        config = preview.load_config(preview.DEFAULT_CONFIG)
        camera = config['camera']
        self.assertEqual(config['projection']['intrinsics']['calibration_size'],
                         [float(camera['image_width']), float(camera['image_height'])])


class InterfaceTests(unittest.TestCase):
    def setUp(self):
        self.config = preview.load_config(preview.DEFAULT_CONFIG)

    def test_adapter_has_only_observation_publishers_and_three_inputs(self):
        ros = mock.Mock()
        types = {key: object() for key in ('String', 'Image', 'Odometry', 'TransformStamped')}
        inputs = preview.PreviewInputs(RedPointSession(self.config))
        pubs, subs = preview.attach_ros(ros, types, self.config, inputs)
        self.assertEqual([(call.args[0], call.args[1]) for call in ros.Publisher.call_args_list], [
            ('/airseafly/red_point/status', types['String']),
            ('/airseafly/red_point/image_annotated', types['Image'])])
        self.assertEqual([call.args[0] for call in ros.Subscriber.call_args_list], [
            '/airseafly/native/raw_to_local', '/airseafly/native/planning_odometry', '/down_camera/image_raw'])
        self.assertEqual(set(call[0] for call in ros.method_calls), {'Publisher', 'Subscriber'})
        self.assertEqual((len(pubs), len(subs)), (2, 3))

    def test_latest_image_mailbox_does_not_accumulate_or_replay(self):
        inputs = preview.PreviewInputs(RedPointSession(self.config))
        inputs.on_image('first')
        inputs.on_image('second')
        self.assertEqual(inputs.take_image(), 'second')
        self.assertIsNone(inputs.take_image())

    def test_independent_lio_process_restart_is_detected_by_uri_not_age(self):
        master = mock.Mock()
        master.getSystemState.return_value = ([('/Odom_high_freq', ['/laserMapping']),
                                              ('/down_camera/image_raw', ['/camera'])], [], [])
        master.lookupNode.return_value = 'http://computer:10001'
        identity = preview.ProducerIdentity(['/Odom_high_freq', '/airseafly/native/planning_odometry'])
        self.assertFalse(identity.changed(master))
        self.assertFalse(identity.changed(master))
        master.lookupNode.return_value = 'http://computer:10002'
        self.assertTrue(identity.changed(master))
        self.assertFalse(identity.changed(master))
        master.getSystemState.return_value = ([], [], [])
        self.assertTrue(identity.changed(master))
        self.assertEqual(set(call[0] for call in master.method_calls), {'getSystemState', 'lookupNode'})

    def test_live_loop_started_aloft_prints_xy_then_disappearance_without_registration(self):
        self.check_live_loop(window=False)

    def test_live_loss_of_circle_confirmation_clears_xy_with_red_still_visible(self):
        self.check_live_loop(window=False, lose_confirmation=True)

    def test_gtk_visible_property_unsupported_still_displays_and_publishes_until_q(self):
        self.check_live_loop(window=True)

    def test_window_only_updates_for_processed_frames_without_image_subscribers(self):
        self.check_live_loop(window=True, image_subscribed=False)

    def test_headless_without_image_subscribers_skips_rendering_but_keeps_status(self):
        self.check_live_loop(window=False, image_subscribed=False)

    def check_live_loop(self, window, image_subscribed=True, lose_confirmation=False):
        callbacks, publications, subscriptions = {}, {}, []
        initial = np.full((720, 1280, 3), 230, np.uint8)
        cv2.circle(initial, (603, 364), 140, (0, 160, 0), -1)
        cv2.circle(initial, (603, 364), 70, (230, 230, 230), -1)
        cv2.circle(initial, (603, 364), 8, (0, 0, 220), -1)
        unconfirmed = np.full_like(initial, 230)
        cv2.circle(unconfirmed, (603, 364), 8, (0, 0, 220), -1)

        def image(stamp, pixels):
            return NS(header=header(stamp, 'down_camera_optical'), width=1280, height=720, pixels=pixels)

        def subscribe(topic, _kind, callback, **_kw):
            callbacks[topic] = callback
            subscription = mock.Mock()
            subscriptions.append(subscription)
            if topic.endswith('planning_odometry'):
                callback(odometry(1., 1.))
            if topic == '/down_camera/image_raw':
                callback(image(1., initial))
            return subscription

        def publisher(topic, _kind, **_kw):
            messages = publications.setdefault(topic, [])

            def publish(message):
                messages.append(message)
                if topic.endswith('/status') and len(messages) == 1:
                    callbacks['/airseafly/native/planning_odometry'](odometry(1.1, 1.2))
                    callbacks['/down_camera/image_raw'](image(1.1, unconfirmed if lose_confirmation else initial))
                if topic.endswith('/status') and len(messages) == 2:
                    callbacks['/airseafly/native/planning_odometry'](odometry(2., 2.))
                    callbacks['/down_camera/image_raw'](image(2., np.full_like(initial, 230)))
            return NS(publish=publish, get_num_connections=lambda: int(image_subscribed))

        ros = NS(init_node=mock.Mock(), get_name=lambda: '/preview', get_time=lambda: 10.,
                 Publisher=publisher, Subscriber=subscribe, is_shutdown=lambda: False,
                 logwarn_throttle=mock.Mock())
        master = mock.Mock()
        master.getSystemState.return_value = ([('/down_camera/image_raw', ['/camera'])], [], [])
        bridge = NS(imgmsg_to_cv2=lambda message, **kw: message.pixels,
                    cv2_to_imgmsg=mock.Mock(side_effect=lambda pixels, **kw: NS(pixels=pixels)))
        modules = {'rospy': ros, 'rosgraph': NS(Master=lambda name: master, MasterError=ValueError, MasterFailure=RuntimeError),
            'roslaunch': NS(), 'cv_bridge': NS(CvBridge=lambda: bridge, CvBridgeError=RuntimeError),
            'geometry_msgs.msg': NS(TransformStamped=object), 'nav_msgs.msg': NS(Odometry=object),
            'sensor_msgs.msg': NS(Image=object), 'std_msgs.msg': NS(String=lambda **kw: NS(**kw))}
        keyboard = mock.MagicMock()
        keyboard.__enter__.return_value.read.side_effect = ['', '', '', '', 'q']
        output = io.StringIO()
        with mock.patch.dict(sys.modules, modules), mock.patch.object(preview, 'TerminalKeys', return_value=keyboard), \
                mock.patch.object(preview, 'open_window', return_value=window), \
                mock.patch.object(cv2, 'getWindowProperty', side_effect=lambda name, prop:
                                  0. if prop == cv2.WND_PROP_AUTOSIZE else -1.), \
                mock.patch.object(cv2, 'waitKey', return_value=-1), \
                mock.patch.object(cv2, 'imshow') as render, mock.patch.object(cv2, 'destroyWindow'), \
                mock.patch.object(preview, 'annotate', wraps=preview.annotate) as annotate, \
                mock.patch.object(preview.time, 'monotonic', return_value=100.), contextlib.redirect_stdout(output):
            self.assertEqual(preview.live(NS(no_window=not window), self.config), 0)
        self.assertEqual(set(publications), {'/airseafly/red_point/status', '/airseafly/red_point/image_annotated'})
        status = [json.loads(message.data) for message in publications['/airseafly/red_point/status']]
        self.assertEqual(len(status), 3)
        self.assertEqual(len(status[0]['candidates']), 1)
        self.assertIsNotNone(status[0]['candidates'][0]['xy_m'])
        self.assertAlmostEqual(status[0]['camera_height_m'], 1.015)
        self.assertEqual(len(status[1]['candidates']), 1)  # Still published while text is throttled.
        self.assertEqual(status[1]['confirmed_count'], 0 if lose_confirmation else 1)
        self.assertEqual(status[1]['candidates'][0]['xy_m'] is None, lose_confirmation)
        self.assertAlmostEqual(status[1]['camera_height_m'], 1.215)
        self.assertEqual(status[2]['candidates'], [])
        self.assertAlmostEqual(status[2]['camera_height_m'], 2.015)
        self.assertEqual(status[2]['ground_reference']['z_m'], 0.)
        self.assertEqual(status[2]['ground_reference']['source'], 'localization_origin')
        text = output.getvalue()
        for expected in ('像素=(603.0, 364.0)', '目标XY=', '相对飞机ΔXY=', '高度=1.015m', 'Δt=+0.0ms', '未检测到红点'):
            self.assertIn(expected, text)
        self.assertNotIn('按 g', text)
        self.assertEqual(text.count('图像t='), 3 if lose_confirmation else 2)
        if lose_confirmation:
            self.assertIn('未确认：缺少可拟合的绿环外缘', text)
            self.assertEqual(text.count('目标XY='), 1)  # Never repeat the last valid target.
        self.assertEqual([message.header.stamp.to_sec() for message in publications['/airseafly/red_point/image_annotated']],
                         [1., 1.1, 2.] if image_subscribed else [])
        self.assertEqual(bridge.cv2_to_imgmsg.call_count, 3 if image_subscribed else 0)
        self.assertEqual(render.call_count, 3 if window else 0)
        self.assertEqual(annotate.call_count, 3 if window or image_subscribed else 0)
        self.assertTrue(all(message['camera_mode'] == 'reused' for message in status))
        for sub in subscriptions:
            sub.unregister.assert_called_once_with()


class WindowTests(unittest.TestCase):
    def test_destroyed_window_and_backend_missing_window_error_are_closed(self):
        for response in (-1., cv2.error('window no longer exists')):
            with self.subTest(response=response), mock.patch.object(cv2, 'getWindowProperty',
                    side_effect=response if isinstance(response, Exception) else None,
                    return_value=response):
                self.assertFalse(preview.window_is_open())

    def test_offline_gtk_window_reaches_key_processing(self):
        config = preview.load_config(preview.DEFAULT_CONFIG)
        args = NS(image=Path(__file__).parent/'fixtures/red_point_target.jpg',
                  output=None, status_json=None, no_window=False)
        with mock.patch.object(preview, 'open_window', return_value=True), \
                mock.patch.object(cv2, 'getWindowProperty', side_effect=lambda name, prop:
                                  0. if prop == cv2.WND_PROP_AUTOSIZE else -1.), \
                mock.patch.object(cv2, 'waitKey', return_value=ord('q')) as key, \
                mock.patch.object(cv2, 'imshow'), mock.patch.object(cv2, 'destroyWindow'), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(preview.offline(args, config), 0)
        key.assert_called_once()


class ConsoleTests(unittest.TestCase):
    def setUp(self):
        config = preview.load_config(preview.DEFAULT_CONFIG)
        config['projection'].pop('intrinsics', None)  # DF200 pixels below are the reference
        self.session = RedPointSession(config)
        self.session.observe_pose(1., [1., 2., 1.], [0., 0., 0., 1.], 'fast_lio_local_enu')

    def status(self, count=1):
        points = [dict(pixel=[602.74071+index*20, 363.67025], target_confirmed=True,
                       confirmation_reason='confirmed', center_error_px=1., center_tolerance_px=3.)
                  for index in range(count)]
        return self.session.describe(points, (1280, 720), 1., 0.)

    def test_five_hz_only_throttles_repeated_terminal_text(self):
        reporter = preview.ConsoleReporter(5.)
        status = self.status()
        self.assertIsNotNone(reporter.observe(status, 1.))
        self.assertIsNone(reporter.observe(status, 1.05))
        self.assertIsNone(reporter.observe(status, 1.19))
        self.assertIsNotNone(reporter.observe(status, 1.2))
        self.assertIsNone(reporter.observe(status, 1.21))
        self.assertIsNotNone(reporter.observe(status, 1.41))
        slow = preview.ConsoleReporter(2.)
        self.assertIsNotNone(slow.observe(status, 1.))
        self.assertIsNone(slow.observe(status, 1.3))
        self.assertIsNotNone(slow.observe(status, 1.5))

    def test_appear_disappear_and_candidate_count_changes_print_immediately(self):
        reporter = preview.ConsoleReporter(5.)
        self.assertIn('未检测到红点', reporter.observe(self.status(0), 1.))
        self.assertIn('红点 C1', reporter.observe(self.status(1), 1.01))
        self.assertIn('红点 C2', reporter.observe(self.status(2), 1.02))
        self.assertIn('红点 C1', reporter.observe(self.status(1), 1.03))
        disappeared = reporter.observe(self.status(0), 1.04)
        self.assertIn('未检测到红点', disappeared)
        self.assertNotIn('目标XY=', disappeared)
        self.assertIsNone(reporter.observe(self.status(0), 1.05))

    def test_all_candidates_include_pixels_absolute_xy_and_world_offsets(self):
        status = self.status(2)
        text = preview.ConsoleReporter(5.).observe(status, 1.)
        for expected in ('坐标系=fast_lio_local_enu', '高度=1.015m', 'Δt=+0.0ms',
                         '像素=(602.7, 363.7)', '目标XY=(1.000, 2.000)m',
                         '相对飞机ΔXY=(+0.000, +0.000)m', '红点 C2',
                         '飞机XYZ(图像时刻)=(1.000, 2.000, 1.000)m', '检测耗时=',
                         '共心误差=1.00px / 容许=3.00px', '已确认'):
            self.assertIn(expected, text)
        self.assertEqual(text.count('目标XY='), 2)
        for c in status['candidates']:
            np.testing.assert_allclose(np.asarray(c['xy_m'])-[1., 2.], c['offset_xy_m'], atol=1e-10)

    def test_confirmation_changes_print_immediately_without_coordinate_spam(self):
        reporter = preview.ConsoleReporter(5.)
        self.assertIn('已确认', reporter.observe(self.status(), 1.))
        point = dict(pixel=[600., 360.], target_confirmed=False, confirmation_reason='not_concentric',
                     center_error_px=8., center_tolerance_px=3.)
        status = self.session.describe([point], (1280, 720), 1., 0.)
        text = reporter.observe(status, 1.01)
        self.assertIn('未确认：三个中心偏差过大', text)
        self.assertIn('像素=(600.0, 360.0)', text)
        self.assertIn('共心误差=8.00px / 容许=3.00px', text)
        self.assertNotIn('目标XY=', text)
        self.assertNotIn('相对飞机ΔXY=', text)
        # Numeric motion inside the same rejection state respects console.rate_hz.
        status['candidates'][0]['center_error_px'] = 9.
        self.assertIsNone(reporter.observe(status, 1.02))
        status['candidates'][0]['confirmation_reason'] = 'missing_green_outer'
        self.assertIn('缺少可拟合的绿环外缘', reporter.observe(status, 1.03))
        self.assertIn('已确认', reporter.observe(self.status(), 1.04))

    def test_candidate_reordering_does_not_bypass_console_rate(self):
        reporter = preview.ConsoleReporter(5.)
        status = self.status(2)
        status['candidates'][0].update(target_confirmed=False, confirmation_reason='missing_white_inner')
        self.assertIsNotNone(reporter.observe(status, 1.))
        status['candidates'].reverse()
        self.assertIsNone(reporter.observe(status, 1.01))

    def test_missing_pose_and_new_session_report_without_registration(self):
        reporter = preview.ConsoleReporter(5.)
        reporter.observe(self.status(), 1.)
        self.session.reset_localization('localization_producer_restarted')
        text = reporter.observe(self.status(), 1.01)
        self.assertIn('目标XY=不可计算', text)
        self.assertIn('无定位', text)
        self.assertNotIn('按 g', text)
        self.session.observe_pose(2., [1., 2., 1.], [0., 0., 0., 1.], 'fast_lio_local_enu')
        # Nearest sample is one second away: displayed, never an age gate.
        text = reporter.observe(self.status(), 1.02)
        self.assertIn('目标XY=(1.000, 2.000)m', text)
        self.assertIn('Δt=+1000.0ms', text)


class ShellTests(unittest.TestCase):
    def shell(self, *args):
        return subprocess.run(['sh', str(ROOT/'flight/red_point.sh'), *map(str, args)],
            cwd='/tmp', env=dict(os.environ, AIRSEAFLY_PYTHON=sys.executable),
            capture_output=True, text=True, timeout=20)

    def test_dry_run_from_other_directory_needs_no_ros_or_camera(self):
        run = self.shell('--dry-run')
        self.assertEqual(run.returncode, 0, run.stderr)
        value = json.loads(run.stdout)
        self.assertFalse(value['flight_commands'])
        self.assertFalse(value['payload_commands'])
        self.assertFalse(value['persistent_ground_reference'])
        self.assertEqual(value['config']['projection']['camera_height_at_reference_m'], .015)
        self.assertEqual(value['config']['topics']['odometry'], '/airseafly/native/planning_odometry')
        self.assertEqual(value['config']['console']['rate_hz'], 5.)
        self.assertEqual(value['height_reference'], dict(source='localization_origin', z_m=0.))
        self.assertEqual(value['height_formula'], 'planning_odometry.z + camera_height_at_reference_m')

    def test_console_frequency_is_configurable_without_changing_projection(self):
        config = preview.load_config(preview.DEFAULT_CONFIG)
        config['console']['rate_hz'] = 2.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'red_point.yaml'
            path.write_text(yaml.safe_dump(config), encoding='utf-8')
            run = self.shell('--config', path, '--dry-run')
            self.assertEqual(run.returncode, 0, run.stderr)
            value = json.loads(run.stdout)
            self.assertEqual(value['config']['console']['rate_hz'], 2.)
            self.assertEqual(value['height_reference']['z_m'], 0.)
            for invalid in (0, -1, float('nan'), float('inf')):
                config['console']['rate_hz'] = invalid
                path.write_text(yaml.safe_dump(config), encoding='utf-8')
                with self.assertRaisesRegex(ValueError, 'console.rate_hz'):
                    preview.load_config(path)

    def test_invalid_circle_configuration_fails_without_ros(self):
        cases = dict(min_contour_points=4, min_arc_degrees=361., max_radial_residual=float('nan'),
                     concentric_tolerance_px=0., concentric_tolerance_ratio=-.1,
                     white_green_radius_ratio=[.7, .3], white_range=[[0, 0, 80], [180, 70, 255]])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'red_point.yaml'
            for key, invalid in cases.items():
                with self.subTest(key=key):
                    config = preview.load_config(preview.DEFAULT_CONFIG)
                    config['detector'][key] = invalid
                    path.write_text(yaml.safe_dump(config), encoding='utf-8')
                    with self.assertRaises(ValueError):
                        preview.load_config(path)

    def test_offline_photo_writes_only_requested_artifacts(self):
        photo = Path(__file__).parent/'fixtures/red_point_target.jpg'
        with tempfile.TemporaryDirectory(prefix='red point ') as directory:
            output, report = Path(directory)/'annotated.jpg', Path(directory)/'status.json'
            run = self.shell('--image', photo, '--no-window', '--output', output, '--status-json', report)
            self.assertEqual(run.returncode, 0, run.stderr)
            status = json.loads(run.stdout)
            self.assertEqual(json.loads(report.read_text()), status)
            self.assertEqual(sorted(path.name for path in Path(directory).iterdir()), ['annotated.jpg', 'status.json'])
            self.assertIsNotNone(cv2.imread(str(output)))
            self.assertIsNone(status['candidates'][0]['xy_m'])
            self.assertEqual(status['confirmed_count'], 1)
            self.assertTrue(status['candidates'][0]['target_confirmed'])
            np.testing.assert_allclose(status['candidates'][0]['pixel'], [458, 682], atol=1.)

    def test_help_and_missing_image_are_ros_free(self):
        self.assertEqual(self.shell('--help').returncode, 0)
        run = self.shell('--image=/no/such/image.jpg', '--no-window')
        self.assertEqual(run.returncode, 1)
        self.assertIn('无法读取图片', run.stderr)
        self.assertNotIn('rospy', run.stderr)


if __name__ == '__main__':
    unittest.main()
