"""Coordinate and binary-cloud replays; no ROS master or flight commands."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import math
import struct
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from airseafly_real400_mission.native_motion import FrozenLioFrame, quaternion, rotation
from airseafly_real400_mission.native_reference import LIO_FRAME
from airseafly_real400_mission.native_planning_geometry import (
    RawToLocal, planning_odometry, transform_observation_packet, transform_registered_cloud)


def xyz(values=(0., 0., 0.)):
    return NS(x=values[0], y=values[1], z=values[2])


def odom(position=(0., 0., 0.), q=(0., 0., 0., 1.), v=(0., 0., 0.), w=(0., 0., 0.)):
    return NS(header=NS(stamp=NS(to_sec=lambda: 12.), frame_id='native_odom', seq=27),
              child_frame_id='native_base',
              pose=NS(pose=NS(position=xyz(position), orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3])),
                      covariance=[0.]*36),
              twist=NS(twist=NS(linear=xyz(v), angular=xyz(w)), covariance=list(np.diag([1., 2., 3., 4., 5., 6.]).flat)))


def transform_message(transform):
    q = quaternion(transform.rotation)
    return NS(header=NS(frame_id=LIO_FRAME), child_frame_id=transform.source_frame,
              transform=NS(translation=xyz(transform.translation), rotation=NS(x=q[0], y=q[1], z=q[2], w=q[3])))


class CoordinateTests(unittest.TestCase):
    def test_tilt_mount_and_rotating_lever_recover_body_world_velocity(self):
        r_nw = rotation((.18, -.09, .26, .94))
        r_bi = rotation((.04, .03, -.08, .99))
        r_nb = rotation((.03, -.02, .21, .98))
        t_bi, t_nw = np.array([.12, -.06, .035]), np.array([4., -2., 1.])
        p_nb, v_nb, omega_nb = np.array([2., 3., .7]), np.array([.4, -.2, .1]), np.array([.1, -.2, 1.])
        r_wi = r_nw.T @ r_nb @ r_bi
        lever_w = r_nw.T @ r_nb @ t_bi
        p_wi = r_nw.T @ (p_nb-t_nw) + lever_w
        v_wi = r_nw.T @ v_nb + np.cross(r_nw.T @ omega_nb, lever_w)
        omega_i = r_wi.T @ r_nw.T @ omega_nb
        frame = FrozenLioFrame(quaternion(r_bi), t_bi)
        frame.initialize(p_wi, quaternion(r_wi), p_nb, quaternion(r_nb))
        body = frame.transform(p_wi, quaternion(r_wi), v_wi, omega_i)
        original = odom(*body)
        planned = planning_odometry(original)
        np.testing.assert_allclose(tuple(vars(planned.pose.pose.position).values()), p_nb, atol=1e-12)
        np.testing.assert_allclose(tuple(vars(planned.twist.twist.linear).values()), v_nb, atol=1e-12)
        np.testing.assert_allclose(tuple(vars(planned.twist.twist.angular).values()), omega_nb, atol=1e-12)
        self.assertEqual(planned.header.frame_id, LIO_FRAME)
        self.assertEqual(planned.header.stamp.to_sec(), 12.)
        self.assertEqual(original.header.frame_id, 'native_odom')
        np.testing.assert_allclose(tuple(vars(original.twist.twist.linear).values()), r_nb.T @ v_nb)
        tf = RawToLocal.from_frozen_frame(frame)
        np.testing.assert_allclose(tf.rotation, r_nw, atol=1e-12)
        np.testing.assert_allclose(tf.translation, t_nw, atol=1e-12)

    def test_ninety_degree_yaw_rotates_velocity_and_covariance(self):
        message = odom(q=(0., 0., math.sqrt(.5), math.sqrt(.5)), v=(1., 0., 0.))
        planned = planning_odometry(message)
        np.testing.assert_allclose(tuple(vars(planned.twist.twist.linear).values()), (0., 1., 0.), atol=1e-12)
        np.testing.assert_allclose(np.diag(np.asarray(planned.twist.covariance).reshape(6, 6)), (2., 1., 3., 5., 4., 6.))

    def test_front_environment_target_is_converted_once_without_lever_correction(self):
        transform = RawToLocal((0., 0., math.sqrt(.5), math.sqrt(.5)), (10., -3., 4.))
        packet = dict(frame_id='world', stamp_s=31.5, pose_stamp_s=31.51,
                      observations=[dict(position_xyz=[2., 1., 3.], detection_index=7, class_name='bunker')],
                      detections=[dict(class_name='bunker')])
        result = transform_observation_packet(packet, transform)
        np.testing.assert_allclose(result['observations'][0]['position_xyz'], (9., -1., 7.))
        self.assertEqual(result['observations'][0]['raw_position_xyz'], [2., 1., 3.])
        self.assertEqual(result['raw_frame_id'], 'world')
        self.assertEqual(result['frame_id'], LIO_FRAME)
        self.assertEqual(result['stamp_s'], 31.5)
        self.assertEqual(result['pose_stamp_s'], 31.51)
        self.assertEqual(transform_observation_packet(result, transform), result)
        self.assertEqual(packet['observations'][0]['position_xyz'], [2., 1., 3.])
        self.assertNotIn('raw_frame_id', packet)

    def test_alias_and_fixed_transform_round_trip(self):
        transform = RawToLocal((.1, -.2, .3, .9), (1., 2., 3.))
        reconstructed = RawToLocal.from_message(transform_message(transform))
        np.testing.assert_allclose(reconstructed.point((4., 5., 6.)), transform.point((4., 5., 6.)))
        self.assertTrue(reconstructed.accepts('camera_init'))
        with self.assertRaises(ValueError):
            transform_observation_packet(dict(frame_id='px4_local_enu'), transform)
        with self.assertRaises(ValueError):
            transform.translation[0] = 5.


def cloud(endian='<', kind='f'):
    size = struct.calcsize(kind)
    point_step, row_step = 3*size+8, 2*(3*size+8)+9
    data = bytearray(b'\xa5'*(row_step*2))
    points = [(1., 2., 3.), (4., 5., 6.), (float('nan'), 7., 8.), (-2., -3., -4.)]
    fields = [NS(name=name, offset=i*size, datatype=7 if kind == 'f' else 8, count=1)
              for i, name in enumerate(('x', 'y', 'z'))]
    fields += [NS(name='intensity', offset=3*size, datatype=4, count=1),
               NS(name='ring', offset=3*size+2, datatype=2, count=1),
               NS(name='time', offset=3*size+4, datatype=6, count=1)]
    for index, point in enumerate(points):
        offset = (index//2)*row_step+(index % 2)*point_step
        struct.pack_into(endian+kind*3, data, offset, *point)
        struct.pack_into(endian+'HBxI', data, offset+3*size, 107+index, 2+index, 123456+index)
    return NS(header=NS(frame_id='camera_init', stamp=123.456, seq=17),
              height=2, width=2, fields=fields, is_bigendian=endian == '>',
              point_step=point_step, row_step=row_step, data=bytes(data), is_dense=False)


class CloudTests(unittest.TestCase):
    def test_xyz_only_transform_preserves_fields_padding_stamp_and_nan_bytes(self):
        transform = RawToLocal((0., 0., math.sqrt(.5), math.sqrt(.5)), (10., -3., 4.))
        for endian in ('<', '>'):
            for kind in ('f', 'd'):
                with self.subTest(endian=endian, kind=kind):
                    before = cloud(endian, kind)
                    result = transform_registered_cloud(before, transform)
                    size = struct.calcsize(kind)
                    for index in range(4):
                        offset = index//2*before.row_step+index % 2*before.point_step
                        original = struct.unpack_from(endian+kind*3, before.data, offset)
                        actual = struct.unpack_from(endian+kind*3, result.data, offset)
                        if index == 2:
                            self.assertEqual(result.data[offset:offset+before.point_step], before.data[offset:offset+before.point_step])
                        else:
                            np.testing.assert_allclose(actual, transform.point(original))
                        self.assertEqual(result.data[offset+3*size:offset+before.point_step],
                                         before.data[offset+3*size:offset+before.point_step])
                    for row in range(2):
                        start = row*before.row_step+before.width*before.point_step
                        self.assertEqual(result.data[start:(row+1)*before.row_step], before.data[start:(row+1)*before.row_step])
                    self.assertEqual(result.header.stamp, 123.456)
                    self.assertEqual(result.header.seq, 17)
                    self.assertEqual(result.fields, before.fields)
                    self.assertEqual(result.header.frame_id, LIO_FRAME)
                    self.assertEqual(before.header.frame_id, 'camera_init')
                    self.assertEqual(transform_registered_cloud(result, transform).data, result.data)

    def test_stationary_environment_point_does_not_follow_vehicle_rotation(self):
        frame = FrozenLioFrame((0., 0., 0., 1.), (.2, 0., 0.))
        frame.initialize((.2, 0., 0.), (0., 0., 0., 1.), (0., 0., 0.), (0., 0., 0., 1.))
        transform = RawToLocal.from_frozen_frame(frame)
        expected = transform_registered_cloud(cloud(), transform).data
        for angle in (0., math.pi/2., math.pi):
            # Rotating around the body origin changes the IMU position but
            # cannot change the frozen transform of an observed wall.
            frame.transform((.2*math.cos(angle), .2*math.sin(angle), 0.),
                            (0., 0., math.sin(angle/2.), math.cos(angle/2.)),
                            (0., 0., 0.), (0., 0., 1.))
            self.assertEqual(transform_registered_cloud(cloud(), RawToLocal.from_frozen_frame(frame)).data, expected)

    def test_malformed_and_empty_cloud(self):
        transform = RawToLocal((0., 0., 0., 1.), (0., 0., 0.))
        for mutate in (lambda c: setattr(c, 'data', b''),
                       lambda c: setattr(c.fields[0], 'datatype', 4),
                       lambda c: setattr(c.fields[1], 'offset', 0)):
            message = cloud()
            mutate(message)
            with self.assertRaises(ValueError):
                transform_registered_cloud(message, transform)
        message = cloud()
        message.width, message.height, message.row_step, message.data = 0, 0, 0, b''
        self.assertEqual(transform_registered_cloud(message, transform).data, b'')


class BridgeWiringTests(unittest.TestCase):
    def test_point_cloud_worker_is_separate_and_opt_in(self):
        root = ET.parse(PACKAGE/'launch/native_localization.launch').getroot()
        arg = next(a for a in root.findall('arg') if a.attrib['name'] == 'publish_planning_interfaces')
        self.assertEqual(arg.attrib['default'], 'false')
        cloud_node = next(n for n in root.findall('node') if n.attrib['type'] == 'native_planning_cloud.py')
        self.assertEqual(cloud_node.attrib['if'], '$(arg publish_planning_interfaces)')
        source = (PACKAGE/'scripts/native_lio_bridge.py').read_text()
        self.assertNotIn('PointCloud2', source)

    def test_old_odometry_unchanged_and_frozen_transform_published_once(self):
        sent = {}
        class Publisher:
            def __init__(self, topic, _type, **kwargs):
                self.topic = topic
                sent[topic] = dict(messages=[], kwargs=kwargs)
            def publish(self, value):
                sent[self.topic]['messages'].append(deepcopy(value))
        def stamped():
            return NS(header=NS(), transform=NS(translation=xyz(), rotation=NS(x=0., y=0., z=0., w=1.)))
        params = {'~mount/quaternion_xyzw': (0., 0., 0., 1.),
                  '~mount/translation_xyz_m': (.2, 0., 0.), '~publish_planning_interfaces': True}
        ros = NS(get_param=lambda key, default=None: params.get(key, default), Publisher=Publisher,
                 Subscriber=lambda *a, **k: None, on_shutdown=lambda f: None,
                 Time=NS(now=lambda: 20.), Duration=lambda t: t, Timer=lambda *a: NS(shutdown=lambda: None),
                 get_time=lambda: 12., is_shutdown=lambda: False, logwarn_throttle=lambda *a: None,
                 ROSException=type('ROSException', (Exception,), {}))
        stubs = {'rospy': ros, 'tf2_ros': NS(StaticTransformBroadcaster=lambda: NS(sendTransform=lambda x: None)),
                 'geometry_msgs.msg': NS(TransformStamped=stamped, PoseStamped=object),
                 'sensor_msgs.msg': NS(Imu=object), 'mavros_msgs.msg': NS(EstimatorStatus=object),
                 'nav_msgs.msg': NS(Odometry=odom), 'std_msgs.msg': NS(Bool=NS, String=NS)}
        spec = importlib.util.spec_from_file_location('test_planning_bridge', PACKAGE/'scripts/native_lio_bridge.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, stubs):
            spec.loader.exec_module(module)
        bridge = module.Bridge()
        bridge.imu = NS(orientation=NS(x=0., y=0., z=0., w=1.))
        sample = odom(position=(.2, 0., 0.), v=(0., .4, 0.), w=(0., 0., 2.))
        sample.header.frame_id = 'world'
        bridge.on_odom(sample)
        bridge.tick(None)
        bridge.on_odom(sample)
        bridge.tick(None)
        frames = sent['/airseafly/native/raw_to_local']
        self.assertTrue(frames['kwargs']['latch'])
        self.assertEqual(len(frames['messages']), 1)
        self.assertEqual(frames['messages'][0].header.frame_id, LIO_FRAME)
        self.assertEqual(frames['messages'][0].child_frame_id, 'world')
        mavros = sent['/mavros/odometry/out']['messages'][-1]
        native = sent['/airseafly/native/lio_odometry']['messages'][-1]
        planner = sent['/airseafly/native/planning_odometry']['messages'][-1]
        self.assertEqual(mavros.header.frame_id, 'native_odom')
        self.assertEqual(native.header.frame_id, 'native_odom')
        self.assertEqual(planner.header.frame_id, LIO_FRAME)
        np.testing.assert_allclose(tuple(vars(planner.twist.twist.linear).values()), (0., 0., 0.), atol=1e-12)
        self.assertEqual(planner.header.stamp.to_sec(), sample.header.stamp.to_sec())
        bridge.shutdown()
        params['~publish_planning_interfaces'] = False
        sent.clear()
        legacy = module.Bridge()
        self.assertNotIn('/airseafly/native/planning_odometry', sent)
        self.assertNotIn('/airseafly/native/raw_to_local', sent)
        legacy.shutdown()


class CloudWorkerTests(unittest.TestCase):
    def test_slow_cloud_transform_keeps_latest_pending_scan_and_same_frame(self):
        sent, errors = [], []
        delivered, entered, release = threading.Event(), threading.Event(), threading.Event()
        def publish(message):
            sent.append(message)
            if len(sent) == 2:
                delivered.set()
        # Nobody subscribes to the memory topic: only the converted scan is sent.
        ros = NS(Publisher=lambda *a, **k: NS(publish=publish, get_num_connections=lambda: 0),
                 Subscriber=lambda *a, **k: None,
                 on_shutdown=lambda f: None, is_shutdown=lambda: False,
                 logerr=lambda *args: errors.append(args), logwarn_throttle=lambda *args: None,
                 ROSException=type('ROSException', (Exception,), {}))
        spec = importlib.util.spec_from_file_location('test_planning_cloud_worker', PACKAGE/'scripts/native_planning_cloud.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'rospy': ros, 'geometry_msgs.msg': NS(TransformStamped=object),
                                      'nav_msgs.msg': NS(Odometry=object),
                                      'sensor_msgs.msg': NS(PointCloud2=object, PointField=object)}):
            spec.loader.exec_module(module)
        transform = RawToLocal((0., 0., 0., 1.), (1., 2., 3.))
        def slow_transform(message, frame):
            if message.header.seq == 1:
                entered.set()
                if not release.wait(2.):
                    raise ValueError('test worker did not receive release')
            return transform_registered_cloud(message, frame)
        with patch.object(module, 'transform_registered_cloud', slow_transform):
            node = module.PlanningCloud()
            try:
                node.on_transform(transform_message(transform))
                # A duplicate latch is harmless; a new frame cannot move an
                # existing map while the bridge session is still running.
                node.on_transform(transform_message(transform))
                node.on_transform(transform_message(RawToLocal((0., 0., 0., 1.), (100., 2., 3.))))
                self.assertEqual(len(errors), 1)
                for seq in (1, 2, 3):
                    message = cloud()
                    message.header.seq = seq
                    node.on_cloud(message)
                    if seq == 1:
                        self.assertTrue(entered.wait(2.))
                release.set()
                self.assertTrue(delivered.wait(2.))
                self.assertEqual([m.header.seq for m in sent], [1, 3])
                for message in sent:
                    self.assertEqual(message.data, transform_registered_cloud(cloud(), transform).data)
            finally:
                release.set()
                node.shutdown()
                node.worker.join(2.)
            self.assertFalse(node.worker.is_alive())


if __name__ == '__main__':
    unittest.main()
