"""Shared observation-only camera, localization and image processing boundary.

Imported by both the standalone UI and the headless mission worker. This module
has no terminal, HighGUI, flight-control or actuator operations.
"""
from pathlib import Path
import threading

import cv2
import numpy as np
import yaml

from .red_point import RedPointSession, detect_targets as detect
from .green_circle import DEFAULTS as GREEN_DEFAULTS
from .landing_pad import validate_settings as validate_landing_pad


PROJECT = Path(__file__).resolve().parents[4]
DEFAULT_CONFIG = PROJECT/'flight/red_point.yaml'
CAMERA_LAUNCH = PROJECT/'src/airseafly_real400_mission/launch/red_point_camera.launch'
# Configs without camera format keys use the current HD USB Camera.
CAMERA_DEFAULTS = dict(image_width=1280, image_height=720, pixel_format='yuyv', framerate=9,
                       calibration='config/aircraft/down_hd_1280x720_yuyv_calib.yaml')


def load_calibration(path):
    """ROS camera_info YAML -> intrinsics consumed by down_projection."""
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    matrix = np.asarray(data['camera_matrix']['data'], dtype=float).reshape(3, 3)
    distortion = [float(v) for v in data['distortion_coefficients']['data']]
    size = [float(data['image_width']), float(data['image_height'])]
    if (not np.all(np.isfinite(matrix)) or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or min(size) <= 0 or len(distortion) not in (4, 5, 8)):
        raise ValueError('invalid camera calibration %s' % path)
    return dict(camera_matrix=matrix.tolist(), distortion=distortion, calibration_size=size,
                camera_name=str(data.get('camera_name', 'down_camera')))


def load_config(path):
    with open(path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    # Validate computability/configuration only. None of these are flight gates.
    detector = config['detector']
    for low, high in list(detector['red_ranges']) + [detector['green_range'], detector['white_range']]:
        if (len(low) != 3 or len(high) != 3 or
                any(not 0 <= a <= b <= maximum for a, b, maximum in zip(low, high, (179, 255, 255)))):
            raise ValueError('invalid HSV range')
    if (detector['min_area_px'] < 1 or detector['max_aspect_ratio'] < 1 or
            not 0 <= detector['min_circularity'] <= 1 or not 0 <= detector['min_solidity'] <= 1):
        raise ValueError('invalid detector settings')
    points = detector['min_contour_points']
    if not isinstance(points, int) or isinstance(points, bool) or points < 5:
        raise ValueError('min_contour_points must be an integer >= 5')
    for key in ('min_arc_degrees', 'max_radial_residual', 'concentric_tolerance_px',
                'concentric_tolerance_ratio'):
        value = detector[key]
        if not np.isfinite(value) or value <= 0:
            raise ValueError(key+' must be finite and positive')
    if detector['min_arc_degrees'] > 360 or detector['max_radial_residual'] >= 1:
        raise ValueError('invalid circle fit settings')
    ratio = detector['white_green_radius_ratio']
    if len(ratio) != 2 or not 0 < ratio[0] <= ratio[1] < 1:
        raise ValueError('white_green_radius_ratio must satisfy 0 < min <= max < 1')
    green = dict(GREEN_DEFAULTS, **detector.get('green_hough', {}))
    for key, value in green.items():
        if isinstance(value, bool) or not np.isfinite(value) or value < 0:
            raise ValueError('invalid green_hough.'+key)
    if not (0 < green['scale'] <= 1 and green['dp'] >= 1 and green['min_dist_px'] > 0
            and green['param1'] > 0 and green['param2'] > 0
            and 0 < green['min_radius_px'] <= green['max_radius_px']
            and 0 < green['min_visible_degrees'] <= 360
            and 0 < green['min_ring_support'] <= 1 and green['max_center_green'] <= 1
            and green['dedup_radius_ratio'] > 0):
        raise ValueError('invalid green_hough settings')
    kernel = green['blur_kernel']
    if not isinstance(kernel, int) or kernel < 1 or kernel % 2 != 1:
        raise ValueError('green_hough.blur_kernel must be a positive odd integer')
    detector['green_hough'] = green
    p = config['projection']
    matrix = np.asarray(p['pixel_to_body'], dtype=float)
    if matrix.shape != (2, 2) or not np.all(np.isfinite(matrix)) or abs(np.linalg.det(matrix)) < 1e-9:
        raise ValueError('pixel_to_body must be a finite invertible 2x2 matrix')
    if not np.isfinite(p['camera_height_at_reference_m']) or p['camera_height_at_reference_m'] <= 0:
        raise ValueError('camera_height_at_reference_m must be positive')
    if not np.isfinite(p['body_yaw_offset_rad']):
        raise ValueError('body_yaw_offset_rad must be finite')
    if p['frame_id'] != 'fast_lio_local_enu':
        raise ValueError('this preview expects fast_lio_local_enu planning odometry')
    camera = config['camera']
    for key, value in CAMERA_DEFAULTS.items():
        camera.setdefault(key, value)
    if min(int(camera['image_width']), int(camera['image_height']), int(camera['framerate'])) <= 0:
        raise ValueError('camera image size and framerate must be positive')
    calibration = Path(camera['calibration']).expanduser()
    if not calibration.is_absolute():
        calibration = PROJECT/calibration  # the mission's frozen copy lives under data/
    camera['calibration'] = str(calibration)
    p['intrinsics'] = load_calibration(calibration)
    rate = float(config['console']['rate_hz'])
    if not np.isfinite(rate) or rate <= 0:
        raise ValueError('console.rate_hz must be finite and positive')
    config['console']['rate_hz'] = rate
    config['landing_pad'] = validate_landing_pad(config.get('landing_pad'))
    RedPointSession(config)
    return config


class CameraOwner:
    """Reuse a registered publisher; shutdown is limited to our launch parent."""
    def __init__(self, config, master, launch_factory, namespace):
        self.config, self.master = config, master
        self.launch_factory, self.namespace = launch_factory, namespace
        self.parent = None
        self.mode = 'subscribe_only'

    def start(self):
        publishers = dict(self.master.getSystemState()[0])
        if publishers.get(self.config['topics']['image']):
            self.mode = 'reused'
        elif self.config['camera']['auto_start']:
            c, topics = self.config['camera'], self.config['topics']
            intrinsics = self.config['projection']['intrinsics']
            arguments = ['owner_ns:='+self.namespace, 'image_topic:='+topics['image'],
                         'camera_info_topic:='+topics['camera_info'],
                         'video_device:='+c['video_device'],
                         'autoexposure:='+str(c['autoexposure']).lower(), 'exposure:='+str(c['exposure']),
                         'image_width:=%d' % int(c['image_width']),
                         'image_height:=%d' % int(c['image_height']),
                         'pixel_format:='+str(c['pixel_format']), 'framerate:=%d' % int(c['framerate']),
                         'camera_name:='+intrinsics['camera_name'],
                         'camera_info_url:=file://'+c['calibration']]
            # A private node namespace cannot replace an existing down_camera.
            self.parent = self.launch_factory(str(CAMERA_LAUNCH), arguments)
            self.parent.start()
            self.mode = 'started_here'
        return self.mode

    def close(self):
        parent, self.parent = self.parent, None
        if parent is not None:
            parent.shutdown()


class ProducerIdentity:
    """Read ROS registration URIs to notice independently restarted FAST-LIO.

    No age timeout or liveness prerequisite. A changed producer invalidates only
    the preview's old pose samples; the new frame still uses its own Z=0.
    """
    def __init__(self, topics):
        self.topics = set(topics)
        self.previous = None

    def changed(self, master):
        publishers = dict(master.getSystemState()[0])
        identity = tuple(sorted((topic, node, master.lookupNode(node))
            for topic in self.topics for node in publishers.get(topic, [])))
        changed = self.previous is not None and identity != self.previous
        self.previous = identity
        return changed


class PreviewInputs:
    def __init__(self, session):
        self.session, self.lock = session, threading.RLock()
        self.pending = None
        self.pose_error = ''

    def on_image(self, message):
        with self.lock:
            self.pending = message

    def take_image(self):
        with self.lock:
            value, self.pending = self.pending, None
            return value

    def on_odometry(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        with self.lock:
            try:
                self.session.observe_pose(message.header.stamp.to_sec(), [p.x, p.y, p.z],
                                          [q.x, q.y, q.z, q.w], message.header.frame_id)
                self.pose_error = ''
            except ValueError as exc:
                self.pose_error = str(exc)

    def on_frame(self, message):
        t, q = message.transform.translation, message.transform.rotation
        token = (message.header.stamp.to_sec(), message.header.frame_id, message.child_frame_id,
                 t.x, t.y, t.z, q.x, q.y, q.z, q.w)
        with self.lock:
            self.session.observe_frame(token)


def attach_ros(rospy, types, config, inputs):
    """The complete ROS I/O boundary: exactly two observation publishers."""
    topics = config['topics']
    publishers = (rospy.Publisher(topics['status'], types['String'], queue_size=1),
                  rospy.Publisher(topics['annotated'], types['Image'], queue_size=1))
    subscribers = [
        rospy.Subscriber(topics['localization_frame'], types['TransformStamped'], inputs.on_frame, queue_size=1),
        rospy.Subscriber(topics['odometry'], types['Odometry'], inputs.on_odometry,
                         queue_size=100, tcp_nodelay=True),
        rospy.Subscriber(topics['image'], types['Image'], inputs.on_image, queue_size=1, buff_size=2**24)]
    return publishers, subscribers


def process_image(message, bridge, config, inputs, camera_mode, processed_stamp,
                  detector=detect, errors=(Exception,)):
    """Describe exactly this source image, clearing candidates on image failure."""
    stamp = message.header.stamp.to_sec()
    try:
        image = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        candidates, elapsed = detector(image, config)
        error = ''
    except errors as exc:
        image = np.zeros((message.height or 720, message.width or 1280, 3), np.uint8)
        candidates, elapsed, error = [], 0., str(exc)
    with inputs.lock:
        status = inputs.session.describe(candidates, image.shape[1::-1], stamp, elapsed)
        status['odometry_error'] = inputs.pose_error
    status.update(image_frame_id=message.header.frame_id, image_error=error,
                  camera_mode=camera_mode, processed_stamp_s=processed_stamp)
    return image, status
