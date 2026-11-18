"""Headless red-point observations for one mission session.

The worker owns only its camera launch and observation topics. Mission progress,
recognition fallback and actuator decisions remain in the mission engine.
"""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import time
import uuid

from .landing_pad import LandingPadDetector
from .red_point import RedPointSession, annotate, detect_targets as detect
from .red_point_runtime import (
    CameraOwner, ProducerIdentity, PreviewInputs, attach_ros, load_config,
    process_image,
)


def session_topics(session_id):
    namespace = '/airseafly/mission/red_point_' + str(uuid.UUID(session_id)).replace('-', '_')
    return dict(status=namespace+'/status', annotated=namespace+'/image_annotated')


TARGETS = ('red_point', 'landing_pad')


class MissionObservations:
    """Consume each source image once; localization changes never replay it.

    A warm red-point worker processes only the current image window. An H
    window keeps its own worker and replaces the red/green detector.
    """
    def __init__(self, config, session_id, landing=None):
        self.config, self.session_id = config, str(uuid.UUID(session_id))
        self.inputs = PreviewInputs(RedPointSession(config))
        self.image_sequence = 0
        self.seen_images = set()
        self.landing = landing

    def detect(self, image, config):
        if self.landing is None:
            return detect(image, config)
        start = time.perf_counter()
        return self.landing.detect(image), (time.perf_counter()-start)*1000

    def error_status(self, error, camera_mode, processed_stamp, image_sequence=0):
        with self.inputs.lock:
            status = self.inputs.session.describe([], (1280, 720), 0., 0.)
            status['odometry_error'] = self.inputs.pose_error
        status.update(session_id=self.session_id, image_sequence=image_sequence,
                      image_frame_id='', image_error=str(error), worker_error=str(error),
                      camera_mode=camera_mode, processed_stamp_s=processed_stamp,
                      projection_status='worker_error')
        return status

    def process(self, message, bridge, camera_mode, processed_stamp):
        header = message.header
        # seq distinguishes images with identical stamps; stamp distinguishes
        # publishers that reset seq. Keep seen identities for this task so an
        # older retransmitted image cannot become a new alignment observation.
        identity = (header.frame_id, header.stamp.to_sec(), getattr(header, 'seq', None))
        if identity in self.seen_images:
            return None
        self.seen_images.add(identity)
        self.image_sequence += 1
        try:
            image, status = process_image(message, bridge, self.config, self.inputs,
                camera_mode, processed_stamp, detector=self.detect)
        except Exception as exc:
            image = None
            status = self.error_status(exc, camera_mode, processed_stamp, self.image_sequence)
        status.update(session_id=self.session_id, image_sequence=self.image_sequence)
        if self.landing is not None:
            status['landing_detector'] = self.landing.status()
        return image, status


class ImageWindow:
    """Select images captured in the current phase, without an image-age limit.

    The camera may run during navigation, but those images are never detected.
    Window changes discard the mailbox; the phase start also excludes an older
    capture still in ROS transport. The mission's existing timeout keeps running.
    """
    def __init__(self, inputs):
        self.inputs = inputs
        self.window = None
        self.started_at = None

    def update(self, message):
        window = message.data or None
        try:
            started_at = float(window.rsplit('@', 1)[1]) if window else None
            if started_at is not None and not math.isfinite(started_at):
                return
        except (ValueError, IndexError, TypeError):
            return
        with self.inputs.lock:
            if window != self.window:
                self.window, self.started_at = window, started_at
                self.inputs.pending = None

    def take(self):
        with self.inputs.lock:
            message = self.inputs.take_image()
            if self.window is None or message is None:
                return None
            if message.header.stamp.to_sec() < self.started_at:
                return None
            return self.window, message


def live(config, session_id, target='red_point', controlled_window=False):
    import rospy
    import rosgraph
    import roslaunch
    from cv_bridge import CvBridge
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    config = deepcopy(config)
    config['topics'].update(session_topics(session_id))
    rospy.init_node('red_point_worker', anonymous=True)
    master = rosgraph.Master(rospy.get_name())
    landing = None
    if target == 'landing_pad':
        # Load the model while the camera starts; both count against the 5 s wait.
        landing = LandingPadDetector(config['landing_pad'])
        landing.start()
    observations, bridge = MissionObservations(config, session_id, landing), CvBridge()
    inputs = observations.inputs
    image_window = ImageWindow(inputs) if controlled_window else None
    identity = ProducerIdentity([config['topics']['raw_odometry'], config['topics']['odometry']])

    def launch_factory(path, arguments):
        launch_id = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(launch_id)
        return roslaunch.parent.ROSLaunchParent(launch_id, [(path, arguments)])

    owner = CameraOwner(config, master, launch_factory, rospy.get_name()+'/camera')
    publishers, subscribers = (), []
    try:
        publishers, subscribers = attach_ros(rospy, locals(), config, inputs)
        if image_window is not None:
            subscribers.append(rospy.Subscriber(config['topics']['status'].rsplit('/', 1)[0]+'/window',
                String, image_window.update, queue_size=1))
        # Observe the existing master. Never start a localization stack or a
        # separate master and never wait for a camera/pose readiness threshold.
        identity.changed(master)
        owner.start()
        next_identity = 0.
        while not rospy.is_shutdown():
            now = time.monotonic()
            if now >= next_identity:
                try:
                    if identity.changed(master):
                        with inputs.lock:
                            inputs.session.reset_localization('localization_producer_restarted')
                except (OSError, rosgraph.MasterError, rosgraph.MasterFailure) as exc:
                    rospy.logwarn_throttle(5., 'Cannot observe localization producer identity: %s', exc)
                next_identity = now+1.  # Master observation polling, not an age gate.
            selected = image_window.take() if image_window is not None else None
            message = (selected[1] if selected is not None else None) if image_window is not None else inputs.take_image()
            if message is None:
                time.sleep(.01)  # Idle mailbox polling only.
                continue
            processed = observations.process(message, bridge, owner.mode, rospy.get_time())
            if processed is None:
                continue
            image, status = processed
            if selected is not None:
                status['image_window'] = selected[0]
            publishers[0].publish(String(data=json.dumps(status, ensure_ascii=False, allow_nan=False)))
            if image is not None and publishers[1].get_num_connections() > 0:
                try:
                    annotated = bridge.cv2_to_imgmsg(annotate(image, status), encoding='bgr8')
                    annotated.header = message.header
                    publishers[1].publish(annotated)
                except Exception as exc:
                    rospy.logwarn_throttle(5., 'Cannot publish red-point annotation: %s', exc)
    except Exception as exc:
        if publishers:
            status = observations.error_status(exc, owner.mode, rospy.get_time())
            publishers[0].publish(String(data=json.dumps(status, ensure_ascii=False, allow_nan=False)))
        raise
    finally:
        owner.close()
        if landing is not None:
            landing.close()
        for subscriber in subscribers:
            subscriber.unregister()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='任务专用红点观察进程；无窗口、按键或飞行/投放命令。')
    parser.add_argument('--config', type=Path, required=True, help='本次任务冻结的红点配置')
    parser.add_argument('--session', required=True, help='本次任务的 UUID')
    parser.add_argument('--target', choices=TARGETS, default='red_point',
                        help='本取图窗口的识别对象：投放红点或最终降落 H 标')
    parser.add_argument('--controlled-window', action='store_true',
                        help='提前启动相机，仅在任务指定取图窗口内识别')
    args = parser.parse_args(argv)
    try:
        session_id = str(uuid.UUID(args.session))
        config = load_config(args.config)
        if args.controlled_window:
            return live(config, session_id, args.target, controlled_window=True)
        return live(config, session_id, args.target)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print('红点任务观察：'+str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
