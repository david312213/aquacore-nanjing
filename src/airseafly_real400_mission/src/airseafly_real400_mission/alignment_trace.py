"""Read-only command/feedback traces for open-loop and alignment comparisons.

Each observation retains its own capture and receipt timestamps. These are
latest-value joins, not a claim that the controller used that exact odometry.
ROS callbacks enqueue bounded records; a separate thread owns all disk I/O.
"""
from copy import deepcopy
import json
import math
import queue
import threading


class TraceBuffer:
    def __init__(self, period_s=.05):
        self.period_s = period_s
        self.latest = {}
        self.last_sample = None

    def update(self, name, values, stamp, received):
        self.latest[name] = dict(stamp_s=stamp, received_s=received, values=deepcopy(values))

    def command(self, values, stamp, received, publisher=None):
        if self.last_sample is not None and 0 <= received-self.last_sample < self.period_s-1e-9:
            return None
        self.last_sample = received
        return dict(sampled_ros_time=received, command=dict(stamp_s=stamp, received_s=received,
                    publisher=publisher, values=deepcopy(values)), feedback=deepcopy(self.latest))


def vector(value):
    return [float(getattr(value, key)) for key in ('x', 'y', 'z')]


def quaternion(value):
    return [float(getattr(value, key)) for key in ('x', 'y', 'z', 'w')]


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [clean(item) for item in value]
    return value


class AlignmentTrace:
    def __init__(self, path):
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from quadrotor_msgs.msg import PositionCommand, Px4ctrlDebug
        from std_msgs.msg import String, Float64MultiArray
        from mavros_msgs.msg import RCIn
        from mavros_msgs.msg import PositionTarget
        self.rospy, self.path = rospy, path
        self.pd_path = path.with_name('alignment_pd.jsonl')
        self.pd_stream, self.pd_samples = None, 0
        self.lock, self.buffer = threading.Lock(), TraceBuffer()
        self.diagnostic_period_s, self.diagnostic_last_sample = .5, {}
        self.queue = queue.Queue(maxsize=256)
        self.closed, self.dropped, self.error = False, 0, None
        self.stream = path.open('a', buffering=1)
        self.worker = threading.Thread(target=self._write, daemon=True)
        self.worker.start()
        self.subscriptions = []
        try:
            for topic, kind, callback in (
                ('/Odom_high_freq', Odometry, self.odom),
                ('/mavros/imu/data', Imu, self.imu),
                ('/debugPx4ctrl', Px4ctrlDebug, self.debug),
                ('/px4ctrl/pd_trace', Float64MultiArray, self.pd),
                ('/mavros/rc/in', RCIn, self.rc),
                ('/px4ctrl/control_state', String, self.control),
                ('/airseafly/mission/state', String, self.state),
                ('/position_cmd', PositionCommand, self.command),
                ('/px4ctrl/native_status', String, self.native_status),
                ('/prediction/status', String, self.prediction_status),
                ('/airseafly/native/localization_status', String, self.localization_status),
                ('/mavros/setpoint_raw/local', PositionTarget, self.native_command),
            ):
                self.subscriptions.append(rospy.Subscriber(topic, kind, callback, queue_size=1))
        except Exception:
            self.close()
            raise

    def _update(self, name, values, message):
        now = self.rospy.get_time()
        header = getattr(message, 'header', None)
        stamp = header.stamp.to_sec() if header is not None else None
        with self.lock:
            self.buffer.update(name, clean(values), stamp, now)

    def odom(self, msg):
        self._update('odom', dict(position_xyz=vector(msg.pose.pose.position),
            orientation_xyzw=quaternion(msg.pose.pose.orientation), velocity_xyz=vector(msg.twist.twist.linear)), msg)

    def imu(self, msg):
        self._update('imu', dict(orientation_xyzw=quaternion(msg.orientation),
                                angular_velocity_xyz=vector(msg.angular_velocity)), msg)

    def debug(self, msg):
        self._update('controller', {key: getattr(msg, key) for key in getattr(msg, '__slots__', ())
                     if key.startswith('des_')}, msg)

    def control(self, msg):
        self._update('control_state', msg.data, msg)

    def rc(self, msg):
        self._update('rc', dict(channels=list(msg.channels), rssi=msg.rssi), msg)

    def pd(self, msg):
        # Exact controller inputs, also during AUTO_HOVER when the mission
        # deliberately stops publishing /position_cmd. No control decisions.
        if self.closed:
            return
        from .hover_pid import decode_trace
        try:
            values = decode_trace(msg.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            packet = dict(record_type='controller_pd', sampled_ros_time=self.rospy.get_time(),
                          controller=values, feedback={name: deepcopy(self.buffer.latest[name])
                          for name in ('rc', 'control_state', 'mission') if name in self.buffer.latest})
        self._enqueue(packet)

    def state(self, msg):
        try:
            state = json.loads(msg.data)
            out = state.get('output') or {}
            self._update('mission', dict(task_id=state.get('task_id'), active=state.get('active'),
                output={key: out.get(key) for key in ('state', 'phase', 'hybrid', 'selected_frame_stamp_s',
                        'selected_bbox', 'pixel_error', 'aim_pixel', 'velocity_xyz', 'position_xyz', 'yaw',
                        'native', 'waypoint_route', 'delivery_route', 'drop_test', 'drop_points',
                        'down_target_association')}), msg)
        except (ValueError, TypeError):
            pass

    def _enqueue(self, packet):
        try:
            self.queue.put_nowait(packet)
        except queue.Full:
            self.dropped += 1

    def command(self, msg):
        if self.closed:
            return
        values = dict(position_xyz=vector(msg.position), velocity_xyz=vector(msg.velocity),
                      acceleration_xyz=vector(msg.acceleration), yaw=msg.yaw, frame_id=msg.header.frame_id)
        with self.lock:
            packet = self.buffer.command(clean(values), msg.header.stamp.to_sec(), self.rospy.get_time(),
                                         (getattr(msg, '_connection_header', None) or {}).get('callerid'))
        if packet is not None:
            try:
                self.queue.put_nowait(packet)
            except queue.Full:
                self.dropped += 1

    def native_status(self, msg):
        try:
            self._update('px4_native', json.loads(msg.data), msg)
        except (ValueError, TypeError):
            pass

    def _json_status(self, name, msg, stamp_key=None):
        # Keep command joins current and retain an independent low-rate status
        # record even when setpoint publication has stopped. Both paths use the
        # same bounded asynchronous queue; neither changes flight decisions.
        if self.closed:
            return
        try:
            values = json.loads(msg.data)
            if not isinstance(values, dict):
                return
            stamp = values.get(stamp_key) if stamp_key else None
            if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                stamp = None
            packet = None
            with self.lock:
                now = self.rospy.get_time()
                self.buffer.update(name, clean(values), stamp, now)
                last = self.diagnostic_last_sample.get(name)
                if last is None or now < last or now-last >= self.diagnostic_period_s-1e-9:
                    self.diagnostic_last_sample[name] = now
                    packet = dict(record_type='diagnostic', event=name, sampled_ros_time=now,
                                  diagnostic=deepcopy(self.buffer.latest[name]))
            if packet is not None:
                self._enqueue(packet)
        except (ValueError, TypeError):
            pass

    def prediction_status(self, msg):
        # The predictor has source/correction stamps in its JSON but no status
        # publication stamp. Preserve them without inventing an envelope time.
        self._json_status('lio_prediction', msg)

    def localization_status(self, msg):
        self._json_status('native_localization', msg, 'stamp')

    def native_command(self, msg):
        if self.closed:
            return
        values = dict(position_xyz=vector(msg.position), velocity_xyz=vector(msg.velocity),
                      yaw=msg.yaw, frame_id='px4_local_enu', controller='PX4_NATIVE', type_mask=msg.type_mask)
        with self.lock:
            packet = self.buffer.command(clean(values), msg.header.stamp.to_sec(), self.rospy.get_time(),
                                         (getattr(msg, '_connection_header', None) or {}).get('callerid'))
        if packet is not None:
            try:
                self.queue.put_nowait(packet)
            except queue.Full:
                self.dropped += 1

    def _write(self):
        try:
            while not self.closed or not self.queue.empty():
                try:
                    packet = self.queue.get(timeout=.1)
                except queue.Empty:
                    continue
                packet['dropped_records'] = self.dropped
                if packet.get('record_type') == 'controller_pd':
                    if self.pd_stream is None:
                        self.pd_stream = self.pd_path.open('a', buffering=1)
                    self.pd_stream.write(json.dumps(packet, ensure_ascii=False, allow_nan=False)+'\n')
                    self.pd_samples += 1
                else:
                    self.stream.write(json.dumps(packet, ensure_ascii=False, allow_nan=False)+'\n')
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.stream.close()
            if self.pd_stream is not None:
                self.pd_stream.close()

    def status(self):
        return dict(path=str(self.path), error=self.error, dropped_records=self.dropped,
                    pd_path=str(self.pd_path), pd_samples=self.pd_samples)

    def close(self):
        self.closed = True
        for sub in self.subscriptions:
            sub.unregister()
        self.worker.join(timeout=2.)
