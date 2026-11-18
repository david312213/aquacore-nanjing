#!/usr/bin/env python3
"""Bounded receive-only PX4/LIO bench audit; no control publishers/services."""
import argparse
import json
import math
import threading
import time

import numpy as np
import rosgraph
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State, TimesyncStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool


def clean(value):
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=10)
    args = parser.parse_args()
    if not 1 <= args.duration <= 30:
        raise ValueError('duration must be between 1 and 30 seconds')
    rospy.init_node('px4_lio_receive_only_audit', anonymous=True,
                    argv=['px4_lio_receive_only_audit'])
    records, latest, pose_records = {}, {}, {}
    lock = threading.Lock()
    states = []
    kinds = [('/airseafly/lio/raw_odom', Odometry),
             ('/airseafly/localization/odom', Odometry),
             ('/mavros/odometry/out', Odometry),
             ('/mavros/odometry/in', Odometry),
             ('/mavros/local_position/odom', Odometry),
             ('/livox/imu', Imu), ('/mavros/imu/data', Imu),
             ('/mavros/state', State), ('/mavros/battery', BatteryState),
             ('/mavros/estimator_status', EstimatorStatus),
             ('/mavros/extended_state', ExtendedState),
             ('/mavros/timesync_status', TimesyncStatus),
             ('/airseafly/localization/healthy', Bool),
             ('/diagnostics', DiagnosticArray)]

    def receive(message, topic):
        now = time.time()
        stamp = message.header.stamp.to_sec() if hasattr(message, 'header') else now
        info = {}
        pose = None
        if isinstance(message, Odometry):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            v, w = message.twist.twist.linear, message.twist.twist.angular
            pose = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z]
            info = {'parent': message.header.frame_id, 'child': message.child_frame_id,
                    'position': pose[:3], 'quaternion_xyzw': pose[3:7],
                    'linear_velocity': pose[7:10], 'angular_velocity': [w.x, w.y, w.z],
                    'pose_covariance_diagonal': list(message.pose.covariance[::7]),
                    'twist_covariance_diagonal': list(message.twist.covariance[::7])}
        elif isinstance(message, DiagnosticArray):
            info = {entry.name: {'level': entry.level, 'message': entry.message,
                    'values': {item.key: item.value for item in entry.values}}
                    for entry in message.status}
        else:
            for name in message.__slots__:
                item = getattr(message, name)
                if isinstance(item, (bool, int, float, str)):
                    info[name] = item
        with lock:
            records.setdefault(topic, []).append([stamp, now])
            if isinstance(message, DiagnosticArray):
                latest.setdefault(topic, {}).update(info)
            else:
                latest[topic] = info
            if pose is not None:
                pose_records.setdefault(topic, []).append(pose)
            if isinstance(message, State):
                states.append(info)

    subscribers = [rospy.Subscriber(topic, kind, receive, callback_args=topic,
                                   queue_size=1000) for topic, kind in kinds]
    start = time.monotonic()
    while time.monotonic() - start < args.duration and not rospy.is_shutdown():
        time.sleep(.05)
    for subscriber in subscribers:
        subscriber.unregister()
    publishers, _, services = rosgraph.Master(rospy.get_name()).getSystemState()
    result = {'time_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
              'duration_s': time.monotonic() - start,
              'any_armed': any(item['armed'] for item in states),
              'state_observed': bool(states),
              'any_disconnected': any(not item['connected'] for item in states),
              'publishers': dict(publishers), 'services': dict(services), 'topics': {}}
    with lock:
        for topic, kind in kinds:
            rows = records.get(topic, [])
            summary = {'count': len(rows), 'latest': latest.get(topic)}
            if rows:
                data = np.asarray(rows)
                elapsed = data[-1, 0] - data[0, 0]
                summary.update(header_rate_hz=(len(rows) - 1) / elapsed if elapsed > 0 else None,
                               nonmonotonic=int(np.sum(np.diff(data[:, 0]) <= 0)),
                               stamp_age_median_s=float(np.median(data[:, 1] - data[:, 0])),
                               stamp_age_p95_s=float(np.percentile(data[:, 1] - data[:, 0], 95)),
                               last_receive_age_s=time.time() - data[-1, 1])
            if topic in pose_records:
                poses = np.asarray(pose_records[topic])
                summary.update(position_mean=poses[:, :3].mean(axis=0).tolist(),
                               position_peak_to_peak_m=np.ptp(poses[:, :3], axis=0).tolist(),
                               velocity_rms_m_s=float(np.sqrt(np.mean(np.sum(poses[:, 7:10]**2, axis=1)))))
            result['topics'][topic] = summary
    print(json.dumps(clean(result), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
