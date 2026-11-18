#!/usr/bin/env python3
"""Export raw IMUs from closed ROS bags; does not create a ROS node."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

TOPICS = {'/livox/imu': 'livox', '/mavros/imu/data_raw': 'px4',
          '/mavros/timesync_status': 'timesync', '/mavros/time_reference': 'time_reference'}


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    import rosbag
    folder = args.session.resolve()
    files = sorted(folder.glob('raw*.bag'))
    if not files or list(folder.glob('*.active')):
        raise ValueError('录包尚未完成或没有找到 raw*.bag')
    output = args.output or folder/'imu.npz'
    rows = {name: [] for name in TOPICS.values()}
    meta = dict(source=str(folder), files=[],
                columns_imu=['header_s', 'record_s', 'wx', 'wy', 'wz', 'ax', 'ay', 'az'],
                units=dict(angular_velocity='rad/s', livox_acceleration='g', px4_acceleration='m/s^2'))
    origin = None
    for path in files:
        before = path.stat()
        with rosbag.Bag(str(path), 'r') as bag:
            if origin is None:
                origin = round(bag.get_start_time()*1e9)
            for topic, msg, stamp in bag.read_messages(topics=list(TOPICS)):
                h, r = (msg.header.stamp.to_nsec()-origin)*1e-9, (stamp.to_nsec()-origin)*1e-9
                if TOPICS[topic] in ('livox', 'px4'):
                    w, a = msg.angular_velocity, msg.linear_acceleration
                    row = [h, r, w.x, w.y, w.z, a.x, a.y, a.z]
                elif TOPICS[topic] == 'timesync':
                    row = [h, r, (int(msg.estimated_offset_ns)-origin)*1e-9,
                           (int(msg.observed_offset_ns)-origin)*1e-9,
                           msg.round_trip_time_ms, msg.remote_timestamp_ns*1e-9]
                else:
                    row = [h, r, msg.time_ref.to_nsec()*1e-9]
                rows[TOPICS[topic]].append(row)
        after = path.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise ValueError('读取期间源 bag 发生变化：'+str(path))
        meta['files'].append(dict(path=str(path), bytes=before.st_size, sha256=sha256(path)))
    meta['bag_start_ns'] = origin
    arrays = {key: np.asarray(value, dtype=float).reshape((-1, 8 if key in ('livox', 'px4')
                                                        else 6 if key == 'timesync' else 3))
              for key, value in rows.items()}
    # Exclusive creation avoids replacing an earlier export.
    with output.open('xb') as stream:
        np.savez_compressed(stream, metadata=np.array(json.dumps(meta, ensure_ascii=False)), **arrays)
    print(json.dumps(dict(output=str(output), samples={k: len(v) for k, v in arrays.items()}),
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
