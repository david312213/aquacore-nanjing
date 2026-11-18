#!/usr/bin/env python3
"""Record raw sensors for mounting calibration. No vehicle command/service API."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[3]
TOPICS = [
    '/livox/lidar', '/livox/imu', '/mavros/imu/data_raw', '/mavros/imu/data',
    '/mavros/state', '/mavros/extended_state', '/mavros/timesync_status',
    '/mavros/time_reference', '/mavros/imu/temperature_imu',
    '/Odometry', '/Odom_high_freq', '/diagnostics',
]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--label', default='mount')
    parser.add_argument('--output-root', type=Path, default=ROOT/'data/calibration/mid360_px4')
    args = parser.parse_args(argv)
    if args.dry_run:
        print(json.dumps(dict(output_root=str(args.output_root), topics=TOPICS,
                              ros_master_uri='http://127.0.0.1:11341',
                              commands_vehicle=False, changes_parameters=False,
                              automatically_applies_calibration=False), ensure_ascii=False, indent=2))
        return 0
    import roslib.packages
    import rospy
    from mavros_msgs.msg import State
    from sensor_msgs.msg import Imu

    stop, lock = threading.Event(), threading.Lock()
    rospy.init_node('mid360_px4_mount_recorder', anonymous=True, disable_signals=True)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    samples, latest_state = {}, {}

    def imu_callback(msg, topic):
        with lock:
            info = samples.setdefault(topic, dict(count=0, first=time.monotonic()))
            info.update(count=info['count']+1, last=time.monotonic(),
                        last_header_stamp=msg.header.stamp.to_sec(), frame=msg.header.frame_id)

    def state_callback(msg):
        with lock:
            latest_state.update(connected=msg.connected, armed=msg.armed, mode=msg.mode)

    subscribers = [rospy.Subscriber(topic, Imu, imu_callback, callback_args=topic, queue_size=500)
                   for topic in ('/livox/imu', '/mavros/imu/data_raw')]
    subscribers.append(rospy.Subscriber('/mavros/state', State, state_callback, queue_size=10))
    writers = roslib.packages.find_node('rosbag', 'record')
    if not writers:
        raise RuntimeError('找不到 ROS rosbag record 可执行文件')
    args.output_root.mkdir(parents=True, exist_ok=True)
    prefix = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_')
    session = Path(tempfile.mkdtemp(prefix=prefix, dir=str(args.output_root)))
    snapshots = {}
    for name in ('frames.yaml', 'fastlio.yaml'):
        data = (ROOT/'config'/name).read_bytes()
        (session/(name+'.before')).write_bytes(data)
        snapshots[name] = hashlib.sha256(data).hexdigest()
    metadata = dict(label=args.label, created_at=datetime.datetime.now().astimezone().isoformat(),
                    ros_master_uri=os.environ.get('ROS_MASTER_URI'), topics=TOPICS,
                    config_sha256=snapshots, status='RECORDING', calibration_applied=False)
    (session/'session.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
    command = [writers[0], '--lz4', '--buffsize=512', '--split', '--size=2048',
               '-O', str(session/'raw')] + TOPICS
    child = subprocess.Popen(command, start_new_session=True)
    print('记录目录：'+str(session), flush=True)
    print('看到 rosbag 的 Recording to 后开始动作；Ctrl+C 保存并结束本组。', flush=True)
    print('仅读取数据；没有解锁、模式切换、设定点或参数写入调用。', flush=True)
    try:
        while child.poll() is None and not stop.wait(2):
            if rospy.is_shutdown():
                break
            with lock:
                rates = {topic: round((s['count']-1)/max(s['last']-s['first'], 1e-9), 1)
                         for topic, s in samples.items()}
                state = dict(latest_state)
            print('采样 Hz：%s；飞控：%s' % (rates, state), flush=True)
    finally:
        if child.poll() is None:
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            # Wait for the writer to close its bag; never kill a writer flushing data.
            while child.poll() is None:
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print('rosbag 仍在完成写盘，保留进程及 .active 文件。', flush=True)
        for sub in subscribers:
            sub.unregister()
        bags = sorted(session.glob('raw*.bag'))
        closed = bool(bags) and not list(session.glob('*.active'))
        with lock:
            metadata.update(imu_samples=samples, latest_state=latest_state)
        metadata.update(status='RECORDED' if closed else 'CHECK_BAG_FILES', bag_closed=closed,
                        recorder_exit_code=child.returncode, bags=[p.name for p in bags])
        (session/'session.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
        print('记录结束：'+str(session), flush=True)
        print('传感器终端保持运行；本组尚未解算、未应用外参。', flush=True)
    return 0 if closed and child.returncode == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
