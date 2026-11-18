#!/usr/bin/env python3
"""Cold-start the mission's red-point worker as an image window does; report its first frames.

Receive-only on the flight master: subscribes to the worker's status topic only, publishes
nothing, calls no service. Same worker, flight/red_point.yaml (frozen copy) and camera
start as the mission. Usage: first_frame_probe.py [TRIALS] [HOLD_S]
"""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid

import rospy
from std_msgs.msg import String

PROJECT = Path(__file__).resolve().parents[2]
WORKER = PROJECT/'src/airseafly_real400_mission/scripts/red_point_worker.py'
SOURCE = PROJECT/'flight/red_point.yaml'


def usable(candidates, kind):
    """Same filter delivery_alignment applies before choosing a target."""
    out = []
    for c in candidates or []:
        if not isinstance(c, dict) or not c.get('target_confirmed'):
            continue
        if c.get('class_name', 'red_point') != kind or c.get('projection_status') != 'ok_near_level':
            continue
        xy = c.get('xy_m')
        if isinstance(xy, list) and len(xy) == 2:
            out.append(xy)
    return out


def camera_free(device):
    return subprocess.run(['fuser', device], capture_output=True).stdout.strip() == b''


def trial(index, hold_s, snapshot, device):
    session = str(uuid.uuid4())
    topic = '/airseafly/mission/red_point_%s/status' % session.replace('-', '_')
    frames = []
    sub = rospy.Subscriber(topic, String, lambda m: frames.append((time.monotonic(), json.loads(m.data))),
                           queue_size=100)
    started = time.monotonic()
    proc = subprocess.Popen([sys.executable, str(WORKER), '--config', snapshot, '--session', session],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(hold_s)
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    sub.unregister()
    for _ in range(40):  # Camera released before the next window, as in the mission.
        if camera_free(device):
            break
        time.sleep(0.25)
    if not frames:
        print('第 %d 次：%.0f 秒内没有任何识别结果（检查相机和三个终端）' % (index, hold_s), flush=True)
        return None
    pose = [i for i, (_, s) in enumerate(frames) if s.get('projection_status') == 'ok_near_level']
    green = [i for i, (_, s) in enumerate(frames) if usable(s.get('green_candidates'), 'green_circle')]
    red = [i for i, (_, s) in enumerate(frames) if usable(s.get('candidates'), 'red_point')]

    def at(seq, key, kind):
        if not seq:
            return '无'
        xy = usable(frames[seq[0]][1].get(key), kind)[0]
        return '第 %d 帧（%.2f 秒）XY=(%.3f, %.3f)' % (seq[0]+1, frames[seq[0]][0]-started, xy[0], xy[1])
    first_t, first = frames[0]
    span = frames[-1][0] - first_t
    rate = (len(frames)-1) / span if span > 0 else 0.
    height = first.get('camera_height_m')
    print('第 %d 次：首帧 %.2f 秒，%d 帧，约 %.0f 帧/秒，相机高度 %s m，定位%s' % (
        index, first_t-started, len(frames), rate, '%.2f' % height if height is not None else '-',
        '第 1 帧就可用' if pose[:1] == [0] else ('从第 %d 帧可用' % (pose[0]+1) if pose else '不可用（%s）'
                                                % first.get('projection_status'))), flush=True)
    print('        红点：%s' % at(red, 'candidates', 'red_point'), flush=True)
    print('        绿环：%s' % at(green, 'green_candidates', 'green_circle'), flush=True)
    return dict(pose=pose[:1] == [0], red=red[:1] == [0], green=green[:1] == [0],
                red_any=bool(red), green_any=bool(green))


def main():
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    hold_s = float(sys.argv[2]) if len(sys.argv) > 2 else 5.
    import yaml
    device = yaml.safe_load(SOURCE.read_text(encoding='utf-8'))['camera']['video_device']
    if not camera_free(device):
        raise SystemExit('相机正被占用，先关掉红点预览或其他用相机的程序。')
    rospy.init_node('first_frame_probe', anonymous=True, disable_signals=True)
    handle, snapshot = tempfile.mkstemp(prefix='first_frame_', suffix='.yaml')
    os.close(handle)
    shutil.copyfile(SOURCE, snapshot)  # Frozen copy, as the manager makes per task.
    print('冷启动识别 %d 次，每次运行 %.0f 秒。保持飞机不动。' % (trials, hold_s), flush=True)
    try:
        results = [trial(i+1, hold_s, snapshot, device) for i in range(trials)]
    finally:
        os.unlink(snapshot)
    ok = [r for r in results if r]
    count = lambda key: sum(1 for r in ok if r[key])
    print('\n汇总：启动 %d/%d；第 1 帧定位可用 %d；第 1 帧算出红点坐标 %d（任一帧 %d）；'
          '第 1 帧算出绿环坐标 %d（任一帧 %d）' % (len(ok), trials, count('pose'), count('red'),
                                         count('red_any'), count('green'), count('green_any')), flush=True)


if __name__ == '__main__':
    main()
