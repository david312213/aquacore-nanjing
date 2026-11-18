#!/usr/bin/env python3
"""Ground stress monitor: CPU/thermal from /proc and FAST-LIO latency from ROS.

Receive-only. It advertises nothing, calls no service, and never arms, switches mode,
starts or stops a process. Type a phase label + Enter to mark a phase; 'q' or Ctrl-C ends
and prints the per-phase report. CSV rows are flushed as written so a power cut keeps them.
"""
import argparse
import collections
import csv
import json
import os
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import stress_core as core  # noqa: E402

PROJECT = Path(__file__).resolve().parents[2]
RED_POINT_SESSION = '5f3a0c1e-2b7d-4c41-9e8a-1a2b3c4d5e6f'  # fixed so the status topic is known
STREAMS = [  # (name, topic, has_header)
    ('lidar', '/livox/lidar', True),
    ('lidar_imu', '/livox/imu', True),
    ('lio_odom', '/Odometry', True),
    ('lio_high', '/Odom_high_freq', True),
    ('planning_odom', '/airseafly/native/planning_odometry', True),
    ('red_point', '/airseafly/mission/red_point_%s/status' % RED_POINT_SESSION.replace('-', '_'),
     False),
]
RATE_WINDOW_S = 2.0


class Streams:
    def __init__(self):
        self.lock = threading.Lock()
        self.started = time.time()
        self.arrivals = {name: collections.deque() for name, _, _ in STREAMS}
        self.stamp = {}
        self.pred = {}
        self.healthy = None
        self.state = None

    def receive(self, name, has_header, message):
        now = time.time()
        with self.lock:
            self.arrivals[name].append(now)
            if has_header:
                stamp = core.header_stamp(message._buff)
                if stamp is not None:
                    self.stamp[name] = stamp

    def snapshot(self, now):
        row = {}
        with self.lock:
            warm = now - self.started >= RATE_WINDOW_S
            for name, _, has_header in STREAMS:
                arrivals = self.arrivals[name]
                while arrivals and arrivals[0] < now - RATE_WINDOW_S:
                    arrivals.popleft()
                row[name + '_hz'] = len(arrivals) / RATE_WINDOW_S if warm else None
                stamp = self.stamp.get(name)
                row[name + '_age_s'] = now - stamp if has_header and stamp else None
            if self.pred.get('text'):
                try:
                    row.update(core.prediction_status(self.pred['text'], now))
                except ValueError:
                    pass
            row['loc_healthy'] = None if self.healthy is None else int(self.healthy)
            if self.state:
                row['fcu_connected'] = int(self.state[0])
                row['armed'] = int(self.state[1])
                row['mode'] = self.state[2]
        return row


def attach_ros(streams):
    import rospy
    rospy.init_node('stress_monitor', anonymous=True, disable_signals=True)
    subs = []
    for name, topic, has_header in STREAMS:
        subs.append(rospy.Subscriber(
            topic, rospy.AnyMsg, lambda m, n=name, h=has_header: streams.receive(n, h, m),
            queue_size=50, buff_size=1 << 24, tcp_nodelay=True))

    def pred(m):
        with streams.lock:
            streams.pred['text'] = core.string_payload(m._buff)

    def healthy(m):
        with streams.lock:
            streams.healthy = bool(m._buff[0])

    def state(m):
        with streams.lock:
            streams.state = core.mavros_state(m._buff)
    subs.append(rospy.Subscriber('/prediction/status', rospy.AnyMsg, pred, queue_size=5))
    subs.append(rospy.Subscriber('/airseafly/native/localization_healthy', rospy.AnyMsg,
                                 healthy, queue_size=5))
    subs.append(rospy.Subscriber('/mavros/state', rospy.AnyMsg, state, queue_size=5))
    return subs


def _read(path, binary=False):
    with open(path, 'rb' if binary else 'r') as handle:
        return handle.read()


def _read_int(path):
    try:
        return int(_read(path).strip())
    except (OSError, ValueError):
        return None


class ProcSampler:
    """System and per-process-group CPU from /proc; thermal/frequency from sysfs."""
    def __init__(self):
        self.ncpu = os.cpu_count() or 1
        self.groups, self.comms = {}, {}
        self.prev_stat = self._stat()
        self.prev_pids = self._pid_jiffies()
        cpu_root = Path('/sys/devices/system/cpu')
        self.freq_paths = sorted(cpu_root.glob('cpu[0-9]*/cpufreq/scaling_cur_freq'))
        self.throttle_paths = sorted(cpu_root.glob('cpu[0-9]*/thermal_throttle/core_throttle_count'))
        self.throttle_paths += sorted(cpu_root.glob('cpu0/thermal_throttle/package_throttle_count'))
        zones = sorted(Path('/sys/class/thermal').glob('thermal_zone*'))
        named = [z for z in zones if (z / 'type').exists() and _read(z / 'type').strip() == 'x86_pkg_temp']
        self.temp_paths = [z / 'temp' for z in (named or zones)]

    @staticmethod
    def _stat():
        text = _read('/proc/stat')
        cores = {}
        for line in text.splitlines():
            head = line.split(' ', 1)[0]
            if head.startswith('cpu') and head != 'cpu':
                cores[head] = core.parse_cpu_line('cpu ' + line.split(' ', 1)[1])
        return core.parse_cpu_line(text), cores

    def _pid_jiffies(self):
        out = {}
        for entry in os.scandir('/proc'):
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            try:
                stat = _read('/proc/%d/stat' % pid)
                out[pid] = core.parse_pid_jiffies(stat)
                comm = stat[stat.index('(') + 1:stat.rindex(')')]
                if self.comms.get(pid) != comm:  # new pid, or exec() replaced a wrapper
                    cmd = _read('/proc/%d/cmdline' % pid, True).replace(b'\0', b' ')
                    self.groups[pid] = core.classify(cmd.decode('utf-8', 'replace'))
                    self.comms[pid] = comm
            except (OSError, ValueError, IndexError):
                continue
        for pid in set(self.groups) - set(out):
            del self.groups[pid]
            self.comms.pop(pid, None)
        return out

    def sample(self):
        total, cores = self._stat()
        pids = self._pid_jiffies()
        prev_total, prev_cores = self.prev_stat
        row = dict(cpu_pct=core.cpu_percent(prev_total, total))
        row['cpu_max_core_pct'] = max([core.cpu_percent(prev_cores[c], v)
                                       for c, v in cores.items() if c in prev_cores] or [0])
        groups = core.group_percent(self.prev_pids, pids, self.groups, total[1] - prev_total[1])
        groups = {name: pct * self.ncpu for name, pct in groups.items()}  # -> % of one core
        self.prev_stat, self.prev_pids = (total, cores), pids
        freqs = [v / 1000.0 for v in map(_read_int, self.freq_paths) if v]
        row['freq_mhz_min'] = min(freqs) if freqs else None
        row['freq_mhz_mean'] = sum(freqs) / len(freqs) if freqs else None
        temps = [v / 1000.0 for v in map(_read_int, self.temp_paths) if v]
        row['temp_c'] = max(temps) if temps else None
        counts = [v for v in map(_read_int, self.throttle_paths) if v is not None]
        row['throttle_count'] = sum(counts) if counts else None
        row['load1'] = float(_read('/proc/loadavg').split()[0])
        for line in _read('/proc/meminfo').splitlines():
            if line.startswith('MemAvailable:'):
                row['mem_avail_mb'] = int(line.split()[1]) / 1024.0
        return row, groups


def load_and_report(out_dir):
    with open(out_dir / 'samples.csv') as handle:
        samples = [core.parse_sample(r) for r in csv.DictReader(handle)]
    with open(out_dir / 'groups.csv') as handle:
        groups = [(r['phase'], r['group'], float(r['pct_one_core'])) for r in csv.DictReader(handle)]
    report = core.summarize(samples, groups)
    with open(out_dir / 'summary.json', 'w') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=1)
    print(core.format_report(report))
    print('\n结果目录：%s' % out_dir)
    return report


def _fmt(value, spec):
    return '-' if value is None else format(value, spec)


def transitions(prev, row, phase):
    """Event strings worth a line in events.csv when a sample differs from the last one."""
    events = []
    for key, label in (('loc_healthy', 'localization_healthy'), ('fcu_connected', 'fcu_connected'),
                       ('armed', 'armed'), ('mode', 'mode')):
        if prev.get(key) != row.get(key) and row.get(key) is not None:
            events.append('%s %s -> %s' % (label, prev.get(key), row.get(key)))
    old_lag = (prev.get('lio_odom_age_s') or 0) > core.ODOM_AGE_FLAG_S
    new_lag = (row.get('lio_odom_age_s') or 0) > core.ODOM_AGE_FLAG_S
    if old_lag != new_lag:
        events.append('lio_odom_age %s %.1fs' % ('above' if new_lag else 'back below',
                                                  core.ODOM_AGE_FLAG_S))
    if (row.get('pred_withheld') or 0) > (prev.get('pred_withheld') or 0):
        events.append('prediction withheld %s (%s)' % (row.get('pred_withheld'),
                                                       row.get('pred_last_withheld')))
    return events


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--out', type=Path, help='output directory (default data/stress/<time>)')
    parser.add_argument('--interval', type=float, default=0.5, help='sample period, s')
    parser.add_argument('--phase', default='A_baseline', help='initial phase label')
    parser.add_argument('--duration', type=float, default=0, help='stop after N s (0 = until q)')
    parser.add_argument('--no-ros', action='store_true', help='CPU/thermal only, no ROS master')
    parser.add_argument('--summarize', type=Path, metavar='DIR', help='re-print report of DIR')
    args = parser.parse_args()
    if args.summarize:
        load_and_report(args.summarize)
        return 0
    out_dir = args.out or PROJECT / 'data' / 'stress' / time.strftime('%Y%m%d_%H%M%S')
    out_dir.mkdir(parents=True, exist_ok=True)
    streams = Streams()
    if not args.no_ros:
        attach_ros(streams)
    sampler = ProcSampler()
    state = dict(phase=args.phase.replace(',', '_'), stop=False)
    files = [open(out_dir / name, 'w', newline='') for name in ('samples.csv', 'groups.csv', 'events.csv')]
    samples = csv.DictWriter(files[0], core.SAMPLE_FIELDS, extrasaction='ignore')
    groups, events = csv.writer(files[1]), csv.writer(files[2])
    samples.writeheader()
    groups.writerow(['wall_time', 'phase', 'group', 'pct_one_core'])
    events.writerow(['wall_time', 'phase', 'event'])

    event_lock = threading.Lock()

    def event(text):
        with event_lock:  # stdin thread and sample loop both log
            if files[2].closed:
                return
            events.writerow(['%.3f' % time.time(), state['phase'], text])
            files[2].flush()
        print('[%s] %s: %s' % (time.strftime('%H:%M:%S'), state['phase'], text), flush=True)

    def read_phases():
        for line in sys.stdin:
            label = line.strip().replace(',', '_')
            if label.lower() in ('q', 'quit', 'exit'):
                state['stop'] = True
                return
            if label:
                state['phase'] = label
                event('phase start')
    # Terminal or pipe (e.g. `tail -F phases.txt |`); /dev/null just ends the thread.
    threading.Thread(target=read_phases, daemon=True).start()
    print('输出：%s\n输入阶段名回车切换阶段；q 回车或 Ctrl-C 结束。' % out_dir, flush=True)
    event('phase start')
    started, prev, ticks = time.time(), {}, 0
    try:
        while not state['stop'] and not (args.duration and time.time() - started >= args.duration):
            time.sleep(max(0.0, args.interval - (time.time() - started) % args.interval))
            now = time.time()
            proc_row, group_pct = sampler.sample()
            row = dict(wall_time='%.3f' % now, phase=state['phase'], **proc_row)
            row.update(streams.snapshot(now))
            samples.writerow(row)
            for name, pct in sorted(group_pct.items()):
                groups.writerow(['%.3f' % now, state['phase'], name, '%.1f' % pct])
            for f in files[:2]:
                f.flush()
            ticks += 1
            if ticks % 10 == 0:
                for f in files:
                    os.fsync(f.fileno())
            for text in transitions(prev, row, state['phase']):
                event(text)
            if row.get('armed') == 1 and prev.get('armed') != 1:
                print('!!! 飞控已解锁：本测试不应解锁，立即用遥控器上锁 !!!', flush=True)
            prev = row
            if ticks % max(1, int(2 / args.interval)) == 0:
                print('%-14s cpu %5s%% core %5s%% %sC %sMHz | lidar %sHz odomAge %ss highAge %ss '
                      'corrLag %ss wthld %s | healthy %s %s' % (
                          state['phase'][:14], _fmt(row['cpu_pct'], '.0f'),
                          _fmt(row['cpu_max_core_pct'], '.0f'), _fmt(row.get('temp_c'), '.0f'),
                          _fmt(row.get('freq_mhz_min'), '.0f'), _fmt(row.get('lidar_hz'), '.1f'),
                          _fmt(row.get('lio_odom_age_s'), '.2f'), _fmt(row.get('lio_high_age_s'), '.2f'),
                          _fmt(row.get('pred_corr_lag_s'), '.2f'), _fmt(row.get('pred_withheld'), '.0f'),
                          row.get('loc_healthy'), row.get('mode') or ''), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        event('monitor stop')
        with event_lock:
            for f in files:
                f.flush()
                os.fsync(f.fileno())
                f.close()
    print()
    load_and_report(out_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
