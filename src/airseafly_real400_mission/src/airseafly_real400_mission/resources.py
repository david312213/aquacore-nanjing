"""One OS-wide manager; only target approach permits EGO and DOWN together."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

from .compute_groups import members, group_name


class ResourceBusy(RuntimeError):
    pass


class ProcessLock:
    def __init__(self, path, description):
        self.path = str(path)
        self.stream = open(self.path, 'a+')
        try:
            fcntl.flock(self.stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.stream.seek(0)
            detail = self.stream.read().strip()
            self.stream.close()
            raise ResourceBusy('计算资源正在使用: ' + detail)
        self.stream.seek(0)
        self.stream.truncate()
        self.stream.write(json.dumps(dict(pid=os.getpid(), description=description)))
        self.stream.flush()

    def close(self):
        if not self.stream.closed:
            fcntl.flock(self.stream, fcntl.LOCK_UN)
            self.stream.close()


SESSION_ENV = 'AIRSEAFLY_COMPUTE_SESSION'


def process_identity(pid):
    """A PID alone is not ownership: Linux can reuse it after the worker exits."""
    try:
        entry = Path('/proc') / str(pid)
        raw = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
        return dict(pid=int(pid), uid=entry.stat().st_uid, state=raw[0],
                    ppid=int(raw[1]), pgid=int(raw[2]), start_ticks=int(raw[19]),
                    cpu_ticks=int(raw[11]) + int(raw[12]),
                    rss_bytes=int(raw[21]) * os.sysconf('SC_PAGE_SIZE'))
    except (OSError, ValueError, IndexError):
        return None


def _processes():
    result = {}
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            identity = process_identity(entry.name)
            if identity is not None:
                result[identity['pid']] = identity
    return result


def _same_process(current, previous):
    return (current is not None and previous is not None and
            current['pid'] == previous['pid'] and current['uid'] == previous['uid'] and
            current['start_ticks'] == previous['start_ticks'])


def processes_in_group(pgid):
    """Compatibility helper; compute ownership uses OwnedProcessTree below."""
    return [entry for entry in _processes().values()
            if entry['pgid'] == pgid and entry['state'] != 'Z']


class OwnedProcessTree:
    """Remember descendants across setsid/reparenting and verify every identity.

    Only an explicitly supplied, fresh worker token is used for orphan recovery.
    A caller's inherited environment is deliberately not used: an unrelated
    private roscore must never be conflated with a compute-worker session.
    """
    def __init__(self, process, session_id=None):
        self.process = process
        self.uid = os.getuid()
        self.session_id = session_id
        self.known = {}
        self.lock = threading.RLock()
        identity = process_identity(process.pid)
        if identity is not None and identity['uid'] == self.uid:
            self.known[process.pid] = identity
        self.snapshot()

    def _has_session(self, pid):
        if self.session_id is None:
            return False
        expected = (SESSION_ENV + '=' + self.session_id).encode()
        try:
            # Read only for an exact token comparison; never retain/log other env.
            return expected in (Path('/proc') / str(pid) / 'environ').read_bytes().split(b'\0')
        except OSError:
            return False

    def snapshot(self):
        with self.lock:
            records = {pid: item for pid, item in _processes().items()
                       if item['uid'] == self.uid}
            live_known = {pid for pid, old in self.known.items()
                          if _same_process(records.get(pid), old)}
            if self.session_id is not None:
                for pid in records:
                    if pid not in live_known and self._has_session(pid):
                        # The root can exit before the first tree scan; inherited
                        # session identity still recovers its orphaned children.
                        live_known.add(pid)
                        self.known[pid] = records[pid]
            while True:
                children = {pid for pid, item in records.items()
                            if item['ppid'] in live_known and pid not in live_known}
                if not children:
                    break
                for pid in children:
                    self.known[pid] = records[pid]
                live_known.update(children)
            return [records[pid] for pid in sorted(live_known)
                    if records[pid]['state'] != 'Z']


def track_process(process, session_id=None):
    """Attach tracking without changing Popen or the stop_process_group API."""
    tracker = getattr(process, '_airseafly_owned_tree', None)
    if tracker is None:
        tracker = OwnedProcessTree(process, session_id)
        process._airseafly_owned_tree = tracker
    return tracker


def _signal_identity(identity, sig):
    current = process_identity(identity['pid'])
    if not _same_process(current, identity) or current['state'] == 'Z':
        return False
    try:
        os.kill(identity['pid'], sig)
        return True
    except ProcessLookupError:
        return False


def unmanaged_algorithms():
    names = {'ego_planner_node', 'traj_server'}
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            args = [a.decode(errors='replace') for a in (entry / 'cmdline').read_bytes().split(b'\0') if a]
            # Only executable/script arguments: do not match shell command bodies.
            if any(Path(a).name in names for a in args[:3]):
                found.append(dict(pid=int(entry.name), argv=args))
        except OSError:
            pass
    return found


def stop_process_group(process, event=lambda *a, **k: None):
    """Stop the owned tree, including children with independent POSIX sessions."""
    tracker = track_process(process)
    for sig, duration in ((signal.SIGINT, 5.), (signal.SIGTERM, 2.), (signal.SIGKILL, 1.)):
        members = tracker.snapshot()
        if not members:
            process.wait(timeout=.5)
            return
        # Preserve the normal roslaunch shutdown path: initially signal only
        # the launcher, which forwards SIGINT. Later phases reach every owned
        # descendant directly, even after the launcher has crashed/reparented.
        launcher_only = sig == signal.SIGINT and process.poll() is None
        signalled = set()
        def signal_new_members(current):
            for member in current:
                if launcher_only and member['pid'] != process.pid:
                    continue
                key = (member['pid'], member['start_ticks'])
                if key not in signalled:
                    _signal_identity(member, sig)
                    signalled.add(key)
        signal_new_members(members)
        until = time.monotonic() + duration
        while time.monotonic() < until:
            process.poll()
            members = tracker.snapshot()
            if not members:
                process.wait(timeout=.5)
                return
            # A terminating worker can spawn another child during cleanup.
            # Discover it before declaring the slot free, and signal it in the
            # current escalation phase rather than missing it after one scan.
            signal_new_members(members)
            time.sleep(.025)
    raise RuntimeError('旧计算组还有未退出的子进程，未启动下一组: %d' % process.pid)


class ComputeRunner:
    """Retain shared workers across EGO -> EGO_DOWN -> DOWN transitions.

    A single lifecycle thread removes every unwanted process tree before adding
    workers. Per-worker generations also remember a rapid NONE -> same-group
    restart; coalescing requests must never reuse a cancelled session.
    """
    def __init__(self, command_factory, directory, on_start, on_error, event, env=None):
        self.command_factory = command_factory
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.on_start, self.on_error, self.event = on_start, on_error, event
        self.env = env
        self.condition = threading.Condition()
        self.wanted = 'NONE'
        self.generations = {group: 0 for group in ('FRONT', 'EGO', 'DOWN')}
        self.jobs = {}
        self.closed = False
        self.error = None
        self.thread = threading.Thread(target=self._loop, name='compute-lifecycle', daemon=True)
        self.thread.start()

    @property
    def process(self):
        """Single-worker compatibility for offline diagnostics."""
        with self.condition:
            return next(iter(self.jobs.values()))['process'] if len(self.jobs) == 1 else None

    def request(self, group):
        desired = members(group)
        with self.condition:
            for worker in desired - members(self.wanted):
                self.generations[worker] += 1
            self.wanted = group
            self.condition.notify_all()

    def snapshot(self):
        """Read cached diagnostics; never scan processes on a control/RPC thread."""
        with self.condition:
            workers = {}
            for group, job in self.jobs.items():
                metrics = job.get('metrics', {})
                workers[group] = dict(session_id=job['session'], pid=job['process'].pid,
                    pids=list(metrics.get('pids', [job['process'].pid])),
                    rss_bytes=metrics.get('rss_bytes', 0),
                    peak_rss_bytes=job['peak_rss'],
                    cpu_seconds=metrics.get('cpu_seconds', 0.))
            single = next(iter(workers.values())) if len(workers) == 1 else {}
            return dict(group=group_name(workers), wanted=self.wanted,
                        session_id=single.get('session_id'), pid=single.get('pid'),
                        pids=[pid for value in workers.values() for pid in value['pids']],
                        rss_bytes=sum(value['rss_bytes'] for value in workers.values()),
                        peak_rss_bytes=sum(value['peak_rss_bytes'] for value in workers.values()),
                        cpu_seconds=sum(value['cpu_seconds'] for value in workers.values()),
                        workers=workers, error=self.error)

    def _sample_jobs(self):
        # Only the lifecycle thread samples /proc. A slow scan must not hold
        # the condition needed by a manager status RPC or cancellation request.
        with self.condition:
            jobs = list(self.jobs.items())
        for group, job in jobs:
            records = track_process(job['process']).snapshot()
            metrics = dict(pids=[p['pid'] for p in records],
                           rss_bytes=sum(p['rss_bytes'] for p in records),
                           cpu_seconds=sum(p['cpu_ticks'] for p in records) / os.sysconf('SC_CLK_TCK'))
            with self.condition:
                # A completed scan belongs to this job, never a replacement
                # generation that happens to use the same worker-group name.
                if self.jobs.get(group) is job:
                    job['metrics'] = metrics
                    job['peak_rss'] = max(job['peak_rss'], metrics['rss_bytes'])

    def _stop(self, group):
        with self.condition:
            job = self.jobs.get(group)
        if job is None:
            return
        self.event('compute_stopping', group=group, session_id=job['session'],
                   metrics=self.snapshot()['workers'][group])
        stop_process_group(job['process'])
        self.event('compute_exited', group=group, session_id=job['session'],
                   elapsed_s=time.monotonic() - job['started_at'])
        with self.condition:
            del self.jobs[group]

    def _loop(self):
        try:
            while True:
                with self.condition:
                    for group, job in self.jobs.items():
                        if job['process'].poll() is not None:
                            raise RuntimeError('%s 进程提前退出: %s' % (group, job['process'].returncode))
                    desired = members('NONE' if self.closed else self.wanted)
                    obsolete = [g for g, job in self.jobs.items()
                                if g not in desired or job['generation'] != self.generations[g]]
                if obsolete:
                    for group in obsolete:
                        self._stop(group)
                    continue  # Requests may have changed during slow shutdown.
                self._sample_jobs()
                with self.condition:
                    for group, job in self.jobs.items():
                        if job['process'].poll() is not None:
                            raise RuntimeError('%s 进程提前退出: %s' % (group, job['process'].returncode))
                    desired = members('NONE' if self.closed else self.wanted)
                    # Recheck cancellation before adding anything, even if request()
                    # ran between the previous check and this lock acquisition.
                    if any(g not in desired or job['generation'] != self.generations[g]
                           for g, job in self.jobs.items()):
                        continue
                    missing = desired - self.jobs.keys()
                    if not missing:
                        if self.closed:
                            break
                        self.condition.wait(.1)
                        continue
                    group = next(g for g in ('EGO', 'DOWN', 'FRONT') if g in missing)
                    generation = self.generations[group]
                session = str(uuid.uuid4())
                argv = self.command_factory(group, session)
                with self.condition:
                    if (self.closed or group not in members(self.wanted)
                            or generation != self.generations[group]):
                        continue
                # No lifecycle lock held while calling into the manager lock.
                self.on_start(group, session)
                start = time.monotonic()
                worker_env = dict(os.environ if self.env is None else self.env)
                worker_env[SESSION_ENV] = session
                with (self.directory / ('%s_%s.log' % (group.lower(), session))).open('ab') as log:
                    process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=log,
                                               stderr=subprocess.STDOUT, start_new_session=True,
                                               env=worker_env)
                with self.condition:
                    # Record ownership before scanning /proc, so failure still cleans up.
                    self.jobs[group] = dict(process=process, session=session,
                        started_at=start, peak_rss=0, generation=generation)
                track_process(process, session)
                self.event('compute_started', group=group, session_id=session,
                           pid=process.pid, argv=argv)
        except Exception as exc:
            self.error = str(exc)
            self.on_error(str(exc))
        finally:
            for group in list(self.jobs):
                try:
                    self._stop(group)
                except Exception as exc:
                    self.error = str(exc)
                    self.on_error(str(exc))

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()
        self.thread.join(20.)  # Up to two independently owned process trees.
        if self.thread.is_alive():
            raise RuntimeError('计算组退出仍未完成')
