"""Best-effort crash durability, independent of flight/control callbacks.

The existing writers flush complete lines to the kernel. This worker syncs
changed files and directory entries periodically; it never owns their streams
or takes a manager/runtime lock. Abrupt power loss can still lose the interval
since the last completed sync, and storage hardware may provide weaker guarantees.
"""
import os
from pathlib import Path
import threading
import time


LOG_FILES = ('events.jsonl', 'native_trajectory.jsonl', 'alignment_control.jsonl',
             'alignment_pd.jsonl', 'status.json', 'effective_config.yaml')


class DurableLogSync:
    def __init__(self, directory, interval_s=.5):
        if interval_s <= 0:
            raise ValueError('log sync interval must be positive')
        self.directory, self.interval_s = Path(directory), float(interval_s)
        self._stop, self._status_lock = threading.Event(), threading.Lock()
        self._signatures = {}
        self._directory_pending = self._parent_pending = True
        self._status = dict(interval_s=self.interval_s, synced_files=0, error_count=0,
                            last_sync_wall_time=None, last_error=None)
        self.worker = threading.Thread(target=self._run, name='mission-log-sync', daemon=True)
        self.worker.start()

    @staticmethod
    def _signature(stat):
        return stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns

    def _sync_file(self, path):
        # Opening separately avoids touching a writer's Python buffering/lock.
        fd = os.open(str(path), os.O_RDONLY)
        try:
            signature = self._signature(os.fstat(fd))
            if self._signatures.get(path.name) == signature:
                return False
            os.fsync(fd)
            # Record the pre-sync signature: a concurrent write/replace must
            # trigger another sync on the next pass, not disappear in this one.
            self._signatures[path.name] = signature
            return True
        finally:
            os.close(fd)

    @staticmethod
    def _sync_directory(path):
        fd = os.open(str(path), os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
        try:
            os.fsync(fd)
        finally:
            os.close(fd)

    def _sync_once(self):
        errors, synced = [], 0
        for name in LOG_FILES:
            try:
                if self._sync_file(self.directory / name):
                    synced += 1
                    self._directory_pending = True
            except FileNotFoundError:
                pass  # Optional trace streams may never have been opened.
            except Exception as exc:
                errors.append(name+': '+str(exc))
        for attr, path in (('_directory_pending', self.directory),
                           ('_parent_pending', self.directory.parent)):
            if getattr(self, attr):
                try:
                    self._sync_directory(path)
                    setattr(self, attr, False)
                except Exception as exc:
                    errors.append(str(path)+': '+str(exc))
        with self._status_lock:
            self._status['synced_files'] += synced
            self._status['error_count'] += len(errors)
            self._status['last_error'] = '; '.join(errors) if errors else None
            if synced and not errors:
                self._status['last_sync_wall_time'] = time.time()

    def _run(self):
        while True:
            closing = self._stop.wait(self.interval_s)
            self._sync_once()
            if closing:
                return

    def status(self):
        with self._status_lock:
            return dict(self._status, running=self.worker.is_alive())

    def close(self, timeout=2.):
        """Shutdown-only final sync; never called from a control callback."""
        self._stop.set()
        self.worker.join(timeout=timeout)
        return not self.worker.is_alive()
