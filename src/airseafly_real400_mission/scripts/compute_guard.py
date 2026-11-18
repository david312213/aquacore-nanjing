#!/usr/bin/env python3
"""Legacy launcher wrapper using the same machine-wide lock as the manager."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import uuid

from airseafly_real400_mission.configuration import runtime_paths
from airseafly_real400_mission.resources import (
    ProcessLock, SESSION_ENV, stop_process_group, track_process, unmanaged_algorithms,
)


def main():
    argv = sys.argv[1:]
    if argv and argv[0] == '--':
        argv.pop(0)
    if not argv:
        raise ValueError('需要原启动脚本路径')
    lock = ProcessLock(runtime_paths()['compute_lock'], '旧入口 ' + Path(argv[0]).name)
    conflicts = unmanaged_algorithms()
    if conflicts:
        raise RuntimeError('发现已有未托管算法: ' + str(conflicts))
    env = dict(os.environ, AIRSEAFLY_COMPUTE_LOCKED='1')
    session = str(uuid.uuid4())
    env[SESSION_ENV] = session
    process = subprocess.Popen(['sh'] + argv, env=env, start_new_session=True)
    track_process(process, session)
    def stop(_sig, _frame):
        stop_process_group(process)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        return process.wait()
    finally:
        stop_process_group(process)
        lock.close()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print('计算互斥:', exc, file=sys.stderr)
        sys.exit(1)
