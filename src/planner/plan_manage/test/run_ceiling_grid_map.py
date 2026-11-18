#!/usr/bin/env python3
"""Execute the compiled map test on a private loopback ROS master; no hardware."""
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import xmlrpc.client
from test_ego_restart_integration import stop_group, wait_for


def main():
    binary = Path(sys.argv[1]).resolve()
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    assert port not in (11311, 11341)
    logs = Path(tempfile.mkdtemp(prefix='real400-ceiling-grid-'))
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
               ROS_IP='127.0.0.1', ROS_LOG_DIR=str(logs))
    env.pop('ROS_HOSTNAME', None)
    env.pop('ROS_NAMESPACE', None)
    master = None
    with (logs / 'master.log').open('wb') as output:
        try:
            master = subprocess.Popen(['roscore', '-p', str(port)], env=env,
                                      stdout=output, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            def ready():
                try:
                    return xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/map_test')[0] == 1
                except (OSError, xmlrpc.client.Error):
                    return False
            wait_for(ready, 'private test master', 15., (master,))
            subprocess.run([str(binary)], env=env, check=True, timeout=20.)
        finally:
            stop_group(master)
    print('Private master exited; logs:', logs)


if __name__ == '__main__':
    main()
