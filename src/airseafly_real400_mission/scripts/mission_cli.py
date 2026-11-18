#!/usr/bin/env python3
"""Human-oriented commands for the single persistent mission manager."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import threading
import fcntl
import xmlrpc.client
import uuid

from airseafly_real400_mission.configuration import load_config, project_root, runtime_paths
from airseafly_real400_mission.ros_network import apply_ros_address
from airseafly_real400_mission.rpc import request, CommandNotSent
from airseafly_real400_mission.resources import ProcessLock, ResourceBusy
from airseafly_real400_mission.telemetry import TerminalReporter, event_line, settings
from airseafly_real400_mission.monitor_events import EventMonitor
from airseafly_real400_mission.ros_connection import process_registered
from airseafly_real400_mission import native_parameters




class MasterTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection=super().make_connection(host)
        connection.timeout=1.5
        return connection


def require_ros_master(uri):
    # A bounded connectivity check before starting rospy.init_node, which can
    # otherwise wait forever. This is daemon startup, not a flight condition.
    try:
        with xmlrpc.client.ServerProxy(uri,transport=MasterTransport()) as master:
            code,message,_=master.getPid('/airseafly_manager_startup')
        if code!=1:raise RuntimeError(message)
    except (OSError,ValueError,RuntimeError,xmlrpc.client.Error) as exc:
        raise CommandNotSent('ROS主节点未连接：{}。请先启动MAVROS并保持其终端运行。详情：{}'.format(uri,exc)) from exc


def manager_lock_held(path):
    with open(path,'a+') as stream:
        try:fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:return True
        fcntl.flock(stream,fcntl.LOCK_UN)
        return False


def socket_identity(path):
    try:
        value=Path(path).stat()
        return value.st_dev,value.st_ino,value.st_ctime_ns
    except OSError:return None


def manager_registered(status, config):
    try:
        return process_registered(config['runtime']['ros_master_uri'], status.get('pid'),
                                  '/airseafly_real400_mission',
                                  ('/Odom_high_freq', '/px4ctrl/control_state',
                                   '/px4ctrl/landed', '/px4ctrl/force_land_active'))
    except RuntimeError as exc:
        raise CommandNotSent(str(exc)) from exc


def retire_disconnected_manager(paths, status):
    if status.get('active') or status.get('handoff_pending'):
        raise CommandNotSent('旧任务管理器已断开ROS，且旧任务尚未结束；未提交新动作。请先停止旧任务。')
    before = socket_identity(paths['socket'])
    try:
        result = request(paths['socket'], dict(command='shutdown'), 2.)
    except (OSError, RuntimeError):
        # rospy may close the RPC connection while completing shutdown.
        result = None
    if result is not None and not result.get('ok'):
        raise CommandNotSent('旧任务管理器未退出：' + result.get('error', '状态已变化'))
    deadline = time.monotonic() + 6.
    while socket_identity(paths['socket']) == before and manager_lock_held(paths['manager_lock']):
        if time.monotonic() >= deadline:
            raise CommandNotSent('失联的旧任务管理器尚未退出；未提交新动作。')
        time.sleep(.1)
    print('基础ROS连接已变化；空闲旧管理器已退出，正在重新连接定位和控制器。', file=sys.stderr)


def connect_or_start(paths, config):
    require_ros_master(config['runtime']['ros_master_uri'])
    try:
        status = request(paths['socket'], dict(command='status'), 1.)
    except (OSError, RuntimeError):
        status = None
    if status is not None:
        if manager_registered(status, config):
            return status
        retire_disconnected_manager(paths, status)
    startup_guard=None;process=None;connected=False
    before_socket=socket_identity(paths['socket'])
    startup_log=project_root()/'data/mission/manager_startup.log'
    try:
        # Concurrent terminals share one startup attempt. If a manager already
        # holds its lifetime lock, wait for its endpoint rather than spawn again.
        try:
            startup_guard=ProcessLock(paths['manager_lock']+'.startup','管理器启动协调')
        except ResourceBusy:
            pass
        if startup_guard is not None and not manager_lock_held(paths['manager_lock']):
            env=dict(os.environ,ROS_MASTER_URI=config['runtime']['ros_master_uri'],PYTHONUNBUFFERED='1')
            apply_ros_address(env)
            startup_log.parent.mkdir(parents=True,exist_ok=True)
            with startup_log.open('ab') as log:
                manager_command = ['/usr/bin/python3',
                    str(project_root()/'src/airseafly_real400_mission/scripts/mission_manager.py'),
                    '--socket',paths['socket']]
                if config.get('recording', {}).get('enabled') is False:
                    manager_command.append('--no-recording')
                process=subprocess.Popen(manager_command,stdin=subprocess.DEVNULL,stdout=log,
                    stderr=subprocess.STDOUT,start_new_session=True,env=env)
        # Only startup is bounded; this does not time out any flight phase.
        deadline=time.monotonic()+12.
        while time.monotonic()<deadline:
            try:
                status=request(paths['socket'],dict(command='status'),1.)
                if manager_registered(status, config):
                    connected=True
                    return status
            except (OSError,RuntimeError):
                if process is not None and process.poll() is not None and not manager_lock_held(paths['manager_lock']):
                    raise CommandNotSent('管理器启动失败，请查看 '+str(startup_log))
            time.sleep(.1)
        raise CommandNotSent('管理器仍在初始化或未响应；未提交动作。请查看 '+str(startup_log))
    except (OSError,ValueError,RuntimeError) as exc:
        if isinstance(exc,CommandNotSent):raise
        raise CommandNotSent(str(exc)) from exc
    finally:
        try:
            # Only reap our own child before it published a new RPC endpoint.
            # Never kill a pre-existing manager or one now serving another client.
            if (process is not None and not connected and process.poll() is None
                    and socket_identity(paths['socket'])==before_socket):
                process.terminate()
                try:process.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    process.kill();process.wait(timeout=2.)
        finally:
            if startup_guard is not None:startup_guard.close()



def watch(paths,args,config,finish_task=False,after=0,completed_status=None,
          expected_task_id=None,expected_height_id=None):
    output_lock=threading.RLock();stop=threading.Event()
    def emit(*values):
        with output_lock:print(*values,flush=True)
    reporter=TerminalReporter(args.verbose,emit)
    monitor=EventMonitor(after=after,
        emit=lambda item: emit(json.dumps(dict(type='event',data=item),ensure_ascii=False)
                               if args.json else event_line(item)),
        warn=lambda message: emit(json.dumps(dict(type='event_gap',message=message),ensure_ascii=False)
                                  if args.json else message))
    def events():
        while not stop.is_set():
            try:
                result=request(paths['socket'],dict(command='events',after=monitor.poll_after(),wait_s=.25),2.)
                if not stop.is_set():monitor.consume(result)
            except (OSError,ValueError,KeyError,RuntimeError):
                if not stop.is_set():monitor.consume()
                stop.wait(.5)
    thread=threading.Thread(target=events,daemon=True)
    try:
        first_status=request(paths['socket'],dict(command='status'))
        monitor.update_status(first_status)
        thread.start()
        while True:
            status=first_status if first_status is not None else request(paths['socket'],dict(command='status'))
            first_status=None
            monitor.update_status(status)
            out=status.get('output') or {}
            finished=(finish_task and (out.get('done') or out.get('state') in ('COMPLETE','HOLD'))
                      and not status.get('handoff_pending') and status.get('compute',{}).get('group')=='NONE')
            if finished:
                # Drain the already-produced records before printing terminal
                # completion. This only reads RPC/logs; it never delays flight.
                stop.set()
                try:
                    final_events=request(paths['socket'],dict(command='events',after=monitor.after(),wait_s=0),2.)
                except (OSError,ValueError,KeyError,RuntimeError):final_events=None
                monitor.consume(final_events)
            with output_lock:
                if args.json:print(json.dumps(dict(type='status',data=status),ensure_ascii=False),flush=True)
                else:reporter.status(status)
            if expected_task_id is not None and status.get('task_id') != expected_task_id:
                emit('原任务已更换，结束观察；不操作后续任务。')
                return 1
            if expected_height_id is not None:
                if (status.get('task_id')!=expected_task_id
                        or (out.get('manual_height') or {}).get('request_id')!=expected_height_id):
                    emit('原调高请求已更换，结束观察；不恢复原菜单。')
                    return 1
                feedback=(out.get('native') or {}).get('feedback') or {}
                if feedback.get('manual_paused') or feedback.get('watchdog_fault'):
                    emit('控制器已暂停或人工接管，结束调高观察；不恢复投放菜单。')
                    return 1
            if finished:
                if completed_status is not None:
                    completed_status.update(status)
                if not args.json:
                    with output_lock:reporter.summary(status)
                return 0 if out.get('done') or out.get('state')=='COMPLETE' else 1
            time.sleep(1./max(.1,float(settings(config)['terminal_hz'])))
    except KeyboardInterrupt:
        if finish_task:
            body=dict(command='hold')
            if expected_task_id is not None:
                body['task_id'] = expected_task_id
            if expected_height_id is not None:
                body.update(task_id=expected_task_id,manual_height_id=expected_height_id)
            emit(json.dumps(dict(type='hold',data=request(paths['socket'],body)),ensure_ascii=False))
        return 0
    finally:
        stop.set()
        if thread.ident is not None:thread.join(1.)


def main():
    parser = argparse.ArgumentParser(description='南京任务管理器的状态与保持接口')
    parser.add_argument('command', choices=('status', 'hold', 'shutdown'))
    args = parser.parse_args()
    result = request(runtime_paths()['socket'], dict(command=args.command))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get('ok') else 1

if __name__ == '__main__':
    sys.exit(main())
