#!/usr/bin/env python3
"""Persistent lightweight controller. Does not import a detector or model runtime."""
import argparse
import copy
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
import uuid

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import TakeoffLand
from airseafly_real400_msgs.msg import EgoGoalCommand
from std_msgs.msg import Bool, String

from airseafly_real400_mission.configuration import load_config, project_root, runtime_paths
from airseafly_real400_mission.flight_core import FlightCore
from airseafly_real400_mission.payload import actuator_command
from airseafly_real400_mission.compute_groups import members
from airseafly_real400_mission.resources import ComputeRunner, ProcessLock, unmanaged_algorithms
from airseafly_real400_mission.rpc import Handler, Server, Reply
from airseafly_real400_mission.telemetry import EventBuffer, settings
from airseafly_real400_mission.ego_identity import goal_key


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def write_json(path, data):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False))
    temporary.replace(path)


class Manager:
    def __init__(self, paths, no_recording=False):
        self.paths = paths
        self.lock = threading.RLock()
        self.log_lock = threading.Lock()
        self.compute_lock = ProcessLock(paths['compute_lock'], 'REAL400任务与独立调试')
        conflicts = unmanaged_algorithms()
        if conflicts:
            self.compute_lock.close()
            raise RuntimeError('已有未托管算法进程，请先通过其原入口退出: ' + json.dumps(conflicts))
        self.config = load_config()
        if no_recording:
            self.config.setdefault('recording', {})['enabled'] = False
        self.engine = FlightCore(self.config)
        self.pose = None
        self.raw_pose = self.planning_pose = self.raw_to_local = None
        self.planning_stamp = None
        # Reference belongs to this manager/localization session, not each debug job.
        # Never restore it from a previous process or infer it from an airborne pose.
        self.flight_origin = None
        self.native_runtime = None
        self.yaw = 0.
        self.odom_stamp = None
        self.control_state = 'UNKNOWN'
        self.landed = None
        self.land_was_requested = False
        self.priority_landing = False
        self.saw_auto_land = False
        self.active = False
        self.last_output = {}
        self.requested_group = 'NONE'
        self.worker_sessions = {}
        self.ready_workers = set()
        self.ego_cmd = None
        self.ego_stream_started = False
        self.ego_goal = None
        self.ego_goal_request = None
        self.ego_goal_id = None
        self.goal_received = False
        self.last_goal_publish = 0.
        self.dynamic_subscribers = []
        self.ego_goal_pub = None
        self.ego_goal_start_pub = None
        self.running = True
        self.last_status_time = 0.
        self.last_state_key = None
        self.pending_rpc_result = None
        self.payload_debug_busy = False
        self.handoff_pending = False
        self.handoff_since = None
        self.ego_hold_position = None
        self.ego_hold_yaw = 0.
        self.service_threads = set()
        self.event_buffer = EventBuffer()
        self.task_id = None
        self.cancelled_manual_heights = set()
        self.payload_records = []
        self.red_point_job = None
        self.red_point_jobs = []
        self.red_point_session = None
        self.red_point_subscriber = None
        self.red_point_status = dict(requested=False, available=False, error='', session_id=None)
        self.red_point_config = None
        self.red_point_window = None
        self.red_point_request = None
        self.last_file_status_time = 0.
        self.last_tick_wall = None
        self.control_intervals = []
        self.log_dir = project_root() / 'data/mission' / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_%d' % os.getpid())
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.config_snapshot = self.log_dir / 'effective_config.yaml'
        self._save_config()
        self.events = (self.log_dir / 'events.jsonl').open('a', buffering=1)
        self.log_sync, self.log_sync_error = None, None
        try:
            from airseafly_real400_mission.log_durability import DurableLogSync
            self.log_sync = DurableLogSync(self.log_dir)
        except Exception as exc:
            # Log durability is diagnostic; it must never prevent a task start.
            self.log_sync_error = str(exc)
        (self.log_dir.parent / 'latest.txt').write_text(str(self.log_dir) + '\n')
        self.state_pub = rospy.Publisher('/airseafly/mission/state', String, queue_size=1, latch=True)
        self.event_pub = rospy.Publisher('/airseafly/mission/events', String, queue_size=10, latch=True)
        self.land_pub = rospy.Publisher('/px4ctrl/takeoff_land', TakeoffLand, queue_size=1)
        rospy.Subscriber('/Odom_high_freq', Odometry, self.on_odom, queue_size=1)
        from geometry_msgs.msg import TransformStamped
        rospy.Subscriber('/airseafly/native/planning_odometry', Odometry, self.on_planning_odom,
                         queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/airseafly/native/raw_to_local', TransformStamped, self.on_raw_transform, queue_size=1)
        rospy.Subscriber('/px4ctrl/control_state', String, self.on_control_state, queue_size=1)
        rospy.Subscriber('/px4ctrl/landed', Bool, self.on_landed, queue_size=1)
        rospy.Subscriber('/airseafly/mission/worker_ready', String, self.on_worker_ready, queue_size=10)
        self.worker_env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
        self.runner = ComputeRunner(self.worker_command, self.log_dir, self.on_compute_start,
                                    self.on_compute_error, self.event, env=self.worker_env)
        # This topic is latched: callback dependencies must exist before subscribing.
        rospy.Subscriber('/px4ctrl/force_land_active', Bool, self.on_priority_landing, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1. / float(self.config['runtime']['control_rate_hz'])), self.tick)
        self.event('manager_started', pid=os.getpid(), config=self.config)

    def _save_config(self):
        import yaml
        self.config_snapshot.write_text(yaml.safe_dump(self.config, allow_unicode=True, sort_keys=False))

    def _invalidate_red_point(self):
        self.red_point_session = None
        subscriber = getattr(self, 'red_point_subscriber', None)
        self.red_point_subscriber = None
        if subscriber is not None:
            subscriber.unregister()
        publisher = getattr(self, 'red_point_window_pub', None)
        self.red_point_window_pub = None
        if publisher is not None:
            publisher.unregister()
        if hasattr(self.engine, 'set_red_point_session'):
            self.engine.set_red_point_session(None)
        status = getattr(self, 'red_point_status', {})
        status.update(available=False, session_id=None)
        self.red_point_status = status

    def stop_red_point_worker(self):
        """Revoke callbacks immediately; owned camera cleanup is asynchronous."""
        self._invalidate_red_point()
        self.red_point_status['requested'] = False
        job = getattr(self, 'red_point_job', None)
        if job is not None:
            job['stop'].set()

    def start_red_point_worker(self, config, target='red_point', controlled_window=False):
        """An unavailable observer never prevents the recorded mission starting."""
        self.stop_red_point_worker()
        previous = getattr(self, 'red_point_job', None)
        if previous is None and getattr(self, 'red_point_jobs', []):
            previous = self.red_point_jobs[-1]
        self.red_point_job = None
        self.red_point_status = dict(requested=True, available=False, error='', session_id=None)
        options = config.get('delivery_route_options', {}).get('visual_alignment', {})
        source = options.get('red_point_config', project_root()/'flight/red_point.yaml')
        try:
            source = Path(source).expanduser()
            if not source.is_absolute():
                source = project_root()/source
            snapshot = self.log_dir/('red_point_%s.yaml' % self.task_id)
            # Freeze the exact detector/projection configuration without loading
            # OpenCV or a detector into the flight-control process. Later image
            # windows of the same task reuse this copy, so edits made mid-task
            # do not apply.
            if not snapshot.exists():
                snapshot.write_text(source.read_text(encoding='utf-8'), encoding='utf-8')
        except (OSError, ValueError, TypeError) as exc:
            self.red_point_status.update(error=str(exc), config_path=str(source))
            self.event('red_point_unavailable', detail=str(exc), config_path=str(source))
            return
        job = dict(task_id=self.task_id, engine=self.engine, config_path=str(snapshot), target=target,
                   controlled_window=controlled_window,
                   python=config['runtime'].get('python', '/usr/bin/python3'),
                   stop=threading.Event(), finished=threading.Event(), runner=None)
        self.red_point_job = job
        self.red_point_status['config_path'] = str(snapshot)
        self.red_point_jobs = [item for item in getattr(self, 'red_point_jobs', [])
                               if not item['finished'].is_set()]
        self.red_point_jobs.append(job)
        thread = threading.Thread(target=self._run_red_point_worker, args=(job, previous),
                                  name='red-point-lifecycle', daemon=True)
        job['thread'] = thread
        thread.start()

    def freeze_red_point_config(self, config):
        """Copy red_point.yaml at task start; each image window's worker reuses it."""
        self.red_point_config = config
        self.red_point_window = None
        self.red_point_request = None
        options = config.get('delivery_route_options', {}).get('visual_alignment', {})
        try:
            source = Path(options.get('red_point_config', project_root()/'flight/red_point.yaml')).expanduser()
            if not source.is_absolute():
                source = project_root()/source
            (self.log_dir/('red_point_%s.yaml' % self.task_id)).write_text(
                source.read_text(encoding='utf-8'), encoding='utf-8')
        except (OSError, ValueError, TypeError):
            pass  # Reported by the first image window's start, as before.

    def sync_red_point_window(self, output):
        """Warm the camera during flight; detect only in the original image phases.

        Healthy red workers span the drop points. H landing keeps its original
        per-window lifecycle. Failures retain an attempt at each new image
        window, with no restart loop or change to the existing timeout.
        """
        if getattr(self, 'red_point_config', None) is None:
            return
        window = output.get('red_point_window')
        changed = window != getattr(self, 'red_point_window', None)
        self.red_point_window = window
        warm = (self.engine.mode in ('delivery_route', 'drop_test', 'drop_points')
                and output.get('state') in ('TAKEOFF', 'APPROACH', 'DROP')
                and not getattr(self.engine, 'route_returning_home', False)
                and not output.get('phase', '').startswith('LAND_'))
        request = ('landing_pad', window) if window and window.startswith('LAND_') else (
            ('red_point', 'warm') if warm else None)
        previous = getattr(self, 'red_point_request', None)
        self.red_point_request = request
        if changed and window is not None:
            self.event('red_point_window', window=window,
                       target='landing_pad' if window.startswith('LAND_') else 'red_point')
        job = getattr(self, 'red_point_job', None)
        failed = job is None or job['stop'].is_set() or job['finished'].is_set()
        if request is None and previous is not None:
            self.stop_red_point_worker()
        elif request is not None and (request != previous or changed and window and failed):
            self.start_red_point_worker(self.red_point_config, request[0], request[0] == 'red_point')
            if request[0] == 'red_point':
                self.event('red_point_camera_prewarm', phase=output.get('phase'), state=output.get('state'))
        if changed:
            self.publish_red_point_window()

    def publish_red_point_window(self):
        publisher = getattr(self, 'red_point_window_pub', None)
        if publisher is not None:
            publisher.publish(String(data=self.engine.red_point_window() or ''))

    def _run_red_point_worker(self, job, previous):
        runner = None
        try:
            # A replacement can navigate immediately, but its camera owner starts
            # after the old owner exits so it cannot reuse a camera being closed.
            if previous is not None:
                previous['finished'].wait()
            with self.lock:
                if job['stop'].is_set() or self.red_point_job is not job:
                    return
                runner = ComputeRunner(
                    lambda group, session: [job['python'], str(project_root()/
                        'src/airseafly_real400_mission/scripts/red_point_worker.py'),
                        '--config', job['config_path'], '--session', session] +
                        (['--target', job['target']] if job['target'] != 'red_point' else []) +
                        (['--controlled-window'] if job['controlled_window'] else []),
                    self.log_dir,
                    lambda group, session: self.on_red_point_start(job, session),
                    lambda message: self.on_red_point_error(job, message),
                    lambda kind, **fields: self.event('red_point_'+kind,
                        task_id=job['task_id'], **fields), env=self.worker_env)
                job['runner'] = runner
                runner.request('DOWN')
            job['stop'].wait()
        except Exception as exc:
            self.on_red_point_error(job, str(exc))
        finally:
            if runner is not None:
                try:
                    runner.request('NONE')
                    runner.close()
                except Exception as exc:
                    self.on_red_point_error(job, str(exc))
            job['finished'].set()

    def on_red_point_start(self, job, session):
        with self.lock:
            if (self.red_point_job is not job or job['stop'].is_set()
                    or self.engine is not job['engine'] or not self.active):
                return
            self._invalidate_red_point()
            self.red_point_session = session
            if hasattr(self.engine, 'set_red_point_session'):
                self.engine.set_red_point_session(session)
            topic = '/airseafly/mission/red_point_'+session.replace('-', '_')+'/status'
            self.red_point_subscriber = rospy.Subscriber(topic, String,
                lambda message: self.on_red_point_observation(job, session, message), queue_size=1)
            if job.get('controlled_window'):
                self.red_point_window_pub = rospy.Publisher(topic.rsplit('/', 1)[0]+'/window',
                    String, queue_size=1, latch=True)
                self.publish_red_point_window()
            self.red_point_status.update(session_id=session, error='')

    def on_red_point_error(self, job, message):
        with self.lock:
            if self.red_point_job is job:
                self._invalidate_red_point()
                self.red_point_status['error'] = str(message)
            job['stop'].set()
        self.event('red_point_unavailable', task_id=job['task_id'], detail=str(message))

    def on_red_point_observation(self, job, session, message):
        try:
            packet = json.loads(message.data)
            with self.lock:
                if (not isinstance(packet, dict) or self.red_point_job is not job
                        or job['stop'].is_set() or self.engine is not job['engine']
                        or not self.active or self.engine.mode not in ('delivery_route', 'drop_test', 'drop_points')
                        or self.engine.state in ('HOLD', 'LAND', 'COMPLETE')
                        or session != self.red_point_session or packet.get('session_id') != session
                        or packet.get('schema') != 'airseafly.red_point.v1'):
                    return
                if job.get('controlled_window'):
                    window = self.engine.red_point_window()
                    if window is None or packet.get('image_window') != window:
                        return  # A previous point/retake cannot supply this window's image.
                self.engine.on_red_point_observation(packet, now=rospy.get_time())
                self.red_point_status.update(available=True, error=packet.get('image_error', ''))
                if job.get('controlled_window') and self.engine.red_point_window() != window:
                    self.publish_red_point_window()
        except (ValueError, KeyError, TypeError) as exc:
            self.event('red_point_observation_invalid', detail=str(exc))

    def ensure_alignment_trace(self, config, engine=None):
        enabled = getattr(engine or self.engine, 'record_control_trace', False)
        if enabled and getattr(self, 'alignment_trace', None) is None:
            try:
                from airseafly_real400_mission.alignment_trace import AlignmentTrace
                self.alignment_trace = AlignmentTrace(self.log_dir / 'alignment_control.jsonl')
                self.event('alignment_trace_started', **self.alignment_trace.status())
            except Exception as exc:
                self.event('alignment_trace_error', error=str(exc))

    def event(self, kind, _batch=None, **fields):
        item = dict(event=kind, wall_time=time.time(), ros_time=rospy.get_time(), **fields)
        item.setdefault('task_id', self.task_id)
        if kind not in ('runtime_sample', 'config_loaded'):
            visible = copy.deepcopy(item)
            if kind == 'state_changed':
                visible['output'] = {k: item['output'].get(k) for k in ('state_id','state','phase','active_target_id','reason')}
                visible['output']['telemetry'] = {'phase_name': (item['output'].get('telemetry') or {}).get('phase_name')}
            visible = self.event_buffer.append(visible)
            item.update(seq=visible['seq'], boot_id=visible['boot_id'])
            if kind in ('payload_result', 'payload_late_result') and item.get('for_mission') and item['task_id'] == self.task_id:
                self.payload_records.append(copy.deepcopy(item))
            # A newly opened panel receives recent events without touching the
            # flight RPC. History is bounded and contains compact event records.
            if _batch is None:
                packet=dict(visible,recent_events=self.event_buffer.read(max(0,visible['seq']-100))['events'])
                self.event_pub.publish(String(data=json.dumps(packet, ensure_ascii=False, allow_nan=False)))
            else:
                # Only informational route-point announcements use this path.
                # Every record still gets its own sequence and synchronous log
                # line; payload requests/results never enter this batch.
                _batch.append(visible)
        with self.log_lock:
            self.events.write(json.dumps(item, ensure_ascii=False, allow_nan=False) + '\n')

    def publish_point_events(self, batch):
        """Publish all point announcements once, including batches > RPC history."""
        if not batch:
            return
        latest = batch[-1]
        recent = self.event_buffer.read(max(0, latest['seq']-100))['events']
        # Keep every batch member even when it has aged out of the bounded
        # event buffer. Existing subscribers already consume recent_events.
        by_seq = {item['seq']: item for item in recent if item['seq'] <= latest['seq']}
        by_seq.update((item['seq'], item) for item in batch)
        packet = dict(latest, recent_events=[by_seq[seq] for seq in sorted(by_seq)])
        self.event_pub.publish(String(data=json.dumps(packet, ensure_ascii=False, allow_nan=False)))

    def worker_command(self, group, session):
        if group != 'EGO':
            raise ValueError('只保留 EGO 规划工作进程')
        origin = self.engine.origin_xyz or self.flight_origin
        origin_z = float(origin[2]) if origin is not None else 0.
        from airseafly_real400_mission.motion_tuning import motion_limits
        from airseafly_real400_mission.map_tuning import ego_launch_arguments
        limits = motion_limits(self.config)
        self.event('ego_motion_profile', purpose=self.engine.goal_purpose,
                   profile='recorded_route', **limits)
        self.event('ego_height_reference', origin_z=origin_z,
                   reference='takeoff' if origin is not None else 'world_zero',
                   config='ego_planner/launch/advanced_param_exp.xml')
        return ['roslaunch', 'airseafly_real400_mission', 'ego_worker.launch',
                'session:='+session.replace('-', '_'), 'yaw:='+str(self.yaw),
                'height_origin_z:='+str(origin_z), 'max_vel:='+str(limits['max_velocity_mps']),
                'max_acc:='+str(limits['max_acceleration_mps2']),
                'odometry_topic:=/airseafly/native/planning_odometry',
                'cloud_topic:=/airseafly/native/planning_cloud',
                'world_frame:=fast_lio_local_enu'] + ego_launch_arguments(self.config)

    @property
    def ready(self):
        desired = members(self.requested_group)
        return bool(desired) and desired <= self.ready_workers

    @property
    def expected_session(self):
        # Legacy single-worker display field. Combined mode uses worker_sessions.
        return self.worker_sessions.get(self.requested_group)

    def clear_ego_io(self):
        if getattr(self.engine, 'full_native', False) and self.native_runtime is not None:
            self.native_runtime.clear_ego()
        self.ego_cmd = self.ego_goal = None
        self.ego_goal_request = None
        self.ego_stream_started = False
        self.goal_received = False
        for sub in self.dynamic_subscribers:
            sub.unregister()
        self.dynamic_subscribers = []
        if self.ego_goal_pub:
            self.ego_goal_pub.unregister()
            self.ego_goal_pub = None
        if getattr(self, 'ego_goal_start_pub', None):
            self.ego_goal_start_pub.unregister()
            self.ego_goal_start_pub = None

    def request_compute(self, group, force=False):
        old, new = members(self.requested_group), members(group)
        removed = old if force else old - new
        added = new if force else new - old
        for worker in removed | added:
            self.worker_sessions.pop(worker, None)
            self.ready_workers.discard(worker)
            self.engine.on_group_ready(worker, False)
        if 'EGO' in removed or group == 'NONE':
            self.clear_ego_io()  # Discard trajectory BEFORE the next outlet publish.
        self.requested_group = group
        self.runner.request(group)

    def on_compute_start(self, group, session):
        with self.lock:
            if group not in members(self.requested_group):
                return
            self.worker_sessions[group] = session
            self.ready_workers.discard(group)
            self.engine.on_group_ready(group, False)
            if group == 'EGO':
                self.clear_ego_io()
                self.ego_hold_position = tuple(self.pose) if self.pose is not None else None
                self.ego_hold_yaw = self.yaw
                suffix = session.replace('-', '_')
                base = '/airseafly/mission/session_' + suffix
                self.ego_goal_pub = rospy.Publisher(base + '/goal', PoseStamped, queue_size=1)
                self.ego_goal_start_pub = rospy.Publisher(base + '/goal_with_start', EgoGoalCommand, queue_size=1)
                self.dynamic_subscribers = [
                    rospy.Subscriber(base + '/ready', Bool,
                                     lambda msg: self.on_ego_ready(session, msg), queue_size=1),
                    rospy.Subscriber(base + '/goal_position_cmd', EgoGoalCommand,
                                     lambda msg: self.on_ego_cmd(session, msg), queue_size=1,
                                     tcp_nodelay=True),
                    rospy.Subscriber(base + '/goal_received', PoseStamped,
                                     lambda msg: self.on_goal_received(session, msg), queue_size=2),
                    rospy.Subscriber(base + '/goal_failed', PoseStamped,
                                     lambda msg: self.on_goal_failed(session, msg), queue_size=2),
                    rospy.Subscriber(base + '/goal_finished', PoseStamped,
                                     lambda msg: self.on_goal_finished(session, msg), queue_size=2)]

    def on_compute_error(self, message):
        with self.lock:
            # Invalidate the native callback gate before committing the worker
            # failure; a queued callback must not revive the failed session.
            runtime = getattr(self, 'native_runtime', None)
            if runtime is not None:
                runtime.clear_ego()
            self.engine.hold(message)
            self.stop_red_point_worker()
            self.active = True
            self.worker_sessions.clear()
            self.ready_workers.clear()
        self.event('compute_error', detail=message)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        values = [p.x, p.y, p.z]
        if all(math.isfinite(v) for v in values):
            with self.lock:
                self.raw_pose = values
                self.raw_yaw, self.raw_stamp = yaw_of(msg.pose.pose.orientation), msg.header.stamp.to_sec()
                if getattr(self.engine, 'full_native', False):
                    return
                self.pose = values
                self.yaw = yaw_of(msg.pose.pose.orientation)
                self.odom_stamp = msg.header.stamp.to_sec()
                if hasattr(self.engine, 'on_odometry'):
                    self.engine.on_odometry(self.odom_stamp, values)

    def on_raw_transform(self, msg):
        from airseafly_real400_mission.native_planning_geometry import RawToLocal
        transform = RawToLocal.from_message(msg)
        with self.lock:
            # The localization process owns the one immutable transform.
            if self.raw_to_local is None:
                self.raw_to_local = transform

    def on_planning_odom(self, msg):
        from airseafly_real400_mission.native_reference import LIO_FRAME
        from airseafly_real400_mission.native_task_feedback import feedback_source
        if msg.header.frame_id != LIO_FRAME:
            return
        p = msg.pose.pose.position
        values = (p.x, p.y, p.z)
        if not all(math.isfinite(v) for v in values):
            return
        with self.lock:
            self.planning_pose, self.planning_stamp = values, msg.header.stamp.to_sec()
            if getattr(self.engine, 'full_native', False) and feedback_source(getattr(self.engine, 'config', {})) == 'lio':
                self.pose, self.odom_stamp = values, self.planning_stamp

    def on_control_state(self, msg):
        with self.lock:
            self.control_state = msg.data
            if msg.data == 'AUTO_LAND' and self.land_was_requested:
                self.saw_auto_land = True

    def on_priority_landing(self, msg):
        with self.lock:
            enabled = bool(msg.data)
            if enabled == self.priority_landing:
                return
            self.priority_landing = enabled
            if enabled:
                # px4ctrl owns the current-position hover and descent. Do not
                # wait for worker shutdown and do not publish another hold.
                self.engine.hold('独立降落优先接管；本段任务已停止')
                runtime = getattr(self, 'native_runtime', None)
                if runtime is not None:
                    runtime.clear_fixed_trajectory()
                self.stop_red_point_worker()
                self.last_output = self.engine.snapshot()
                self.last_output['control'] = 'NONE'
                self.active = self.handoff_pending = False
                self.handoff_since = None
                self.request_compute('NONE')
            self.event('priority_landing', active=enabled,
                       message='控制器独立降落接管' if enabled else '控制器独立降落接管已结束；任务不自动恢复')

    def on_landed(self, msg):
        with self.lock:
            self.landed = bool(msg.data)
            if msg.data and self.land_was_requested and self.saw_auto_land:
                self.engine.on_landed()

    def _valid_worker(self, data, group):
        return (group in members(self.requested_group)
                and data.get('session_id') is not None
                and data['session_id'] == self.worker_sessions.get(group))

    def on_worker_ready(self, msg):
        try:
            data = json.loads(msg.data)
            group = str(data.get('group', '')).upper()
            with self.lock:
                if not self._valid_worker(data, group) or not data.get('ready'):
                    return
                self.ready_workers.add(group)
                self.engine.on_group_ready(group, True)
                self.event('compute_ready', group=group, session_id=self.worker_sessions[group], detail=data)
                self.record_state_change(self.engine.snapshot())
        except (ValueError, KeyError) as exc:
            rospy.logwarn_throttle(2., str(exc))





    def on_ego_ready(self, session, msg):
        with self.lock:
            if not self._valid_worker({'session_id': session}, 'EGO'):
                return
            if msg.data:
                self.ready_workers.add('EGO')
            else:
                self.ready_workers.discard('EGO')
            self.engine.on_group_ready('EGO', bool(msg.data))
            if msg.data:
                self.event('compute_ready', group='EGO', session_id=session)

    def on_ego_cmd(self, session, msg):
        runtime = getattr(self, 'native_runtime', None)
        if runtime is not None and runtime.full_enabled:
            # The runtime's selected session/source/epoch is the authoritative
            # gate. Never wait for status RPC, process accounting or disk I/O
            # under the manager lock before accepting a time-critical sample.
            # Engine/yaw display updates remain in tick(), under self.lock.
            runtime.feed_ego(session, msg.command, msg.goal)
            return
    def on_goal_failed(self, session, msg):
        with self.lock:
            if not self._valid_worker({'session_id': session}, 'EGO') or self.ego_goal is None:
                return
            goal = self.ego_goal
            if (msg.header.stamp != goal.header.stamp or msg.header.frame_id != goal.header.frame_id
                    or any(getattr(msg.pose.position, k) != getattr(goal.pose.position, k)
                           for k in ('x', 'y', 'z'))):
                return
            failed = getattr(self.engine, 'on_navigation_failed', None)
            if failed is not None and failed(self.ego_goal_id):
                # A matching planning result also proves receipt, even if the
                # separate receipt topic was delayed or lost.
                self.on_goal_received(session, msg)
                self.event('ego_planning_failed', session_id=session, goal_id=self.ego_goal_id,
                           recovery=None)

    def on_goal_finished(self, session, msg):
        with self.lock:
            if not self._valid_worker({'session_id': session}, 'EGO') or self.ego_goal is None:
                return
            goal = self.ego_goal
            if (msg.header.stamp != goal.header.stamp or msg.header.frame_id != goal.header.frame_id
                    or any(getattr(msg.pose.position, k) != getattr(goal.pose.position, k)
                           for k in ('x', 'y', 'z'))):
                return
            finished = getattr(self.engine, 'on_navigation_finished', None)
            if finished is None:
                return
            # Different ROS topics can be delivered out of order. A matching
            # completion also proves receipt of this particular request.
            self.on_goal_received(session, msg)
            if finished(self.ego_goal_id):
                self.event('ego_trajectory_finished', session_id=session, goal_id=self.ego_goal_id,
                           goal_xyz=[goal.pose.position.x, goal.pose.position.y, goal.pose.position.z])

    def on_goal_received(self, session, msg):
        with self.lock:
            if not self._valid_worker({'session_id': session}, 'EGO') or self.ego_goal is None:
                return
            goal = self.ego_goal
            if goal_key(msg) != goal_key(goal) or self.engine.state == 'HOLD':
                return
            if not self.goal_received:
                self.engine.on_goal_received(self.ego_goal_id)
                self.event('goal_received', goal_id=self.ego_goal_id)
                self.record_state_change(self.engine.snapshot())
            self.goal_received = True


    def record_state_change(self, output):
        # Callbacks can enter a short stage that the next control tick completes.
        # Record it here as well as in tick, without waiting for monitor refresh.
        key = (output.get('state'), output.get('phase'), output.get('goal_id'))
        if key != self.last_state_key:
            self.event('state_changed', output=output)
            self.last_state_key = key

    def record_fixed_trajectory(self, trajectory):
        """Persist the actual accepted coefficients once, outside the timer."""
        leg = getattr(trajectory, 'leg_sequence', 0)
        identity = (self.task_id, id(trajectory), leg)
        if getattr(self, 'recorded_fixed_trajectory', None) == identity:
            return
        self.recorded_fixed_trajectory = identity
        route = next((self.engine.config.get(key) for key in ('waypoint_route', 'delivery_route', 'drop_points', 'drop_test')
                      if self.engine.config.get(key)), None)
        route_json = json.dumps(route, sort_keys=True, separators=(',', ':'), allow_nan=False)
        # A delivery task records one file per leg.
        path = self.log_dir / ('fixed_trajectory-'+self.task_id+('-leg%d' % leg if leg else '')+'.json')
        document = dict(format='aquacore_fixed_trajectory', version=1, leg_sequence=leg,
            task_id=self.task_id, reference_source='FIXED_TRAJECTORY',
            frame='fast_lio_local_enu', origin_xyz=list(self.engine.origin_xyz),
            yaw=self.engine.command_yaw, route_sha256=hashlib.sha256(route_json.encode()).hexdigest(),
            route=route, trajectory=trajectory.to_dict(), summary=trajectory.summary())
        try:
            write_json(path, document)
        except OSError as exc:
            # Recording is diagnostic, never a new flight admission gate.
            self.event('fixed_trajectory_record_failed', detail=str(exc), path=str(path))
        else:
            self.event('fixed_trajectory_recorded', path=str(path), route_sha256=document['route_sha256'])

    def tick(self, _event):
        wall = time.monotonic()
        previous = getattr(self, 'last_tick_wall', None)
        if previous is not None:
            if not hasattr(self, 'control_intervals'): self.control_intervals = []
            self.control_intervals.append(wall-previous)
            self.control_intervals = self.control_intervals[-300:]
        self.last_tick_wall = wall
        try:
            with self.lock:
                if not self.active or self.pose is None:
                    output = self.last_output
                else:
                    now = rospy.get_time()
                    if getattr(self.engine, 'uses_native', False):
                        self.native_runtime.update(self.engine)
                    if getattr(self.engine, 'full_native', False):
                        from airseafly_real400_mission.native_task_feedback import feedback_source
                        if feedback_source(self.engine.config) == 'px4':
                            task = self.engine.native_task_feedback
                            if task is not None:
                                self.pose, self.odom_stamp = task['position'], task['pose_stamp']
                    if getattr(self.engine, 'full_native', False) and self.engine.native_feedback:
                        self.yaw = float(self.engine.native_feedback['yaw'])
                    if hasattr(self.engine, 'on_odometry') and self.odom_stamp is not None:
                        self.engine.on_odometry(self.odom_stamp, self.pose)
                    if (getattr(self.engine, 'full_native', False) and
                            getattr(self.engine, 'fixed_trajectory', None) is not None):
                        progress = self.native_runtime.fixed_progress(self)
                        if progress is not None:
                            self.engine.on_fixed_progress(progress)
                    was_taking_off = self.engine.state == 'TAKEOFF'
                    output = self.engine.tick(now, self.pose, self.yaw, self.control_state)
                    if output.get('done') or self.engine.state in ('HOLD', 'LAND', 'COMPLETE'):
                        self.stop_red_point_worker()
                        self.red_point_config = None
                    else:
                        self.sync_red_point_window(output)
                    control = output.get('control', 'NONE')
                    fixed_installed = False
                    if getattr(self.engine, 'full_native', False) and control == 'FIXED_TRAJECTORY':
                        # Start the independent sampler before route telemetry
                        # and diagnostic file writes, without changing its clock
                        # again on subsequent manager ticks.
                        fixed_installed = self.native_runtime.install_fixed_trajectory(
                            self, self.engine.fixed_trajectory, self.engine.command_yaw)
                    if was_taking_off and output['state'] in ('DETECT', 'COMPLETE', 'APPROACH'):
                        self.flight_origin = tuple(self.engine.origin_xyz)
                        self.event('takeoff_origin_recorded', origin_xyz=list(self.flight_origin),
                                   message='起飞完成，后续调试复用本次起飞基准')
                    self.last_output = output
                    desired = output['desired_group']
                    if desired != self.requested_group:
                        self.request_compute(desired)
                        self.event('compute_requested', group=desired)
                    point_events = []
                    for effect in output.get('effects', []):
                        if control == 'FIXED_TRAJECTORY' and effect['type'] == 'delivery_route_point':
                            self.event('effect', effect=effect, _batch=point_events)
                        else:
                            self.publish_point_events(point_events)
                            point_events.clear()
                            self.apply_effect(effect)
                    self.publish_point_events(point_events)
                    if (getattr(self.engine, 'native_active', False) and
                            (output.get('done') or self.engine.state == 'HOLD') and
                            self.engine.native_pending is None and
                            (self.engine.native_held or self.engine.phase == 'LANDED')):
                        self.active = self.handoff_pending = False
                    if (output.get('done') and self.engine.mode in ('delivery_route', 'drop_test', 'drop_points')
                            and output.get('phase') == 'LANDED'):
                        self.active = False  # on_landed already confirmed completion.
                    if getattr(self.engine, 'full_native', False):
                        if control == 'PX4_NATIVE':
                            self.native_runtime.set_reference(output['native_command'], output.get('reference_source', 'HOLD'))
                        elif control == 'FIXED_TRAJECTORY':
                            if fixed_installed:
                                self.record_fixed_trajectory(self.engine.fixed_trajectory)
                        elif control == 'EGO':
                            session = self.worker_sessions.get('EGO')
                            self.native_runtime.select_ego(session, self.ego_goal)
                            reference = self.native_runtime.ego_reference(session)
                            if reference is not None:
                                if not self.ego_stream_started:
                                    self.event('ego_trajectory_executing', goal_id=self.ego_goal_id,
                                               trajectory_id=reference['trajectory_id'])
                                self.ego_stream_started = True
                                self.engine.on_navigation_yaw(reference['yaw'])
                        if output.get('done') or self.engine.state == 'HOLD':
                            self.active = self.handoff_pending = False
                        self.engine.native_px4_command = getattr(self.native_runtime, 'last_converted', None)
                        output.setdefault('native', {})['px4_command'] = self.engine.native_px4_command
                    if self.ego_goal is not None and not self.goal_received and 'EGO' in members(self.requested_group) and 'EGO' in self.ready_workers:
                        if now - self.last_goal_publish >= .25:
                            if self.ego_goal_request is not None:
                                self.ego_goal_start_pub.publish(self.ego_goal_request)
                            else:
                                self.ego_goal_pub.publish(self.ego_goal)
                            self.last_goal_publish = now
                    self.record_state_change(output)
                if time.monotonic() - self.last_status_time >= 1. / settings(self.config)['status_hz']:
                    snapshot = self.status_unlocked()
                    self.state_pub.publish(String(data=json.dumps(snapshot, ensure_ascii=False)))
                    if time.monotonic() - self.last_file_status_time >= .5:
                        write_json(self.log_dir / 'status.json', snapshot)
                        self.event('runtime_sample', compute=snapshot['compute'])
                        self.last_file_status_time = time.monotonic()
                    self.last_status_time = time.monotonic()
        except Exception as exc:
            rospy.logerr_throttle(1., '任务控制错误: %s', exc)
            with self.lock:
                self.engine.hold('任务控制错误: ' + str(exc))
                runtime = getattr(self, 'native_runtime', None)
                if runtime is not None and runtime.reference_source == 'FIXED_TRAJECTORY':
                    runtime.hold_current()
                self.stop_red_point_worker()
                self.event('manager_error', detail=str(exc))






    def stop_failed_takeoff(self, requesting_engine, reason):
        """A failed request never acquires ownership of position commands.

        Leave px4ctrl's existing state alone, including a takeoff whose service
        reply was lost. Do not turn a ground-side rejection into a hover stream.
        """
        with self.lock:
            if self.engine is not requesting_engine or self.engine.state != 'TAKEOFF':
                return
            self.engine.hold(reason)
            self.last_output = self.engine.snapshot()
            self.last_output['control'] = 'NONE'
            self.active = False
            self.handoff_pending = False
            self.handoff_since = None
            self.last_state_key = (self.engine.state, self.engine.phase, None)
            self.event('state_changed', output=self.last_output)

    def apply_effect(self, effect):
        kind = effect['type']
        self.event('effect', effect=effect)
        if kind == 'restart_ego':
            # The explicit NONE -> EGO edge advances ComputeRunner's worker
            # generation even when both requests coalesce before its next loop.
            # Merely forcing manager bookkeeping for EGO would reuse the worker.
            self.request_compute('NONE')
            self.request_compute('EGO')
        elif kind == 'native_owner':
            self.native_runtime.request(self, effect)
        elif kind == 'native_target':
            pass  # Logged above; the fixed target is sampled by the control timer.
        elif kind == 'native_delivery_result':
            result = dict(effect['result'], task_id=self.task_id, recorded_ros_time=rospy.get_time())
            write_json(self.log_dir / ('native_%s_%s_slot%s.json' %
                       (result['profile'], self.task_id, result['slot'])), result)
        elif kind == 'goal':
            msg = PoseStamped()
            msg.header.frame_id = 'fast_lio_local_enu' if getattr(self.engine, 'full_native', False) else 'world'
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = effect['position_xyz']
            yaw = effect.get('yaw', self.yaw)
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2.), math.cos(yaw / 2.)
            self.ego_goal, self.ego_goal_id = msg, effect['goal_id']
            runtime = getattr(self, 'native_runtime', None)
            if runtime is not None:
                runtime.set_ego_goal(msg)
            self.ego_goal_request = None
            if runtime is not None and getattr(self.engine, 'full_native', False):
                reference = runtime.ego_start_reference()
                if reference is not None:
                    request = EgoGoalCommand()
                    request.goal = copy.deepcopy(msg)
                    request.command.header = copy.deepcopy(msg.header)
                    for field in ('position', 'velocity', 'acceleration'):
                        vector = getattr(request.command, field)
                        vector.x, vector.y, vector.z = reference.get(field, (0., 0., 0.))
                    request.command.yaw = reference['yaw']
                    if 'map_inflation_m' in effect:
                        request.apply_map_inflation = True
                        request.map_inflation_m = effect['map_inflation_m']
                    self.ego_goal_request = request
                    self.event('ego_start_reference', goal_id=effect['goal_id'],
                               goal_xyz=list(effect['position_xyz']), reference=reference)
            self.goal_received = False
            self.ego_cmd = None
            self.ego_stream_started = False
            self.last_goal_publish = 0.
        elif kind == 'takeoff':
            self.native_runtime.start_takeoff(self, effect)
        elif kind == 'release':
            self.send_payload(effect['slot'], 'release', effect['request_id'], for_mission=True)
        elif kind == 'land':
            msg = TakeoffLand()
            msg.takeoff_land_cmd = TakeoffLand.LAND
            self.land_was_requested = True
            self.land_pub.publish(msg)

    def send_payload(self, slot, action, request_id, for_mission=False, done=None):
        config = copy.deepcopy(self.config['payload'])
        payload_task_id = self.task_id
        requesting_engine = self.engine
        native_request = for_mission and getattr(requesting_engine, 'native_active', False)
        parameters = rospy.get_param('/mavros/param', {}) if config.get('main_outputs') else None
        command = actuator_command(slot, action, config, parameters)
        deadline = None if native_request else time.monotonic() + float(config['ack_timeout_s'])
        timed_out, dispatched = threading.Event(), threading.Event()
        reservation = (payload_task_id, request_id)
        with self.lock:
            if not hasattr(self, 'payload_blocked_slots'):
                self.payload_blocked_slots = {}
            blocked = for_mission and action == 'release' and slot in self.payload_blocked_slots
            if for_mission and action == 'release' and not blocked:
                self.payload_blocked_slots[slot] = reservation
        # NaN is intentional MAVLink "leave this other actuator unchanged".
        self.event('payload_request', request_id=request_id, task_id=payload_task_id,
                   command={k: v for k, v in command.items() if k != 'params'})
        completed, reported = threading.Event(), threading.Event()
        finished_at = [None]
        result_box = {}
        def call():
            try:
                if blocked:
                    result_box.update(success=False, result=None, outcome='not_sent',
                                      message='该投放口有未确认释放；先人工核实并重新锁止，未发送命令')
                    return
                from mavros_msgs.srv import CommandLong, CommandLongRequest
                if deadline is None:
                    rospy.wait_for_service('/mavros/cmd/command')
                else:
                    rospy.wait_for_service('/mavros/cmd/command', timeout=max(.001, deadline-time.monotonic()))
                req = CommandLongRequest()
                req.broadcast, req.command, req.confirmation = False, 187, 0
                for i, value in enumerate(command['params'], 1):
                    setattr(req, 'param%d' % i, value)
                req.param7 = 0.
                # The dispatch token is revoked under the same lock as timeout
                # and task replacement. A late service discovery cannot release.
                with self.lock:
                    if (timed_out.is_set() or deadline is not None and time.monotonic() >= deadline or self.priority_landing
                            or for_mission and (self.engine is not requesting_engine or self.engine.state == 'HOLD')):
                        result_box.update(success=False, result=None, outcome='not_sent', message='任务已停止或超时，未发送舵机命令')
                        return
                    dispatched.set()
                response = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)(req)
                result_box.update(success=bool(response.success and response.result == 0),
                                  outcome='accepted' if response.success and response.result == 0 else 'rejected',
                                  result=int(response.result), message='飞控接受回执' if response.success and response.result == 0 else '飞控拒绝命令')
            except Exception as exc:
                            result_box.update(success=False,
                                message='舵机命令已接受，但PD档位切换未确认；不重发投放: ' + str(exc))
            except Exception as exc:
                result_box.update(success=False, result=None, outcome='unknown' if dispatched.is_set() else 'not_sent', message=str(exc))
            finally:
                with self.lock:
                    finished_at[0] = time.monotonic()
                    outcome = result_box.get('outcome')
                    on_time = ((deadline is None or finished_at[0] < deadline) and not timed_out.is_set()
                               and getattr(requesting_engine, 'payload_outcome', None) != 'unknown')
                    if (for_mission and action == 'release' and not blocked
                            and self.payload_blocked_slots.get(slot) == reservation
                            and (outcome == 'not_sent' or outcome in ('accepted', 'rejected') and on_time)):
                        self.payload_blocked_slots.pop(slot, None)
                    if action == 'lock' and result_box.get('success'):
                        self.payload_blocked_slots.pop(slot, None)
                    completed.set()
                if timed_out.is_set() or deadline is not None and finished_at[0] >= deadline:
                    # Publish timeout before the late reply, including when the
                    # service thread wins the lock exactly at the deadline.
                    reported.wait()
                    self.event('payload_late_result', request_id=request_id, slot=slot, action=action,
                               task_id=payload_task_id, for_mission=for_mission, **dict(result_box))
                    with self.lock:
                        if for_mission and self.engine is requesting_engine:
                            self.engine.on_payload_result(result_box['success'], request_id, result_box['message'], outcome=result_box.get('outcome'))
                            if self.engine.state == 'HOLD':
                                self.last_output['late_payload_results'] = copy.deepcopy(self.engine.late_payload_results)
        def await_result():
            completed.wait(None if deadline is None else max(0., deadline-time.monotonic()))
            with self.lock:
                if completed.is_set() and (deadline is None or finished_at[0] < deadline):
                    result = dict(result_box)
                else:
                    timed_out.set()
                    result = dict(success=False, result=None,
                                  outcome='unknown' if dispatched.is_set() else 'not_sent',
                                  message='投放流程回执超时；执行结果未知，未重发' if dispatched.is_set() else '等待投放服务超时，未发送命令')
                if for_mission and self.engine is requesting_engine:
                    self.engine.on_payload_result(result['success'], request_id, result['message'], outcome=result.get('outcome'))
                self.event('payload_result', request_id=request_id, slot=slot, action=action, task_id=payload_task_id, for_mission=for_mission, **result)
                reported.set()
            if done:
                done(dict(result, slot=slot, main_output=command['main_output'], action=action, pwm_us=command['pwm_us'], channel=command['channel'], normalized=command['normalized']))
        threading.Thread(target=call, daemon=True).start()
        threading.Thread(target=await_result, daemon=True).start()

    def status_unlocked(self):
        # Finished/debug tasks keep their ledger and progress, while the live
        # aircraft position continues to update. Recompute display errors only.
        output=copy.deepcopy(self.last_output)
        telemetry=output.get('telemetry')
        native = output.get('native')
        display_pose, display_yaw = self.pose, self.yaw
        runtime = getattr(self, 'native_runtime', None)
        if native and native.get('full_flight') and runtime is not None:
            # Paused tasks retain their anchor, but display live task-frame height
            # and error instead of the last active tick's frozen measurements.
            task, feedback = runtime.task_feedback(self.engine.config), runtime.feedback()
            if feedback is not None:
                native['feedback'] = feedback
                display_yaw = float(feedback['yaw'])
            native['task_feedback'] = task
            display_pose = task['position'] if task is not None else None
            if task is not None:
                native['lio_feedback'] = runtime.lio_feedback()
                actual = task['position']
                if native.get('target_xyz') is not None:
                    native['error_xyz'] = [a-b for a,b in zip(native['target_xyz'],actual)]
                if telemetry:
                    telemetry['actual_xyz'] = list(actual)
                    origin = telemetry.get('origin_xyz')
                    if origin is not None:telemetry['relative_height_m'] = actual[2]-origin[2]
                    if telemetry.get('height_target_z') is not None:
                        telemetry['height_error_m'] = telemetry['height_target_z']-actual[2]
        route = output.get('waypoint_route')
        if route and route.get('backend') == 'fixed_min_snap' and display_pose is not None:
            target = route.get('reference_position_xyz')
            if runtime is not None:
                with runtime.mutex:
                    reference = (runtime.reference_command if runtime.full_engine is self.engine else None)
                    if reference is not None:
                        target = list(reference['position'])
            if target is not None:
                # COMPLETE and HOLD keep publishing after the task loop stops;
                # use the accepted hold anchor and fresh measured position.
                error = [a-b for a, b in zip(target, display_pose)]
                route.update(reference_position_xyz=list(target), actual_error_xyz=error)
                if native:
                    native.update(target_xyz=list(target), error_xyz=list(error))
                if telemetry:
                    telemetry.update(height_target_z=target[2], height_error_m=error[2],
                        horizontal_distance_m=math.hypot(*error[:2]),
                        distance_3d_m=math.sqrt(sum(value*value for value in error)))
                    if telemetry.get('navigation_goal_xyz') is not None:
                        telemetry.update(navigation_goal_xyz=list(target), navigation_error_xyz=list(error))
        if telemetry and display_pose is not None and not (output.get('native') or {}).get('active'):
            target_yaw = output.get('yaw', display_yaw)
            telemetry.update(actual_yaw_rad=display_yaw, target_yaw_rad=target_yaw,
                             yaw_error_rad=math.atan2(math.sin(target_yaw-display_yaw),
                                                     math.cos(target_yaw-display_yaw)))
            origin=telemetry.get('origin_xyz')
            telemetry['relative_height_m']=display_pose[2]-origin[2] if origin is not None else None
            goal=telemetry.get('navigation_goal_xyz')
            if goal is not None:
                error=[a-b for a,b in zip(goal,display_pose)]
                telemetry.update(navigation_error_xyz=error,horizontal_distance_m=math.hypot(*error[:2]),distance_3d_m=math.sqrt(sum(v*v for v in error)))
            if telemetry.get('height_target_z') is not None:telemetry['height_error_m']=telemetry['height_target_z']-display_pose[2]
        return dict(ok=True, pid=os.getpid(), active=self.active,
                    priority_landing=self.priority_landing,
                    task_id=self.task_id, boot_id=self.event_buffer.boot_id,
                    last_event_seq=self.event_buffer.seq, payload_records=copy.deepcopy(self.payload_records),
                    sampled_wall_time=time.time(), sampled_ros_time=rospy.get_time(),
                    compute_session=self.expected_session, compute_ready=self.ready,
                    compute_sessions=dict(self.worker_sessions), compute_ready_groups=sorted(self.ready_workers),
                    control_interval_max_s=max(self.control_intervals, default=0.),
                    control_interval_mean_s=(sum(self.control_intervals)/len(self.control_intervals)) if self.control_intervals else 0.,
                    output=output, pose=self.pose, yaw=self.yaw, odom_stamp=self.odom_stamp,
                    navigation_wait_reason=('trajectory' if output.get('control') == 'EGO'
                                            and not self.ego_stream_started else None),
                    flight_origin_xyz=getattr(self, 'flight_origin', None),
                    control_state=self.control_state, landed=self.landed,
                    compute=self.runner.snapshot(), log_dir=str(self.log_dir),
                    log_durability=(self.log_sync.status() if getattr(self, 'log_sync', None) is not None
                                    else dict(running=False, last_error=getattr(self, 'log_sync_error', None))),
                    red_point=copy.deepcopy(getattr(self, 'red_point_status', {})),
                    alignment_trace=(self.alignment_trace.status()
                                     if getattr(self, 'alignment_trace', None) is not None else None),
                    handoff_pending=self.handoff_pending,
                    config_path=self.config.get('_config_path'))

    def dispatch(self, request):
        action = request.get('command')
        if action == 'events':
            return dict(ok=True, **self.event_buffer.read(int(request.get('after', 0)), float(request.get('wait_s', 0.))))
        with self.lock:
            if action == 'status':
                return self.status_unlocked()
            if action == 'hold':
                if 'task_id' in request and request['task_id'] != self.task_id:
                    return dict(ok=False, error=('原调高任务已更换，未停止其他任务' if 'manual_height_id' in request
                                                 else '原任务已更换，未停止其他任务'))
                if 'manual_height_id' in request:
                    if request.get('task_id') != self.task_id:
                        return dict(ok=False, error='原调高任务已更换，未停止其他任务')
                    # Cancellation can arrive before the original RPC worker
                    # obtains this lock. Revoke that explicit request as well.
                    self.cancelled_manual_heights = getattr(self,'cancelled_manual_heights',set())
                    self.cancelled_manual_heights.add(request['manual_height_id'])
                    if (getattr(self.engine,'manual_height',None) or {}).get('request_id')!=request['manual_height_id']:
                        return dict(ok=True, message='原调高请求已取消；未停止其他调高请求')
                self.stop_red_point_worker()
                if self.priority_landing:
                    return dict(ok=True, message='控制器正在独立降落；任务已停止，不重新发布悬停指令')
                if not self.active and self.last_output.get('control') == 'NONE':
                    return dict(ok=True, message='本段已经停止位置输出；保持控制器当前状态')
                if getattr(self.engine, 'full_native', False):
                    self.native_runtime.update(self.engine)
                self.engine.hold('用户请求悬停')
                if getattr(self.engine, 'full_native', False):
                    self.native_runtime.hold_current()
                    self.request_compute('NONE')
                    self.active = self.handoff_pending = False
                    self.last_output = self.engine.snapshot()
                    self.record_state_change(self.last_output)
                    return dict(ok=True, message='任务已停止，继续保持所选任务坐标系中的固定位置')
                self.active = self.handoff_pending = False
                self.request_compute('NONE')
                self.last_output = self.engine.snapshot()
                return dict(ok=True, message='已停止空闲任务')
            if action == 'shutdown':
                # Release the manager and machine compute lock after hover handoff.
                if self.active:
                    return dict(ok=False, error='先完成悬停交接再关闭管理器')
                return Reply(dict(ok=True), lambda: rospy.signal_shutdown('管理器退出'))
            if action not in ('start', 'payload', 'manual_height'):
                return dict(ok=False, error='未知命令')
            if self.priority_landing:
                return dict(ok=False, error='独立降落正在接管，本次不能启动任务或舵机动作')
            busy = self.active and self.engine.state not in ('HOLD', 'COMPLETE', 'IDLE')
            if busy or self.payload_debug_busy:
                return dict(ok=False, error='当前任务仍在执行；先使用 flight/hold.sh 结束该段')
            if self.handoff_pending:
                return dict(ok=False, error='悬停交接正在进行，完成后再开始下一段')
            if action == 'manual_height':
                # Continue the captured takeoff task; never initialize another
                # origin, reload YAML or acquire/reset an owner here.
                if request.get('task_id') != self.task_id:
                    return dict(ok=False, error='原起飞任务已更换，未提交调高')
                if request.get('request_id') in getattr(self,'cancelled_manual_heights',set()):
                    return dict(ok=False, error='该调高请求已取消，不重新执行')
                if not getattr(self.engine, 'full_native', False) or self.native_runtime is None:
                    return dict(ok=False, error='当前任务不是原生起飞悬停，未提交调高')
                self.native_runtime.update(self.engine)
                if (self.native_runtime.manual_takeover or self.native_runtime.watchdog_fault
                        or (self.engine.native_feedback or {}).get('manual_paused')):
                    return dict(ok=False, error='当前任务已由控制器暂停或人工接管，未提交调高')
                event_start = self.event_buffer.seq
                try:
                    output = self.engine.begin_manual_height(request['height_m'], request['request_id'], rospy.get_time())
                except (ValueError, KeyError, TypeError) as exc:
                    return dict(ok=False, error=str(exc))
                self.last_output = output
                self.native_runtime.set_reference(output['native_command'], output['reference_source'])
                self.active = True
                self.record_state_change(output)
                return dict(ok=True, task_id=self.task_id, request_id=request['request_id'],
                            event_start=event_start, log_dir=str(self.log_dir))
            config = (copy.deepcopy(request['resolved_config']) if 'resolved_config' in request else
                      load_config(request.get('config'), request.get('overrides', [])))
            if config['runtime']['ros_master_uri'] != os.environ.get('ROS_MASTER_URI', ''):
                return dict(ok=False, error='管理器已连接另一ROS地址；请使用一致配置')
            if action == 'payload':
                actuator_command(int(request['slot']), request['action'], config['payload'])
                self.config = config
                self._save_config()
                self.event('config_loaded', config=self.config)
                self.payload_debug_busy = True
                done = threading.Event()
                result = {}
                def callback(value):
                    result.update(value)
                    done.set()
                    with self.lock:
                        self.payload_debug_busy = False
                self.send_payload(int(request['slot']), request['action'], str(time.time_ns()), done=callback)
            else:
                mode = request['mode']
                if mode not in ('basic_takeoff', 'waypoint_route', 'delivery_route', 'drop_test', 'drop_points'):
                    return dict(ok=False, error='仅支持基础起飞、普通航点、南京比赛航线、单点及录点投放测试')
                from airseafly_real400_mission.native_task_feedback import feedback_source, validate_task_mode
                try:
                    task_source = validate_task_mode(config, mode)
                except ValueError as exc:
                    return dict(ok=False, error=str(exc))
                if self.pose is None:
                    return dict(ok=False, error='等待现有 /Odom_high_freq 定位；脚本不会自动重启定位')
                if mode == 'delivery_route':
                    from airseafly_real400_mission.native_delivery_route import DeliveryRouteEngine
                    new_engine = DeliveryRouteEngine(config)
                elif mode == 'drop_test':
                    from airseafly_real400_mission.native_single_drop import SingleDropEngine
                    new_engine = SingleDropEngine(config)
                elif mode == 'drop_points':
                    from airseafly_real400_mission.native_drop_points import DropPointsEngine
                    new_engine = DropPointsEngine(config)
                elif mode == 'waypoint_route':
                    from airseafly_real400_mission.native_waypoint_route import WaypointRouteEngine
                    new_engine = WaypointRouteEngine(config)
                elif mode == 'basic_takeoff':
                    from airseafly_real400_mission.native_waypoint_route import NativeTakeoffEngine
                    new_engine = NativeTakeoffEngine(config)
                full_native = getattr(new_engine, 'full_native', False)
                start_pose, start_yaw, start_stamp = self.pose, self.yaw, self.odom_stamp
                if full_native:
                    if task_source == 'lio' and (getattr(self, 'planning_pose', None) is None or getattr(self, 'raw_to_local', None) is None):
                        return dict(ok=False, error='等待原生规划定位和固定世界系变换，请启动全程原生定位入口')
                    if self.native_runtime is None:
                        from airseafly_real400_mission.native_runtime import NativeRuntime
                        self.native_runtime = NativeRuntime()
                    until = time.monotonic()+2.
                    while self.native_runtime.feedback() is None and time.monotonic()<until:
                        time.sleep(.02)
                    feedback = self.native_runtime.feedback()
                    if feedback is None:
                        return dict(ok=False, error='等待原生控制接口')
                    if task_source == 'px4':
                        # A first PX4 task has no frozen frame to read yet.
                        # Register it on the ground before engine.start.
                        try:
                            task = self.native_runtime.prepare_task_frame(config, mode)
                        except (ValueError, KeyError) as exc:
                            return dict(ok=False, error=str(exc))
                        if task is None:
                            return dict(ok=False, error='等待新鲜PX4位置、速度和状态输入')
                        start_pose, start_stamp = task['position'], task['pose_stamp']
                    else:
                        start_pose, start_stamp = self.planning_pose, self.planning_stamp
                    start_yaw = float(feedback['yaw'])
                task_origin = getattr(self, 'flight_origin', None)
                if (full_native and task_source == 'px4' and
                        mode in ('delivery_route', 'waypoint_route', 'basic_takeoff', 'drop_test', 'drop_points') and (feedback or {}).get('landed')):
                    task_origin = None  # Use this ground registration, not the previous flight's origin.
                previous_source = feedback_source(self.engine.config)
                if task_source != previous_source and 'px4' in (task_source, previous_source):
                    # A PX4 takeoff origin cannot seed a subsequent radar task,
                    # even when both tasks use the same native controller.
                    px4_config = config if task_source == 'px4' else self.engine.config
                    if self.native_runtime is None:
                        return dict(ok=False, error='切换任务反馈需要新鲜PX4位置、速度和落地状态')
                    with self.native_runtime.mutex:
                        # Sample progress and landed must describe the same
                        # packet snapshot; a stale landed=true is not evidence
                        # that changing origins is currently safe.
                        fresh_px4 = self.native_runtime.task_feedback(px4_config)
                        feedback = self.native_runtime.feedback()
                    if fresh_px4 is None:
                        return dict(ok=False, error='切换任务反馈需要新鲜PX4位置、速度和落地状态')
                    if mode not in ('delivery_route', 'waypoint_route', 'basic_takeoff', 'drop_test', 'drop_points') or not (feedback or {}).get('landed'):
                        return dict(ok=False, error='切换PX4/雷达任务反馈必须落地后重新起飞，禁止复用另一坐标系起飞原点')
                    task_origin = None
                if full_native != getattr(self.engine, 'full_native', False):
                    task_origin = None
                new_engine.start(mode, start_pose, start_yaw, rospy.get_time(), request.get('target'),
                                 origin=task_origin, slot=request.get('slot'),
                                 stop_after=request.get('stop_after'))
                if getattr(new_engine, 'uses_native', False):
                    if getattr(self, 'native_runtime', None) is None:
                        from airseafly_real400_mission.native_runtime import NativeRuntime
                        self.native_runtime = NativeRuntime()
                    try:
                        self.native_runtime.raw_origin = getattr(self, 'raw_pose', None)
                        self.native_runtime.prepare(new_engine, mode, origin=task_origin)
                    except (ValueError, KeyError) as exc:
                        return dict(ok=False, error=str(exc))
                self.ensure_alignment_trace(config, new_engine)
                if self.runner.error:
                    if self.runner.thread.is_alive() or self.runner.snapshot()['pids']:
                        return dict(ok=False, error='旧计算进程仍在退出，请等待其退出完成')
                    self.runner = ComputeRunner(self.worker_command, self.log_dir, self.on_compute_start,
                                                self.on_compute_error, self.event, env=self.worker_env)
                old_rate = float(self.config['runtime']['control_rate_hz'])
                self.config = config
                if float(config['runtime']['control_rate_hz']) != old_rate:
                    self.timer.shutdown()
                    self.timer = rospy.Timer(rospy.Duration(1. / float(config['runtime']['control_rate_hz'])), self.tick)
                self.task_id = str(uuid.uuid4())
                self.stop_red_point_worker()
                self.engine = new_engine
                self.pose, self.yaw, self.odom_stamp = start_pose, start_yaw, start_stamp
                if full_native:
                    self.native_runtime.enable_full(self)
                if mode in ('delivery_route', 'waypoint_route', 'basic_takeoff', 'drop_test', 'drop_points'):
                    self.flight_origin = None
                elif getattr(new_engine, 'uses_native', False) and task_origin is not None:
                    self.flight_origin = tuple(task_origin)
                self.cancelled_manual_heights = set()
                self.payload_records = []
                event_start = self.event_buffer.seq
                self._save_config()
                self.event('config_loaded', config=self.config)
                self.active = True
                self.handoff_pending = False
                self.handoff_since = None
                self.land_was_requested = False
                self.saw_auto_land = False
                self.last_output = {}
                self.last_state_key = None
                # Force a fresh compute session even when restarting the same debug mode.
                self.request_compute('NONE', force=True)
                self.red_point_config = None
                if mode in ('delivery_route', 'drop_test', 'drop_points'):
                    # Camera startup overlaps takeoff/navigation. Recognition is
                    # still requested only by the existing image phase.
                    self.freeze_red_point_config(config)
                    self.sync_red_point_window(new_engine.snapshot())
                return dict(ok=True, message='已启动 ' + mode, config=self.config, log_dir=str(self.log_dir),
                            event_start=event_start, task_id=self.task_id, boot_id=self.event_buffer.boot_id)
        done.wait(float(config['payload']['ack_timeout_s']) + .5)
        return dict(ok=bool(result.get('success')), result=result, log_dir=str(self.log_dir))

    def close(self):
        self.timer.shutdown()
        with self.lock:
            self.stop_red_point_worker()
            observer_jobs = list(getattr(self, 'red_point_jobs', []))
        if self.native_runtime is not None and hasattr(self.native_runtime, 'close'):
            self.native_runtime.close()
        if getattr(self, 'alignment_trace', None) is not None:
            self.alignment_trace.close()
        self.runner.close()
        for job in observer_jobs:
            job['thread'].join()
        self.compute_lock.close()
        self.events.close()
        if getattr(self, 'log_sync', None) is not None:
            self.log_sync.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--socket', default=runtime_paths()['socket'])
    parser.add_argument('--no-recording', action='store_true',
                        help='启动管理器时不启动相机录像服务')
    args = parser.parse_args()
    paths = runtime_paths()
    paths['socket'] = args.socket
    manager_lock = ProcessLock(paths['manager_lock'], 'REAL400轻量任务管理器')
    rospy.init_node('airseafly_real400_mission', anonymous=False, disable_signals=True)
    manager = Manager(paths, no_recording=args.no_recording)
    if Path(args.socket).exists():
        Path(args.socket).unlink()
    server = Server(args.socket, Handler)
    os.chmod(args.socket, 0o600)
    server.dispatch = manager.dispatch
    threading.Thread(target=server.serve_forever, daemon=True).start()
    def stop(_signum, _frame):
        manager.dispatch(dict(command='hold'))
        # SIGINT of the daemon is an explicit daemon stop. Existing px4ctrl
        # command timeout provides its established AUTO_HOVER handoff.
        rospy.signal_shutdown('任务管理器收到退出信号')
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        rospy.spin()
    finally:
        server.shutdown()
        server.server_close()
        manager.close()
        Path(args.socket).unlink(missing_ok=True)
        manager_lock.close()


if __name__ == '__main__':
    main()
