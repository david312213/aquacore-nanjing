#!/usr/bin/env python3
"""Offline manager wiring tests; every ROS endpoint and process runner is fake.

Manager.__init__ is deliberately bypassed: no timers, subprocesses, ROS sockets,
MAVROS clients, or flight-controller connections can be created by these tests.
"""
import copy
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
import types
import unittest
from unittest import mock

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
from airseafly_real400_mission.flight_core import FlightCore
from airseafly_real400_mission.configuration import load_config


class Clock:
    now = 100.


class Stamp:
    def __init__(self, value=0.):
        self.value = float(value)

    @classmethod
    def now(cls):
        return cls(Clock.now)

    def to_sec(self):
        return self.value

    def __eq__(self, other):
        return isinstance(other, Stamp) and self.value == other.value


class Vector:
    def __init__(self, x=0., y=0., z=0.):
        self.x, self.y, self.z = x, y, z


class CheckSearchPointsRequest:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=Stamp(), frame_id='')
        self.points = []


class PoseStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=Stamp(), frame_id='')
        self.pose = types.SimpleNamespace(position=Vector(),
                                          orientation=types.SimpleNamespace(x=0., y=0., z=0., w=1.))


class PositionCommand:
    TRAJECTORY_STATUS_READY = 1

    def __init__(self):
        self.header = types.SimpleNamespace(stamp=Stamp(), frame_id='')
        self.position, self.velocity, self.acceleration = Vector(), Vector(), Vector()
        self.yaw = 0.
        self.yaw_dot = 0.
        self.trajectory_flag = 0
        self.trajectory_id = 0


class TakeoffLand:
    LAND = 2

    def __init__(self):
        self.takeoff_land_cmd = 0


class EgoGoalCommand:
    def __init__(self, goal=None, command=None):
        self.goal = goal
        self.command = command or PositionCommand()


def bound_command(manager, command):
    return EgoGoalCommand(copy.deepcopy(manager.ego_goal), command)


class String:
    def __init__(self, data=''):
        self.data = data


class Bool:
    def __init__(self, data=False):
        self.data = data


class PayloadPDRequest:
    STATUS, RESET, RELEASE = 0, 1, 2

    def __init__(self):
        self.operation, self.slot, self.loaded_mask = 0, 0, 0


class Publisher:
    def __init__(self, *args, **kwargs):
        self.messages = []
        self.unregistered = False

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))

    def unregister(self):
        self.unregistered = True


class Subscriber:
    def __init__(self, *args, **kwargs):
        self.unregistered = False

    def unregister(self):
        self.unregistered = True


class Timer:
    def __init__(self, duration, callback):
        self.duration = duration
        self.callback = callback
        self.shutdown_calls = 0

    def shutdown(self):
        self.shutdown_calls += 1


class Runner:
    def __init__(self):
        self.group = 'NONE'
        self.requests = []
        self.error = None
        self.thread = types.SimpleNamespace(is_alive=lambda: False)

    def request(self, group):
        self.group = group
        self.requests.append(group)

    def snapshot(self):
        return {'group': self.group, 'pids': []}


def module(name, **values):
    result = types.ModuleType(name)
    result.__dict__.update(values)
    return result


ROS = module('rospy', Time=Stamp, get_time=lambda: Clock.now,
             get_param=lambda name, default=None: default,
             Publisher=Publisher, Subscriber=Subscriber, Timer=Timer, Duration=lambda seconds: seconds,
             logwarn_throttle=mock.Mock(), logerr_throttle=mock.Mock(), signal_shutdown=mock.Mock(),
             wait_for_service=mock.Mock(), ServiceProxy=mock.Mock())
STUBS = {'rospy': ROS,
         'geometry_msgs': module('geometry_msgs'),
         'geometry_msgs.msg': module('geometry_msgs.msg', PoseStamped=PoseStamped, Point=Vector),
         'nav_msgs': module('nav_msgs'),
         'nav_msgs.msg': module('nav_msgs.msg', Odometry=object),
         'quadrotor_msgs': module('quadrotor_msgs'),
         'quadrotor_msgs.msg': module('quadrotor_msgs.msg', PositionCommand=PositionCommand,
                                     TakeoffLand=TakeoffLand),
         'std_msgs': module('std_msgs'),
         'std_msgs.msg': module('std_msgs.msg', Bool=Bool, String=String),
         'mavros_msgs': module('mavros_msgs'),
         'mavros_msgs.srv': module('mavros_msgs.srv', CommandLong=object,
                                  CommandLongRequest=type('CommandLongRequest', (), {})),
         'px4ctrl': module('px4ctrl'),
         'px4ctrl.srv': module('px4ctrl.srv', PayloadPD=object, PayloadPDRequest=PayloadPDRequest),
         'airseafly_real400_msgs': module('airseafly_real400_msgs'),
         'airseafly_real400_msgs.msg': module('airseafly_real400_msgs.msg', EgoGoalCommand=EgoGoalCommand),
         'airseafly_real400_msgs.srv': module('airseafly_real400_msgs.srv', MissionTakeoff=object,
                                            CheckSearchPoints=object, CheckSearchPointsRequest=CheckSearchPointsRequest)}
with mock.patch.dict(sys.modules, STUBS):
    spec = importlib.util.spec_from_file_location('mission_manager_under_test', PACKAGE / 'scripts/mission_manager.py')
    MANAGER = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(MANAGER)


def config():
    value = load_config()
    value['runtime'] = {'ros_master_uri': 'http://offline.invalid:11341',
                        'control_rate_hz': 30., 'python': '/never/run/python'}
    # Keep these legacy transport fixtures independent of current mechanism tuning.
    for key in ('main_outputs', 'pwm_command_max_us', 'reset_pwm_us'):
        value['payload'].pop(key, None)
    value['payload'].update(released_pwm_us_by_slot={}, pwm_max_us_by_slot={},
                            actuator_indices={'1': 1, '2': 2, '3': 3},
                            pwm_min_us=1100, pwm_center_us=1500, pwm_max_us=2200,
                            locked_pwm_us=1260, released_pwm_us=2200)
    value['_config_path'] = '/offline/mission.yaml'
    return value


def make_manager(mode=None):
    manager = MANAGER.Manager.__new__(MANAGER.Manager)
    manager.paths = {}
    manager.lock, manager.log_lock = threading.RLock(), threading.Lock()
    manager.config = config()
    manager.engine = FlightCore(manager.config)
    manager.pose, manager.yaw, manager.odom_stamp = [0., 0., .8], 0., Clock.now
    manager.flight_origin = None
    manager.control_state = 'AUTO_HOVER'
    manager.landed = None
    manager.land_was_requested = manager.saw_auto_land = False
    manager.priority_landing = False
    manager.active = False
    manager.last_output = {}
    manager.requested_group = 'NONE'
    manager.worker_sessions = {}
    manager.ready_workers = set()
    manager.ego_cmd = manager.ego_goal = manager.ego_goal_id = None
    manager.goal_received = False
    manager.last_goal_publish = 0.
    manager.dynamic_subscribers = []
    manager.ego_goal_pub = None
    manager.running = True
    manager.last_status_time = float('inf')
    manager.last_state_key = None
    manager.pending_rpc_result = None
    manager.latest_front = manager.latest_down = None
    manager.payload_debug_busy = False
    manager.handoff_pending = False
    manager.handoff_since = None
    manager.ego_hold_position = None
    manager.ego_hold_yaw = 0.
    manager.service_threads = set()
    manager.event_buffer = MANAGER.EventBuffer()
    manager.task_id = "offline-task"
    manager.payload_records = []
    manager.last_file_status_time = float("inf")
    manager.control_intervals = []
    manager.observation_pub = Publisher()
    manager.event_pub = Publisher()
    manager.worker_env = {}
    manager.log_dir = Path('/offline/not-created')
    manager.config_snapshot = manager.log_dir / 'effective_config.yaml'
    manager._save_config = mock.Mock()
    manager.events_seen = []
    manager.event = lambda kind, **values: manager.events_seen.append((kind, values))
    manager.position_pub, manager.state_pub, manager.land_pub = Publisher(), Publisher(), Publisher()
    manager.runner = Runner()
    manager.timer = Timer(1. / manager.config['runtime']['control_rate_hz'], manager.tick)
    if mode is not None:
        manager.engine.start(mode, manager.pose, manager.yaw, Clock.now)
        manager.active = True
    return manager


def packet(group, session='current', stamp=None, error=None, **values):
    result = dict(group=group, session_id=session, stamp_s=Clock.now if stamp is None else stamp,
                  width=1280, height=720, error=error)
    result.update(values)
    return String(json.dumps(result))


def make_ready(manager, group, session='current'):
    manager.requested_group = group
    manager.on_compute_start(group, session)
    if group == 'EGO':
        manager.on_ego_ready(session, Bool(True))
    else:
        manager.on_worker_ready(packet(group, session, ready=True))


def tick(manager, dt=.1):
    Clock.now += dt
    manager.tick(None)
    return manager.last_output



from types import SimpleNamespace as NS
import io
import xmlrpc.client
import copy as deepcopy_module
from airseafly_real400_mission.native_runtime import NativeRuntime
from airseafly_real400_mission.native_reference import LioReference, LIO_FRAME
from airseafly_real400_mission.native_freshness import SourceProgress

class Target:
    FRAME_LOCAL_NED = 1
    IGNORE_VX, IGNORE_VY, IGNORE_VZ = 8, 16, 32
    IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ, IGNORE_YAW_RATE = 64, 128, 256, 2048

    def __init__(self):
        self.header = NS(stamp=Stamp(), frame_id='')
        self.position, self.velocity, self.acceleration_or_force = Vector(), Vector(), Vector()
        self.yaw = self.yaw_rate = 0.


class ImmediateThread:
    def __init__(self, target, **_kwargs):
        self.target = target

    def start(self):
        self.target()


def runtime_for(manager, h):
    runtime = NativeRuntime.__new__(NativeRuntime)
    runtime.mutex, runtime.service_mutex = threading.RLock(), threading.Lock()
    runtime.packet = deepcopy_module.deepcopy(h.engine.native_feedback)
    runtime.received, runtime.origin, runtime.lio_origin = 0., None, None
    runtime.reference = LioReference()
    runtime.reference.feed(h.now, h.p, h.velocity)
    runtime.full_enabled = runtime.manual_takeover = runtime.seen_offboard = False
    runtime.full_engine = runtime.manager = None
    runtime.reference_command = runtime.ego_command = None
    runtime.reference_source, runtime.ego_session = 'HOLD', None
    runtime.start_epoch = 0
    # Real MAVROS callbacks provide independent timestamps for position and
    # velocity; runtime must not substitute the status packet's single stamp.
    runtime.px4_pose_sample = (h.now, runtime.packet['position'])
    runtime.px4_velocity_sample = (h.now, runtime.packet['velocity'])
    runtime.full_timer, runtime.trace = None, io.StringIO()
    runtime.last_send_wall = None
    runtime.input_progress = {key: SourceProgress() for key in ('lio','pose','velocity','status')}
    runtime.reference_progress = SourceProgress()
    runtime.watchdog_fault, runtime.watchdog_holding = '', False
    refresh_inputs(runtime)
    runtime.Target, runtime.publisher = Target, Publisher()
    runtime.parameters = {}
    def set_param(key, value):
        xmlrpc.client.dumps(('/native_manager_test', key, value), methodname='setParam')
        runtime.parameters[key] = deepcopy_module.deepcopy(value)
    runtime.rospy = NS(Time=Stamp, get_time=lambda: Clock.now,
        get_param=lambda key, default=None: runtime.parameters.get(key, default),
        set_param=set_param,
        Timer=Timer, Duration=lambda value: value, logerr_throttle=mock.Mock(),
        wait_for_service=mock.Mock(), ServiceProxy=mock.Mock(), is_shutdown=lambda:False)
    manager.native_runtime = runtime
    return runtime


def refresh_inputs(runtime):
    # Fresh inert sensor streams while the service-sequencing clock advances.
    for progress in runtime.input_progress.values(): progress.observe(Clock.now, Clock.now)
    runtime.packet['stamp'] = Clock.now
    latest = runtime.reference.latest
    runtime.reference.feed(Clock.now, latest['position'], latest['velocity'])
    runtime.px4_pose_sample = (Clock.now, runtime.px4_pose_sample[1])
    runtime.px4_velocity_sample = (Clock.now, runtime.px4_velocity_sample[1])


