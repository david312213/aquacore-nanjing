"""Exercise the full-flight ROS adapter with message-shaped inputs and fake RPCs.

No ROS sockets, real timers, mode commands or motor commands are created.
"""
from copy import deepcopy
import io
import json
import math
import os
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import xmlrpc.client

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
sys.path.insert(0, str(PACKAGE/'test'))
from airseafly_real400_mission import native_runtime as runtime_module
from airseafly_real400_mission.native_runtime import NativeRuntime
from airseafly_real400_mission.native_reference import LIO_FRAME
from ros_fixtures import make_manager, MANAGER, STUBS, ROS, config as manager_config


def xyz(value=(0., 0., 0.)):
    return NS(x=value[0], y=value[1], z=value[2])


class Stamp:
    def __init__(self, stamp): self.stamp = stamp
    def to_sec(self): return self.stamp


class Target:
    FRAME_LOCAL_NED = 1
    IGNORE_VX, IGNORE_VY, IGNORE_VZ = 8, 16, 32
    IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ, IGNORE_YAW_RATE = 64, 128, 256, 2048
    def __init__(self):
        self.header = NS()
        self.position, self.velocity, self.acceleration_or_force = xyz(), xyz(), xyz()


class Timer:
    def __init__(self, period, callback):
        self.period, self.callback, self.stopped = period, callback, False
    def shutdown(self): self.stopped = True
    def step(self):
        if not self.stopped: self.callback(None)


class Engine:
    full_native = True
    def __init__(self):
        self.native_feedback = self.native_lio_feedback = self.native_origin = None
        self.position = self.origin_xyz = (1., 2., 3.)
        self.state = 'TAKEOFF'
        self.holds = []
        self.started = False
    def on_native_feedback(self, packet): self.native_feedback = packet
    def on_native_lio_feedback(self, packet): self.native_lio_feedback = packet
    def native_ready(self, now): return self.native_feedback is not None and self.native_lio_feedback is not None
    def begin_native_flight(self): self.started = True
    def hold(self, reason): self.state = 'HOLD'; self.holds.append(reason)
    def snapshot(self): return dict(state=self.state, control='PX4_NATIVE')


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.parameters, self.sent, self.timers, self.subscriptions, self.rpc_calls = {}, [], [], [], []
        self.now, self.wall = 10., 0.
        clock_patch = patch.object(runtime_module.time, 'monotonic', lambda: self.wall)
        clock_patch.start(); self.addCleanup(clock_patch.stop)
        def publisher(*args, **kwargs):
            return NS(publish=lambda message: self.sent.append(deepcopy(message)))
        def timer(period, callback):
            value = Timer(period, callback); self.timers.append(value); return value
        self.wait_hook = lambda name: None
        def service(name, _type):
            def call(*args, **kwargs):
                self.rpc_calls.append((name, args, kwargs))
                return NS(accepted=True, success=True, mode_sent=True, message='ok')
            return call
        def set_param(name, value):
            # ROS parameters cross XML-RPC; a dict-only stub accepts NumPy
            # scalars that the real XML-RPC client cannot serialize.
            xmlrpc.client.dumps(('/native_runtime_test', name, value), methodname='setParam')
            self.parameters[name] = deepcopy(value)
        self.ros = NS(Publisher=publisher,
            Subscriber=lambda name, _type, callback, **kwargs: self.subscriptions.append((name, callback)),
            Time=NS(now=lambda: Stamp(self.now)), get_time=lambda: self.now,
            Duration=lambda value: value, Timer=timer,
            get_param=lambda name, default=None: self.parameters.get(name, default),
            set_param=set_param,
            logerr_throttle=lambda *args: None, is_shutdown=lambda: False,
            wait_for_service=lambda name, **kwargs: self.wait_hook(name), ServiceProxy=service)
        self.stubs = {'rospy': self.ros, 'std_msgs.msg': NS(String=object),
            'mavros_msgs.msg': NS(PositionTarget=Target), 'nav_msgs.msg': NS(Odometry=object),
            'geometry_msgs.msg': NS(PoseStamped=object, TwistStamped=object),
            'mavros_msgs.srv': NS(SetMode=object, CommandBool=object),
            'px4ctrl.srv': NS(ControlOwner=object)}
        self.patcher = patch.dict(sys.modules, self.stubs)
        self.patcher.start(); self.addCleanup(self.patcher.stop)
        self.runtime = NativeRuntime()
        self.runtime.trace = io.StringIO()
        self.engine = Engine()
        self.manager = NS(engine=self.engine, lock=threading.RLock(), active=True,
            handoff_pending=True, priority_landing=False, requested=[], events=[])
        self.manager.request_compute = self.manager.requested.append
        self.manager.event = lambda kind, **values: self.manager.events.append((kind, values))
        self.manager.record_state_change = lambda output: None
        self.runtime.packet = dict(pose_received=True, pose_stamp=10., position=(101., 202., 303.),
            velocity=(50., 60., 70.), yaw=.4, owner='PX4_NATIVE', offboard=True,
            mode='OFFBOARD', armed=True, landed=False)
        self.runtime.reference.feed(10., (1., 2., 3.), (.1, .2, .3))
        self.runtime.input_progress['lio'].observe(10., self.wall)
        self.feed_status()
        self.feed_px4(10., (101., 202., 303.), 10., (.6, -.2, .4))

    def feed_status(self):
        packet = dict(self.runtime.packet, stamp=self.now)
        self.runtime._feedback(NS(data=json.dumps(packet)))

    def refresh_inputs(self, skip=()):
        if 'status' not in skip: self.feed_status()
        if 'lio' not in skip:
            latest = self.runtime.reference.latest
            self.runtime._lio_feedback(NS(header=NS(stamp=Stamp(self.now)),
                pose=NS(pose=NS(position=xyz(latest['position']), orientation=NS(x=0.,y=0.,z=0.,w=1.))),
                twist=NS(twist=NS(linear=xyz(latest['velocity'])))))
        if 'pose' not in skip:
            self.runtime._px4_pose(NS(header=NS(stamp=Stamp(self.now)),
                pose=NS(position=xyz(self.runtime.px4_pose_sample[1]))))
        if 'velocity' not in skip:
            self.runtime._px4_velocity(NS(header=NS(stamp=Stamp(self.now)),
                twist=NS(linear=xyz(self.runtime.px4_velocity_sample[1]))))

    def advance(self, seconds, skip=()):
        self.now += seconds; self.wall += seconds
        self.refresh_inputs(skip)

    def feed_px4(self, p_stamp, position, v_stamp, velocity):
        self.runtime._px4_pose(NS(header=NS(stamp=Stamp(p_stamp)), pose=NS(position=xyz(position))))
        self.runtime._px4_velocity(NS(header=NS(stamp=Stamp(v_stamp)), twist=NS(linear=xyz(velocity))))

    def ego(self, p=(1.3, 1.8, 3.6), v=(.2, -.1, .05), a=(.02, -.03, .04), frame=LIO_FRAME):
        return NS(header=NS(stamp=Stamp(9.99), frame_id=frame), position=xyz(p), velocity=xyz(v),
                  acceleration=xyz(a), yaw=.7, yaw_dot=.12, trajectory_id=43)

    def activate(self):
        self.runtime.enable_full(self.manager)
        # Timer tests start after the owner RPC and its matching status sample;
        # startup/cancellation RPC sequencing is exercised separately below.
        self.runtime.full_owner_since = self.now
        self.runtime.packet['stamp'] = self.now
        return self.timers[0]

    def test_ego_forwards_all_fields_and_each_error_is_in_radar_coordinates(self):
        timer = self.activate()
        self.runtime.select_ego('session-A')
        message = self.ego()
        self.assertTrue(self.runtime.feed_ego('session-A', message))
        timer.step()
        target = self.sent[-1]
        np.testing.assert_allclose(tuple(vars(target.position).values()), (101.3, 201.8, 303.6))
        np.testing.assert_allclose(tuple(vars(target.velocity).values()), (.7, -.5, .15))
        np.testing.assert_allclose(tuple(vars(target.acceleration_or_force).values()), (.02, -.03, .04))
        self.assertEqual((target.yaw, target.yaw_rate), (.7, .12))
        self.assertEqual(target.coordinate_frame, Target.FRAME_LOCAL_NED)
        self.assertEqual(target.type_mask, 0)
        converted = self.runtime.last_converted
        self.assertEqual(converted['source_stamp'], 9.99)
        self.assertEqual(converted['trajectory_id'], 43)
        self.assertEqual(converted['session_id'], 'session-A')
        np.testing.assert_allclose(np.array(converted['position'])-np.array(self.runtime.px4_pose_sample[1]),
                                   np.array((1.3, 1.8, 3.6))-np.array((1., 2., 3.)))
        np.testing.assert_allclose(np.array(converted['velocity'])-np.array(self.runtime.px4_velocity_sample[1]),
                                   np.array((.2, -.1, .05))-np.array((.1, .2, .3)))
        self.assertEqual(tuple(vars(message.position).values()), (1.3, 1.8, 3.6))

    def test_every_recorded_route_keeps_wait_height_until_its_first_ego_command(self):
        timer = self.activate()
        expected = (1.075, 2.111, 3.212)
        for mode in ('waypoint_route', 'delivery_route', 'drop_test', 'drop_points'):
            with self.subTest(mode=mode):
                self.engine.mode, self.engine.state = mode, 'APPROACH'
                self.engine._navigation_hold = expected
                self.engine.command_yaw = .2
                # New goal, readiness, and session selection all used to
                # recapture actual Z=3.0 in delivery tasks.
                self.runtime.clear_ego()
                self.runtime.select_ego(mode)
                self.assertEqual(self.runtime.reference_command['position'], expected)
                self.assertEqual(self.runtime.reference_command['velocity'], (0., 0., 0.))
                timer.step()
                self.assertAlmostEqual(self.sent[-1].position.z, 303.212)
                self.runtime.clear_ego()
                self.runtime.select_ego(mode+'-restart')
                self.assertEqual(self.runtime.reference_command['position'], expected)
                message = self.ego(p=(1.08, 2.12, 3.21))
                self.assertTrue(self.runtime.feed_ego(mode+'-restart', message))
                timer.step()
                self.assertAlmostEqual(self.sent[-1].position.z, 303.21)
                self.assertEqual(self.runtime.reference_source, 'EGO')

    def test_first_ego_measured_start_is_forwarded_even_with_height_residual(self):
        # The handoff repair covers the waiting reference only. EGO still
        # plans from measured odometry, so its first sample may differ from
        # the completed climb endpoint; no new wait, clamp or blend is added.
        timer = self.activate()
        self.engine.mode, self.engine.state = 'drop_points', 'APPROACH'
        self.engine._navigation_hold = (1., 2., 3.212)
        self.engine.command_yaw = .2
        self.runtime.select_ego('measured-start')
        timer.step()
        waiting_z = self.sent[-1].position.z
        message = self.ego(p=(1., 2., 3.), v=(0., 0., 0.), a=(0., 0., 0.))
        self.assertTrue(self.runtime.feed_ego('measured-start', message))
        timer.step()
        self.assertAlmostEqual(self.sent[-1].position.z-waiting_z, -.212)
        self.assertEqual(self.runtime.ego_command['position'], (1., 2., 3.))
        self.assertEqual(self.runtime.reference_source, 'EGO')


    def test_position_only_has_no_compensated_velocity_or_acceleration_feedforward(self):
        timer = self.activate()
        command = dict(position=(1.2, 2., 3.), velocity=(9., 8., 7.), yaw=.4,
                       source_stamp=self.now, setpoint_mode='position_yaw')
        self.runtime.set_reference(command, 'DOOR_NETWORK')
        timer.step()
        self.assertEqual(self.sent[-1].type_mask, 2552)
        np.testing.assert_allclose(tuple(vars(self.sent[-1].velocity).values()), (0., 0., 0.))
        self.assertEqual(self.runtime.last_converted['velocity_offset_xyz'], (0., 0., 0.))
        with self.assertRaises(ValueError):
            self.runtime.publish(dict(command, acceleration=(0., 0., 0.)), full=True)
        with self.assertRaises(ValueError):
            self.runtime.publish(dict(command, setpoint_mode='position'), full=True)

    def test_ego_timing_records_age_without_rejecting_delayed_or_duplicate_reference(self):
        timer = self.activate()
        self.runtime.select_ego('session-A')
        message = self.ego()
        self.assertTrue(self.runtime.feed_ego('session-A', message))
        original_stamp = self.runtime.reference_progress.stamp
        self.advance(.10)
        self.assertTrue(self.runtime.feed_ego('session-A', message))
        self.advance(.30)
        self.assertTrue(self.runtime.feed_ego('session-A', message))
        timing = self.runtime.ego_timing
        self.assertEqual(timing['accepted_count'], 3)
        self.assertEqual(timing['rejected_nonprogress_count'], 0)
        self.assertEqual(timing['rejected_stale_count'], 0)
        self.assertAlmostEqual(timing['source_age_at_callback_s'], .41)
        self.assertAlmostEqual(timing['callback_gap_s'], .30)
        self.assertEqual(self.runtime.reference_progress.stamp, original_stamp)
        reference = self.runtime.ego_reference('session-A')
        reference['yaw'] = 123.
        self.assertEqual(self.runtime.ego_reference('session-A')['yaw'], .7)
        self.assertIsNone(self.runtime.ego_reference('old-session'))
        timer.step()
        self.assertFalse(self.runtime.reference_stale)
        self.assertAlmostEqual(self.sent[-1].velocity.x, .7)  # compensated original feedforward
        self.runtime.clear_ego()
        self.assertEqual(self.runtime.ego_timing, {})
        self.assertIsNone(self.runtime.ego_reference('session-A'))

    def test_pose_and_velocity_use_separate_timestamps_not_status_or_receipt_time(self):
        self.now = 10.2
        self.runtime.reference.feed(10.2, (1.2, 2.4, 3.6), (.3, .4, .5))
        self.feed_px4(10.1, (101.1, 202.2, 303.3), 10.15, (5.25, -5.65, 9.45))
        command = dict(position=(2., 3., 4.), velocity=(.5, .6, .7), yaw=0.)
        converted = self.runtime.publish(command, full=True)
        np.testing.assert_allclose(converted['translation_xyz'], (100., 200., 300.))
        np.testing.assert_allclose(converted['velocity_offset_xyz'], (5., -6., 9.))
        np.testing.assert_allclose(converted['position'], (102., 203., 304.))
        np.testing.assert_allclose(converted['velocity'], (5.5, -5.4, 9.7))
        self.assertEqual(converted['px4_pose_stamp'], 10.1)
        self.assertEqual(converted['px4_velocity_stamp'], 10.15)

    def test_direct_ego_ignores_estimator_offsets_and_logs_them_separately(self):
        self.engine.config = dict(flight=dict(controller='px4_native', native_reference_mode='lio_direct'))
        timer = self.activate()
        self.runtime.select_ego('direct-session')
        self.runtime.feed_ego('direct-session', self.ego())
        timer.step()
        converted = self.runtime.last_converted
        for field in ('position', 'velocity', 'acceleration_or_force'):
            expected = dict(position=(1.3, 1.8, 3.6), velocity=(.2, -.1, .05),
                            acceleration_or_force=(.02, -.03, .04))[field]
            np.testing.assert_allclose(tuple(vars(getattr(self.sent[-1], field)).values()), expected)
        self.assertEqual((self.sent[-1].yaw, self.sent[-1].yaw_rate, self.sent[-1].type_mask), (.7, .12, 0))
        self.assertEqual(converted['translation_xyz'], (0., 0., 0.))
        self.assertEqual(converted['velocity_offset_xyz'], (0., 0., 0.))
        np.testing.assert_allclose(converted['estimator_position_delta_xyz'], (100., 200., 300.))
        np.testing.assert_allclose(converted['estimator_velocity_delta_xyz'], (.5, -.4, .1))
        logged = json.loads(self.runtime.trace.getvalue().splitlines()[-1])
        self.assertEqual(logged['px4_command']['reference_mode'], 'lio_direct')
        self.assertEqual(logged['px4_command']['position'], logged['command']['position'])
        self.assertEqual(logged['px4_command']['velocity'], logged['command']['velocity'])

    def test_direct_mode_is_snapshotted_and_hold_has_zero_velocity(self):
        self.engine.config = dict(flight=dict(controller='px4_native', native_reference_mode='lio_direct'))
        timer = self.activate()
        self.engine.config['flight']['native_reference_mode'] = 'estimate_compensated'
        timer.step()
        self.assertEqual(self.runtime.last_converted['reference_mode'], 'lio_direct')
        np.testing.assert_allclose(tuple(vars(self.sent[-1].position).values()), (1., 2., 3.))
        np.testing.assert_allclose(tuple(vars(self.sent[-1].velocity).values()), (0., 0., 0.))
        self.runtime.enable_full(self.manager)
        timer.step()
        self.assertEqual(self.runtime.last_converted['reference_mode'], 'estimate_compensated')


    def test_direct_mode_keeps_radar_commands_when_px4_pose_stops(self):
        self.engine.config = dict(flight=dict(controller='px4_native', native_reference_mode='lio_direct'))
        timer = self.activate()
        timer.step()
        count = len(self.sent)
        last_pose = self.runtime.px4_pose_sample
        for _ in range(6):
            self.advance(.1, skip=('pose',))
            command = dict(position=(1., 2., 4.), velocity=(.1, 0., 0.), yaw=.2,
                           source_stamp=self.now)
            self.runtime.set_reference(command, 'TAKEOFF')
            timer.step()
            self.assertFalse(self.runtime.watchdog_fault)
            self.assertTrue(self.manager.active)
            np.testing.assert_allclose(tuple(vars(self.sent[-1].position).values()), command['position'])
            np.testing.assert_allclose(tuple(vars(self.sent[-1].velocity).values()), command['velocity'])
        self.assertEqual(len(self.sent), count+6)
        self.assertEqual(self.runtime.px4_pose_sample, last_pose)
        self.assertFalse(self.runtime.input_progress['pose'].fresh(self.now, self.wall))
        self.assertEqual(self.engine.holds, [])
        count = len(self.sent)
        self.advance(.31, skip=('lio', 'pose'))
        timer.step()
        self.assertEqual(len(self.sent), count+1)
        self.assertFalse(self.runtime.watchdog_fault)
        self.assertTrue(self.manager.active)
        self.assertEqual(self.engine.holds, [])

    def test_finite_px4_timestamps_accepted_but_nan_and_inf_do_not_poison_cache(self):
        self.engine.config = dict(flight=dict(controller='px4_native', native_reference_mode='lio_direct'))
        timer = self.activate()
        last_pose = self.runtime.px4_pose_sample
        for stamp in (10., 9.99, 0., 20., math.nan, math.inf):
            with self.subTest(stamp=stamp):
                self.advance(.1, skip=('pose',))
                self.runtime._px4_pose(NS(header=NS(stamp=Stamp(stamp)),
                    pose=NS(position=xyz((999., 999., 999.)))))
                timer.step()
                if math.isfinite(stamp):
                    self.assertEqual(self.runtime.px4_pose_sample, (stamp, (999., 999., 999.)))
                    last_pose = self.runtime.px4_pose_sample
                else:
                    self.assertEqual(self.runtime.px4_pose_sample, last_pose)
                self.assertFalse(self.runtime.watchdog_fault)
                self.assertTrue(self.manager.active)
                np.testing.assert_allclose(tuple(vars(self.sent[-1].position).values()), (1., 2., 3.))
        self.assertFalse(self.runtime.input_progress['pose'].fresh(self.now, self.wall))
        self.assertEqual(self.engine.holds, [])

    def test_invalid_reference_mode_fails_before_starting_timer_or_rpc(self):
        self.engine.config = dict(flight=dict(controller='px4_native', native_reference_mode='typo'))
        with self.assertRaisesRegex(ValueError, 'native_reference_mode'):
            self.activate()
        self.assertFalse(self.runtime.full_enabled)
        self.assertEqual(self.timers, [])
        self.assertEqual(self.rpc_calls, [])

    def test_full_mode_does_not_relabel_status_velocity_with_pose_timestamp(self):
        self.runtime.px4_velocity_sample = None
        with self.assertRaisesRegex(ValueError, 'timestamped'):
            self.runtime.publish(dict(position=(1., 2., 3.), velocity=(0., 0., 0.), yaw=0.), full=True)
        self.assertEqual(self.sent, [])
        # The previously working alignment-only boundary is unchanged.
        converted = self.runtime.publish(dict(position=(1., 2., 3.), velocity=(0., 0., 0.), yaw=0.))
        self.assertEqual(converted['velocity'], (0., 0., 0.))

    def test_clear_session_cannot_commit_until_inflight_reference_publish_finishes(self):
        timer = self.activate()
        self.runtime.select_ego('old')
        self.runtime.feed_ego('old', self.ego())
        entered, release, clearing, cleared = [threading.Event() for _ in range(4)]
        original_publish = self.runtime.publisher.publish
        def delayed_publish(message):
            entered.set()
            if not release.wait(2.): raise ValueError('test publication was not released')
            original_publish(message)
        self.runtime.publisher.publish = delayed_publish
        tick = threading.Thread(target=timer.step)
        def clear():
            clearing.set()
            self.runtime.clear_ego()
            cleared.set()
        cleanup = threading.Thread(target=clear)
        try:
            tick.start()
            self.assertTrue(entered.wait(2.))
            cleanup.start()
            self.assertTrue(clearing.wait(2.))
            self.assertFalse(cleared.wait(.01))
            release.set()
            self.assertTrue(cleared.wait(2.))
        finally:
            release.set()
            tick.join(2.)
            if cleanup.ident is not None: cleanup.join(2.)
        self.runtime.publisher.publish = original_publish
        self.assertFalse(self.runtime.feed_ego('old', self.ego()))
        timer.step()
        self.assertEqual(self.runtime.last_converted['reference_source'], 'HOLD')
        self.assertEqual(self.runtime.last_converted['lio_reference_xyz'], (1., 2., 3.))

    def test_body_velocity_rotates_once_while_ego_world_command_does_not_rotate(self):
        self.now = 11.
        self.feed_status()
        q = NS(x=0., y=0., z=math.sqrt(.5), w=math.sqrt(.5))
        self.runtime._lio_feedback(NS(header=NS(stamp=Stamp(11.)),
            pose=NS(pose=NS(position=xyz((1., 2., 3.)), orientation=q)),
            twist=NS(twist=NS(linear=xyz((.4, 0., 0.))))))
        self.feed_px4(11., (1., 2., 3.), 11., (0., .4, 0.))
        converted = self.runtime.publish(dict(position=(2., 2., 3.), velocity=(.2, 0., 0.), yaw=math.pi/2), full=True)
        np.testing.assert_allclose(self.runtime.reference.latest['velocity'], (0., .4, 0.), atol=1e-12)
        np.testing.assert_allclose(converted['velocity'], (.2, 0., 0.), atol=1e-12)
        np.testing.assert_allclose(converted['position'], (2., 2., 3.))

    def test_timer_runs_at_fifty_hz_without_mission_ticks_and_keeps_completion_hold(self):
        timer = self.activate()
        self.assertAlmostEqual(timer.period, .02)
        self.manager.active = False  # a completed alignment/takeoff still holds.
        anchor = self.runtime.reference_command['position']
        self.runtime.reference.feed(10.1, (1.1, 2.2, 3.3), (.1, .2, .3))
        for _ in range(5): timer.step()
        self.assertEqual(len(self.sent), 5)
        self.assertEqual(self.runtime.reference_command['position'], anchor)
        self.runtime.disable_full()
        timer.step()
        self.assertEqual(len(self.sent), 5)
        self.runtime.close()
        self.assertTrue(timer.stopped)

    def test_switching_ego_sessions_and_hold_rejects_late_samples(self):
        timer = self.activate()
        self.runtime.select_ego('old')
        self.assertTrue(self.runtime.feed_ego('old', self.ego()))
        self.runtime.select_ego('new')
        self.assertIsNone(self.runtime.ego_command)
        self.assertFalse(self.runtime.feed_ego('old', self.ego()))
        with self.assertRaises(ValueError): self.runtime.feed_ego('new', self.ego(frame='world'))
        self.assertTrue(self.runtime.feed_ego('new', self.ego(p=(1.8, 2., 3.))))
        self.runtime.clear_ego()
        self.assertFalse(self.runtime.feed_ego('new', self.ego()))
        timer.step()
        self.assertEqual(self.runtime.last_converted['reference_source'], 'HOLD')
        self.assertEqual(self.runtime.last_converted['lio_reference_xyz'], (1., 2., 3.))

    def test_manual_mode_change_stops_stream_and_does_not_resume_automatically(self):
        timer = self.activate()
        timer.step()
        self.runtime.packet.update(offboard=False, mode='POSCTL')
        timer.step()
        self.assertEqual(len(self.sent), 1)
        self.assertTrue(self.runtime.manual_takeover)
        self.assertFalse(self.manager.active)
        self.assertEqual(self.manager.requested, ['NONE'])
        self.assertEqual(self.engine.state, 'HOLD')
        self.runtime.packet.update(offboard=True, mode='OFFBOARD')
        timer.step()
        self.assertEqual(len(self.sent), 1)

    def test_auto_land_owner_stops_reference_without_false_manual_takeover(self):
        timer = self.activate()
        timer.step()
        self.runtime.packet.update(owner='PX4_AUTO_LAND', offboard=False, mode='AUTO.LAND')
        timer.step()
        self.assertEqual(len(self.sent), 1)
        self.assertFalse(self.runtime.manual_takeover)
        self.assertEqual(self.manager.requested, [])

    def test_full_origin_restore_matches_corrected_ground_and_preserves_raw_record(self):
        self.runtime.raw_origin = (40., -20., 80.)
        self.runtime.packet.update(landed=True)
        self.parameters['/px4ctrl/auto_takeoff_land/takeoff_land_speed'] = .17
        self.runtime.prepare(self.engine, 'takeoff')
        stored = self.parameters[NativeRuntime.ORIGIN_PARAM]
        self.assertEqual(stored['radar_origin_xyz'], [1., 2., 3.])
        self.assertEqual(stored['lio_origin_xyz'], [40., -20., 80.])
        self.assertEqual(self.engine.native_takeoff_speed, .17)
        restarted = NativeRuntime()
        restarted.packet = deepcopy(self.runtime.packet)
        restarted.packet.update(landed=False, position=(99., 88., 77.))
        restarted.reference.feed(12., (1., 2., 3.65), (0., 0., 0.))
        engine = Engine()
        engine.origin_xyz = (1., 2., 3.)
        restarted.prepare(engine, 'align', origin=(1., 2., 3.))
        self.assertEqual(engine.native_origin, (1., 2., 3.))
        self.assertEqual(restarted.lio_origin, (40., -20., 80.))
        self.assertTrue(engine.started)

    def test_drop_test_registers_current_ground_and_uses_same_airborne_origin_semantics(self):
        self.runtime.raw_origin = (40., -20., 80.)
        self.runtime.origin = (99., 98., 97.)
        self.runtime.packet.update(landed=True)
        self.runtime.prepare(self.engine, 'drop_test')
        self.assertEqual(self.engine.native_origin, (1., 2., 3.))
        self.assertEqual(self.parameters[NativeRuntime.ORIGIN_PARAM]['radar_origin_xyz'], [1., 2., 3.])
        self.assertTrue(self.engine.started)
        self.runtime.packet.update(landed=False)
        continued = Engine()
        continued.origin_xyz = (1., 2., 4.)
        self.runtime.prepare(continued, 'drop_test')
        self.assertEqual(continued.native_origin, (1., 2., 3.))

    def test_drop_test_px4_frame_is_refrozen_on_a_new_ground_start(self):
        config = manager_config()
        config['flight'].update(controller='px4_native', native_reference_mode='lio_direct',
                                native_task_feedback='px4')
        self.runtime.packet.update(landed=True)
        first = self.runtime.prepare_task_frame(config, 'drop_test')
        self.assertEqual(first['position'], (1., 2., 3.))
        previous = self.runtime.px4_task_frame
        self.advance(1.)
        self.runtime.reference.feed(self.now, (5., 6., 7.), (0., 0., 0.))
        self.feed_px4(self.now, (205., 306., 407.), self.now, (0., 0., 0.))
        second = self.runtime.prepare_task_frame(config, 'drop_test')
        self.assertEqual(second['position'], (5., 6., 7.))
        self.assertEqual(self.runtime.px4_task_frame.ground_origin, (5., 6., 7.))
        self.assertIsNot(self.runtime.px4_task_frame, previous)

    def test_cancel_during_mode_or_arming_service_wait_sends_no_later_rpc(self):
        class ImmediateThread:
            def __init__(self, target, **kwargs): self.target = target
            def start(self): self.target()
        for blocked in ('/mavros/set_mode', '/mavros/cmd/arming'):
            with self.subTest(blocked=blocked):
                self.engine.state = 'TAKEOFF'
                self.runtime.enable_full(self.manager)
                self.rpc_calls.clear()
                self.wall = 0.
                self.wait_hook = lambda name: self.runtime.disable_full() if name == blocked else None
                with patch.object(runtime_module.threading, 'Thread', ImmediateThread), \
                     patch.object(runtime_module.time, 'monotonic', lambda: self.wall), \
                     patch.object(runtime_module.time, 'sleep', self.advance):
                    self.runtime.start_takeoff(self.manager, {})
                self.assertNotIn(blocked, [item[0] for item in self.rpc_calls])

    def test_ego_reference_gap_keeps_whole_reference_until_next_sample(self):
        self.engine.config = dict(flight=dict(controller='px4_native', native_reference_mode='lio_direct'))
        timer = self.activate()
        self.runtime.select_ego('flight')
        self.assertTrue(self.runtime.feed_ego('flight', self.ego()))
        expected = deepcopy(self.runtime.ego_command)
        timer.step()
        np.testing.assert_allclose(self.runtime.last_converted['velocity'], expected['velocity'])
        for delay in (.32, 1., 5.):
            self.advance(delay)
            timer.step()
            self.assertFalse(self.runtime.watchdog_fault)
            self.assertFalse(self.runtime.watchdog_holding)
            self.assertTrue(self.manager.active)
            self.assertEqual(self.runtime.ego_command, expected)
            self.assertEqual(self.runtime.last_converted['reference_source'], 'EGO')
            self.assertEqual(self.runtime.last_converted['lio_reference_xyz'], expected['position'])
            self.assertEqual(self.runtime.last_converted['yaw'], expected['yaw'])
            np.testing.assert_allclose(self.runtime.last_converted['velocity'], expected['velocity'])
            np.testing.assert_allclose(self.runtime.last_converted['acceleration'], expected['acceleration'])
            self.assertEqual(self.runtime.last_converted['yaw_rate'], expected['yaw_rate'])
            self.assertEqual(self.sent[-1].header.stamp.to_sec(), self.now)
        self.assertEqual(self.manager.requested, [])
        self.assertEqual(self.engine.holds, [])
        message = self.ego(p=(2., 3., 4.))
        message.header.stamp = Stamp(self.now)
        self.assertTrue(self.runtime.feed_ego('flight', message))
        timer.step()
        self.assertEqual(self.runtime.last_converted['lio_reference_xyz'], (2., 3., 4.))
        np.testing.assert_allclose(self.runtime.last_converted['velocity'], expected['velocity'])
        stale_events = [values['active'] for kind, values in self.manager.events
                        if kind == 'native_reference_stale_hold']
        self.assertEqual(stale_events, [])
        lines = [json.loads(line) for line in self.runtime.trace.getvalue().splitlines()]
        for line in lines:
            self.assertIn('px4_command', line)
        self.assertFalse(any(line['reference_stale_hold'] for line in lines))
        self.assertFalse(lines[-1]['reference_stale_hold'])

    def test_task_reference_gap_keeps_whole_reference_for_each_source(self):
        timer = self.activate()
        for source in ('TAKEOFF', 'NATIVE', 'SEARCH', 'HOLD'):
            with self.subTest(source=source):
                self.advance(.1)
                command = dict(position=(1., 2., 4.), velocity=(0., 0., .3),
                    acceleration=(.1, .2, .3), yaw=.5, yaw_rate=.2)
                self.runtime.set_reference(command, source)
                timer.step()
                self.assertEqual(self.runtime.last_converted['acceleration'], command['acceleration'])
                self.advance(2.)
                timer.step()
                self.assertFalse(self.runtime.watchdog_fault)
                self.assertTrue(self.manager.active)
                self.assertEqual(self.runtime.reference_source, source)
                self.assertEqual(self.runtime.last_converted['lio_reference_xyz'], command['position'])
                self.assertEqual(self.runtime.last_converted['yaw'], command['yaw'])
                np.testing.assert_allclose(self.runtime.last_converted['acceleration'], command['acceleration'])
                self.assertEqual(self.runtime.last_converted['yaw_rate'], command['yaw_rate'])
                self.assertEqual(self.runtime.last_reference_sent['velocity'], command['velocity'])
                self.assertEqual(self.runtime.reference_command['velocity'], command['velocity'])
        self.assertEqual(self.engine.holds, [])

    def test_manual_takeover_still_stops_after_task_reference_gap(self):
        timer = self.activate()
        self.runtime.select_ego('flight')
        self.assertTrue(self.runtime.feed_ego('flight', self.ego()))
        timer.step()
        self.advance(1.)
        timer.step()
        sent = len(self.sent)
        self.runtime.packet.update(offboard=False, mode='POSCTL')
        self.advance(.02)
        timer.step()
        self.assertTrue(self.runtime.manual_takeover)
        self.assertFalse(self.manager.active)
        self.assertEqual(len(self.sent), sent)
        self.assertEqual(len(self.engine.holds), 1)
        self.assertEqual(self.manager.requested, ['NONE'])
        self.assertFalse(self.runtime.feed_ego('flight', self.ego()))

    def test_frozen_feedback_streams_keep_task_and_recover_without_restart(self):
        timer = self.activate()
        for key in ('lio', 'velocity', 'status'):
            with self.subTest(key=key):
                self.engine.state = 'TAKEOFF'
                self.advance(.1)
                self.runtime.enable_full(self.manager)
                timer.step(); sent = len(self.sent)
                self.advance(.32, skip=(key,)); timer.step()
                self.assertEqual(len(self.sent), sent+1)
                self.assertFalse(self.runtime.watchdog_fault)
                self.assertFalse(self.runtime.watchdog_holding)
                self.assertTrue(self.manager.active)
                self.advance(.1); timer.step()
                self.assertEqual(len(self.sent), sent+2)
                self.assertEqual(self.engine.holds, [])

    def test_reference_and_localization_gaps_do_not_revoke_task(self):
        timer = self.activate()
        self.runtime.set_reference(dict(position=(1.,2.,4.), velocity=(0.,0.,.3), yaw=0.), 'TAKEOFF')
        timer.step(); self.advance(.32); timer.step()
        self.assertFalse(self.runtime.watchdog_fault)
        self.assertTrue(self.manager.active)
        sent = len(self.sent)
        self.advance(.32, skip=('lio',)); timer.step()
        self.assertFalse(self.runtime.watchdog_fault)
        self.assertFalse(self.runtime.watchdog_holding)
        self.assertTrue(self.manager.active)
        self.assertEqual(len(self.sent), sent+1)
        self.assertEqual(self.engine.holds, [])
        self.assertEqual(self.rpc_calls, [])

    def test_zero_motion_hold_does_not_expire_with_fresh_feedback(self):
        timer = self.activate()
        for _ in range(100):
            self.advance(.1); timer.step()
        self.assertFalse(self.runtime.watchdog_fault)
        self.assertEqual(len(self.sent), 100)
        self.assertEqual(self.runtime.reference_command['position'], (1.,2.,3.))

    def test_repeated_and_out_of_order_ego_update_current_session_reference(self):
        timer = self.activate()
        self.runtime.select_ego('flight')
        message = self.ego()
        self.assertTrue(self.runtime.feed_ego('flight', message))
        message.position.x = 999.
        self.advance(.2)
        self.assertTrue(self.runtime.feed_ego('flight', message))
        message.header.stamp = Stamp(9.98)
        self.assertTrue(self.runtime.feed_ego('flight', message))
        self.assertEqual(self.runtime.ego_command['position'][0], 999.)
        self.advance(.1); timer.step()
        self.assertFalse(self.runtime.watchdog_fault)
        self.assertEqual(self.runtime.last_converted['lio_reference_xyz'][0], 999.)
        self.assertTrue(self.manager.active)

    def test_new_ego_session_holds_until_first_sample_then_retains_last_reference(self):
        timer = self.activate()
        self.runtime.set_reference(dict(position=(9.,9.,9.), velocity=(1.,1.,1.), yaw=0.), 'TAKEOFF')
        self.runtime.select_ego(None)
        timer.step()
        self.assertFalse(self.runtime.feed_ego(None, self.ego()))
        self.runtime.select_ego('new')
        timer.step()
        self.assertFalse(self.runtime.watchdog_fault)
        self.assertEqual(self.runtime.last_converted['lio_reference_xyz'], (1.,2.,3.))
        for _ in range(30): self.advance(.1); timer.step()
        self.assertFalse(self.runtime.watchdog_fault)
        self.assertEqual(self.runtime.last_converted['lio_reference_xyz'], (1.,2.,3.))
        message = self.ego()
        message.header.stamp = Stamp(self.now)
        self.assertTrue(self.runtime.feed_ego('new', message))
        self.advance(.32); timer.step()
        self.assertFalse(self.runtime.watchdog_fault)
        self.assertTrue(self.manager.active)
        self.assertEqual(self.runtime.last_converted['lio_reference_xyz'], (1.3, 1.8, 3.6))
        self.assertEqual(self.runtime.ego_command['session_id'], 'new')

    def test_lio_age_gap_during_startup_does_not_cancel_accepted_task(self):
        class ImmediateThread:
            def __init__(self, target, **kwargs): self.target = target
            def start(self): self.target()
        self.activate()
        with patch.object(runtime_module.threading, 'Thread', ImmediateThread), \
             patch.object(runtime_module.time, 'sleep', lambda dt: self.advance(dt, skip=('lio',))):
            self.runtime.start_takeoff(self.manager, {})
        self.assertEqual([c[0] for c in self.rpc_calls],
                         ['/px4ctrl/control_owner', '/mavros/set_mode', '/mavros/cmd/arming'])
        self.assertEqual(self.engine.state, 'TAKEOFF')
        self.assertEqual(self.engine.holds, [])
        self.assertTrue(any(kind == 'native_takeoff_requested' for kind, _ in self.manager.events))

    def test_gateway_fault_blocks_arming_even_if_publication_timer_did_not_run(self):
        class ImmediateThread:
            def __init__(self, target, **kwargs): self.target = target
            def start(self): self.target()
        self.activate()
        def fault_at_arm(name):
            if name == '/mavros/cmd/arming':
                self.runtime.packet.update(watchdog_fault=True, stamp=self.now)
        self.wait_hook = fault_at_arm
        with patch.object(runtime_module.threading, 'Thread', ImmediateThread), \
             patch.object(runtime_module.time, 'sleep', self.advance):
            self.runtime.start_takeoff(self.manager, {})
        self.assertNotIn('/mavros/cmd/arming', [c[0] for c in self.rpc_calls])
        self.assertEqual(self.engine.state, 'HOLD')

    def test_gateway_fault_stops_active_task_but_old_fault_does_not_block_new_acquire(self):
        timer = self.activate()
        self.runtime.packet.update(watchdog_fault=True, watchdog_reason='gateway stopped')
        timer.step()
        self.assertTrue(self.runtime.watchdog_fault)
        self.assertFalse(self.manager.active)
        self.assertEqual(self.sent, [])
        self.engine.state = 'TAKEOFF'
        self.runtime.enable_full(self.manager)  # Explicit new acquisition.
        timer.step()
        self.assertFalse(self.runtime.watchdog_fault)
        self.assertEqual(len(self.sent), 1)

    def test_missing_status_or_reference_does_not_bypass_input_watchdog(self):
        timer = self.activate()
        self.runtime.packet = self.runtime.reference_command = None
        self.now += .32; self.wall += .32
        timer.step()
        self.assertTrue(self.runtime.watchdog_fault)
        self.assertFalse(self.manager.active)
        self.assertEqual(self.sent, [])

    def test_invalid_status_keeps_last_valid_packet_and_waits_without_cancelling_task(self):
        timer = self.activate()
        original = deepcopy(self.runtime.packet)
        self.runtime._feedback(NS(data='{broken'))
        timer.step()
        self.assertEqual(len(self.sent), 1)
        self.advance(.32, skip=('status',))
        self.runtime._feedback(NS(data='{broken'))
        timer.step()
        self.assertFalse(self.runtime.watchdog_fault)
        self.assertTrue(self.manager.active)
        self.assertEqual(self.runtime.packet, original)
        self.assertFalse(self.runtime.input_progress['status'].fresh(self.now, self.wall))
        self.assertEqual(len(self.sent), 2)
        self.assertEqual(self.engine.holds, [])

    def test_duplicate_old_and_future_px4_packets_update_cached_inputs(self):
        timer = self.activate()
        for stamp in (10.,9.99,11.):
            self.feed_px4(stamp, (999.,999.,999.), stamp, (9.,9.,9.))
        self.assertEqual(self.runtime.px4_pose_sample[1], (999.,999.,999.))
        self.assertEqual(self.runtime.px4_velocity_sample[1], (9.,9.,9.))
        self.advance(.32, skip=('pose','velocity'))
        self.feed_px4(10., (999.,999.,999.), 10., (9.,9.,9.))
        timer.step()
        self.assertFalse(self.runtime.watchdog_fault)
        self.assertTrue(self.manager.active)
        self.assertEqual(self.runtime.px4_pose_sample[1], (999.,999.,999.))
        self.assertEqual(self.runtime.px4_velocity_sample[1], (9.,9.,9.))
        self.assertFalse(self.runtime.input_progress['velocity'].fresh(self.now, self.wall))
        self.assertEqual(len(self.sent), 1)
        self.assertEqual(self.engine.holds, [])


class ManagerFrameTests(unittest.TestCase):
    def test_rejected_new_profile_does_not_change_active_tasks_frame(self):
        manager = make_manager()
        manager.pose, manager.yaw, manager.odom_stamp = (100., 200., 300.), .2, 10.
        manager.raw_pose, manager.raw_yaw, manager.raw_stamp = manager.pose, manager.yaw, manager.odom_stamp
        manager.planning_pose, manager.planning_stamp = (1., 2., 3.), 10.02
        manager.raw_to_local = object()
        old_engine, expected = manager.engine, (manager.pose, manager.yaw, manager.odom_stamp)
        config = manager_config()
        def reject(*args, **kwargs): raise ValueError('no saved radar ground')
        manager.native_runtime = NS(feedback=lambda: dict(yaw=.8), prepare=reject)
        candidate = NS(full_native=True, uses_native=True, start=lambda *args, **kwargs: None)
        with patch.dict(sys.modules, STUBS), \
             patch.dict(os.environ, ROS_MASTER_URI=config['runtime']['ros_master_uri']), \
             patch.object(MANAGER, 'load_config', return_value=config), \
             patch('airseafly_real400_mission.native_waypoint_route.NativeTakeoffEngine', return_value=candidate), \
             patch.object(ROS, 'get_param', return_value=None, create=True):
            result = manager.dispatch(dict(command='start', mode='basic_takeoff'))
        self.assertFalse(result['ok'])
        self.assertIs(manager.engine, old_engine)
        self.assertEqual((manager.pose, manager.yaw, manager.odom_stamp), expected)

    def test_raw_callback_cannot_replace_full_native_planning_pose(self):
        manager = make_manager()
        manager.engine = NS(full_native=True)
        manager.raw_pose = manager.planning_pose = None
        planning = NS(header=NS(stamp=Stamp(10.), frame_id=LIO_FRAME),
                      pose=NS(pose=NS(position=xyz((1., 2., 3.)), orientation=NS(x=0., y=0., z=0., w=1.))))
        raw = deepcopy(planning)
        raw.header.frame_id, raw.header.stamp = 'world', Stamp(10.01)
        raw.pose.pose.position = xyz((100., 200., 300.))
        manager.on_planning_odom(planning)
        manager.on_odom(raw)
        self.assertEqual(manager.pose, (1., 2., 3.))
        self.assertEqual(manager.odom_stamp, 10.)
        self.assertEqual(manager.raw_pose, [100., 200., 300.])
        manager.engine = NS(full_native=False)
        manager.on_odom(raw)
        manager.on_planning_odom(planning)
        self.assertEqual(manager.pose, [100., 200., 300.])
        self.assertEqual(manager.odom_stamp, 10.01)


if __name__ == '__main__': unittest.main()
