"""ROS adapter; only px4ctrl may publish the MAVROS native setpoint topic."""
import json
import threading
import time
from copy import deepcopy
from .native_motion import vector, rotation
from .native_reference import LioReference, LIO_FRAME, COMPENSATED, reference_mode
from .native_freshness import SourceProgress, MAX_AGE_S
from .ego_identity import goal_key
from .native_task_feedback import (PX4_FRAME, feedback_source, validate_task_mode,
                                   FrozenPx4TaskFrame)



class NativeRuntime:
    ORIGIN_PARAM = '/airseafly/native/takeoff_origin'
    PX4_ORIGIN_PARAM = '/airseafly/native/px4_trial_takeoff_origin'

    def __init__(self):
        import rospy
        from std_msgs.msg import String
        from mavros_msgs.msg import PositionTarget
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from sensor_msgs.msg import BatteryState
        self.rospy, self.Target, self.Odometry = rospy, PositionTarget, Odometry
        self.packet, self.received, self.origin = None, 0., None
        self.lio_origin = None
        self.reference = LioReference()
        self.reference_mode = COMPENSATED
        self.mutex, self.service_mutex = threading.RLock(), threading.Lock()
        self.full_enabled = False
        self.full_engine = self.manager = None
        self.full_task_id = None
        self.reference_command = self.ego_command = None
        self.reference_source, self.ego_session = 'HOLD', None
        self.fixed_trajectory = self.fixed_identity = self.fixed_sample = None
        self.fixed_started_at = None
        self.fixed_revoked = set()
        self.manual_takeover = self.seen_offboard = False
        self.full_owner_since = None
        self.start_epoch = 0
        self.px4_pose_sample = self.px4_velocity_sample = None
        self.px4_orientation_sample = None
        self.battery_sample = None
        self.px4_task_frame = None
        self.task_odometry_stamp = None
        self.full_timer = self.trace = None
        self.last_send_wall = None
        self.input_progress = {key: SourceProgress() for key in ('status', 'lio', 'pose', 'velocity')}
        self.reference_progress = SourceProgress()
        self.reference_stale = False
        self.ego_timing = {}
        self.watchdog_fault = ''
        self.watchdog_holding = False
        self.publisher = rospy.Publisher('/px4ctrl/native_setpoint', PositionTarget, queue_size=1)
        self.task_odometry_publisher = rospy.Publisher('/airseafly/native/task_odometry', Odometry, queue_size=1)
        self.subscriber = rospy.Subscriber('/px4ctrl/native_status', String, self._feedback, queue_size=1)
        self.lio_subscriber = rospy.Subscriber('/airseafly/native/lio_odometry', Odometry,
                                              self._lio_feedback, queue_size=1, tcp_nodelay=True)
        self.pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
            self._px4_pose, queue_size=1, tcp_nodelay=True)
        self.velocity_subscriber = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped,
            self._px4_velocity, queue_size=1, tcp_nodelay=True)
        # Logging only: battery voltage/current are written to the trace, never used for control.
        self.battery_subscriber = rospy.Subscriber('/mavros/battery', BatteryState, self._battery, queue_size=1)

    def _battery(self, msg):
        finite = lambda value: float(value) if value == value and abs(value) != float('inf') else None
        sample = dict(stamp=msg.header.stamp.to_sec(), voltage=finite(msg.voltage), current=finite(msg.current))
        with self.mutex:
            self.battery_sample = sample

    def _px4_pose(self, msg):
        p = msg.pose.position
        position, stamp = tuple(vector((p.x,p.y,p.z))), msg.header.stamp.to_sec()
        q = getattr(msg.pose, 'orientation', None)
        orientation = None
        if q is not None:
            orientation = (q.x, q.y, q.z, q.w)
            rotation(orientation)  # Validate before accepting this sample.
        with self.mutex:
            if self.accept_input('pose', stamp):
                self.px4_pose_sample = (stamp, position)
                self.px4_orientation_sample = (stamp, orientation) if orientation is not None else None

    def _px4_velocity(self, msg):
        v = msg.twist.linear
        velocity, stamp = tuple(vector((v.x,v.y,v.z))), msg.header.stamp.to_sec()
        with self.mutex:
            if self.accept_input('velocity', stamp):
                self.px4_velocity_sample = (stamp, velocity)

    def _lio_feedback(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        # REP-147 twist is in the child/body frame; task references use ENU.
        velocity = rotation((q.x,q.y,q.z,q.w)) @ vector((v.x,v.y,v.z))
        position, stamp = tuple(vector((p.x,p.y,p.z))), msg.header.stamp.to_sec()
        with self.mutex:
            if self.accept_input('lio', stamp):
                self.reference.feed(stamp, position, velocity)

    def accept_input(self, key, stamp):
        # Call under mutex. Source timestamps are diagnostics, not admission gates.
        return self.input_progress[key].record(stamp, time.monotonic())

    def _feedback(self, msg):
        try:
            packet = json.loads(msg.data)
            with self.mutex:
                if isinstance(packet, dict) and self.accept_input('status', packet.get('stamp', 0.)):
                    self.packet, self.received = packet, time.monotonic()
        except (ValueError, TypeError):
            pass  # Keep the last structurally valid status until another one arrives.

    def feedback(self):
        with self.mutex:
            return dict(self.packet) if self.packet else None

    def update(self, engine):
        packet = self.feedback()
        if packet is None:
            engine.native_feedback = None
        else:
            engine.on_native_feedback(packet)
        radar = self.lio_feedback()
        if radar is None:
            engine.native_lio_feedback = None
        else:
            engine.on_native_lio_feedback(radar)
        # Keep the raw radar packet separately for diagnostics. The task packet
        # has its own frame/source and never masquerades as FAST-LIO feedback.
        engine.native_task_feedback = self.task_feedback(getattr(engine, 'config', {}))

    def task_feedback(self, config=None):
        config = config if config is not None else getattr(getattr(self, 'full_engine', None), 'config', {})
        if feedback_source(config) == 'lio':
            return self.lio_feedback()
        with self.mutex:
            frame = getattr(self, 'px4_task_frame', None)
            raw = self.raw_px4_feedback()
            return frame.feedback(raw) if frame is not None and raw is not None else None

    def raw_px4_feedback(self):
        """Received PX4 samples, also usable before a frame exists."""
        with self.mutex:
            now, wall = self.rospy.get_time(), time.monotonic()
            if (self.px4_pose_sample is None or self.px4_velocity_sample is None or
                    self.packet is None):
                return None
            pose_stamp, position = self.px4_pose_sample
            velocity_stamp, velocity = self.px4_velocity_sample
            return dict(stamp=pose_stamp, pose_stamp=pose_stamp, velocity_stamp=velocity_stamp,
                        position=position, velocity=velocity, frame=PX4_FRAME, position_source='PX4_EKF')

    def prepare_task_frame(self, config, mode):
        """Register ground before engine.start; never recover a frame from ROS params."""
        if validate_task_mode(config, mode) != 'px4':
            return self.lio_feedback()
        with self.mutex:
            raw = self.raw_px4_feedback()
            if raw is None or self.input_fault():
                raise ValueError('PX4任务坐标初始化需要新鲜雷达、PX4位置、速度和落地状态')
            landed = bool((self.packet or {}).get('landed'))
            frame = getattr(self, 'px4_task_frame', None)
            if landed and mode in ('full', 'takeoff', 'gate', 'delivery_route', 'waypoint_route', 'basic_takeoff', 'drop_test', 'drop_points'):
                stamp = raw['pose_stamp']
                radar, matched_stamp, alignment = self.reference.position_at(stamp)
                if alignment == 'nearest' or matched_stamp != stamp:
                    raise ValueError('PX4任务坐标初始化需要同一时刻的雷达位置，等待雷达历史覆盖PX4时间戳')
                # Do not interpolate across an earlier localization outage.
                before = max(p['stamp'] for p in self.reference.history if p['stamp'] <= stamp)
                after = min(p['stamp'] for p in self.reference.history if p['stamp'] >= stamp)
                if after-before > MAX_AGE_S:
                    raise ValueError('PX4任务坐标初始化的雷达插值间隔过大')
                frame = FrozenPx4TaskFrame(radar, raw['position'], stamp)
                self.px4_task_frame = frame
                self.task_odometry_stamp = None
                self.rospy.set_param(self.PX4_ORIGIN_PARAM, dict(frame=LIO_FRAME,
                    position_source='PX4_EKF', source_frame=PX4_FRAME,
                    origin_xyz=list(frame.ground_origin), translation_xyz=list(frame.translation_xyz),
                    pose_stamp=stamp))
            if frame is None:
                raise ValueError('缺少本进程冻结的PX4任务坐标；必须落地后从起飞任务初始化')
            return frame.feedback(raw)

    def publish_task_odometry(self):
        """Planner-only PX4 pose in the LIO world; never feeds the EV bridge."""
        with self.mutex:
            config = getattr(getattr(self, 'full_engine', None), 'config', {})
            if not self.full_enabled or feedback_source(config) != 'px4' or self.input_fault():
                return False
            packet = self.task_feedback(config)
            orientation = getattr(self, 'px4_orientation_sample', None)
            publisher = getattr(self, 'task_odometry_publisher', None)
            if (packet is None or publisher is None or orientation is None or
                    orientation[0] != packet['pose_stamp']):
                return False
            out = self.Odometry()
            out.header.stamp = self.rospy.Time.from_sec(packet['pose_stamp'])
            out.header.frame_id = LIO_FRAME
            # Same legacy EGO convention as planning_odometry: body pose with
            # WORLD velocity. Never route this message to MAVROS odometry/out.
            out.child_frame_id = 'native_base'
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = packet['position']
            (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
             out.pose.pose.orientation.z, out.pose.pose.orientation.w) = orientation[1]
            out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = packet['velocity']
            publisher.publish(out)
            self.task_odometry_stamp = packet['pose_stamp']
            return True

    def lio_feedback(self):
        with self.mutex:
            return dict(self.reference.latest) if self.reference.latest is not None else None

    def prepare(self, engine, mode, origin=None):
        # Runs only before a task is activated. No waiting in the control timer.
        config = getattr(engine, 'config', {})
        source = validate_task_mode(config, mode)
        until = time.monotonic()+2.
        while (self.feedback() is None or self.lio_feedback() is None) and time.monotonic() < until:
            time.sleep(.02)
        if source == 'px4' and getattr(self, 'px4_task_frame', None) is None:
            self.prepare_task_frame(config, mode)
        self.update(engine)
        f = engine.native_feedback
        if engine.native_lio_feedback is None:
            raise ValueError('缺少FAST-LIO位置输入')
        if not engine.native_ready(self.rospy.get_time()):
            raise ValueError('缺少PX4姿态或坐标接口输入')
        if source == 'px4':
            if engine.native_task_feedback is None:
                raise ValueError('PX4位置、速度或状态超过0.30秒未更新或时间戳异常')
            px4_origin = self.px4_task_frame.ground_origin
            if origin is not None and tuple(vector(origin)) != px4_origin:
                raise ValueError('PX4任务不能复用另一坐标登记的起飞原点')
        elif mode in ('full', 'takeoff', 'gate', 'delivery_route', 'waypoint_route', 'basic_takeoff', 'drop_test', 'drop_points') and f.get('landed'):
            # Heights use the corrected radar body origin, before EKF fusion.
            # Keep raw LIO's ground pose only for the existing EGO/manager API.
            self.origin = tuple(float(v) for v in vector(engine.native_lio_feedback['position']))
            raw_origin = getattr(self, 'raw_origin', None) if getattr(engine, 'full_native', False) else None
            self.lio_origin = tuple(float(v) for v in vector(raw_origin if raw_origin is not None else engine.position))
            self.rospy.set_param(self.ORIGIN_PARAM, dict(frame=LIO_FRAME, position_source='FAST_LIO',
                radar_origin_xyz=list(self.origin), lio_origin_xyz=list(self.lio_origin)))
        else:
            saved = self.rospy.get_param(self.ORIGIN_PARAM, None)
            if isinstance(saved, dict) and saved.get('frame') == LIO_FRAME:
                saved_origin = tuple(float(v) for v in vector(saved['radar_origin_xyz']))
                saved_lio = tuple(float(v) for v in vector(saved['lio_origin_xyz']))
            else:
                saved_origin, saved_lio = self.origin, self.lio_origin
            expected_lio = origin
            if expected_lio is None and mode not in ('full', 'takeoff', 'gate', 'delivery_route', 'waypoint_route', 'basic_takeoff', 'drop_test', 'drop_points'):
                expected_lio = engine.origin_xyz
            if saved_origin is None:
                raise ValueError('缺少已记录的雷达起飞高度基准')
            # Pair with the existing raw-LIO takeoff record, without treating
            # raw world Z as the vertical in the corrected radar frame.
            expected_saved = saved_origin if getattr(engine, 'full_native', False) else saved_lio
            if expected_lio is not None and tuple(vector(expected_lio)) != expected_saved:
                raise ValueError('缺少对应本次LIO起飞原点的雷达高度基准')
            self.origin, self.lio_origin = saved_origin, saved_lio
        engine.native_origin = px4_origin if source == 'px4' else self.origin
        if getattr(engine, 'full_native', False):
            speed = getattr(engine, 'config', {}).get('flight', {}).get('native_takeoff_speed_mps')
            if speed is None:
                speed = self.rospy.get_param('/px4ctrl/auto_takeoff_land/takeoff_land_speed', .30)
            engine.native_takeoff_speed = float(speed)
            engine.begin_native_flight()

    def publish(self, command, full=False):
        position_only = command.get('setpoint_mode') == 'position_yaw'
        if command.get('setpoint_mode') not in (None, 'position_yaw'):
            raise ValueError('unsupported native setpoint_mode')
        if position_only and ('acceleration' in command or 'yaw_rate' in command):
            raise ValueError('position_yaw cannot contain acceleration or yaw-rate feedforward')
        with self.mutex:
            feedback = dict(self.packet) if self.packet else None
            if full and feedback is not None:
                if self.px4_pose_sample is None or self.px4_velocity_sample is None:
                    raise ValueError('missing timestamped PX4 position/velocity input')
                feedback['pose_stamp'], feedback['position'] = self.px4_pose_sample
                feedback['velocity_stamp'], feedback['velocity'] = self.px4_velocity_sample
                fault = self.input_fault()
                if fault:
                    raise ValueError(fault)
            mode = self.reference_mode if full else COMPENSATED
            source = feedback_source(getattr(getattr(self, 'full_engine', None), 'config', {}))
            if full and source == 'px4':
                frame = getattr(self, 'px4_task_frame', None)
                if frame is None:
                    raise ValueError('缺少已冻结的PX4任务坐标')
                converted = frame.to_px4(command, feedback)
            else:
                if command.get('frame') == PX4_FRAME:
                    raise ValueError('PX4任务坐标不能按雷达坐标补偿')
                converted = self.reference.to_px4(command, feedback,
                                                  map_velocity=full and not position_only, mode=mode)
        msg = self.Target()
        msg.header.stamp = self.rospy.Time.now()
        msg.header.frame_id = 'map'
        # ROS-side values are ENU. MAVROS performs the only ENU/NED conversion.
        msg.coordinate_frame = self.Target.FRAME_LOCAL_NED
        msg.type_mask = (self.Target.IGNORE_AFX | self.Target.IGNORE_AFY |
                         self.Target.IGNORE_AFZ | self.Target.IGNORE_YAW_RATE)
        msg.position.x, msg.position.y, msg.position.z = converted['position']
        msg.yaw = converted['yaw']
        if position_only:
            msg.type_mask |= self.Target.IGNORE_VX | self.Target.IGNORE_VY | self.Target.IGNORE_VZ
            converted['velocity'] = (0., 0., 0.)
            converted['setpoint_mode'] = 'position_yaw'
        else:
            msg.velocity.x, msg.velocity.y, msg.velocity.z = converted['velocity']
        if not position_only and 'acceleration' in converted:
            msg.type_mask = 0  # Full p/v/a/yaw/yaw-rate; FORCE is deliberately clear.
            msg.acceleration_or_force.x, msg.acceleration_or_force.y, msg.acceleration_or_force.z = converted['acceleration']
            msg.yaw_rate = converted['yaw_rate']
        self.publisher.publish(msg)
        return converted

    def input_fault(self):
        # Missing first inputs remain a startup error. Age/duplicate filters
        # keep the last accepted samples; a gap never revokes an active task.
        available = dict(lio=self.reference.latest is not None,
                         velocity=self.px4_velocity_sample is not None,
                         status=self.packet is not None)
        for key, label in (('lio', '雷达定位'),
                           ('velocity', '飞控速度'), ('status', '飞控状态')):
            if not available[key] or self.input_progress[key].stamp is None:
                return label+'尚未收到有效输入'
        return ''

    def latch_watchdog(self, reason, hold=False):
        if self.watchdog_fault and not (self.watchdog_holding and not hold):
            return None
        self.watchdog_fault, self.watchdog_holding = reason, bool(hold)
        self.start_epoch += 1  # Cancel pending arm/mode RPCs as well as task references.
        self.ego_command = self.ego_session = None
        self.clear_fixed_trajectory()
        if hold:
            self.hold_current('WATCHDOG_HOLD')
        return self.start_epoch

    def pause_faulted_task(self, epoch, reason):
        # Called without the runtime mutex, preserving the manager/runtime lock order.
        with self.manager.lock:
            if self.manager.engine is self.full_engine and self.start_epoch == epoch:
                self.full_engine.hold(reason)
                self.manager.active = self.manager.handoff_pending = False
                self.manager.request_compute('NONE')
                self.manager.last_output = self.full_engine.snapshot()
                self.manager.record_state_change(self.manager.last_output)
                self.manager.event('native_watchdog', reason=reason)

    def enable_full(self, manager):
        mode = reference_mode(getattr(manager.engine, 'config', {}))
        validate_task_mode(getattr(manager.engine, 'config', {}), getattr(manager.engine, 'mode', None))
        with self.mutex:
            self.clear_fixed_trajectory()
            self.start_epoch += 1
            self.manager, self.full_engine = manager, manager.engine
            self.full_task_id = getattr(manager, 'task_id', None)
            self.fixed_revoked = set()
            # Snapshot once per explicit task start, before the 50 Hz timer.
            self.reference_mode = mode
            self.full_enabled = True
            self.manual_takeover = self.seen_offboard = False
            self.full_owner_since = None
            self.ego_session = self.ego_command = None
            self.ego_timing = {}
            self.reference_command = None
            self.reference_source = 'HOLD'
            self.watchdog_fault, self.watchdog_holding = '', False
            self.reference_progress = SourceProgress()
            self.reference_stale = False
            self.last_send_wall = None
            self.last_reference_sent = None
            if self.trace is None:
                self.trace = (manager.log_dir / 'native_trajectory.jsonl').open('a', buffering=1)
            if self.full_timer is None:
                self.full_timer = self.rospy.Timer(self.rospy.Duration(.02), self._full_tick)
        self.hold_current('HOLD')
        if manager.engine.state != 'TAKEOFF':
            self.acquire_full(manager)

    def acquire_full(self, manager):
        """Explicit airborne debug entry acquires the same gateway, without arm/mode RPCs."""
        engine, epoch = manager.engine, self.start_epoch
        def run():
            from px4ctrl.srv import ControlOwner
            try:
                self.rospy.wait_for_service('/px4ctrl/control_owner', timeout=2.)
                with self.service_mutex:
                    with manager.lock:
                        if (manager.engine is not engine or epoch != self.start_epoch or
                                engine.state == 'HOLD' or self.manual_takeover): return
                    result = self.rospy.ServiceProxy('/px4ctrl/control_owner', ControlOwner)('PX4_NATIVE')
                if not result.accepted: raise ValueError(result.message)
                with self.mutex:
                    if epoch != self.start_epoch: return
                    self.full_owner_since = self.rospy.get_time()
                    self.seen_offboard = False
                manager.event('native_debug_acquired', owner=result.owner)
            except Exception as exc:
                with manager.lock:
                    if manager.engine is engine and epoch == self.start_epoch:
                        engine.hold('原生调试接管失败: '+str(exc))
                        self.hold_current()
                        manager.event('native_debug_acquire_failed', reason=str(exc))
        threading.Thread(target=run, daemon=True).start()

    def disable_full(self):
        with self.mutex:
            self.clear_fixed_trajectory()
            self.start_epoch += 1
            self.full_enabled = False
            self.ego_session = self.ego_command = self.reference_command = None



    def hold_current(self, source='HOLD'):
        with self.mutex:
            self.clear_fixed_trajectory()
            task = self.task_feedback()
            if task is None or self.packet is None:
                return
            self.reference_command = dict(position=task['position'], velocity=(0.,0.,0.),
                yaw=self.packet['yaw'], source_stamp=self.rospy.get_time(), frame=task['frame'],
                position_source=task['position_source'])
            self.reference_source = source
            self.ego_command = None
            self.reference_progress = SourceProgress()
            self.reference_progress.record(self.reference_command['source_stamp'], time.monotonic())

    def set_reference(self, command, source='HOLD'):
        with self.mutex:
            if self.watchdog_fault:
                return
            # Fixed trajectories own their 50 Hz sampler. A manager reference
            # replaces (and revokes) it; it must never reset that sampler.
            self.clear_fixed_trajectory()
            candidate = deepcopy(command)
            candidate.setdefault('source_stamp', self.rospy.get_time())
            if self.reference_source != source:
                self.reference_progress = SourceProgress()
            if not self.reference_progress.record(candidate['source_stamp'], time.monotonic()):
                return
            self.reference_command = candidate
            self.reference_source = source

    def clear_fixed_trajectory(self):
        """Revoke an installed trajectory without requesting another owner.

        Called under manager -> runtime lock order, or by the 50 Hz timer with
        only the runtime mutex. A late install of the same identity stays
        cancelled for the remainder of this explicitly started task.
        """
        with self.mutex:
            identity = getattr(self, 'fixed_identity', None)
            if identity is not None:
                self.fixed_revoked.add(identity)
            self.fixed_trajectory = self.fixed_identity = self.fixed_sample = None
            self.fixed_started_at = None
            if self.reference_source == 'FIXED_TRAJECTORY':
                self.reference_command = None
                self.reference_source = 'HOLD'

    def install_fixed_trajectory(self, manager, trajectory, yaw):
        """Install once, preserving the accepted clock on every manager tick."""
        with self.mutex:
            engine = manager.engine
            # A delivery task installs one trajectory per leg. The leg sequence
            # keeps a new leg distinct even if it reuses a freed object's id.
            identity = (getattr(manager, 'task_id', None), self.start_epoch,
                        id(engine), id(trajectory), getattr(trajectory, 'leg_sequence', 0))
            if (not self.full_enabled or self.manual_takeover or self.watchdog_fault or
                    manager is not self.manager or engine is not self.full_engine or
                    getattr(manager, 'task_id', None) != self.full_task_id or
                    not manager.active or manager.priority_landing or
                    getattr(engine, 'state', None) != 'APPROACH' or
                    getattr(engine, 'mode', None) not in ('waypoint_route', 'delivery_route', 'drop_test', 'drop_points') or
                    trajectory is None or trajectory is not getattr(engine, 'fixed_trajectory', None) or
                    identity in self.fixed_revoked):
                return False
            if identity == self.fixed_identity:
                return True
            self.clear_fixed_trajectory()
            self.fixed_trajectory, self.fixed_identity = trajectory, identity
            self.fixed_started_at = self.rospy.get_time()
            self.fixed_yaw = float(yaw)
            self.ego_session = self.ego_command = None
            self.reference_source = 'FIXED_TRAJECTORY'
            self.reference_progress = SourceProgress()
            self._sample_fixed_trajectory(self.fixed_started_at)
            return True

    def _sample_fixed_trajectory(self, now):
        # Only the independent publication timer advances this clock after the
        # initial sample. A ROS clock correction cannot run a route backwards.
        previous = self.fixed_sample or {}
        elapsed = min(self.fixed_trajectory.duration,
                      max(0., previous.get('elapsed_s', 0.), now-self.fixed_started_at))
        sample = dict(self.fixed_trajectory.sample(elapsed))
        task = self.task_feedback()
        sample.update(elapsed_s=elapsed, duration_s=self.fixed_trajectory.duration,
                      started_at=self.fixed_started_at, task_id=self.full_task_id,
                      start_epoch=self.start_epoch,
                      actual_position_xyz=list(task['position']) if task else None,
                      error_xyz=([a-b for a, b in zip(sample['position'], task['position'])]
                                 if task else None))
        self.fixed_sample = sample
        self.reference_command = dict(position=sample['position'], velocity=sample['velocity'],
            acceleration=sample['acceleration'], jerk=sample.get('jerk', (0., 0., 0.)),
            yaw=self.fixed_yaw, yaw_rate=0., source_stamp=now,
            trajectory_id='%s:%s' % (self.full_task_id, self.start_epoch),
            trajectory_elapsed_s=elapsed, completed_laps=sample.get('completed_laps', 0),
            lap=sample.get('lap', 0),
            frame=task['frame'] if task else LIO_FRAME,
            position_source=task['position_source'] if task else 'FAST_LIO')
        self.reference_progress.record(now, time.monotonic())

    def fixed_progress(self, manager):
        """Return the exact accepted timer sample, without advancing its clock."""
        with self.mutex:
            if (manager is not self.manager or manager.engine is not self.full_engine or
                    getattr(manager, 'task_id', None) != self.full_task_id or
                    self.reference_source != 'FIXED_TRAJECTORY'):
                return None
            return deepcopy(self.fixed_sample)

    def hold_ego_handoff(self):
        """Keep the route's existing wait reference while the next spline arrives."""
        with self.mutex:
            self.hold_current()
            engine = self.full_engine
            if (getattr(engine, 'mode', None) in ('waypoint_route', 'delivery_route', 'drop_test', 'drop_points')
                    and engine.state == 'APPROACH' and self.reference_command is not None):
                self.reference_command['position'] = tuple(engine._navigation_hold)
                self.reference_command['velocity'] = (0., 0., 0.)
                self.reference_command['yaw'] = engine.command_yaw

    def set_ego_goal(self, goal):
        with self.mutex:
            key = goal_key(goal)
            if key != getattr(self, 'ego_goal_key', None):
                self.ego_goal_key = key
                self.ego_command = None
                self.reference_source = 'HOLD'
                self.hold_ego_handoff()
                self.ego_timing = {}

    def ego_start_reference(self):
        """Freeze the reference actually held while EGO prepares this goal.

        Measured velocity is feedback, not the start of a replacement reference.
        In particular the completed takeoff holds cruise Z with zero velocity.
        """
        with self.mutex:
            return deepcopy(self.reference_command)

    def select_ego(self, session, goal=None):
        with self.mutex:
            if (feedback_source(getattr(getattr(self, 'full_engine', None), 'config', {})) == 'px4' and
                    getattr(self, 'px4_task_frame', None) is None):
                raise ValueError('缺少已冻结的PX4任务坐标')
            if self.watchdog_fault:
                return
            self.set_ego_goal(goal)
            if self.ego_session != session or self.reference_source != 'EGO':
                self.hold_ego_handoff()
                # Wait at the current position for this session's first command.
                self.reference_progress = SourceProgress()
                self.ego_timing = {}
            self.ego_session, self.reference_source = session, 'EGO'

    def clear_ego(self):
        with self.mutex:
            self.ego_session = self.ego_command = None
            self.ego_goal_key = None
            self.ego_timing = {}
            if self.reference_source == 'EGO':
                self.hold_ego_handoff()

    def ego_reference(self, session):
        """Read the accepted native reference for noncritical engine telemetry."""
        with self.mutex:
            if (not self.full_enabled or self.manual_takeover or self.watchdog_fault or
                    self.reference_source != 'EGO' or session != self.ego_session):
                return None
            return deepcopy(self.ego_command)

    def feed_ego(self, session, msg, goal=None):
        callback_ros, callback_wall = self.rospy.get_time(), time.monotonic()
        with self.mutex:
            if not session or self.watchdog_fault or not self.full_enabled or self.manual_takeover or self.reference_source != 'EGO' or session != self.ego_session:
                return False
            if goal_key(goal) != getattr(self, 'ego_goal_key', None):
                return False
            if msg.header.frame_id != LIO_FRAME:
                raise ValueError('EGO reference is not in '+LIO_FRAME)
            xyz = lambda v: tuple(vector((v.x,v.y,v.z)))
            candidate = dict(position=xyz(msg.position), velocity=xyz(msg.velocity),
                acceleration=xyz(msg.acceleration), yaw=float(msg.yaw), yaw_rate=float(msg.yaw_dot),
                source_stamp=msg.header.stamp.to_sec(), trajectory_id=int(msg.trajectory_id),
                session_id=session, frame=LIO_FRAME, goal_key=goal_key(goal))
            now, wall = self.rospy.get_time(), time.monotonic()
            previous = getattr(self, 'ego_timing', {})
            self.ego_timing = dict(session_id=session, source_stamp=candidate['source_stamp'],
                callback_ros_time=callback_ros,
                source_age_at_callback_s=callback_ros-candidate['source_stamp'],
                callback_gap_s=(callback_ros-previous['callback_ros_time']
                                if 'callback_ros_time' in previous else None),
                callback_to_accept_s=wall-callback_wall,
                accepted_count=previous.get('accepted_count', 0),
                rejected_stale_count=previous.get('rejected_stale_count', 0),
                rejected_nonprogress_count=previous.get('rejected_nonprogress_count', 0))
            if not self.reference_progress.record(candidate['source_stamp'], wall):
                return False
            self.ego_timing['accepted_count'] += 1
            self.ego_command = candidate
            return True

    def trace_context(self, feedback):
        """Diagnostic identity and fixed endpoints; never gates control output."""
        engine = self.full_engine
        origin = getattr(engine, 'origin_xyz', None)
        cruise_z = getattr(engine, 'cruise_z', None)
        points = getattr(engine, 'route_points', ())
        return dict(task_id=getattr(self, 'full_task_id', None),
                    mode=getattr(engine, 'mode', None), state=getattr(engine, 'state', None),
                    phase=getattr(engine, 'phase', None),
                    waypoint_index=getattr(engine, 'route_index', None), waypoint_count=len(points),
                    ground_origin=origin, target_xyz=getattr(engine, 'command_position', None),
                    takeoff_target_xyz=([origin[0], origin[1], cruise_z]
                                        if origin is not None and cruise_z is not None else None),
                    control_status={key: feedback.get(key) for key in
                                    ('owner', 'armed', 'offboard', 'mode', 'manual_paused', 'watchdog_fault')})

    def _full_tick(self, _event):
        # Independent of the 30 Hz task loop; no RPCs or image processing here.
        takeover_epoch = None
        fault_epoch, fault_reason = None, ''
        with self.mutex:
            if not self.full_enabled or self.manual_takeover:
                return
            self.publish_task_odometry()
            feedback = deepcopy(self.packet) or {}
            # Keep the accepted reference unchanged across source-time gaps.
            # Only actual gateway faults revoke this task. Ordinary input gaps
            # keep the cached point and recover when valid samples resume.
            owner_since = self.full_owner_since
            current_owner_status = owner_since is not None and feedback.get('stamp', 0.) >= owner_since
            fault_reason = self.input_fault()
            if current_owner_status and feedback.get('watchdog_fault'):
                fault_reason = feedback.get('watchdog_reason') or '飞控指令保护已触发'
            if fault_reason:
                fault_epoch = self.latch_watchdog(fault_reason)
                self.watchdog_holding = False
            # Input/reference age never pauses a task. A gateway connection
            # fault or manual takeover can still revoke this ownership.
            landing = feedback.get('owner') == 'PX4_AUTO_LAND'
            if current_owner_status:
                # The gateway latches even a brief mode exit. Ignore pre-acquire
                # status from the prior task when explicitly starting a new one.
                if feedback.get('manual_paused') or (self.seen_offboard and not landing and not feedback.get('offboard')):
                    self.manual_takeover = True
                    self.clear_fixed_trajectory()
                    takeover_epoch = self.start_epoch
                elif feedback.get('offboard'):
                    self.seen_offboard = True
            if landing:
                self.clear_fixed_trajectory()
            source = self.reference_source
            if source == 'FIXED_TRAJECTORY' and self.fixed_trajectory is not None:
                if (getattr(self.full_engine, 'state', None) in ('HOLD', 'LAND') or
                        self.fixed_trajectory is not getattr(self.full_engine, 'fixed_trajectory', None)):
                    self.clear_fixed_trajectory()
                    source = self.reference_source
                else:
                    self._sample_fixed_trajectory(self.rospy.get_time())
            command = deepcopy(self.ego_command if source == 'EGO' and self.ego_command else self.reference_command)
            can_publish = command is not None and not landing and (not self.watchdog_fault or self.watchdog_holding)
            if takeover_epoch is None and can_publish:
                try:
                    command['reference_source'] = source
                    # Selection and publish share the mutex: an old task or
                    # session cannot publish after its replacement is committed.
                    converted = self.publish(command, full=True)
                    now = time.monotonic()
                    radar = deepcopy(self.reference.latest)
                    task = self.task_feedback()
                    interval = None if self.last_send_wall is None else now-self.last_send_wall
                    self.last_send_wall = now
                    self.last_converted = converted
                    previous = getattr(self, 'last_reference_sent', None)
                    switch = None
                    if previous and any(previous.get(k) != command.get(k) for k in
                                        ('reference_source', 'session_id', 'trajectory_id')):
                        switch = dict(from_source=previous.get('reference_source'), to_source=source,
                            previous_trajectory_id=previous.get('trajectory_id'),
                            delta_position=[a-b for a,b in zip(command['position'], previous['position'])],
                            delta_velocity=[a-b for a,b in zip(command['velocity'], previous['velocity'])])
                    self.last_reference_sent = deepcopy(command)
                    if self.trace:
                        self.trace.write(json.dumps(dict(wall_time=time.time(), stamp=self.rospy.get_time(),
                            controller='PX4_NATIVE', reference_source=source, command=command,
                            px4_command=converted, radar=radar, interval_s=interval, reference_switch=switch,
                            task_feedback=task,
                            ego_timing=(dict(getattr(self, 'ego_timing', {})) if source == 'EGO' else None),
                            fixed_trajectory=(deepcopy(self.fixed_sample) if source == 'FIXED_TRAJECTORY' else None),
                            battery=dict(getattr(self, 'battery_sample', None) or {}) or None,
                            reference_stale_hold=bool(command.get('reference_stale_hold')),
                            error_xyz=None if task is None else
                                [a-b for a,b in zip(command['position'],task['position'])],
                            **self.trace_context(feedback)), allow_nan=False)+'\n')
                except (ValueError, TypeError, KeyError) as exc:
                    self.rospy.logerr_throttle(1., '原生参考接口: %s', exc)
        if fault_epoch is not None:
            self.pause_faulted_task(fault_epoch, fault_reason)
        if takeover_epoch is not None:
            # Never take the manager lock while holding the runtime mutex.
            with self.manager.lock:
                if self.manager.engine is self.full_engine and self.start_epoch == takeover_epoch:
                    self.full_engine.hold('飞控已退出 OFFBOARD，任务停止自动推进；退出原因需结合飞控日志')
                    self.manager.active = self.manager.handoff_pending = False
                    self.manager.request_compute('NONE')
                    self.manager.last_output = self.full_engine.snapshot()
                    self.manager.record_state_change(self.manager.last_output)
                    self.manager.event('native_offboard_left', mode=feedback.get('mode'))

    def start_takeoff(self, manager, effect):
        engine, epoch = manager.engine, self.start_epoch
        self.hold_current('TAKEOFF')
        def current():
            with manager.lock:
                return (manager.engine is engine and engine.state == 'TAKEOFF' and
                        epoch == self.start_epoch and not self.manual_takeover and not self.watchdog_fault
                        and not manager.priority_landing)
        def run():
            from px4ctrl.srv import ControlOwner
            from mavros_msgs.srv import SetMode, CommandBool
            try:
                self.rospy.wait_for_service('/px4ctrl/control_owner', timeout=2.)
                with self.service_mutex:
                    if not current(): return
                    self.require_fresh_start_inputs()
                    result = self.rospy.ServiceProxy('/px4ctrl/control_owner', ControlOwner)('PX4_NATIVE')
                if not result.accepted:
                    raise ValueError(result.message)
                with self.mutex:
                    if epoch != self.start_epoch: return
                    self.full_owner_since = self.rospy.get_time()
                    self.seen_offboard = False
                until = time.monotonic()+1.2
                while time.monotonic()<until:
                    if not current() or self.rospy.is_shutdown(): return
                    time.sleep(.02)
                if not current(): return
                self.rospy.wait_for_service('/mavros/set_mode', timeout=2.)
                if not current(): return
                self.require_fresh_start_inputs()
                result = self.rospy.ServiceProxy('/mavros/set_mode', SetMode)(base_mode=0, custom_mode='OFFBOARD')
                if not result.mode_sent: raise ValueError('MAVROS未发送OFFBOARD请求')
                # MAVROS mode_sent only acknowledges sending SET_MODE. PX4's
                # heartbeat may still be STABILIZED, where centered throttle
                # can reject arming. Keep the ground reference until the actual
                # mode feedback confirms OFFBOARD; never retry arm blindly.
                until = time.monotonic()+3.
                while True:
                    if not current() or self.rospy.is_shutdown(): return
                    feedback = self.feedback() or {}
                    if feedback.get('offboard'): break
                    if time.monotonic() >= until:
                        raise ValueError('等待OFFBOARD模式反馈超时，当前模式='+str(feedback.get('mode', '未知')))
                    time.sleep(.02)
                if not current(): return
                if self.rospy.get_param('/px4ctrl/auto_takeoff_land/enable_auto_arm', True):
                    self.rospy.wait_for_service('/mavros/cmd/arming', timeout=2.)
                    if not current(): return
                    self.require_fresh_start_inputs()
                    if not (self.feedback() or {}).get('offboard'):
                        raise ValueError('解锁前已退出OFFBOARD，未发送解锁请求')
                    result = self.rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)(value=True)
                    if not result.success:
                        raise ValueError('PX4未接受解锁请求，结果码='+str(getattr(result, 'result', '未知')))
                if current(): manager.event('native_takeoff_requested', message='等待实际OFFBOARD与armed反馈')
            except Exception as exc:
                with manager.lock:
                    if current():
                        engine.hold('原生起飞请求失败: '+str(exc))
                        self.hold_current()
                        manager.event('native_takeoff_failed', reason=str(exc))
        threading.Thread(target=run, daemon=True).start()

    def require_fresh_start_inputs(self):
        with self.mutex:
            fault = self.input_fault()
            if fault:
                raise ValueError(fault)
            packet = self.packet or {}
            if self.full_owner_since is not None and packet.get('stamp', 0.) >= self.full_owner_since:
                if packet.get('watchdog_fault') or packet.get('manual_paused'):
                    # This check also runs if the publication timer has stalled
                    # and therefore has not copied the gateway fault locally.
                    raise ValueError('飞控指令保护或人工接管已触发，未继续起飞请求')

    def close(self):
        self.disable_full()
        if self.full_timer is not None: self.full_timer.shutdown()
        with self.mutex:
            if self.trace is not None: self.trace.close(); self.trace = None

    def request(self, manager, effect):
        engine, requested = manager.engine, effect['owner']
        def call():
            from px4ctrl.srv import ControlOwner
            accepted, owner, message = False, '', ''
            capture = None
            with self.service_mutex:
                try:
                    self.rospy.wait_for_service('/px4ctrl/control_owner', timeout=1.)
                    with manager.lock:
                        if manager.engine is not engine or engine.native_pending != requested:
                            return
                    service = self.rospy.ServiceProxy('/px4ctrl/control_owner', ControlOwner)
                    result = service(requested)
                    accepted, owner, message = result.accepted, result.owner, result.message
                    if accepted and requested=='PX4_NATIVE':
                        capture=dict(position=result.position_enu, yaw=result.yaw_enu, stamp=result.pose_stamp)
                    with manager.lock:
                        cancelled = manager.engine is not engine or engine.native_pending != requested
                    if cancelled and accepted and requested == 'PX4_NATIVE':
                        service('PX4_HOLD')  # A late acquisition must never restart a stopped task.
                except Exception as exc:
                    message = str(exc)
                with manager.lock:
                    if manager.engine is engine:
                        engine.on_native_owner_result(requested, accepted, owner, message, capture=capture)
                    manager.event('native_owner_result', requested=requested, accepted=accepted,
                                  owner=owner, message=message)
        threading.Thread(target=call, daemon=True).start()
