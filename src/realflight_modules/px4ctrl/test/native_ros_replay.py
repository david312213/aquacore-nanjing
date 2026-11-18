#!/usr/bin/env python3
"""Run the real px4ctrl binary against simulated MAVROS endpoints, never a FCU.

Requires a separate roscore on 11346. No MAVROS node may exist on that master.
This is ROS integration replay, not a PX4 dynamics/SITL accuracy test.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import signal
import threading
import time
import yaml
import rospy
import rosnode
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from mavros_msgs.msg import State, ExtendedState, EstimatorStatus, AttitudeTarget, PositionTarget
from mavros_msgs.srv import SetMode, SetModeResponse, CommandBool, CommandBoolResponse
from quadrotor_msgs.msg import PositionCommand
from px4ctrl.srv import ControlOwner


def wait_for(predicate, timeout=5.):
    until=time.monotonic()+timeout
    while time.monotonic()<until:
        if predicate(): return
        time.sleep(.02)
    raise AssertionError('condition timeout')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--binary',required=True)
    parser.add_argument('--config',required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_MASTER_URI')!='http://127.0.0.1:11346':
        raise RuntimeError('requires dedicated loopback master port 11346')
    rospy.init_node('native_ros_replay')
    nodes=rosnode.get_node_names()
    if '/mavros' in nodes or ('/px4ctrl' in nodes and rosnode.rosnode_ping('/px4ctrl',max_count=1,verbose=False)):
        raise RuntimeError('existing MAVROS/controller detected; refusing to run')
    config=yaml.safe_load(Path(args.config).read_text())
    config.pop('native_control',None)
    config['auto_takeoff_land'].update(no_RC=True,enable=True,enable_auto_arm=True)
    config['native_stop_on_mode_exit']=True
    rospy.set_param('/px4ctrl',config)
    mode,armed,z=['POSCTL'],[False],[0.]
    connected,bridge_healthy=[True],[False]
    native_stream,pose_stream,running=[False],[False],[True]
    native_mask=[2496]
    frozen_command_stamp=[None]
    frozen_input_stamp=[None]
    px4_x,px4_vx=[2.],[0.]
    horizontal_aiding,constant_position=[False],[True]
    old_stream=[True]
    status,outputs,mode_calls={},{'attitude':[],'local':[]},[]
    publishers={}
    def pub(topic,kind):
        publishers[topic]=rospy.Publisher(topic,kind,queue_size=1)
    for topic,kind in (
        ('/mavros/state',State),('/mavros/extended_state',ExtendedState),
        ('/Odom_high_freq',Odometry),('/mavros/imu/data',Imu),
        ('/mavros/local_position/pose',PoseStamped),('/mavros/local_position/velocity_local',TwistStamped),
        ('/mavros/estimator_status',EstimatorStatus),('/airseafly/native/localization_healthy',Bool),
        ('/px4ctrl/native_setpoint',PositionTarget),('/mavros/setpoint_raw/target_attitude',AttitudeTarget),
        ('/position_cmd',PositionCommand)):
        pub(topic,kind)
    def setmode(req):
        if req.custom_mode=='AUTO.LAND': time.sleep(2.8)
        mode[0]=req.custom_mode; mode_calls.append(mode[0]); return SetModeResponse(mode_sent=True)
    def arm(req):
        armed[0]=req.value; return CommandBoolResponse(success=True,result=0)
    servers=[rospy.Service('/mavros/set_mode',SetMode,setmode),rospy.Service('/mavros/cmd/arming',CommandBool,arm)]
    def feedback(msg): status.update(json.loads(msg.data))
    subscriptions=[rospy.Subscriber('/px4ctrl/native_status',String,feedback),
                   rospy.Subscriber('/px4ctrl/control_state',String,lambda m:status.update(control=m.data))]
    for name,topic,kind in (('attitude','/mavros/setpoint_raw/attitude',AttitudeTarget),
                             ('local','/mavros/setpoint_raw/local',PositionTarget)):
        subscriptions.append(rospy.Subscriber(topic,kind,
            lambda msg,key=name: outputs[key].append((time.monotonic(),msg))))
    def stream():
        while running[0] and not rospy.is_shutdown():
            now=rospy.Time.now()
            state=State(); state.header.stamp=now; state.connected=connected[0]; state.armed=armed[0]; state.mode=mode[0]
            publishers['/mavros/state'].publish(state)
            ext=ExtendedState(); ext.landed_state=2 if armed[0] else 1
            publishers['/mavros/extended_state'].publish(ext)
            odom=Odometry(); odom.header.stamp=now; odom.pose.pose.orientation.w=1.; odom.pose.pose.position.z=z[0]
            publishers['/Odom_high_freq'].publish(odom)
            imu=Imu(); imu.header.stamp=now; imu.orientation.w=1.; imu.linear_acceleration.z=9.81
            publishers['/mavros/imu/data'].publish(imu)
            if pose_stream[0]:
                pose=PoseStamped(); pose.header.stamp=frozen_input_stamp[0] or now; pose.pose.orientation.w=1.
                pose.pose.position.x=px4_x[0]; pose.pose.position.y=3.; pose.pose.position.z=z[0]
                publishers['/mavros/local_position/pose'].publish(pose)
                vel=TwistStamped(); vel.header.stamp=frozen_input_stamp[0] or now; vel.twist.linear.x=px4_vx[0]
                publishers['/mavros/local_position/velocity_local'].publish(vel)
            estimator=EstimatorStatus(); estimator.header.stamp=now
            for key in ('attitude','velocity_horiz','velocity_vert','pos_horiz_rel','pos_vert_abs'):
                setattr(estimator,key+'_status_flag',True)
            estimator.pred_pos_horiz_rel_status_flag=horizontal_aiding[0]
            estimator.const_pos_mode_status_flag=constant_position[0]
            publishers['/mavros/estimator_status'].publish(estimator)
            publishers['/airseafly/native/localization_healthy'].publish(Bool(data=bridge_healthy[0]))
            if native_stream[0]:
                target=PositionTarget(); target.header.stamp=frozen_command_stamp[0] or now
                target.coordinate_frame=1; target.type_mask=native_mask[0]
                target.position.x=2.1; target.position.y=3.; target.position.z=z[0]
                target.velocity.z=.3
                publishers['/px4ctrl/native_setpoint'].publish(target)
            if old_stream[0] and status.get('owner')=='PX4_NATIVE':
                # A stale EGO producer cannot bypass native ownership.
                old=PositionCommand(); old.header.stamp=now; old.position.x=50.; old.trajectory_flag=1
                publishers['/position_cmd'].publish(old)
            time.sleep(.01)
    worker=threading.Thread(target=stream,daemon=True); worker.start()
    log=Path(args.binary).parent/'native_ros_replay.log'
    with log.open('w') as output:
        process=subprocess.Popen([args.binary,'__name:=px4ctrl','~odom:=/Odom_high_freq','~cmd:=/position_cmd'],stdout=output,stderr=subprocess.STDOUT)
        try:
            rospy.wait_for_service('/px4ctrl/control_owner',timeout=8.)
            owner=rospy.ServiceProxy('/px4ctrl/control_owner',ControlOwner)
            def acquire():
                result=owner('PX4_NATIVE')
                assert result.accepted,result.message
                wait_for(lambda:status.get('owner')=='PX4_NATIVE')
            wait_for(lambda:'healthy' in status)
            assert not status['healthy']
            # Acquisition requires fresh localization and pose; no arm/mode
            # command is issued by this simulated fixture.
            missing=owner('PX4_NATIVE')
            assert not missing.accepted and 'pose' in missing.message
            pose_stream[0]=True
            bridge_healthy[0]=horizontal_aiding[0]=True
            wait_for(lambda:status.get('pose_stamp',0.)>0.)
            wait_for(lambda:status.get('healthy'))
            acquire()
            mark=time.monotonic(); time.sleep(.2)
            assert not [x for x in outputs['attitude'] if x[0]>mark+.05]
            assert len([x for x in outputs['local'] if x[0]>mark+.05])>=3
            assert abs(outputs['local'][-1][1].position.x-2.)<1e-6
            native_stream[0]=True
            wait_for(lambda:abs(outputs['local'][-1][1].position.x-2.1)<1e-6)
            native_mask[0]=2552
            wait_for(lambda:outputs['local'][-1][1].type_mask==2552)
            assert abs(outputs['local'][-1][1].position.x-2.1)<1e-6
            mark=time.monotonic(); time.sleep(.15)
            assert not [x for x in outputs['attitude'] if x[0]>mark+.05]
            native_mask[0]=2496
            wait_for(lambda:outputs['local'][-1][1].type_mask==2496)
            frozen_command_stamp[0]=rospy.Time.now()
            time.sleep(.15)
            # Source time is retained for diagnostics, with no age-based rejection.
            assert outputs['local'][-1][1].header.stamp==frozen_command_stamp[0]
            time.sleep(.45)
            assert status['owner']=='PX4_NATIVE' and not status['watchdog_fault']
            assert outputs['local'][-1][1].type_mask==2496
            assert outputs['local'][-1][1].header.stamp==frozen_command_stamp[0]
            assert abs(outputs['local'][-1][1].position.x-2.1)<1e-6
            assert abs(outputs['local'][-1][1].velocity.z-.3)<1e-6
            frozen_command_stamp[0]=rospy.Time.now()-rospy.Duration(2.)
            wait_for(lambda:outputs['local'][-1][1].header.stamp==frozen_command_stamp[0])
            assert abs(outputs['local'][-1][1].velocity.z-.3)<1e-6
            frozen_command_stamp[0]=None
            wait_for(lambda:outputs['local'][-1][1].type_mask==2496)
            frozen_input_stamp[0]=rospy.Time.now()-rospy.Duration(2.)
            px4_x[0],px4_vx[0]=2.3,.4
            wait_for(lambda:abs(status.get('position',[0.])[0]-2.3)<1e-6 and
                            abs(status.get('velocity',[0.])[0]-.4)<1e-6)
            frozen_input_stamp[0]=frozen_input_stamp[0]-rospy.Duration(1.)
            px4_x[0],px4_vx[0]=2.4,.5
            wait_for(lambda:abs(status.get('position',[0.])[0]-2.4)<1e-6 and
                            abs(status.get('velocity',[0.])[0]-.5)<1e-6)
            frozen_input_stamp[0]=None
            px4_x[0],px4_vx[0]=2.,0.
            wait_for(lambda:status.get('pose_fresh'))
            assert status['owner']=='PX4_NATIVE' and not status['watchdog_fault']
            pose_stream[0]=False
            mark=time.monotonic(); time.sleep(.45)
            assert not status['pose_fresh'] and not status['watchdog_fault']
            assert status['owner']=='PX4_NATIVE'
            assert len([x for x in outputs['local'] if x[0]>mark+.35])>=3
            pose_stream[0]=True
            wait_for(lambda:status.get('pose_fresh'))
            # A reported FCU disconnect, unlike a short source-age gap, still stops output.
            pose_stream[0]=False; connected[0]=False
            mark=time.monotonic(); time.sleep(.45)
            assert not status['healthy'] and status['watchdog_fault']
            assert not [x for x in outputs['local'] if x[0]>mark+.35]
            assert not [x for x in outputs['attitude'] if x[0]>mark+.05]
            connected[0]=True; pose_stream[0]=True; px4_x[0]=2.6
            wait_for(lambda:abs(status.get('position',[0.])[0]-2.6)<1e-6)
            mark=time.monotonic(); time.sleep(.15)
            assert not [x for x in outputs['local'] if x[0]>mark]
            acquire()
            assert owner('PX4_HOLD').accepted
            wait_for(lambda:abs(outputs['local'][-1][1].position.x-2.6)<1e-6)
            time.sleep(.4)
            assert not status['watchdog_fault']  # An intentional hold persists.
            acquire()
            mode[0]='OFFBOARD'; armed[0]=True
            wait_for(lambda:status.get('offboard'))
            mode[0]='ALTCTL'; calls=len(mode_calls)
            time.sleep(.4)
            assert status['manual_paused'] and len(mode_calls)==calls
            mark=time.monotonic(); time.sleep(.15)
            assert not [x for x in outputs['local'] if x[0]>mark]
            # Handoff needs no stationary gate or measured thrust sample.
            # Configured hover thrust supplies the missing measurement.
            mode[0]='OFFBOARD'; armed[0]=True; px4_vx[0]=1.
            time.sleep(.15)
            assert status['manual_paused']
            old_stream[0]=False
            old_stamp=rospy.Time.now()
            assert owner('PX4CTRL').accepted
            old=PositionCommand(); old.header.stamp=old_stamp; old.position.x=50.; old.trajectory_flag=1
            publishers['/position_cmd'].publish(old)
            mark=time.monotonic(); time.sleep(.2)
            assert status.get('control')=='AUTO_HOVER',status
            assert not [x for x in outputs['local'] if x[0]>mark+.05]
            assert [x for x in outputs['attitude'] if x[0]>mark+.05]
            acquire()
            # Bridge age/health loss does not request a mode or revoke the task.
            calls=len(mode_calls)
            bridge_healthy[0]=False
            wait_for(lambda:not status.get('bridge_healthy'))
            mark=time.monotonic(); time.sleep(.45)
            assert status['owner']=='PX4_NATIVE' and not status['watchdog_fault']
            assert len(mode_calls)==calls
            assert len([x for x in outputs['local'] if x[0]>mark+.35])>=3
            bridge_healthy[0]=True
            wait_for(lambda:status.get('bridge_healthy'))
            # A slow single RPC must retain valid hold beyond the old two-second
            # cutoff; only actual mode feedback confirms the landing handoff.
            assert owner('PX4_AUTO_LAND').accepted
            wait_for(lambda:status.get('owner')=='PX4_AUTO_LAND')
            requested=time.monotonic()
            time.sleep(2.2)
            assert status['owner']=='PX4_AUTO_LAND' and not status['watchdog_fault']
            assert status['landing_state']=='requesting' and status['landing_attempts']==1
            assert status['mode']=='OFFBOARD'
            assert len([x for x in outputs['local'] if x[0]>requested+2.05])>=3
            holds=[x[1] for x in outputs['local'] if requested+.1<x[0]]
            assert len(holds)>=45
            assert all(abs(m.velocity.z)<1e-6 for m in holds)
            wait_for(lambda:status.get('mode')=='AUTO.LAND' and status.get('landing_state')=='confirmed')
            mark=time.monotonic(); time.sleep(.2)
            assert not [x for rows in outputs.values() for x in rows if x[0]>mark+.05]
            print(json.dumps(dict(passed=True,checks=['exclusive_outputs','missing_pose_reported',
                'fresh_localization_required','duplicate_command_accepted_without_age_or_progress_gate',
                'old_and_backwards_command_pose_velocity_accepted',
                'pose_gap_keeps_output','fresh_data_resumes_without_acquisition',
                'actual_connection_loss_stops_output','connection_recovery_requires_acquisition','intentional_hold_persists',
                'manual_mode_exit_latches','moving_handoff_without_thrust_measurement',
                'old_ego_cleared','bridge_gap_does_not_request_land','explicit_land_hold_beyond_two_seconds',
                'actual_land_mode_confirms_and_stops_hold'],attitude_count=len(outputs['attitude']),
                local_count=len(outputs['local']),log=str(log))))
        finally:
            running[0]=False; process.send_signal(signal.SIGINT)
            try: process.wait(timeout=5.)
            except subprocess.TimeoutExpired: process.kill()


if __name__=='__main__': main()
