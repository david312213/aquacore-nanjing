"""Radar reference, time alignment and the outgoing ROS setpoint boundary."""
from pathlib import Path
import math
import sys
import threading
from types import SimpleNamespace as NS
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from airseafly_real400_mission.native_reference import LioReference, LIO_FRAME
from airseafly_real400_mission.native_runtime import NativeRuntime
from airseafly_real400_mission.native_motion import FrozenLioFrame, rotation, quaternion


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.ref=LioReference()
        self.command=dict(position=(4.,5.,6.),velocity=(.02,-.03,.04),yaw=.7)

    def test_outgoing_position_preserves_radar_error_across_px4_offsets(self):
        radar=np.array([1.,2.,3.])
        self.ref.feed(10.,radar,(0.,0.,0.))
        for offset in ((10.,-20.,30.),(-100.,200.,-300.),(0.,0.,0.)):
            px4=radar+offset
            converted=self.ref.to_px4(self.command,dict(pose_stamp=10.,position=px4))
            np.testing.assert_allclose(np.array(converted['position'])-px4,
                                       np.array(self.command['position'])-radar)
            np.testing.assert_allclose(converted['translation_xyz'],offset)
            self.assertEqual(converted['velocity'],self.command['velocity'])
            self.assertEqual(converted['yaw'],self.command['yaw'])
            self.assertEqual(converted['source_frame'],LIO_FRAME)
            self.assertEqual(self.command['position'],(4.,5.,6.))

    def test_transport_delay_is_not_mistaken_for_position_bias(self):
        self.ref.feed(10.,(1.,2.,3.),(.1,0.,0.))
        self.ref.feed(10.2,(1.02,2.,3.),(.1,0.,0.))
        converted=self.ref.to_px4(self.command,dict(pose_stamp=10.1,position=(8.01,-2.,5.)))
        np.testing.assert_allclose(converted['translation_xyz'],(7.,-4.,2.))
        np.testing.assert_allclose(converted['position'],(11.,1.,8.))
        self.assertEqual(converted['sample_alignment'],'interpolated')

    def test_exact_and_unbracketed_samples_do_not_expire(self):
        self.ref.feed(10.,(1.,2.,3.),(0.,0.,0.))
        for stamp,label in ((-100.,'nearest'),(10.,'exact'),(1000.,'nearest')):
            converted=self.ref.to_px4(self.command,dict(pose_stamp=stamp,position=(8.,-2.,5.)))
            np.testing.assert_allclose(converted['translation_xyz'],(7.,-4.,2.))
            self.assertEqual(converted['sample_alignment'],label)
        self.ref.feed(10.,(2.,2.,3.),(0.,0.,0.))
        self.assertEqual(len(self.ref.history),1)
        self.assertEqual(self.ref.position_at(10.)[0][0],2.)

    def test_missing_inputs_do_not_replace_radar_with_ekf(self):
        with self.assertRaisesRegex(ValueError,'FAST-LIO'):
            self.ref.to_px4(self.command,dict(pose_stamp=10.,position=(0.,0.,0.)))
        self.ref.feed(10.,(1.,2.,3.),(0.,0.,0.))
        with self.assertRaisesRegex(ValueError,'PX4 pose'):
            self.ref.to_px4(self.command,dict(pose_received=False))

    def test_tilted_world_mount_and_ekf_offset_share_one_conversion_chain(self):
        r_nw=rotation((.18,-.09,.26,.94))
        r_bi=rotation((.04,.03,-.08,.99))
        r_nb=rotation((.03,-.02,.21,.98))
        t_bi=np.array([.12,-.06,.035])
        raw_ground=np.array([20.,-4.,8.])
        raw_orientation=quaternion(r_nw.T@r_nb@r_bi)
        frame=FrozenLioFrame(quaternion(r_bi),t_bi)
        frame.initialize(raw_ground,raw_orientation,(0.,0.,0.),quaternion(r_nb))
        radar,_,_,_=frame.transform(raw_ground+r_nw.T@np.array([.3,-.2,.65]),
                                   raw_orientation,(0.,0.,0.),(0.,0.,0.))
        np.testing.assert_allclose(radar,(.3,-.2,.65),atol=1e-12)
        self.ref.feed(10.,radar,(0.,0.,0.))
        for height in (.25,.20):
            command=dict(position=(.3,-.2,height),velocity=(0.,0.,-.15),yaw=0.)
            px4=radar+np.array([5.,8.,-3.])
            converted=self.ref.to_px4(command,dict(pose_stamp=10.,position=px4))
            np.testing.assert_allclose(np.array(converted['position'])-px4,(0.,0.,height-.65),atol=1e-12)


class RuntimeBoundaryTests(unittest.TestCase):
    def test_body_velocity_rotates_to_radar_enu_and_mavros_receives_enu(self):
        from airseafly_real400_mission.native_freshness import SourceProgress
        runtime=NativeRuntime.__new__(NativeRuntime)
        runtime.mutex=threading.Lock()
        runtime.rospy=NS(get_time=lambda:10.)
        runtime.input_progress={'lio':SourceProgress()}
        runtime.reference=LioReference()
        runtime.packet=dict(pose_stamp=10.,position=(8.,-2.,5.))
        v=lambda x=0.,y=0.,z=0.:NS(x=x,y=y,z=z)
        msg=NS(header=NS(stamp=NS(to_sec=lambda:10.)),
            pose=NS(pose=NS(position=v(1.,2.,3.),
                orientation=NS(x=0.,y=0.,z=math.sqrt(.5),w=math.sqrt(.5)))),
            twist=NS(twist=NS(linear=v(.1,0.,0.))))
        runtime._lio_feedback(msg)
        np.testing.assert_allclose(runtime.lio_feedback()['velocity'],(0.,.1,0.),atol=1e-12)
        class Target:
            FRAME_LOCAL_NED=1
            IGNORE_AFX=64; IGNORE_AFY=128; IGNORE_AFZ=256; IGNORE_YAW_RATE=2048
            def __init__(self):
                self.header=NS(); self.position=v(); self.velocity=v()
        runtime.Target=Target
        runtime.rospy=NS(Time=NS(now=lambda:123.))
        sent=[]
        runtime.publisher=NS(publish=sent.append)
        result=runtime.publish(dict(position=(1.1,1.8,3.25),velocity=(.02,-.03,.04),yaw=.7))
        self.assertEqual(len(sent),1)
        target=sent[0]
        np.testing.assert_allclose((target.position.x,target.position.y,target.position.z),(8.1,-2.2,5.25))
        self.assertEqual((target.velocity.x,target.velocity.y,target.velocity.z),(.02,-.03,.04))
        self.assertEqual(target.coordinate_frame,Target.FRAME_LOCAL_NED)
        self.assertEqual(target.type_mask,2496)  # Position, velocity and yaw remain enabled.
        self.assertEqual(target.header.frame_id,'map')
        self.assertEqual(target.yaw,.7)
        self.assertEqual(result['source_frame'],LIO_FRAME)
        np.testing.assert_allclose(result['translation_xyz'],(7.,-4.,2.))


if __name__=='__main__': unittest.main()
