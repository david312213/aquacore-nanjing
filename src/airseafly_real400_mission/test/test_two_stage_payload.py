"""Two physical dispensers, four inventory slots; all transport is mocked."""
from copy import deepcopy
import math
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

from airseafly_real400_mission.flight_configuration import base_configuration, apply_flight_configuration
from airseafly_real400_mission.payload import actuator_command, reset_slots
from airseafly_real400_mission.main_servo_test import (
    actuator_request, inspect_mapping, main_pwm_request, run_reset, send_once,
)
import test_main1_pwm as cli_helpers
from test_native_delivery_route import Replay, configuration


class TwoStagePayloadTests(unittest.TestCase):
    def setUp(self):
        self.config = apply_flight_configuration(base_configuration(), task='delivery-route')
        self.payload = self.config['payload']
        # These wire-encoding expectations are a test fixture, not assertions
        # that the operator's current field PWM settings equal historical ones.
        self.payload['released_pwm_us_by_slot'] = {'1':1190, '2':1190, '3':1350, '4':1350}
        self.params = dict(PWM_MAIN_FUNC1=0, PWM_MAIN_FUNC6=301, PWM_MAIN_FUNC7=302,
            PWM_MAIN_MIN6=900, PWM_MAIN_MAX6=2000, PWM_MAIN_MIN7=900, PWM_MAIN_MAX7=2000,
            PWM_MAIN_REV=0, COM_MOT_TEST_EN=0)

    def assert_wire(self, command, output, pwm, params=None):
        params = params or self.params
        channel = 1 if output == 6 else 2
        self.assertEqual(command['main_output'], output)
        self.assertEqual(command['command'], 187)
        finite = [i for i, value in enumerate(command['params'], 1) if math.isfinite(value)]
        self.assertEqual(finite, [channel])
        value = command['params'][channel-1]
        if params['PWM_MAIN_REV'] & (1 << (output-1)):
            value *= -1
        low, high = params['PWM_MAIN_MIN'+str(output)], params['PWM_MAIN_MAX'+str(output)]
        self.assertAlmostEqual(low+(value+1)*(high-low)/2, pwm)

    def test_four_inventory_slots_alternate_two_servos_without_reset(self):
        before = deepcopy(self.payload)
        expected = [(6,1190),(7,1190),(6,1350),(7,1350)]
        self.assertEqual(self.payload['sequence'], [1,2,3,4])
        for slot, (output, pwm) in zip(self.payload['sequence'], expected):
            self.assert_wire(actuator_command(slot, 'release', self.payload), output, pwm)
        self.assertEqual(before, self.payload)

    def test_current_field_release_values_are_preserved_in_encoded_commands(self):
        configured = apply_flight_configuration(base_configuration(), task='delivery-route')['payload']
        before = deepcopy(configured)
        for slot in (1, 2, 3, 4):
            self.assert_wire(actuator_command(slot, 'release', configured),
                             int(configured['main_outputs'][str(slot)]),
                             configured['released_pwm_us_by_slot'][str(slot)])
        self.assertEqual(configured, before)

    def test_live_physical_range_and_reverse_use_main6_and_main7(self):
        params = dict(self.params, PWM_MAIN_MIN6=800, PWM_MAIN_MAX7=1950,
                      PWM_MAIN_REV=(1<<5))
        for slot, output, pwm in ((1,6,1190),(2,7,1190),(3,6,1350),(4,7,1350)):
            self.assert_wire(actuator_command(slot,'release',self.payload,params),output,pwm,params)

    def test_reset_and_lock_use_closed_range_once_per_physical_servo(self):
        self.assertEqual(reset_slots(self.payload), (1,2))
        sent=[]
        def send(slot, action):
            sent.append(actuator_request(slot,action,self.payload,parameters=self.params))
            return True
        self.assertEqual(run_reset(send, emit=lambda _:None, slots=reset_slots(self.payload)),0)
        self.assertEqual([(x['main_output'],x['configured_pwm_us']) for x in sent],[(6,900),(7,900)])
        for slot in range(1,5):
            self.assert_wire(actuator_command(slot,'lock',self.payload),6 if slot in (1,3) else 7,900)
            self.assertTrue(800 <= actuator_request(slot,'center',self.payload)['configured_pwm_us'] <= 980)

    def test_shared_inventory_mapping_is_valid_but_shared_physical_function_is_not(self):
        rows, errors = inspect_mapping(self.params,self.payload)
        self.assertEqual(errors,[])
        self.assertEqual([row['output'] for row in rows],['MAIN6','MAIN7'])
        for update in ({'PWM_MAIN_FUNC1':301},{'PWM_AUX_FUNC2':302},{'PWM_MAIN_FUNC6':0}):
            transport=mock.Mock()
            with self.assertRaises(ValueError):
                send_once(actuator_request(3,'release',self.payload),
                    SimpleNamespace(connected=True,armed=False),dict(self.params,**update),self.payload,transport)
            transport.assert_not_called()

    def test_2000_ceiling_survives_wider_fcu_range_and_config_overrides(self):
        params=dict(self.params,PWM_MAIN_MAX6=2500)
        for value in (2000.1,2200,float('inf'),float('nan')):
            payload=dict(self.payload,pwm_max_us=2500,pwm_command_max_us=3000,
                released_pwm_us_by_slot={'1':value})
            with self.assertRaises(ValueError):
                actuator_command(1,'release',payload,params)
        payload=dict(self.payload,released_pwm_us_by_slot={'1':2000})
        self.assert_wire(actuator_command(1,'release',payload,params),6,2000,params)
        cli=cli_helpers.Main1PWMTests().load_cli()
        with self.assertRaises(ValueError):
            main_pwm_request(6,2001,cli.port_payload(self.payload,6),params)

    def test_real_manager_sends_exactly_four_commands_with_cached_physical_calibration(self):
        from ros_fixtures import STUBS,ROS,make_manager
        params=dict(self.params,PWM_MAIN_MIN6=800,PWM_MAIN_REV=1<<6)
        with mock.patch.dict(sys.modules,STUBS):
            manager=make_manager()
            manager.config['payload']=deepcopy(self.payload)
            manager.engine.native_active=True
            manager.engine.on_payload_result=mock.Mock()
            requests=[]
            def transport(req):
                requests.append(req)
                return SimpleNamespace(success=True,result=0)
            with mock.patch.object(ROS,'get_param',return_value=params), \
                 mock.patch.object(ROS,'wait_for_service'), \
                 mock.patch.object(ROS,'ServiceProxy',return_value=transport):
                for slot in (1,2,3,4):
                    done=threading.Event()
                    manager.send_payload(slot,'release',slot,for_mission=True,done=lambda _:done.set())
                    self.assertTrue(done.wait(1))
            self.assertEqual(len(requests),4)
            for req,(output,pwm) in zip(requests,[(6,1190),(7,1190),(6,1350),(7,1350)]):
                command=dict(main_output=output,command=req.command,
                             params=[getattr(req,'param'+str(i)) for i in range(1,7)])
                self.assert_wire(command,output,pwm,params)
                self.assertEqual(req.param7,0.)
            self.assertEqual(manager.engine.on_payload_result.call_count,4)

    def test_complete_route_replay_emits_only_four_alternating_release_steps(self):
        cfg=configuration()
        cfg['payload']=deepcopy(self.payload)
        replay=Replay(cfg)
        replay.until(lambda e:e.drops_completed==4)
        slots=[effect['slot'] for effect in replay.effects if effect['type']=='release']
        self.assertEqual(slots,[1,2,3,4])
        self.assertEqual(replay.engine.route_released_slots,[1,2,3,4])
        self.assertEqual([(actuator_command(s,'release',self.payload)['main_output'],
                           actuator_command(s,'release',self.payload)['pwm_us']) for s in slots],
                         [(6,1190),(7,1190),(6,1350),(7,1350)])

    def test_rejected_ack_does_not_consume_or_send_next_stage(self):
        cfg=configuration();cfg['payload']=deepcopy(self.payload)
        replay=Replay(cfg)
        replay.until(lambda e:e.phase=='WAIT_ACK',acknowledge=False)
        e=replay.engine
        e.on_payload_result(False,e._payload_request,outcome='rejected')
        replay.step(acknowledge=False)
        self.assertEqual(e.state,'HOLD')
        self.assertEqual(e.drops_completed,0)
        self.assertEqual(e.route_released_slots,[])
        self.assertEqual([x['slot'] for x in replay.effects if x['type']=='release'],[1])
        e.start('delivery_route',replay.ground,replay.yaw,replay.now+1)
        self.assertEqual(e.route_released_slots,[])
        self.assertEqual(e.drops_completed,0)


if __name__ == '__main__':
    unittest.main()
