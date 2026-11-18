"""Per-physical-slot release PWM across mission and manual entry points."""
from copy import deepcopy
from pathlib import Path
import json
import math
import os
import subprocess
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

PROJECT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT/'src/airseafly_real400_mission/src'))
from airseafly_real400_mission.configuration import load_config
from airseafly_real400_mission.flight_configuration import TASKS, apply_flight_configuration, load_observer_configuration
from airseafly_real400_mission.main_servo_test import actuator_request, send_once
from airseafly_real400_mission.payload import actuator_command
from airseafly_real400_mission.payload_console import run_console, run_hover_drop_console

LEGACY = dict(actuator_indices={'1': 1, '2': 2, '3': 3}, pwm_min_us=1100,
              pwm_center_us=1500, pwm_max_us=2200, locked_pwm_us=1260,
              released_pwm_us=2200)


class SlotPWMTests(unittest.TestCase):
    def setUp(self):
        self.payload = dict(deepcopy(LEGACY), released_pwm_us_by_slot={'2': 2020})

    def assert_release(self, payload):
        for slot, pwm in ((1, 2200), (2, 2020), (3, 2200)):
            command = actuator_command(slot, 'release', payload)
            self.assertEqual(command['pwm_us'], pwm)
            maximum = payload.get('pwm_max_us_by_slot', {}).get(str(slot), payload['pwm_max_us'])
            self.assertAlmostEqual(1100+(command['normalized']+1)*(maximum-1100)/2., pwm)
            channel = command['channel']
            self.assertEqual(command['params'][channel-1], command['normalized'])
            self.assertTrue(all(math.isnan(v) for i, v in enumerate(command['params'], 1) if i != channel))

    def test_release_uses_actual_pwm_and_keeps_config_and_other_slots(self):
        before = deepcopy(self.payload)
        self.assert_release(self.payload)
        self.assertAlmostEqual(actuator_command(2, 'release', self.payload)['normalized'], .6727272727272727)
        self.assertEqual(self.payload, before)

    def test_override_tracks_physical_slot_after_channel_remap(self):
        self.payload['actuator_indices']['2'] = 5
        self.assert_release(self.payload)
        self.assertAlmostEqual(actuator_command(2, 'release', self.payload)['params'][4], .6727272727272727)

    def test_unquoted_numeric_keys_and_legacy_configs(self):
        self.payload['released_pwm_us_by_slot'] = {2: 2020}
        self.assert_release(self.payload)
        for slot in (1, 2, 3):
            self.assertEqual(actuator_command(slot, 'release', LEGACY)['normalized'], 1.)

    def test_second_port_2020_range_reaches_release_without_changing_other_commands(self):
        original = deepcopy(self.payload)
        old = actuator_command(2, 'release', original)
        self.assertAlmostEqual(1100+(old['normalized']+1)*460, 1869.4545454545455)
        self.payload['pwm_max_us_by_slot'] = {'2': 2020}
        saved = deepcopy(self.payload)
        self.assert_release(self.payload)
        self.assertEqual(actuator_command(2, 'release', self.payload)['normalized'], 1.)
        for slot in (1, 2, 3):
            self.assertEqual(actuator_command(slot, 'lock', self.payload)['params'][slot-1],
                             actuator_command(slot, 'lock', original)['params'][slot-1])
        for slot in (1, 3):
            self.assertEqual(actuator_command(slot, 'release', self.payload)['normalized'], 1.)
        self.assertEqual(self.payload, saved)
        self.payload['pwm_max_us_by_slot'] = {2: 2020}
        self.payload['actuator_indices']['2'] = 5
        self.assertEqual(actuator_command(2, 'release', self.payload)['params'][4], 1.)

    def test_bench_preview_uses_saved_limit_and_live_parameters_take_precedence(self):
        self.payload['pwm_max_us_by_slot'] = {'2': 2020}
        for maximum, parameters in ((2020, None), (2020, {'PWM_MAIN_MAX2': 2020}),
                                    (2200, {'PWM_MAIN_MAX2': 2200})):
            for action, pwm in (('release', 2020), ('center', 1500)):
                with self.subTest(maximum=maximum, action=action, parameters=parameters):
                    request = actuator_request(2, action, self.payload, parameters=parameters)
                    self.assertAlmostEqual(1100+(request['normalized']+1)*(maximum-1100)/2., pwm)
                    self.assertEqual(request['command'], 187 if action == 'center' else 310)

    def test_closed_and_persistent_center_outputs_stay_unchanged(self):
        for slot in (1, 2, 3):
            self.assertEqual(actuator_command(slot, 'lock', self.payload)['normalized'], -.6)
            center = actuator_request(slot, 'center', self.payload)
            self.assertEqual(center['command'], 187)
            self.assertTrue(center['persistent'])
            self.assertAlmostEqual(1100+(center['normalized']+1)*550, 1500)

    def test_existing_pwm_validation_also_applies_to_slot_override(self):
        for value in (1000, 2300, float('nan'), float('inf')):
            self.payload['released_pwm_us_by_slot']['2'] = value
            with self.assertRaises(ValueError):
                actuator_command(2, 'release', self.payload)


    def test_independent_menu_reports_each_slot_and_keeps_explicit_order(self):
        send = mock.Mock(return_value=dict(ok=True, result=dict(success=True)))
        lines = []
        run_console(dict(payload=self.payload), send,
                    mock.Mock(side_effect=['2','1','1','1','3','1','q']), lines.append)
        self.assertEqual(send.call_args_list, [mock.call(slot, 'release') for slot in (2, 1, 3)])
        text = '\n'.join(lines)
        self.assertIn('释放：1号 2200 μs、2号 2020 μs、3号 2200 μs', text)
        self.assertIn('投放编号 2 → MAIN2 释放；配置通道 2，PWM 2020', text)

    def test_menu_summary_does_not_add_validation_of_unused_channels(self):
        self.payload['actuator_indices'].pop('3')
        send = mock.Mock()
        self.assertEqual(run_console(dict(payload=self.payload), send,
                                     mock.Mock(return_value='q'), lambda _: None), 0)
        send.assert_not_called()

    def test_hover_menu_uses_slot_value_and_repeat_is_one_new_command(self):
        send = mock.Mock(return_value=dict(ok=True, result=dict(success=True)))
        land, lines = mock.Mock(return_value=0), []
        run_hover_drop_console(dict(payload=self.payload), send, land,
            mock.Mock(side_effect=['2','1','3','2','q']), lines.append)
        self.assertEqual(send.call_args_list, [mock.call(slot, 'release') for slot in (2, 1, 3, 2)])
        land.assert_called_once_with()
        self.assertEqual('\n'.join(lines).count('投放编号 2 → MAIN2 释放；配置通道 2，PWM 2020'), 2)

    def test_bench_transport_uses_same_2020_with_one_request(self):
        params = {'COM_MOT_TEST_EN': 1}
        for slot in (1, 2, 3):
            params.update({'PWM_MAIN_FUNC'+str(slot): 300+slot,
                           'PWM_MAIN_MIN'+str(slot): 1100, 'PWM_MAIN_MAX'+str(slot): 2200})
        transport = mock.Mock(return_value=SimpleNamespace(success=True, result=0))
        command = actuator_request(2, 'release', self.payload)
        send_once(command, SimpleNamespace(connected=True, armed=False), params, self.payload, transport)
        transport.assert_called_once()
        fields = transport.call_args.kwargs
        self.assertEqual(fields['command'], 310)
        self.assertEqual(fields['param5'], 1302.)
        self.assertEqual(fields['param2'], 2.)
        self.assertAlmostEqual(1100+(fields['param1']+1)*550, 2020)



    def test_auto_and_manual_manager_dispatch_real_requests_with_new_value(self):
        from ros_fixtures import STUBS, ROS, make_manager
        for for_mission, native, maximum in ((mission, native, maximum)
                for mission, native in ((False, False), (True, False), (True, True))
                for maximum in (2200, 2020)):
            self.payload['pwm_max_us_by_slot'] = {'2': maximum}
            with self.subTest(for_mission=for_mission, native=native, maximum=maximum), mock.patch.dict(sys.modules, STUBS):
                manager = make_manager()
                manager.config['payload'].update(deepcopy(self.payload))
                manager.config['payload']['pd_profiles_enabled'] = False
                manager.engine.native_active = native
                manager.engine.on_payload_result = mock.Mock()
                service = mock.Mock(return_value=SimpleNamespace(success=True, result=0))
                with mock.patch.object(ROS, 'wait_for_service'), \
                     mock.patch.object(ROS, 'ServiceProxy', return_value=service):
                    for slot in (2, 1, 3):
                        done = threading.Event()
                        manager.send_payload(slot, 'release', slot, for_mission=for_mission,
                                             done=lambda _: done.set())
                        self.assertTrue(done.wait(1.))
                self.assertEqual(service.call_count, 3)
                for slot, call in zip((2, 1, 3), service.call_args_list):
                    request = call.args[0]
                    self.assertEqual(request.command, 187)
                    self.assertEqual(request.param7, 0.)
                    expected = 2020 if slot == 2 else 2200
                    high = maximum if slot == 2 else 2200
                    self.assertAlmostEqual(1100+(getattr(request, 'param'+str(slot))+1)*(high-1100)/2., expected)
                    self.assertTrue(all(math.isnan(getattr(request, 'param'+str(i)))
                                        for i in range(1, 7) if i != slot))
                requests = [event['command'] for kind, event in manager.events_seen if kind == 'payload_request']
                self.assertEqual([event['pwm_us'] for event in requests], [2020, 2200, 2200])


if __name__ == '__main__':
    unittest.main()
