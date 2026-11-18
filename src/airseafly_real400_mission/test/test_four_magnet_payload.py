"""Four independent magnet outputs retain the mission's existing 187 semantics."""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest import mock
import yaml

from airseafly_real400_mission.delivery_tuning import delivery_tuning_summary
from airseafly_real400_mission.main_servo_test import actuator_request, inspect_mapping, run_reset, send_once
from airseafly_real400_mission.payload import actuator_command, configured_slots
from airseafly_real400_mission.payload_console import run_console, run_hover_drop_console

PROJECT = Path(__file__).resolve().parents[3]


class FourMagnetPayloadTests(unittest.TestCase):
    def setUp(self):
        self.payload = dict(actuator_indices={str(i): i for i in range(1, 5)},
                            pwm_min_us=1100, pwm_center_us=1500, pwm_max_us=2200,
                            locked_pwm_us=1100, released_pwm_us=2200)
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.flight_config = Path(folder.name)/'flight.yaml'
        document = yaml.safe_load((PROJECT/'flight/flight.yaml').read_text())
        task = document['mission_profiles']['delivery-route']
        task['payload']['released_pwm_us_by_slot'] = {'1':1190, '2':1190, '3':1350, '4':1350}
        task['delivery_route_options']['mount_offsets_body_m'] = {str(i):[0.,0.] for i in range(1,5)}
        self.flight_config.write_text(json.dumps(document))

    def test_four_outputs_work_with_either_calibrated_polarity(self):
        for locked, released, expected_lock in ((1100, 2200, -1.), (2200, 1100, 1.)):
            payload = dict(self.payload, locked_pwm_us=locked, released_pwm_us=released)
            for slot in configured_slots(payload):
                for action, value in (('lock', expected_lock), ('release', -expected_lock)):
                    command = actuator_command(slot, action, payload)
                    self.assertEqual(command['command'], 187)
                    self.assertEqual(command['params'][slot-1], value)
                    self.assertEqual([i+1 for i, v in enumerate(command['params']) if math.isfinite(v)], [slot])

    def test_fourth_physical_output_uses_its_configured_function(self):
        payload = deepcopy(self.payload)
        payload['actuator_indices']['4'] = 6
        command = actuator_command(4, 'release', payload)
        self.assertEqual(command['channel'], 6)
        self.assertEqual(command['params'][5], 1.)
        self.assertTrue(all(math.isnan(v) for v in command['params'][:5]))

    def test_legacy_slot_two_override_is_explicitly_removed_for_shared_low_release(self):
        payload = dict(self.payload, locked_pwm_us=2200, released_pwm_us=1100,
                       pwm_max_us_by_slot={'2': 2020}, released_pwm_us_by_slot={'2': 2020})
        self.assertEqual(actuator_command(2, 'release', payload)['normalized'], 1.)
        payload['released_pwm_us_by_slot'] = {}
        for slot in configured_slots(payload):
            self.assertEqual(actuator_command(slot, 'release', payload)['normalized'], -1.)
        # The payload converter still preserves the old slot-specific behavior.
        self.assertEqual(actuator_command(2, 'lock', dict(payload, locked_pwm_us=1260))['normalized'], -.6)

    def test_old_configuration_keeps_three_ports_and_new_one_can_reset_four(self):
        old = deepcopy(self.payload)
        del old['actuator_indices']['4']
        self.assertEqual(configured_slots(old), (1, 2, 3))
        with self.assertRaises(ValueError):
            actuator_command(4, 'release', old)
        send = mock.Mock(return_value=True)
        self.assertEqual(run_reset(send, emit=lambda _: None, slots=configured_slots(self.payload)), 0)
        self.assertEqual(send.call_args_list, [mock.call(slot, 'center') for slot in range(1, 5)])

    def test_fourth_port_mapping_and_timed_bench_test_remain_distinct_from_mission(self):
        parameters = {'COM_MOT_TEST_EN': 1}
        for slot in configured_slots(self.payload):
            parameters.update({'PWM_MAIN_FUNC'+str(slot): 300+slot,
                               'PWM_MAIN_MIN'+str(slot): 1100,
                               'PWM_MAIN_MAX'+str(slot): 2200})
        rows, errors = inspect_mapping(parameters, self.payload)
        self.assertFalse(errors)
        self.assertEqual([row['output'] for row in rows], ['MAIN1', 'MAIN2', 'MAIN3', 'MAIN4'])
        transport = mock.Mock()
        command = actuator_request(4, 'lock', self.payload)
        send_once(command, SimpleNamespace(connected=True, armed=False), parameters, self.payload, transport)
        self.assertEqual(transport.call_args.kwargs['command'], 310)
        self.assertEqual(transport.call_args.kwargs['param5'], 1304.)
        self.assertEqual(actuator_command(4, 'lock', self.payload)['command'], 187)

    def test_persistent_payload_console_sends_fourth_port_once_per_explicit_action(self):
        send = mock.Mock(return_value=dict(ok=True, result=dict(success=True)))
        run_console({'payload': self.payload}, send,
                    read=mock.Mock(side_effect=['4', 'lock', '4', 'release', 'q']), emit=lambda _: None)
        self.assertEqual(send.call_args_list, [mock.call(4, 'lock'), mock.call(4, 'release')])

    def test_hover_console_accepts_configured_fourth_port(self):
        send = mock.Mock(return_value=dict(ok=True, result=dict(success=True)))
        land = mock.Mock(return_value=0)
        self.assertEqual(run_hover_drop_console({'payload': self.payload}, send, land,
            read=mock.Mock(side_effect=['4', 'q']), emit=lambda _: None), 0)
        send.assert_called_once_with(4, 'release')
        land.assert_called_once_with()

    def test_existing_manager_dispatches_fourth_magnet_through_persistent_187(self):
        from ros_fixtures import STUBS, ROS, make_manager
        with mock.patch.dict(sys.modules, STUBS):
            manager = make_manager()
            manager.config['payload'].update(deepcopy(self.payload))
            manager.config['payload']['pd_profiles_enabled'] = False
            manager.engine.native_active = True
            manager.engine.on_payload_result = mock.Mock()
            service = mock.Mock(return_value=SimpleNamespace(success=True, result=0))
            with mock.patch.object(ROS, 'wait_for_service'), \
                 mock.patch.object(ROS, 'ServiceProxy', return_value=service):
                for action in ('lock', 'release'):
                    done = threading.Event()
                    manager.send_payload(4, action, action, for_mission=action == 'release',
                                         done=lambda _: done.set())
                    self.assertTrue(done.wait(1.))
            self.assertEqual(service.call_count, 2)
            for call, expected in zip(service.call_args_list, (-1., 1.)):
                request = call.args[0]
                self.assertEqual(request.command, 187)
                self.assertEqual(request.param7, 0.)
                self.assertEqual(request.param4, expected)
                self.assertTrue(all(math.isnan(getattr(request, 'param'+str(i)))
                                    for i in (1, 2, 3, 5, 6)))
            manager.engine.on_payload_result.assert_called_once()

    def test_route_keeps_four_mount_offsets_without_enabling_extra_dwell(self):
        from airseafly_real400_mission.flight_configuration import base_configuration, apply_flight_configuration
        config = apply_flight_configuration(base_configuration(), self.flight_config, task='delivery-route')
        self.assertEqual(config['delivery_route_options']['mount_offsets_body_m']['4'], [0., 0.])
        self.assertFalse(delivery_tuning_summary(config)['release_stability_enabled'])

    def test_delivery_observer_and_task_clear_old_servo_overrides_for_all_four_outputs(self):
        from airseafly_real400_mission.flight_configuration import (
            apply_flight_configuration, base_configuration, load_observer_configuration)
        path = self.flight_config
        machine = base_configuration(project=PROJECT)
        machine['payload']['pd_profiles_enabled'] = True
        configs = []
        configs += [apply_flight_configuration(machine, path, task=task, project=PROJECT)
                    for task in ('delivery-route',)]
        for config in configs:
            payload = config['payload']
            self.assertFalse(payload['pd_profiles_enabled'])
            self.assertEqual(payload['released_pwm_us_by_slot'], {'1':1190,'2':1190,'3':1350,'4':1350})
            self.assertEqual(payload['pwm_max_us_by_slot'], {})
            self.assertEqual(configured_slots(payload), (1, 2, 3, 4))
            for slot in configured_slots(payload):
                self.assertEqual(actuator_command(slot, 'lock', payload)['pwm_us'], 900)
                self.assertEqual(actuator_command(slot, 'release', payload)['pwm_us'], 1190 if slot <= 2 else 1350)

    def test_magnet_entry_dry_run_builds_persistent_commands_for_each_output(self):
        env = dict(os.environ, AIRSEAFLY_PYTHON=sys.executable)
        for slot in (1, 2, 3, 4):
            for action, pwm in (('lock', 900), ('release', 1190 if slot <= 2 else 1350)):
                with self.subTest(slot=slot, action=action):
                    result = subprocess.run(['sh', str(PROJECT/'flight/magnet.sh'), action,
                        str(slot), '--dry-run', '--flight-config', str(self.flight_config)],
                        env=env, capture_output=True, text=True, timeout=15)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    preview = json.loads(result.stdout)
                    self.assertTrue(preview['dry_run'])
                    command = preview['payload']
                    self.assertEqual(command['command'], 187)
                    self.assertEqual(command['slot'], slot)
                    channel = 1 if slot in (1, 3) else 2
                    self.assertEqual(command['channel'], channel)
                    self.assertEqual(command['main_output'], channel+5)
                    self.assertAlmostEqual(900+(command['params'][channel-1]+1)*550, pwm)
                    self.assertTrue(all(v is None for i, v in enumerate(command['params']) if i != channel-1))


if __name__ == '__main__':
    unittest.main()
