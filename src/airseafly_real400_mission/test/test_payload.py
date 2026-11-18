import math
import unittest
import copy
import importlib.util
import io
import tempfile
import socket
from contextlib import redirect_stdout, redirect_stderr
from pathlib import Path
from unittest import mock
from airseafly_real400_mission.payload import actuator_command, normalize_pwm
from airseafly_real400_mission.payload_console import run_console
from airseafly_real400_mission.rpc import CommandNotSent

CONFIG = dict(pwm_min_us=1100, pwm_center_us=1500, pwm_max_us=2200,
              locked_pwm_us=1260, released_pwm_us=2200,
              actuator_indices={'1': 1, '2': 2, '3': 3})


class PayloadTest(unittest.TestCase):
    def test_historical_pwm_piecewise_mapping(self):
        self.assertAlmostEqual(normalize_pwm(1260, CONFIG), -.6)
        self.assertEqual(normalize_pwm(2200, CONFIG), 1.)
        self.assertEqual(normalize_pwm(1500, CONFIG), 0.)

    def test_physical_2_1_3_and_untouched_channels(self):
        for slot in (2, 1, 3):
            command = actuator_command(slot, 'release', CONFIG)
            self.assertEqual(command['params'][slot - 1], 1.)
            self.assertTrue(all(math.isnan(x) for i, x in enumerate(command['params']) if i != slot - 1))

    def test_invalid_input_does_not_construct_actuator_command(self):
        for slot in (0, 4):
            with self.assertRaises(ValueError):
                actuator_command(slot, 'release', CONFIG)
        with self.assertRaises(ValueError):
            normalize_pwm(2300, CONFIG)


class PayloadConsoleTest(unittest.TestCase):
    def exercise(self, choices, reply=None, error=None, config=None):
        inputs=iter(choices);calls=[];lines=[]
        def read(_):
            value=next(inputs)
            if isinstance(value,BaseException):raise value
            return value
        def send(slot,action):
            calls.append((slot,action))
            if error:raise error
            command=actuator_command(slot,action,config or CONFIG)
            return reply if reply is not None else dict(ok=True,result=dict(success=True,**command))
        self.assertEqual(run_console({'payload':config or CONFIG},send,read,lines.append),0)
        return calls,'\n'.join(lines)

    def test_each_slot_release_and_close_once_in_menu_order(self):
        calls,lines=self.exercise(['1','1','1','2','2','1','2','2','3','1','3','2','q'])
        self.assertEqual(calls,[(s,a) for s in (1,2,3) for a in ('release','lock')])
        self.assertEqual(lines.count('回执：飞控已接受'),6)
        self.assertIn('PWM 1260 μs',lines);self.assertIn('PWM 2200 μs',lines)

    def test_invalid_inputs_back_and_quit_never_send(self):
        calls,_=self.exercise(['','4','1','nonsense','b','3','q'])
        self.assertEqual(calls,[])

    def test_missing_receipt_has_no_automatic_retry(self):
        calls,lines=self.exercise(['2','1','q'],error=TimeoutError('lost receipt'))
        self.assertEqual(calls,[(2,'release')])
        self.assertIn('未自动重发',lines)
        self.assertNotIn('回执：飞控已接受',lines)

    def test_startup_failure_is_explicitly_not_a_sent_payload(self):
        calls,lines=self.exercise(['2','1','q'],error=CommandNotSent('ROS主节点未连接'))
        self.assertEqual(calls,[(2,'release')])
        self.assertIn('未发送舵机命令',lines)
        self.assertNotIn('结果未确认',lines)

    def test_rejected_action_returns_to_menu_without_a_second_command(self):
        for reply in (dict(ok=False,error='当前任务仍在执行'),
                      dict(ok=False,result=dict(success=False,message='飞控拒绝命令'))):
            calls,lines=self.exercise(['3','2','q'],reply=reply)
            self.assertEqual(calls,[(3,'lock')]);self.assertIn('失败或未确认',lines)

    def test_interrupt_does_not_close_all_slots_or_send_hold(self):
        for interrupt in (EOFError(),KeyboardInterrupt()):
            calls,_=self.exercise([interrupt]);self.assertEqual(calls,[])
        calls,lines=self.exercise(['1','1'],error=KeyboardInterrupt())
        self.assertEqual(calls,[(1,'release')]);self.assertIn('回执未确认',lines)

    def test_configured_channel_and_pwm_are_reported(self):
        changed=copy.deepcopy(CONFIG)
        changed['actuator_indices']['3']=5
        changed['locked_pwm_us']=1300
        calls,lines=self.exercise(['3','闭合','q'],config=changed)
        self.assertEqual(calls,[(3,'lock')])
        self.assertIn('通道 5，PWM 1300 μs',lines)
        self.assertIn('归一化值 -0.500',lines)


    def test_incomplete_cli_action_does_not_default_to_a_slot(self):
        spec=importlib.util.spec_from_file_location('payload_cli_test_invalid',Path(__file__).resolve().parents[1]/'scripts/mission_cli.py')
        cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
        with mock.patch('sys.argv',['mission_cli.py','payload','release']), \
             mock.patch.object(cli,'connect_or_start') as connect,redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:cli.main()
        self.assertEqual(error.exception.code,2);connect.assert_not_called()


class ManagerStartupTest(unittest.TestCase):
    def setUp(self):
        spec=importlib.util.spec_from_file_location('startup_cli_test',Path(__file__).resolve().parents[1]/'scripts/mission_cli.py')
        self.cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.cli)
        self.directory=tempfile.TemporaryDirectory()
        self.root=Path(self.directory.name)
        self.paths=dict(socket=str(self.root/'rpc.sock'),manager_lock=str(self.root/'manager.lock'))
        self.config=dict(runtime=dict(ros_master_uri='http://127.0.0.1:11341'))
        self.root_patch=mock.patch.object(self.cli,'project_root',return_value=self.root)
        self.root_patch.start()
        self.connection_patch=mock.patch.object(self.cli,'manager_registered',return_value=True)
        self.connection_patch.start()

    def tearDown(self):
        self.connection_patch.stop();self.root_patch.stop();self.directory.cleanup()

    def test_existing_manager_is_reused_without_launch(self):
        with mock.patch.object(self.cli,'request',return_value=dict(ok=True,pid=123)), \
             mock.patch.object(self.cli,'require_ros_master') as master, \
             mock.patch.object(self.cli.subprocess,'Popen') as spawn:
            self.assertEqual(self.cli.connect_or_start(self.paths,self.config)['pid'],123)
        spawn.assert_not_called();master.assert_called_once_with(self.config['runtime']['ros_master_uri'])

    def test_missing_master_repeated_attempts_never_spawn_or_take_locks(self):
        # Bound but not listening: deterministic refusal, no real ROS endpoint.
        with socket.socket() as closed:
            closed.bind(('127.0.0.1',0))
            self.config['runtime']['ros_master_uri']='http://127.0.0.1:%s'%closed.getsockname()[1]
            with mock.patch.object(self.cli,'request',side_effect=ConnectionRefusedError()), \
                 mock.patch.object(self.cli.subprocess,'Popen') as spawn, \
                 mock.patch.object(self.cli,'ProcessLock') as lock:
                for _ in range(3):
                    with self.assertRaisesRegex(CommandNotSent,'请先启动MAVROS'):
                        self.cli.connect_or_start(self.paths,self.config)
            spawn.assert_not_called();lock.assert_not_called()

    def test_manager_already_starting_does_not_spawn_duplicate(self):
        with mock.patch.object(self.cli,'request',side_effect=[ConnectionRefusedError(),dict(ok=True)]), \
             mock.patch.object(self.cli,'require_ros_master'), \
             mock.patch.object(self.cli,'manager_lock_held',return_value=True), \
             mock.patch.object(self.cli.subprocess,'Popen') as spawn:
            self.assertTrue(self.cli.connect_or_start(self.paths,self.config)['ok'])
        spawn.assert_not_called()

    def test_another_terminal_owns_startup_wait_for_its_rpc(self):
        with mock.patch.object(self.cli,'request',side_effect=[ConnectionRefusedError(),dict(ok=True)]), \
             mock.patch.object(self.cli,'require_ros_master'), \
             mock.patch.object(self.cli,'ProcessLock',side_effect=self.cli.ResourceBusy('starting')), \
             mock.patch.object(self.cli.subprocess,'Popen') as spawn:
            self.assertTrue(self.cli.connect_or_start(self.paths,self.config)['ok'])
        spawn.assert_not_called()

    def test_timeout_reaps_only_owned_child_before_rpc_is_published(self):
        for published in (False,True):
            process=mock.Mock();process.poll.return_value=None
            with self.subTest(published=published), \
                 mock.patch.object(self.cli,'request',side_effect=ConnectionRefusedError()), \
                 mock.patch.object(self.cli,'require_ros_master'), \
                 mock.patch.object(self.cli,'manager_lock_held',return_value=False), \
                 mock.patch.object(self.cli.subprocess,'Popen',return_value=process), \
                 mock.patch.object(self.cli.time,'monotonic',side_effect=[0.,13.]), \
                 mock.patch.object(self.cli,'socket_identity',side_effect=[None,(1,2,3) if published else None]):
                with self.assertRaises(CommandNotSent):self.cli.connect_or_start(self.paths,self.config)
            if published:process.terminate.assert_not_called()
            else:process.terminate.assert_called_once();process.wait.assert_called_once()

    def test_successful_new_manager_is_preserved(self):
        process=mock.Mock();process.poll.return_value=None
        with mock.patch.object(self.cli,'request',side_effect=[ConnectionRefusedError(),dict(ok=True)]), \
             mock.patch.object(self.cli,'require_ros_master'), \
             mock.patch.object(self.cli,'manager_lock_held',return_value=False), \
             mock.patch.object(self.cli.subprocess,'Popen',return_value=process) as spawn:
            self.assertTrue(self.cli.connect_or_start(self.paths,self.config)['ok'])
        spawn.assert_called_once();process.terminate.assert_not_called()


if __name__ == '__main__':
    unittest.main()
