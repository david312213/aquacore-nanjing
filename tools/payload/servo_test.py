#!/usr/bin/env python3
"""Configured MAIN-output bench menu; only explicit actions send commands."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xmlrpc.client

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT/'src/airseafly_real400_mission/src'))
from airseafly_real400_mission.flight_configuration import base_configuration, apply_flight_configuration
from airseafly_real400_mission.main_servo_test import inspect_mapping, run_menu, run_reset, send_once, actuator_request
from airseafly_real400_mission.payload import configured_slots, reset_slots


class ShortTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2
        return connection


def has_mavros(master):
    try:
        with xmlrpc.client.ServerProxy(master, transport=ShortTransport()) as server:
            return server.lookupService('/main_servo_test', '/mavros/cmd/command')[0] == 1
    except (OSError, xmlrpc.client.Error):
        return False


class Connection:
    def __init__(self, master, fcu_url, log_dir):
        self.master, self.fcu_url, self.log_dir = master, fcu_url, log_dir
        self.process, self.log = None, None

    def __enter__(self):
        if has_mavros(self.master):
            print('使用已运行的MAVROS。')
            return self
        url = self.fcu_url
        if not url:
            devices = sorted(Path('/dev/serial/by-id').glob('*PX4*'))
            if len(devices) != 1:
                raise ValueError('没有唯一的PX4 USB设备；可用 --fcu-url 显式指定')
            url = 'serial://'+str(devices[0])+':921600'
        if url.startswith('serial://'):
            device = url[len('serial://'):].rsplit(':', 1)[0]
            busy = subprocess.run(['fuser', device], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if busy.returncode == 0:
                raise ValueError('飞控串口正在使用；请退出QGC或使用已运行的MAVROS')
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log = (self.log_dir/'mavros.log').open('w')
        self.process = subprocess.Popen(['roslaunch', '-p', '11341', 'mavros', 'px4.launch',
            'fcu_url:='+url, 'gcs_url:='], stdin=subprocess.DEVNULL, stdout=self.log,
            stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic()+20
            while time.monotonic() < deadline:
                if has_mavros(self.master):
                    print('已启动仅用于舵机测试的MAVROS，退出时自动关闭。')
                    return self
                if self.process.poll() is not None:
                    break
                time.sleep(.2)
            raise RuntimeError('MAVROS启动未完成；查看 '+str(self.log_dir/'mavros.log'))
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *_args):
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait()
        if self.log:
            self.log.close()


def parser():
    result = argparse.ArgumentParser(description='地面投放编号1/2/3/4测试；实际MAIN输出按配置，不起飞、不解锁')
    result.add_argument('action', nargs='?', choices=('open', 'close', 'release', 'lock', 'center', 'reset'),
                        help='open 打开 / close 闭合（兼容 release/lock）；center 指定编号复位并保持；reset 每只舵机复位并保持')
    result.add_argument('slot', nargs='?', type=int, choices=(1, 2, 3, 4))
    result.add_argument('--check', action='store_true', help='只读取MAIN映射和电机解锁状态，不动作')
    result.add_argument('--dry-run', action='store_true', help='离线显示配置及命令，不连接飞机')
    result.add_argument('--seconds', type=float, default=2.,
                        help='仅旧版定时测试使用，默认2秒，最大3秒；当前双舵机配置持续保持，不使用此参数')
    result.add_argument('--config', help='原机器配置路径')
    result.add_argument('--flight-config', default=str(PROJECT/'flight/flight.yaml'))
    result.add_argument('--fcu-url', help='无MAVROS时使用的飞控连接；默认自动寻找PX4 USB')
    return result


def main(argv=None):
    cli = parser()
    args = cli.parse_args(argv)
    args.action = {'open': 'release', 'close': 'lock'}.get(args.action, args.action)
    if args.action == 'reset' and args.slot is not None:
        cli.error('reset 复位所有已配置舵机，不填写单个编号')
    if args.action != 'reset' and (args.action is None) != (args.slot is None):
        cli.error('动作和编号须一起给出，例如 open 1 或 close 1')
    if args.check and args.action:
        cli.error('--check不能和动作一起使用')
    config = apply_flight_configuration(base_configuration(args.config), args.flight_config, task='delivery-route')
    payload = config['payload']
    slots = configured_slots(payload)
    if args.action not in ('center', 'reset'):
        # Preserve the existing early validation for timed bench actions.
        actuator_request(args.slot or 1, args.action or 'release', payload, args.seconds)
    print('原机器配置：'+config['_config_path'])
    print('调参文件：'+config['_flight_configuration']['path'])
    print('投放编号按配置映射物理MAIN输出；双舵机开合和复位按实际量程换算并持续保持。'
          if payload.get('main_outputs') else '各口释放PWM和回中位按飞控实际量程换算；其余开合沿用原任务值。')
    if args.dry_run:
        print('离线预览按配置量程计算；未读取飞控各口的实际MIN/MAX。')
        if args.action == 'reset':
            selected = [actuator_request(slot, 'center', payload, args.seconds) for slot in reset_slots(payload)]
        else:
            selected = ([actuator_request(args.slot, args.action, payload, args.seconds)]
                        if args.action else [actuator_request(s, a, payload, args.seconds)
                                             for s in slots for a in ('release', 'lock')])
        if args.action in ('reset', 'center'):
            print('复位目标：{} μs；飞控保存并保持，退出脚本不恢复旧位置。'.format(
                payload.get('reset_pwm_us', payload['pwm_center_us'])))
            print('无定时回弹；--seconds 不影响回位保持。飞控重启后需重新执行。')
            print('以下预览按输出未反向计算；执行时读取飞控 MAIN 反向配置换算。')
        print(json.dumps(dict(connected=False, commands=selected), ensure_ascii=False, indent=2))
        return 0
    if not os.environ.get('AIRSEAFLY_SERVO_ROS_READY'):
        setup = Path('/opt/ros/noetic/setup.sh')
        if not setup.is_file():
            raise ValueError('本机没有ROS；本地可用 --dry-run 预览，在机载运行实际测试')
        env = dict(os.environ, AIRSEAFLY_SERVO_ROS_READY='1')
        # Pass all caller arguments as argv, not interpolated shell text.
        os.execve('/bin/sh', ['sh', '-ec', 'load_env() { . /opt/ros/noetic/setup.sh; }; load_env; exec "$@"',
                  'servo-test', sys.executable, str(Path(__file__).resolve()),
                  *(sys.argv[1:] if argv is None else argv)], env)
    master = 'http://127.0.0.1:11341'
    os.environ.update(ROS_MASTER_URI=master, ROS_IP='127.0.0.1')
    os.environ.pop('ROS_HOSTNAME', None)
    log_dir = PROJECT/'data/servo_test'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    log_dir.mkdir(parents=True, exist_ok=True)
    with Connection(master, args.fcu_url, log_dir):
        import rospy
        from mavros_msgs.msg import State
        from mavros_msgs.srv import CommandLong, ParamPull
        rospy.init_node('main_servo_test', anonymous=True, disable_signals=True)
        deadline = time.monotonic()+20
        while True:
            state = rospy.wait_for_message('/mavros/state', State, timeout=5)
            if state.connected:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('MAVROS未连接飞控')
        rospy.wait_for_service('/mavros/param/pull', timeout=10)
        result = rospy.ServiceProxy('/mavros/param/pull', ParamPull)(force_pull=True)
        if not result.success:
            raise RuntimeError('未能读取完整PX4参数')
        parameters = rospy.get_param('/mavros/param')
        (log_dir/'parameters.json').write_text(json.dumps(parameters, ensure_ascii=False, indent=2))
        rows, errors = inspect_mapping(parameters, payload)
        for row in rows:
            print('{output}：功能{function}（期望{expected_function}），输出范围{minimum_us}～{maximum_us} μs'.format(**row))
        print('电机解锁={}；COM_MOT_TEST_EN={}；COM_PREARM_MODE={}'.format(
            state.armed, parameters.get('COM_MOT_TEST_EN'), parameters.get('COM_PREARM_MODE')))
        print('日志：'+str(log_dir))
        if errors:
            raise ValueError('；'.join(errors))
        if args.check:
            return 0
        transport = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        def send(slot, action):
            current = rospy.wait_for_message('/mavros/state', State, timeout=3)
            # Refresh the cached mapping so a concurrent parameter change is
            # not silently hidden for the entire menu session.
            current_parameters = rospy.get_param('/mavros/param')
            command = actuator_request(slot, action, payload, args.seconds, current_parameters)
            with (log_dir/'actions.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(event='request', command=command))+'\n')
            label = {'release': '释放', 'lock': '闭合', 'center': '关闭复位' if payload.get('main_outputs') else '回中位'}[action]
            duration = ('保持至下一次舵机指令，退出脚本不恢复旧位置'
                        if command.get('persistent') else '持续{}秒'.format(args.seconds))
            print('投放编号{} → MAIN{} {}，配置目标 {} μs，归一化{:+.3f}，{}。'.format(
                slot, command.get('main_output', slot), label, command['configured_pwm_us'], command['normalized'], duration))
            try:
                reply = send_once(command, current, current_parameters, payload, transport)
            except Exception as exc:
                raise RuntimeError(str(exc)+'；执行结果未确认，未自动重发') from exc
            with (log_dir/'actions.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(event='ack', success=reply.success, result=reply.result))+'\n')
            print('飞控回执：success={}，result={}；请观察对应舵机。'.format(reply.success, reply.result))
            if not reply.success:
                if action == 'center':
                    print('飞控未接受回位保持命令；未自动重发。')
                else:
                    print('飞控未接受：检查其执行器测试设置和机体安全按钮状态；本脚本不会自动改参数或解锁。')
            # Keep an owned MAVROS alive until the single test expires. This
            # waits locally and does not repeat/renew the actuator command.
            if command['command'] == 310:
                time.sleep(args.seconds)
            return bool(reply.success)
        if args.action == 'reset':
            return run_reset(send, slots=reset_slots(payload))
        if args.action:
            return 0 if send(args.slot, args.action) else 2
        return run_menu(send, slots=slots, persistent=bool(payload.get('main_outputs')))


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (KeyboardInterrupt, EOFError):
        print('\n退出测试；未追加舵机动作。')
        sys.exit(130)
    except Exception as exc:
        print('舵机测试：'+str(exc), file=sys.stderr)
        sys.exit(2)
