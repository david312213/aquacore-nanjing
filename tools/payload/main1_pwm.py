#!/usr/bin/env python3
"""Explicit MAIN1 or MAIN6/7 PWM commands via the existing bench connection."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys
import time

from servo_test import Connection

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT/'src/airseafly_real400_mission/src'))
from airseafly_real400_mission.flight_configuration import base_configuration, apply_flight_configuration
from airseafly_real400_mission.main_servo_test import (
    inspect_mapping, main1_payload, main_pwm_request, run_pwm_menu, run_port_pwm_menu, send_once,
)
from airseafly_real400_mission.payload import output_pwm_config


def finite_pwm(value):
    try:
        pwm = float(value)
    except ValueError:
        raise argparse.ArgumentTypeError('PWM 必须是有限数值，单位 μs')
    if not math.isfinite(pwm):
        raise argparse.ArgumentTypeError('PWM 必须是有限数值，单位 μs')
    return pwm


def parser():
    cli = argparse.ArgumentParser(description='MAIN PWM 保持测试；无数值时交互输入，退出不自动回位')
    cli.add_argument('pwm', nargs='?', type=finite_pwm, help='目标 PWM（μs）；发送一次后退出')
    cli.add_argument('--port', type=int, choices=(1, 6, 7), help='指定物理 MAIN 端口；默认 MAIN1')
    cli.add_argument('--main67', action='store_true', help='MAIN6/7 双路菜单，输入“端口 PWM”')
    cli.add_argument('--dry-run', action='store_true', help='离线预览；未指定 PWM 时只显示配置')
    cli.add_argument('--fcu-url', help='无MAVROS时使用的飞控连接；默认自动寻找PX4 USB')
    cli.add_argument('--config', help='原机器配置路径')
    cli.add_argument('--flight-config', default=str(PROJECT/'flight/flight.yaml'))
    return cli


def port_payload(payload, slot):
    if slot == 1:
        return main1_payload(payload)
    # Bench wiring selected by the user: MAIN6 -> Set1, MAIN7 -> Set2.
    # These offline bounds describe the readback on 2026-10-25; live FCU
    # parameters still override them. Mission configuration is untouched.
    return dict(payload, actuator_indices={str(slot): slot-5}, main_outputs={},
                pwm_min_us=900, pwm_max_us=2000, pwm_max_us_by_slot={}, pwm_command_max_us=2000)


def main(argv=None):
    cli = parser()
    args = cli.parse_args(argv)
    if args.main67 and args.port == 1:
        cli.error('MAIN6/7 入口仅选择 --port 6 或 --port 7')
    slots = (args.port,) if args.port is not None else ((6, 7) if args.main67 else (1,))
    if args.pwm is not None and len(slots) > 1:
        cli.error('单次 PWM 请指定端口，例如 --port 6 1500')
    config = apply_flight_configuration(base_configuration(args.config), args.flight_config, task='delivery-route')
    payloads = {slot: port_payload(config['payload'], slot) for slot in slots}
    # ROS bootstrap re-execs this program; print startup details only once.
    if args.dry_run or os.environ.get('AIRSEAFLY_SERVO_ROS_READY'):
        print('原机器配置：'+config['_config_path'])
        print('调参文件：'+config['_flight_configuration']['path'])
        print('MAIN{} PWM 保持测试：启动不发送初始值，退出不自动回位；飞控重启后需重新发送。'.format('/'.join(map(str, slots))))
    if args.dry_run:
        source = 'MAIN6/7 测试默认量程（2026-10-25 读回值）' if slots != (1,) else '配置量程'
        print('离线预览：使用{}，按未反向计算；未读取飞控当前 MIN/MAX 和反向配置。'.format(source))
        rows = []
        for slot in slots:
            payload = payloads[slot]
            output = output_pwm_config(slot, payload)
            rows.append(dict(output='MAIN'+str(slot),
                expected_function=300+int(payload['actuator_indices'][str(slot)]),
                minimum_us=output['pwm_min_us'], maximum_us=output['pwm_max_us']))
        commands = [] if args.pwm is None else [main_pwm_request(slots[0], args.pwm, payloads[slots[0]])]
        preview = dict(connected=False, outputs=rows, commands=commands)
        if len(rows) == 1:
            preview.update(rows[0])
        print(json.dumps(preview, ensure_ascii=False, indent=2))
        return 0

    # Keep the existing bench ROS bootstrap, including argv-safe re-execution.
    if not os.environ.get('AIRSEAFLY_SERVO_ROS_READY'):
        if not Path('/opt/ros/noetic/setup.sh').is_file():
            raise ValueError('本机没有ROS；本地可用 --dry-run 预览，在机载运行实际测试')
        env = dict(os.environ, AIRSEAFLY_SERVO_ROS_READY='1')
        os.execve('/bin/sh', ['sh', '-ec',
            'load_env() { . /opt/ros/noetic/setup.sh; }; load_env; exec "$@"',
            'main1-pwm', sys.executable, str(Path(__file__).resolve()),
            *(sys.argv[1:] if argv is None else argv)], env)

    master = 'http://127.0.0.1:11341'
    os.environ.update(ROS_MASTER_URI=master, ROS_IP='127.0.0.1')
    os.environ.pop('ROS_HOSTNAME', None)
    log_dir = PROJECT/('data/main1_pwm' if slots == (1,) else 'data/main67_pwm')/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    log_dir.mkdir(parents=True, exist_ok=True)

    def log(event, **fields):
        with (log_dir/'actions.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(time=datetime.now().isoformat(), event=event, **fields),
                                    ensure_ascii=False)+'\n')

    with Connection(master, args.fcu_url, log_dir):
        import rospy
        from mavros_msgs.msg import State
        from mavros_msgs.srv import CommandLong, ParamPull
        rospy.init_node('main1_pwm_test', anonymous=True, disable_signals=True)
        # These are the existing servo_test connection waits, unchanged.
        deadline = time.monotonic()+20
        while True:
            state = rospy.wait_for_message('/mavros/state', State, timeout=5)
            if state.connected:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('MAVROS未连接飞控')
        rospy.wait_for_service('/mavros/param/pull', timeout=10)
        pulled = rospy.ServiceProxy('/mavros/param/pull', ParamPull)(force_pull=True)
        if not pulled.success:
            raise RuntimeError('未能读取完整PX4参数')
        parameters = rospy.get_param('/mavros/param')
        (log_dir/'parameters.json').write_text(json.dumps(parameters, ensure_ascii=False, indent=2))
        errors = []
        for slot in slots:
            rows, port_errors = inspect_mapping(parameters, payloads[slot], slots=(slot,))
            for row in rows:
                print('{output}：功能{function}（期望{expected_function}），飞控范围{minimum_us}～{maximum_us} μs'.format(**row))
            if port_errors:
                print('MAIN{}：{}'.format(slot, '；'.join(port_errors)))
            errors.extend(port_errors)
        print('电机解锁={}；COM_PREARM_MODE={}；MAIN反向位掩码={}'.format(
            state.armed, parameters.get('COM_PREARM_MODE'), parameters.get('PWM_MAIN_REV', 0)))
        print('日志：'+str(log_dir))
        if errors and len(slots) == 1:
            raise ValueError('；'.join(errors))
        transport = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        def send(slot, pwm):
            try:
                payload = payloads[slot]
                current = rospy.wait_for_message('/mavros/state', State, timeout=3)
                current_parameters = rospy.get_param('/mavros/param')
                output = output_pwm_config(slot, payload, current_parameters)
                print('MAIN{} 目标 {:g} μs；当前换算范围 {:g}～{:g} μs。'.format(
                    slot, pwm, float(output['pwm_min_us']), float(output['pwm_max_us'])))
                if any('PWM_MAIN_'+suffix+str(slot) not in current_parameters for suffix in ('MIN', 'MAX')):
                    print('飞控未提供的 MIN/MAX 沿用配置值。')
                command = main_pwm_request(slot, pwm, payload, current_parameters)
                log('request', command=command)
                print('归一化值{:+.6f}，请求保持至下一次舵机指令。'.format(command['normalized']))
                reply = send_once(command, current, current_parameters, payload, transport, slots=(slot,))
            except Exception as exc:
                log('error', slot=slot, pwm_us=pwm, error=str(exc))
                raise RuntimeError(str(exc)+'；未自动重发') from exc
            except KeyboardInterrupt:
                log('interrupted', slot=slot, pwm_us=pwm)
                raise
            log('ack', slot=slot, pwm_us=pwm, success=reply.success, result=reply.result)
            print('飞控回执：success={}，result={}。'.format(reply.success, reply.result))
            if reply.success:
                print('飞控已接受指令；机构实际位置和是否投放请观察确认。')
            else:
                print('飞控未接受指令；未自动重发。')
            return bool(reply.success)

        if args.pwm is not None:
            return 0 if send(slots[0], args.pwm) else 2
        if len(slots) > 1:
            return run_port_pwm_menu(send, slots=slots)
        single_send = lambda pwm: send(slots[0], pwm)
        if slots == (1,):
            return run_pwm_menu(single_send)
        return run_pwm_menu(single_send, slot=slots[0])


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (KeyboardInterrupt, EOFError):
        print('\n退出测试；未追加舵机动作，不自动回位。')
        sys.exit(130)
    except Exception as exc:
        print('MAIN PWM 测试：'+str(exc), file=sys.stderr)
        sys.exit(2)
