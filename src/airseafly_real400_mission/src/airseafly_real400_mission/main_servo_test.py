"""Ground actuator-test commands. No ROS or hardware IO in this module.

PX4 Commander maps MAV_CMD_ACTUATOR_TEST param5 >= 1000 to its output
function minus 1000. Peripheral Actuator Set 1..6 are functions 301..306.
Explicit main_outputs mappings use persistent commands for two-stage
dispensers. Legacy release/lock tests remain timed. Physical PWM commands
use each output's current MIN/MAX and persistent commands honor reversal.
"""
import math
import re

from .payload import (actuator_command, configured_slots, normalize_output_pwm,
                      output_pwm_config, main_output, check_pwm_limit)


def persistent_request(slot, action, channel, pwm, normalized):
    """Replace one stored actuator value; NaN leaves other channels alone."""
    fields = {'param'+str(i): float('nan') for i in range(1, 7)}
    fields['param'+str(channel)] = normalized
    return dict(slot=slot, action=action, channel=channel, function=300+channel,
                configured_pwm_us=pwm, normalized=normalized, persistent=True,
                command=187, broadcast=False, confirmation=0, param7=0., **fields)


def main1_payload(payload):
    """Scope existing mapping checks to MAIN1 without mutating task config."""
    if '1' not in payload['actuator_indices']:
        raise ValueError('配置中没有 MAIN1 的执行器通道')
    return dict(payload, actuator_indices={'1': payload['actuator_indices']['1']}, main_outputs={})


def main1_pwm_request(pwm, payload, parameters=None):
    return main_pwm_request(1, pwm, main1_payload(payload), parameters)


def main_pwm_request(slot, pwm, payload, parameters=None):
    """Physical MAIN port and actuator-set channel are separate addresses."""
    channel = int(payload['actuator_indices'][str(slot)])
    if channel not in range(1, 7):
        raise ValueError('Actuator Set 1通道必须为1至6')
    parameters = parameters or {}
    output = output_pwm_config(slot, payload, parameters)
    pwm = check_pwm_limit(pwm, payload)
    low, high = float(output['pwm_min_us']), float(output['pwm_max_us'])
    # Raw PWM uses the physical limits, independently of the mission's center.
    if not all(map(math.isfinite, (pwm, low, high))) or not low < high or not low <= pwm <= high:
        raise ValueError('MAIN{} PWM 须在当前范围 {:g}～{:g} μs 内'.format(slot, low, high))
    value = 2. * (pwm-low) / (high-low) - 1.
    if int(parameters.get('PWM_MAIN_REV', 0)) & (1 << (slot-1)):
        value *= -1.
    return persistent_request(slot, 'pwm', channel, pwm, value)


def run_pwm_menu(send, read=input, emit=print, slot=1):
    emit('输入 MAIN{} PWM（μs）并回车；q退出。每次输入发送一次，退出不自动回位。'.format(slot))
    try:
        while True:
            value = read('MAIN{} PWM [μs/q]：'.format(slot)).strip().lower()
            if value in ('q', 'quit', 'exit'):
                break
            if not value:
                continue
            try:
                pwm = float(value)
                if not math.isfinite(pwm):
                    raise ValueError()
            except ValueError:
                emit('请输入有限的 PWM 数值，单位 μs。')
                continue
            try:
                send(pwm)
            except (RuntimeError, OSError, ValueError) as exc:
                emit('测试失败：{}；可继续输入 PWM。'.format(exc))
    except (EOFError, KeyboardInterrupt):
        pass
    emit('\n已退出；未追加舵机动作，不自动回位。')
    return 0


def run_port_pwm_menu(send, read=input, emit=print, slots=(6, 7)):
    emit('输入“端口 PWM”，例如 6 1500 或 7 1500；q退出。每次只控制指定端口，退出不自动回位。')
    try:
        while True:
            value = read('MAIN端口 PWM [例如 6 1500/q]：').strip().lower()
            if value in ('q', 'quit', 'exit'):
                break
            if not value:
                continue
            try:
                port, raw = value.split()
                slot, pwm = int(port), float(raw)
                if slot not in slots or not math.isfinite(pwm):
                    raise ValueError()
            except ValueError:
                emit('请输入端口 {} 和有限的 PWM 数值，例如 6 1500。'.format('/'.join(map(str, slots))))
                continue
            try:
                send(slot, pwm)
            except (RuntimeError, OSError, ValueError) as exc:
                emit('测试失败：{}；可继续输入。'.format(exc))
    except (EOFError, KeyboardInterrupt):
        pass
    emit('\n已退出；未追加舵机动作，不自动回位。')
    return 0


def actuator_request(slot, action, payload, seconds=2.0, parameters=None):
    flight = actuator_command(slot, 'lock' if action == 'center' else action, payload, parameters)
    parameters = parameters or {}
    output_payload = output_pwm_config(flight['slot'], payload, parameters)
    if payload.get('main_outputs'):
        # Two-stage dispensers keep their chosen position; reset means closed.
        pwm = float(payload.get('reset_pwm_us', payload['locked_pwm_us'])) if action == 'center' else flight['pwm_us']
        value = normalize_output_pwm(pwm, output_payload)
        if int(parameters.get('PWM_MAIN_REV', 0)) & (1 << (flight['main_output']-1)):
            value *= -1.
        command = persistent_request(slot, action, flight['channel'], pwm, value)
        command['main_output'] = flight['main_output']
        return command
    if action == 'center':
        # PX4's final PWM conversion is linear over MIN..MAX. Mission values
        # use a piecewise scale, whose zero need not produce pwm_center_us.
        # Center uses the inverse of the selected physical output conversion,
        # including MAIN reversal; the ports need not have identical limits.
        flight['pwm_us'] = float(payload['pwm_center_us'])
        flight['normalized'] = normalize_output_pwm(flight['pwm_us'], output_payload)
        if int(parameters.get('PWM_MAIN_REV', 0)) & (1 << (flight['slot'] - 1)):
            flight['normalized'] *= -1.
        # DO_SET_ACTUATOR has no expiry: each finite value replaces the
        # corresponding stored actuator value. NaN leaves other slots alone.
        return persistent_request(flight['slot'], action, flight['channel'],
                                  flight['pwm_us'], flight['normalized'])
    overrides = payload.get('released_pwm_us_by_slot', {})
    if action == 'release' and (str(flight['slot']) in overrides or flight['slot'] in overrides):
        # Per-slot overrides specify physical PWM, not a fraction of the
        # YAML's common range. Use the selected MAIN output's actual range.
        flight['normalized'] = normalize_output_pwm(flight['pwm_us'], output_payload)
    seconds = float(seconds)
    if not math.isfinite(seconds) or not 0 < seconds <= 3:
        raise ValueError('单次测试时长须大于0且不超过3秒')
    function = 300 + flight['channel']
    return dict(slot=flight['slot'], action=action, function=function,
                configured_pwm_us=flight['pwm_us'], normalized=flight['normalized'],
                command=310, broadcast=False, confirmation=0,
                param1=flight['normalized'], param2=seconds, param3=0., param4=0.,
                param5=float(1000+function), param6=0., param7=0.)


def run_reset(send, emit=print, slots=(1, 2, 3)):
    """One explicit center command for each slot, with no automatic retry."""
    emit('投放装置复位：依次请求编号 {} 回到配置复位位置并保持，退出脚本不恢复旧位置。'.format(
        '、'.join(map(str, slots))))
    accepted = []
    for slot in slots:
        try:
            if send(slot, 'center'):
                accepted.append(slot)
        except (RuntimeError, OSError, ValueError) as exc:
            emit('{}号口回位失败或回执未确认：{}；未自动重发。'.format(slot, exc))
    if len(accepted) == len(slots):
        emit('全部复位保持请求均获飞控接受；下一次舵机指令前保持，请观察实际位置。')
        return 0
    missing = [str(slot) for slot in slots if slot not in accepted]
    emit('回位未全部确认：{}号口。'.format('、'.join(missing)))
    return 2


def inspect_mapping(parameters, payload, slots=None):
    rows, errors, seen = [], [], set()
    for slot in configured_slots(payload) if slots is None else slots:
        if slots is None:
            expected = actuator_request(slot, 'lock', payload)['function']
        else:
            channel = int(payload['actuator_indices'][str(slot)])
            if channel not in range(1, 7):
                raise ValueError('Actuator Set 1通道必须为1至6')
            expected = 300+channel
        output = main_output(slot, payload)
        if (output, expected) in seen:
            continue
        seen.add((output, expected))
        name = 'PWM_MAIN_FUNC'+str(output)
        actual = parameters.get(name)
        row = dict(slot=slot, main_output=output, output='MAIN'+str(output), function=actual,
                   expected_function=expected,
                   minimum_us=parameters.get('PWM_MAIN_MIN'+str(output)),
                   maximum_us=parameters.get('PWM_MAIN_MAX'+str(output)))
        rows.append(row)
        if actual != expected:
            errors.append('{}={}，任务需要{}'.format(name, actual, expected))
        # Function addressing would move every output sharing that function.
        duplicates = sorted(k for k, v in parameters.items() if v == expected
                            and k != name and re.fullmatch(r'PWM_(MAIN|AUX)_FUNC\d+', k))
        if duplicates:
            errors.append('功能{}重复映射到{}'.format(expected, ', '.join(duplicates)))
        # Different MIN/MAX values are normal per-output calibration. They
        # are displayed above and used when constructing physical PWM commands.
    return rows, errors


def send_once(command, state, parameters, payload, transport, slots=None):
    """Validate the selected physical mapping, then exactly one transport call."""
    if not state.connected or state.armed:
        raise ValueError('地面舵机测试要求飞控已连接且电机未解锁')
    _rows, errors = inspect_mapping(parameters, payload, slots=slots)
    if errors:
        raise ValueError('；'.join(errors))
    if command['command'] == 310 and parameters.get('COM_MOT_TEST_EN') != 1:
        raise ValueError('PX4未启用执行器测试（COM_MOT_TEST_EN）；未发送')
    fields = ('command', 'broadcast', 'confirmation') + tuple('param'+str(i) for i in range(1, 8))
    return transport(**{key: command[key] for key in fields})


def run_menu(send, read=input, emit=print, slots=(1, 2, 3), persistent=False):
    choices = tuple(map(str, slots))
    numbers = '/'.join(choices)
    emit('投放开合测试：输入 {} 选择投放编号，再选打开或闭合；q退出。'.format(numbers))
    emit('每次选择只发送一次，保持至下一次命令；不会自动解锁。' if persistent else
         '每次选择只发送一次，最长3秒后PX4恢复原输出；不会自动解锁。')
    try:
        while True:
            choice = read('投放编号 [{}/q]：'.format(numbers)).strip().lower()
            if choice in ('q', 'quit', 'exit'):
                return 0
            if choice not in choices:
                continue
            action = read('1打开（释放） / 2闭合 / b返回 / q退出：').strip().lower()
            if action == 'q':
                return 0
            action = {'1': 'release', '2': 'lock', 'open': 'release', 'close': 'lock',
                      'release': 'release', 'lock': 'lock'}.get(action)
            if action is None:
                continue
            try:
                send(int(choice), action)
            except (RuntimeError, OSError, ValueError) as exc:
                emit('测试失败或回执未确认：{}；未自动重发。'.format(exc))
    except (EOFError, KeyboardInterrupt):
        emit('\n已退出；未追加舵机动作。')
    return 0
