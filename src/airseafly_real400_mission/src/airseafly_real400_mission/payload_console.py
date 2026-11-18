"""Interactive payload choices. Transport is injected; this module has no ROS IO."""
from .payload import actuator_command, configured_slots
from .rpc import CommandNotSent
from .native_parameters import positive


ACTIONS = {'1': 'release', '2': 'lock', 'release': 'release', 'lock': 'lock',
           '释放': 'release', '闭合': 'lock'}
LABELS = {'release': '释放', 'lock': '闭合'}


def run_console(config, send, read=input, emit=print):
    payload = config['payload']
    slots = configured_slots(payload)
    choices = tuple(map(str, slots))
    emit('舵机独立调试：选择编号和动作后立即发送一次命令。')
    overrides = payload.get('released_pwm_us_by_slot', {})
    releases = '、'.join('{}号 {} μs'.format(slot,
                        overrides.get(str(slot), overrides.get(slot, payload['released_pwm_us'])))
                        for slot in slots)
    emit('释放：{}；闭合 {} μs。退出菜单保持最后指令。'.format(
        releases, payload['locked_pwm_us']))
    emit('飞控接受回执不代表物品已经脱落，请观察机构和快递。')
    pending = False
    try:
        while True:
            emit('\n投放编号：{}；q 退出'.format(' / '.join(choices)))
            choice = read('选择舵机编号：').strip().lower()
            if choice in ('q', 'quit', 'exit'):
                break
            if choice not in choices:
                emit('请输入 {} 或 q。'.format('、'.join(choices)))
                continue
            slot = int(choice)
            while True:
                action = read('舵机 {}：1 释放，2 闭合，b 返回，q 退出：'.format(slot)).strip().lower()
                if action in ('q', 'quit', 'exit'):
                    emit('已退出舵机菜单。')
                    return 0
                if action == 'b':
                    break
                if action not in ACTIONS:
                    emit('请输入 1（释放）、2（闭合）、b 或 q。')
                    continue
                action = ACTIONS[action]
                command = actuator_command(slot, action, payload)
                emit('请求：投放编号 {} → MAIN{} {}；配置通道 {}，PWM {:g} μs，归一化值 {:+.3f}'.format(
                    slot, command['main_output'], LABELS[action], command['channel'], command['pwm_us'], command['normalized']))
                pending = True
                try:
                    # One explicit choice makes one call. Never retry a lost receipt.
                    reply = send(slot, action)
                except CommandNotSent as exc:
                    emit('连接未建立，未发送舵机命令：{}'.format(exc))
                except (OSError, RuntimeError, ValueError) as exc:
                    emit('未取得执行回执：{}；结果未确认，未自动重发。'.format(exc))
                else:
                    result = reply.get('result') or {}
                    if result:
                        emit('实际命令：投放编号 {}，MAIN{}，通道 {}，PWM {} μs'.format(
                            result.get('slot', slot), result.get('main_output', command['main_output']),
                            result.get('channel', '未知'), result.get('pwm_us', '未知')))
                    if reply.get('ok') and result.get('success'):
                        emit('回执：飞控已接受{}命令。'.format(LABELS[action]))
                    else:
                        emit('回执：失败或未确认；{}；未自动重发。'.format(
                            result.get('message') or reply.get('error') or '无有效回执'))
                    if reply.get('log_dir'):
                        emit('日志：' + str(reply['log_dir']))
                pending = False
                break
    except (EOFError, KeyboardInterrupt):
        if pending:
            emit('\n已提交的请求可能仍在执行，回执未确认；未重发。')
    emit('已退出舵机菜单。')
    return 0


def run_hover_drop_console(config, send, land, read=None, emit=print, change_height=None):
    """One number is one manual release; the manager owns the hover stream."""
    read = input if read is None else read
    payload = config['payload']
    choices = tuple(map(str, configured_slots(payload)))
    emit('起飞流程已完成，持续悬停。输入投放编号并回车，立即释放一次。')
    for slot in configured_slots(payload):
        command = actuator_command(slot, 'release', payload)
        emit('{} → MAIN{}，{:g} μs'.format(slot, command['main_output'], command['pwm_us']))
    emit('{}：释放对应口；h 高度：调整高度（米）；q：当前位置自动降落。'.format(' / '.join(choices)))
    emit('例如 h 0.45：调整到本次起飞基准上方 0.45 米，支持下降和上升。')
    emit('重复输入同一编号会再次发送；菜单等待时 Ctrl+C 或输入关闭只退出菜单。')
    emit('调高中 Ctrl+C 停止当前运动并悬停，然后退出菜单。')
    pending = False
    try:
        while True:
            choice = read('投放 [{}]，调高 [h 米数]，降落 [q]：'.format('/'.join(choices))).strip().lower()
            parts = choice.split()
            if parts and parts[0] == 'h':
                try:
                    if len(parts) != 2:
                        raise ValueError('请输入 h 高度，例如 h 0.45')
                    height = positive(float(parts[1]), '目标高度')
                except ValueError as exc:
                    emit(str(exc)); continue
                if change_height is None:
                    emit('当前入口未连接调高接口。'); continue
                # This callback observes one manager-owned motion. It never
                # publishes setpoints from this input thread or retries it.
                try:
                    result = change_height(height)
                except (OSError,RuntimeError,ValueError) as exc:
                    emit('调高结果未确认：{}；未自动重发，退出菜单。'.format(exc))
                    return 1
                if result != 0:
                    return result
                continue
            if choice == 'q':
                # Never return to payload input after landing, even if it fails.
                emit('已结束投放菜单，正在请求当前位置自动降落。')
                try:
                    return land()
                except (EOFError, KeyboardInterrupt):
                    emit('已退出降落观察；已提交的降落不会因此撤销，投放菜单保持结束。')
                    return 0
                except (OSError, RuntimeError, ValueError) as exc:
                    emit('降落结果未确认：{}；投放菜单已结束，未重新请求。'.format(exc))
                    return 1
            if choice not in choices:
                emit('请输入 {}、h 高度或 q。'.format('、'.join(choices)))
                continue
            slot = int(choice)
            command = actuator_command(slot, 'release', payload)
            emit('请求：投放编号 {} → MAIN{} 释放；配置通道 {}，PWM {} μs'.format(
                slot, command['main_output'], command['channel'], command['pwm_us']))
            pending = True
            try:
                reply = send(slot, 'release')
            except CommandNotSent as exc:
                emit('未发送投放请求：{}'.format(exc))
            except (OSError, RuntimeError, ValueError) as exc:
                emit('未取得执行回执：{}；结果未确认，未自动重发。'.format(exc))
            else:
                result = reply.get('result') or {}
                if reply.get('ok') and result.get('success'):
                    emit('回执：飞控已接受投放编号 {} 的释放命令。'.format(slot))
                else:
                    emit('回执：失败或未确认；{}；未自动重发。'.format(
                        result.get('message') or reply.get('error') or '无有效回执'))
                if reply.get('log_dir'):
                    emit('日志：' + str(reply['log_dir']))
            pending = False
    except (EOFError, KeyboardInterrupt):
        if pending:
            emit('\n已提交的投放请求可能仍在执行，回执未确认；未重发。')
    emit('已退出投放菜单，悬停由后台继续保持。')
    return 0
