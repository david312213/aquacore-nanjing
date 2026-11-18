"""RC switch positions and a single command for both MAIN6/7 dispensers."""
import math

from .payload import actuator_command, main_output, reset_slots


def switch_position(channels, settings):
    index = int(settings['channel']) - 1
    if index < 0 or index >= len(channels):
        return None
    pwm = channels[index]
    # RCIn uses uint16 values; absent/unavailable channels are not switch positions.
    if pwm in (0, 65535):
        return None
    if pwm <= settings['low_max_us']:
        return 'low'
    if pwm >= settings['high_min_us']:
        return 'high'
    return 'middle'


def paired_command(position, payload, settings, parameters=None):
    """Use the existing physical-output calibration, leaving other outputs alone."""
    if position == 'high':
        return None
    action = {'low': 'release', 'middle': 'lock'}[position]
    pwm = settings['open_pwm_us' if position == 'low' else 'closed_pwm_us']
    manual = dict(payload, released_pwm_us=pwm, locked_pwm_us=pwm,
                  released_pwm_us_by_slot={})
    slots = {main_output(slot, payload): slot for slot in reset_slots(payload)}
    params = [float('nan')] * 6
    for output in (6, 7):
        command = actuator_command(slots[output], action, manual, parameters)
        index = command['channel'] - 1
        if math.isfinite(params[index]):
            raise ValueError('MAIN6/7 必须使用不同的 Actuator Set 通道')
        params[index] = command['normalized']
    return dict(position=position, pwm_us=pwm, command=187, params=params)
