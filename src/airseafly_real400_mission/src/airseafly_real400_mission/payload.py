"""PWM translation only. This module never connects to the aircraft."""
import math


def configured_slots(config):
    """Inventory slots; several slots may share one physical servo output."""
    return tuple(slot for slot in (1, 2, 3, 4)
                 if str(slot) in config['actuator_indices'])


def main_output(slot, config):
    outputs = config.get('main_outputs', {})
    return int(outputs.get(str(slot), outputs.get(slot, slot)))


def reset_slots(config):
    """One close/reset command per physical servo, not per loaded object."""
    seen, slots = set(), []
    for slot in configured_slots(config):
        output = main_output(slot, config)
        if output not in seen:
            slots.append(slot)
            seen.add(output)
    return tuple(slots)


def check_pwm_limit(pwm, config):
    pwm = float(pwm)
    maximum = float(config.get('pwm_command_max_us', float('inf')))
    if config.get('main_outputs'):
        maximum = min(2000., maximum)
    if math.isnan(maximum) or not math.isfinite(pwm) or pwm > maximum:
        raise ValueError('PWM 不得超过 {:g} μs，且必须为有限数值'.format(maximum))
    return pwm


def normalize_pwm(pwm, config):
    low, center, high = (float(config[k]) for k in ('pwm_min_us', 'pwm_center_us', 'pwm_max_us'))
    pwm = float(pwm)
    if not low < center < high or not math.isfinite(pwm) or not low <= pwm <= high:
        raise ValueError('PWM与min/center/max配置不匹配')
    return (pwm - center) / (high - center if pwm >= center else center - low)


def normalize_output_pwm(pwm, config):
    """Actual PWM for PX4 Peripheral Actuator Set outputs, using MIN..MAX."""
    pwm = check_pwm_limit(pwm, config)
    low, high = float(config['pwm_min_us']), float(config['pwm_max_us'])
    if not all(map(math.isfinite, (low, high))) or not low < high or not low <= pwm <= high:
        raise ValueError('PWM与输出MIN/MAX配置不匹配')
    return 2. * (float(pwm) - low) / (high - low) - 1.


def output_pwm_config(slot, config, parameters=None):
    """Selected physical port's scale; bench feedback overrides saved limits."""
    output = dict(config)
    maxima = config.get('pwm_max_us_by_slot', {})
    output['pwm_max_us'] = maxima.get(str(slot), maxima.get(slot, config['pwm_max_us']))
    for suffix, key in (('MIN', 'pwm_min_us'), ('MAX', 'pwm_max_us')):
        output[key] = (parameters or {}).get('PWM_MAIN_'+suffix+str(main_output(slot, config)), output[key])
    return output


def actuator_command(slot, action, config, parameters=None):
    slot = int(slot)
    if slot not in configured_slots(config) or action not in ('release', 'lock'):
        raise ValueError('指定已配置的投放编号1/2/3/4和release/lock动作')
    channel = int(config['actuator_indices'][str(slot)])
    if channel not in range(1, 7):
        raise ValueError('Actuator Set 1通道必须为1至6')
    pwm = config['released_pwm_us' if action == 'release' else 'locked_pwm_us']
    overrides = config.get('released_pwm_us_by_slot', {})
    key = str(slot) if str(slot) in overrides else slot
    if action == 'release' and key in overrides:
        # Per-slot values describe actual output PWM. Keep the legacy scale
        # for unmodified slots and lock commands, including old YAML files.
        pwm = overrides[key]
    pwm = check_pwm_limit(pwm, config)
    if config.get('main_outputs') or action == 'release' and key in overrides:
        value = normalize_output_pwm(pwm, output_pwm_config(slot, config, parameters))
        if config.get('main_outputs') and int((parameters or {}).get('PWM_MAIN_REV', 0)) & (1 << (main_output(slot, config)-1)):
            value *= -1.
    else:
        value = normalize_pwm(pwm, config)
    params = [float('nan')] * 6
    params[channel - 1] = value
    return dict(slot=slot, main_output=main_output(slot, config), action=action, channel=channel, pwm_us=pwm,
                normalized=value, params=params, command=187)
