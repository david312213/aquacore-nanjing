"""Read-only progress tracking for the controller-owned priority landing."""


class LandingObservation:
    def __init__(self, stale_s=2.0):
        self.stale_s = stale_s
        self.values = {}
        self.times = {}
        self.saw_landing = False
        self.saw_priority = False

    def update(self, name, value, now):
        self.values[name] = value
        self.times[name] = now
        if name == 'control' and value == 'AUTO_LAND':
            self.saw_landing = True
        if name == 'mode' and value == 'AUTO.LAND':
            self.saw_landing = True
        if name == 'landing_state' and value == 'confirmed':
            # The controller confirms actual FCU mode (or landed+disarmed),
            # never just an accepted mode request.
            self.saw_landing = True
        if name == 'priority' and value:
            self.saw_priority = True

    def fresh(self, name, now):
        return name in self.times and 0 <= now - self.times[name] <= self.stale_s

    def complete(self, now):
        # A px4ctrl contact detector alone is not proof of stopped motors.
        return (self.saw_landing and self.fresh('armed', now) and self.fresh('on_ground', now)
                and self.fresh('connected', now) and self.values['connected']
                and self.values['armed'] is False and self.values['on_ground'] is True)

    def interrupted(self, now):
        return ((self.saw_priority or self.saw_landing)
                and self.fresh('control', now) and self.fresh('priority', now)
                and self.values['priority'] is False
                and self.values['control'] in ('MANUAL_CTRL', 'AUTO_HOVER', 'CMD_CTRL', 'PX4_FAULT', 'PX4_HOLD')
                and not self.complete(now))

    def terminal_failure(self, now):
        if not self.fresh('landing_state', now):
            return None
        label = {'failed': '降落未确认', 'cancelled': '降落已取消／人工接管'}.get(self.values['landing_state'])
        if label:
            return label + '：' + str(self.values.get('landing_reason') or self.values['landing_state'])
        return None

    def summary(self, now):
        def shown(name):
            if not self.fresh(name, now):
                return '未收到/历史数据'
            value = self.values[name]
            if isinstance(value, bool):
                return '是' if value else '否'
            if name == 'xyz':
                return '(%+.2f, %+.2f, %+.2f)m' % tuple(value)
            return str(value)
        return ('控制器=%s | 优先降落=%s | XYZ=%s | 控制器着地=%s | PX4地面=%s | 已解锁=%s' %
                tuple(shown(k) for k in ('control', 'priority', 'xyz', 'landed', 'on_ground', 'armed')))
