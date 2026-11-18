#!/usr/bin/env python3
"""Localization gate: PX4/FAST-LIO divergence while armed in OFFBOARD -> /px4ctrl/force_land.

Same landing path as flight/safe_land.sh; the mission manager already stops the
task when px4ctrl reports force_land_active. Latched until disarm, no auto resume.
A pilot mode switch (leaving OFFBOARD) stops any further landing request.
"""
import json
import threading
import time

from airseafly_real400_mission.flight_configuration import (
    apply_flight_configuration, base_configuration,
)
from airseafly_real400_mission.localization_gate import DivergenceGate

RETRY_S = 1.0


class LocalizationGateNode:
    def __init__(self, ros, state_type, string_type, trigger_type, config):
        self.ros, self.string_type = ros, string_type
        settings = config['localization_gate']
        self.enabled = bool(settings['enabled'])
        self.gate = DivergenceGate(settings['max_error_m'], settings['hold_s'])
        self.condition = threading.Condition()
        self.armed = self.offboard = False
        self.mode = ''
        self.land_accepted, self.land_attempts, self.last_message = False, 0, ''
        self.generation, self.running = 0, True
        self.status_pub = ros.Publisher('/airseafly/native/localization_gate', string_type,
                                        queue_size=1, latch=True)
        self.land = ros.ServiceProxy('/px4ctrl/force_land', trigger_type)
        self.worker = threading.Thread(target=self.run, name='localization-gate', daemon=True)
        ros.on_shutdown(self.close)
        self.worker.start()
        ros.Subscriber('/mavros/state', state_type, self.on_state, queue_size=1)
        ros.Subscriber('/airseafly/native/localization_status', string_type, self.on_status, queue_size=10)
        if self.enabled:
            ros.loginfo('定位安全门：解锁且 OFFBOARD 时，PX4 与 FAST-LIO 位置差 > %.2f m 持续 %.2f s '
                        '即结束任务并请求当前点降落', self.gate.max_error_m, self.gate.hold_s)
        else:
            ros.logwarn('定位安全门已关闭（localization_gate.enabled: false），只发布状态')
        self.publish()

    def on_state(self, message):
        with self.condition:
            armed, mode = bool(message.armed), str(message.mode)
            if self.armed and not armed:
                if self.gate.tripped:
                    self.ros.loginfo('定位安全门：已上锁，触发状态清除')
                self.gate.reset()
                self.generation += 1  # a reply still in flight belongs to the previous flight
                self.land_accepted, self.land_attempts, self.last_message = False, 0, ''
            self.armed, self.mode, self.offboard = armed, mode, mode == 'OFFBOARD'
            self.condition.notify()
        self.publish()

    def on_status(self, message):
        try:
            values = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(values, dict):
            return
        with self.condition:
            tripped = self.gate.update(values.get('stamp'), values.get('px4_error_m'),
                                       self.enabled and self.armed and self.offboard)
            if tripped:
                self.ros.logerr('定位安全门触发：PX4 与 FAST-LIO 位置差 %.2f m 已持续 >= %.2f s（阈值 %.2f m）；'
                                '结束任务并请求当前点降落', self.gate.trip_error_m, self.gate.hold_s,
                                self.gate.max_error_m)
                self.condition.notify()
        if tripped:
            self.publish()

    def _should_request(self):
        return (self.running and self.gate.tripped and not self.land_accepted
                and self.armed and self.offboard)

    def run(self):
        next_attempt = 0.
        while True:
            with self.condition:
                while self.running and not (self._should_request() and time.monotonic() >= next_attempt):
                    wait = next_attempt - time.monotonic() if self._should_request() else None
                    self.condition.wait(wait)
                if not self.running:
                    return
                self.land_attempts += 1
                attempt, generation = self.land_attempts, self.generation
                next_attempt = time.monotonic() + RETRY_S
            try:
                result = self.land()
                accepted, message = bool(result.success), str(result.message)
            except Exception as exc:
                accepted, message = False, '降落服务未确认：%s' % exc
            with self.condition:
                if generation != self.generation:
                    continue
                self.last_message = message
                if accepted:
                    self.land_accepted = True
            if accepted:
                self.ros.logerr('定位安全门：第 %d 次降落请求已被控制器接受：%s', attempt, message)
            else:
                self.ros.logerr('定位安全门：第 %d 次降落请求未被接受：%s；%.0f s 后在仍为 OFFBOARD 时重试',
                                attempt, message, RETRY_S)
            self.publish()

    def publish(self):
        with self.condition:
            packet = dict(enabled=self.enabled, armed=self.armed, mode=self.mode,
                          max_error_m=self.gate.max_error_m, hold_s=self.gate.hold_s,
                          tripped=self.gate.tripped, trip_stamp=self.gate.trip_stamp,
                          trip_error_m=self.gate.trip_error_m, land_attempts=self.land_attempts,
                          land_accepted=self.land_accepted, last_message=self.last_message)
        self.status_pub.publish(self.string_type(data=json.dumps(packet, ensure_ascii=False)))

    def close(self):
        with self.condition:
            self.running = False
            self.condition.notify()


def main():
    import rospy
    from mavros_msgs.msg import State
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    rospy.init_node('localization_gate')
    config = apply_flight_configuration(base_configuration(),
        rospy.get_param('~flight_config', '') or None, task='delivery-route')
    node = LocalizationGateNode(rospy, State, String, Trigger, config)
    try:
        rospy.spin()
    finally:
        node.close()


if __name__ == '__main__':
    main()
