#!/usr/bin/env python3
"""CH9: middle closes MAIN6/7, low opens both, high sends nothing."""
import threading

from airseafly_real400_mission.flight_configuration import (
    apply_flight_configuration, base_configuration,
)
from airseafly_real400_mission.rc_payload import paired_command, switch_position


class RCPayloadControl:
    def __init__(self, ros, rc_type, command_type, config):
        self.ros = ros
        self.payload, self.settings = config['payload'], config['rc_payload']
        self.condition = threading.Condition()
        self.position, self.pending = None, None
        self.revision, self.running = 0, True
        self.transport = ros.ServiceProxy('/mavros/cmd/command', command_type)
        self.worker = threading.Thread(target=self.run, name='rc-payload', daemon=True)
        ros.on_shutdown(self.close)
        self.worker.start()
        self.subscriber = ros.Subscriber('/mavros/rc/in', rc_type, self.on_rc, queue_size=1)
        ros.loginfo('RC%d 舵机控制：中位 MAIN6/7=%s μs，低位=%s μs，高位不动作；首帧及换挡发送一次',
                    self.settings['channel'], self.settings['closed_pwm_us'], self.settings['open_pwm_us'])

    def on_rc(self, message):
        position = switch_position(message.channels, self.settings)
        with self.condition:
            if not self.running or position is None or position == self.position:
                return
            self.position = position
            self.revision += 1
            # A newer switch position replaces an unsent action. High cancels it;
            # an already sent command retains its output and real FCU receipt.
            self.pending = None if position == 'high' else (self.revision, position)
            self.condition.notify()

    def run(self):
        while True:
            with self.condition:
                while self.running and self.pending is None:
                    self.condition.wait()
                if not self.running:
                    return
                revision, position = self.pending
                self.pending = None
            try:
                parameters = self.ros.get_param('/mavros/param', {})
                command = paired_command(position, self.payload, self.settings, parameters)
                with self.condition:
                    if not self.running or revision != self.revision:
                        continue
                fields = dict(broadcast=False, command=command['command'], confirmation=0, param7=0.)
                fields.update({'param%d' % index: value
                               for index, value in enumerate(command['params'], 1)})
                self.ros.loginfo('RC%d %s：请求 MAIN6/7 同时到 %s μs',
                                 self.settings['channel'], position, command['pwm_us'])
                response = self.transport(**fields)
                log = self.ros.loginfo if response.success and response.result == 0 else self.ros.logerr
                log('RC%d %s 舵机回执：success=%s result=%s',
                    self.settings['channel'], position, response.success, response.result)
            except Exception as exc:
                self.ros.logerr('RC%d %s 舵机命令未确认：%s', self.settings['channel'], position, exc)

    def close(self):
        with self.condition:
            self.running, self.pending = False, None
            self.condition.notify()


def main():
    import rospy
    from mavros_msgs.msg import RCIn
    from mavros_msgs.srv import CommandLong

    rospy.init_node('rc_payload_control')
    config = apply_flight_configuration(base_configuration(),
        rospy.get_param('~flight_config', '') or None, task='delivery-route')
    node = RCPayloadControl(rospy, RCIn, CommandLong, config)
    try:
        rospy.spin()
    finally:
        node.close()


if __name__ == '__main__':
    main()
