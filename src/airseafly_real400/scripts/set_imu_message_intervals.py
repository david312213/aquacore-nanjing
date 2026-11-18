#!/usr/bin/env python3
"""Issue the two original MAV_CMD_SET_MESSAGE_INTERVAL requests, sequentially."""
import threading


def configure_intervals(command_service, message_ids, interval_us, loginfo, logerr,
                        is_shutdown=lambda: False):
    """No retries; failure of one request is reported, then the other is tried."""
    accepted = True
    for message_id in message_ids:
        if is_shutdown():
            return False
        loginfo('请求 MAVLink 消息 %s：周期 %s μs' % (message_id, interval_us))
        try:
            reply = command_service(broadcast=False, command=511, confirmation=0,
                                    param1=float(message_id), param2=float(interval_us),
                                    param3=0., param4=0., param5=0., param6=0., param7=0.)
            ok = bool(reply.success) and int(reply.result) == 0
            text = '消息 %s 频率请求：success=%s，result=%s' % (message_id, reply.success, reply.result)
            (loginfo if ok else logerr)(text)
            accepted = accepted and ok
        except Exception as exc:
            logerr('消息 %s 频率请求失败：%s；未自动重发' % (message_id, exc))
            accepted = False
    return accepted


def main():
    import rospy
    from mavros_msgs.msg import State
    from mavros_msgs.srv import CommandLong

    rospy.init_node('set_imu_message_intervals')
    connected = threading.Event()

    def on_state(message):
        (connected.set if message.connected else connected.clear)()

    state_topic = rospy.get_param('~state_topic', '/mavros/state')
    service_name = rospy.get_param('~command_service', '/mavros/cmd/command')
    subscriber = rospy.Subscriber(state_topic, State, on_state, queue_size=1)
    try:
        while not rospy.is_shutdown() and not connected.wait(.1):
            rospy.loginfo_throttle(5., '等待 MAVROS 与飞控连接；其他基础节点继续启动。')
        if rospy.is_shutdown():
            return 0
        rospy.loginfo('MAVROS 已连接；等待频率设置服务 %s', service_name)
        rospy.wait_for_service(service_name)
        service = rospy.ServiceProxy(service_name, CommandLong)
        success = configure_intervals(service, rospy.get_param('~message_ids', [105, 31]),
                                      rospy.get_param('~interval_us', 5000),
                                      rospy.loginfo, rospy.logerr, rospy.is_shutdown)
        if success:
            rospy.loginfo('两条频率请求已接受，设置节点正常退出；基础节点继续运行。')
        elif not rospy.is_shutdown():
            rospy.logerr('频率设置有失败，请查看上述回执；基础节点继续运行。')
        return 0 if success or rospy.is_shutdown() else 1
    except rospy.ROSInterruptException:
        return 0
    finally:
        subscriber.unregister()


if __name__ == '__main__':
    raise SystemExit(main())
