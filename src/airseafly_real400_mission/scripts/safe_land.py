#!/usr/bin/env python3
"""Request controller-owned landing once, then observe; no position/servo output."""
import json
import threading
import time

import rospy
from mavros_msgs.msg import ExtendedState, State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from airseafly_real400_mission.landing import LandingObservation


def main(stop_on_terminal=False):
    rospy.init_node('safe_land', anonymous=True)
    status = LandingObservation(float(rospy.get_param('~display_stale_s', 2.0)))
    lock = threading.RLock()
    native_observation_enabled = False

    def update(name, value):
        with lock:
            old = status.values.get(name)
            status.update(name, value, time.monotonic())
            if name in ('control', 'priority', 'armed') and old != value:
                rospy.loginfo('降落状态变化：%s = %s', name, value)

    def state(msg):
        update('connected', bool(msg.connected))
        update('armed', bool(msg.armed))
        update('mode', msg.mode)

    def native_state(msg):
        try:
            packet = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(packet, dict):
            return
        with lock:
            # The controller resets its landing transaction in force_land.
            # Do not interpret an earlier transaction's latched status as q's result.
            if not native_observation_enabled:
                return
            update('landing_state', packet.get('landing_state', 'idle'))
            update('landing_reason', packet.get('landing_reason', ''))

    rospy.Subscriber('/px4ctrl/control_state', String, lambda m: update('control', m.data), queue_size=1)
    rospy.Subscriber('/px4ctrl/force_land_active', Bool, lambda m: update('priority', bool(m.data)), queue_size=1)
    rospy.Subscriber('/px4ctrl/landed', Bool, lambda m: update('landed', bool(m.data)), queue_size=1)
    rospy.Subscriber('/mavros/state', State, state, queue_size=1)
    if stop_on_terminal:
        rospy.Subscriber('/px4ctrl/native_status', String, native_state, queue_size=1)
    rospy.Subscriber('/mavros/extended_state', ExtendedState,
                     lambda m: update('on_ground', m.landed_state == ExtendedState.LANDED_STATE_ON_GROUND), queue_size=1)
    rospy.Subscriber('/Odom_high_freq', Odometry,
                     lambda m: update('xyz', (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)), queue_size=1)
    rospy.loginfo('请求当前点降落：由当前控制方执行，等待实际降落状态与停桨确认。')
    try:
        rospy.wait_for_service('/px4ctrl/force_land', timeout=3.0)
    except rospy.ROSException as exc:
        rospy.logerr('降落接口不可用，未发送请求。需启动本次更新后的 px4ctrl：%s', exc)
        return 1
    try:
        result = rospy.ServiceProxy('/px4ctrl/force_land', Trigger)()
        if not result.success:
            rospy.logerr('控制器未接受降落：%s', result.message)
            return 1
        rospy.loginfo('降落请求已提交：%s；等待飞控实际进入降落模式。', result.message)
    except rospy.ServiceException as exc:
        rospy.logerr('服务回执未确认，不自动重复发送；继续观察实际状态：%s', exc)
    with lock:
        native_observation_enabled = True
    try:
        next_print = 0.0
        was_interrupted = False
        landing_reported = False
        while not rospy.is_shutdown():
            now = time.monotonic()
            with lock:
                if status.complete(now):
                    rospy.loginfo('落地停桨：已收到降落确认，PX4 确认在地面且已解除解锁。')
                    return 0
                if not landing_reported and status.fresh('mode', now) and status.values['mode'] == 'AUTO.LAND':
                    rospy.loginfo('降落中：飞控实际模式已进入 AUTO.LAND。')
                    landing_reported = True
                failure = status.terminal_failure(now) if stop_on_terminal else None
                if failure:
                    rospy.logerr('%s；投放菜单已结束，不重新启动投放或降落请求。', failure)
                    return 1
                interrupted = status.interrupted(now)
                if interrupted and not was_interrupted:
                    # FCU disarm/state messages may arrive after px4ctrl's mode
                    # transition. Keep observing; never confuse that race with
                    # a confirmed failure or send a second landing request.
                    rospy.logwarn('独立降落控制权已交还或人工接管，尚未确认落地停桨；继续观察，不自动重新接管。')
                was_interrupted = interrupted
                if now >= next_print:
                    rospy.loginfo(status.summary(now))
                    next_print = now + 1.0
            time.sleep(.05)
    finally:
        rospy.loginfo('降落监视入口退出；已被控制器接受的降落不会因关闭此终端而撤销。')
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
