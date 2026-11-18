#!/usr/bin/env python3
"""Bounded, allowlisted PX4 inspection over the existing MAVROS connection.

Sends only SERIAL_CONTROL packets for the fixed read-only commands below.
Does not open the FCU serial device, change parameters, arm, or send setpoints.
The shell wire layout is from mavlink_msg_serial_control.h (CRC extra 220).
"""
import argparse
import json
import struct
import threading
import time

import rospy
from mavros_msgs.msg import Mavlink, State


def crc_x25(data):
    crc = 0xffff
    for byte in data:
        tmp = byte ^ (crc & 0xff)
        tmp ^= (tmp << 4) & 0xff
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff
    return crc


def serial_packet(data, seq, close=False):
    if len(data) > 70:
        raise ValueError('SERIAL_CONTROL payload is limited to 70 bytes')
    # device 10 = SHELL; flags 6 = RESPOND | EXCLUSIVE, as PX4's shell tool.
    payload = struct.pack('<IHBBB70s', 0, 0, 10, 0 if close else 6,
                          len(data), data)
    header = struct.pack('<BBBBB', len(payload), seq, 245, 190, 126)
    packet = Mavlink()
    packet.header.stamp = rospy.Time.now()
    packet.framing_status = Mavlink.FRAMING_OK
    packet.magic = Mavlink.MAVLINK_V10
    packet.len = len(payload)
    packet.seq, packet.sysid, packet.compid, packet.msgid = seq, 245, 190, 126
    packet.checksum = crc_x25(header + payload + bytes([220]))
    padded = payload + b'\0' * ((-len(payload)) % 8)
    packet.payload64 = struct.unpack('<%dQ' % (len(padded) // 8), padded)
    return packet


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=['identity', 'height', 'topics'], default='identity')
    parser.add_argument('--instance', type=int, choices=range(4), default=0)
    parser.add_argument('--timeout', type=float, default=4.0)
    args = parser.parse_args()
    if not 1 <= args.timeout <= 10:
        parser.error('timeout must be 1 to 10 seconds per command')
    identity = ['ver all', 'ekf2 status', 'listener estimator_selector_status -n 1',
                'param show EKF2_EV_CTRL', 'param show EKF2_HGT_REF', 'logger status']
    height = ['listener estimator_selector_status -n 1',
              'listener estimator_status_flags -n 1 -i %d' % args.instance,
              # Aid-source uORB instance allocation need not equal EKF instance.
              # Inspect estimator_instance inside BOTH messages on this 2-EKF FCU.
              'listener estimator_aid_src_ev_hgt -n 1 -i 0',
              'listener estimator_aid_src_ev_hgt -n 1 -i 1',
              'listener estimator_event_flags -n 1 -i %d' % args.instance,
              'listener estimator_aid_src_baro_hgt -n 1 -i %d' % args.instance,
              'listener estimator_baro_bias -n 1 -i %d' % args.instance,
              'listener vehicle_visual_odometry -n 1',
              'listener vehicle_local_position -n 1']
    commands = {'identity': identity, 'height': height, 'topics': ['uorb status']}[args.profile]
    rospy.init_node('px4_shell_readonly', anonymous=True, argv=['px4_shell_readonly'])
    pub = rospy.Publisher('/mavlink/to', Mavlink, queue_size=10)
    state = {'value': None, 'received': 0.0}
    lock = threading.Lock()
    buffer = []

    def on_state(msg):
        state.update(value=msg, received=time.monotonic())

    def on_mavlink(msg):
        if msg.msgid != 126 or msg.sysid != 1 or msg.compid != 1:
            return
        raw = struct.pack('<%dQ' % len(msg.payload64), *msg.payload64)[:msg.len]
        if len(raw) >= 9 and raw[6] == 10 and raw[8] <= 70:
            with lock:
                buffer.append(raw[9:9 + raw[8]].decode('utf-8', 'replace'))

    subscribers = [rospy.Subscriber('/mavros/state', State, on_state, queue_size=5),
                   rospy.Subscriber('/mavlink/from', Mavlink, on_mavlink, queue_size=2000)]

    def check_state():
        msg = state['value']
        return (msg is not None and msg.connected and not msg.armed
                and time.monotonic() - state['received'] < 2.0)

    deadline = time.monotonic() + 6
    while (not check_state() or not pub.get_num_connections()) and time.monotonic() < deadline:
        time.sleep(.05)
    if not check_state() or not pub.get_num_connections():
        raise RuntimeError('Fresh connected, disarmed state and MAVROS subscriber required')
    seq = 0

    def send(data, close=False):
        nonlocal seq
        pub.publish(serial_packet(data, seq, close=close))
        seq = (seq + 1) % 256

    def request(command):
        if not check_state():
            raise RuntimeError('FCU disconnected, armed, or state stale: inspection stopped')
        with lock:
            buffer.clear()
        send(command.encode('ascii') + b'\n')
        deadline = time.monotonic() + args.timeout
        response = ''
        while time.monotonic() < deadline and not rospy.is_shutdown():
            time.sleep(.04)
            if not check_state():
                raise RuntimeError('FCU state became unsafe/stale during inspection')
            with lock:
                response = ''.join(buffer)
            if 'nsh>' in response or 'pxh>' in response:
                break
        return response

    results = []
    try:
        greeting = request('')
        if not greeting:
            raise RuntimeError('No PX4 shell response; did not send inspection commands')
        for command in commands:
            output = request(command)
            results.append({'command': command, 'output': output})
            print(json.dumps(results[-1], ensure_ascii=False), flush=True)
            if not output or ('nsh>' not in output and 'pxh>' not in output):
                raise RuntimeError('Shell command did not finish; no further commands sent')
    finally:
        # Release only this diagnostic shell, not the existing MAVROS connection.
        send(b'', close=True)
        time.sleep(.1)
        for subscriber in subscribers:
            subscriber.unregister()


if __name__ == '__main__':
    main()
