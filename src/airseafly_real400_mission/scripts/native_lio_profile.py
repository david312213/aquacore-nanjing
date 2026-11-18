#!/usr/bin/env python3
"""Audit PX4 1.17 FAST-LIO position/yaw inputs; --apply changes only seven EKF2 parameters.

No arming, mode changes, reboot, controller gains or failsafe changes are sent.
Parameter conformance does not prove that EKF2 is accepting external odometry.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid


# Enable the chosen input before removing other position/height observations.
# Fuse external position/height and yaw; external velocity remains off.
PROFILE = {
    'EKF2_EV_CTRL': 11,
    'EKF2_HGT_REF': 3,
    'EKF2_GPS_CTRL': 0,
    'EKF2_BARO_CTRL': 0,
    'EKF2_RNG_CTRL': 0,
    'EKF2_OF_CTRL': 0,
    'EKF2_AGP_CTRL': 0,
}
PROFILE_NAME = 'px4_1_17_fast_lio_position_yaw'
LIMITATIONS = [
    'Parameter readback is configuration evidence, not evidence of active EV fusion or flight accuracy.',
    'PX4 1.17 auxiliary velocity can consume static landing_target_pose without an EKF2 enable parameter; inspect runtime aid-source status.',
    'Verify that FAST-LIO is the only publisher feeding vehicle_visual_odometry; parameters cannot identify its producer.',
    'No restart, arming, flight, failsafe validation or controller tuning is performed.',
]


def numeric(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def version_check(vehicle):
    version = int(vehicle.get('flight_sw_version', 0))
    decoded = ((version >> 24) & 255, (version >> 16) & 255, (version >> 8) & 255)
    if int(vehicle.get('autopilot', -1)) != 12 or not int(vehicle.get('available_info', 0)) & 2:
        raise RuntimeError('PX4 AUTOPILOT_VERSION is unavailable')
    if decoded[:2] != (1, 17):
        raise RuntimeError('This profile requires PX4 1.17; received %s' % (decoded,))
    return '.'.join(map(str, decoded))


def ground_check(state):
    for key in ('state_age_s', 'state_stamp_age_s', 'landed_age_s', 'landed_stamp_age_s'):
        age = state.get(key)
        if not numeric(age) or not -.05 <= age <= 2.5:
            raise RuntimeError('Fresh MAVROS state and extended_state required: ' + key)
    if not state.get('connected') or state.get('armed') is not False or state.get('landed_state') != 1:
        raise RuntimeError('Connected, disarmed, on-ground vehicle required')


def audit(parameters):
    missing = [key for key in PROFILE if key not in parameters]
    invalid = [key for key in PROFILE if key in parameters and not numeric(parameters[key])]
    mismatches = {key: {'actual': parameters[key], 'expected': value}
                  for key, value in PROFILE.items()
                  if key in parameters and numeric(parameters[key]) and parameters[key] != value}
    auxiliary = {key: parameters.get(key) for key in
                 ('EKF2_AVEL_DELAY', 'LTEST_MODE', 'EKF2_DRAG_CTRL', 'EKF2_ARSP_THR', 'EKF2_FUSE_BETA')}
    return dict(conformant=not (missing or invalid or mismatches), missing=missing,
                invalid=invalid, mismatches=mismatches, auxiliary_configuration=auxiliary)


def write_json(path, data, exclusive=False):
    payload = json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + '\n'
    target = path if exclusive else path.with_name(path.name + '.tmp')
    with target.open('x' if exclusive else 'w') as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    if not exclusive:
        os.replace(str(target), str(path))


def execute(backend, apply=False, backup_dir=None):
    """Run against the small ROS adapter or a deterministic fake in tests.

    A failed set may have reached the FCU. Every attempted key is journaled before
    sending it, and rollback starts with a new full pull rather than stale cache.
    """
    if apply and backup_dir is None:
        raise ValueError('--apply requires --backup-dir')
    report = dict(profile=PROFILE_NAME, desired=PROFILE.copy(), apply_requested=apply,
                  applied=False, success=False, attempted=[], verified=[], rollback=None,
                  runtime_fusion_verified=False, limitations=LIMITATIONS.copy())
    directory = None
    try:
        ground_check(backend.ground_state())
        report['vehicle'] = backend.vehicle_info()
        report['version'] = version_check(report['vehicle'])
        parameters = backend.pull_all()
        if not isinstance(parameters, dict) or not parameters or any(not numeric(v) for v in parameters.values()):
            raise RuntimeError('Complete finite parameter snapshot required')
        ground_check(backend.ground_state())
        report['before'] = audit(parameters)
        report['parameter_count'] = len(parameters)
        if backup_dir is not None:
            directory = Path(backup_dir) / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:8])
            directory.mkdir(parents=True, exist_ok=False)
            report['backup_directory'] = str(directory.resolve())
            write_json(directory / 'parameters_before.json', parameters, exclusive=True)
            write_json(directory / 'vehicle_before.json', report['vehicle'], exclusive=True)
            write_json(directory / 'report.json', report)
        if not apply:
            report['success'] = report['before']['conformant']
            report['status'] = 'audit_match' if report['success'] else 'audit_mismatch'
        else:
            if report['before']['missing'] or report['before']['invalid']:
                raise RuntimeError('Required profile parameters missing or invalid; nothing changed')
            for key, desired in PROFILE.items():
                if parameters[key] == desired:
                    continue
                ground_check(backend.ground_state())
                if backend.get(key) != parameters[key]:
                    raise RuntimeError('Parameter changed since backup: ' + key)
                report['attempted'].append(key)
                write_json(directory / 'report.json', report)
                backend.set(key, desired)
                if backend.get(key) != desired:
                    raise RuntimeError('Set/readback mismatch: ' + key)
                report['verified'].append(key)
                write_json(directory / 'report.json', report)
            ground_check(backend.ground_state())
            after = backend.pull_all()  # ParamGet alone returns the MAVROS cache.
            ground_check(backend.ground_state())
            report['after'] = audit(after)
            write_json(directory / 'parameters_after.json', after, exclusive=True)
            if not report['after']['conformant']:
                raise RuntimeError('Final forced pull does not match the profile')
            report.update(success=True, applied=True, status='applied_and_read_back')
    except Exception as error:
        report.update(error=str(error), status='failed')
        if report['attempted']:
            report['rollback'] = dict(restored=[], failures={}, verified=False)
            rollback = report['rollback']
            try:
                ground_check(backend.ground_state())
                current = backend.pull_all()
                for key in reversed(report['attempted']):
                    try:
                        ground_check(backend.ground_state())
                        value = current.get(key)
                        if value != parameters[key]:
                            if value != PROFILE[key]:
                                raise RuntimeError('Unexpected value; refusing to overwrite a concurrent change')
                            backend.set(key, parameters[key])
                            if backend.get(key) != parameters[key]:
                                raise RuntimeError('Rollback readback mismatch')
                        rollback['restored'].append(key)
                    except Exception as restore_error:
                        rollback['failures'][key] = str(restore_error)
                ground_check(backend.ground_state())
                restored = backend.pull_all()
                ground_check(backend.ground_state())
                rollback['verified'] = (not rollback['failures'] and
                    all(restored.get(key) == parameters[key] for key in report['attempted']))
                write_json(directory / 'parameters_after_failure.json', restored, exclusive=True)
            except Exception as restore_error:
                rollback['failures']['transaction'] = str(restore_error)
            report['status'] = 'failed_rolled_back' if rollback['verified'] else 'failed_rollback_incomplete'
    if directory:
        write_json(directory / 'report.json', report)
    return report


class RosBackend:
    def __init__(self, namespace='/mavros', timeout=15.):
        import rospy
        from mavros_msgs.msg import ExtendedState, State, ParamValue
        from mavros_msgs.srv import ParamGet, ParamPull, ParamSet, VehicleInfoGet
        self.ros, self.namespace, self.value_type = rospy, namespace.rstrip('/'), ParamValue
        self.lock, self.state, self.landed = threading.Lock(), None, None
        def receive(which, message):
            with self.lock:
                setattr(self, which, (message, time.monotonic()))
        self.subscribers = [
            rospy.Subscriber(self.namespace + '/state', State, lambda m: receive('state', m), queue_size=1),
            rospy.Subscriber(self.namespace + '/extended_state', ExtendedState, lambda m: receive('landed', m), queue_size=1)]
        for name, service in [('param/pull', ParamPull), ('param/get', ParamGet),
                              ('param/set', ParamSet), ('vehicle_info_get', VehicleInfoGet)]:
            topic = self.namespace + '/' + name
            rospy.wait_for_service(topic, timeout=timeout)
            setattr(self, name.replace('/', '_'), rospy.ServiceProxy(topic, service))
        until = time.monotonic() + timeout
        while time.monotonic() < until and not rospy.is_shutdown():
            with self.lock:
                ready = self.state is not None and self.landed is not None
            if ready:
                return
            time.sleep(.05)
        raise RuntimeError('Timed out waiting for MAVROS state and extended_state')

    def ground_state(self):
        with self.lock:
            if self.state is None or self.landed is None:
                return {}
            state, state_at = self.state
            landed, landed_at = self.landed
        now, ros_now = time.monotonic(), self.ros.get_time()
        return dict(connected=state.connected, armed=state.armed, landed_state=landed.landed_state,
                    state_age_s=now-state_at, landed_age_s=now-landed_at,
                    state_stamp_age_s=ros_now-state.header.stamp.to_sec(),
                    landed_stamp_age_s=ros_now-landed.header.stamp.to_sec())

    def vehicle_info(self):
        reply = self.vehicle_info_get(sysid=0, compid=0, get_all=False)
        if not reply.success or len(reply.vehicles) != 1:
            raise RuntimeError('Exactly one target vehicle with version information required')
        vehicle = reply.vehicles[0]
        return {key: getattr(vehicle, key) for key in ('sysid', 'compid', 'autopilot', 'type',
                'available_info', 'flight_sw_version', 'flight_custom_version', 'uid')}

    def pull_all(self):
        reply = self.param_pull(force_pull=True)
        if not reply.success or not reply.param_received:
            raise RuntimeError('MAVROS force_pull failed or returned no parameters')
        parameters = self.ros.get_param(self.namespace + '/param', {})
        # MAVROS omits PX4's internal _HASH_CHECK from rosparam. All actual
        # parameters must still be present; do not silently accept an incomplete backup.
        if len(parameters) != reply.param_received:
            hash_reply = self.param_get(param_id='_HASH_CHECK')
            if not (hash_reply.success and len(parameters) + 1 == reply.param_received):
                raise RuntimeError('Full-pull count does not match ROS parameter snapshot')
        return parameters

    def get(self, key):
        reply = self.param_get(param_id=key)
        if not reply.success:
            raise RuntimeError('Parameter not found: ' + key)
        return reply.value.integer if reply.value.integer else reply.value.real

    def set(self, key, value):
        reply = self.param_set(param_id=key, value=self.value_type(integer=int(value), real=0.))
        if not reply.success:
            raise RuntimeError('Parameter write not acknowledged: ' + key)
        if (reply.value.integer if reply.value.integer else reply.value.real) != value:
            raise RuntimeError('Parameter acknowledgement has unexpected value: ' + key)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='Write the seven profile parameters after a complete backup')
    parser.add_argument('--backup-dir', type=Path, help='Parent directory for a unique full snapshot/report (required for --apply)')
    parser.add_argument('--mavros-ns', default='/mavros')
    args = parser.parse_args()
    if args.apply and args.backup_dir is None:
        parser.error('--apply requires --backup-dir')
    import rospy
    rospy.init_node('native_lio_profile', anonymous=True)
    try:
        report = execute(RosBackend(args.mavros_ns), args.apply, args.backup_dir)
    except Exception as error:
        report = dict(success=False, applied=False, error=str(error), status='failed')
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0 if report['success'] else 2 if args.apply else 1


if __name__ == '__main__':
    raise SystemExit(main())
