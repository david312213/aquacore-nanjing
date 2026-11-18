#!/usr/bin/env python3
"""Read-only PX4 EKF shell snapshots, permitted only while connected and landed.

Uses MAVROS' MAVLink transport without pymavlink. Snapshots are diagnostic
evidence, never a real-time fusion-health signal or authorization to fly.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import struct
import threading
import time


TOPICS = ('estimator_status_flags', 'estimator_aid_src_ev_pos',
          'estimator_aid_src_ev_hgt')
EXCLUSION_TOPICS = ('landing_target_pose', 'estimator_aid_src_aux_vel')
COMMANDS = tuple('listener %s -n 1' % topic for topic in TOPICS)
ALLOWED_COMMANDS = frozenset(COMMANDS + ('ekf2 status', 'ver all') + tuple(
    'listener %s -n 1 -i %s' % (topic, instance)
    for topic in TOPICS + EXCLUSION_TOPICS for instance in range(16)))
EXPECTED_FLAGS = dict.fromkeys((
    'cs_gnss_pos', 'cs_gnss_vel', 'cs_gps_hgt', 'cs_baro_hgt', 'cs_rng_hgt',
    'cs_opt_flow', 'cs_aux_gpos', 'cs_fake_pos', 'cs_fake_hgt',
    'cs_inertial_dead_reckoning', 'cs_ev_vel'), False)
EXPECTED_FLAGS.update(cs_ev_pos=True, cs_ev_hgt=True, cs_ev_yaw=True)
# Yaw is checked through its control flag; aid-source snapshots cover position/height.
# PX4 v1.17 uses cs_gps_hgt, not cs_gnss_hgt, and does not expose cs_aux_vel
# in EstimatorStatusFlags. Missing fields remain unknown, never false.
ADDITIONAL_FLAGS = ('cs_gnss_hgt', 'cs_aux_vel', 'cs_rng_terrain')
ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
PROMPT = re.compile(r'(?:^|[\r\n])(?:pxh|nsh)>[ \t]*$')
UNSUPPORTED = re.compile(
    r'never published|not (?:yet )?published|not advertised|unknown topic|'
    r'not a valid topic|did not match any known topics|topic .* not found', re.I)


def x25_crc(data):
    """MAVLink X.25 accumulator, including CRC extra supplied by the caller."""
    crc = 0xffff
    for value in data:
        tmp = value ^ (crc & 0xff)
        tmp ^= (tmp << 4) & 0xff
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff
    return crc


def serial_control_fields(data, seq=0, sysid=255, compid=190, release=False):
    """Build the ROS Mavlink fields for common SERIAL_CONTROL (ID 126).

    MAVLink v1: payload <uint32 baud, uint16 timeout, uint8 device, flags,
    count, uint8 data[70]>, CRC extra 220. No shell-command generalization is
    exposed by the CLI; command validation occurs before transmission.
    """
    if not isinstance(data, bytes) or len(data) > 70 or (release and data):
        raise ValueError('SERIAL_CONTROL requires at most 70 bytes; release is empty')
    if any(not isinstance(v, int) or not 0 <= v <= 255 for v in (seq, sysid, compid)):
        raise ValueError('MAVLink v1 identifiers must be uint8')
    payload = struct.pack('<IHBBB70s', 0, 0, 10, 0 if release else 6, len(data), data)
    header = bytes((len(payload), seq, sysid, compid, 126))
    return dict(framing_status=1, magic=0xfe, len=len(payload), incompat_flags=0,
                compat_flags=0, seq=seq, sysid=sysid, compid=compid, msgid=126,
                checksum=x25_crc(header + payload + bytes((220,))),
                payload64=list(struct.unpack('<10Q', payload + b'\0')))


def serial_control_text(message, target_system=1):
    """Decode shell response data from a MAVROS Mavlink message, or ignore it."""
    if message.msgid != 126 or message.sysid != target_system:
        return None
    # framing_status == 1 is MAVLINK_FRAMING_OK. Do not accept CRC failures.
    # v1.17 can return MAVLink 2's target_system/target_component extensions
    # after data[70], so a valid response is up to 81 bytes, not only 79.
    if message.framing_status != 1 or not 9 <= message.len <= 81:
        return None
    try:
        payload = b''.join(struct.pack('<Q', word) for word in message.payload64)[:message.len]
    except (TypeError, struct.error):
        return None
    if len(payload) < 9 or payload[6] != 10:
        return None
    count = payload[8]
    if count > 70 or len(payload) < 9 + count:
        return None
    return payload[9:9 + count].decode('utf-8', errors='replace')


def prompt_complete(text):
    return PROMPT.search(ANSI.sub('', text).replace('\x00', '')) is not None


def ground_ready(state, extended, now, freshness=2.5):
    """state/extended are (message, monotonic receipt time), never ROS time."""
    if state is None or extended is None:
        return False, 'missing_state_or_extended_state'
    for name, record in (('state', state), ('extended_state', extended)):
        age = now - record[1]
        if not math.isfinite(age) or not 0 <= age <= freshness:
            return False, 'stale_' + name
    if not state[0].connected:
        return False, 'not_connected'
    if state[0].armed:
        return False, 'armed'
    if extended[0].landed_state != 1:  # mavros_msgs/ExtendedState.ON_GROUND
        return False, 'not_on_ground'
    return True, ''


def parse_listener(raw, topic):
    """Parse one complete listener response; preserve missing/duplicate fields."""
    text = ANSI.sub('', raw).replace('\r', '')
    if UNSUPPORTED.search(text):
        return dict(status='unsupported', fields={}, duplicate_fields=[],
                    availability=('not_published' if re.search(
                        r'never published|not (?:yet )?published|not advertised', text, re.I) else 'unknown'),
                    timestamp_age_s=None)
    fields, duplicates = {}, []
    for line in text.splitlines():
        match = re.match(r'^\s*([A-Za-z_][A-Za-z_0-9]*)\s*:\s*(.*?)\s*$', line)
        if not match or match[1] == 'TOPIC':
            continue
        key, value = match.groups()
        token = value.split()[0] if value else ''
        if token.lower() in ('true', 'false'):
            parsed = token.lower() == 'true'
        elif re.fullmatch(r'[-+]?\d+', token):
            parsed = int(token)
        else:
            try:
                parsed = float(token)
                if not math.isfinite(parsed):
                    parsed = value
            except ValueError:
                parsed = value
        if key in fields:
            duplicates.append(key)
        fields[key] = parsed
    advertised = re.search(r'\bTOPIC:\s*' + re.escape(topic) + r'\b', text) is not None
    age = re.search(r'^\s*timestamp:\s*\d+\s*\(([\d.eE+-]+) seconds? ago\)', text, re.M)
    return dict(status='ok' if advertised and not duplicates else 'incomplete',
                fields=fields, duplicate_fields=duplicates,
                availability='published' if advertised else 'unknown',
                timestamp_age_s=float(age[1]) if age else None)


def boolean_field(fields, key):
    value = fields.get(key)
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    return None


def parse_ekf_status(raw):
    """Find the active instance without assuming listener instance 0 is primary."""
    text = ANSI.sub('', raw).replace('\r', '')
    instances = sorted(set(int(value) for value in re.findall(r'\bekf2:(\d+) EKF dt:', text)))
    selected = re.findall(r'^.*?\b(\d+): ACC:.*\*\s*$', text, re.M)
    selector_present = 'available instances:' in text
    primary = int(selected[0]) if len(selected) == 1 else (
        0 if not selector_present and instances == [0] else None)
    return dict(primary_instance=primary, instances=instances, selector_present=selector_present)


def auxiliary_velocity_evidence(sample, estimator_instance):
    """Missing aux flags are not evidence; inspect the input and aid topics."""
    source = sample.get('landing_target_pose', {})
    aid = sample.get('estimator_aid_src_aux_vel', {})
    source_parsed, aid_parsed = source.get('parsed', {}), aid.get('parsed', {})
    if source.get('status') != 'complete' or aid.get('status') != 'complete':
        return dict(excluded=None, reason='missing_or_incomplete_auxiliary_topic_snapshots')
    if (source_parsed.get('availability') == 'not_published'
            and aid_parsed.get('availability') == 'not_published'):
        return dict(excluded=True, reason='landing_target_input_and_auxiliary_aid_never_published')
    fields = aid_parsed.get('fields', {})
    age = aid_parsed.get('timestamp_age_s')
    instance = fields.get('estimator_instance')
    if (aid_parsed.get('status') == 'ok' and age is not None and math.isfinite(age)
            and 0 <= age <= 2.0 and isinstance(instance, int) and not isinstance(instance, bool)
            and instance == estimator_instance):
        fused = boolean_field(fields, 'fused')
        if fused is not None:
            return dict(excluded=not fused, reason='fresh_auxiliary_aid_fused_flag')
    return dict(excluded=None, reason='auxiliary_velocity_fusion_unknown')


def evaluate_samples(samples, requested_samples, estimator_instance=0):
    """Distinguish observed EV fusion, known source flags, and unobserved sources."""
    issues, checks, timestamps = [], [], {topic: [] for topic in TOPICS}
    sample_stamps = {topic: [] for topic in TOPICS[1:]}
    fuse_stamps = {topic: [] for topic in TOPICS[1:]}
    if len(samples) != requested_samples or requested_samples < 2:
        issues.append('at_least_two_complete_samples_required')
    ev_ok, flags_ok = bool(samples), bool(samples)
    for index, sample in enumerate(samples):
        observed = {'sample': index + 1, 'flags': {}, 'aid_sources': {}}
        flag_result = sample.get(TOPICS[0], {}).get('parsed', {})
        flags = flag_result.get('fields', {})
        if any(boolean_field(flags, key) is not True for key in ('cs_ev_pos', 'cs_ev_hgt')):
            ev_ok = False
        for key, expected in EXPECTED_FLAGS.items():
            actual = boolean_field(flags, key)
            observed['flags'][key] = actual
            if actual is not expected:
                flags_ok = False
                issues.append('sample_%s:%s=%s_expected_%s' % (index + 1, key, actual, expected))
        observed['additional_flags'] = {key: boolean_field(flags, key) for key in ADDITIONAL_FLAGS}
        observed['auxiliary_velocity'] = auxiliary_velocity_evidence(sample, estimator_instance)
        for topic in TOPICS:
            record = sample.get(topic, {})
            parsed = record.get('parsed', {})
            fields = parsed.get('fields', {})
            timestamp = fields.get('timestamp')
            if record.get('status') != 'complete' or parsed.get('status') != 'ok':
                issues.append('sample_%s:%s_unavailable' % (index + 1, topic))
                if topic == TOPICS[0]:
                    flags_ok = False
                else:
                    ev_ok = False
            if isinstance(timestamp, int) and not isinstance(timestamp, bool) and timestamp > 0:
                timestamps[topic].append(timestamp)
            else:
                issues.append('sample_%s:%s_missing_timestamp' % (index + 1, topic))
                if topic == TOPICS[0]:
                    flags_ok = False
                else:
                    ev_ok = False
            age = parsed.get('timestamp_age_s')
            if age is None or not math.isfinite(age) or not 0 <= age <= 2.0:
                issues.append('sample_%s:%s_timestamp_freshness_unknown_or_stale' % (index + 1, topic))
                if topic == TOPICS[0]:
                    flags_ok = False
                else:
                    ev_ok = False
            if topic in TOPICS[1:]:
                fused = boolean_field(fields, 'fused')
                rejected = boolean_field(fields, 'innovation_rejected')
                last_fuse = fields.get('time_last_fuse')
                instance = fields.get('estimator_instance')
                if (not isinstance(instance, int) or isinstance(instance, bool)
                        or instance != estimator_instance):
                    ev_ok = False
                    issues.append('sample_%s:%s_estimator_instance_unknown_or_mismatched' % (index + 1, topic))
                for collection, field in ((sample_stamps, 'timestamp_sample'),
                                          (fuse_stamps, 'time_last_fuse')):
                    stamp = fields.get(field)
                    if isinstance(stamp, int) and not isinstance(stamp, bool) and stamp > 0:
                        collection[topic].append(stamp)
                recent_fusion = (isinstance(last_fuse, int) and not isinstance(last_fuse, bool)
                                 and isinstance(timestamp, int) and 0 < last_fuse <= timestamp
                                 and timestamp - last_fuse <= 500000)
                observed['aid_sources'][topic] = dict(fused=fused, innovation_rejected=rejected,
                                                      recent_fusion=recent_fusion)
                if fused is not True or rejected is not False or not recent_fusion:
                    ev_ok = False
                    issues.append('sample_%s:%s_not_fusing_accepted_recent_measurement' % (index + 1, topic))
        checks.append(observed)
    advancing = {topic: len(values) == requested_samples and len(values) >= 2
                 and all(b > a for a, b in zip(values, values[1:]))
                 for topic, values in timestamps.items()}
    for topic, valid in advancing.items():
        if not valid:
            issues.append(topic + ':timestamps_not_advancing')
    sample_advancing = {topic: len(values) == requested_samples and len(values) >= 2
                        and all(b > a for a, b in zip(values, values[1:]))
                        for topic, values in sample_stamps.items()}
    fusion_advancing = {topic: len(values) == requested_samples and len(values) >= 2
                        and all(b > a for a, b in zip(values, values[1:]))
                        for topic, values in fuse_stamps.items()}
    for name, collection in (('sample', sample_advancing), ('fusion', fusion_advancing)):
        for topic, valid in collection.items():
            if not valid:
                ev_ok = False
                issues.append(topic + ':' + name + '_timestamps_not_advancing')
    # Aux velocity is not present in v1.17 flags. Exclusion needs independent
    # input/aid snapshots; an absent flag itself remains explicitly unknown.
    aux_values = []
    for check in checks:
        flag = check['additional_flags']['cs_aux_vel']
        excluded = check['auxiliary_velocity']['excluded']
        aux_values.append(flag if flag is not None else None if excluded is None else not excluded)
    auxiliary_velocity_excluded = (False if any(value is True for value in aux_values)
                                   else None if not aux_values or any(value is None for value in aux_values)
                                   else True)
    if auxiliary_velocity_excluded is False:
        issues.append('auxiliary_velocity_fusion_observed')
    exclusion_values = [check['flags'][key] for check in checks
                        for key, expected in EXPECTED_FLAGS.items() if not expected] + aux_values
    all_excluded = (False if any(value is True for value in exclusion_values)
                    else None if not exclusion_values or any(value is None for value in exclusion_values)
                    else True)
    return dict(passed=not issues,
                passed_means='ground_snapshot_ev_position_height_fusion_and_listed_v1_17_flags',
                listener_estimator_instance=estimator_instance,
                ev_position_height_fusion_observed=ev_ok and all(advancing[t] for t in TOPICS[1:]),
                expected_px4_1_17_flags_observed=flags_ok,
                auxiliary_velocity_excluded=auxiliary_velocity_excluded,
                all_non_lio_sources_excluded=all_excluded,
                timestamps_advancing=advancing, sample_timestamps_advancing=sample_advancing,
                fusion_timestamps_advancing=fusion_advancing, sample_checks=checks, issues=issues,
                realtime_fusion_monitor=False,
                limitation=('Sequential ground snapshots only. Unreported sources are unknown. '
                            'Does not prove continuous fusion, independent position accuracy, '
                            'flight performance or failsafe behavior.'))


class GroundShell:
    def __init__(self, rospy, message_type, state_type, extended_type, timeout, target_system):
        self.rospy, self.message_type = rospy, message_type
        self.timeout, self.target_system = timeout, target_system
        self.condition = threading.Condition()
        self.state = self.extended = None
        self.buffer, self.waiting, self.seq = '', False, 0
        self.serial_responses = []
        # Synchronous TCPROS writes prevent close() from dropping a release
        # still queued in rospy. Completion is not an FCU acknowledgement.
        self.pub = rospy.Publisher('/mavlink/to', message_type, queue_size=None)
        self.subs = [rospy.Subscriber('/mavlink/from', message_type, self.receive, queue_size=1000),
                     rospy.Subscriber('/mavros/state', state_type, self.state_callback, queue_size=10),
                     rospy.Subscriber('/mavros/extended_state', extended_type, self.extended_callback,
                                      queue_size=10)]

    def state_callback(self, message):
        with self.condition:
            self.state = (message, time.monotonic())
            self.condition.notify_all()

    def extended_callback(self, message):
        with self.condition:
            self.extended = (message, time.monotonic())
            self.condition.notify_all()

    def receive(self, message):
        if message.msgid == 126:
            with self.condition:
                self.serial_responses.append(dict(sysid=message.sysid, compid=message.compid,
                                                  length=message.len, framing_status=message.framing_status))
                self.serial_responses = self.serial_responses[-100:]
        data = serial_control_text(message, self.target_system)
        if data is not None:
            with self.condition:
                if self.waiting:
                    self.buffer += data
                    self.condition.notify_all()

    def send(self, data, release=False):
        fields = serial_control_fields(data, self.seq, release=release)
        self.seq = (self.seq + 1) & 255
        message = self.message_type()
        for key, value in fields.items():
            setattr(message, key, value)
        message.header.stamp = self.rospy.Time.now()
        self.pub.publish(message)

    def ready(self):
        return ground_ready(self.state, self.extended, time.monotonic())

    def await_ground(self):
        until = time.monotonic() + self.timeout
        with self.condition:
            while not self.rospy.is_shutdown() and time.monotonic() < until:
                ready, reason = self.ready()
                if ready and self.pub.get_num_connections() > 0:
                    return
                self.condition.wait(.05)
        raise RuntimeError('ground_precondition_failed:' + self.ready()[1])

    def run(self, command):
        if command is not None and command not in ALLOWED_COMMANDS:
            raise ValueError('command not in read-only diagnostic allowlist')
        with self.condition:
            ready, reason = self.ready()
            if not ready:
                raise RuntimeError('ground_precondition_failed:' + reason)
            self.buffer, self.waiting = '', True
            started = time.monotonic()
            # A bare newline acquires the shell and synchronizes its prompt.
            self.send(((command or '') + '\n').encode('ascii'))
            status, error = 'timeout', ''
            try:
                while not self.rospy.is_shutdown() and time.monotonic() - started < self.timeout:
                    ready, reason = self.ready()
                    if not ready:
                        status, error = 'aborted', reason
                        break
                    if len(self.buffer) > 262144:
                        status, error = 'aborted', 'response_too_large'
                        break
                    if prompt_complete(self.buffer):
                        status = 'complete'
                        break
                    self.condition.wait(.05)
                return dict(command=command or '<prompt synchronization>', status=status,
                            error=error, elapsed_s=time.monotonic() - started, raw=self.buffer)
            finally:
                self.waiting = False

    def close(self):
        # Empty flags=0 releases SERIAL_CONTROL ownership, even after a timeout
        # or a changed ground state. No shell command is issued during release.
        self.send(b'', release=True)
        for sub in self.subs:
            sub.unregister()
        self.pub.unregister()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--samples', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=12., help='per-command timeout, 8..60 seconds')
    parser.add_argument('--interval', type=float, default=1.1)
    parser.add_argument('--target-system', type=int, default=1)
    args = parser.parse_args()
    if not 2 <= args.samples <= 20:
        parser.error('--samples must be between 2 and 20')
    if not math.isfinite(args.timeout) or not 8. <= args.timeout <= 60.:
        parser.error('--timeout must be between 8 and 60 seconds')
    if not math.isfinite(args.interval) or not 0. <= args.interval <= 10.:
        parser.error('--interval must be between 0 and 10 seconds')
    if not 1 <= args.target_system <= 255:
        parser.error('--target-system must be between 1 and 255')
    import rospy
    from mavros_msgs.msg import Mavlink, State, ExtendedState
    rospy.init_node('native_ekf_snapshot', anonymous=True)
    shell = GroundShell(rospy, Mavlink, State, ExtendedState, args.timeout, args.target_system)
    result = dict(schema_version=1, collected_at=datetime.now(timezone.utc).isoformat(),
                  requested_samples=args.samples, transport='/mavlink/to,/mavlink/from SERIAL_CONTROL',
                  target_system=args.target_system, samples=[], information=[], error='',
                  flight_or_payload_commands_sent=False, parameters_modified=False)
    try:
        shell.await_ground()
        result['prompt_synchronization'] = shell.run(None)
        if result['prompt_synchronization']['status'] != 'complete':
            raise RuntimeError('shell_prompt_synchronization_failed')
        for command in ('ver all', 'ekf2 status'):
            record = shell.run(command)
            result['information'].append(record)
            if record['status'] != 'complete':
                raise RuntimeError('shell_command_' + record['status'])
        primary = parse_ekf_status(result['information'][-1]['raw'])['primary_instance']
        result['listener_estimator_instance'] = primary
        if primary is None or not 0 <= primary < 16:
            raise RuntimeError('active_estimator_instance_unknown_or_unsupported')
        for index in range(args.samples):
            sample = {}
            result['samples'].append(sample)
            for topic in TOPICS + EXCLUSION_TOPICS:
                command = 'listener %s -n 1 -i %s' % (topic, 0 if topic == 'landing_target_pose' else primary)
                record = shell.run(command)
                record['parsed'] = parse_listener(record['raw'], topic)
                sample[topic] = record
                if record['status'] != 'complete':
                    raise RuntimeError('shell_command_' + record['status'])
            if index + 1 < args.samples:
                time.sleep(args.interval)
        record = shell.run('ekf2 status')
        result['information'].append(record)
        if record['status'] != 'complete':
            raise RuntimeError('shell_command_' + record['status'])
    except (RuntimeError, KeyboardInterrupt, rospy.ROSException) as exc:
        result['error'] = str(exc) or type(exc).__name__
    finally:
        try:
            shell.close()
            result['shell_release_requested'] = True
        except rospy.ROSException as exc:
            result['shell_release_requested'] = False
            result['error'] = result['error'] or ('shell_release_failed:' + str(exc))
    result['evidence'] = evaluate_samples(result['samples'], args.samples,
                                           result.get('listener_estimator_instance'))
    result['serial_response_headers'] = shell.serial_responses
    statuses = [parse_ekf_status(record['raw']) for record in result['information']
                if record['command'] == 'ekf2 status' and record['status'] == 'complete']
    primary = result.get('listener_estimator_instance')
    primary_matches = (primary is not None and len(statuses) == 2
                       and all(status['primary_instance'] == primary for status in statuses))
    result['evidence']['primary_estimator_instance_checks'] = statuses
    result['evidence']['primary_instance_unchanged_before_and_after'] = primary_matches
    if not primary_matches:
        result['evidence']['passed'] = False
        result['evidence']['issues'].append('primary_instance_not_verified_unchanged_before_and_after')
    if result['error']:
        result['evidence']['passed'] = False
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(dict(output=str(args.output), error=result['error'],
                          evidence=result['evidence']), ensure_ascii=False))
    return 0 if result['evidence']['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
