"""Fail-closed checks of bounded PX4 listener snapshots; no ROS dependency."""
import re


def verify_ev_state(records, instance, enabled):
    """Require a fresh selected-EKF snapshot, not a stale aid-source topic.

    This confirms fusion stopped/started, not an unobservable private filter
    state. The disarmed harness additionally enforces a no-data timeout while
    the EV configuration remains enabled before declaring a clean restart.
    """
    by_command = {record['command']: record['output'] for record in records}
    selector = by_command['listener estimator_selector_status -n 1']
    status = by_command['listener estimator_status_flags -n 1 -i %d' % instance]
    for output in (selector, status):
        age = re.search(r'^\s+timestamp: \d+ \(([0-9.]+) seconds ago\)', output, re.M)
        if not age or float(age.group(1)) > 2.0:
            raise RuntimeError('Missing or stale selected-EKF status snapshot')
    primary = re.search(r'^\s+primary_instance: (\d+)\s*$', selector, re.M)
    if not primary or int(primary.group(1)) != instance:
        raise RuntimeError('Selected EKF instance changed; bench session stopped')
    flags = {}
    for name in ('cs_ev_pos', 'cs_ev_hgt', 'cs_ev_yaw', 'cs_ev_vel'):
        match = re.search(r'^\s+' + name + r': (True|False)\s*$', status, re.M)
        if not match:
            raise RuntimeError('Missing EV status flag: ' + name)
        flags[name] = match.group(1) == 'True'
    if flags != dict(cs_ev_pos=enabled, cs_ev_hgt=enabled,
                     cs_ev_yaw=False, cs_ev_vel=False):
        raise RuntimeError('Unexpected EV fusion state: %r' % flags)
    return flags
