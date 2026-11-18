"""Read-only PD trace schema and flight lifecycle; no ROS or actuation imports."""
import math

TRACE_SIZE = 62
VECTOR_FIELDS = {
    'desired_position': (6, 9), 'position': (9, 12),
    'desired_velocity': (12, 15), 'velocity': (15, 18), 'feedforward': (18, 21),
    'kp': (21, 24), 'kv': (24, 27), 'p_term': (27, 30), 'd_term': (30, 33),
    'gravity': (33, 36), 'acceleration': (36, 39),
    'desired_quaternion': (39, 43), 'quaternion': (43, 47),
    'odom_quaternion': (52, 56), 'angular_velocity': (56, 59), 'imu_acceleration': (59, 62),
}
SCALAR_FIELDS = dict(stamp=1, odom_stamp=2, odom_received=3, imu_stamp=4, imu_received=5,
                     thrust=47, gravity_constant=48, thrust_mapping=49,
                     loaded_hover_percentage=50, desired_yaw=51)
CONTROL_STATES = ('AUTO_TAKEOFF', 'AUTO_HOVER', 'CMD_CTRL', 'AUTO_LAND')


def decode_trace(values):
    if len(values) != TRACE_SIZE or values[0] != 1:
        raise ValueError('unsupported PD trace schema')
    if not all(math.isfinite(x) for x in values):
        raise ValueError('nonfinite PD trace')
    result = {k: list(values[a:b]) for k, (a, b) in VECTOR_FIELDS.items()}
    result.update({k: values[i] for k, i in SCALAR_FIELDS.items()})
    result['schema'] = 1
    return result


def sample_problem(row):
    """Inputs are the exact values used by the controller, with source ages."""
    if row.get('control_state') not in CONTROL_STATES:
        return 'not_automatic_control'
    if not row.get('armed') or not row.get('connected') or row.get('fcu_mode') != 'OFFBOARD':
        return 'not_armed_offboard'
    t = row['stamp']
    for key in ('odom_stamp', 'odom_received', 'imu_stamp', 'imu_received'):
        if row[key] <= 0 or not 0 <= t-row[key] <= .20:
            return 'stale_controller_input'
    if not 0 <= row['received_ros']-t <= .25:
        return 'stale_trace'
    for key, limit in (('control_age', .30), ('state_age', 2.0)):
        if not 0 <= row.get(key, float('inf')) <= limit:
            return 'stale_mode'
    if any(v < 0 for v in row['kp'] + row['kv']):
        return 'invalid_gains'
    for key in ('quaternion', 'desired_quaternion'):
        norm = sum(x*x for x in row[key])**.5
        if not .8 <= norm <= 1.2:
            return 'invalid_attitude'
    return None


class Lifecycle:
    """Start with fresh arming; end only with fresh ground AND disarmed state."""
    def __init__(self):
        self.values = {}
        self.active = False
        self.ground_since = None

    def observe(self, key, value, now):
        self.values[key] = (value, now)

    def fresh(self, key, now):
        value, stamp = self.values.get(key, (None, -float('inf')))
        return value if 0 <= now-stamp <= 2.0 else None

    def update(self, now):
        armed = self.fresh('armed', now)
        control = self.fresh('control', now)
        if not self.active:
            if armed is True and control in CONTROL_STATES:
                self.active = True
                self.ground_since = None
                return 'start'
            return None
        ground = self.fresh('landed', now) is True or self.fresh('extended', now) == 1
        if armed is False and ground:
            if self.ground_since is None:
                self.ground_since = now
            if now-self.ground_since >= .5:
                self.active = False
                self.ground_since = None
                return 'finish'
        else:
            self.ground_since = None
        return None
