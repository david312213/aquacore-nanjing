"""Onboard ROS advertise address; mirrors scripts/ros_network.sh."""
import socket


def _resolves_to_loopback(host):
    try:
        address = socket.getaddrinfo(host, None, socket.AF_INET)[0][4][0]
    except (OSError, IndexError):
        return False
    return address.startswith('127.')


def apply_ros_address(env):
    """Set exactly one of ROS_IP / ROS_HOSTNAME in ``env`` (a dict or os.environ).

    By default the machine is advertised by hostname. /etc/hosts maps it to
    127.0.1.1 locally, so onboard links stay on loopback and survive WiFi drops
    or IP changes, while a remote RViz maps the same name to the current LAN
    address. AIRSEAFLY_ROS_IP pins an explicit address (127.0.0.1 = local only).
    """
    env.pop('ROS_IP', None)
    env.pop('ROS_HOSTNAME', None)
    pinned = env.get('AIRSEAFLY_ROS_IP')
    if pinned:
        env['ROS_IP'] = pinned
        return env
    host = socket.gethostname()
    if host and _resolves_to_loopback(host):
        env['ROS_HOSTNAME'] = host
    else:
        env['ROS_IP'] = '127.0.0.1'
    return env
