"""Onboard ROS advertise address: hostname on loopback, else explicit IP."""
import unittest
from unittest import mock

from airseafly_real400_mission import ros_network


def resolving(address):
    return mock.patch.object(ros_network.socket, 'getaddrinfo',
                             return_value=[(2, 1, 6, '', (address, 0))])


class RosAddressTests(unittest.TestCase):
    def setUp(self):
        patcher = mock.patch.object(ros_network.socket, 'gethostname', return_value='drone-test')
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_loopback_hostname_is_advertised_by_name(self):
        with resolving('127.0.1.1'):
            env = ros_network.apply_ros_address(dict(ROS_IP='192.0.2.10', ROS_HOSTNAME='old-host'))
        self.assertEqual(env, dict(ROS_HOSTNAME='drone-test'))

    def test_hostname_off_loopback_falls_back_to_local_ip(self):
        with resolving('172.18.9.122'):
            env = ros_network.apply_ros_address(dict(ROS_HOSTNAME='old-host'))
        self.assertEqual(env, dict(ROS_IP='127.0.0.1'))

    def test_unresolvable_hostname_falls_back_to_local_ip(self):
        with mock.patch.object(ros_network.socket, 'getaddrinfo', side_effect=OSError):
            env = ros_network.apply_ros_address({})
        self.assertEqual(env, dict(ROS_IP='127.0.0.1'))

    def test_pinned_address_wins(self):
        with resolving('127.0.1.1'):
            env = ros_network.apply_ros_address(dict(AIRSEAFLY_ROS_IP='127.0.0.1', ROS_HOSTNAME='x'))
        self.assertEqual(env, dict(AIRSEAFLY_ROS_IP='127.0.0.1', ROS_IP='127.0.0.1'))


if __name__ == '__main__':
    unittest.main()
