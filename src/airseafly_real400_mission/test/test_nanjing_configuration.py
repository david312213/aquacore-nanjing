"""Public parameter migration and entry presets, without any ROS connection."""
import unittest
from airseafly_real400_mission.flight_configuration import base_configuration, apply_flight_configuration
from airseafly_real400_mission.native_parameters import summary
from airseafly_real400_mission.map_tuning import ego_launch_arguments

class NanjingConfigurationTests(unittest.TestCase):
    def test_basic_and_competition_preserve_effective_limits(self):
        for task, heights, ceiling in [('takeoff', (1.,1.,.2),1.4),
                ('waypoint-route',(1.,1.,.2),1.4),('delivery-route',(1.4,1.6,.7),2.4)]:
            config = apply_flight_configuration(base_configuration(), task=task)
            values = summary(config)
            self.assertEqual(tuple(values[k] for k in ('height_m','gate_height_m','drop_height_m')), heights)
            self.assertEqual(tuple(values[k] for k in ('takeoff_speed_mps','takeoff_acceleration_mps2','ego_speed_mps','ego_acceleration_mps2')), (.5,.25,.5,.25))
            self.assertEqual(values['map']['max_height_m'],ceiling)
            self.assertLess(max(heights[:2]), ceiling)
            self.assertEqual([values['map']['size_'+a+'_m'] for a in 'xyz'], [100.,50.,3.])
            self.assertFalse(config['sensors']['cameras_enabled'])
            self.assertFalse(config['recording']['enabled'])
            self.assertFalse(any(key in config for key in ('front','down','search','h_landing','corridor')))
    def test_override_reaches_planner_without_mutating_next_task(self):
        config = apply_flight_configuration(base_configuration(), task='waypoint-route', overrides=[
            'flight.cruise_height_m=1.2','planning.max_velocity_mps=0.4','planning.map.size_x_m=40',
            'motion.native_acceleration_mps2=0.3'])
        self.assertEqual(summary(config)['height_m'],1.2)
        self.assertEqual(summary(config)['ego_speed_mps'],.4)
        self.assertIn('map_size_x:=40',ego_launch_arguments(config))
        self.assertEqual(summary(config)['align_acceleration_mps2'],.3)
        next_config=apply_flight_configuration(base_configuration(), task='waypoint-route')
        self.assertEqual(summary(next_config)['height_m'],1.)
    def test_competition_clears_original_slot_override(self):
        machine=base_configuration()
        machine['payload']['released_pwm_us_by_slot']={'2':2020}
        config=apply_flight_configuration(machine,task='delivery-route')
        self.assertEqual(config['payload']['released_pwm_us_by_slot'],{'1':1190,'2':1190,'3':1350,'4':1350})
        self.assertEqual(config['payload']['sequence'],[1,2,3,4])
