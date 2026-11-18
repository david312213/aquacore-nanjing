"""Climb-to-navigation regression replays; no ROS endpoints or aircraft."""
import unittest

from test_native_delivery_route import Replay
from test_single_drop import SingleDropReplay
from test_drop_points import DropPointsReplay


class DeliveryHandoffTests(unittest.TestCase):
    def test_takeoff_residual_never_replaces_completed_reference_in_any_delivery_entry(self):
        for factory in (Replay, SingleDropReplay, DropPointsReplay):
            with self.subTest(entry=factory.__name__):
                h = factory()
                e = h.engine
                h.step(follow=False)  # Start the climb's reference clock.
                h.position = (h.ground[0]+.075, h.ground[1]+.111, e.cruise_z-.212)
                h.now = e._native_takeoff_motion.started + e._native_takeoff_motion.duration
                output = h.step(dt=.01, follow=False)
                self.assertEqual((e.state, e.phase), ('APPROACH', 'WAIT_EGO'))
                self.assertEqual(output['native_command']['position'], (*h.ground[:2], e.cruise_z))
                self.assertEqual(output['native_command']['velocity'], (0., 0., 0.))
                self.assertEqual(e.drops_completed, 0)
                # Existing readiness/receipt steps still dispatch the real
                # waypoint, without waiting for the residual to disappear.
                output = h.step(follow=False)
                goals = [x for x in output['effects'] if x['type'] == 'goal']
                self.assertEqual(len(goals), 1)
                self.assertEqual(goals[0]['position_xyz'], list(e.command_position))
                self.assertEqual(output['native_command']['position'][2], e.cruise_z)

    def test_each_recovery_preserves_height_for_next_target_and_return_home(self):
        for count in range(1, 5):
            with self.subTest(count=count):
                h = DropPointsReplay(count)
                e = h.engine
                for point in range(count):
                    h.until(lambda e: e.route_index == point and e.phase == 'NATIVE_RECOVER')
                    target = e.native_goal
                    h.position = (target[0]+.04, target[1]-.03, target[2]-.212)
                    h.now = e.native_motion.started + e.native_motion.duration
                    output = h.step(dt=.01, follow=False)
                    self.assertEqual(e.state, 'APPROACH')
                    self.assertEqual(output['native_command']['position'], (*h.position[:2], target[2]))
                    self.assertEqual(output['native_command']['velocity'], (0., 0., 0.))
                    self.assertEqual(e.drops_completed, point+1)
                    self.assertEqual(e.route_returning_home, point == count-1)
                self.assertEqual(e.route_released_slots, list(range(1, count+1)))

    def test_navigation_height_changes_keep_existing_actual_position_wait_reference(self):
        h = Replay()
        e = h.engine
        for previous_role, next_role in (('cruise', 'gate'), ('gate', 'cruise')):
            h.until(lambda e: e.state == 'APPROACH' and
                    e.itinerary[e.itinerary_index]['height_role'] == previous_role and
                    e.itinerary[e.itinerary_index+1]['height_role'] == next_role)
            previous_index = e.itinerary_index
            # The established arrival tolerance can accept a residual. A
            # normal route transition must not turn the old nominal height
            # into a new immediate height command during EGO preparation.
            h.position = (*e.command_position[:2], e.command_position[2]-.05)
            h.until(lambda e: e.itinerary_index != previous_index, follow=False)
            self.assertEqual(e._navigation_hold, h.position)
            height = e.config['flight']['gate_height_m'] if next_role == 'gate' else 1.4
            self.assertEqual(e.command_position[2], h.ground[2]+height)


if __name__ == '__main__':
    unittest.main()
