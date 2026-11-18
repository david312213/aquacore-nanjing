"""Delivery mission with fixed legs between stops. No ROS, services or aircraft."""
import math
import unittest
from unittest.mock import patch

import numpy as np

from test_native_delivery_route import Replay, configuration, route_document
from airseafly_real400_mission import native_delivery_route
from airseafly_real400_mission.native_delivery_route import DeliveryRouteEngine, build_itinerary, plan_fixed_legs
from airseafly_real400_mission.guided_route import validate_route
from airseafly_real400_mission.delivery_route_cli import preview


def free_document():
    """Free recording: starts at an ordinary point, two drops, one ring pass."""
    route = [dict(kind='transit', xy=[0., 0.]), dict(kind='drop', xy=[3., 0.], drop_slot=1),
             dict(kind='gate', xy=[4., 1.]), dict(kind='gate', xy=[5., 1.]),
             dict(kind='transit', xy=[6., 0.]), dict(kind='drop', xy=[6., 3.], drop_slot=2),
             dict(kind='transit', xy=[3., 4.])]
    return validate_route(dict(version=2, frame='fast_lio_local_enu', mode='free', home=[-4., -5.],
                               route=route), require_complete=True)


def fixed_configuration(gate=None):
    config = configuration()
    config['navigation'] = dict(backend='fixed_min_snap')
    config['planning']['gate_velocity_mps'] = gate
    return config


class FixedReplay(Replay):
    """Follows the fixed reference like the runtime timer, optionally offset.

    engine_class/mode select a drop test; start_xyz is where the task starts,
    which may differ from the corrected ground origin.
    """
    def __init__(self, config=None, engine_class=None, mode='delivery_route', start_xyz=None):
        self.offset, self.pin, self.legs = (0., 0., 0.), None, []
        self._leg, self._leg_started = None, None
        if engine_class is None:
            return super().__init__(config or fixed_configuration())
        self.engine = engine_class(config)
        self.ground, self.now = (12., -8., -1.3), 100.
        self.position, self.velocity, self.yaw = self.ground, (0., 0., 0.), .2
        self.armed, self.mode, self.owner = True, 'OFFBOARD', 'PX4_NATIVE'
        self.effects, self.groups, self.image_sequence = [], [], 0
        self.engine.start(mode, start_xyz or self.ground, self.yaw, self.now)
        self.engine.set_red_point_session('offline-red-point')
        self.engine.native_origin = self.ground
        self.feedback()
        self.engine.begin_native_flight()

    def step(self, dt=.1, follow=True, acknowledge=True):
        e = self.engine
        if e.fixed_trajectory is not None and e.phase in ('FIXED_TRAJECTORY', 'FIXED_ARRIVAL'):
            if e.fixed_trajectory is not self._leg:
                self._leg, self._leg_started = e.fixed_trajectory, self.now
                self.legs.append(e.fixed_trajectory)
            sample = dict(self._leg.sample(min(self._leg.duration, self.now+dt-self._leg_started)))
            sample['actual_position_xyz'] = list(self.position)
            e.on_fixed_progress(sample)
            if follow:
                reference = self._leg.sample(min(self._leg.duration, self.now+dt-self._leg_started))
                self.position = self.pin or tuple(a+b for a, b in zip(reference['position'], self.offset))
                self.velocity = reference['velocity']
        return super().step(dt, follow, acknowledge)


class FixedDeliveryRouteTests(unittest.TestCase):
    def test_four_drops_and_landing_without_ego(self):
        h = FixedReplay()
        # Fixed legs are followed in real time, unlike the EGO replay's jump.
        h.until(lambda e: e.phase == 'LANDING', limit=40000)
        e = h.engine
        self.assertEqual([v['slot'] for v in h.effects if v['type'] == 'release'], [1, 2, 3, 4])
        self.assertEqual((e.completed_laps, e.drops_completed), (2, 4))
        self.assertEqual(set(h.groups), {'NONE'})
        self.assertFalse([v for v in h.effects if v['type'] == 'goal'])
        # Takeoff -> drop1, drop1 -> drop2, -> drop3, -> drop4, drop4 -> HOME.
        self.assertEqual(len(h.legs), 5)
        arrived = [v['stage'] for v in h.effects if v['type'] == 'delivery_route_point_arrived']
        self.assertEqual(arrived, ['drop1', 'drop2', 'drop3', 'drop4', 'home'])
        laps = [v['completed_laps'] for v in h.effects if v['type'] == 'delivery_route_lap_complete']
        self.assertEqual(laps, [1, 2])
        events = [v['type'] for v in h.effects]
        self.assertLess(max(i for i, v in enumerate(events) if v == 'delivery_route_lap_complete'),
                        events.index('delivery_route_home_arrived'))
        points = [v for v in h.effects if v['type'] == 'delivery_route_point']
        expected = [s for s in e.itinerary if s['kind'] == 'navigation']
        self.assertEqual([v['position_xyz'][:2] for v in points], [s['xy'] for s in expected])
        for actual, step in zip(points, expected):
            self.assertAlmostEqual(actual['position_xyz'][2], h.ground[2]+(1.6 if step['height_role'] == 'gate' else 1.4))
        # Each later leg starts at the post-drop recovery endpoint, at cruise height.
        for leg, slot in zip(h.legs[1:], range(1, 5)):
            start = leg.sample(0.)['position']
            drop = next(p for p in e.route_points if p.get('drop_slot') == slot)
            self.assertLess(math.dist(start[:2], drop['xy']), .05)
            self.assertAlmostEqual(start[2], h.ground[2]+1.4, places=6)
        self.assertEqual(h.legs[-1].end[:2], tuple(e.route_home))
        h.armed = False
        h.step()
        self.assertEqual((e.done, e.phase), (True, 'LANDED'))

    def test_leg_end_waits_for_the_existing_arrival_radius_and_dwell(self):
        h = FixedReplay()
        h.offset = (.5, 0., 0.)
        h.until(lambda e: e.phase == 'FIXED_ARRIVAL')
        for _ in range(10):
            h.step(dt=30.)
        self.assertEqual(h.engine.phase, 'FIXED_ARRIVAL')
        self.assertFalse([v for v in h.effects if v['type'] == 'delivery_route_point_arrived'])
        h.offset = (0., 0., 0.)
        for _ in range(3):
            h.step(dt=.3)
        self.assertEqual(h.engine.state, 'DROP')
        self.assertEqual([v['stage'] for v in h.effects if v['type'] == 'delivery_route_point_arrived'], ['drop1'])

    def home_arrival(self, offset):
        h = FixedReplay()
        h.until(lambda e: e.phase == 'FIXED_ARRIVAL' and e._fixed_leg_lands, limit=40000)
        h.offset = offset
        return h

    def test_home_needs_five_cm_horizontally_and_ignores_height(self):
        h = self.home_arrival((.03, .03, .15))  # 4.2 cm XY, 15 cm low: still an arrival.
        h.step(dt=.1)
        start = h.now
        for _ in range(10):
            h.step(dt=.1)
            if h.engine.phase != 'FIXED_ARRIVAL':
                break
        self.assertNotEqual(h.engine.phase, 'FIXED_ARRIVAL')
        self.assertLess(h.now-start, 1.)
        self.assertEqual([v['stage'] for v in h.effects if v['type'] == 'delivery_route_point_arrived'][-1], 'home')
        self.assertFalse([v for v in h.effects if v['type'] == 'home_arrival_timeout'])

    def test_home_outside_five_cm_lands_in_place_fifteen_seconds_after_the_reference(self):
        h = self.home_arrival((.06, 0., 0.))  # 6 cm: inside the old 0.2 m, outside the new 5 cm.
        e = h.engine
        entered = e.phase_started_at
        while h.now < entered+14.8:
            h.step(dt=.2)
        self.assertEqual(e.phase, 'FIXED_ARRIVAL')
        self.assertFalse([v for v in h.effects if v['type'] == 'delivery_route_home_arrived'])
        h.step(dt=.3)
        self.assertNotEqual(e.phase, 'FIXED_ARRIVAL')
        timeout = [v for v in h.effects if v['type'] == 'home_arrival_timeout']
        self.assertEqual(len(timeout), 1)
        self.assertAlmostEqual(timeout[0]['xy_error_m'], .06, places=6)
        self.assertEqual(timeout[0]['xy_tolerance_m'], .05)
        self.assertIn('delivery_route_home_arrived', [v['type'] for v in h.effects])
        # Landing starts from where it is, not from HOME: the H wait holds the actual XY.
        self.assertEqual(e.phase, 'LAND_FIRST_FRAME')
        self.assertAlmostEqual(e.native_goal[0], e.route_home[0]+.06, places=6)

    def test_drop_points_keep_the_existing_xyz_radius(self):
        h = FixedReplay()
        h.offset = (.06, 0., .08)  # 10 cm XYZ: inside 0.2 m, so a drop point still arrives.
        h.until(lambda e: e.phase == 'FIXED_ARRIVAL')
        self.assertFalse(h.engine._fixed_leg_lands)
        for _ in range(8):
            h.step(dt=.1)
        self.assertEqual(h.engine.state, 'DROP')
        self.assertFalse([v for v in h.effects if v['type'] == 'home_arrival_timeout'])

    def test_passing_the_leg_end_mid_leg_is_not_an_arrival(self):
        h = FixedReplay()
        h.until(lambda e: e.phase == 'FIXED_TRAJECTORY')
        leg = h.engine.fixed_trajectory
        # The loaded lap starts and ends at drop1; sitting there mid-leg is not arrival.
        h.pin = tuple(leg.end)
        for _ in range(20):
            h.step(dt=.1)
        self.assertEqual(h.engine.phase, 'FIXED_TRAJECTORY')
        self.assertIs(h.engine.fixed_trajectory, leg)

    def test_hold_mid_leg_revokes_the_leg(self):
        h = FixedReplay()
        h.until(lambda e: e.phase == 'FIXED_TRAJECTORY')
        h.step(dt=2.)
        h.engine.hold('operator hold')
        output = h.step()
        self.assertIsNone(h.engine.fixed_trajectory)
        self.assertEqual(h.engine.state, 'HOLD')
        self.assertEqual(output['control'], 'PX4_NATIVE')

    def test_skipped_drop_chains_the_next_leg_from_the_previous_end(self):
        h = FixedReplay()
        h.engine.route_released_points.add(0)
        h.until(lambda e: e.phase == 'FIXED_ARRIVAL')
        first = h.engine.fixed_trajectory
        h.until(lambda e: e.fixed_trajectory is not None and e.fixed_trajectory is not first)
        self.assertEqual(h.engine.fixed_trajectory.sample(0.)['position'], first.end)
        self.assertTrue([v for v in h.effects if v['type'] == 'delivery_route_point_skipped'])

    def test_solve_failure_before_takeoff_queues_nothing_and_in_flight_holds(self):
        e = DeliveryRouteEngine(fixed_configuration())
        with patch.object(native_delivery_route, 'plan_fixed_legs', side_effect=RuntimeError('library unavailable')):
            with self.assertRaisesRegex(RuntimeError, 'library unavailable'):
                e.start('delivery_route', (0., 0., -1.), .2, 100.)
        self.assertFalse([v for v in e._effects if v['type'] == 'takeoff'])
        h = FixedReplay()
        h.until(lambda e: e.phase == 'FIXED_ARRIVAL')
        h.engine.route_released_points.add(0)
        with patch.object(native_delivery_route, 'build_leg', side_effect=RuntimeError('library unavailable')):
            h.until(lambda e: e.state == 'HOLD')
        self.assertIn('固定航段求解失败', h.engine.reason)

    def test_gate_speed_caps_the_ring_in_both_laps(self):
        config = fixed_configuration(gate=.1)
        e = DeliveryRouteEngine(config)
        legs = plan_fixed_legs(config, e.itinerary, (0., 0., 0.))
        warped = [leg for leg in legs if leg['trajectory'].summary().get('gate_windows')]
        # The loaded lap and the drop3 -> drop4 leg both pass the ring.
        self.assertEqual([legs.index(leg) for leg in warped], [0, 3])
        for leg in warped:
            trajectory = leg['trajectory']
            gates = [ev for ev in trajectory.events if e.itinerary[ev['itinerary_index']]['height_role'] == 'gate']
            self.assertTrue(gates)
            for event in gates:
                self.assertLessEqual(np.linalg.norm(trajectory.sample(event['elapsed_s'])['velocity']), .1*(1.+1e-7))
        self.assertTrue(all(not leg['trajectory'].summary().get('gate_windows')
                            for i, leg in enumerate(legs) if i not in (0, 3)))

    def test_free_route_with_two_drops_and_ordinary_start_in_both_modes(self):
        document = free_document()
        steps = build_itinerary(document)
        self.assertEqual([s['drop_slot'] for s in steps if s['kind'] == 'drop'], [1, 2])
        # Lap two does not drop at the start when the first point is ordinary.
        self.assertNotEqual(steps[len(document['route'])+1]['kind'], 'drop')
        for fixed in (True, False):
            config = fixed_configuration() if fixed else configuration()
            config['delivery_route'] = document
            h = FixedReplay(config) if fixed else Replay(config)
            h.until(lambda e: e.phase == 'LANDING', limit=40000)
            e = h.engine
            self.assertEqual([v['slot'] for v in h.effects if v['type'] == 'release'], [1, 2])
            self.assertEqual((e.completed_laps, e.drops_completed), (2, 2))
            self.assertEqual(e.snapshot()['payload_total'], 2)
            if fixed:
                # Takeoff -> drop 1 (lap one and the start of lap two), -> drop 2, -> HOME.
                self.assertEqual(len(h.legs), 3)
                self.assertEqual(set(h.groups), {'NONE'})

    def test_drop_tests_fly_fixed_legs_and_return_to_the_corrected_takeoff_xy(self):
        from airseafly_real400_mission.native_drop_points import DropPointsEngine
        from airseafly_real400_mission.native_single_drop import SingleDropEngine
        points = dict(format='aquacore_drop_points', version=1, frame='fast_lio_local_enu',
                      points=[[2., 1.], [4., -2.]])
        cases = ((SingleDropEngine, 'drop_test', dict(drop_test=dict(target_xy=[2., 1.], slot=3)), [3], 2),
                 # Starts away from the corrected origin; HOME follows the correction.
                 (DropPointsEngine, 'drop_points', dict(drop_points=points), [1, 2], 3))
        for cls, mode, extra, slots, legs in cases:
            config = dict(fixed_configuration(), **extra)
            h = FixedReplay(config, cls, mode, start_xyz=(99., 88., 7.) if mode == 'drop_points' else None)
            h.until(lambda e: e.phase == 'LANDING', limit=40000)
            self.assertEqual([v['slot'] for v in h.effects if v['type'] == 'release'], slots)
            self.assertEqual(len(h.legs), legs)
            self.assertEqual(set(h.groups), {'NONE'})
            self.assertFalse([v for v in h.effects if v['type'] == 'goal'])
            self.assertEqual(h.legs[-1].end[:2], h.ground[:2])
            self.assertAlmostEqual(h.legs[0].sample(0.)['position'][2], h.ground[2]+1.4, places=6)

    def test_laps_one_flies_one_lap_dropping_in_order(self):
        document = configuration()['delivery_route']
        one, two = build_itinerary(document, 0), build_itinerary(document)
        self.assertEqual(two, build_itinerary(document, 1))
        navigation = [s for s in one if s['kind'] == 'navigation']
        points = [p['xy'] for p in document['route']]
        last = max(i for i, p in enumerate(document['route']) if p['kind'] == 'drop')
        self.assertEqual([s['xy'] for s in navigation], points+[points[0], document['home']])
        self.assertEqual([s['drop_slot'] for s in one if s['kind'] == 'drop'], [1, 2, 3, 4])
        self.assertTrue(all(s['lap'] == 1 for s in one if s['kind'] == 'drop'))
        # Each drop comes right after reaching its own point.
        for i, step in enumerate(one):
            if step['kind'] == 'drop':
                self.assertEqual((one[i-1]['kind'], one[i-1]['route_index']), ('navigation', step['route_index']))
        for value in (-1, 1.5, True, '1', None):
            with self.assertRaisesRegex(ValueError, 'laps'):
                build_itinerary(document, value)
        for fixed in (True, False):
            config = fixed_configuration() if fixed else configuration()
            config['delivery_route_options']['laps'] = 0
            h = FixedReplay(config) if fixed else Replay(config)
            stages = set()
            for _ in range(40000):
                if h.engine.phase == 'LANDING':
                    break
                stages.add(h.engine.snapshot()['delivery_route']['stage'])
                h.step()
            e = h.engine
            self.assertEqual([v['slot'] for v in h.effects if v['type'] == 'release'], [1, 2, 3, 4])
            self.assertEqual((e.completed_laps, e.drops_completed), (1, 4))
            self.assertNotIn('loaded_lap', stages)
            if fixed:
                # Takeoff -> drop1 -> drop2 -> drop3 -> drop4 -> HOME.
                self.assertEqual(len(h.legs), 5)
        value = preview(config, config['delivery_route'])
        self.assertEqual(value['laps'], 0)
        self.assertEqual(sum(s['kind'] == 'navigation' for s in value['itinerary']), len(points)+2)

    def test_five_laps_fly_loaded_laps_as_one_fixed_leg(self):
        config = fixed_configuration()
        config['delivery_route_options']['laps'] = 4
        h = FixedReplay(config)
        h.until(lambda e: e.phase == 'LANDING', limit=200000)
        e = h.engine
        self.assertEqual([v['slot'] for v in h.effects if v['type'] == 'release'], [1, 2, 3, 4])
        self.assertEqual((e.completed_laps, e.drops_completed), (5, 4))
        # Takeoff through four loaded laps to drop1 -> drop2 -> drop3 -> drop4 -> HOME.
        self.assertEqual(len(h.legs), 5)
        self.assertEqual(set(h.groups), {'NONE'})
        # The contest route lands at the recorded HOME, not the takeoff XY.
        self.assertEqual(h.legs[-1].end[:2], tuple(config['delivery_route']['home']))

    def test_dropping_lap_closes_at_first_point_then_home_and_flight_order_text(self):
        from airseafly_real400_mission.native_delivery_route import describe_laps
        document = free_document()  # 7 points; drops at 2 and 6, point 7 after the last drop.
        for laps in (5, 2, 1):
            steps = build_itinerary(document, laps-1)
            last_drop = max(i for i, s in enumerate(steps) if s['kind'] == 'drop')
            self.assertEqual([(s['stage'], s['route_index']) for s in steps[last_drop+1:]],
                             [('transit', 6), ('transit', 0), ('home', None), ('home', None)])
            self.assertTrue(steps[last_drop+2]['closing'])
            self.assertEqual(steps[last_drop+2]['completes_lap'], laps)
            self.assertIsNone(steps[last_drop+3]['completes_lap'])
        five = describe_laps(build_itinerary(document, 4))
        self.assertEqual(len(five), 6)
        self.assertEqual(five[1], '第2圈：2.投放1 → 3.圆环 → 4.圆环 → 5.普通 → 6.投放2 → 7.普通 → 1.普通（闭环）')
        self.assertEqual(five[4], '第5圈：2.投放1[投] → 3.圆环 → 4.圆环 → 5.普通 → 6.投放2[投] → 7.普通 → 1.普通（闭环）')
        two = describe_laps(build_itinerary(document, 1))
        self.assertEqual(two, ['第1圈：1.普通 → 2.投放1 → 3.圆环 → 4.圆环 → 5.普通 → 6.投放2 → 7.普通 → 1.普通（闭环）',
                               '第2圈：2.投放1[投] → 3.圆环 → 4.圆环 → 5.普通 → 6.投放2[投] → 7.普通 → 1.普通（闭环）',
                               '返航：1.普通 → HOME 降落'])
        self.assertEqual(describe_laps(build_itinerary(document, 0))[-1], '返航：1.普通 → HOME 降落')
        # Fixed legs: the last leg starts at the last drop, passes 7 and 1, and ends at HOME.
        config = fixed_configuration()
        config['delivery_route'] = document
        legs = plan_fixed_legs(config, build_itinerary(document), (0., 0., 0.))
        self.assertEqual(legs[-1]['trajectory'].end[:2], tuple(document['home']))
        self.assertEqual(len(legs[-1]['trajectory'].events), 3)

    def test_preview_lists_fixed_legs_and_ego_preview_does_not(self):
        config = fixed_configuration()
        value = preview(config, config['delivery_route'])
        self.assertEqual(value['backend'], 'fixed_min_snap')
        self.assertEqual([leg['ends'] for leg in value['fixed_legs']['legs']], ['drop']*4+['land'])
        self.assertTrue(all(leg['duration_s'] > 0. for leg in value['fixed_legs']['legs']))
        self.assertIsNone(preview(configuration(), configuration()['delivery_route'])['fixed_legs'])
        config['navigation']['backend'] = 'fixed_mni_snap'
        with self.assertRaisesRegex(ValueError, 'navigation.backend'):
            DeliveryRouteEngine(config)


if __name__ == '__main__':
    unittest.main()
