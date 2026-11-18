"""One selected logical drop, return to actual takeoff XY, and confirmed land."""
from copy import deepcopy
import math
import unittest

from test_native_delivery_route import Replay
from airseafly_real400_mission.configuration import load_config
from airseafly_real400_mission.native_single_drop import (
    SingleDropEngine, build_single_drop_itinerary, validate_drop_test)


def configuration(slot=4, target=(-3., 6.)):
    config = load_config()
    config.pop('delivery_route', None)
    config['flight'].update(controller='px4_native', native_reference_mode='lio_direct',
        native_task_feedback='lio', cruise_height_m=1.4, drop_height_m=.7)
    config['drop_test'] = dict(target_xy=list(target), slot=slot)
    return config


class SingleDropReplay(Replay):
    def __init__(self, config=None, start_position=None):
        self.engine = SingleDropEngine(config or configuration())
        self.ground, self.now = (12., -8., -1.3), 100.
        self.position, self.velocity, self.yaw = self.ground, (0., 0., 0.), .2
        self.armed, self.mode, self.owner = True, 'OFFBOARD', 'PX4_NATIVE'
        self.effects, self.groups, self.image_sequence = [], [], 0
        self.engine.start('drop_test', start_position or self.ground, self.yaw, self.now)
        self.engine.set_red_point_session('offline-red-point')
        self.engine.native_origin = self.ground
        self.feedback()
        self.engine.begin_native_flight()

    def until(self, predicate, limit=5000, observe=False, **kwargs):
        for _ in range(limit):
            if predicate(self.engine):
                return
            if observe and self.engine.phase == 'NATIVE_SECOND_FRAME':
                self.frame()
            self.step(**kwargs)
        raise AssertionError((self.engine.state, self.engine.phase, self.engine.snapshot()['drop_test']))


class SingleDropTests(unittest.TestCase):
    def test_input_contract_rejects_invalid_target_and_slot_without_full_route(self):
        self.assertEqual(validate_drop_test({'target_xy': (1, -2), 'slot': 4}),
                         {'target_xy': [1., -2.], 'slot': 4, 'finish': 'land'})
        for target in (None, [1], [1, 2, 3], [True, 2], [float('nan'), 2], [1, float('inf')]):
            with self.subTest(target=target), self.assertRaises(ValueError):
                validate_drop_test({'target_xy': target, 'slot': 1})
        for slot in (None, True, 0, 5, 1.0, '4'):
            with self.subTest(slot=slot), self.assertRaises(ValueError):
                validate_drop_test({'target_xy': [1, 2], 'slot': slot})
        engine = SingleDropEngine(configuration())
        self.assertIsNone(engine.route_document)
        self.assertEqual(len(engine.route_points), 1)
        self.assertEqual(engine.route_points[0]['drop_slot'], 4)

    def test_itinerary_contains_one_real_target_and_a_deferred_or_actual_home(self):
        for home in (None, [8., 9.]):
            steps = build_single_drop_itinerary([3., 4.], 4, home)
            self.assertEqual([s['kind'] for s in steps], ['navigation', 'drop', 'navigation', 'land'])
            self.assertEqual([s['xy'] for s in steps], [[3., 4.], [3., 4.], home, home])
            self.assertEqual([s['drop_slot'] for s in steps if s['kind'] == 'drop'], [4])
            self.assertTrue(all(s['lap'] is None for s in steps))

    def test_each_logical_slot_releases_once_then_returns_home_and_confirms_landing(self):
        for slot in (1, 2, 3, 4):
            with self.subTest(slot=slot):
                h = SingleDropReplay(configuration(slot)); e = h.engine
                h.until(lambda e: e.phase == 'LANDING', observe=True)
                releases = [v for v in h.effects if v['type'] == 'release']
                self.assertEqual([v['slot'] for v in releases], [slot])
                self.assertEqual(e.route_released_slots, [slot])
                self.assertEqual(e.route_released_points, {0})
                self.assertEqual(e.drops_completed, 1)
                self.assertEqual(len(e.native_fixes), 1)
                goals = [v for v in h.effects if v['type'] == 'goal']
                self.assertEqual([v['position_xyz'][:2] for v in goals], [[-3., 6.], list(h.ground[:2])])
                self.assertTrue(all(v['purpose'] == 'drop_test' for v in goals))
                self.assertTrue(all(abs(v['position_xyz'][2]-(h.ground[2]+1.4)) < 1e-9 for v in goals))
                self.assertEqual(e.landing_xy, h.ground[:2])
                self.assertEqual(e.mode, 'drop_test')
                self.assertFalse(e.done)
                self.assertFalse(any(v['type'] == 'delivery_route_lap_complete' for v in h.effects))
                self.assertFalse(any(v['type'] == 'restart_ego' for v in h.effects))
                h.step()
                self.assertFalse(e.done)  # AUTO.LAND mode alone is not actual landing.
                h.armed = False; h.step()
                self.assertTrue(e.done)
                self.assertEqual(e.phase, 'LANDED')
                self.assertEqual(e.snapshot()['payload_total'], 1)
                self.assertEqual(e.snapshot()['drop_test']['stage'], 'complete')
                self.assertNotIn('delivery_route', e.snapshot())

    def test_home_is_prepared_actual_takeoff_origin_and_does_not_follow_later_pose(self):
        h = SingleDropReplay(start_position=(80., 60., 4.)); e = h.engine
        self.assertEqual(e.origin_xyz, h.ground)
        self.assertEqual(e.route_home, h.ground[:2])
        self.assertEqual(e.snapshot()['drop_test']['home_xy'], list(h.ground[:2]))
        h.until(lambda e: e.phase == 'NATIVE_SECOND_FRAME')
        h.position = (-9., 8., e.cruise_z); h.feedback()
        self.assertEqual(e.route_home, h.ground[:2])
        h.until(lambda e: e.phase == 'LANDING', observe=True)
        home_goal = [v for v in h.effects if v['type'] == 'goal'][-1]
        self.assertEqual(home_goal['position_xyz'][:2], list(h.ground[:2]))

    def test_no_image_fallback_uses_supplied_target_and_returns_home_after_single_drop(self):
        h = SingleDropReplay(); e = h.engine
        h.until(lambda e: e.phase == 'NATIVE_SECOND_FRAME')
        h.step(dt=14.99, follow=False)
        self.assertEqual(e.phase, 'NATIVE_SECOND_FRAME')
        h.step(dt=.01, follow=False)
        self.assertEqual(e.phase, 'NATIVE_DESCEND')
        self.assertEqual(e.native_goal[:2], (-3., 6.))
        h.until(lambda e: e.phase == 'LANDING')
        self.assertEqual([v['slot'] for v in h.effects if v['type'] == 'release'], [4])
        self.assertTrue(e._delivery_fallback_active)
        self.assertFalse(e.native_fixes)
        self.assertEqual(e.route_home, h.ground[:2])

    def test_waits_for_real_ack_once_then_settles_recovers_and_returns(self):
        h = SingleDropReplay(); e = h.engine
        h.until(lambda e: e.phase == 'WAIT_ACK', observe=True, acknowledge=False)
        request = e._payload_request
        h.step(dt=120., acknowledge=False)
        self.assertEqual(e.phase, 'WAIT_ACK')
        self.assertEqual(e.drops_completed, 0)
        self.assertEqual(e.route_released_points, set())
        self.assertEqual(len([v for v in h.effects if v['type'] == 'release']), 1)
        self.assertEqual(len([v for v in h.effects if v['type'] == 'goal']), 1)
        e.on_payload_result(True, request+1); h.step(acknowledge=False)
        self.assertEqual(e.phase, 'WAIT_ACK')
        e.on_payload_result(True, request); e.on_payload_result(True, request)
        h.step(acknowledge=False)
        self.assertEqual(e.phase, 'SETTLE')
        h.step(dt=e.config['payload']['settle_s']/2, acknowledge=False)
        self.assertEqual(e.phase, 'SETTLE')
        h.until(lambda e: e.phase == 'NATIVE_RECOVER', acknowledge=False)
        self.assertAlmostEqual(e.native_goal[2], e.cruise_z)
        self.assertEqual(len([v for v in h.effects if v['type'] == 'goal']), 1)
        h.until(lambda e: e.phase == 'LANDING', acknowledge=False)
        e.on_payload_result(True, request); h.step(acknowledge=False)
        self.assertEqual(e.drops_completed, 1)
        self.assertEqual(len([v for v in h.effects if v['type'] == 'release']), 1)

    def test_failed_or_unknown_release_does_not_return_land_or_repeat(self):
        for outcome in ('rejected', 'unknown', 'not_sent'):
            with self.subTest(outcome=outcome):
                h = SingleDropReplay(); e = h.engine
                h.until(lambda e: e.phase == 'WAIT_ACK', observe=True, acknowledge=False)
                e.on_payload_result(False, e._payload_request, 'test failure', outcome)
                h.step(dt=120., acknowledge=False)
                self.assertEqual(e.state, 'HOLD')
                self.assertEqual(e.drops_completed, 0)
                self.assertEqual(e.route_released_points, set())
                self.assertEqual(len([v for v in h.effects if v['type'] == 'release']), 1)
                self.assertEqual(len([v for v in h.effects if v['type'] == 'goal']), 1)
                self.assertFalse(any(v.get('owner') == 'PX4_AUTO_LAND' for v in h.effects))

    def test_selected_fourth_slot_uses_only_its_offset_after_one_fix(self):
        config = configuration(4)
        config.setdefault('delivery_route_options', {})['mount_offsets_body_m'] = {
            '1': [9., 9.], '4': [.2, -.1]}
        before = deepcopy(config['payload'])
        h = SingleDropReplay(config); e = h.engine
        h.yaw = math.pi/2; h.feedback()
        h.until(lambda e: e.phase == 'NATIVE_OFFSET_MOVE', observe=True)
        self.assertEqual(e.native_offset['slot'], 4)
        self.assertAlmostEqual(e.native_goal[0], -3.1)
        self.assertAlmostEqual(e.native_goal[1], 5.8)
        self.assertEqual(len(e.native_fixes), 1)
        h.until(lambda e: e.phase == 'LANDING')
        self.assertEqual([v['slot'] for v in h.effects if v['type'] == 'release'], [4])
        self.assertEqual(e.config['payload'], before)

    def test_snapshot_has_single_test_stages_without_full_route_lap_semantics(self):
        h = SingleDropReplay(); e = h.engine
        self.assertEqual(e.snapshot()['drop_test']['stage'], 'takeoff')
        h.until(lambda e: e.state == 'APPROACH')
        self.assertEqual(e.snapshot()['drop_test']['stage'], 'approach_target')
        h.until(lambda e: e.phase == 'NATIVE_SECOND_FRAME')
        status = e.snapshot()['drop_test']
        self.assertEqual(status['current_point']['drop_slot'], 4)
        self.assertEqual(status['stage'], 'delivery')
        self.assertNotIn('lap', status)
        h.until(lambda e: e.phase == 'NATIVE_RECOVER', observe=True)
        self.assertEqual(e.snapshot()['drop_test']['stage'], 'recover')
        h.until(lambda e: e.state == 'APPROACH')
        self.assertEqual(e.snapshot()['drop_test']['stage'], 'return_home')
        self.assertIn('返回本次起飞点', e.snapshot()['telemetry']['phase_name'])


if __name__ == '__main__':
    unittest.main()
