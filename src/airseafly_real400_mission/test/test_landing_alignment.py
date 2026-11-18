"""Final H-pad landing replay: two fixes, 5 s waits, low descent, AUTO.LAND. No ROS."""
import unittest

from test_native_delivery_route import Replay, configuration
from airseafly_real400_mission.delivery_alignment import landing_settings
from airseafly_real400_mission.native_delivery_route import DeliveryRouteEngine
from airseafly_real400_mission.native_drop_points import DropPointsEngine
from airseafly_real400_mission.native_single_drop import SingleDropEngine


def pad(xy, confirmed=True, projection='ok_near_level', kind='landing_pad'):
    return dict(class_name=kind, target_confirmed=confirmed, projection_status=projection, xy_m=list(xy))


def landing(config=None):
    h = Replay(config)
    h.until(lambda e: e.phase == 'LAND_FIRST_FRAME')
    h.landing_start = len(h.effects)
    return h


def deadline(h, before=0.):
    end = h.engine._alignment_deadline()
    return h.step(max(0., end-h.now-before), follow=False, acknowledge=False)


def raw(h, **lists):
    h.image_sequence += 1
    packet = dict(schema='airseafly.red_point.v1', session_id='offline-red-point',
                  image_sequence=h.image_sequence, frame_id='fast_lio_local_enu', image_stamp_s=h.now)
    packet.update(lists)
    return h.engine.on_red_point_observation(packet)


def types(h):
    return [effect['type'] for effect in h.effects[h.landing_start:]]


class LandingAlignmentTests(unittest.TestCase):
    def test_two_h_fixes_then_low_descent_hands_off_to_auto_land(self):
        h = landing(); e = h.engine
        home = e.route_home
        self.assertTrue(e.red_point_window().startswith("LAND_"))
        self.assertEqual(e.snapshot()['desired_group'], 'EGO')
        self.assertAlmostEqual(e._alignment_deadline()-e.phase_started_at, 5.)
        self.assertTrue(h.frame(candidates=[pad((home[0]+3., home[1])), pad((home[0]+.12, home[1]-.08))]))
        first = e.native_goal
        self.assertEqual((e.phase, first[:2]), ('LAND_FIRST_MOVE', (home[0]+.12, home[1]-.08)))
        self.assertIsNone(e.red_point_window())
        h.until(lambda e: e.phase == 'LAND_SECOND_FRAME')
        self.assertEqual(e.native_goal, (*first[:2], .5))
        self.assertTrue(h.frame(candidates=[pad((9., 9.)), pad((first[0]+.03, first[1]+.02))]))
        second = e.native_goal
        self.assertEqual(second[:2], (first[0]+.03, first[1]+.02))
        h.until(lambda e: e.phase == 'LAND_DESCEND')
        self.assertEqual(e.native_goal[:2], second[:2])
        self.assertAlmostEqual(e.native_goal[2], h.ground[2]+.15)
        h.until(lambda e: e.state == 'LAND')
        self.assertEqual(e.phase, 'WAIT_LAND')
        self.assertEqual(e.command_position, (*second[:2], h.ground[2]+.15))
        self.assertEqual(e.landing_alignment_result['fixes'][-1]['target_xyz'][:2], list(second[:2]))
        self.assertEqual(e.snapshot()['native_command']['position'], (*second[:2], h.ground[2]+.15))
        complete = [v for v in h.effects if v['type'] == 'landing_alignment_complete']
        self.assertEqual([(v['fixes'], v['fallback']) for v in complete], [(2, False)])
        phases = [v['phase'] for v in h.effects[h.landing_start:] if v['type'] == 'native_target']
        self.assertEqual(phases, ['LAND_FIRST_MOVE', 'LAND_INTERMEDIATE_DESCEND', 'LAND_SECOND_MOVE', 'LAND_DESCEND'])
        self.assertNotIn('release', types(h))
        h.until(lambda e: e.phase == 'LANDING')
        self.assertIn({'type': 'native_owner', 'owner': 'PX4_AUTO_LAND'},
                      [{k: v[k] for k in ('type', 'owner')} for v in h.effects if v['type'] == 'native_owner'])
        h.armed = False
        h.step()
        self.assertEqual((e.done, e.phase, e.drops_completed), (True, 'LANDED', 4))
        self.assertEqual(e.snapshot()['delivery_route']['landing_alignment']['result']['samples'], 2)

    def test_first_h_wait_timeout_descends_straight_down_at_current_xy(self):
        h = landing(); e = h.engine
        start = e.native_goal
        deadline(h, before=.01)
        self.assertEqual(e.phase, 'LAND_FIRST_FRAME')
        deadline(h)
        self.assertEqual(e.phase, 'LAND_DESCEND')
        self.assertEqual(e.native_goal[:2], start[:2])
        self.assertAlmostEqual(e.native_goal[2], h.ground[2]+.15)
        self.assertTrue(e._delivery_fallback_active)
        self.assertIn('H 标', e._delivery_fallback_reason)
        self.assertFalse(h.frame((9., 9.)))
        self.assertEqual(e.native_goal[:2], start[:2])
        h.until(lambda e: e.state == 'LAND')
        self.assertEqual(e.command_position[:2], start[:2])
        self.assertEqual([v['fallback'] for v in h.effects if v['type'] == 'landing_alignment_complete'], [True])

    def test_second_h_wait_timeout_lands_at_first_fix(self):
        h = landing(); e = h.engine
        home = e.route_home
        h.frame((home[0]+.1, home[1]+.05))
        first = e.native_goal
        h.until(lambda e: e.phase == 'LAND_SECOND_FRAME')
        self.assertTrue(e.red_point_window().startswith("LAND_"))
        self.assertAlmostEqual(e._alignment_deadline()-e.phase_started_at, 5.)
        deadline(h)
        self.assertEqual(e.phase, 'LAND_DESCEND')
        self.assertEqual(e.native_goal[:2], first[:2])

    def test_landing_never_retakes_after_move_timeout(self):
        h = landing(); e = h.engine
        home = e.route_home
        h.frame((home[0]+.3, home[1]))
        target = e.native_goal
        deadline(h, before=.01)
        self.assertEqual(e.phase, 'LAND_FIRST_MOVE')
        deadline(h)  # follow=False: the vehicle never arrives.
        self.assertEqual((e.phase, e._delivery_retakes), ('LAND_DESCEND', 0))
        self.assertEqual(e.native_goal[:2], target[:2])
        self.assertNotIn('native_alignment_retake', types(h))
        status = e._route_alignment_status()
        self.assertEqual((status['fallback']['retake_limit'], status['fallback']['frame_timeout_s']), (0, 5.))

    def test_only_landing_pad_candidates_count_and_drops_ignore_h(self):
        h = landing(); e = h.engine
        xy = e.route_home
        self.assertTrue(raw(h, candidates=[pad(xy, kind='red_point')],
                            green_candidates=[pad(xy, kind='green_circle')]))
        self.assertTrue(raw(h, landing_candidates=[pad(xy, kind='red_point'), pad(xy, confirmed=False),
                                                   pad(xy, projection='missing_odometry')]))
        self.assertEqual(e.phase, 'LAND_FIRST_FRAME')
        self.assertTrue(raw(h, landing_candidates=[pad(xy)]))
        self.assertEqual(e.phase, 'LAND_FIRST_MOVE')
        drop = Replay()
        drop.until(lambda e: e.phase == 'NATIVE_SECOND_FRAME')
        self.assertTrue(raw(drop, landing_candidates=[pad(drop.engine.route_points[0]['xy'])]))
        self.assertEqual(drop.engine.phase, 'NATIVE_SECOND_FRAME')
        self.assertTrue(drop.engine.red_point_window().startswith("NATIVE_"))

    def test_descent_speeds_and_phase_labels(self):
        h = landing(); e = h.engine
        self.assertEqual(e._vertical_speed('LAND_DESCEND'), .30)
        self.assertEqual(e._vertical_speed('LAND_INTERMEDIATE_DESCEND'),
                         e._vertical_speed('NATIVE_INTERMEDIATE_DESCEND'))
        snapshot = e.snapshot()
        self.assertEqual(snapshot['telemetry']['phase_name'], 'H 标第一次对准：等待识别')
        self.assertEqual(snapshot['delivery_route']['visual_alignment']['purpose'], 'land')
        self.assertEqual(snapshot['delivery_route']['visual_alignment']['target_kind'], 'landing_pad')
        self.assertTrue(snapshot['delivery_route']['landing_alignment']['enabled'])
        self.assertEqual(snapshot['reference_source'], 'ALIGN')

    def test_hold_during_landing_alignment_never_requests_auto_land(self):
        h = landing(); e = h.engine
        e.hold('operator pause')
        for _ in range(10):
            h.step(dt=10.)
        self.assertEqual(e.state, 'HOLD')
        self.assertIsNone(e.red_point_window())
        self.assertNotIn('native_owner', types(h))

    def test_disabled_and_takeoff_xy_engines_keep_coordinate_landing(self):
        config = configuration()
        config['delivery_route_options']['landing_alignment'] = dict(enabled=False)
        h = Replay(config)
        h.until(lambda e: e.state == 'LAND')
        self.assertEqual(h.engine.phase, 'WAIT_LAND')
        self.assertFalse(any(v['type'] == 'landing_alignment_start' for v in h.effects))
        self.assertFalse(SingleDropEngine.landing_alignment_supported)
        self.assertFalse(DropPointsEngine.landing_alignment_supported)
        self.assertTrue(DeliveryRouteEngine.landing_alignment_supported)

    def test_invalid_landing_settings_are_rejected(self):
        self.assertEqual(landing_settings(None), dict(enabled=True, frame_timeout_s=5.,
                                                      final_height_m=.15, final_speed_mps=.3))
        for bad in (dict(enabled='yes'), dict(frame_timeout_s=0), dict(final_height_m=-.1),
                    dict(final_speed_mps=True), dict(frame_timeout_s=float('inf'))):
            with self.assertRaises(ValueError):
                landing_settings(bad)
            config = configuration()
            config['delivery_route_options']['landing_alignment'] = bad
            with self.assertRaises(ValueError):
                DeliveryRouteEngine(config)


if __name__ == '__main__':
    unittest.main()
