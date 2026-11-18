"""Offline two-lap mission replay. No ROS, services, cameras or aircraft."""
from copy import deepcopy
import math
from pathlib import Path
import sys
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from airseafly_real400_mission.configuration import load_config
from airseafly_real400_mission.native_delivery_route import DeliveryRouteEngine, build_itinerary


def route_document(extra=2):
    route = []
    for slot in range(1, 5):
        route.append(dict(stage='drop%d' % slot, kind='drop', xy=[slot*3., slot*2.], drop_slot=slot))
        stage, kind = (('obstacle2', 'transit') if slot == 1 else
                       ('obstacle3', 'transit') if slot == 2 else
                       ('gate', 'gate') if slot == 3 else ('return_drop1', 'transit'))
        for index in range(extra):
            route.append(dict(stage=stage, kind=kind, xy=[slot*3.+index+.5, slot*2.+1.]))
    return dict(version=1, frame='fast_lio_local_enu', home=[-4., -5.],
                route=route, wizard=dict(stage_index=9))


def configuration(document=None):
    config = load_config()
    config['flight'].update(controller='px4_native', native_reference_mode='lio_direct',
        native_task_feedback='lio', cruise_height_m=1.4, drop_height_m=.7, gate_height_m=1.6)
    config['payload'].update(sequence=[1, 2, 3, 4], release_dwell_s=0., release_max_speed_mps=None)
    config['delivery_route'] = deepcopy(document or route_document())
    return config


class Replay:
    def __init__(self, config=None):
        self.engine = DeliveryRouteEngine(config or configuration())
        self.ground, self.now = (12., -8., -1.3), 100.
        self.position, self.velocity, self.yaw = self.ground, (0., 0., 0.), .2
        self.armed, self.mode, self.owner = True, 'OFFBOARD', 'PX4_NATIVE'
        self.effects, self.groups = [], []
        self.engine.start('delivery_route', self.position, self.yaw, self.now)
        self.engine.set_red_point_session('offline-red-point')
        self.image_sequence = 0
        self.engine.native_origin = self.ground
        self.feedback()
        self.engine.begin_native_flight()

    def frame(self, xy=None, candidates=None, stamp=None, sequence=None, session='offline-red-point', received_at=None):
        self.image_sequence += 1
        phase = self.engine.phase
        landing = phase.startswith('LAND_')
        if candidates is None:
            xy = xy or (self.engine.route_home if landing else
                        self.engine.route_points[self.engine.route_index]['xy'])
            candidates = [dict(class_name='red_point', target_confirmed=True,
                               projection_status='ok_near_level', xy_m=list(xy))]
        return self.engine.on_red_point_observation(dict(schema='airseafly.red_point.v1',
            session_id=session, image_sequence=self.image_sequence if sequence is None else sequence,
            frame_id='fast_lio_local_enu', image_stamp_s=self.now if stamp is None else stamp,
            candidates=candidates if phase == 'NATIVE_SECOND_FRAME' else [],
            green_candidates=[dict(c, class_name='green_circle') for c in candidates]
                if phase == 'NATIVE_FIRST_FRAME' else [],
            landing_candidates=[dict(c, class_name='landing_pad') for c in candidates]
                if landing else []), now=received_at)

    def feedback(self):
        self.engine.on_native_lio_feedback(dict(stamp=self.now,
            position=self.position, velocity=self.velocity))
        self.engine.on_native_feedback(dict(stamp=self.now, pose_stamp=self.now,
            position=tuple(v+40 for v in self.position), velocity=self.velocity,
            yaw=self.yaw, owner=self.owner, armed=self.armed,
            offboard=self.mode == 'OFFBOARD', mode=self.mode, landed=not self.armed))

    def step(self, dt=.1, follow=True, acknowledge=True):
        self.now += dt
        e = self.engine
        motion = e._native_takeoff_motion if e.state == 'TAKEOFF' else e.native_motion
        if follow and motion:
            self.position, self.velocity, _ = motion.sample(self.now)
        elif follow and e.state == 'APPROACH' and e._goal_acked:
            self.position, self.velocity = tuple(e.command_position), (0., 0., 0.)
        self.feedback()
        if e.group == 'EGO':
            e.on_group_ready('EGO')
        output = e.tick(self.now, self.position, self.yaw, self.owner)
        self.groups.append(output['desired_group'])
        self.effects.extend(output['effects'])
        for effect in output['effects']:
            if effect['type'] == 'goal':
                e.on_goal_received(effect['goal_id'])
            elif effect['type'] == 'release' and acknowledge:
                e.on_payload_result(True, effect['request_id'])
            elif effect['type'] == 'native_owner':
                e.on_native_owner_result(effect['owner'], True, effect['owner'], 'offline replay')
                if effect['owner'] == 'PX4_AUTO_LAND':
                    self.owner, self.mode = 'PX4_AUTO_LAND', 'AUTO.LAND'
        return output

    def until(self, predicate, limit=5000, **kwargs):
        for _ in range(limit):
            if predicate(self.engine):
                return
            self.step(**kwargs)
        raise AssertionError((self.engine.state, self.engine.phase, self.engine.snapshot()['delivery_route']))


class DeliveryRouteTests(unittest.TestCase):
    def test_shared_itinerary_closes_every_lap_then_returns_home_from_the_first_point(self):
        for extra in (0, 1, 10):
            document = route_document(extra)
            steps = build_itinerary(document)
            navigation = [s for s in steps if s['kind'] == 'navigation']
            points = [p['xy'] for p in document['route']]
            last = max(i for i, p in enumerate(document['route']) if p['kind'] == 'drop')
            # Lap two also flies every point and closes at the first point, then HOME.
            self.assertEqual([s['xy'] for s in navigation],
                             points+[points[0]]+points[1:]+[points[0]]+[document['home']])
            self.assertEqual([s['completes_lap'] for s in navigation if s['closing']], [1, 2])
            self.assertIsNone(navigation[-1]['completes_lap'])
            self.assertEqual([s['drop_slot'] for s in steps if s['kind'] == 'drop'], [1, 2, 3, 4])
            self.assertTrue(all(s['lap'] == 2 for s in steps if s['kind'] == 'drop'))
            self.assertEqual(steps[-1]['kind'], 'land')

    def test_configured_laps_fly_loaded_laps_then_drop_on_last_lap(self):
        for laps in (1, 2, 5):
            for extra in (0, 2):
                document = route_document(extra)
                steps = build_itinerary(document, laps-1)  # laps-1 cruise laps + the dropping lap
                points = [p['xy'] for p in document['route']]
                last = max(i for i, p in enumerate(document['route']) if p['kind'] == 'drop')
                navigation = [s for s in steps if s['kind'] == 'navigation']
                # Every lap, the dropping lap included, closes at the first point;
                # HOME is reached from there.
                expected = [points[0]]+(points[1:]+[points[0]])*laps+[document['home']]
                self.assertEqual([s['xy'] for s in navigation], expected, (laps, extra))
                self.assertEqual([s['completes_lap'] for s in navigation if s['closing']], list(range(1, laps+1)))
                self.assertIsNone(navigation[-1]['completes_lap'])
                drops = [s for s in steps if s['kind'] == 'drop']
                self.assertEqual([s['drop_slot'] for s in drops], [1, 2, 3, 4])
                self.assertTrue(all(s['lap'] == laps for s in drops))
                self.assertTrue(all(s['lap'] < laps for s in steps[:steps.index(drops[0])] if laps > 1))
                self.assertEqual([s['kind'] for s in steps[-2:]], ['navigation', 'land'])

    def test_laps_must_be_zero_or_positive_integer(self):
        for value in (-1, 1.5, '2', True, None):
            config = configuration()
            config['delivery_route_options']['laps'] = value
            with self.assertRaisesRegex(ValueError, '正整数'):
                DeliveryRouteEngine(config)

    def test_replay_each_lap_count_releases_only_on_last_lap(self):
        for laps in (1, 2, 5):
            config = configuration()
            config['delivery_route_options']['laps'] = laps-1  # cruise laps
            h = Replay(config)
            stages = []
            for _ in range(20000):
                if h.engine.phase == 'LANDING':
                    break
                output = h.step()
                route = output['delivery_route']
                stages.append((route['stage'], route['lap'], h.engine.drops_completed))
                for effect in output['effects']:
                    if effect['type'] == 'release':
                        self.assertEqual((h.engine.route_lap, h.engine.completed_laps), (laps, laps-1))
            else:
                self.fail((laps, h.engine.state, h.engine.phase))
            e = h.engine
            self.assertEqual([v['slot'] for v in h.effects if v['type'] == 'release'], [1, 2, 3, 4])
            self.assertEqual((e.completed_laps, e.drops_completed, e.route_laps), (laps, 4, laps-1))
            completes = [v['completed_laps'] for v in h.effects if v['type'] == 'delivery_route_lap_complete']
            self.assertEqual(completes, list(range(1, laps+1)))
            self.assertTrue(all((stage == 'loaded_lap') == (lap < laps) for stage, lap, _ in stages))
            self.assertTrue(all(stage == 'delivery' for stage, _, drops in stages if drops))
            self.assertEqual(any(stage == 'loaded_lap' for stage, _, _ in stages), laps > 1)
            navigation = [v for v in h.effects if v['type'] == 'delivery_route_point']
            self.assertEqual([v['position_xyz'][:2] for v in navigation],
                             [s['xy'] for s in e.itinerary if s['kind'] == 'navigation'])
            h.armed = False
            h.step()
            self.assertEqual((e.done, e.phase), (True, 'LANDED'))
            self.assertEqual(e.snapshot()['telemetry']['phase_name'], '巡航%d圈及投放完成，已落地上锁' % (laps-1))

    def test_entire_two_laps_exactly_four_releases_then_home_and_actual_landing(self):
        h = Replay()
        h.until(lambda e: e.phase == 'LANDING')
        e = h.engine
        releases = [v for v in h.effects if v['type'] == 'release']
        self.assertEqual([v['slot'] for v in releases], [1, 2, 3, 4])
        self.assertEqual(len({v['request_id'] for v in releases}), 4)
        self.assertEqual(e.completed_laps, 2)
        self.assertEqual(e.drops_completed, 4)
        self.assertEqual(e.route_released_slots, [1, 2, 3, 4])
        self.assertEqual(set(h.groups), {'NONE', 'EGO'})
        events = [v['type'] for v in h.effects]
        self.assertLess(events.index('delivery_route_lap_complete'), events.index('release'))
        self.assertLess(max(i for i,v in enumerate(events) if v == 'delivery_route_lap_complete'),
                        events.index('delivery_route_home_arrived'))
        navigation = [v for v in h.effects if v['type'] == 'delivery_route_point']
        expected = [s for s in e.itinerary if s['kind'] == 'navigation']
        self.assertEqual([v['position_xyz'][:2] for v in navigation], [s['xy'] for s in expected])
        for actual, step in zip(navigation, expected):
            self.assertAlmostEqual(actual['position_xyz'][2], h.ground[2]+(1.6 if step['height_role'] == 'gate' else 1.4))
        self.assertFalse(e.done)
        h.armed = False
        h.step()
        self.assertTrue(e.done)
        self.assertEqual(e.phase, 'LANDED')
        self.assertEqual(sum(v['type'] == 'takeoff' for v in h.effects), 1)

    def test_ack_wait_does_not_add_timeout_and_duplicate_reply_does_not_release_again(self):
        h = Replay()
        h.until(lambda e: e.phase == 'WAIT_ACK', acknowledge=False)
        e = h.engine
        request = e._payload_request
        h.step(dt=120., acknowledge=False)
        self.assertEqual(e.phase, 'WAIT_ACK')
        self.assertEqual(e.drops_completed, 0)
        self.assertEqual(len([v for v in h.effects if v['type'] == 'release']), 1)
        e.on_payload_result(True, request)
        e.on_payload_result(True, request)
        h.step(acknowledge=False)
        self.assertEqual(e.phase, 'SETTLE')
        self.assertEqual(e.drops_completed, 1)
        h.step(dt=e.config['payload']['settle_s']/2, acknowledge=False)
        self.assertEqual(e.phase, 'SETTLE')
        h.until(lambda e: e.phase == 'WAIT_ACK', acknowledge=False)
        e.on_payload_result(True, request)
        h.step(acknowledge=False)
        self.assertEqual((e.phase, e.drops_completed), ('WAIT_ACK', 1))

    def test_both_laps_and_drop_phases_keep_ego_running_until_landing(self):
        h = Replay()
        h.until(lambda e: e.state == 'APPROACH')
        phases, releases = set(), []
        for _ in range(5000):
            output = h.step()
            e = h.engine
            if e.state == 'LAND':
                self.assertEqual(output['desired_group'], 'NONE')
                break
            self.assertEqual(output['desired_group'], 'EGO', (e.state, e.phase))
            self.assertFalse(any(v['type'] == 'restart_ego' for v in output['effects']))
            if e.completed_laps == 0:
                self.assertEqual(e.drops_completed, 0)
                self.assertFalse(any(v['type'] == 'release' for v in output['effects']))
            if e.state == 'DROP':
                phases.add(e.phase)
                self.assertEqual(e.route_lap, 2)
                self.assertEqual(output['control'], 'PX4_NATIVE')
                self.assertIn(output['reference_source'], ('ALIGN', 'RECOVER'))
            releases.extend((e.route_lap, v['slot']) for v in output['effects'] if v['type'] == 'release')
        else:
            self.fail((h.engine.state, h.engine.phase))
        # No H frames are sent here: the first H wait times out to a straight descent.
        self.assertEqual(phases, {'NATIVE_INTERMEDIATE_DESCEND', 'NATIVE_SECOND_FRAME', 'NATIVE_DESCEND', 'WAIT_ACK', 'SETTLE', 'NATIVE_RECOVER',
                                  'LAND_FIRST_FRAME', 'LAND_DESCEND'})
        self.assertEqual(releases, [(2, 1), (2, 2), (2, 3), (2, 4)])
        self.assertEqual(h.engine.completed_laps, 2)

    def test_drop_offset_uses_body_reference_and_yaw_not_old_camera_offset(self):
        config = configuration()
        config['delivery_route_options'] = dict(mount_offsets_body_m={'1': [.2, -.1]})
        h = Replay(config)
        h.yaw = math.pi/2
        h.until(lambda e: e.phase == 'NATIVE_OFFSET_MOVE')
        point = config['delivery_route']['route'][0]['xy']
        self.assertAlmostEqual(h.engine.native_goal[0], point[0]-.1)
        self.assertAlmostEqual(h.engine.native_goal[1], point[1]-.2)
        self.assertAlmostEqual(h.engine.native_goal[2], h.ground[2]+.7)

    def test_hold_during_descent_never_releases(self):
        h = Replay()
        h.until(lambda e: e.phase == 'NATIVE_DESCEND')
        h.engine.hold('operator pause')
        for _ in range(10):
            h.step(dt=10.)
        self.assertEqual(h.engine.state, 'HOLD')
        self.assertFalse(any(v['type'] == 'release' for v in h.effects))

    def test_previous_visual_deadline_does_not_interrupt_a_recorded_point(self):
        h = Replay()
        h.until(lambda e: e.state == 'APPROACH')
        point = h.engine.command_position
        h.step(dt=400., follow=False)
        self.assertEqual(h.engine.state, 'APPROACH')
        self.assertEqual(h.engine.command_position, point)
        self.assertEqual(h.engine.completed_laps, 0)
        self.assertFalse(h.engine.final_route_deadline_enabled)
        self.assertFalse(any(v['type'] in ('release', 'delivery_route_lap_complete') for v in h.effects))


if __name__ == '__main__':
    unittest.main()
