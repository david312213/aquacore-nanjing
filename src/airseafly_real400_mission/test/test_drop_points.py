"""Replay actual mission logic with synthetic localization, imagery and ACKs."""
from copy import deepcopy
import unittest

from test_native_delivery_route import Replay
from test_single_drop import configuration
from airseafly_real400_mission.drop_points import DropPointRecorder, select_points, validate_points
from airseafly_real400_mission.native_drop_points import DropPointsEngine


def document(count=4):
    return dict(format='aquacore_drop_points', version=1, frame='fast_lio_local_enu',
                points=[[float(i), float(-i)] for i in range(1, count+1)])


class DropPointsReplay(Replay):
    def __init__(self, count=4, points=None):
        config = configuration()
        config.pop('drop_test')
        config['drop_points'] = deepcopy(points or document(count))
        self.engine = DropPointsEngine(config)
        self.ground, self.now = (12., -8., -1.3), 100.
        self.position, self.velocity, self.yaw = self.ground, (0., 0., 0.), .2
        self.armed, self.mode, self.owner = True, 'OFFBOARD', 'PX4_NATIVE'
        self.effects, self.groups, self.image_sequence = [], [], 0
        self.engine.start('drop_points', (99., 88., 7.), self.yaw, self.now)
        self.engine.set_red_point_session('offline-red-point')
        self.engine.native_origin = self.ground
        self.feedback()
        self.engine.begin_native_flight()

    def until(self, predicate, limit=5000, observe=True, **kwargs):
        for _ in range(limit):
            if predicate(self.engine):
                return
            if observe and self.engine.phase == 'NATIVE_SECOND_FRAME':
                self.frame()
            self.step(**kwargs)
        raise AssertionError((self.engine.state, self.engine.phase, self.engine.snapshot()['drop_points']))


class DropPointsTests(unittest.TestCase):
    def test_one_through_four_targets_release_once_each_then_return_to_prepared_origin(self):
        for count in range(1, 5):
            with self.subTest(count=count):
                h = DropPointsReplay(count); e = h.engine
                h.until(lambda e: e.phase == 'LANDING')
                self.assertEqual(e.route_home, h.ground[:2])
                self.assertEqual([x['slot'] for x in h.effects if x['type'] == 'release'], list(range(1, count+1)))
                self.assertEqual(e.route_released_points, set(range(count)))
                goals = [x['position_xyz'] for x in h.effects if x['type'] == 'goal']
                self.assertEqual([x[:2] for x in goals], document(count)['points']+[list(h.ground[:2])])
                self.assertTrue(all(abs(x[2]-(h.ground[2]+1.4)) < 1e-9 for x in goals))
                self.assertEqual(e.completed_laps, 0)
                self.assertFalse(any(x['type'] == 'restart_ego' for x in h.effects))
                self.assertFalse(e.done)
                h.step(); self.assertFalse(e.done)
                h.armed = False; h.step()
                self.assertTrue(e.done)
                self.assertEqual(e.snapshot()['payload_total'], count)
                self.assertEqual(e.snapshot()['drop_points']['stage'], 'complete')
                self.assertNotIn('delivery_route', e.snapshot())
                self.assertEqual(e.drops_completed, count)

    def test_old_ack_does_not_advance_second_point_and_hold_preserves_success(self):
        h = DropPointsReplay(3); e = h.engine
        h.until(lambda e: e.phase == 'WAIT_ACK', acknowledge=False)
        first = e._payload_request
        h.step(dt=120, acknowledge=False)
        self.assertEqual(e.drops_completed, 0)
        self.assertEqual(len([x for x in h.effects if x['type'] == 'release']), 1)
        e.on_payload_result(True, first)
        h.until(lambda e: e.route_index == 1 and e.phase == 'WAIT_ACK', acknowledge=False)
        e.on_payload_result(True, first)
        h.step(acknowledge=False)
        self.assertEqual(e.phase, 'WAIT_ACK')
        self.assertEqual(e.route_released_points, {0})
        e.hold('user hold')
        h.step(dt=60, acknowledge=False)
        self.assertEqual(e.state, 'HOLD')
        self.assertEqual(e.route_released_points, {0})
        self.assertEqual([x['slot'] for x in h.effects if x['type'] == 'release'], [1, 2])
        self.assertEqual(DropPointsReplay(3).engine.route_released_points, set())

    def test_no_images_keeps_existing_fallback_and_records_each_point_separately(self):
        points = document(2)
        points['points'] = [[2., 3.], [2., 3.]]
        h = DropPointsReplay(points=points); e = h.engine
        h.until(lambda e: e.phase == 'LANDING', observe=False)
        self.assertEqual(e.route_released_points, {0, 1})
        self.assertEqual(e.route_released_slots, [1, 2])
        self.assertTrue(e._delivery_fallback_active)
        self.assertEqual(e.native_fixes, [])

    def test_recorder_undo_keeps_physical_order_and_selection_is_independent(self):
        recorder = DropPointRecorder()
        self.assertEqual(recorder.key(' ')['kind'], 'ignored')
        for i in range(4):
            recorder.observe_xy(i, -i)
            self.assertEqual(recorder.key(' ')['index'], i+1)
        self.assertEqual(recorder.key(' ')['kind'], 'ignored')
        self.assertEqual(recorder.key('x')['index'], 4)
        recorder.observe_xy(8, 9); recorder.key(' ')
        self.assertEqual(recorder.to_dict()['points'][-1], [8., 9.])
        chosen = select_points(recorder.to_dict(), 2)
        self.assertEqual(chosen['points'], [[0., 0.], [1., -1.]])
        self.assertEqual(len(recorder.to_dict()['points']), 4)
        self.assertEqual(recorder.key('q')['kind'], 'save_exit')

    def test_invalid_files_cannot_turn_into_flight_targets(self):
        for value in ({}, document(5), dict(document(), frame='other'),
                      dict(document(), points=[[float('nan'), 2]])):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_points(value)
        for value, count in ((document(0), None), (document(1), 2)):
            with self.assertRaises(ValueError):
                select_points(value, count)


if __name__ == '__main__':
    unittest.main()
