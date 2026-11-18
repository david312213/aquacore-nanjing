"""Entry points, a cruise loop, a separate delivery loop and return points.

Offline only: recording, itinerary, fixed legs and engine replays without ROS,
services, cameras or aircraft.
"""
from copy import deepcopy
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
SCRIPT = PACKAGE/'scripts/record_delivery_route.py'
WAYPOINT_SCRIPT = PACKAGE/'scripts/record_waypoints.py'
sys.path.insert(0, str(PACKAGE/'src'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from airseafly_real400_mission.guided_route import (
    FRAME, LoopRouteRecorder, load_route, route_recorder, save_route, validate_route,
)
from airseafly_real400_mission.native_delivery_route import (
    DeliveryRouteEngine, build_itinerary, describe_laps, plan_fixed_legs,
)
from airseafly_real400_mission import fixed_trajectory
from airseafly_real400_mission.waypoint_route import WaypointRecorder, load_route as load_waypoints
from airseafly_real400_mission.native_waypoint_route import WaypointRouteEngine
import test_native_delivery_route as ego_delivery
import test_native_fixed_delivery_route as fixed_delivery
import test_native_fixed_waypoint_route as fixed_waypoints
import test_native_waypoint_route as ego_waypoints


def point(kind, x, y, slot=None):
    value = dict(stage=kind if slot is None else 'drop%d' % slot, kind=kind, xy=[float(x), float(y)])
    if slot is not None:
        value['drop_slot'] = slot
    return value


def loop_document():
    return dict(version=4, frame=FRAME, mode='loops', home=[0., 0.],
                entry=[point('transit', 0, 1), point('transit', 0, 2), point('gate', 1, 3)],
                cruise_loop=[point('transit', 3, 3), point('gate', 3, 0), point('transit', 1, 0)],
                delivery_loop=[point('transit', 2, 2), point('drop', 2.5, 2, 1), point('drop', 3, 2, 2),
                               point('transit', 3.5, 1), point('drop', 3.6, .5, 3)],
                recording=dict(section='done'),
                **{'return': [point('transit', 1, 1)]})


def recorder(keys, document=None):
    r = LoopRouteRecorder(document)
    events = []
    for xy, key in keys:
        if xy is not None:
            r.observe_xy(*xy)
        events.append(r.key(key))
    return r, events


class LoopFormatTests(unittest.TestCase):
    def test_validate_and_save_round_trip(self):
        document = validate_route(loop_document(), require_complete=True)
        self.assertEqual(document, validate_route(document))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'route.yaml'
            save_route(path, document)
            self.assertEqual(load_route(path, require_complete=True), document)
            self.assertIsInstance(route_recorder(load_route(path)), LoopRouteRecorder)

    def test_empty_sections_are_allowed_and_only_home_and_drops_are_required(self):
        minimal = dict(version=4, frame=FRAME, mode='loops', home=[0., 0.],
                       delivery_loop=[point('drop', 1, 1, 1)])
        document = validate_route(minimal, require_complete=True)
        self.assertEqual((document['entry'], document['cruise_loop'], document['return']), ([], [], []))
        for change in (dict(home=None), dict(delivery_loop=[point('transit', 1, 1)])):
            with self.assertRaises(ValueError):
                validate_route(dict(minimal, **change), require_complete=True)
            validate_route(dict(minimal, **change))  # Partial saves stay valid.

    def test_drops_only_in_delivery_loop_numbered_in_order(self):
        base = loop_document()
        for section in ('entry', 'cruise_loop', 'return'):
            with self.assertRaisesRegex(ValueError, 'kind'):
                validate_route(dict(base, **{section: [point('drop', 1, 1, 1)]}))
        wrong = deepcopy(base)
        wrong['delivery_loop'][1]['drop_slot'] = 2
        with self.assertRaisesRegex(ValueError, '1～4'):
            validate_route(wrong)


class LoopRecorderTests(unittest.TestCase):
    def test_entry_moves_on_after_three_then_enter_ends_each_section(self):
        r, events = recorder([((0, 0), 'h'), ((0, 1), ' '), ((0, 2), ' '), ((1, 3), ' '),
                              ((3, 3), ' '), (None, '\n'), ((2, 2), ' '), (None, '3'), ((2.5, 2), ' '),
                              (None, '\n'), ((1, 1), ' '), (None, '\n')])
        self.assertEqual(events[3]['next_section'], 'cruise_loop')
        d = r.to_dict()
        self.assertEqual([len(d[s]) for s in ('entry', 'cruise_loop', 'delivery_loop', 'return')], [3, 1, 2, 1])
        self.assertEqual(d['delivery_loop'][1]['drop_slot'], 1)
        self.assertEqual(d['recording']['section'], 'done')
        self.assertTrue(r.complete)
        self.assertEqual(r.key(' ')['kind'], 'ignored')

    def test_enter_ends_entry_early_and_drop_only_in_delivery_loop(self):
        r, events = recorder([((0, 1), ' '), (None, '\n'), (None, '3')])
        self.assertEqual((len(r.to_dict()['entry']), r.section), (1, 'cruise_loop'))
        self.assertEqual(events[-1]['kind'], 'ignored')
        self.assertEqual(r.selected_kind, 'transit')

    def test_undo_crosses_back_into_the_previous_section(self):
        r, _ = recorder([((0, 1), ' '), ((0, 2), ' '), ((0, 3), ' '), ((5, 5), ' ')])
        self.assertEqual(r.key('x')['section'], 'cruise_loop')
        self.assertEqual(r.key('x')['section'], 'entry')
        self.assertEqual(r.section, 'entry')
        r.observe_xy(9, 9)
        self.assertEqual(r.key(' ')['next_section'], 'cruise_loop')
        self.assertEqual(r.to_dict()['entry'][-1]['xy'], [9., 9.])

    def test_stdin_demo_records_and_resumes_loop_file(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/'route.yaml'
            first = subprocess.run([sys.executable, str(SCRIPT), '--demo', '--stdin', '--output', str(output)],
                input='xy 0 0\nh\nxy 0 1\nspace\nxy 0 2\nspace\nq\n', capture_output=True, text=True, timeout=10)
            self.assertEqual(first.returncode, 0, first.stderr)
            self.assertIn('暂不能飞行', first.stdout)
            second = subprocess.run([sys.executable, str(SCRIPT), '--demo', '--stdin', '--resume', str(output)],
                input='xy 1 3\nspace\nxy 3 3\nspace\nxy 3 0\nspace\nenter\n3\nxy 2 2\nspace\nenter\nenter\nq\n',
                capture_output=True, text=True, timeout=30)
            self.assertEqual(second.returncode, 0, second.stderr)
            self.assertIn('可以飞行', second.stdout)
            self.assertIn('进场点：1.普通 → 2.普通 → 3.普通', second.stdout)
            self.assertIn('从 2.普通 直接进入投放环', second.stdout)
            self.assertIn('投放环：1.投放1[投]', second.stdout)
            result = load_route(output, require_complete=True)
            self.assertEqual([len(result[s]) for s in ('entry', 'cruise_loop', 'delivery_loop', 'return')],
                             [3, 2, 1, 0])


class LoopItineraryTests(unittest.TestCase):
    def test_order_entry_then_cruise_laps_then_delivery_then_return(self):
        document = loop_document()
        for laps in (0, 1, 3):
            with self.subTest(laps=laps):
                steps = build_itinerary(document, laps)
                navigation = [s['xy'] for s in steps if s['kind'] == 'navigation']
                entry = [p['xy'] for p in document['entry']]
                cruise = [p['xy'] for p in document['cruise_loop']]
                delivery = [p['xy'] for p in document['delivery_loop']]
                # Earlier laps close on the first point; the last lap leaves
                # from the loop's last point straight to the delivery loop.
                self.assertEqual(navigation, entry+cruise*laps+delivery+[[1., 1.], document['home']])
                self.assertEqual([s['completes_lap'] for s in steps if s.get('closing')], list(range(1, laps)))
                cruise_done = [s['completes_lap'] for s in steps
                               if s.get('route_source') == 'cruise_loop' and s.get('completes_lap')]
                self.assertEqual(cruise_done, list(range(1, laps+1)))
                if laps:
                    last = max(i for i, s in enumerate(steps) if s.get('route_source') == 'cruise_loop')
                    self.assertEqual((steps[last]['xy'], steps[last].get('closing')), (cruise[-1], False))
                    self.assertEqual((steps[last+1]['route_source'], steps[last+1]['route_index']), ('delivery_loop', 0))
                drops = [s for s in steps if s['kind'] == 'drop']
                self.assertEqual([s['drop_slot'] for s in drops], [1, 2, 3])
                self.assertTrue(all(s['lap'] == laps+1 and s['route_source'] == 'delivery_loop' for s in drops))
                self.assertEqual([s['route_index'] for s in drops], [1, 2, 4])
                self.assertEqual(steps[-1]['kind'], 'land')
                self.assertTrue(all(s['point']['xy'] == s['xy'] for s in steps if 'point' in s))

    def test_flight_order_text_lists_each_section_once(self):
        lines = describe_laps(build_itinerary(loop_document(), 4))
        self.assertEqual(lines, ['进场点：1.普通 → 2.普通 → 3.圆环',
                                 '巡航闭环 ×4圈：1.普通 → 2.圆环 → 3.普通（前 3 圈回到 1.普通 闭环；第 4 圈从 3.普通 直接进入投放环）',
                                 '投放环：1.普通 → 2.投放1[投] → 3.投放2[投] → 4.普通 → 5.投放3[投]',
                                 '返航点：1.普通', '→ HOME 降落'])

    def test_first_fixed_leg_spans_entry_and_all_cruise_laps(self):
        try:
            fixed_trajectory._load_library()
        except RuntimeError as exc:
            self.skipTest(str(exc))
        config = fixed_delivery.fixed_configuration()
        config['delivery_route'] = loop_document()
        config['delivery_route_options']['laps'] = 2
        steps = build_itinerary(config['delivery_route'], 2)
        legs = plan_fixed_legs(config, steps, (0., 0., 0.))
        self.assertEqual([steps[leg['last']+1]['kind'] for leg in legs], ['drop', 'drop', 'drop', 'land'])
        first = [steps[e['itinerary_index']] for e in legs[0]['trajectory'].events]
        self.assertEqual({s['route_source'] for s in first}, {'entry', 'cruise_loop', 'delivery_loop'})
        self.assertEqual(sum(bool(s.get('closing')) for s in first), 1)
        # Retiming still applies to the cruise leg without the last closure.
        self.assertIn('local_retiming', legs[0]['trajectory'].summary())
        one = plan_fixed_legs(config, build_itinerary(config['delivery_route'], 1), (0., 0., 0.))
        self.assertIn('local_retiming', one[0]['trajectory'].summary())
        self.assertEqual(steps[legs[-1]['first']]['route_source'], 'return')


class LoopEngineTests(unittest.TestCase):
    def configuration(self, fixed, laps=2):
        config = fixed_delivery.fixed_configuration() if fixed else ego_delivery.configuration()
        config['delivery_route'] = loop_document()
        config['delivery_route_options']['laps'] = laps
        return config

    def test_three_releases_in_delivery_loop_then_home_in_both_modes(self):
        for fixed in (False, True):
            with self.subTest(fixed=fixed):
                config = self.configuration(fixed)
                h = fixed_delivery.FixedReplay(config) if fixed else ego_delivery.Replay(config)
                stages = []
                for _ in range(60000):
                    if h.engine.phase == 'LANDING':
                        break
                    stage = h.engine.snapshot()['delivery_route']['stage']
                    if not stages or stages[-1] != stage:
                        stages.append(stage)
                    h.step()
                e = h.engine
                self.assertEqual(e.phase, 'LANDING')
                self.assertEqual([v['slot'] for v in h.effects if v['type'] == 'release'], [1, 2, 3])
                self.assertEqual(e.route_released_points, {1, 2, 4})
                self.assertEqual(e.completed_laps, 3)
                self.assertEqual(e.snapshot()['payload_total'], 3)
                for stage in ('entry', 'loaded_lap', 'delivery', 'return'):
                    self.assertIn(stage, stages)
                self.assertLess(stages.index('entry'), stages.index('loaded_lap'))
                self.assertLess(stages.index('loaded_lap'), stages.index('delivery'))
                navigation = [v for v in h.effects if v['type'] == 'delivery_route_point']
                expected = [s for s in e.itinerary if s['kind'] == 'navigation']
                self.assertEqual([v['position_xyz'][:2] for v in navigation], [s['xy'] for s in expected])
                if fixed:
                    self.assertEqual(len(h.legs), 4)

    def test_snapshot_reports_section_point(self):
        e = DeliveryRouteEngine(self.configuration(False))
        e._select_step(1)
        snapshot = e.snapshot()['delivery_route']
        self.assertEqual((snapshot['route_source'], snapshot['index'], snapshot['total_points']), ('entry', 1, 3))
        self.assertEqual(snapshot['current_point']['xy'], [0., 2.])


class WaypointEntryTests(unittest.TestCase):
    def test_recorder_starts_with_entry_and_moves_on_after_three(self):
        r = WaypointRecorder()
        for i in range(4):
            r.observe_xy(i, 0)
            event = r.key(' ')
        self.assertEqual((r.document['entry_points'], r.document['points']),
                         ([[0., 0.], [1., 0.], [2., 0.]], [[3., 0.]]))
        self.assertEqual(r.document['version'], 3)
        self.assertEqual(event['kind'], 'recorded')
        r.key('2')
        self.assertEqual(r.document['version'], 3)

    def test_stdin_demo_saves_entry_points(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/'waypoints.yaml'
            run = subprocess.run([sys.executable, str(WAYPOINT_SCRIPT), '--demo', '--stdin', '--output', str(output)],
                input='xy 0 1\nspace\nenter\nxy 3 3\nspace\nxy 3 0\nspace\nq\n',
                capture_output=True, text=True, timeout=30)
            self.assertEqual(run.returncode, 0, run.stderr)
            self.assertIn('1 个进场点、2 个航点', run.stdout)
            value = load_waypoints(output)
            self.assertEqual((value['entry_points'], value['points']), ([[0., 1.]], [[3., 3.], [3., 0.]]))

    def test_fixed_trajectory_passes_entries_once_then_laps(self):
        try:
            fixed_trajectory._load_library()
        except RuntimeError as exc:
            self.skipTest(str(exc))
        trajectory = fixed_trajectory.build_fixed_trajectory(
            (0., 0., 1.), [(3., 3., 1.), (3., 0., 1.), (1., 0., 1.)], 2, 1., 1.,
            entry_xyz=[(0., 1., 1.), (0., 2., 1.)])
        events = trajectory.events
        self.assertEqual([e.get('entry_index') for e in events[:2]], [0, 1])
        self.assertTrue(all('waypoint_index' in e for e in events[2:]))
        self.assertEqual(sum(e['waypoint_index'] == 0 for e in events[2:]), 3)
        self.assertEqual(events[-1]['completed_laps'], 2)

    def test_fixed_engine_reports_entry_then_loop(self):
        try:
            fixed_trajectory._load_library()
        except RuntimeError as exc:
            self.skipTest(str(exc))
        h = fixed_waypoints.Replay([[3., 3.], [3., 0.]])
        config = fixed_waypoints.configuration([[3., 3.], [3., 0.]], ['cruise', 'cruise'])
        config['waypoint_route'].update(version=3, entry_points=[[0., 1.]], entry_profiles=['gate'])
        h.engine = WaypointRouteEngine(config)
        h.engine.start('waypoint_route', h.position, h.yaw, h.now)
        h.engine.native_origin = h.ground
        h.engine.begin_native_flight()
        h.begin_route()
        e = h.engine
        self.assertEqual(e.snapshot()['waypoint_route']['section'], 'entry')
        entry = next(ev for ev in e.fixed_trajectory.events if 'entry_index' in ev)
        h.progress(entry['elapsed_s'])
        self.assertIn('waypoint_route_entry_passed', [v['type'] for v in h.effects])
        self.assertEqual((e.route_entry_index, e.route_index), (None, 0))
        h.progress(e.fixed_trajectory.duration)
        self.assertEqual(e.fixed_completed_laps, 2)

    def test_ego_visits_entries_then_loop_points(self):
        config = ego_waypoints.configuration([[4., 5.], [6., 5.]])
        config['waypoint_route'].update(version=3, point_profiles=['cruise', 'cruise'],
                                        entry_points=[[1., 1.], [2., 2.]], entry_profiles=['cruise', 'cruise'])
        with mock.patch.object(ego_waypoints, 'configuration', return_value=config):
            h = ego_waypoints.Replay(None)
        h.until(lambda e: e.done)
        goals = [v for v in h.effects if v['type'] == 'goal']
        self.assertEqual([g["position_xyz"][:2] for g in goals][:4],
                         [[1., 1.], [2., 2.], [4., 5.], [6., 5.]])


if __name__ == '__main__':
    unittest.main()
