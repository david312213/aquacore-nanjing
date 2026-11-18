"""One shared route, including migration of legacy split files; no live aircraft."""
from copy import deepcopy
import contextlib
import io
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from airseafly_real400_mission.guided_route import (
    FRAME, FreeRouteRecorder, RecorderKeyDecoder,
    load_route, save_route, route_recorder, validate_route,
)
from airseafly_real400_mission.native_delivery_route import (
    DeliveryRouteEngine, build_itinerary, describe_laps, plan_fixed_legs,
)
from airseafly_real400_mission import delivery_route_cli as cli
from test_native_fixed_delivery_route import FixedReplay, fixed_configuration, free_document
from test_native_delivery_route import Replay, route_document


def split_document():
    """Raw legacy input: deliberately different lists to detect accidental reuse."""
    route = validate_route(dict(version=2, frame=FRAME, mode='free', home=[-2., -2.],
        route=[dict(kind='transit', xy=[-1., .5]), dict(kind='drop', xy=[1., .5], drop_slot=1),
               dict(kind='gate', xy=[2., 1.]), dict(kind='gate', xy=[3., 1.]),
               dict(kind='drop', xy=[4., 1.], drop_slot=2),
               dict(kind='drop', xy=[3., 3.], drop_slot=3),
               dict(kind='drop', xy=[1., 3.], drop_slot=4),
               dict(kind='transit', xy=[-10., 9.])]), require_complete=True)
    return dict(route, version=3, mode='split',
        loaded_route=[dict(stage='transit', kind='transit', xy=p)
                      for p in ([0., 0.], [4., 0.], [4., 4.], [0., 4.])])


def split_config(laps=2):
    """Keep legacy fixture API for the local-retiming integration regressions."""
    config = fixed_configuration(gate=.6)
    config['delivery_route'] = split_document()
    config['delivery_route_options']['laps'] = laps-1  # cruise laps before the dropping lap
    config['delivery_route_options']['landing_alignment']['enabled'] = False
    config['planning'].update(max_velocity_mps=2., max_acceleration_mps2=2.)
    return config


class SharedRecordingTests(unittest.TestCase):
    def test_legacy_conversion_preserves_route_home_and_input_without_loaded_list(self):
        original = split_document()
        for loaded in (original['loaded_route'], [], None, 'obsolete'):
            doc = dict(original, loaded_route=loaded)
            before = deepcopy(doc)
            actual = validate_route(doc, require_complete=True)
            self.assertEqual(actual, dict(version=2, mode='free', frame=FRAME,
                                         home=original['home'], route=original['route']))
            self.assertEqual(doc, before)
        doc = dict(original); del doc['loaded_route']
        self.assertEqual(validate_route(doc), validate_route(original))
        for doc in (free_document(), route_document()):
            self.assertEqual(route_recorder(doc).to_dict(), validate_route(doc))

    def test_partial_legacy_resume_uses_one_list_and_does_not_invent_home(self):
        doc = split_document(); doc['home'] = None
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'partial.yaml'
            path.write_text(json.dumps(doc))
            recorder = route_recorder(load_route(path))
            self.assertIsInstance(recorder, FreeRouteRecorder)
            self.assertIsNone(recorder.latest_xy)
            self.assertFalse(recorder.complete)
            self.assertEqual(recorder.key(' ')['kind'], 'ignored')
            self.assertEqual(recorder.key('x')['point'], doc['route'][-1])
            recorder.observe_xy(8., 9.); recorder.key('h')
            recorder.key('x'); recorder.key('3'); recorder.key(' ')
            save_route(path, recorder.to_dict())
            result = load_route(path, require_complete=True)
            self.assertEqual(result['version'], 2)
            self.assertNotIn('loaded_route', result)
            self.assertEqual(result['home'], [8., 9.])
            self.assertEqual([p['drop_slot'] for p in result['route'] if p['kind']=='drop'], [1, 2, 3, 4])

    def test_decoder_ignores_removed_route_switch_keys_and_pasted_commands(self):
        decoder = RecorderKeyDecoder()
        self.assertEqual(decoder.feed(b'bDh12 3xq'), ['h', '1', '2', ' ', '3', 'x', 'q'])
        self.assertEqual(decoder.feed(b'\x1b[200~h3 \x1b[201~b'), [])
        recorder = FreeRouteRecorder(); before = recorder.to_dict()
        self.assertEqual(recorder.key('b')['kind'], 'ignored')
        self.assertEqual(recorder.key('d')['kind'], 'ignored')
        self.assertEqual(recorder.to_dict(), before)

    def test_real_shell_default_saves_one_list_and_resumes_without_second_recording(self):
        with tempfile.TemporaryDirectory(prefix='shared route ') as folder:
            path = Path(folder)/'route.yaml'
            command = ['sh', str(ROOT/'flight/record_route.sh'), '--shared', '--demo', '--stdin']
            env = dict(os.environ, AIRSEAFLY_PYTHON=sys.executable)
            run = subprocess.run(command+['--output', str(path)], env=env, text=True, capture_output=True,
                input='xy 0 0\nh\nxy 2 0\nspace\n2\nxy 2 2\nspace\n3\nxy 1 1\nspace\nq\n')
            self.assertEqual(run.returncode, 0, run.stderr)
            document = load_route(path, require_complete=True)
            self.assertEqual((document['version'], document['mode']), (2, 'free'))
            self.assertNotIn('loaded_route', document)
            self.assertEqual([p['kind'] for p in document['route']], ['transit', 'gate', 'drop'])
            self.assertIn('绕圈和投放只录一套点', run.stdout)
            self.assertNotIn('双路线', run.stdout)
            self.assertIn('返航：1.普通 → HOME 降落', run.stdout)
            run = subprocess.run(command+['--resume', str(path)], env=env, text=True, capture_output=True,
                input='x\n3\nxy 3 1\nspace\nq\n')
            self.assertEqual(run.returncode, 0, run.stderr)
            result = load_route(path, require_complete=True)
            self.assertEqual(result['route'][:2], document['route'][:2])
            self.assertEqual(result['route'][-1]['drop_slot'], 1)
            self.assertEqual(result['route'][-1]['xy'], [3., 1.])

    def test_real_shell_resumes_raw_legacy_file_and_saves_shared_points(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'legacy.yaml'; original = split_document()
            path.write_text(json.dumps(original))
            run = subprocess.run(['sh', str(ROOT/'flight/record_route.sh'), '--demo', '--stdin',
                                  '--resume', str(path)], text=True, capture_output=True, input='q\n',
                                 env=dict(os.environ, AIRSEAFLY_PYTHON=sys.executable))
            self.assertEqual(run.returncode, 0, run.stderr)
            self.assertEqual(load_route(path), validate_route(original))
            self.assertNotIn('loaded_route', path.read_text())


class SharedItineraryTests(unittest.TestCase):
    def test_all_laps_use_identical_points_and_close_before_home(self):
        doc = split_document(); before = deepcopy(doc)
        points = [p['xy'] for p in doc['route']]
        for laps in (1, 2, 9):
            with self.subTest(laps=laps):
                steps = build_itinerary(doc, laps-1)
                nav = [s for s in steps if s['kind'] == 'navigation']
                expected = [points[0]]+(points[1:]+[points[0]])*laps+[doc['home']]
                self.assertEqual([s['xy'] for s in nav], expected)
                self.assertEqual(steps, build_itinerary(validate_route(doc), laps-1))
                self.assertEqual([s['completes_lap'] for s in nav if s['closing']], list(range(1, laps+1)))
                drops = [s for s in steps if s['kind'] == 'drop']
                self.assertEqual([s['route_index'] for s in drops], [1, 4, 5, 6])
                self.assertEqual([s['drop_slot'] for s in drops], [1, 2, 3, 4])
                self.assertTrue(all(s['lap'] == laps and s['route_source'] == 'shared' for s in drops))
                self.assertTrue(all(s['route_source'] in ('shared', 'home') for s in steps))
                self.assertEqual(steps[-3]['completes_lap'], laps)
                self.assertEqual(steps[-3]['route_index'], 0)
                self.assertEqual(steps[-2]['stage'], 'home')
                self.assertIn('（闭环）', describe_laps(steps)[-2])
                self.assertEqual(describe_laps(steps)[-1], '返航：1.普通 → HOME 降落')
        self.assertEqual(doc, before)

    def test_empty_legacy_loaded_list_does_not_block_preview_or_submission(self):
        doc = split_document(); doc['loaded_route'] = []
        config = split_config(); config['delivery_route'] = doc
        self.assertEqual(DeliveryRouteEngine(config).route_document, validate_route(doc))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'route.yaml'; path.write_text(json.dumps(doc))
            for dry in ([], ['--dry-run']):
                with patch.object(cli, 'resolve_config', return_value=deepcopy(config)), \
                     patch.object(cli, 'manager_client') as client, \
                     patch.object(cli, 'request', return_value=dict(ok=True, event_start=0)) as rpc, \
                     contextlib.redirect_stdout(io.StringIO()):
                    client.return_value.watch.return_value = 0
                    self.assertEqual(cli.main(['--route', str(path)]+dry), 0)
                    if dry:
                        client.assert_not_called(); rpc.assert_not_called()
                    else:
                        submitted = rpc.call_args.args[1]['resolved_config']['delivery_route']
                        self.assertEqual(submitted, validate_route(doc))

    def test_closures_remain_in_same_fixed_leg_with_no_duplicate_entry_or_stop(self):
        config = split_config(9); doc = config['delivery_route']
        steps = build_itinerary(doc, 8)
        legs = plan_fixed_legs(config, steps, (*doc['home'], 0.))
        self.assertEqual(len(legs), 5)
        first = legs[0]['trajectory']
        closures = [ev for ev in first.events if steps[ev['itinerary_index']].get('closing')]
        self.assertEqual(len(closures), 8)
        for event in closures:
            self.assertGreater(math.dist(first.sample(event['elapsed_s'])['velocity'], (0., 0., 0.)), .01)
            next_step = steps[event['itinerary_index']+1]
            self.assertEqual(next_step['route_index'], 1)
        first_drop = next(i for i,s in enumerate(steps) if s['kind']=='drop')
        self.assertEqual(legs[0]['last']+1, first_drop)

    def test_first_point_drop_uses_closure_without_duplicate_navigation(self):
        config = split_config(2)
        config['delivery_route']['route'].pop(0)
        steps = build_itinerary(config['delivery_route'], 1)
        closure = next(i for i,s in enumerate(steps) if s.get('closing'))
        self.assertEqual(steps[closure+1]['kind'], 'drop')
        self.assertEqual(steps[closure+1]['route_index'], 0)
        self.assertEqual(steps[closure]['xy'], steps[closure+1]['xy'])
        replay = FixedReplay(config)
        replay.until(lambda e: e.phase == 'LANDING', limit=40000)
        self.assertEqual([v['slot'] for v in replay.effects if v['type']=='release'], [1,2,3,4])

    def test_timing_preview_matches_shared_events_without_separate_handoff(self):
        for laps in (1, 2, 9):
            config = split_config(laps); before = deepcopy(config)
            value = cli.preview(config, config['delivery_route'])
            timing = value['fixed_legs']['navigation_timing']
            self.assertEqual(len(timing['loaded_laps']), laps-1)
            self.assertEqual(timing['handoff_s'], 0.)
            parts = sum(timing[k] for k in ('entry_s', 'delivery_navigation_s', 'return_home_s'))
            self.assertAlmostEqual(parts+sum(x['duration_s'] for x in timing['loaded_laps']), timing['total_navigation_s'])
            self.assertAlmostEqual(timing['total_navigation_s'], value['fixed_legs']['flight_time_s'])
            self.assertEqual(value['recording_mode'], 'free')
            self.assertEqual(value['loaded_route_points'], value['route_points'])
            stream = io.StringIO()
            with contextlib.redirect_stdout(stream): cli.show_preview(value)
            self.assertIn('绕圈和投放共用同一套航点', stream.getvalue())
            self.assertNotIn('独立绕圈路线', stream.getvalue())
            self.assertNotIn('独立投放路线', stream.getvalue())
            self.assertEqual(config, before)

    def test_status_indexes_the_same_list_in_both_stages(self):
        config = split_config(); config['delivery_route']['loaded_route'] *= 3
        engine = DeliveryRouteEngine(config)
        for lap in (1, 2):
            target = next(i for i,s in enumerate(engine.itinerary)
                          if s['lap']==lap and s['kind']=='navigation' and s['route_index']==6)
            engine._select_step(target)
            status = engine.snapshot()['delivery_route']
            self.assertEqual(status['route_source'], 'shared')
            self.assertEqual((status['total_points'], status['loaded_points'], status['delivery_points']), (8,8,8))
            self.assertEqual(status['current_point'], config['delivery_route']['route'][6])

    def test_replay_four_releases_on_last_lap_hold_retains_records_new_task_clears(self):
        for laps in (1, 2, 9):
            h = FixedReplay(split_config(laps))
            h.until(lambda e: e.phase == 'LANDING', limit=40000)
            self.assertEqual([v['slot'] for v in h.effects if v['type']=='release'], [1,2,3,4])
            self.assertEqual(h.engine.route_released_points, {1,4,5,6})
            self.assertEqual((h.engine.completed_laps, len(h.legs)), (laps,5))
            self.assertEqual(h.legs[-1].end[:2], tuple(h.engine.route_home))
            self.assertTrue(all(v['route_source']=='shared' for v in h.effects
                                if v['type']=='delivery_route_release_ack'))
            h.engine.hold('operator hold')
            self.assertEqual(h.engine.route_released_points, {1,4,5,6})
            self.assertIsNone(h.engine.fixed_trajectory)
            self.assertEqual(DeliveryRouteEngine(split_config(laps)).route_released_points, set())

    def test_ego_fallback_also_reuses_shared_order(self):
        config = split_config(); config['navigation']['backend'] = 'ego'
        h = Replay(config); h.until(lambda e: e.phase == 'LANDING', limit=20000)
        self.assertEqual(h.engine.route_released_points, {1,4,5,6})
        self.assertEqual(h.engine.completed_laps, 2)
        self.assertEqual(h.engine.itinerary, build_itinerary(validate_route(config['delivery_route']), 1))

    def test_stale_duplicate_ack_and_hold_do_not_repeat_release(self):
        h = FixedReplay(split_config())
        h.until(lambda e: e.phase == 'WAIT_ACK', acknowledge=False, limit=20000)
        engine = h.engine; request = engine._payload_request
        engine.on_payload_result(True, 'wrong-request'); h.step(dt=120., acknowledge=False)
        self.assertEqual((engine.phase, engine.route_released_points), ('WAIT_ACK', set()))
        self.assertEqual(sum(v['type']=='release' for v in h.effects), 1)
        engine.on_payload_result(True, request); engine.on_payload_result(True, request)
        h.step(acknowledge=False)
        self.assertEqual(engine.route_released_points, {1})
        h.until(lambda e: e.phase == 'WAIT_ACK', acknowledge=False, limit=20000)
        engine.on_payload_result(True, request); h.step(acknowledge=False)
        self.assertEqual((engine.phase, engine.route_released_points), ('WAIT_ACK', {1}))
        engine.hold('operator hold')
        for _ in range(5): h.step(acknowledge=False)
        self.assertEqual(engine.route_released_points, {1})
        self.assertEqual(sum(v['type']=='release' for v in h.effects), 2)


    def test_return_leg_closes_at_first_point_with_original_timing(self):
        config = split_config(2); doc = config['delivery_route']
        steps = build_itinerary(doc, 1)
        legs = plan_fixed_legs(config, steps, (*doc['home'], 0.))
        last = legs[-1]
        self.assertEqual(steps[last['last']+1]['kind'], 'land')
        indices = [ev['itinerary_index'] for ev in last['trajectory'].events]
        self.assertTrue(steps[indices[-2]]['closing'])
        self.assertEqual(steps[indices[-2]]['route_index'], 0)
        self.assertEqual(steps[indices[-1]]['stage'], 'home')
        self.assertFalse((last['trajectory'].summary().get('local_retiming') or {}).get('applied'))

    def test_preview_html_shows_every_leg_and_return_from_first_point(self):
        from airseafly_real400_mission.delivery_route_html import delivery_preview_data, write_delivery_preview_html
        config = split_config(2); doc = config['delivery_route']
        value = delivery_preview_data(config, doc)
        self.assertEqual([leg['ends'] for leg in value['legs']], ['drop']*4+['land'])
        self.assertEqual(value['order'][-1], '返航：1.普通 → HOME 降落')
        self.assertEqual({s['cat'] for s in value['samples']}, {'entry', 'loaded', 'delivery', 'home'})
        self.assertAlmostEqual(value['samples'][-1]['t'], value['duration_s'])
        self.assertAlmostEqual(math.dist(value['samples'][-1]['p'][:2], doc['home']), 0., places=3)
        with tempfile.TemporaryDirectory() as folder:
            text = Path(write_delivery_preview_html(Path(folder)/'x.html', config, doc)).read_text()
        self.assertIn('比赛航线', text)
        self.assertNotIn('__DATA__', text)


if __name__ == '__main__':
    unittest.main()
