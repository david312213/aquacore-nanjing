"""Mixed route replay: schema, keys, per-leg P/Z/inflation and goal binding."""
from copy import deepcopy
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
from test_native_waypoint_route import Replay, configuration
from test_native_waypoint_manager import WaypointManagerTests
from airseafly_real400_mission.waypoint_route import WaypointRecorder, load_route, save_route, validate_route
from airseafly_real400_mission.route_profiles import resolve_profiles
from airseafly_real400_mission.waypoint_route_cli import preview
from airseafly_real400_mission.guided_route import RecorderKeyDecoder


def mixed_config(points):
    c = configuration(points)
    c['waypoint_route'].update(version=2, point_profiles=['cruise', 'gate', 'cruise', 'gate'][:len(points)])
    c['waypoint_profiles'] = dict(cruise=dict(height_m=1., inflation_m=.3), gate=dict(height_m=1.6, inflation_m=.1))
    return c



def loop_recorder():
    """A new recording past its entry section, i.e. the original loop-only flow."""
    recorder = WaypointRecorder()
    recorder.key('\n')
    return recorder


class ProfileTests(unittest.TestCase):
    def test_record_resume_undo_and_keyboard_decoder(self):
        r = loop_recorder()
        r.observe_xy(1, 2)
        for key in RecorderKeyDecoder().feed(b'1 2 1 x2 '):
            r.key(key)
        self.assertEqual(r.to_dict()['point_profiles'], ['cruise', 'gate', 'gate'])
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'route.yaml'
            save_route(p,r.to_dict())
            resumed=WaypointRecorder(load_route(p))
            resumed.key('x'); resumed.key('1'); resumed.observe_xy(3,4); resumed.key(' ')
            self.assertEqual(resumed.to_dict()['point_profiles'], ['cruise','gate','cruise'])
        # Paste/escape sequences cannot silently change the selection.
        self.assertEqual(RecorderKeyDecoder().feed(b'\x1b[200~2 \x1b[201~\x1b[A1'), ['1'])

    def test_legacy_route_defaults_and_invalid_metadata(self):
        c=configuration([[1,2]])
        self.assertEqual(validate_route(c['waypoint_route'])['version'],1)
        bad=dict(c['waypoint_route'], version=2, point_profiles=[])
        with self.assertRaises(ValueError): validate_route(bad)
        bad['point_profiles']=['unknown']
        with self.assertRaises(ValueError): validate_route(bad)

    def test_mixed_heights_preserve_previous_hold_and_one_origin(self):
        points=[[2.,0.],[4.,0.],[6.,0.],[8.,0.]]
        c=mixed_config(points)
        with patch('test_native_waypoint_route.configuration', return_value=c):
            h=Replay(points)
        for index, previous_height in ((0,1.),(1,1.),(2,1.6),(3,1.)):
            h.until(lambda e: e.state=='APPROACH' and e.route_index==index)
            self.assertAlmostEqual(h.engine._navigation_hold[2], h.ground[2]+previous_height)
            self.assertAlmostEqual(h.engine.command_position[2],h.ground[2]+[1.,1.6,1.,1.6][index])
            h.step()
        h.until(lambda e:e.done)
        goals=[e for e in h.effects if e['type']=='goal']
        self.assertEqual([e['map_inflation_m'] for e in goals],[.3,.1,.3,.1])
        self.assertEqual([e['waypoint_profile'] for e in goals],['cruise','gate','cruise','gate'])
        self.assertAlmostEqual(h.engine.command_position[2], h.ground[2]+1.6)
        self.assertFalse(any(e['type'] in ('restart_ego','release','land') for e in h.effects))
        p=preview(c,c['waypoint_route'])
        self.assertEqual([s['height_m'] for s in p['itinerary']], [1.,1.,1.6,1.,1.6,1.6])

    def test_explicit_cli_overrides_and_finite_values(self):
        c=mixed_config([[0,0]])
        c['_overrides']=['flight.cruise_height_m=1.2','planning.map.inflation_m=0.2']
        c['flight']['cruise_height_m']=1.2; c['planning']['map']['inflation_m']=.2
        self.assertEqual(resolve_profiles(c)['cruise'],dict(height_m=1.2,inflation_m=.2))
        self.assertEqual(resolve_profiles(c)['gate'],dict(height_m=1.6,inflation_m=.2))
        c['_overrides']=[]; c['waypoint_profiles']['gate']['height_m']=float('nan')
        with self.assertRaises(ValueError): resolve_profiles(c)


class ProfileManagerTests(WaypointManagerTests):
    def test_mixed_goal_binds_inflation_and_previous_height_reference(self):
        manager,runtime,replay=self.setup_manager()
        c=mixed_config([[2.,0.],[4.,0.]])
        self.dispatch_start(manager,runtime,c)
        self.activate(manager,runtime)
        self.until(manager,runtime,lambda:manager.ego_goal_request is not None)
        self.assertTrue(manager.ego_goal_request.apply_map_inflation)
        self.assertEqual(manager.ego_goal_request.map_inflation_m,.3)
        self.until(manager,runtime,lambda:manager.engine.route_index==1 and manager.engine._goal_sent)
        request=manager.ego_goal_request
        self.assertEqual(request.map_inflation_m,.1)
        self.assertAlmostEqual(request.goal.pose.position.z,replay.ground[2]+1.6)
        self.assertAlmostEqual(request.command.position.z,replay.ground[2]+1.)

if __name__=='__main__': unittest.main()
