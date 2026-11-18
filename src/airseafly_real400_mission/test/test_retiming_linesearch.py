"""Thrust timing must not fall back to a uniform slow-down on field routes (2026-11-19).

Both fixtures are the base spatial curves of real flights where SLSQP stopped
with "Positive directional derivative for linesearch" from the uniform
start. Before the restart fix the whole leg was slowed uniformly and flew
slower than the original timing (02:27: 34.9 -> 41.8 s; 04:14: 44.6 -> 52.7 s).
Pure numpy/SciPy: no MINCO library, ROS or hardware is needed.
"""
import json
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from airseafly_real400_mission import local_retiming
from airseafly_real400_mission.fixed_trajectory import FixedTrajectory
from airseafly_real400_mission.native_delivery_route import build_itinerary
from test_local_retiming_dynamics import budgets

FIXTURE = Path(__file__).resolve().parent/'fixtures/retiming_linesearch_20261119.json'
# Field values at the time of both flights (flight.yaml planning.retiming).
DYN = local_retiming.dynamics_limits({'retiming': {'feedforward_tilt_deg': 30, 'tilt_rate_dps': 120,
                                                   'drag_per_s': .25, 'max_thrust_g': 1.3}})
ORIGINAL_S = {'waypoints_20261119_0227': 34.85, 'delivery_leg1_20261119_0414': 44.55}


def case(name):
    value = json.loads(FIXTURE.read_text(encoding='utf-8'))[name]
    trajectory = FixedTrajectory.from_dict(value['trajectory'])
    if 'groups' in value:
        groups = [tuple(group) for group in value['groups']]
    else:
        steps = build_itinerary(value['route'], value['laps'])
        groups = [(steps[event['itinerary_index']].get('route_source', 'shared'),
                   steps[event['itinerary_index']]['route_index']) for event in trajectory.events]
    limits = value['limits']
    return trajectory, value['caps'], groups, limits['max_velocity_mps'], limits['max_acceleration_mps2']


class LineSearchRestartTests(unittest.TestCase):
    def check(self, name):
        trajectory, caps, groups, vmax, amax = case(name)
        result = local_retiming.retime_loaded_leg(trajectory, caps, groups, vmax, amax, DYN)
        detail = result.summary()['local_retiming']
        self.assertTrue(detail['applied'], detail)
        self.assertNotIn('solver_fallback', detail)
        self.assertTrue(detail['solver_converged'], detail)
        self.assertFalse(detail.get('slower_than_original', False))
        self.assertLess(result.duration, ORIGINAL_S[name])
        self.assertTrue(detail['spatial_curve_unchanged'])
        # Independent runtime check: budgets hold on the sampled reference.
        value = budgets(result, DYN)
        self.assertLessEqual(value['tilt'], DYN['feedforward_tilt_deg']*1.01)
        self.assertLessEqual(value['rate'], DYN['tilt_rate_dps']*1.01)
        self.assertLessEqual(value['thrust'], DYN['max_thrust_g']*1.01)
        self.assertLessEqual(value['speed'], vmax*1.001)
        self.assertLessEqual(value['acceleration'], amax*1.001)
        # The same recorded events are kept, in order, at increasing times.
        self.assertEqual([e['node_index'] for e in result.events], [e['node_index'] for e in trajectory.events])
        self.assertTrue(np.all(np.diff([e['elapsed_s'] for e in result.events]) > 0))
        return detail

    def test_ordinary_route_with_entry_point_on_takeoff_spot(self):
        self.check('waypoints_20261119_0227')

    def test_contest_leg_with_gates_and_delivery_loop(self):
        detail = self.check('delivery_leg1_20261119_0414')
        self.assertGreaterEqual(detail['solver_retries'], 1)

    def test_optimized_mode_is_unchanged(self):
        # No thrust budgets: the restart path is never taken.
        trajectory, caps, groups, vmax, amax = case('delivery_leg1_20261119_0414')
        detail = local_retiming.retime_loaded_leg(trajectory, caps, groups, vmax, amax, None).summary()['local_retiming']
        self.assertEqual(detail['solver_retries'], 0)
        self.assertNotIn('solver_fallback', detail)


if __name__ == '__main__':
    unittest.main()
