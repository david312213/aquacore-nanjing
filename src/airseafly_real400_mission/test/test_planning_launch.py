"""Resolve real ROS launch files without starting nodes or contacting hardware."""
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

try:
    import roslaunch.config
    import rosgraph.names
except ImportError:
    roslaunch = None

ROOT = Path(__file__).resolve().parents[3]
PLANNER = Path('src/planner/plan_manage/launch')
TASK = Path('src/airseafly_real400_mission/launch/ego_worker.launch')
LEGACY = Path('src/airseafly_real400/launch/planner_raw_lio.launch')
KEY = '/drone_0_ego_planner_node/grid_map/obstacles_inflation'


@unittest.skipIf(roslaunch is None, 'ROS launch parser is required; run on NUC')
class PlanningLaunchTests(unittest.TestCase):
    def test_task_uses_native_ego_heading_and_actual_initial_yaw(self):
        for yaw in (.7, -3.12):
            with self.subTest(yaw=yaw):
                config = roslaunch.config.load_config_default(
                    [(str(ROOT / TASK), ['session:=native_heading', 'yaw:=' + str(yaw)])],
                    11341, verbose=False)
                prefix = '/drone_0_traj_server/traj_server/'
                self.assertIs(config.params[prefix + 'mission_fixed_yaw'].value, False)
                self.assertAlmostEqual(config.params[prefix + 'initial_yaw'].value, yaw)
                self.assertIs(config.params[prefix + 'hold_at_trajectory_end'].value, True)

    def test_endpoint_hold_is_enabled_only_for_task_launches(self):
        for path, arguments, expected in (
                (TASK, ['session:=endpoint', 'yaw:=0.7'], True),
                (PLANNER / 'mission_worker.launch', [], True),
                (LEGACY, [], False)):
            with self.subTest(entry=str(path)):
                config = roslaunch.config.load_config_default(
                    [(str(ROOT / path), arguments)], 11341, verbose=False)
                key = '/drone_0_traj_server/traj_server/hold_at_trajectory_end'
                self.assertIs(config.params[key].value, expected)

    def test_task_session_topics_reach_manager(self):
        # Resolve the actual outer task launch. ROS does not recursively apply
        # /position_cmd -> intermediate -> session mappings.
        for session in ('offline_first', 'offline_second'):
            with self.subTest(session=session):
                config = roslaunch.config.load_config_default(
                    [(str(ROOT / TASK), ['session:=' + session, 'yaw:=0.0'])],
                    11341, verbose=False)
                prefix = '/airseafly/mission/session_' + session
                traj = next(node for node in config.nodes if node.type == 'traj_server')
                planner = next(node for node in config.nodes if node.type == 'ego_planner_node')

                def resolve(node, topic):
                    caller = rosgraph.names.ns_join(node.namespace, node.name)
                    return rosgraph.names.resolve_name(topic, caller, dict(node.remap_args))

                self.assertEqual(resolve(traj, '/position_cmd'), prefix + '/position_cmd')
                self.assertEqual(resolve(planner, '/airseafly/front/goal'), prefix + '/goal')
                self.assertEqual(resolve(planner, '/airseafly/front/goal_received'), prefix + '/goal_received')
                self.assertEqual(resolve(planner, '/airseafly/mission/ego_ready'), prefix + '/ready')

    def check_entries(self, root, expected):
        entries = ((TASK, ['session:=offline', 'yaw:=0.0', 'height_origin_z:=0.37']),
                   (PLANNER / 'mission_worker.launch', []), (LEGACY, []))
        for path, arguments in entries:
            with self.subTest(entry=str(path), inflation=expected):
                config = roslaunch.config.load_config_default(
                    [(str(root / path), arguments)], 11341, verbose=False)
                self.assertAlmostEqual(config.params[KEY].value, expected)

    def test_real_entries_use_xml_value(self):
        xml = ET.parse(ROOT / PLANNER / 'advanced_param_exp.xml')
        value = next(node.get('default') for node in xml.iter('arg')
                     if node.get('name') == 'map_inflation')
        self.check_entries(ROOT, float(value))

    def test_editing_only_xml_changes_all_entries(self):
        # Use a temporary copy: never edit the user's live planning parameters.
        paths = [TASK, LEGACY] + [PLANNER / name for name in (
            'mission_worker.launch', 'single_run_in_exp.launch', 'advanced_param_exp.xml')]
        with tempfile.TemporaryDirectory(prefix='real400-inflation-launch-') as directory:
            copied = Path(directory)
            for path in paths:
                source = (ROOT / path).read_text()
                source = source.replace('$(find ego_planner)/launch/', str(copied / PLANNER) + '/')
                target = copied / path
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(source)
            file = copied / PLANNER / 'advanced_param_exp.xml'
            tree = ET.parse(file)
            for node in tree.iter('arg'):
                if node.get('name') == 'map_inflation':
                    node.set('default', '0.23')
            tree.write(file, encoding='utf-8', xml_declaration=True)
            self.check_entries(copied, .23)


if __name__ == '__main__':
    unittest.main(verbosity=2)
