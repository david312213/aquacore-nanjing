"""Resolve the real ROS launch argument chain without requiring ROS."""
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[4]
PLANNER = ROOT/'src/planner/plan_manage'
ENTRY = ROOT/'src/airseafly_real400_mission/launch/ego_worker.launch'


def resolve_launch(path, overrides, namespace='', node_types=None):
    tree = ET.parse(path).getroot()
    declared = {node.get('name') for node in tree.findall('arg')}
    unknown = set(overrides)-declared
    if unknown:
        raise AssertionError('%s unknown arguments %s' % (path, sorted(unknown)))
    values = {}

    def sub(text):
        text = re.sub(r"\$\(eval arg\('([^']+)'\) != ''\)",
                      lambda match: str(values[match[1]] != '').lower(), text)
        text = re.sub(r'\$\(arg ([^)]+)\)', lambda match: values[match[1]], text)
        return text.replace('$(find ego_planner)', str(PLANNER)).replace('$(dirname)', str(path.parent)).replace('$(find fast_lio)', str(ROOT/'src/realflight_modules/mid360_fastlio/src/FAST_LIO'))

    def enabled(element):
        for attribute, expected in (('if', True), ('unless', False)):
            if attribute not in element.attrib:
                continue
            value = sub(element.get(attribute)).lower()
            if value not in ('true', 'false', '1', '0'):
                raise AssertionError('%s invalid %s condition %s' % (path, attribute, value))
            if (value in ('true', '1')) != expected:
                return False
        return True

    def name_in_scope(name, scope):
        if name.startswith('/'):
            return name
        return '/' + '/'.join(part for part in (scope.strip('/'), name.strip('/')) if part)

    for argument in tree.findall('arg'):
        if not enabled(argument):
            continue
        key = argument.get('name')
        raw = argument.get('value', overrides.get(key, argument.get('default')))
        if raw is None:
            raise AssertionError('%s missing argument %s' % (path, key))
        values[key] = sub(raw)
    nodes, params = {}, {}

    def walk(element, prefix):
        for child in element:
            if child.tag == 'arg' or not enabled(child):
                continue
            if child.tag == 'include':
                extra = {arg.get('name'):sub(arg.get('value')) for arg in child.findall('arg') if enabled(arg)}
                scope = name_in_scope(sub(child.get('ns')), prefix) if child.get('ns') else prefix
                nested_nodes, nested_params = resolve_launch(Path(sub(child.get('file'))), extra,
                                                            scope, node_types)
                nodes.update(nested_nodes);params.update(nested_params)
            elif child.tag == 'node':
                scope = name_in_scope(sub(child.get('ns')), prefix) if child.get('ns') else prefix
                name = name_in_scope(sub(child.get('name')), scope)
                nodes[name] = {sub(remap.get('from')):sub(remap.get('to'))
                               for remap in child.findall('remap') if enabled(remap)}
                if node_types is not None:
                    node_types[name] = sub(child.get('type'))
                walk(child, name)
            elif child.tag == 'param':
                params[name_in_scope(sub(child.get('name')), prefix)] = sub(child.get('value'))
            elif child.tag == 'group':
                scope = name_in_scope(sub(child.get('ns')), prefix) if child.get('ns') else prefix
                walk(child, scope)
    walk(tree, namespace)
    return nodes, params


class NativeFrameLaunchTests(unittest.TestCase):
    def check_profile(self, extra, frame, odometry, cloud):
        nodes, params=resolve_launch(ENTRY,dict(session='17',yaw='.73',max_vel='.4',max_acc='.2',**extra))
        planner='/drone_0_ego_planner_node';sender='/drone_0_traj_server'
        self.assertEqual(nodes[planner]['~odom_world'],odometry)
        self.assertEqual(nodes[planner]['~grid_map/odom'],odometry)
        self.assertEqual(nodes[planner]['~grid_map/cloud'],cloud)
        self.assertEqual(params[planner+'/world_frame'],frame)
        self.assertEqual(params[planner+'/grid_map/frame_id'],frame)
        self.assertEqual(params[sender+'/world_frame'],frame)
        self.assertEqual(params[sender+'/traj_server/initial_yaw'],'.73')
        self.assertEqual(params[sender+'/traj_server/mission_yaw'],'.73')
        self.assertEqual(params[sender+'/traj_server/mission_fixed_yaw'],'false')
        self.assertEqual(params[sender+'/traj_server/hold_at_trajectory_end'],'true')
        self.assertEqual(nodes[sender]['/position_cmd'],'/airseafly/mission/session_17/position_cmd')
        self.assertEqual(params[planner+'/manager/max_vel'],'.4')
        self.assertEqual(params[planner+'/manager/max_acc'],'.2')

    def test_legacy_topics_and_frame_remain_defaults(self):
        self.check_profile({},'world','/Odom_high_freq','/cloud_registered')

    def test_native_topics_and_frame_reach_every_consumer(self):
        self.check_profile(dict(odometry_topic='/airseafly/native/planning_odometry',
                                cloud_topic='/airseafly/native/planning_cloud',
                                world_frame='fast_lio_local_enu'),
                           'fast_lio_local_enu','/airseafly/native/planning_odometry',
                           '/airseafly/native/planning_cloud')



    def test_floor_and_ceiling_share_the_actual_takeoff_reference(self):
        _, params = resolve_launch(ENTRY, dict(session='offset', yaw='0', height_origin_z='.37', map_min_height='.3'))
        for boundary in ('virtual_floor_origin_z', 'virtual_ceil_origin_z'):
            self.assertEqual(params['/drone_0_ego_planner_node/grid_map/'+boundary], '.37')

    def test_normal_launch_still_starts_original_planner_and_trajectory_server(self):
        types = {}
        resolve_launch(PLANNER/'launch/single_run_in_exp.launch', {}, node_types=types)
        self.assertEqual(types, {'/drone_0_ego_planner_node':'ego_planner_node',
                                '/drone_0_traj_server':'traj_server'})


if __name__=='__main__': unittest.main(verbosity=2)
