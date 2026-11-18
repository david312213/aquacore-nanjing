#!/usr/bin/env python3
"""Structural/import/launch/relocation checks only; no ROS connections."""
import argparse
import ast
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]
EXTERNAL_ROS = {'mavros','rosbag','rviz','usb_cam','image_view','nodelet','tf','tf2_ros'}
EXCLUDED = {'build_ws','runtime','data','.git','__pycache__','.venv','.pytest_cache'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--environment', action='store_true')
    parser.add_argument('--relocate', action='store_true')
    args = parser.parse_args()
    if args.environment:
        print(json.dumps(dict(python=sys.executable, ros_noetic=Path('/opt/ros/noetic/setup.sh').is_file(),
                              cmake=shutil.which('cmake'), catkin_make=shutil.which('catkin_make'))))
        return
    import yaml
    packages = {}
    for p in (ROOT/'src').rglob('package.xml'):
        name = ET.parse(p).getroot().findtext('name')
        assert name not in packages, 'Duplicate package '+name
        packages[name] = p.parent
    links = 0
    for p in ROOT.rglob('*'):
        if EXCLUDED.intersection(p.relative_to(ROOT).parts): continue
        if p.is_symlink():
            assert p.exists(), 'Broken link '+str(p)
            assert ROOT in p.resolve().parents, 'External source link '+str(p)
            links += 1
    document_links = 0
    for p in [ROOT/'README.md',ROOT/'config/README.md',ROOT/'tools/README.md',
              *sorted((ROOT/'flight').glob('*.md')),*sorted((ROOT/'docs').glob('*.md'))]:
        for target in re.findall(r'\[[^\]]*\]\(([^)]+)\)',p.read_text()):
            if re.match(r'[a-zA-Z]+:',target) or target.startswith('#'): continue
            dest=(p.parent/unquote(target.split('#')[0].strip('<>'))).resolve()
            assert dest.exists(), 'Broken document link '+str(p)+' -> '+target
            document_links += 1
    seen=set()
    def launch(path):
        path=path.resolve()
        if path in seen:return
        assert path.is_file(), 'Missing launch '+str(path)
        seen.add(path)
        for element in ET.parse(path).iter():
            if element.tag=='node':
                pkg=element.get('pkg','')
                assert pkg in packages or pkg in EXTERNAL_ROS, 'Missing node package '+pkg
            if element.tag=='include':
                match=re.match(r'\$\(find ([^)]+)\)/(.*)',element.get('file',''))
                if match:
                    pkg,suffix=match.groups()
                    if pkg in packages:launch(packages[pkg]/suffix)
                    else:assert pkg in EXTERNAL_ROS, 'Missing include package '+pkg
    for pkg,rel in [('airseafly_real400','launch/base_sensors.launch'),
                    ('airseafly_real400_mission','launch/red_point_camera.launch'),
                    ('airseafly_real400_mission','launch/native_localization.launch'),
                    ('airseafly_real400_mission','launch/ego_worker.launch'),
                    ('airseafly_real400_mission','launch/safe_land.launch'),
                    ('px4ctrl','launch/run_ctrl.launch')]:launch(packages[pkg]/rel)
    core=ROOT/'src/airseafly_real400_mission/src/airseafly_real400_mission'
    python_files=list(core.glob('*.py'))+list((packages['airseafly_real400_mission']/'scripts').glob('*.py'))+list((ROOT/'scripts').glob('*.py'))
    for p in python_files:
        tree=ast.parse(p.read_text(),filename=str(p))
        for n in ast.walk(tree):
            names=[]
            if isinstance(n,ast.ImportFrom):
                if n.level and p.parent==core:names=[n.module.split('.')[0]] if n.module else [x.name for x in n.names]
                elif n.module and n.module.startswith('airseafly_real400_mission.'):names=[n.module.split('.')[1]]
                elif n.module=='airseafly_real400_mission':names=[x.name for x in n.names]
                assert not (n.module or '').startswith(('airseafly_front_goal','airseafly_obstacle_mission')),str(p)
            for name in names:assert (core/(name+'.py')).is_file(), str(p)+' missing '+name
    for p in [*(ROOT/'config').rglob('*.yaml'),*(ROOT/'flight').glob('*.yaml')]:yaml.safe_load(p.read_text())
    spec=importlib.util.spec_from_file_location('df200_constants',core/'df200_geometry.py')
    geometry=importlib.util.module_from_spec(spec);spec.loader.exec_module(geometry)
    calibration=yaml.safe_load((ROOT/'config/aircraft/df200_calib.yaml').read_text())
    assert (calibration['image_width'],calibration['image_height'])==geometry.CALIBRATION_SIZE
    assert calibration['camera_matrix']['data']==[v for row in geometry.CAMERA_MATRIX for v in row]
    assert calibration['distortion_coefficients']['data']==list(geometry.DISTORTION)
    def previews(root):
        env=dict(os.environ,AIRSEAFLY_PYTHON=sys.executable,PYTHONDONTWRITEBYTECODE='1')
        # The relocation check builds the native dependency in the relocated
        # tree too; an override pointing back at the original would hide errors.
        if root != ROOT:
            env.pop('AIRSEAFLY_MIN_SNAP_LIBRARY', None)
            build=subprocess.run(['sh',str(root/'scripts/build_fixed_trajectory.sh')],
                                 cwd=str(root),env=env,capture_output=True,text=True)
            assert build.returncode==0, 'Relocated MINCO build failed: '+build.stderr
            env['AIRSEAFLY_MIN_SNAP_LIBRARY']=build.stdout.strip().splitlines()[-1]
        routes=root/'data/checks/relocation-routes';routes.mkdir(parents=True,exist_ok=True)
        ordinary=routes/'ordinary.yaml';contest=routes/'contest.yaml';drop_points=routes/'drop_points.yaml'
        ordinary.write_text(yaml.safe_dump(dict(format='aquacore_waypoints',version=1,frame='fast_lio_local_enu',points=[[1.,2.],[3.,4.]])))
        drop_points.write_text(yaml.safe_dump(dict(format='aquacore_drop_points',version=1,frame='fast_lio_local_enu',points=[[1.,2.],[3.,4.]])))
        data=dict(version=1,frame='fast_lio_local_enu',home=[0.,0.],wizard=dict(stage_index=9),route=[])
        for slot,stage in zip(range(1,5),('obstacle2','obstacle3','gate','return_drop1')):
            data['route'].append(dict(stage='drop%d'%slot,kind='drop',xy=[float(slot),0.],drop_slot=slot))
            data['route'].append(dict(stage=stage,kind='gate' if stage=='gate' else 'transit',xy=[float(slot),1.]))
        contest.write_text(yaml.safe_dump(data))
        commands=[('1_sensors.sh',[]),('2_localization.sh',[]),('3_controller.sh',[]),
                  ('takeoff.sh',['--json']),('hold.sh',[]),('safe_land.sh',[]),
                  ('fly_points.sh',['--route',str(ordinary),'--json']),
                  ('4_delivery_route.sh',['--route',str(contest),'--json']),('red_point.sh',[]),
                  ('drop_test.sh',['--x','1.25','--y','-.5','--slot','4','--json']),
                  ('drop_points.sh',['--route',str(drop_points),'--count','2','--json']),
                  ('hover_drop.sh',['--height','1.0','--json']),
                  ('magnet.sh',['release','4'])]
        outputs={}
        for script,extra in commands:
            run=subprocess.run(['sh',str(root/'flight'/script),*extra,'--dry-run'],cwd=str(root.parent),env=env,capture_output=True,text=True,timeout=20)
            assert run.returncode==0,script+' '+run.stderr
            value=json.loads(run.stdout)
            # Provenance/paths are allowed to relocate; numeric command values must match.
            if script in ('takeoff.sh','fly_points.sh','hover_drop.sh'):
                value.get('parameters',{}).pop('effective_motion_sources',None)
            outputs[script]=value
        return outputs
    original=previews(ROOT)
    if args.relocate:
        with tempfile.TemporaryDirectory(prefix='aquacore-relocation-') as tmp:
            moved=(Path(tmp)/'南京 project').resolve()
            shutil.copytree(ROOT,moved,symlinks=True,ignore=shutil.ignore_patterns(*EXCLUDED,'._*','.DS_Store'))
            second=previews(moved)
            assert json.dumps(original,sort_keys=True,ensure_ascii=False).replace(str(ROOT),'${PROJECT}')==json.dumps(second,sort_keys=True,ensure_ascii=False).replace(str(moved),'${PROJECT}'), 'Relocation changed previews'
    print(json.dumps(dict(packages=len(packages),internal_links=links,launch_files=len(seen),
        python_files=len(python_files),document_links=document_links,dry_runs=len(original)*(2 if args.relocate else 1),
        df200_intrinsics_match=True,hardware_started=False),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
