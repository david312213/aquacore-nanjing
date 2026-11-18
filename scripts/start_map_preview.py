#!/usr/bin/env python3
import os, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/airseafly_real400_mission/src'))
from airseafly_real400_mission.flight_configuration import load_observer_configuration
from airseafly_real400_mission.map_tuning import ego_launch_arguments
args = list(sys.argv[1:])
task = 'delivery-route' if '--delivery' in args else None
args = [x for x in args if x != '--delivery']
from airseafly_real400_mission.flight_configuration import base_configuration, apply_flight_configuration
config = apply_flight_configuration(base_configuration(project=ROOT), task=task, project=ROOT)
command = ['roslaunch', 'ego_planner', 'single_run_in_exp.launch',
 'map_preview:=true', 'odometry_topic:=/airseafly/native/planning_odometry',
 'cloud_topic:=/airseafly/native/planning_cloud', 'world_frame:=fast_lio_local_enu'
] + ego_launch_arguments(config) + args
if '--dry-run' in args:
 print(' '.join(x for x in command if x != '--dry-run'))
 sys.exit(0)
os.execvp(command[0], command)
