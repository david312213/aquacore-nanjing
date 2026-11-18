#!/usr/bin/env python3
"""Run the current task's offline checks; all ROS and actuator calls are mocked."""
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SUITES = ('test_basic_flight_entry.py', 'test_delivery_route_entry.py', 'test_delivery_alignment.py', 'test_single_drop.py', 'test_single_drop_entry.py', 'test_drop_points.py', 'test_drop_points_entry.py', 'test_native_drop_points_manager.py', 'test_hover_drop_entry.py', 'test_ego_goal_identity.py', 'test_four_magnet_payload.py', 'test_guided_route.py', 'test_landing_alignment.py', 'test_landing_pad.py', 'test_main1_pwm.py', 'test_main67_pwm.py', 'test_main_servo_test.py', 'test_nanjing_configuration.py', 'test_native_covariance.py', 'test_log_durability.py', 'test_native_delivery_handoff.py', 'test_native_delivery_manager.py', 'test_native_single_drop_manager.py', 'test_native_delivery_route.py', 'test_native_flight_report.py', 'test_native_freshness.py', 'test_native_full_runtime.py', 'test_native_landing_confirmation.py', 'test_native_localization_health.py', 'test_native_payload_transport.py', 'test_native_planning_geometry.py', 'test_native_reference.py', 'test_native_waypoint_manager.py', 'test_native_waypoint_route.py', 'test_payload.py', 'test_payload_slot_pwm.py', 'test_planning_launch.py', 'test_red_point.py', 'test_red_point_entry.py', 'test_red_point_worker.py', 'test_resource_snapshot.py', 'test_rc_payload.py', 'test_localization_gate.py', 'test_safe_land.py', 'test_servo_reset.py', 'test_two_stage_payload.py', 'test_waypoint_route.py', 'test_ros_network.py')
TOOL_SUITES = (
    ('src/planner/plan_manage/test', 'test_goal_completion.py'),
    ('src/planner/plan_manage/test', 'test_short_trajectory.py'),
    ('src/planner/plan_manage/test', 'test_short_waypoint_math.py'),
    ('src/planner/plan_manage/test', 'test_trajectory_time.py'),
    ('src/planner/plan_manage/test', 'test_traj_server_end.py'),
    ('src/planner/plan_manage/test', 'test_mission_interfaces.py'),
    ('src/planner/plan_manage/test', 'test_native_frame_launch.py'),
    ('tools/px4', 'test_fusion_checks.py'),
    ('tools/calibration/down_camera', 'test_calibrate.py'))

FIXED_SUITES = ('test_local_retiming.py', 'test_split_delivery_route.py', 'test_fixed_trajectory.py', 'test_native_fixed_waypoint_route.py',
                'test_fixed_trajectory_runtime.py', 'test_fixed_trajectory_manager.py',
                'test_fixed_trajectory_preview.py', 'test_waypoint_profiles.py',
                'test_virtual_boundary.py', 'test_native_fixed_delivery_route.py',
                'test_loop_sections.py', 'test_retiming_linesearch.py',
                'test_single_drop_hover.py', 'test_recover_acceleration.py')


def main():
    out = ROOT/'data/checks'
    out.mkdir(parents=True, exist_ok=True)
    results = {}
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', AIRSEAFLY_PYTHON=sys.executable,
        PYTHONPATH=os.pathsep.join([str(ROOT/'src/airseafly_real400_mission/src'),
                                   os.environ.get('PYTHONPATH', '')]))
    build = subprocess.run(['sh', str(ROOT/'scripts/build_fixed_trajectory.sh')],
                           cwd=str(ROOT), env=env, capture_output=True, text=True)
    (out/'fixed_trajectory_build.log').write_text(build.stdout+build.stderr)
    results['fixed_trajectory_build'] = dict(exit_code=build.returncode)
    print('fixed_trajectory_build', 'PASS' if build.returncode == 0 else 'FAIL', flush=True)
    if build.returncode:
        print(build.stdout+build.stderr, flush=True)
    else:
        env['AIRSEAFLY_MIN_SNAP_LIBRARY'] = build.stdout.strip().splitlines()[-1]
    suites = [('src/airseafly_real400_mission/test', suite) for suite in SUITES+FIXED_SUITES] + list(TOOL_SUITES)
    for directory, suite in suites:
        command = [sys.executable, '-c', 'import numpy; import unittest; unittest.main(module=None)',
                   'discover', '-s', str(ROOT/directory), '-p', suite]
        run = subprocess.run(command, cwd=str(ROOT), env=env, capture_output=True, text=True, timeout=90)
        output = run.stdout+run.stderr
        (out/(suite+'.log')).write_text(output)
        count = re.search(r'Ran (\d+) tests?', output)
        skipped = re.search(r'skipped=(\d+)', output)
        count = int(count.group(1)) if count else 0
        results[suite] = dict(exit_code=run.returncode, tests=count,
                             skipped=int(skipped.group(1)) if skipped else 0)
        label = 'PASS' if run.returncode == 0 and count else 'FAIL'
        if count and results[suite]['skipped'] == count and run.returncode == 0:
            label = 'SKIP (requires ROS)'
        print(suite, label, count, flush=True)
        if run.returncode or not count: print(output[-4500:], flush=True)
    for test in ('native_owner_test', 'native_watchdog_test', 'native_setpoint_test', 'landing_request_test', 'payload_pd_test'):
        compiler = shutil.which('c++')
        if not compiler:
            results[test] = dict(skipped='C++ compiler unavailable')
            continue
        src = ROOT/'src/realflight_modules/px4ctrl/test'/(test+'.cpp')
        run = subprocess.run([compiler, '-std=c++14', '-O2', str(src), '-o', str(out/test)],
                             capture_output=True, text=True)
        if run.returncode == 0: run = subprocess.run([str(out/test)], capture_output=True, text=True)
        (out/(test+'.log')).write_text(run.stdout+run.stderr)
        results[test] = dict(exit_code=run.returncode, programs=1)
        print(test, 'PASS' if run.returncode == 0 else 'FAIL', flush=True)
    fastlio = ROOT/'src/realflight_modules/mid360_fastlio/src/FAST_LIO'
    eigen_candidates = [os.environ.get('FASTLIO_EIGEN_INCLUDE', ''),
                        '/usr/include/eigen3', '/usr/local/include/eigen3', '/opt/homebrew/include/eigen3']
    eigen = next((path for path in eigen_candidates if path and (Path(path)/'Eigen/Core').is_file()), None)
    for test in ('test_predictor', 'test_prediction_covariance', 'test_prediction_bridge'):
        compiler = shutil.which('c++')
        if not compiler or (test == 'test_prediction_bridge' and not eigen):
            reason = 'C++ compiler unavailable' if not compiler else 'set FASTLIO_EIGEN_INCLUDE to the Eigen include directory'
            results[test] = dict(skipped=reason)
            print(test, 'SKIP ('+reason+')', flush=True)
            continue
        command = [compiler, '-std=c++14', '-Wall', '-Wextra', '-Werror', '-O2', '-I'+str(fastlio/'src')]
        if test == 'test_prediction_bridge':
            command += ['-I'+str(fastlio/'test/stub_include'), '-isystem', eigen]
        command += [str(fastlio/'test'/(test+'.cpp')), '-o', str(out/test)]
        run = subprocess.run(command, capture_output=True, text=True, timeout=90)
        if run.returncode == 0:
            run = subprocess.run([str(out/test)], capture_output=True, text=True, timeout=90)
        (out/(test+'.log')).write_text(run.stdout+run.stderr)
        results[test] = dict(exit_code=run.returncode, programs=1)
        print(test, 'PASS' if run.returncode == 0 else 'FAIL', flush=True)
        if run.returncode:
            print(run.stdout+run.stderr, flush=True)
    core = ROOT/'src/airseafly_real400_core'
    for test in ('core_test', 'control_law_test', 'bspline_test'):
        if not shutil.which('c++') or not eigen:
            results['core_'+test] = dict(skipped='C++ compiler/Eigen unavailable')
            continue
        includes = [core/'include', core/'test/stubs',
                    ROOT/'src/realflight_modules/px4ctrl/src',
                    ROOT/'src/planner/bspline_opt/include', ROOT/'src/planner/bspline_opt/src']
        binary = out/('core_'+test)
        command = [shutil.which('c++'), '-std=c++14', '-O2', '-isystem', eigen]
        command += ['-I'+str(p) for p in includes]
        command += [str(core/'test'/(test+'.cpp')), '-o', str(binary)]
        run = subprocess.run(command, capture_output=True, text=True, timeout=90)
        if run.returncode == 0:
            run = subprocess.run([str(binary)], capture_output=True, text=True, timeout=90)
        (out/('core_'+test+'.log')).write_text(run.stdout+run.stderr)
        results['core_'+test] = dict(exit_code=run.returncode, programs=1)
        print('core_'+test, 'PASS' if run.returncode == 0 else 'FAIL', flush=True)
        if run.returncode: print(run.stdout+run.stderr, flush=True)
    run = subprocess.run([sys.executable, str(ROOT/'scripts/check_project.py'), '--relocate'],
                         capture_output=True, text=True, timeout=180)
    (out/'project_check.log').write_text(run.stdout+run.stderr)
    results['project_check'] = dict(exit_code=run.returncode)
    print('project_check', 'PASS' if run.returncode == 0 else 'FAIL', flush=True)
    if run.returncode: print(run.stdout+run.stderr)
    (out/'results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2)+'\n')
    return int(any(value.get('exit_code', 0) != 0 or value.get('tests') == 0 for value in results.values()))


if __name__ == '__main__':
    sys.exit(main())
