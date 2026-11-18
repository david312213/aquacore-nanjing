"""Offline diagnostics distinguish task/phase boundaries and missing observations."""
from contextlib import redirect_stdout
from io import StringIO
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from airseafly_real400_mission.native_flight_report import assess, analyze, main, report_text


def row(stamp, source='TAKEOFF', position=(0., 0., 3.), reference=(0., 0., 3.),
        velocity=(0., 0., 0.), desired_velocity=(0., 0., 0.), task='flight-a'):
    return dict(stamp=stamp, task_id=task, mode='takeoff' if source != 'EGO' else 'points',
        state='TAKEOFF' if source == 'TAKEOFF' else 'COMPLETE', phase='ASCEND',
        reference_source=source, ground_origin=[0., 0., 2.], takeoff_target_xyz=[0., 0., 3.],
        target_xyz=[0., 0., 3.], waypoint_index=0,
        control_status=dict(owner='PX4_NATIVE', armed=True, offboard=True, manual_paused=False),
        command=dict(frame='fast_lio_local_enu', position=list(reference), velocity=list(desired_velocity)),
        task_feedback=dict(frame='fast_lio_local_enu', position=list(position), velocity=list(velocity),
                           stamp=stamp, position_source='FAST_LIO'))


class NativeFlightReportTests(unittest.TestCase):
    def test_fixed_tracking_is_reported_separately_from_ego(self):
        rows = [row(1., 'FIXED_TRAJECTORY', position=(.3, .4, 3.)), row(2., 'EGO')]
        result = assess(rows)
        fixed = result['tasks'][0]['fixed_trajectory']['position_error']
        self.assertEqual(fixed['samples'], 1)
        self.assertAlmostEqual(fixed['rmse_3d'], .5)
        self.assertEqual(result['tasks'][0]['ego']['position_error']['rmse_3d'], 0.)
        self.assertIn('固定轨迹 位置跟踪', report_text(result))

    def test_takeoff_uses_final_endpoint_not_moving_reference(self):
        rows = [row(1., position=(0., 0., 2.7), reference=(0., 0., 2.4)),
                row(2., position=(0., 0., 3.1)),
                row(3., source='HOLD', position=(0., 0., 3.2)),
                row(4., source='EGO', position=(0., 0., 8.), reference=(1., 0., 8.))]
        result = assess(rows)['tasks'][0]['takeoff']
        self.assertAlmostEqual(result['observed_overshoot_m'], .2)
        self.assertAlmostEqual(result['target_height_m'], 1.)
        self.assertAlmostEqual(result['peak_height_m'], 1.2)
        self.assertTrue(result['reference_finished_observed'])
        self.assertEqual(result['samples'], 3)

    def test_moving_reference_error_is_not_takeoff_overshoot(self):
        result = assess([row(1., position=(0., 0., 2.7), reference=(0., 0., 2.4))])
        self.assertEqual(result['tasks'][0]['takeoff']['observed_overshoot_m'], 0.)
        self.assertFalse(result['tasks'][0]['takeoff']['reference_finished_observed'])
        self.assertIn('不覆盖完整响应', report_text(result))

    def test_no_final_target_cannot_infer_overshoot(self):
        record = row(1., position=(0., 0., 3.5))
        record.pop('takeoff_target_xyz')
        record.pop('target_xyz')
        result = assess([record])['tasks'][0]['takeoff']
        self.assertFalse(result['available'])
        self.assertNotIn('observed_overshoot_m', result)

    def test_no_ground_origin_cannot_infer_relative_height(self):
        record = row(1.)
        record.pop('ground_origin')
        result = assess([record])['tasks'][0]['takeoff']
        self.assertTrue(result['available'])
        self.assertIsNone(result['peak_height_m'])
        self.assertIsNone(result['target_height_m'])

    def test_changed_takeoff_target_does_not_mix_results(self):
        changed = row(2., position=(0., 0., 4.))
        changed['takeoff_target_xyz'][2] = 4.
        result = assess([row(1.), changed])['tasks'][0]['takeoff']
        self.assertFalse(result['available'])

    def test_hold_segments_separate_fixed_targets_and_no_settling_gate(self):
        rows = [row(1., 'HOLD', position=(.3, .4, 3.)),
                row(1.1, 'HOLD', position=(0., 0., 3.)),
                row(1.2, 'EGO'), row(1.3, 'HOLD', reference=(2., 0., 3.), position=(2.1, 0., 3.)),
                row(1.4, 'HOLD', reference=(4., 0., 3.), position=(4., 0., 3.))]
        report = assess(rows)['tasks'][0]['hover']
        self.assertEqual(len(report['segments']), 3)
        metric = report['segments'][0]['position_error']
        self.assertAlmostEqual(metric['rmse_3d'], (.25/2)**.5)
        self.assertEqual(metric['max_3d'], .5)
        self.assertEqual(metric['mean_error_xyz'], [-.15, -.2, 0.])
        self.assertEqual(report['segments'][2]['duration_s'], 0.)

    def test_ego_tracks_position_and_velocity_per_waypoint(self):
        first = row(1., 'EGO', position=(.3, .4, 3.), velocity=(.1, .2, .2), desired_velocity=(0., .2, 0.))
        second = row(2., 'EGO')
        second['waypoint_index'] = 1
        metric = assess([first, second])['tasks'][0]['ego']
        self.assertAlmostEqual(metric['position_error']['rmse_3d'], (.25/2)**.5)
        self.assertAlmostEqual(metric['velocity_error']['rmse_3d'], (.05/2)**.5)
        self.assertEqual(metric['waypoints']['0']['position_error']['samples'], 1)
        self.assertEqual(metric['waypoints']['1']['position_error']['samples'], 1)

    def test_tasks_separate_even_with_reset_clock_or_reappearing_id(self):
        rows = [row(10., 'EGO'), row(1., 'EGO', task='flight-b'), row(11., 'EGO')]
        result = assess(rows)
        self.assertEqual([t['task_id'] for t in result['tasks']], ['flight-a', 'flight-b', 'flight-a'])
        self.assertEqual([t['samples'] for t in result['tasks']], [1, 1, 1])
        self.assertEqual(len(assess(rows, task_id='flight-b')['tasks']), 1)

    def test_duplicate_and_backward_record_times_are_excluded(self):
        rows = [row(stamp, 'EGO') for stamp in (10., 10., 9., 9.5, 11.)]
        result = assess(rows)
        self.assertEqual(result['rejected']['duplicate_or_backwards_stamp'], 3)
        self.assertEqual(result['tasks'][0]['ego']['position_error']['samples'], 2)

    def test_duplicate_feedback_does_not_fake_more_position_samples(self):
        first, duplicate, third = row(1., 'EGO'), row(2., 'EGO'), row(3., 'EGO')
        duplicate['task_feedback']['stamp'] = 1.
        result = assess([first, duplicate, third])['tasks'][0]
        self.assertEqual(result['ego']['position_error']['samples'], 2)
        self.assertEqual(result['rejected']['duplicate_or_backwards_position_stamp'], 1)

    def test_repeated_feedback_does_not_fragment_fixed_hold(self):
        rows = [row(1., 'HOLD'), row(2., 'HOLD'), row(3., 'HOLD')]
        rows[1]['task_feedback']['stamp'] = 1.
        result = assess(rows)['tasks'][0]['hover']
        self.assertEqual(len(result['segments']), 1)
        self.assertEqual(result['segments'][0]['position_error']['samples'], 2)
        self.assertEqual(result['segments'][0]['duration_s'], 2.)

    def test_manual_handoff_splits_hold_and_ends_takeoff_observation(self):
        rows = [row(1.), row(2., 'HOLD'), row(3., 'HOLD'),
                row(4., 'HOLD', position=(0., 0., 6.))]
        rows[2]['control_status']['manual_paused'] = True
        result = assess(rows)['tasks'][0]
        self.assertEqual(result['takeoff']['samples'], 2)
        self.assertEqual(result['takeoff']['observed_overshoot_m'], 0.)
        self.assertEqual(len(result['hover']['segments']), 2)

    def test_mismatched_frames_missing_velocity_and_nan_are_not_zero(self):
        bad_frame = row(1., 'EGO')
        bad_frame['task_feedback']['frame'] = 'px4_local_enu'
        nan = row(2., 'EGO')
        nan['task_feedback']['position'][0] = float('nan')
        missing_velocity = row(3., 'EGO')
        missing_velocity['task_feedback'].pop('velocity')
        result = assess([bad_frame, nan, missing_velocity])['tasks'][0]
        self.assertEqual(result['ego']['position_error']['samples'], 1)
        self.assertEqual(result['ego']['velocity_error']['samples'], 1)
        self.assertEqual(result['rejected']['missing_or_mismatched_frame'], 1)
        self.assertEqual(result['rejected']['invalid_position_feedback'], 1)
        self.assertEqual(result['rejected']['invalid_velocity_feedback'], 1)

    def test_control_handoff_excludes_non_native_data(self):
        rows = [row(1., 'EGO')]
        for index, change in enumerate((dict(armed=False), dict(offboard=False), dict(manual_paused=True),
                                        dict(owner='PX4_AUTO_LAND')), 2):
            record = row(float(index), 'EGO', position=(10., 10., 10.))
            record['control_status'].update(change)
            rows.append(record)
        result = assess(rows)['tasks'][0]
        self.assertEqual(result['ego']['position_error']['samples'], 1)
        self.assertEqual(result['rejected']['not_native_armed_offboard'], 4)

    def test_legacy_unattributed_data_not_merged_as_one_task(self):
        record = row(1.)
        record.pop('task_id')
        result = assess([record])
        self.assertEqual(result['tasks'], [])
        self.assertEqual(result['rejected']['missing_task_id'], 1)
        self.assertIn('不可计算', report_text(result))
        self.assertTrue(result['diagnostic_only'])
        self.assertFalse(result['automatically_applied'])

    def test_log_directory_and_json_cli_work_without_ros(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory)/'native_trajectory.jsonl'
            log.write_text(json.dumps(row(1., 'EGO'))+'\nnot json\n[]\n'+json.dumps(dict(stamp='bad'))+'\n')
            result = analyze(directory)
            self.assertEqual(result['rejected']['malformed_json'], 2)
            self.assertEqual(result['rejected']['invalid_stamp'], 1)
            output = Path(directory)/'report.json'
            text = StringIO()
            with redirect_stdout(text):
                self.assertEqual(main([directory, '--json', str(output)]), 0)
            self.assertIn('EGO 位置跟踪', text.getvalue())
            self.assertTrue(json.loads(output.read_text())['diagnostic_only'])
            executable = Path(__file__).resolve().parents[1]/'scripts/native_flight_report.py'
            completed = subprocess.run([sys.executable, str(executable), directory, '--json', '-'],
                                       text=True, capture_output=True, check=True)
            self.assertEqual(json.loads(completed.stdout)['tasks'][0]['task_id'], 'flight-a')

    def test_cli_cannot_overwrite_source_log(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory)/'native_trajectory.jsonl'
            original = json.dumps(row(1.))+'\n'
            log.write_text(original)
            with self.assertRaises(SystemExit) as error:
                main([str(log), '--json', str(log)])
            self.assertEqual(error.exception.code, 2)
            self.assertEqual(log.read_text(), original)


if __name__ == '__main__':
    unittest.main()
