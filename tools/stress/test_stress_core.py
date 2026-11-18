import struct
import unittest

import stress_core as core


def header(secs, nsecs, frame=b'world'):
    return struct.pack('<III', 7, secs, nsecs) + struct.pack('<I', len(frame)) + frame


def ros_string(text):
    data = text.encode()
    return struct.pack('<I', len(data)) + data


def sample(t, phase, **values):
    row = {key: None for key in core.SAMPLE_FIELDS}
    row.update(wall_time=t, phase=phase, mode='', pred_last_withheld='', **values)
    return row


class StressCoreTest(unittest.TestCase):
    def test_classify_orders_specific_before_generic(self):
        self.assertEqual(core.classify('/usr/bin/python3 /h/src/airseafly_real400_mission/scripts/'
                                       'red_point_worker.py --config x'), 'red_point')
        self.assertEqual(core.classify('/opt/ros/noetic/bin/roslaunch airseafly_real400_mission '
                                       'ego_worker.launch'), 'ros_core')
        self.assertEqual(core.classify('/h/devel/lib/fast_lio/fastlio_mapping __name:=laserMapping'),
                         'fast_lio')
        self.assertEqual(core.classify('/h/devel/lib/ego_planner/ego_planner_node __name:=x'), 'ego')
        self.assertEqual(core.classify('/usr/NX/bin/nxnode.bin'), 'remote_desktop')
        self.assertEqual(core.classify('/usr/bin/bash'), 'other')

    def test_cpu_percent_from_proc_stat(self):
        before = core.parse_cpu_line('cpu  100 0 100 800 0 0 0 0 0 0\ncpu0 1 2 3 4\n')
        after = core.parse_cpu_line('cpu  200 0 200 900 0 0 0 0 0 0\n')
        self.assertEqual(before, (200, 1000))
        self.assertAlmostEqual(core.cpu_percent(before, after), 200 / 3)

    def test_pid_jiffies_tolerates_spaces_in_comm(self):
        stat = '42 (QGround Contr) S ' + ' '.join(['0'] * 10) + ' 150 50 0 0'
        self.assertEqual(core.parse_pid_jiffies(stat), 200)

    def test_group_percent_counts_new_and_existing_processes(self):
        groups = core.group_percent({1: 100}, {1: 150, 2: 25}, {1: 'fast_lio', 2: 'ego'}, 400)
        self.assertEqual(groups, {'fast_lio': 12.5, 'ego': 6.25})

    def test_header_string_and_state_bytes(self):
        self.assertAlmostEqual(core.header_stamp(header(1793112953, 500000000)), 1793112953.5)
        self.assertIsNone(core.header_stamp(header(0, 0)))
        self.assertEqual(core.string_payload(ros_string('{"a":1}')), '{"a":1}')
        state = header(1, 0) + bytes([1, 1, 0, 0]) + ros_string('ALTCTL') + bytes([3])
        self.assertEqual(core.mavros_state(state), (True, True, 'ALTCTL'))

    def test_prediction_status_lags(self):
        text = ('{"withheld":115,"last_withheld":"correction_expired",'
                '"last_correction_stamp_s":100.0,"last_output_stamp_s":null}')
        values = core.prediction_status(text, 101.5)
        self.assertEqual(values['pred_corr_lag_s'], 1.5)
        self.assertIsNone(values['pred_out_lag_s'])
        self.assertEqual(values['pred_withheld'], 115)

    def test_summary_flags_crash_like_phase_only(self):
        rows = [sample(t, 'A_baseline', cpu_pct=40, lio_odom_age_s=.05, lidar_hz=10,
                       pred_withheld=0, loc_healthy=1, armed=0) for t in range(4)]
        rows += [sample(4, 'D_red_ego', cpu_pct=95, lio_odom_age_s=.1, lidar_hz=10,
                        pred_withheld=0, loc_healthy=1, armed=0),
                 sample(5, 'D_red_ego', cpu_pct=99, lio_odom_age_s=2.0, pred_corr_lag_s=1.8,
                        lidar_hz=10, pred_withheld=40, loc_healthy=0, armed=0),
                 sample(6, 'D_red_ego', cpu_pct=99, lio_odom_age_s=3.0, lidar_hz=10,
                        pred_withheld=90, loc_healthy=0, armed=0)]
        report = core.summarize(rows, [('D_red_ego', 'ego', 150.0), ('D_red_ego', 'fast_lio', 90.0)])
        self.assertEqual(report[0]['flags'], [])
        self.assertEqual(report[1]['flags'], ['LIO_LAG', 'PRED_HORIZON', 'WITHHELD', 'LOC_UNHEALTHY'])
        self.assertEqual(report[1]['loc_unhealthy_s'], 1)
        self.assertEqual(report[1]['top_groups'][0], ('ego', 50.0))
        self.assertIn('D_red_ego', core.format_report(report))

    def test_parse_sample_keeps_text_fields(self):
        row = core.parse_sample({'phase': 'A', 'mode': 'POSCTL', 'cpu_pct': '12.5', 'armed': ''})
        self.assertEqual(row, {'phase': 'A', 'mode': 'POSCTL', 'cpu_pct': 12.5, 'armed': None})


if __name__ == '__main__':
    unittest.main()
