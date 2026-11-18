"""Recorded-point visual delivery replay; no ROS, camera or actuator access."""
import math
import unittest
from unittest import mock

from test_native_delivery_route import Replay, configuration
from airseafly_real400_mission.telemetry import TerminalReporter, numbers


def candidate(xy, confirmed=True, projection='ok_near_level'):
    return dict(class_name='red_point', target_confirmed=confirmed,
                projection_status=projection, xy_m=list(xy))


def descending(config=None):
    replay = Replay(config)
    replay.until(lambda e: e.phase == 'NATIVE_INTERMEDIATE_DESCEND')
    return replay


def waiting(config=None):
    replay = descending(config)
    replay.until(lambda e: e.phase == 'NATIVE_SECOND_FRAME')
    return replay


def one_fix(h, xy=None):
    h.frame(xy)
    h.until(lambda e: e.phase == 'NATIVE_DESCEND')


def deadline(h, before=0., follow=False):
    end = h.engine._alignment_deadline()
    return h.step(max(0., end-h.now-before), follow=follow, acknowledge=False)


def releases(h):
    return [effect for effect in h.effects if effect['type'] == 'release']


class DeliveryAlignmentTests(unittest.TestCase):
    def test_descends_over_recorded_point_then_one_red_fix_nearest_recorded(self):
        h = descending(); e = h.engine
        self.assertEqual(e.native_goal, (3., 2., .5))
        self.assertFalse(h.frame((3.1, 2.1)))  # No image is used while descending.
        h.until(lambda e: e.phase == 'NATIVE_SECOND_FRAME')
        self.assertTrue(h.frame(candidates=[candidate((30., 20.)), candidate((3.21, 2.12))], stamp=1.))
        fix = e.native_goal
        self.assertEqual(fix, (3.21, 2.12, .5))
        self.assertEqual(e.snapshot()['position_xyz'], list(fix))
        self.assertEqual(e.snapshot()['yaw'], e.native_yaw)
        self.assertFalse(h.frame((8., 8.)))
        self.assertEqual(e.native_goal, fix)
        h.until(lambda e: e.phase == 'WAIT_ACK', acknowledge=False)
        phases = [effect['phase'] for effect in h.effects if effect['type'] == 'native_target']
        self.assertEqual(phases, ['NATIVE_INTERMEDIATE_DESCEND', 'NATIVE_SECOND_MOVE', 'NATIVE_DESCEND'])
        self.assertEqual(len(e.native_fixes), 1)
        self.assertEqual(e._route_alignment_status()['required_samples'], 1)
        self.assertIsNone(e.native_offset)
        self.assertEqual(len(releases(h)), 1)
        self.assertEqual(e.snapshot()['reference_source'], 'ALIGN')
        self.assertEqual(e.snapshot()['desired_group'], 'EGO')

    def test_image_wait_has_fifteen_seconds_and_falls_back_to_recorded_point(self):
        h = waiting(); e = h.engine
        deadline(h, before=.01)
        self.assertEqual(e.phase, 'NATIVE_SECOND_FRAME')
        deadline(h)
        self.assertEqual(e.phase, 'NATIVE_DESCEND')
        self.assertEqual(e.native_goal[:2], (3., 2.))
        self.assertTrue(e._delivery_fallback_active)
        self.assertFalse(h.frame((9., 9.)))
        self.assertEqual(e.native_goal[:2], (3., 2.))

    def test_empty_unconfirmed_and_invalid_projection_do_not_extend_image_wait(self):
        h = waiting(); e = h.engine
        original = e._alignment_deadline()
        for values in ([], [candidate((3., 2.), confirmed=False)],
                       [candidate((3., 2.), projection='missing_odometry')],
                       [candidate((float('nan'), 2.))]):
            h.step(dt=3., follow=False)
            self.assertTrue(h.frame(candidates=values))
            self.assertEqual(e.phase, 'NATIVE_SECOND_FRAME')
            self.assertEqual(e._alignment_deadline(), original)
        deadline(h)
        self.assertEqual(e.native_goal[:2], (3., 2.))

    def test_callback_receipt_clock_enforces_deadline_between_ticks(self):
        for elapsed in (15., 15.1):
            with self.subTest(elapsed=elapsed):
                h = waiting(); e = h.engine
                self.assertFalse(h.frame((3.1, 2.1), received_at=h.now+elapsed))
                self.assertEqual(e.phase, 'NATIVE_SECOND_FRAME')
                self.assertFalse(e.native_fixes)
                h.step(dt=elapsed, follow=False)
                self.assertEqual(e.phase, 'NATIVE_DESCEND')
        h = waiting(); e = h.engine
        receipt = h.now+14.99
        self.assertTrue(h.frame((3.1, 2.1), received_at=receipt, stamp=1.))
        self.assertEqual(e.native_motion.started, receipt)

    def test_existing_control_check_stays_at_release_without_new_movement_gate(self):
        h = waiting(); e = h.engine
        one_fix(h)
        h.owner = 'PX4_HOLD'
        h.position = e.native_goal
        h.step(dt=e.native_motion.duration+1., follow=False, acknowledge=False)
        self.assertTrue(e._delivery_release_pending)
        self.assertEqual(e.phase, 'NATIVE_DESCEND')
        self.assertFalse(releases(h))
        h.owner = 'PX4_NATIVE'
        h.velocity = (10., 10., 10.)  # No extra speed/stability requirement.
        h.step(follow=False, acknowledge=False)
        self.assertEqual(e.phase, 'WAIT_ACK')
        self.assertEqual(len(releases(h)), 1)

    def test_move_deadline_retakes_once_and_relaxes_only_xy(self):
        h = waiting(); e = h.engine
        h.frame((3.5, 2.4)); deadline(h, before=.01)
        self.assertEqual(e.phase, 'NATIVE_SECOND_MOVE')
        deadline(h)
        self.assertEqual(e.phase, 'NATIVE_SECOND_FRAME')
        self.assertEqual(e._delivery_retakes, 1)
        self.assertEqual(e.native_goal[:2], h.position[:2])
        self.assertEqual(e._delivery_tolerances(), [.13, .13, .05])
        h.frame((3.4, 2.2)); goal = e.native_goal
        h.position = (goal[0]+.12, goal[1]+.12, goal[2]+.051)
        h.step(dt=e.native_motion.duration+1., follow=False)
        self.assertEqual(e.phase, 'NATIVE_SECOND_MOVE')
        h.position = (goal[0]+.12, goal[1]+.12, goal[2]+.049)
        h.step(follow=False)
        self.assertEqual(e.phase, 'NATIVE_DESCEND')
        self.assertEqual(e.native_goal[:2], (3.4, 2.2))
        self.assertEqual([fix['stage'] for fix in e.native_fixes], [2, 2])
        self.assertFalse(e._delivery_fallback_active)

    def test_second_move_deadline_after_retake_descends_at_last_fix(self):
        h = waiting(); e = h.engine
        h.frame((3.4, 2.2)); deadline(h)
        self.assertEqual(e.phase, 'NATIVE_SECOND_FRAME')
        h.frame((3.9, 2.7)); deadline(h)
        self.assertEqual(e.phase, 'NATIVE_DESCEND')
        self.assertEqual(e.native_goal[:2], (3.9, 2.7))
        self.assertEqual(e._delivery_retakes, 1)
        self.assertTrue(e._delivery_fallback_active)

    def test_new_image_sequence_required_for_retake_without_timestamp_age_gate(self):
        h = waiting(); e = h.engine
        h.frame((3.5, 2.3), sequence=10, stamp=1.)
        self.assertFalse(h.frame(sequence=15, stamp=2.))  # Observed while moving.
        deadline(h)
        self.assertFalse(h.frame(sequence=15, stamp=h.now+100.))
        self.assertFalse(h.frame(sequence=14, stamp=h.now+200.))
        self.assertTrue(h.frame(sequence=16, stamp=1.))
        self.assertEqual(e.phase, 'NATIVE_SECOND_MOVE')

    def test_retake_image_timeout_descends_using_last_attempted_fix(self):
        h = waiting(); e = h.engine
        h.frame((3.7, 2.4)); deadline(h)
        deadline(h)
        self.assertEqual(e.phase, 'NATIVE_DESCEND')
        self.assertEqual(e.native_goal[:2], (3.7, 2.4))
        self.assertEqual(e._delivery_retakes, 1)

    def test_arrival_at_motion_deadline_wins_without_retake(self):
        h = waiting(); e = h.engine
        h.frame((3.3, 2.2)); h.position = e.native_goal
        deadline(h)
        self.assertEqual(e.phase, 'NATIVE_DESCEND')
        self.assertEqual(e._delivery_retakes, 0)

    def test_descend_checks_xy_until_deadline_then_releases_at_actual_position(self):
        h = waiting(); e = h.engine
        one_fix(h)
        nominal = e.native_goal
        h.position = (nominal[0]+.4, nominal[1]-.2, nominal[2])
        h.step(dt=e.native_motion.duration+1., follow=False, acknowledge=False)
        self.assertEqual(e.phase, 'NATIVE_DESCEND')
        self.assertFalse(releases(h))
        deadline(h)
        self.assertEqual(e.phase, 'WAIT_ACK')
        self.assertEqual(e.native_goal, h.position)
        self.assertFalse(e._delivery_forced_height)
        self.assertTrue(e._delivery_fallback_active)
        self.assertEqual(len(releases(h)), 1)

    def test_descent_timeout_uses_actual_height_and_never_retries_release(self):
        h = waiting(); e = h.engine
        one_fix(h)
        actual = h.position
        deadline(h)
        self.assertEqual(e.phase, 'WAIT_ACK')
        self.assertEqual(e.native_goal, actual)
        self.assertIsNone(e.native_motion)
        self.assertTrue(e._delivery_forced_height)
        self.assertAlmostEqual(e._delivery_release_height, actual[2]-h.ground[2])
        h.position = (actual[0]+.2, actual[1], actual[2]+.2)
        h.step(dt=120., follow=False, acknowledge=False)
        self.assertEqual(e.native_goal, actual)
        self.assertEqual(len(releases(h)), 1)
        self.assertEqual(e.drops_completed, 0)

    def test_nonzero_offset_is_once_after_descent_and_timeout_does_not_reapply(self):
        config = configuration()
        config['delivery_route_options'] = dict(mount_offsets_body_m={'1': [.2, -.1]})
        h = waiting(config); e = h.engine
        h.yaw = math.pi/2; h.feedback()
        one_fix(h, (3.1, 2.1))
        self.assertIsNone(e.native_offset)
        h.until(lambda e: e.phase == 'NATIVE_OFFSET_MOVE')
        self.assertAlmostEqual(e.native_goal[0], 3.0)
        self.assertAlmostEqual(e.native_goal[1], 1.9)
        self.assertAlmostEqual(e.native_goal[2], e.drop_z)
        offset = dict(e.native_offset)
        self.assertFalse(h.frame((30., 20.)))
        deadline(h)
        self.assertEqual(e.phase, 'WAIT_ACK')
        self.assertEqual(e.native_offset, offset)
        self.assertEqual(e._delivery_offset_status, 'forced')
        self.assertEqual(sum(x['type'] == 'native_target' and x.get('phase') == 'NATIVE_OFFSET_MOVE'
                             for x in h.effects), 1)
        self.assertEqual(len(releases(h)), 1)

    def test_descent_timeout_nonzero_offset_keeps_measured_z(self):
        config = configuration()
        config['delivery_route_options'] = dict(mount_offsets_body_m={'1': [.2, 0.]})
        h = waiting(config); e = h.engine
        one_fix(h); actual_z = h.position[2]
        deadline(h)
        self.assertEqual(e.phase, 'NATIVE_OFFSET_MOVE')
        self.assertEqual(e.native_goal[2], actual_z)
        self.assertTrue(e._delivery_forced_height)
        h.until(lambda e: e.phase == 'WAIT_ACK', acknowledge=False)
        self.assertEqual(len(releases(h)), 1)

    def test_existing_stale_pose_release_wait_resumes_without_reissuing_motion(self):
        h = waiting(); e = h.engine
        one_fix(h)
        with mock.patch.object(h, 'feedback'):
            deadline(h)
        self.assertEqual(e.phase, 'NATIVE_DESCEND')
        self.assertTrue(e._delivery_release_pending)
        anchor = e.native_goal
        self.assertFalse(releases(h))
        h.position = (anchor[0]+.1, anchor[1], anchor[2])
        h.step(follow=False, acknowledge=False)
        self.assertEqual(e.phase, 'WAIT_ACK')
        self.assertEqual(e.native_goal, anchor)
        self.assertEqual(len(releases(h)), 1)

    def test_all_four_red_points_release_by_route_identity_and_new_task_resets(self):
        h = Replay(); e = h.engine
        for _ in range(5000):
            if e.phase == 'NATIVE_SECOND_FRAME':
                h.frame()
            h.step()
            if e.phase == 'LANDING':
                break
        else:
            self.fail(e.phase)
        self.assertEqual([v['slot'] for v in releases(h)], [1, 2, 3, 4])
        self.assertEqual(len(e.route_released_points), 4)
        self.assertEqual(len(e.native_results), 4)
        self.assertTrue(all(result['alignment']['samples'] == 1 for result in e.native_results))
        e.hold('operator pause')
        self.assertEqual(len(e.route_released_points), 4)
        e.start('delivery_route', h.ground, 0., h.now)
        self.assertEqual(e.route_released_points, set())
        self.assertEqual(e.route_released_slots, [])
        self.assertIsNone(e.red_point_session_id)

    def test_only_success_ack_records_point_and_revisiting_skips_without_new_release(self):
        h = waiting(); e = h.engine
        one_fix(h); h.until(lambda e: e.phase == 'WAIT_ACK', acknowledge=False)
        index, request = e.route_index, e._payload_request
        self.assertEqual(e.route_released_points, set())
        e.on_payload_result(True, request+1); h.step(acknowledge=False)
        self.assertEqual(e.route_released_points, set())
        e.on_payload_result(True, request); h.step(acknowledge=False)
        self.assertEqual(e.route_released_points, {index})
        e._begin_route_drop(e.route_points[index])
        h.step(acknowledge=False)
        self.assertEqual(e.drops_completed, 1)
        self.assertEqual(len(releases(h)), 1)
        self.assertEqual(e.state, 'APPROACH')

    def test_session_invalidation_and_hold_prevent_observations_or_release(self):
        h = waiting(); e = h.engine
        self.assertFalse(h.frame(session='old-task'))
        self.assertFalse(h.frame(sequence=0))
        self.assertFalse(h.frame(sequence=-1))
        e.set_red_point_session(None)
        self.assertFalse(h.frame())
        e.set_red_point_session('new-session')
        self.assertTrue(h.frame(session='new-session', sequence=1, stamp=0.))
        e.hold('operator pause')
        self.assertFalse(h.frame(session='new-session', sequence=2))
        h.step(dt=120., acknowledge=False)
        self.assertEqual(e.state, 'HOLD')
        self.assertFalse(releases(h))

    def test_snapshot_and_terminal_show_native_endpoint_during_alignment_offset_and_recovery(self):
        config = configuration()
        config['delivery_route_options'] = dict(mount_offsets_body_m={'1': [.2, -.1]})
        h = waiting(config); e = h.engine
        h.yaw = math.pi/2; h.feedback()
        h.frame((4., 3.))

        def check_status():
            output = e.snapshot()
            expected = list(e.native_goal)
            self.assertEqual(output['position_xyz'], expected)
            self.assertEqual(output['yaw'], e.native_yaw)
            error = [g-p for g, p in zip(expected, h.position)]
            self.assertAlmostEqual(output['telemetry']['horizontal_distance_m'], math.hypot(*error[:2]))
            self.assertAlmostEqual(output['telemetry']['distance_3d_m'], math.sqrt(sum(v*v for v in error)))
            self.assertEqual(output['telemetry']['target_yaw_rad'], e.native_yaw)
            self.assertAlmostEqual(output['telemetry']['yaw_error_rad'], e.native_yaw-h.yaw)
            lines = []
            TerminalReporter(emit=lines.append).status(dict(output=output, pose=h.position))
            self.assertIn('目标 '+numbers(expected), lines[0])
            return output

        output = check_status()
        self.assertNotEqual(list(output['native_command']['position']), output['position_xyz'])
        h.until(lambda e: e.phase == 'NATIVE_OFFSET_MOVE')
        check_status()
        h.until(lambda e: e.phase == 'NATIVE_RECOVER')
        check_status()


if __name__ == '__main__':
    unittest.main()
