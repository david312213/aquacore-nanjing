"""Mode acknowledgement and touchdown evidence; no ROS or aircraft endpoints."""
import unittest

from test_native_delivery_route import Replay


class LandingConfirmationTests(unittest.TestCase):
    def queued(self):
        h = Replay()
        e = h.engine
        e._state('LAND', 'WAIT_LAND', 'NONE')
        e._tick_native_landing()
        e.on_native_owner_result('PX4_AUTO_LAND', True, 'PX4_AUTO_LAND', 'request queued')
        self.assertEqual(e.phase, 'WAIT_LAND_MODE')
        self.assertFalse(e._land_sent)
        return h

    def feedback_tick(self, h, *, age=0., state_age=0., extended_age=0., **values):
        h.now += .05
        h.feedback()
        e = h.engine
        packet = dict(e.native_feedback, stamp=h.now-age, state_stamp=h.now-state_age,
                      extended_state_stamp=h.now-extended_age)
        packet.update(values)
        e.on_native_feedback(packet)
        return e.tick(h.now, h.position, 0., h.owner)

    def test_queued_owner_and_gateway_confirmed_do_not_replace_actual_mode(self):
        h = self.queued(); e = h.engine
        for gateway in ('requesting', 'confirmed'):
            self.feedback_tick(h, owner='PX4_AUTO_LAND', landing_state=gateway,
                               mode='OFFBOARD', landed=False, armed=True)
            self.assertEqual(e.phase, 'WAIT_LAND_MODE')
            self.assertFalse(e.done)
            self.assertFalse(e._land_sent)
        self.feedback_tick(h, mode='AUTO.LAND', landing_state='confirmed', landed=False, armed=True)
        self.assertEqual(e.phase, 'LANDING')
        self.assertEqual(e.snapshot()['native']['landing']['state'], 'confirmed')
        self.assertFalse(e.done)

    def test_current_mode_feedback_does_not_add_another_timestamp_gate(self):
        h = self.queued(); e = h.engine
        self.feedback_tick(h, age=20., state_age=20., mode='AUTO.LAND', landed=False, armed=True)
        self.assertEqual(e.phase, 'LANDING')
        self.assertFalse(e.done)

    def test_contact_or_disarmed_alone_cannot_complete_landing(self):
        h = self.queued(); e = h.engine
        self.feedback_tick(h, mode='OFFBOARD', landed=True, armed=True)
        self.assertEqual(e.phase, 'WAIT_LAND_MODE')
        self.assertFalse(e.done)
        self.feedback_tick(h, mode='AUTO.LAND', landed=True, armed=True)
        self.assertEqual(e.phase, 'LANDING'); self.assertFalse(e.done)
        self.feedback_tick(h, mode='AUTO.LAND', landed=False, armed=False)
        self.assertFalse(e.done)
        self.feedback_tick(h, mode='MANUAL', landed=True, armed=False, state_age=20., extended_age=20.)
        self.assertEqual((e.phase, e.done), ('LANDED', True))

    def test_landed_disarmed_completes_when_intermediate_auto_land_sample_is_missed(self):
        for gateway in ('requesting', 'confirmed', 'cancelled'):
            with self.subTest(gateway=gateway):
                h = self.queued(); e = h.engine
                self.feedback_tick(h, mode='MANUAL', landing_state=gateway, landed=True, armed=False)
                self.assertEqual((e.phase, e.done), ('LANDED', True))
                self.assertEqual(e.native_landing_state, 'confirmed')

    def test_exhausted_gateway_attempts_report_unconfirmed_without_success(self):
        for state in ('failed', 'cancelled'):
            with self.subTest(state=state):
                h = self.queued(); e = h.engine
                self.feedback_tick(h, mode='OFFBOARD', owner='PX4_HOLD', landing_state=state,
                    landing_attempts=3, landing_reason='mode confirmation deadline exceeded')
                self.assertEqual(e.state, 'HOLD')
                self.assertIn('降落未确认', e.reason)
                self.assertFalse(e.done)
                self.assertEqual(e.snapshot()['native']['landing']['attempts'], 3)
                self.feedback_tick(h, mode='AUTO.LAND', landed=True, armed=False)
                self.assertEqual(e.state, 'HOLD')
                self.assertFalse(e.done)

    def test_pause_ignores_late_queue_ack_and_late_mode(self):
        h = self.queued(); e = h.engine
        e.hold('pilot pause')
        e.on_native_owner_result('PX4_AUTO_LAND', True, 'PX4_AUTO_LAND', 'late queued reply')
        self.feedback_tick(h, mode='AUTO.LAND', landed=True, armed=False)
        self.assertEqual(e.state, 'HOLD')
        self.assertFalse(e.done)

    def test_missing_gateway_terminal_status_has_bounded_confirmation_wait(self):
        for stale in (False, True):
            h = self.queued(); e = h.engine
            h.now += 9.95
            self.feedback_tick(h, age=20. if stale else 0., mode='OFFBOARD', landing_state='requesting')
            self.assertEqual(e.state, 'HOLD')
            self.assertIn('降落未确认', e.reason)
            self.assertFalse(e.done)
            self.assertEqual(e.native_landing_state, 'failed')

    def test_missing_status_and_task_pose_still_reach_ten_second_deadline(self):
        for source in ('lio', 'px4'):
            with self.subTest(source=source):
                h = self.queued(); e = h.engine
                e.config['flight'].update(native_task_feedback=source, native_reference_mode='lio_direct')
                e.native_feedback = e.native_lio_feedback = e.native_task_feedback = None
                h.now += 10.01
                e.tick(h.now, h.position, 0., h.owner)
                self.assertEqual(e.state, 'HOLD')
                self.assertIn('降落未确认', e.reason)
                self.assertEqual(e.native_landing_state, 'failed')
                self.assertFalse(e.done)


