import unittest

from fusion_checks import verify_ev_state


def records(enabled=True, primary=1, age=.1):
    return [
        {'command': 'listener estimator_selector_status -n 1',
         'output': '\n timestamp: 123 (%s seconds ago)\n primary_instance: %s\n' % (age, primary)},
        {'command': 'listener estimator_status_flags -n 1 -i 1',
         'output': '\n timestamp: 124 (%s seconds ago)\n cs_ev_pos: %s\n cs_ev_hgt: %s\n'
                   ' cs_ev_vel: False\n cs_ev_yaw: False\n' % (age, enabled, enabled)},
    ]


class FusionChecksTest(unittest.TestCase):
    def test_enabled_and_timed_out_states(self):
        for enabled in (False, True):
            self.assertEqual(verify_ev_state(records(enabled), 1, enabled)['cs_ev_hgt'], enabled)

    def test_wrong_state_is_not_a_reset(self):
        for enabled in (False, True):
            with self.assertRaises(RuntimeError):
                verify_ev_state(records(not enabled), 1, enabled)

    def test_stale_or_switched_estimator_rejected(self):
        for values in (dict(age=3), dict(primary=0)):
            with self.assertRaises(RuntimeError):
                verify_ev_state(records(**values), 1, True)

    def test_missing_or_unexpected_yaw_fusion_rejected(self):
        for replacement in ('', ' cs_ev_yaw: True\n'):
            data = records()
            data[1]['output'] = data[1]['output'].replace(' cs_ev_yaw: False\n', replacement)
            with self.assertRaises(RuntimeError):
                verify_ev_state(data, 1, True)


if __name__ == '__main__':
    unittest.main()
