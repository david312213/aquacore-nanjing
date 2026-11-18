"""Post-drop climb acceleration is separate from the alignment acceleration.

Offline only: no ROS, services, cameras, servos or aircraft.
"""
import os
from pathlib import Path
import subprocess
import sys
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from airseafly_real400_mission import native_parameters
import test_single_drop as single


def motions(recover_acceleration=None, align_acceleration=.5):
    """Acceleration of every native move in one single-drop flight, by phase."""
    config = single.configuration(slot=1)
    config['motion']['native_acceleration_mps2'] = align_acceleration
    config['motion']['recover_acceleration_mps2'] = recover_acceleration
    h = single.SingleDropReplay(config)
    seen, last = {}, None
    for _ in range(20000):
        e = h.engine
        if e.native_motion is not None and e.native_motion is not last:
            last = e.native_motion
            seen.setdefault(e.phase, e.native_motion.acceleration)
        if e.phase in ('LANDING', 'LANDED') or e.done:
            break
        if e.phase in ('NATIVE_FIRST_FRAME', 'NATIVE_SECOND_FRAME'):
            h.frame(xy=(-2.95, 6.04))  # A red point near the target: real alignment moves.
        h.step()
    return seen


class RecoverAccelerationTests(unittest.TestCase):
    def test_null_keeps_the_shared_alignment_acceleration(self):
        seen = motions(None, .5)
        self.assertIn('NATIVE_RECOVER', seen)
        self.assertTrue(all(value == .5 for value in seen.values()), seen)

    def test_separate_value_changes_only_the_climb(self):
        seen = motions(1.6, .3)
        self.assertEqual(seen['NATIVE_RECOVER'], 1.6)
        others = {phase: value for phase, value in seen.items() if phase != 'NATIVE_RECOVER'}
        self.assertIn('NATIVE_DESCEND', others)
        self.assertTrue(any(phase.endswith('_MOVE') for phase in others), others)
        self.assertTrue(all(value == .3 for value in others.values()), others)

    def test_validation_and_summary(self):
        config = single.configuration()
        config['motion']['recover_acceleration_mps2'] = -1.
        with self.assertRaisesRegex(ValueError, 'recover_acceleration'):
            native_parameters.validate(config)
        config['motion']['recover_acceleration_mps2'] = 1.2
        native_parameters.validate(config)
        value = native_parameters.delivery_motion(config)
        self.assertEqual((value['recover_acceleration_mps2'], value['recover_acceleration_separate']), (1.2, True))
        self.assertIn('回升加速度 1.20', native_parameters.delivery_motion_line(value))

    def test_command_line_option_overrides_and_preview_shows_it(self):
        env = dict(os.environ, AIRSEAFLY_PYTHON=sys.executable)
        run = subprocess.run(['sh', str(ROOT/'flight/drop_test.sh'), '--x', '1', '--y', '2', '--slot', '1',
                              '--recover-acceleration', '1.5', '--align-acceleration', '0.4', '--dry-run'],
                             env=env, text=True, capture_output=True, timeout=60)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertIn('对准/下降加速度 0.40 m/s²，回升加速度 1.50 m/s²。', run.stdout)


if __name__ == '__main__':
    unittest.main()
