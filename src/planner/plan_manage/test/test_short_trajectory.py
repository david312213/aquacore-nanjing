"""Native spline math and collision checks; not a ROS/optimizer integration test."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[4]
EIGEN = next((p for p in (os.environ.get('FASTLIO_EIGEN_INCLUDE', ''), '/usr/include/eigen3',
                          '/usr/local/include/eigen3', '/opt/homebrew/include/eigen3')
              if p and (Path(p)/'Eigen/Core').exists()), None)


class ShortTrajectoryTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('c++') and EIGEN, 'C++ compiler and Eigen required')
    def test_real_spline_math_and_collision_sampling(self):
        with tempfile.TemporaryDirectory() as temp:
            d = Path(temp)
            (d/'ros').mkdir()
            (d/'ros/ros.h').write_text('// uniform_bspline.cpp does not use ROS APIs\n')
            binary = d/'short_test'
            command = ['c++', '-std=c++14', '-O2', '-I'+str(d), '-isystem', EIGEN,
                       '-I'+str(ROOT/'src/planner/plan_manage/include'),
                       '-I'+str(ROOT/'src/planner/bspline_opt/include'),
                       str(Path(__file__).with_suffix('.cpp')),
                       str(ROOT/'src/planner/bspline_opt/src/uniform_bspline.cpp'), '-o', str(binary)]
            result = subprocess.run(command, capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stderr)
            result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
