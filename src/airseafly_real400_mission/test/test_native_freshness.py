"""Source progress must be fresh on both ROS and elapsed-time clocks."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from airseafly_real400_mission.native_freshness import SourceProgress, fresh_stamp


class FreshnessTests(unittest.TestCase):
    def test_absent_invalid_old_and_future_stamps(self):
        for stamp in (None, 'bad', 0., -1., float('nan'), float('inf'), 9.699, 10.051):
            self.assertFalse(fresh_stamp(stamp, 10.), stamp)
        for stamp in (9.701, 10., 10.049):
            self.assertTrue(fresh_stamp(stamp, 10.), stamp)

    def test_republishing_old_sample_does_not_renew_receipt_time(self):
        source = SourceProgress()
        self.assertFalse(source.fresh(10., 1.))
        self.assertTrue(source.observe(10., 1.))
        self.assertTrue(source.fresh(10.299, 1.299))
        self.assertFalse(source.observe(10., 1.2))
        self.assertFalse(source.observe(9.99, 1.25))
        self.assertEqual((source.stamp, source.received), (10., 1.))
        self.assertFalse(source.fresh(10.301, 1.301))

    def test_frozen_or_rewound_ros_clock_cannot_keep_cached_data_alive(self):
        source = SourceProgress()
        source.observe(10., 1.)
        self.assertFalse(source.fresh(10., 1.301))
        self.assertFalse(source.fresh(9., 1.01))
        self.assertFalse(source.fresh(10.01, .99))
        self.assertTrue(source.observe(10.4, 1.4))
        self.assertTrue(source.fresh(10.4, 1.4))

    def test_invalid_observation_does_not_poison_next_valid_sample(self):
        source = SourceProgress()
        for stamp, received in ((None,1.),('bad',1.),(float('nan'),1.),(10.,float('inf'))):
            self.assertFalse(source.observe(stamp, received))
        self.assertTrue(source.observe(10., 1.))
        self.assertTrue(source.fresh(10., 1.))


if __name__ == '__main__': unittest.main()
