"""Numerical geometry checks and ROS-free bridge replay for covariance trial."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from airseafly_real400_mission.native_covariance import (
    checked_source_pose, covariances, fixed_covariances, pose_jacobian, validate_mode, validate_settings)
from airseafly_real400_mission.native_motion import FrozenLioFrame, quaternion, rotation
import test_native_localization_health as bridge_fixture


def exp_rotation(angle):
    """Independent Rodrigues construction for the finite-difference oracle."""
    angle = np.asarray(angle)
    theta = np.linalg.norm(angle)
    if theta == 0.:
        return np.eye(3)
    x, y, z = angle/theta
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + np.sin(theta)*cross + (1.-np.cos(theta))*(cross @ cross)


class CovarianceTests(unittest.TestCase):
    def setUp(self):
        self.frame = FrozenLioFrame((.04, -.03, .08, .99), (.12, -.06, .035))
        self.p = np.array([2., -4., .6])
        self.q = quaternion(exp_rotation((.3, -.2, .7)))
        self.frame.initialize(self.p, self.q, (.2, .3, -.4), (.1, .02, -.3, .95))
        self.variances = (.0025, .0025, .01)

    def test_pose_jacobian_matches_numerical_left_world_perturbations(self):
        base_p, base_q, _, _ = self.frame.transform(self.p, self.q, (0.,0.,0.), (0.,0.,0.))
        base_rotation = rotation(base_q)
        def perturb(error):
            changed_q = quaternion(exp_rotation(error[3:]) @ rotation(self.q))
            changed_p, changed_q, _, _ = self.frame.transform(
                self.p+error[:3], changed_q, (0.,0.,0.), (0.,0.,0.))
            delta = rotation(changed_q) @ base_rotation.T
            dtheta = .5*np.array([delta[2,1]-delta[1,2], delta[0,2]-delta[2,0],
                                  delta[1,0]-delta[0,1]])
            return np.concatenate((changed_p-base_p, dtheta))
        step = 1e-6
        numerical = np.column_stack([(perturb(axis*step)-perturb(-axis*step))/(2*step)
                                     for axis in np.eye(6)])
        np.testing.assert_allclose(pose_jacobian(self.frame, self.q), numerical, atol=1e-9)

    def test_source_correlations_and_psd_survive_transformation(self):
        generator = np.random.default_rng(9827)
        square = generator.normal(size=(6, 6))*.2
        source = square @ square.T + .01*np.eye(6)
        pose, twist, report = covariances('source_pose', source, self.frame, self.q, self.variances)
        jacobian = pose_jacobian(self.frame, self.q)
        np.testing.assert_allclose(pose, jacobian @ source @ jacobian.T, atol=1e-12)
        self.assertGreater(np.linalg.eigvalsh(pose)[0], 0.)
        self.assertGreater(np.max(np.abs(pose-np.diag(np.diag(pose)))), .001)
        self.assertEqual(report['pose'], 'source_pose_with_floors')
        self.assertEqual(report['twist'], 'fixed')
        np.testing.assert_array_equal(twist, fixed_covariances(self.variances, self.frame.t_bi)[1])

    def test_lever_arm_uses_this_samples_attitude_not_initial_attitude(self):
        frame = FrozenLioFrame((0.,0.,0.,1.), (.2,0.,0.))
        frame.initialize((.2,0.,0.), (0.,0.,0.,1.), (0.,0.,0.), (0.,0.,0.,1.))
        source = np.diag([.01, .02, .03, .1, .2, .3])
        q = quaternion(exp_rotation((0.,0.,np.pi/2)))
        pose, _, _ = covariances('source_pose', source, frame, q, (1e-6, 1e-6, .01))
        self.assertAlmostEqual(pose[0,0], .01+.2**2*.3)
        self.assertAlmostEqual(pose[1,1], .02)
        self.assertAlmostEqual(pose[0,5], .2*.3)
        self.assertAlmostEqual(pose[2,3], -.2*.1)

    def test_floors_only_add_diagonal_uncertainty(self):
        source = np.eye(6)*1e-5
        source[0,3] = source[3,0] = 2e-6
        jacobian = pose_jacobian(self.frame, self.q)
        expected = jacobian @ source @ jacobian.T
        pose, _, _ = covariances('source_pose', source, self.frame, self.q, self.variances)
        difference = pose-expected
        np.testing.assert_allclose(difference-np.diag(np.diag(difference)), 0., atol=1e-18)
        self.assertTrue(np.all(np.diag(difference) >= 0.))
        self.assertTrue(np.all(np.diag(pose) >= np.diag(fixed_covariances(self.variances, self.frame.t_bi)[0])))
        self.assertGreaterEqual(np.linalg.eigvalsh(pose)[0], 0.)

    def test_floor_disabled_transports_small_covariance_without_added_uncertainty(self):
        source = np.diag([1., 2., 3., 4., 5., 6.])*1e-7
        source[0,3] = source[3,0] = 2e-8
        jacobian = pose_jacobian(self.frame, self.q)
        expected = jacobian @ source @ jacobian.T
        pose, _, report = covariances('source_pose', source, self.frame, self.q, self.variances,
                                      pose_covariance_floor_enabled=False)
        np.testing.assert_allclose(pose, expected, rtol=1e-14, atol=1e-20)
        self.assertTrue(np.all(np.diag(pose) < .0025))
        self.assertEqual(report['pose'], 'source_pose')
        self.assertEqual(report['pose_scale_applied'], 1.)
        self.assertFalse(report['pose_floor_applied'])

    def test_scale_multiplies_complete_matrix_and_preserves_psd(self):
        generator = np.random.default_rng(1234)
        square = generator.normal(size=(6, 6))*.003
        source = square @ square.T
        baseline, _, _ = covariances('source_pose', source, self.frame, self.q, self.variances,
                                     pose_covariance_floor_enabled=False)
        for scale in (.2, 3., 12.):
            with self.subTest(scale=scale):
                pose, _, report = covariances('source_pose', source, self.frame, self.q, self.variances,
                    pose_covariance_scale=scale, pose_covariance_floor_enabled=False)
                np.testing.assert_allclose(pose, baseline*scale, rtol=1e-14, atol=1e-20)
                self.assertGreater(np.linalg.eigvalsh(pose)[0], 0.)
                self.assertNotEqual(pose[0,3], 0.)
                self.assertEqual(report['pose_scale_applied'], scale)

    def test_scale_applies_before_optional_floors_and_does_not_scale_twist(self):
        source = np.diag([.001, .005, .03, .001, .005, .03])
        q = (0., 0., 0., 1.)
        frame = FrozenLioFrame(q, (0.,0.,0.))
        frame.initialize((0.,0.,0.), q, (0.,0.,0.), q)
        pose, twist, _ = covariances('source_pose', source, frame, q, self.variances,
            pose_covariance_scale=.5, angular_velocity_variance=.08)
        np.testing.assert_allclose(np.diag(pose), [.0025, .0025, .015]*2)
        np.testing.assert_array_equal(np.diag(twist), [.01]*3+[.08]*3)

    def test_fixed_and_fallback_ignore_source_scale_but_use_configured_values(self):
        values = (.04, .09, .16)
        expected_pose, expected_twist = fixed_covariances(values, self.frame.t_bi, .25)
        for mode in ('fixed', 'source_pose'):
            with self.subTest(mode=mode):
                pose, twist, report = covariances(mode, np.zeros((6,6)), self.frame, self.q, values,
                    pose_covariance_scale=4., pose_covariance_floor_enabled=False, angular_velocity_variance=.25)
                np.testing.assert_array_equal(pose, expected_pose)
                np.testing.assert_array_equal(twist, expected_twist)
                self.assertIsNone(report['pose_scale_applied'])
                self.assertFalse(report['pose_floor_applied'])
        # Historical three-argument callers still borrow the angular value.
        _, legacy_twist = fixed_covariances(values, self.frame.t_bi)
        np.testing.assert_array_equal(np.diag(legacy_twist)[3:], [.09]*3)

    def test_invalid_configuration_raises_even_when_source_is_invalid_or_mode_fixed(self):
        changes = [dict(pose_covariance_scale=value) for value in
                   (0., -1., float('nan'), float('inf'), True, '2', None)]
        changes += [dict(pose_covariance_floor_enabled=value) for value in (0, 1, 'false', None)]
        changes += [dict(angular_velocity_variance=value) for value in
                    (0., -1., float('nan'), float('inf'), False, '.02')]
        for mode in ('fixed', 'source_pose'):
            for options in changes:
                with self.subTest(mode=mode, options=options), self.assertRaises(ValueError):
                    covariances(mode, np.zeros(36), self.frame, self.q, self.variances, **options)
        for index in range(3):
            for invalid in (0., -1., float('nan'), float('inf'), True, '.02'):
                values = list(self.variances)
                values[index] = invalid
                with self.subTest(index=index, invalid=invalid), self.assertRaises(ValueError):
                    validate_settings(values)

    def test_invalid_source_covariance_falls_back_and_explains_reason(self):
        asymmetric = np.eye(6)
        asymmetric[0,1] = .1
        indefinite = np.eye(6)
        indefinite[0,1] = indefinite[1,0] = 2.
        cases = (np.zeros(36), np.diag([-1.,1.,1.,1.,1.,1.]), [float('nan')]*36,
                 asymmetric, indefinite, indefinite*1e-14, [1.]*35, None)
        fixed_pose, fixed_twist = fixed_covariances(self.variances, self.frame.t_bi)
        for source in cases:
            with self.subTest(source=repr(source)[:60]):
                pose, twist, report = covariances('source_pose', source, self.frame, self.q, self.variances)
                np.testing.assert_array_equal(pose, fixed_pose)
                np.testing.assert_array_equal(twist, fixed_twist)
                self.assertEqual(report['pose'], 'fixed_fallback')
                self.assertIs(report['source_pose_valid'], False)
                self.assertTrue(report['fallback_reason'])

    def test_fixed_mode_preserves_old_output_and_ignores_source_covariance(self):
        pose, twist, report = covariances('fixed', [float('nan')]*36, self.frame, self.q, self.variances)
        pv, rv, vv = self.variances
        np.testing.assert_array_equal(pose, np.diag([pv+float(self.frame.t_bi@self.frame.t_bi)*rv]*3+[rv]*3))
        np.testing.assert_array_equal(twist, np.diag([vv]*3+[rv]*3))
        self.assertIsNone(report['source_pose_valid'])
        self.assertEqual(report['fallback_reason'], '')

    def test_unknown_mode_is_configuration_error(self):
        with self.assertRaisesRegex(ValueError, 'fixed or source_pose'):
            validate_mode('source')
        with self.assertRaises(ValueError):
            covariances('typo', np.eye(6), self.frame, self.q, self.variances)

    def test_tiny_roundoff_may_be_removed_but_unknown_zero_axes_are_rejected(self):
        source = np.ones((6,6))
        self.assertGreaterEqual(np.linalg.eigvalsh(checked_source_pose(source))[0], -1e-14)
        source[0,0] = 0.
        with self.assertRaisesRegex(ValueError, 'unknown or nonpositive'):
            checked_source_pose(source)


class BridgeCovarianceTests(unittest.TestCase):
    def setUp(self):
        self.replay = bridge_fixture.LiveHealthTests()
        self.replay.setUp()
        self.bridge = self.replay.bridge
        self.bridge.covariance_mode = 'source_pose'

    def test_bridge_uses_same_message_covariance_and_reports_fixed_twist(self):
        self.replay.now = 12.
        message = bridge_fixture.odometry(12.)
        source = np.eye(6)*.02
        source[0,4] = source[4,0] = .002
        message.pose.covariance = source.reshape(-1).tolist()
        self.replay.feed(message)
        status = self.replay.tick()
        output = self.bridge.output.messages[-1]
        self.assertEqual(output.header.stamp.to_sec(), 12.)
        np.testing.assert_allclose(np.array(output.pose.covariance).reshape(6,6), source)
        self.assertEqual(status['covariance_mode'], 'source_pose')
        self.assertEqual(status['covariance']['twist'], 'fixed')
        self.assertFalse(status['covariance_calibration_verified'])
        self.assertEqual(status['covariance_fallback_count'], 0)

    def test_zero_source_covariance_is_counted_fallback_then_next_valid_sample_recovers(self):
        self.replay.now = 12.
        self.replay.feed(bridge_fixture.odometry(12.))
        status = self.replay.tick()
        self.assertEqual(status['covariance_fallback_count'], 1)
        self.assertEqual(status['covariance']['pose'], 'fixed_fallback')
        self.assertTrue(status['healthy'])
        self.replay.now = 12.1
        valid = bridge_fixture.odometry(12.1)
        valid.pose.covariance = np.eye(6).reshape(-1).tolist()
        self.replay.feed(valid)
        status = self.replay.tick()
        self.assertEqual(status['covariance']['pose'], 'source_pose_with_floors')
        self.assertEqual(status['covariance_fallback_count'], 1)
        self.assertEqual(status['covariance']['fallback_reason'], '')

    def test_bridge_exposes_manual_settings_and_fallback_does_not_apply_scale(self):
        self.bridge.pose_covariance_scale = 3.
        self.bridge.pose_covariance_floor_enabled = False
        self.bridge.angular_velocity_variance = .07
        self.bridge.variances = (.04, .05, .06)
        self.replay.now = 12.
        self.replay.feed(bridge_fixture.odometry(12.))
        status = self.replay.tick()
        self.assertEqual(status['covariance_settings'], dict(position_variance=.04, orientation_variance=.05,
            velocity_variance=.06, angular_velocity_variance=.07,
            pose_covariance_scale=3., pose_covariance_floor_enabled=False))
        self.assertIsNone(status['covariance']['pose_scale_applied'])
        output = self.bridge.output.messages[-1]
        np.testing.assert_allclose(np.diag(np.asarray(output.pose.covariance).reshape(6,6)), [.04]*3+[.05]*3)
        np.testing.assert_allclose(np.diag(np.asarray(output.twist.covariance).reshape(6,6)), [.06]*3+[.07]*3)
        self.replay.now = 12.1
        valid = bridge_fixture.odometry(12.1)
        source = np.eye(6)*1e-6
        source[0,3] = source[3,0] = 1e-7
        valid.pose.covariance = source.reshape(-1).tolist()
        self.replay.feed(valid)
        status = self.replay.tick()
        output = self.bridge.output.messages[-1]
        np.testing.assert_allclose(np.asarray(output.pose.covariance).reshape(6,6), 3.*source)
        self.assertEqual(status['covariance']['pose'], 'source_pose')
        self.assertEqual(status['covariance']['pose_scale_applied'], 3.)
        self.assertFalse(status['covariance']['pose_floor_applied'])
        self.assertEqual(status['covariance_fallback_count'], 1)

    def test_bridge_rejects_bad_ros_parameters_before_creating_publishers(self):
        invalid = [('~pose_covariance_scale', '2'), ('~pose_covariance_scale', 0),
                   ('~pose_covariance_floor_enabled', 'false'), ('~pose_covariance_floor_enabled', 1),
                   ('~angular_velocity_variance', None), ('~angular_velocity_variance', float('nan')),
                   ('~position_variance', True), ('~orientation_variance', -1.), ('~velocity_variance', 0.)]
        for key, value in invalid:
            with self.subTest(key=key, value=value):
                parameters = {'~mount/quaternion_xyzw': (0.,0.,0.,1.),
                              '~mount/translation_xyz_m': (0.,0.,0.), key: value}
                with patch.object(bridge_fixture.bridge_module.rospy, 'get_param',
                        lambda name, default=None: parameters.get(name, default), create=True), \
                     patch.object(bridge_fixture.bridge_module.rospy, 'Publisher', create=True) as publisher:
                    with self.assertRaises(ValueError):
                        bridge_fixture.bridge_module.Bridge()
                    publisher.assert_not_called()


if __name__ == '__main__':
    unittest.main()
