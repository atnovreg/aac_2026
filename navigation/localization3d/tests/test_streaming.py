import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from navigation.localization3d.estimator import Registration
from navigation.localization3d.streaming import MapSurfaces, MapTracker, initial_pose, stamp_is_fresh


class LiveRegistrationTest(unittest.TestCase):
    def setUp(self):
        self.seed = initial_pose([5., 9., -.8, .02, -.01, 1.3])
        self.base_from_imu = initial_pose([.12, .02, .15, 0., 0., -np.pi/2])
        self.tracker = MapTracker(None, self.seed, self.base_from_imu)
        self.points = np.ones((120, 3))

    @staticmethod
    def accept(world, points, prior, **kwargs):
        return Registration(prior.copy(), True, 'accepted')

    def test_pose_and_twist_refer_to_the_axle_midpoint(self):
        # The FAST-LIO origin is offset from the axle midpoint and rotated 90°.
        odom0 = initial_pose([0., 0., 0., 0., 0., .1])
        body0 = odom0 @ np.linalg.inv(self.base_from_imu)
        motion = initial_pose([.09, -.01, .005, 0., 0., .12])
        with patch('navigation.localization3d.streaming.register', side_effect=self.accept):
            first, _ = self.tracker.update(1_000_000_000, odom0, self.points)
            self.assertIsNone(first)
            value, _ = self.tracker.update(1_100_000_000, body0 @ motion @ self.base_from_imu, self.points)
        pose, velocity, rate = value
        np.testing.assert_allclose(pose, self.seed @ motion, atol=1e-12)
        np.testing.assert_allclose(velocity, motion[:3, :3].T @ motion[:3, 3]/.1, atol=1e-12)
        np.testing.assert_allclose(rate, [0., 0., 1.2], atol=1e-12)

    def test_rejected_corrections_do_not_publish_and_clock_rewind_requires_restart(self):
        odom = np.eye(4)
        rejected = Registration(odom, False, 'low correspondence support')
        with patch('navigation.localization3d.streaming.register', side_effect=[
            self.accept(None, self.points, odom), rejected
        ]):
            self.tracker.update(2_000_000_000, odom, self.points)
            self.assertIsNone(self.tracker.update(2_100_000_000, odom, self.points)[0])
            self.assertIsNone(self.tracker.update(10_000_000, odom, self.points)[0])
            self.assertTrue(self.tracker.requires_restart)
            self.assertIsNone(self.tracker.update(2_200_000_000, odom, self.points)[0])

    def test_search_preserves_matches_inside_the_registration_gate(self):
        world = MapSurfaces.__new__(MapSurfaces)
        world.centres = np.array([[0., 0., 0.], [2., 0., 0.]])
        world.normals = np.array([[0., 0., 1.], [0., 1., 0.]])
        world.tree = cKDTree(world.centres)
        centres, normals, distances = world.nearest_faces(np.array([
            [0., .4, 0.], [2., .59, 0.], [10., 0., 0.],
        ]))
        np.testing.assert_allclose(centres[:2], world.centres)
        np.testing.assert_allclose(normals[:2], world.normals)
        np.testing.assert_allclose(distances, [.4, .59, np.inf])

    def test_clock_after_registration_can_invalidate_a_previously_fresh_scan(self):
        stamp = 1_795_000_000_000_000_000
        self.assertTrue(stamp_is_fresh(stamp, stamp + 20_000_000, .5))
        self.assertFalse(stamp_is_fresh(stamp, stamp + 500_000_001, .5))
        self.assertFalse(stamp_is_fresh(stamp, stamp - 50_000_001, .5))
        self.assertFalse(stamp_is_fresh(stamp, 0, .5))

    def test_map_alignment_correction_does_not_introduce_velocity(self):
        odom = np.eye(4)
        def corrected(world, points, prior, **kwargs):
            pose = prior.copy()
            pose[0, 3] += .04
            pose[:3, :3] = Rotation.from_euler('z', .03).as_matrix() @ pose[:3, :3]
            return Registration(pose, True, 'accepted')
        with patch('navigation.localization3d.streaming.register', side_effect=corrected):
            self.tracker.update(1_000_000_000, odom, self.points)
            (pose, velocity, rate), _ = self.tracker.update(1_100_000_000, odom, self.points)
        self.assertGreater(abs(pose[0, 3]-self.seed[0, 3]), .01)
        np.testing.assert_allclose(velocity, 0., atol=1e-12)
        np.testing.assert_allclose(rate, 0., atol=1e-12)


if __name__ == '__main__':
    unittest.main()
