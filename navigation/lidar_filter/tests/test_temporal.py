"""Synthetic visibility evidence must preserve static, unseen and occluded points."""
import unittest
import numpy as np
from tools.lidar_filter.temporal import TemporalVoxelEvidence


class TemporalTests(unittest.TestCase):
    def evidence(self, points, **kwargs):
        return TemporalVoxelEvidence(np.array(points, dtype=float), floor_plane=(0, 0, 0),
                                     min_free_ratio=.7, **kwargs)

    def test_repeated_clear_rays_remove_departed_target_but_keep_wall_and_floor(self):
        p = np.array([[1.05, .05, 1.05], [3.05, .05, 1.05], [1.05, .05, .05]])
        e = self.evidence(p)
        origin = np.array([.05, .05, 1.05])
        e.observe(origin, p[:1], 0)
        for t in [1., 2., 3.]:
            e.observe(origin, p[1:2], t)
        self.assertEqual(e.removal_mask(p).tolist(), [True, False, False])

    def test_missing_ray_and_occlusion_are_not_free_space(self):
        p = np.array([[1.05, .05, 1.05], [2.05, 1.05, 1.05], [3.05, .05, 1.05]])
        e = self.evidence(p)
        for t in range(5):
            e.observe([.05, .05, 1.05], p[:1], t)
        self.assertFalse(e.removal_mask(p).any())

    def test_duplicate_rays_and_nearby_times_cannot_manufacture_three_frames(self):
        p = np.array([[1.05, .05, 1.05]])
        e = self.evidence(p)
        rays = np.tile([3.05, .05, 1.05], (100, 1))
        for t in [0, .1, .2]:
            e.observe([.05, .05, 1.05], rays, t)
        self.assertFalse(e.removal_mask(p).any())
        self.assertEqual(int(e.free_counts.max()), 1)

    def test_hit_support_overrides_clear_ray_in_same_frame(self):
        p = np.array([[1.05, .05, 1.05]])
        e = self.evidence(p)
        for t in range(5):
            e.observe([.05, .05, 1.05], np.array([[3.05, .05, 1.05], *p]), t)
        self.assertFalse(e.removal_mask(p).any())
        self.assertEqual(int(e.free_counts.max()), 0)

    def test_per_ray_origins_and_endpoint_margin_and_height_protection(self):
        p = np.array([[1.05, .05, 1.05], [1.05, 1.05, 1.05], [1.05, 2.05, .15]])
        e = self.evidence(p, height_range=(.2, 1.9))
        origins = np.array([[.05, .05, 1.05], [.05, 1.05, 1.05], [.05, 2.05, .15]])
        endpoints = np.array([[3.05, .05, 1.05], [1.2, 1.05, 1.05], [3.05, 2.05, .15]])
        for t in range(4):
            e.observe(origins, endpoints, t)
        self.assertEqual(e.removal_mask(p).tolist(), [True, False, False])

    def test_static_support_and_negative_coordinates(self):
        p = np.array([[-1.05, -.05, 1.05]])
        e = self.evidence(p, min_static_frames=3)
        for t in range(3):
            e.observe([-3.05, -.05, 1.05], p, t)
        for t in range(3, 20):
            e.observe([-3.05, -.05, 1.05], np.array([[2.05, -.05, 1.05]]), t)
        self.assertFalse(e.removal_mask(p).any())


if __name__ == '__main__':
    unittest.main()
