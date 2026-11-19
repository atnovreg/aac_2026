import unittest

import numpy as np

from navigation.localization3d.particle_localization import ParticleInitializer


def pose(x=0., y=0., z=.055):
    result = np.eye(4)
    result[:3, 3] = [x, y, z]
    return result


class ParticleInitializerTest(unittest.TestCase):
    def test_similar_aisles_stay_ambiguous_until_new_geometry_appears(self):
        x, z = np.meshgrid(np.arange(-1., 5.01, .15), np.arange(0., 1.51, .15))
        walls = np.concatenate([
            np.c_[x.ravel(), np.full(x.size, side), z.ravel()]
            for side in [-1., 1.]
        ])
        y, z = np.meshgrid(np.arange(-1., 1.01, .12), np.arange(.2, 1.4, .12))
        end_wall = np.c_[np.full(y.size, 3.), y.ravel(), z.ravel()]
        cloud = np.concatenate([walls, walls + [0., 6., 0.], end_wall])
        usable = np.zeros((90, 60), dtype=bool)
        usable[10:30, 10:42] = True
        usable[70:90, 10:42] = True
        initializer = ParticleInitializer(cloud, usable, [-1., -2.], .1, [0., 0., 0.])
        start = walls[(walls[:, 0] > -.9) & (walls[:, 0] < 1.2)]
        for _ in range(4):
            estimate, _ = initializer.update(pose(), start - [0., 0., .055])
            self.assertIsNone(estimate)

        for step in np.arange(.25, 2.01, .25):
            nearby = walls[abs(walls[:, 0] - step) < 1.1]
            if step >= 1.:
                nearby = np.concatenate([nearby, end_wall])
            estimate, _ = initializer.update(pose(step), nearby - [step, 0., .055])
            if estimate is not None:
                break
        self.assertIsNotNone(estimate)
        np.testing.assert_allclose(estimate[:2, 3], [step, 0.], atol=.28)
        self.assertGreater(estimate[0, 0], .95)


if __name__ == '__main__':
    unittest.main()
