"""A virtual-progress jump must never count as physical lap completion."""
import unittest
from pathlib import Path
import numpy as np

from control.acados_mpcc.track import Track
from control.acados_mpcc.simulation import assess_pose, input_increment_violation


class PhysicalProgressTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.track = Track.from_csv(Path('tracks/indoor_final/track.csv'))

    def test_virtual_progress_cannot_fake_distance(self):
        ref = self.track.reference(2.)
        state = np.r_[ref['center'], ref['yaw'], 1., 0., 0., 999.]
        progress, _, violation = assess_pose(self.track, state, 2., .045)
        self.assertAlmostEqual(progress, 2., places=5)
        self.assertLessEqual(violation, 0.)

    def test_outside_corridor_is_reported(self):
        ref = self.track.reference(2.)
        tangent = ref['tangent']
        xy = ref['center'] + (ref['width_left'] + .2)*np.array([-tangent[1], tangent[0]])
        state = np.r_[xy, ref['yaw'], 1., 0., 0., 2.]
        _, _, violation = assess_pose(self.track, state, 2., .045)
        self.assertGreater(violation, .15)

    def test_increment_validation_uses_normalized_steering_change(self):
        previous = np.array([.2, -.2, 1.])
        # Both steering angles are within absolute bounds; the change is too large.
        self.assertAlmostEqual(input_increment_violation([.2, .2, 1.], previous), 1/7)
        self.assertEqual(input_increment_violation([.2, .15, 1.], previous), 0.)


if __name__ == '__main__':
    unittest.main()
