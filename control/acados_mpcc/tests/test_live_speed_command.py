import unittest

import numpy as np

from control.acados_mpcc.ros_shadow import speed_setpoint


class SpeedSetpointTest(unittest.TestCase):
    def test_uses_predicted_speed_and_bounded_steering(self):
        prediction = np.zeros((8, 10))
        prediction[:, 3] = [1., 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7]
        self.assertAlmostEqual(speed_setpoint(prediction, .02, -.15, 2.2)[0], 1.5)
        self.assertEqual(speed_setpoint(prediction, .02, -.4, 2.2)[1], -.3)
        prediction[:, 3] = 4.
        self.assertEqual(speed_setpoint(prediction, .02, 0., 2.2)[0], 2.2)
