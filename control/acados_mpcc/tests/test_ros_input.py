import math
from types import SimpleNamespace as Fields
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from control.acados_mpcc.ros_input import Feedback
from control.acados_mpcc.track import Track


def vector(values):
    return Fields(**dict(zip("xyzw", values)))


class FeedbackTests(unittest.TestCase):
    def setUp(self):
        s = np.linspace(0., math.tau, 120, endpoint=False)
        self.track = Track(s, np.column_stack((np.cos(s), np.sin(s))),
                           np.ones_like(s), np.ones_like(s), math.tau)
        self.feedback = Feedback(self.track)

    def odom(self, stamp, progress=0., rpy=(0., 0., 0.), velocity=(1., 0., 0.), omega=(0., 0., 0.)):
        sec, ns = divmod(stamp, 10**9)
        xy = self.track.reference(progress)["center"]
        pose = Fields(position=vector([*xy, .1]),
                      orientation=vector(Rotation.from_euler("xyz", rpy).as_quat()))
        return Fields(header=Fields(frame_id="indoor_map", stamp=Fields(sec=sec, nanosec=ns)),
                      child_frame_id="base_link", pose=Fields(pose=pose),
                      twist=Fields(twist=Fields(linear=vector(velocity), angular=vector(omega))))

    def test_seam_gap_reverse_and_clock_restart(self):
        stamp = 10**9
        def update(progress, yaw, delta):
            nonlocal stamp
            stamp += delta
            return self.feedback.update(self.odom(stamp, progress, (0, 0, yaw)), stamp)
        update(math.tau - .1, math.pi - .1, 0)
        forward = update(.1, -math.pi + .1, 100_000_000)
        np.testing.assert_allclose(forward[[2, 6]], [math.pi + .1, math.tau + .1], atol=1e-6)
        gap = update(2.6, -math.pi + .2, 800_000_000)
        self.assertAlmostEqual(gap[6], math.tau + 2.6, places=5)
        reverse = update(2.55, -math.pi + .15, 100_000_000)
        self.assertLess(reverse[6], gap[6])
        restarted = update(.1, -math.pi + .1, -900_000_000)
        np.testing.assert_allclose(restarted[[2, 6]], [-math.pi + .1, .1], atol=1e-6)

    def test_tilted_body_linear_and_angular_to_planar_velocity(self):
        r, p, y = .3, -.25, .8
        velocity = [1.1, -.3, .4]
        omega = [.2, -.1, .6]
        msg = self.odom(10**9, rpy=[r, p, y], velocity=velocity, omega=omega)
        state = self.feedback.update(msg, 10**9)
        planar = Rotation.from_euler("xy", [r, p]).apply(velocity)
        yaw_rate = (math.sin(r) * omega[1] + math.cos(r) * omega[2]) / math.cos(p)
        np.testing.assert_allclose(state[2:6], [y, *planar[:2], yaw_rate], atol=1e-12)

    def test_rejects_unusable_inputs_without_advancing_state(self):
        self.feedback.update(self.odom(10**9), 10**9)
        bad_frame = self.odom(1_100_000_000)
        bad_frame.child_frame_id = "imu"
        for msg, now in [(self.odom(10**9), 10**9), (bad_frame, 1_100_000_000),
                         (self.odom(1_200_000_000), 1_800_000_000),
                         (self.odom(2_000_000_000), 1_800_000_000),
                         (self.odom(1_800_000_000, velocity=[np.nan, 0, 0]), 1_800_000_000),
                         (self.odom(1_800_000_000, velocity=[75, 0, 0]), 1_800_000_000),
                         (self.odom(1_800_000_000, rpy=[0., math.pi/2, 0.]), 1_800_000_000)]:
            with self.subTest(msg=msg), self.assertRaises(ValueError):
                self.feedback.update(msg, now)
            self.assertEqual(self.feedback.stamp, 10**9)
