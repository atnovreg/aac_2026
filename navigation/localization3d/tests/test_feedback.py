import unittest
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from control.acados_mpcc.track import Track
try:
    from feedback import estimated_feedback
except ImportError:
    estimated_feedback=None


class FeedbackTests(unittest.TestCase):
    def setUp(self):self.assertIsNotNone(estimated_feedback,'Estimated-state feedback not implemented')

    def estimate(self):
        angle=np.arange(40)*2*np.pi/40
        track=Track(np.arange(40.),np.c_[5*np.cos(angle),5*np.sin(angle)],np.ones(40),np.ones(40),40.)
        pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('z',-3.12).as_matrix();pose[:2,3]=[-5.,.05]
        estimate=SimpleNamespace(pose=pose,velocity_map=np.array([-1.,0.,0]),gyro=np.array([0.,0.,.1]),last_accepted_time=1.)
        return track,estimate

    def test_yaw_wrap_is_continuous_and_state_uses_estimated_pose(self):
        track,estimate=self.estimate()
        state=estimated_feedback(estimate,track,20.,3.13,1.1)
        np.testing.assert_allclose(state[:2],estimate.pose[:2,3])
        self.assertGreater(state[2],np.pi)
        self.assertLess(abs(state[2]-3.13),.1)

    def test_stale_localization_stops_instead_of_returning_a_state(self):
        track,estimate=self.estimate()
        with self.assertRaisesRegex(RuntimeError,'stale'):
            estimated_feedback(estimate,track,20.,3.13,1.6)


if __name__=='__main__':unittest.main()
