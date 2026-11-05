import unittest
import numpy as np
try:
    from measurements import road_pose,sensor_packet
except ImportError:
    road_pose=sensor_packet=None


class MeasurementsTests(unittest.TestCase):
    def setUp(self):self.assertIsNotNone(road_pose,'3D measurement model not implemented')

    def test_road_pose_retains_height_roll_and_pitch(self):
        pose=road_pose([2.,3.,.4,1.,0.,0.,0.],[.1,-.05,-1.])
        np.testing.assert_allclose(pose[:3,:3].T@pose[:3,:3],np.eye(3),atol=1e-12)
        np.testing.assert_allclose(pose[:3,2],np.array([-.1,.05,1])/np.linalg.norm([-.1,.05,1]))
        self.assertAlmostEqual(pose[2,3],-.8)
        self.assertAlmostEqual(np.arctan2(pose[1,0],pose[0,0]),.4)

    def test_stationary_imu_reports_body_gravity_without_pose_in_packet(self):
        pose=road_pose([2.,3.,.4,0.,0.,0.,0.],[.1,-.05,-1.])
        packet=sensor_packet(pose,pose,np.zeros(3),np.zeros(3),.02,np.random.default_rng(1),noise=False,wheel_scale=1.,gyro_bias=np.zeros(3))
        np.testing.assert_allclose(packet['gyro'],0.,atol=1e-12)
        np.testing.assert_allclose(packet['acceleration'],pose[:3,:3].T@np.array([0.,0.,9.80665]))
        self.assertEqual(set(packet),{'wheel','gyro','acceleration'})


if __name__=='__main__':unittest.main()
