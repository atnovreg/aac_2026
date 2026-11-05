import unittest
import numpy as np
from navigation.lidar_filter.registration import PoseTrajectory


class RegistrationTests(unittest.TestCase):
    def test_translation_rotation_and_lidar_offset_at_each_point_time(self):
        poses=np.array([[100,0,0,0,0,0,0,1], [102,2,0,0,0,0,1,0]],float)
        trajectory=PoseTrajectory(poses)
        world,origin,valid=trajectory.project(np.array([[1,0,0],[1,0,0],[1,0,0]],float),np.array([100,101,102.]),np.array([.5,0,0]))
        np.testing.assert_allclose(world,[[1.5,0,0],[1,1.5,0],[.5,0,0]],atol=1e-12)
        np.testing.assert_allclose(origin,[[.5,0,0],[1,.5,0],[1.5,0,0]],atol=1e-12)
        self.assertTrue(valid.all())

    def test_does_not_extrapolate_outside_recorded_poses(self):
        trajectory=PoseTrajectory(np.array([[10,0,0,0,0,0,0,1],[11,0,0,0,0,0,0,1]],float))
        world,_,valid=trajectory.project(np.ones((3,3)),np.array([9,10.5,12]),np.zeros(3))
        np.testing.assert_array_equal(valid,[False,True,False])
        self.assertTrue(np.isnan(world[[0,2]]).all())

if __name__=='__main__':unittest.main()
