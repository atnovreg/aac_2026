import unittest
import numpy as np
try:
    from voxel_sensor import VoxelWorld
except ImportError:
    VoxelWorld=None


class SensorTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(VoxelWorld,'3D voxel sensor not implemented')

    def test_exact_first_hit_occludes_farther_wall(self):
        occ=np.zeros((12,12,12),bool);occ[5,:,:]=True;occ[8,:,:]=True
        world=VoxelWorld(occ,np.zeros(3),1.)
        pose=np.eye(4);pose[:3,3]=[1.5,4.5,1.5]
        dirs=np.array([[1.,0,0],[1,0,1]])
        dirs/=np.linalg.norm(dirs,axis=1)[:,None]
        packet=world.scan(pose,dirs,min_range=.1,max_range=9.)
        np.testing.assert_allclose(packet['ranges'],[3.5,3.5*np.sqrt(2)],atol=1e-10)
        np.testing.assert_allclose(packet['points'][:,2],[0,3.5],atol=1e-10)

    def test_no_return_is_not_a_fabricated_max_range_hit(self):
        world=VoxelWorld(np.zeros((8,8,8),bool),np.zeros(3),1.)
        pose=np.eye(4);pose[:3,3]=[2.5,2.5,2.5]
        packet=world.scan(pose,np.array([[1.,0,0],[0,0,1.]]),max_range=4.)
        self.assertEqual(packet['points'].shape,(0,3))
        self.assertFalse(packet['valid'].any())

    def test_rotation_is_applied_in_all_three_dimensions(self):
        occ=np.zeros((10,10,10),bool);occ[:,:,7]=True
        world=VoxelWorld(occ,np.zeros(3),1.)
        pose=np.eye(4);pose[:3,3]=[3.5,3.5,2.5]
        pose[:3,:3]=np.array([[0,0,-1],[0,1,0],[1,0,0.]])
        packet=world.scan(pose,np.array([[1.,0,0]]),max_range=8.)
        np.testing.assert_allclose(packet['points'],[[4.5,0,0]],atol=1e-10)


if __name__=='__main__':unittest.main()
