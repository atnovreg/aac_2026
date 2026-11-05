import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from voxel_sensor import VoxelWorld
try:
    from estimator import register,StateEstimator,DeadReckoner,mpcc_state
except ImportError:
    register=StateEstimator=DeadReckoner=mpcc_state=None


class EstimatorTests(unittest.TestCase):
    def setUp(self):self.assertIsNotNone(register,'3D estimator not implemented')

    @staticmethod
    def scene():
        occ=np.zeros((40,40,30),bool)
        occ[2,:,:]=True;occ[37,:,:]=True;occ[:,2,:]=True;occ[:,37,:]=True
        occ[:,:,2]=True;occ[:,:,27]=True
        occ[25:29,23:27,3:18]=True
        world=VoxelWorld(occ,[-4,-4,-1],.2)
        rng=np.random.default_rng(5);directions=rng.normal(size=(4000,3))
        directions/=np.linalg.norm(directions,axis=1)[:,None]
        true=np.eye(4);true[:3,:3]=Rotation.from_euler('xyz',[.03,-.04,.25]).as_matrix();true[:3,3]=[.3,-.2,.2]
        cloud=world.scan(true,directions,min_range=.1,max_range=9.)['points']
        prior=true.copy();prior[:3,3]+=[.1,-.1,.08]
        prior[:3,:3]=Rotation.from_rotvec([.02,-.01,.025]).as_matrix()@prior[:3,:3]
        return world,cloud,true,prior

    def test_nonplanar_six_dof_transform_recovered(self):
        world,cloud,true,prior=self.scene()
        result=register(world,cloud,prior)
        self.assertTrue(result.accepted,result.reason)
        self.assertLess(np.linalg.norm(result.pose[:3,3]-true[:3,3]),.015)
        self.assertLess(Rotation.from_matrix(result.pose[:3,:3]@true[:3,:3].T).magnitude(),.01)

    def test_no_return_scan_rejected(self):
        world,_,_,prior=self.scene()
        result=register(world,np.empty((0,3)),prior)
        self.assertFalse(result.accepted)
        np.testing.assert_array_equal(result.pose,prior)

    def test_wheel_measurement_spike_is_filtered_before_controller_velocity(self):
        world,_,_,prior=self.scene()
        estimate=StateEstimator(world,prior,np.eye(4));estimate.wheel=1.
        estimate.predict(1.03,np.zeros(3),.02)
        self.assertGreater(estimate.wheel,1.)
        self.assertLess(estimate.wheel,1.015)

    def test_flat_plane_does_not_claim_observable_six_dof_localization(self):
        class Plane:
            def nearest_faces(self,points):
                centres=points.copy();centres[:,2]=0.
                normals=np.zeros_like(points);normals[:,2]=1.
                return centres,normals,np.abs(points[:,2])
        rng=np.random.default_rng(2)
        cloud=np.c_[rng.uniform(-2,2,(300,2)),np.zeros(300)]
        prior=np.eye(4);prior[2,3]=.1
        result=register(Plane(),cloud,prior)
        self.assertFalse(result.accepted)
        self.assertIn('degenerate',result.reason)

    def test_raw_odometry_not_overwritten_by_scan_correction(self):
        world,cloud,true,prior=self.scene()
        raw=DeadReckoner(prior)
        estimate=StateEstimator(world,prior,np.eye(4))
        before=raw.pose.copy()
        self.assertTrue(estimate.correct(cloud,0.).accepted)
        np.testing.assert_array_equal(raw.pose,before)
        self.assertGreater(np.linalg.norm(raw.pose[:3,3]-estimate.pose[:3,3]),.1)

    def test_sloped_body_converted_to_planar_controller_velocity_and_yaw_rate(self):
        pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('xyz',[.2,-.1,.4]).as_matrix();pose[:3,3]=[3,4,1]
        yaw=np.arctan2(pose[1,0],pose[0,0])
        velocity=np.array([2*np.cos(yaw)-.3*np.sin(yaw),2*np.sin(yaw)+.3*np.cos(yaw),.7])
        gyro=pose[:3,:3].T@np.array([0.,0.,.7])
        state=mpcc_state(pose,velocity,gyro,5.)
        np.testing.assert_allclose(state,[3,4,yaw,2,.3,.7,5.],atol=1e-12)


if __name__=='__main__':unittest.main()
