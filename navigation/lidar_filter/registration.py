"""Register raw returns using their own timestamps and map-from-IMU TUM poses."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


class PoseTrajectory:
    def __init__(self, poses):
        poses=np.asarray(poses,dtype=float)
        if poses.ndim!=2 or poses.shape[1]!=8 or len(poses)<2 or not np.isfinite(poses).all() or np.any(np.diff(poses[:,0])<=0):
            raise ValueError('Expected finite, time-ordered TUM poses')
        self.poses=poses
        self.rotation=Slerp(poses[:,0],Rotation.from_quat(poses[:,4:8]))

    def project(self, lidar_xyz, timestamps, lidar_to_imu_translation):
        xyz=np.asarray(lidar_xyz);times=np.asarray(timestamps,dtype=float)
        valid=np.isfinite(times)&np.isfinite(xyz).all(axis=1)&(times>=self.poses[0,0])&(times<=self.poses[-1,0])
        world=np.full(xyz.shape,np.nan,dtype=float);origins=world.copy()
        if np.any(valid):
            ts=times[valid];rot=self.rotation(ts)
            translation=np.column_stack([np.interp(ts,self.poses[:,0],self.poses[:,i]) for i in (1,2,3)])
            offset=rot.apply(np.broadcast_to(lidar_to_imu_translation,(len(ts),3)).copy())
            origins[valid]=translation+offset
            world[valid]=rot.apply(xyz[valid])+origins[valid]
        return world,origins,valid
