"""Full XYZ scan-to-prior-map ICP with wheel/3-axis-gyro prediction.

No ground-truth pose enters these interfaces. This is a transparent baseline,
not FAST-LIO or tightly coupled accelerometer-based inertial navigation.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class Registration:
    pose:np.ndarray
    accepted:bool
    reason:str
    rmse:float=np.inf
    inlier_ratio:float=0.
    condition:float=np.inf
    iterations:int=0


def register(world,points_body,prior,max_iterations=30,max_distance=.6):
    points=np.asarray(points_body,dtype=float)
    original=np.asarray(prior,dtype=float).copy()
    if points.ndim!=2 or points.shape[1:]!=(3,) or len(points)<100 or not np.isfinite(points).all():
        return Registration(original,False,'insufficient finite 3D returns')
    # Deterministic uniform subsampling preserves the 3D scan, not an XY projection.
    if len(points)>2500:points=points[np.linspace(0,len(points)-1,2500,dtype=int)]
    pose=original.copy()
    for iteration in range(max_iterations):
        rotated=points@pose[:3,:3].T
        mapped=rotated+pose[:3,3]
        centres,normals,distance=world.nearest_faces(mapped)
        residual=np.einsum('ij,ij->i',normals,mapped-centres)
        keep=(distance<max_distance)&(np.abs(residual)<.4)
        ratio=float(keep.mean())
        if keep.sum()<100 or ratio<.45:
            return Registration(original,False,'low correspondence support',inlier_ratio=ratio,iterations=iteration+1)
        n=normals[keep];q=rotated[keep];r=residual[keep]
        jac=np.c_[n,np.cross(q,n)]
        weight=np.minimum(1.,.05/np.maximum(np.abs(r),1e-12))
        h=jac.T@(weight[:,None]*jac)
        eigen=np.linalg.eigvalsh(h)
        condition=float(eigen[-1]/max(eigen[0],1e-20))
        if eigen[0]<1e-5 or condition>1e9:
            return Registration(original,False,'degenerate 3D geometry',inlier_ratio=ratio,condition=condition,iterations=iteration+1)
        step=np.linalg.solve(h+np.eye(6)*1e-5,-jac.T@(weight*r))
        factor=max(1.,np.linalg.norm(step[:3])/.2,np.linalg.norm(step[3:])/.08)
        step/=factor
        pose[:3,3]+=step[:3]
        pose[:3,:3]=Rotation.from_rotvec(step[3:]).as_matrix()@pose[:3,:3]
        if np.linalg.norm(step[:3])<1e-4 and np.linalg.norm(step[3:])<1e-5:break
    mapped=points@pose[:3,:3].T+pose[:3,3]
    centres,normals,distance=world.nearest_faces(mapped)
    residual=np.einsum('ij,ij->i',normals,mapped-centres)
    keep=(distance<max_distance)&(np.abs(residual)<.15)
    ratio=float(keep.mean())
    rmse=float(np.sqrt(np.mean(residual[keep]**2))) if keep.any() else np.inf
    displacement=np.linalg.norm(pose[:3,3]-original[:3,3])
    angle=Rotation.from_matrix(pose[:3,:3]@original[:3,:3].T).magnitude()
    accepted=ratio>=.5 and rmse<.08 and displacement<.8 and angle<.3
    return Registration(pose if accepted else original,accepted,'accepted' if accepted else 'quality/correction gate',
                        rmse,ratio,condition,iteration+1)


def mpcc_state(pose,velocity_map,gyro_body,progress):
    """Project estimated motion into Slash's PLANAR yaw frame, not tilted body."""
    r=pose[:3,:3]
    yaw=np.arctan2(r[1,0],r[0,0])
    c,s=np.cos(yaw),np.sin(yaw)
    vx,vy=np.asarray(velocity_map)[:2]
    gyro=np.asarray(gyro_body)
    omega=np.array([[0,-gyro[2],gyro[1]],[gyro[2],0,-gyro[0]],[-gyro[1],gyro[0],0]])
    df=(r@omega)[:,0];f=r[:,0]
    yaw_rate=(f[0]*df[1]-f[1]*df[0])/(f[0]**2+f[1]**2)
    return np.array([pose[0,3],pose[1,3],yaw,c*vx+s*vy,-s*vx+c*vy,yaw_rate,progress])


class DeadReckoner:
    def __init__(self,initial_pose):
        self.pose=np.asarray(initial_pose,dtype=float).copy()
        self.velocity_map=np.zeros(3)
        self.gyro=np.zeros(3)
        self.wheel=0.
        self.lateral=0.

    def predict(self,wheel_m_s,gyro_rad_s,dt):
        gyro=np.asarray(gyro_rad_s,dtype=float)
        if gyro.shape!=(3,) or not np.isfinite(gyro).all() or not np.isfinite(wheel_m_s) or dt<=0:
            raise ValueError('Finite scalar wheel speed, 3-axis gyro and positive dt required')
        self.gyro=gyro.copy();self.wheel=float(wheel_m_s)
        half=Rotation.from_rotvec(gyro*dt/2).as_matrix()
        self.velocity_map=self.pose[:3,:3]@half@np.array([wheel_m_s,self.lateral,0.])
        self.pose[:3,3]+=self.velocity_map*dt
        self.pose[:3,:3]=self.pose[:3,:3]@Rotation.from_rotvec(gyro*dt).as_matrix()


class StateEstimator(DeadReckoner):
    def __init__(self,world,initial_pose,lidar_to_body):
        super().__init__(initial_pose)
        self.world=world
        self.lidar_to_body=np.asarray(lidar_to_body,dtype=float).copy()
        self.last_scan_pose=None
        self.last_scan_time=None
        self.last_accepted_time=None

    def predict(self,wheel_m_s,gyro_rad_s,dt):
        # Measurement noise must not become an instantaneous hard-state jump.
        # Fixed .08s low-pass; timing/lag is part of this baseline's limitations.
        alpha=1-np.exp(-dt/.08)
        filtered=self.wheel+alpha*(wheel_m_s-self.wheel)
        super().predict(filtered,gyro_rad_s,dt)

    def correct(self,scan_xyz,timestamp):
        if self.last_scan_time is not None and timestamp<=self.last_scan_time:
            raise ValueError('Scan timestamps must strictly increase')
        body=np.asarray(scan_xyz)@self.lidar_to_body[:3,:3].T+self.lidar_to_body[:3,3]
        result=register(self.world,body,self.pose)
        self.last_scan_time=float(timestamp)
        if result.accepted:
            self.pose=result.pose.copy()
            if self.last_scan_pose is not None:
                delta=timestamp-self.last_accepted_time
                scan_velocity=(self.pose[:3,3]-self.last_scan_pose[:3,3])/delta
                lateral=float(self.pose[:3,1]@scan_velocity)
                # Lateral velocity is inferred from two sensor-estimated poses;
                # low-pass time constant .4s, never copied from the plant.
                alpha=1-np.exp(-delta/.4)
                self.lateral=(1-alpha)*self.lateral+alpha*np.clip(lateral,-.5,.5)
            self.last_scan_pose=self.pose.copy();self.last_accepted_time=float(timestamp)
        return result

    def state(self,track,previous_progress):
        progress=track.project(self.pose[:2,3],previous_progress)
        return mpcc_state(self.pose,self.velocity_map,self.gyro,progress)
