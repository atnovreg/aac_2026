"""Plant-side 3D sensor generation. Outputs contain no ground-truth pose."""
import numpy as np
from scipy.spatial.transform import Rotation


def road_pose(state,plane,height=.15):
    x,y,yaw=state[:3]
    a,b,c=plane
    normal=np.array([-a,-b,1.]);normal/=np.linalg.norm(normal)
    forward=np.array([np.cos(yaw),np.sin(yaw),a*np.cos(yaw)+b*np.sin(yaw)])
    forward/=np.linalg.norm(forward)
    left=np.cross(normal,forward)
    pose=np.eye(4);pose[:3,:3]=np.c_[forward,left,normal]
    pose[:3,3]=[x,y,a*x+b*y+c+height]
    return pose


def map_velocity(state,plane):
    yaw,vx,vy=state[2:5]
    xy=np.array([vx*np.cos(yaw)-vy*np.sin(yaw),vx*np.sin(yaw)+vy*np.cos(yaw)])
    return np.r_[xy,np.asarray(plane[:2])@xy]


def sensor_packet(previous_pose,pose,previous_velocity,velocity,dt,rng,
                  noise=True,wheel_scale=1.02,gyro_bias=None):
    if gyro_bias is None:gyro_bias=np.array([0.,0.,.001])
    gyro=Rotation.from_matrix(previous_pose[:3,:3].T@pose[:3,:3]).as_rotvec()/dt+gyro_bias
    acceleration=pose[:3,:3].T@((np.asarray(velocity)-previous_velocity)/dt+np.array([0.,0.,9.80665]))
    wheel=float(pose[:3,0]@velocity)*wheel_scale
    if noise:
        gyro+=rng.normal(0,.002,3)
        acceleration+=rng.normal(0,.02,3)
        wheel+=float(rng.normal(0,.01))
    return dict(wheel=wheel,gyro=gyro,acceleration=acceleration)
