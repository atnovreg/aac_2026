"""First-return 3D voxel ray sensor; geometry is independent of the raceline."""
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree


class VoxelWorld:
    def __init__(self,occupied,origin,voxel):
        self.occupied=np.asarray(occupied,dtype=bool)
        self.origin=np.asarray(origin,dtype=float)
        self.voxel=float(voxel)
        if self.occupied.ndim!=3 or self.origin.shape!=(3,) or self.voxel<=0:
            raise ValueError('3D occupancy, XYZ origin and positive voxel required')
        self.shape=np.array(self.occupied.shape)
        self.centres=None
        self.normals=None
        self.tree=None

    @classmethod
    def from_points(cls,points,voxel=.15):
        points=np.asarray(points)
        points=points[np.isfinite(points).all(axis=1)]
        origin=np.floor(points.min(axis=0)/voxel)*voxel-voxel
        indices=np.floor((points-origin)/voxel).astype(int)
        shape=indices.max(axis=0)+2
        if np.prod(shape)>250_000_000: raise ValueError('Voxel geometry exceeds memory cap')
        occupied=np.zeros(shape,bool)
        occupied[tuple(indices.T)]=True
        return cls(occupied,origin,voxel)

    def save(self,path):
        np.savez_compressed(path,occupied=self.occupied,origin=self.origin,voxel=self.voxel)

    @classmethod
    def load(cls,path):
        with np.load(path) as d:return cls(d['occupied'],d['origin'],float(d['voxel']))

    def surfaces(self):
        if self.tree is not None:return self.centres,self.normals
        indices=np.argwhere(self.occupied)
        centres=[];normals=[]
        for axis in range(3):
            for sign in [-1,1]:
                neighbour=indices.copy();neighbour[:,axis]+=sign
                inside=((neighbour>=0)&(neighbour<self.shape)).all(axis=1)
                empty=np.ones(len(indices),bool)
                empty[inside]=~self.occupied[tuple(neighbour[inside].T)]
                xyz=self.origin+(indices[empty]+.5)*self.voxel
                xyz[:,axis]+=sign*self.voxel/2
                normal=np.zeros_like(xyz);normal[:,axis]=sign
                centres.append(xyz);normals.append(normal)
        self.centres=np.vstack(centres);self.normals=np.vstack(normals)
        if not len(self.centres):raise ValueError('Empty sensor/map geometry')
        self.tree=cKDTree(self.centres)
        return self.centres,self.normals

    def nearest_faces(self,points):
        self.surfaces()
        distance,index=self.tree.query(points,workers=1)
        return self.centres[index],self.normals[index],distance

    def scan(self,pose,directions,min_range=.5,max_range=30.,noise_std=0.,dropout=0.,rng=None):
        pose=np.asarray(pose);directions=np.asarray(directions,dtype=float)
        if pose.shape!=(4,4) or directions.ndim!=2 or directions.shape[1]!=3:
            raise ValueError('SE3 pose and Nx3 directions required')
        directions=directions/np.linalg.norm(directions,axis=1)[:,None]
        rays=directions@pose[:3,:3].T
        start=pose[:3,3]
        count=len(rays)
        indices=np.tile(np.floor((start-self.origin)/self.voxel).astype(int),(count,1))
        step=np.sign(rays).astype(int)
        boundary=self.origin+(indices+(step>0))*self.voxel
        tmax=np.full_like(rays,np.inf);delta=np.full_like(rays,np.inf)
        np.divide(boundary-start,rays,out=tmax,where=np.abs(rays)>1e-12)
        np.divide(self.voxel,np.abs(rays),out=delta,where=np.abs(rays)>1e-12)
        current=np.zeros(count)
        ranges=np.full(count,np.inf)
        active=np.ones(count,bool)
        for _ in range(int(np.ceil(max_range/self.voxel*np.sqrt(3)))+5):
            ids=np.flatnonzero(active)
            if not len(ids):break
            inside=((indices[ids]>=0)&(indices[ids]<self.shape)).all(axis=1)&(current[ids]<=max_range)
            active[ids[~inside]]=False;ids=ids[inside]
            if not len(ids):continue
            hit=self.occupied[tuple(indices[ids].T)]
            hitids=ids[hit]
            ranges[hitids]=current[hitids]
            active[hitids]=False
            ids=ids[~hit]
            if not len(ids):continue
            axis=np.argmin(tmax[ids],axis=1)
            current[ids]=tmax[ids,axis]
            indices[ids,axis]+=step[ids,axis]
            tmax[ids,axis]+=delta[ids,axis]
        valid=np.isfinite(ranges)&(ranges>=min_range)&(ranges<=max_range)
        if rng is None:rng=np.random.default_rng(0)
        if dropout:valid&=rng.random(count)>=dropout
        if noise_std:ranges[valid]+=rng.normal(0,noise_std,valid.sum())
        valid&=(ranges>=min_range)&(ranges<=max_range)
        return dict(points=directions[valid]*ranges[valid,None],ranges=ranges,valid=valid)


def ray_grid(horizontal=240,vertical=16):
    azimuth=np.linspace(-np.pi,np.pi,horizontal,endpoint=False)
    elevation=np.deg2rad(np.linspace(-7,52,vertical))
    az,el=np.meshgrid(azimuth,elevation,indexing='xy')
    return np.c_[np.cos(el).ravel()*np.cos(az).ravel(),
                 np.cos(el).ravel()*np.sin(az).ravel(),np.sin(el).ravel()]
