"""Geometry shared by the offline localization audit."""
from matplotlib.path import Path as MplPath
from matplotlib.patches import PathPatch
import numpy as np
from scipy.spatial import cKDTree
from shapely.geometry.polygon import orient


def paint(ax,geometry,color):
    parts=list(geometry.geoms) if geometry.geom_type=='MultiPolygon' else [geometry]
    for p in parts:
        if p.is_empty: continue
        p=orient(p,sign=1)
        vertices=[];codes=[]
        for r in [p.exterior,*p.interiors]:
            xy=np.asarray(r.coords);vertices.extend(xy)
            codes.extend([MplPath.MOVETO]+[MplPath.LINETO]*(len(xy)-2)+[MplPath.CLOSEPOLY])
        ax.add_patch(PathPatch(MplPath(vertices,codes),facecolor=color,edgecolor='none'))


class GridDistance:
    def __init__(self,mask,origin,res):
        self.origin,self.res=np.asarray(origin),float(res)
        self.upper=self.origin+np.array(mask.shape[::-1])*self.res
        self.centres=self.origin+(np.argwhere(~mask)[:,::-1]+.5)*self.res
        self.tree=cKDTree(self.centres)

    def __call__(self,points):
        nearest,_=self.tree.query(points)
        groups=self.tree.query_ball_point(points,nearest+self.res/np.sqrt(2)+1e-10)
        result=[]
        for p,indices in zip(points,groups):
            delta=np.maximum(np.abs(self.centres[indices]-p)-self.res/2,0)
            result.append(max(0.,min(np.linalg.norm(delta,axis=1).min(),
                                     np.min(np.r_[p-self.origin,self.upper-p]))))
        return np.asarray(result)


def analytic_check():
    mask=np.ones((5,5),bool);mask[2,2]=False
    dist=GridDistance(mask,[0,0],1.)
    np.testing.assert_allclose(dist(np.array([[2.5,2.5],[1.5,2.5],[1.,1.],[-1,0]])),[0,.5,1,0])


