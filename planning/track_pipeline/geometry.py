"""Map-derived loop geometry. CSV samples omit the duplicate endpoint."""
import numpy as np
import networkx as nx
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d
from scipy.spatial import cKDTree
from skimage.morphology import skeletonize


def area(xy):
    return .5*np.sum(xy[:,0]*np.roll(xy[:,1],-1)-xy[:,1]*np.roll(xy[:,0],-1))


def extract_loop(free, resolution, origin, route):
    nodes=set(map(tuple,np.argwhere(skeletonize(free))))
    graph=nx.Graph()
    for u in sorted(nodes):
        for di,dj in [(0,1),(1,-1),(1,0),(1,1)]:
            v=(u[0]+di,u[1]+dj)
            if v not in nodes: continue
            # Avoid tiny graph triangles at 8-connected raster corners.
            if di and dj and ((u[0]+di,u[1]) in nodes or (u[0],u[1]+dj) in nodes):continue
            graph.add_edge(u,v)
    candidates=[]
    for cycle in nx.cycle_basis(graph):
        xy=origin+(np.asarray(cycle)[:,::-1]+.5)*resolution
        if len(xy)<20 or abs(area(xy))<5:continue
        route_distance=cKDTree(xy).query(route)[0]
        candidates.append((np.quantile(route_distance,.8),-abs(area(xy)),xy))
    if not candidates:raise ValueError('No closed corridor loop in map evidence; do not close an open route artificially')
    xy=min(candidates,key=lambda c:c[:2])[2]
    if area(xy)<0:xy=xy[::-1]
    return np.roll(xy,-np.argmin(np.linalg.norm(xy-route[0],axis=1)),axis=0)


def resample_closed(xy, spacing, smoothing):
    xy=np.asarray(xy,float)
    if np.linalg.norm(xy[0]-xy[-1])<1e-9:xy=xy[:-1]
    keep=np.r_[True,np.linalg.norm(np.diff(xy,axis=0),axis=1)>1e-8];xy=xy[keep]
    if len(xy)<4 or spacing<=0:raise ValueError('Need a closed curve with >=4 distinct samples and positive spacing')
    arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(np.vstack([xy,xy[0]]),axis=0),axis=1))]
    u=np.linspace(0,arc[-1],max(100,int(np.ceil(arc[-1]/.02))),endpoint=False)
    spline=CubicSpline(arc,np.vstack([xy,xy[0]]),bc_type='periodic')
    dense=spline(u)
    if smoothing>0:dense=gaussian_filter1d(dense,smoothing/(arc[-1]/len(u)),axis=0,mode='wrap')
    arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(np.vstack([dense,dense[0]]),axis=0),axis=1))]
    spline=CubicSpline(arc,np.vstack([dense,dense[0]]),bc_type='periodic')
    length=float(arc[-1]);s=np.linspace(0,length,int(np.ceil(length/spacing)),endpoint=False)
    result=spline(s);d=spline(s,1);dd=spline(s,2)
    yaw=np.arctan2(d[:,1],d[:,0]);speed=np.linalg.norm(d,axis=1)
    if np.any(speed<1e-6):raise ValueError('Degenerate spline tangent')
    curvature=(d[:,0]*dd[:,1]-d[:,1]*dd[:,0])/speed**3
    return s,result,yaw,curvature,length


def sample_grid(grid,xy,resolution,origin,outside=False):
    ij=np.floor((xy-origin)/resolution).astype(int)
    valid=(ij[:,0]>=0)&(ij[:,1]>=0)&(ij[:,0]<grid.shape[1])&(ij[:,1]<grid.shape[0])
    values=np.full(len(xy),outside,dtype=grid.dtype)
    values[valid]=grid[ij[valid,1],ij[valid,0]]
    return values


def ray_boundaries(xy,yaw,obstacles,usable,resolution,origin,max_distance):
    normal=np.c_[-np.sin(yaw),np.cos(yaw)]
    distances=np.arange(resolution/2,max_distance+resolution/4,resolution/2)
    if not len(distances):raise ValueError('Ray distance smaller than grid step')
    results=[]
    for sign in [1,-1]:
        rays=xy[:,None,:]+sign*normal[:,None,:]*distances[None,:,None]
        hits=sample_grid(obstacles,rays.reshape(-1,2),resolution,origin).reshape(len(xy),-1)
        observed=sample_grid(usable,rays.reshape(-1,2),resolution,origin).reshape(len(xy),-1)
        found=hits.any(axis=1);first=np.argmax(hits,axis=1)
        # Without structural evidence, stop at the first unsupported floor cell.
        missing=~observed;fallback=np.where(missing.any(axis=1),np.argmax(missing,axis=1),len(distances)-1)
        index=np.where(found,first,np.maximum(fallback-1,0))
        results.extend([xy+sign*normal*distances[index,None],found.astype(np.uint8)])
    return results[0],results[2],results[1],results[3]


def self_intersections(xy):
    """Count proper nonadjacent crossings of a closed polygon."""
    a=np.asarray(xy);b=np.roll(a,-1,axis=0);count=0
    def cross(x,y):return x[...,0]*y[...,1]-x[...,1]*y[...,0]
    for i in range(len(a)):
        js=np.arange(i+2,len(a))
        if i==0:js=js[js!=len(a)-1]
        if not len(js):continue
        c,d=a[js],b[js]
        count+=int(np.sum((cross(b[i]-a[i],c-a[i])*cross(b[i]-a[i],d-a[i]) < -1e-12)&(cross(d-c,a[i]-c)*cross(d-c,b[i]-c)<-1e-12)))
    return count


def track_limits(centre,yaw,curvature,usable,resolution,origin,margin=.10,max_half_width=.75):
    """Inscribed MPCC corridor, distinct from physical/map-area boundaries.

    Distance-transform clearance prevents strips from entering unknown cells.
    Curvature radius cap prevents folds in normal coordinates at tight corners.
    """
    from scipy.ndimage import distance_transform_edt,minimum_filter1d
    clearance=sample_grid(distance_transform_edt(usable)*resolution,centre,resolution,origin)
    cap=np.minimum(np.minimum(clearance-margin,max_half_width),np.divide(.70,np.maximum(np.abs(curvature),1e-6)))
    if np.any(cap<=0):raise ValueError('Centreline has insufficient clearance')
    local=minimum_filter1d(cap,size=11,mode='wrap')
    width=np.minimum(gaussian_filter1d(local,2,mode='wrap'),cap)
    normal=np.c_[-np.sin(yaw),np.cos(yaw)]
    return centre+normal*width[:,None],centre-normal*width[:,None]


def mutual_crossings(first,second):
    a=np.asarray(first);b=np.roll(a,-1,axis=0);c=np.asarray(second);d=np.roll(c,-1,axis=0)
    def cross(x,y):return x[...,0]*y[...,1]-x[...,1]*y[...,0]
    return sum(int(np.sum((cross(bi-ai,c-ai)*cross(bi-ai,d-ai)<-1e-12)&(cross(d-c,ai-c)*cross(d-c,bi-c)<-1e-12))) for ai,bi in zip(a,b))


def validate_strip(centre,left,right,usable,resolution,origin):
    """Check interpolated complete cross-sections, not only CSV endpoints."""
    outside=0;checked=0
    for i in range(len(centre)):
        j=(i+1)%len(centre)
        steps=max(2,int(np.ceil(max(np.linalg.norm(left[j]-left[i]),np.linalg.norm(right[j]-right[i]))/(resolution/3)))+1)
        u=np.linspace(0,1,steps)
        l=left[i]+u[:,None]*(left[j]-left[i]);r=right[i]+u[:,None]*(right[j]-right[i])
        columns=max(2,int(np.ceil(np.max(np.linalg.norm(l-r,axis=1))/(resolution/3)))+1)
        v=np.linspace(0,1,columns)
        points=(l[:,None,:]+v[None,:,None]*(r-l)[:,None,:]).reshape(-1,2)
        outside+=int((~sample_grid(usable,points,resolution,origin)).sum());checked+=len(points)
    return {'strip_samples_checked':checked,'strip_samples_outside_usable':outside,'mutual_boundary_crossings':mutual_crossings(left,right)}
