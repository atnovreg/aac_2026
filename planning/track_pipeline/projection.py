"""Height-aware 2D evidence; unobserved space is never labelled observed free."""
from pathlib import Path
import numpy as np
from scipy import ndimage as nd
from scipy.spatial import cKDTree


def read_pcd(path):
    with Path(path).open('rb') as stream:
        header={}
        while True:
            line=stream.readline()
            if not line:raise ValueError('Incomplete PCD header')
            line=line.decode('ascii').strip()
            if not line or line.startswith('#'):continue
            key,*value=line.split();header[key]=value
            if key=='DATA':break
        offset=stream.tell()
    if header['DATA']!=['binary']:raise ValueError('Expected uncompressed binary PCD')
    formats={('F','4'):'<f4',('F','8'):'<f8',('U','1'):'u1',('U','4'):'<u4',('I','4'):'<i4'}
    dtype=np.dtype([(name,formats[(kind,size)],int(count)) if int(count)>1 else (name,formats[(kind,size)]) for name,kind,size,count in zip(header['FIELDS'],header['TYPE'],header['SIZE'],header['COUNT'])])
    count=int(header['POINTS'][0])
    if Path(path).stat().st_size-offset != count*dtype.itemsize:raise ValueError('PCD payload size mismatch')
    return np.memmap(path,mode='r',dtype=dtype,offset=offset,shape=(count,))


def xyz(points):return np.column_stack([points[k] for k in ['x','y','z']])


def floor_plane(points,trajectory):
    sample=xyz(points[::max(1,len(points)//600000)])
    distance,index=cKDTree(trajectory[:,1:3]).query(sample[:,:2])
    height=sample[:,2]-trajectory[index,3]
    candidates=sample[(distance<.8)&(height> -1.3)&(height<-.4)]
    if len(candidates)<100:raise ValueError('Insufficient floor evidence 0.4–1.3 m below sensor; check mounting height')
    matrix=np.c_[candidates[:,:2],np.ones(len(candidates))]
    coefficients=np.linalg.lstsq(matrix,candidates[:,2],rcond=None)[0]
    for _ in range(8):
        residual=candidates[:,2]-matrix@coefficients
        inliers=np.abs(residual-np.median(residual))<.10
        if inliers.sum()<100:raise ValueError('Floor fit has insufficient inliers')
        coefficients=np.linalg.lstsq(matrix[inliers],candidates[inliers,2],rcond=None)[0]
    error=np.abs(candidates[inliers,2]-matrix[inliers]@coefficients)
    if np.quantile(error,.95)>.1:raise ValueError('Floor plane residual too large')
    return coefficients,{'inliers':int(inliers.sum()),'p95_residual_m':float(np.quantile(error,.95))}


def fill_small_holes(mask,max_pixels):
    holes=nd.binary_fill_holes(mask)&~mask
    labels,_=nd.label(holes);sizes=np.bincount(labels.ravel())
    fill=(sizes<=max_pixels);fill[0]=False
    return mask|fill[labels]


def project(points,trajectory,config):
    from skimage.morphology import disk
    resolution=config['resolution_m'];origin=trajectory[:,1:3].min(0)-config['roi_padding_m']
    upper=trajectory[:,1:3].max(0)+config['roi_padding_m'];shape=np.ceil((upper-origin)/resolution).astype(int)[::-1]
    if np.prod(shape)>10000000:raise ValueError('Grid too large; crop route or increase resolution')
    plane,fit=floor_plane(points,trajectory)
    grids={k:np.zeros(shape,np.uint32) for k in ['floor','structural','low_obstacles']}
    sample=points
    for start in range(0,len(sample),300000):
        cloud=xyz(sample[start:start+300000]);height=cloud[:,2]-(cloud[:,:2]@plane[:2]+plane[2])
        ij=np.floor((cloud[:,:2]-origin)/resolution).astype(int)
        valid=np.isfinite(cloud).all(1)&(ij>=0).all(1)&(ij[:,0]<shape[1])&(ij[:,1]<shape[0])
        selections={'floor':np.abs(height)<config['floor_tolerance_m'],
          'structural':(height>config['wall_height_m'][0])&(height<config['wall_height_m'][1]),
          'low_obstacles':(height>.15)&(height<config['wall_height_m'][0])}
        stride_mask=(np.arange(start,start+len(cloud)) % config['point_stride'])==0
        for name,selection in selections.items():
            if name!='low_obstacles':selection=selection&stride_mask
            cells=ij[valid&selection];np.add.at(grids[name],(cells[:,1],cells[:,0]),1)
    structural=grids['structural']>=config['min_wall_hits']
    obstacles=nd.binary_dilation(structural,iterations=1)
    observed=grids['floor']>0
    usable=nd.binary_closing(nd.binary_dilation(observed,structure=disk(round(config['floor_dilation_m']/resolution))),structure=disk(round(config['floor_closing_m']/resolution)))&~obstacles
    usable=fill_small_holes(usable,round(config['fill_holes_m2']/resolution**2))&~obstacles
    labels,_=nd.label(usable)
    if not labels.max():raise ValueError('No usable connected component')
    cells=np.floor((trajectory[:,1:3]-origin)/resolution).astype(int)
    route_labels=labels[cells[:,1],cells[:,0]]
    counts=np.bincount(route_labels,minlength=int(labels.max())+1);counts[0]=0
    if counts.max()==0:raise ValueError('Recorded route does not intersect floor-supported space')
    usable=labels==np.argmax(counts)
    return dict(grids,obstacles=obstacles,observed_floor=observed,usable=usable,origin=origin,plane=plane,fit=fit,resolution=resolution)
