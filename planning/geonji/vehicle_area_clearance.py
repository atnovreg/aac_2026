"""Orientation-independent enclosing-circle footprint on a candidate grid."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy import ndimage as ndi

def fit_centers(candidate,clearance,side_a,side_b,margin):
    if min(side_a,side_b)<=0 or margin<0:raise ValueError('Invalid footprint dimensions')
    radius=float(np.hypot(side_a,side_b)/2+margin)
    return candidate&(clearance>=radius),radius

def main(folder,a,b,margin):
    d=np.load(folder/'area.npz');centers,radius=fit_centers(d['candidate'],d['clearance'],a,b,margin)
    labels,count=ndi.label(centers);sizes=np.bincount(labels.ravel());sizes[0]=0
    largest=(labels==int(sizes.argmax())) if sizes.max() else np.zeros(centers.shape,bool)
    poses=d['poses'];res=float(d['resolution']);ij=np.floor((poses[:,1:3]-d['origin'])/res).astype(int)
    m={'rectangle_side_lengths_m':[a,b],'dimension_order_needed_for_circle':False,'extra_margin_m':margin,'extra_margin_basis':'provisional visualization margin, not measured calibration','enclosing_circle_radius_m':radius,'method':'orientation-independent enclosing circle; conservative against occupied and unknown cells','vehicle_centers_area_m2':float(centers.sum()*res**2),'vehicle_center_components':int(count),'largest_component_area_m2':float(largest.sum()*res**2),'recorded_imu_xy_in_vehicle_center_mask_fraction':float(centers[ij[:,1],ij[:,0]].mean()),'ready_for_vehicle':False,'limitations':['Vehicle-center to IMU extrinsic is unknown; path comparison is approximate.','Only a 2D footprint is applied, not vertical clearance, grade, traction or control feasibility.','Cell interior is represented at grid resolution; single observed ground returns do not prove free volume.']}
    np.savez_compressed(folder/'vehicle_clearance.npz',vehicle_centers=centers,largest_component=largest,radius=radius)
    (folder/'vehicle_clearance.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    code=np.zeros(centers.shape,np.uint8);code[d['candidate']]=1;code[d['obstacle']]=2;code[centers]=3
    shape=code.shape;origin=d['origin'];extent=[origin[0],origin[0]+shape[1]*res,origin[1],origin[1]+shape[0]*res]
    fig,ax=plt.subplots(figsize=(12,9),layout='constrained');ax.imshow(code,origin='lower',extent=extent,cmap=ListedColormap(['#59616c','#b6d4c2','#e37c54','#238447']),vmin=0,vmax=3,interpolation='nearest');ax.plot(poses[:,1],poses[:,2],c='#244ec8',lw=.8);ax.set(xlabel='x (m)',ylabel='y (m)',aspect='equal',title=f'Geonji Plaza | rectangle {a*100:.0f} x {b*100:.0f} cm + {margin*100:.0f} cm provisional margin\nDark green: enclosing-circle center candidates / Light green: observed surface\nOrange: obstacle evidence / Gray: unknown / Blue: recorded IMU trajectory');fig.savefig(folder/'vehicle_area_review.png',dpi=150);plt.close(fig)
    print(json.dumps(m),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--sides',type=float,nargs=2,required=True);p.add_argument('--margin',type=float,default=.05);a=p.parse_args();main(a.folder,*a.sides,a.margin)
