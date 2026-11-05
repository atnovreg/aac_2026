from pathlib import Path
from system.f1tenth.workspace import find_workspace
import json
import os
import numpy as np
import yaml
from scipy.io import savemat
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


def export(out,source,config,evidence,trajectory,s,centre,yaw,curvature,left,right,ls,rs,metrics):
    link=out/'map.pcd'
    if link.exists() and not link.is_symlink():raise ValueError('Refusing to replace regular map.pcd; select a new output directory')
    if link.is_symlink():link.unlink()
    link.symlink_to(os.path.relpath(source/'map.pcd',out))
    for stale in out.glob('usable_area_boundary_*.csv'):stale.unlink()
    def csv(name,values,header):np.savetxt(out/name,values,delimiter=',',header=header,comments='',fmt='%.9f')
    q=trajectory[:,4:8]
    angle=np.arctan2(2*(q[:,3]*q[:,2]+q[:,0]*q[:,1]),1-2*(q[:,1]**2+q[:,2]**2))
    csv('trajectory.csv',np.c_[trajectory[:,:4],angle],'t,x,y,z,yaw')
    csv('centerline.csv',np.c_[s,centre,yaw,curvature],'s,x,y,yaw,curvature')
    csv('left_boundary.csv',np.c_[s,left,np.linalg.norm(left-centre,axis=1)],'s,x,y,half_width')
    csv('right_boundary.csv',np.c_[s,right,np.linalg.norm(right-centre,axis=1)],'s,x,y,half_width')
    csv('track.csv',np.c_[s,centre,yaw,curvature,np.linalg.norm(left-centre,axis=1),np.linalg.norm(right-centre,axis=1)],'s,x,y,yaw,curvature,width_left,width_right')
    savemat(out/'track_liniger.mat',{'track2':{'center':centre.T,'inner':left.T,'outer':right.T}})
    (out/'track_liniger.json').write_text(json.dumps({'X':centre[:,0].tolist(),'Y':centre[:,1].tolist(),'X_i':left[:,0].tolist(),'Y_i':left[:,1].tolist(),'X_o':right[:,0].tolist(),'Y_o':right[:,1].tolist()},indent=2)+'\n')
    from skimage.measure import find_contours
    contours=find_contours(evidence['usable'].astype(float),.5)
    for index,contour in enumerate(sorted(contours,key=len,reverse=True)):
        xy=evidence['origin']+(contour[:,::-1]+.5)*evidence['resolution']
        csv(f'usable_area_boundary_{index}.csv',xy,'x,y')
    np.savez_compressed(out/'map_evidence.npz',**{k:v for k,v in evidence.items() if isinstance(v,np.ndarray)})
    (out/'extraction.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    metrics['geometry_valid']=not any(metrics[k] for k in ['centre_self_crossings','left_self_crossings','right_self_crossings','centre_on_structural_obstacle','strip_samples_outside_usable','mutual_boundary_crossings']) and metrics['min_half_width_m']>=.15
    metrics['ready_for_vehicle']=False
    (out/'validation.json').write_text(json.dumps(metrics,indent=2)+'\n')
    bag=source.parent.name;workspace=find_workspace(__file__)
    (out/'bag_metadata.txt').write_text((workspace/'bag'/bag/'metadata.yaml').read_text()+'\n\nSOURCE MAP RUN: '+str(source)+'\nCalibration assumption: MID-360/internal IMU, upstream factory extrinsics.\n')
    # Conservative occupancy keeps lower returns occupied, even if dynamic.
    occupancy=np.full(evidence['usable'].shape,205,np.uint8)
    occupancy[evidence['observed_floor']]=254
    occupancy[(evidence['low_obstacles']>0)|evidence['obstacles']]=0
    with (out/'occupancy.pgm').open('wb') as stream:
        stream.write(f'P5\n{occupancy.shape[1]} {occupancy.shape[0]}\n255\n'.encode());stream.write(np.flipud(occupancy).tobytes())
    (out/'occupancy.yaml').write_text(yaml.safe_dump({'image':'occupancy.pgm','resolution':evidence['resolution'],'origin':[*evidence['origin'].tolist(),0.0],'negate':0,'occupied_thresh':.65,'free_thresh':.196,'mode':'trinary'}))
    origin=evidence['origin'];r=evidence['resolution'];shape=evidence['usable'].shape
    extent=[origin[0],origin[0]+shape[1]*r,origin[1],origin[1]+shape[0]*r]
    display=np.zeros(shape,np.uint8);display[evidence['usable']]=1;display[evidence['structural']>=config['min_wall_hits']]=2
    fig,ax=plt.subplots(figsize=(10,9),constrained_layout=True)
    ax.imshow(display,origin='lower',extent=extent,cmap=ListedColormap(['#d1d5db','#ffffff','#202938']),vmin=0,vmax=2)
    ax.plot(trajectory[:,1],trajectory[:,2],color='#94a3b8',lw=.65,label='Driven LIO trajectory (reference only)')
    for curve,color,label in [(left,'#16a34a','Conservative left MPCC limit'),(right,'#f97316','Conservative right MPCC limit'),(centre,'#2563eb','Map-derived centerline')]:
        closed=np.vstack([curve,curve[0]]);ax.plot(closed[:,0],closed[:,1],c=color,lw=1.7,label=label)
    ax.scatter(*centre[0],marker='*',s=90,c='#2563eb',zorder=6,label='Start (CCW)')
    ax.set(xlabel='x [m]',ylabel='y [m]',aspect='equal',title=f"Measured indoor corridor → MPCC track\n{metrics['length_m']:.2f} m loop • {len(s)} samples • structural geometry, review required")
    ax.legend(loc='upper right',fontsize=8);ax.grid(alpha=.15);fig.savefig(out/'track_overlay.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(13,6),constrained_layout=True)
    axes[0].imshow(occupancy,origin='upper',extent=[extent[0],extent[1],extent[3],extent[2]],cmap='gray',vmin=0,vmax=255)
    # Plot directly in lower-origin convention for alignment with LIO XY.
    axes[0].clear();axes[0].imshow(occupancy,origin='lower',extent=extent,cmap='gray',vmin=0,vmax=255)
    axes[0].set_title('Conservative occupancy: lower returns retained')
    low=(evidence['low_obstacles']>0)&evidence['usable']
    axes[1].imshow(display,origin='lower',extent=extent,cmap=ListedColormap(['#d1d5db','#fff','#202938']))
    cells=np.argwhere(low);xy=origin+(cells[:,::-1]+.5)*r;axes[1].scatter(xy[:,0],xy[:,1],s=.5,c='#dc2626');axes[1].set_title('Lower-height returns inside structural corridor')
    for ax in axes:ax.plot(centre[:,0],centre[:,1],c='#2563eb',lw=1);ax.set(xlabel='x [m]',ylabel='y [m]',aspect='equal')
    fig.savefig(out/'evidence_review.png',dpi=150);plt.close(fig)
