"""Conservative observed-surface candidate mask; unknown is never hole-filled."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy import ndimage as ndi
from scipy.spatial import cKDTree


def classify(floor_scans,obstacle_scans,static_obstacle,resolution,min_floor=3,min_obstacle=3,obstacle_source="static_map"):
    if obstacle_source=="static_map":
        obstacle=np.array(static_obstacle,dtype=bool,copy=True)
    elif obstacle_source=="raw_or_static":
        obstacle=(obstacle_scans>=min_obstacle)|static_obstacle
    else:
        raise ValueError("Unknown obstacle source: "+str(obstacle_source))
    candidate=(floor_scans>=min_floor)&~obstacle
    occupancy=np.full(candidate.shape,-1,dtype=np.int8);occupancy[candidate]=0;occupancy[obstacle]=100
    # Padding also makes out-of-ROI cells unavailable. Subtract half the cell diagonal so
    # distance is to the occupied/unknown cell boundary, not its center.
    clearance=np.maximum(0,ndi.distance_transform_edt(np.pad(candidate,1))[1:-1,1:-1]*resolution-resolution/np.sqrt(2))
    clearance[~candidate]=0
    return dict(candidate=candidate,obstacle=obstacle,occupancy=occupancy,clearance=clearance)


def fit_road(raw,poses):
    distance,index=cKDTree(poses[:,1:3]).query(raw[:,:2])
    relative=raw[:,2]-poses[index,3]
    points=raw[(distance<.6)&(relative>-.95)&(relative<-.45)]
    if len(points)<1000:raise ValueError('Insufficient road points: check mounting height and floor selection')
    rng=np.random.default_rng(1);p=points[rng.choice(len(points),min(len(points),20000),False)]
    m=np.c_[p[:,:2],np.ones(len(p))];best=0;plane=None
    for _ in range(200):
        ids=rng.choice(len(p),3,False)
        if abs(np.linalg.det(m[ids]))<1:continue
        c=np.linalg.solve(m[ids],p[ids,2])
        if np.linalg.norm(c[:2])>.2:continue
        score=(abs(m@c-p[:,2])<.07).sum()
        if score>best:best=score;plane=c
    if plane is None:raise ValueError('Road fit failed')
    for _ in range(5):
        good=abs(m@plane-p[:,2])<.1;plane=np.linalg.lstsq(m[good],p[good,2],rcond=None)[0]
    residual=abs(m@plane-p[:,2])
    if good.mean()<.8:raise ValueError('Single plane insufficient for road; require terrain model')
    return plane,{'candidate_points':len(points),'sample_points':len(p),'inlier_fraction':float(good.mean()),'all_candidate_abs_residual_p95_m':float(np.percentile(residual,95)),'candidate_selection':'within 0.6m XY of estimated path, 0.45–0.95m below IMU; review-specific assumption'}


def main(config_path):
    import yaml
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores,get_typestore
    from scipy.spatial.transform import Rotation,Slerp
    from sensors.bag_adapter.pointcloud_layout import OUTPUT_DTYPE
    from planning.track_pipeline.projection import read_pcd
    cfg=yaml.safe_load(config_path.read_text());run=Path(cfg['source_run']);out=Path(cfg['output'])
    if out.exists():raise FileExistsError('Use a new output directory: '+str(out))
    cache=np.load(cfg['review_cache']);poses=cache['poses'];plane,fit=fit_road(cache['input_map'],poses)
    res=cfg['resolution_m'];origin=np.floor((poses[:,1:3].min(0)-cfg['roi_padding_m'])/res)*res
    upper=np.ceil((poses[:,1:3].max(0)+cfg['roi_padding_m'])/res)*res;wh=np.rint((upper-origin)/res).astype(int);shape=tuple(wh[::-1])
    floor=np.zeros(shape,np.uint32);obs=np.zeros(shape,np.uint32);static=np.zeros(shape,bool)
    def ids(p):
        xy=np.floor((p[:,:2]-origin)/res).astype(int)
        valid=(xy>=0).all(1)&(xy<wh).all(1)&np.isfinite(p).all(1)
        return xy,valid
    def height(p):return p[:,2]-p[:,:2]@plane[:2]-plane[2]
    # Obstacle source is configurable; raw scan obstacle counts are diagnostic in static_map mode.
    pc=read_pcd(run/'map_static.pcd');sp=np.column_stack([pc[k] for k in 'xyz']);xy,v=ids(sp);h=height(sp)
    sel=v&(h>=cfg['obstacle_min_height_m'])&(h<=cfg['obstacle_max_height_m']);static[xy[sel,1],xy[sel,0]]=True
    times=poses[:,0]-poses[0,0];rot=Slerp(times,Rotation.from_quat(poses[:,4:8]));store=get_typestore(Stores.ROS1_NOETIC)
    from system.f1tenth.workspace import find_workspace, resolve_data_path
    bag=resolve_data_path(find_workspace(__file__), json.loads((run/'manifest.json').read_text())['bag']);scans=0
    with Reader(bag) as reader:
        cs=[c for c in reader.connections if c.msgtype.endswith('PointCloud2')]
        for conn,_,data in reader.messages(connections=cs):
            msg=store.deserialize_ros1(data,conn.msgtype);p=np.frombuffer(msg.data,dtype=OUTPUT_DTYPE)
            t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9-poses[0,0]+p['time'].astype(float)
            keep=(t>=0)&(t<=times[-1]);t=t[keep];p=p[keep]
            if not len(t):continue
            local=np.column_stack([p[k] for k in 'xyz']).astype(float)+[-.011,-.02329,.04412]
            world=rot(t).apply(local)+np.column_stack([np.interp(t,times,poses[:,i]) for i in (1,2,3)])
            xy,v=ids(world);h=height(world)
            for grid,sel in [(floor,abs(h)<=cfg['floor_tolerance_m']),(obs,(h>=cfg['obstacle_min_height_m'])&(h<=cfg['obstacle_max_height_m']))]:
                flat=np.unique(xy[v&sel,1]*wh[0]+xy[v&sel,0]);grid.ravel()[flat]+=1
            scans+=1
    evidence=classify(floor,obs,static,res,cfg['min_floor_scans'],cfg['min_obstacle_scans'],cfg.get('obstacle_source','static_map'))
    candidate=evidence['candidate'];labels,n=ndi.label(candidate,structure=ndi.generate_binary_structure(2,1))
    ij=np.floor((poses[:,1:3]-origin)/res).astype(int);pathlabels=labels[ij[:,1],ij[:,0]]
    hits=np.bincount(pathlabels,minlength=n+1);hits[0]=0;mainlabel=int(hits.argmax());mainmask=(labels==mainlabel) if hits.max() else np.zeros(shape,bool)
    # A pose on a missing cell is still missing: never force the path into the mask.
    state=evidence['occupancy'][ij[:,1],ij[:,0]];pathclear=evidence['clearance'][ij[:,1],ij[:,0]]
    metrics={'obstacle_source':cfg.get('obstacle_source','static_map'),'raw_obstacle_counts_used_for_classification':cfg.get('obstacle_source','static_map')=='raw_or_static','source_run':str(run),'frame':'review_map','resolution_m':res,'origin_xy':origin.tolist(),'shape_yx':[int(v) for v in shape],'input_scans':scans,'road_plane_z_ax_by_c':plane.tolist(),'road_fit':fit,'road_plane_offset_from_review_plane_m':float(plane[2]-cache['plane'][2]),'area_m2':{'observed_surface_candidate':float(candidate.sum()*res**2),'obstacle_evidence':float(evidence['obstacle'].sum()*res**2),'unknown':float((evidence['occupancy']==-1).sum()*res**2),'main_path_component':float(mainmask.sum()*res**2)},'components_4_connected':n,'path_pose_count':len(poses),'path_pose_candidate_fraction':float((state==0).mean()),'path_pose_obstacle_fraction':float((state==100).mean()),'path_pose_unknown_fraction':float((state==-1).mean()),'path_pose_main_component_fraction':float(mainmask[ij[:,1],ij[:,0]].mean()),'path_clearance_p5_median_p95_m':np.percentile(pathclear,[5,50,95]).tolist(),'vehicle_footprint_applied':False,'ready_for_vehicle':False,'free_volume_raycast_verified':False,'scope':'Observed near-planar surface candidate, not certified drivable/free volume. Unknown holes preserved. No centerline generated.'}
    assert not (candidate&evidence['obstacle']).any()
    assert not (candidate&(floor<cfg['min_floor_scans'])).any()
    assert np.all(evidence['clearance'][~candidate]==0)
    out.mkdir(parents=True)
    np.savez_compressed(out/'area.npz',**evidence,floor_scans=floor,obstacle_scans=obs,static_obstacle=static,main_component=mainmask,origin=origin,plane=plane,resolution=res,poses=poses)
    (out/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n');(out/'config.yaml').write_text(config_path.read_text())
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [config_path,run/'trajectory.tum',run/'map_static.pcd',run/'manifest.json']}
    (out/'provenance.json').write_text(json.dumps({'source_sha256':hashes,'input_bag':str(bag),'input_bag_size':bag.stat().st_size},indent=2)+'\n')
    plot(out,evidence,mainmask,origin,res,poses,metrics)
    print(json.dumps(metrics,ensure_ascii=False),flush=True)


def plot(out,e,mainmask,origin,res,poses,metrics):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    shape=e['candidate'].shape;extent=[origin[0],origin[0]+shape[1]*res,origin[1],origin[1]+shape[0]*res]
    code=np.zeros(shape,np.uint8);code[e['candidate']]=1;code[e['obstacle']]=2
    fig,axs=plt.subplots(1,3,figsize=(20,7),layout='constrained')
    axs[0].imshow(code,origin='lower',extent=extent,cmap=ListedColormap(['#59616c','#69c787','#e37c54']),vmin=0,vmax=2,interpolation='nearest');axs[0].set_title('Gray: unknown / Green: surface candidate / Orange: obstacle')
    axs[1].imshow(mainmask,origin='lower',extent=extent,cmap=ListedColormap(['#59616c','#69c787']),vmin=0,vmax=1,interpolation='nearest');axs[1].set_title('Connected component with most recorded path poses')
    im=axs[2].imshow(np.ma.masked_where(~e['candidate'],e['clearance']),origin='lower',extent=extent,cmap='viridis',vmin=0,vmax=1.5,interpolation='nearest');axs[2].set_facecolor('#59616c');axs[2].set_title('Distance to obstacle OR unknown boundary (m)');fig.colorbar(im,ax=axs[2],shrink=.65)
    for ax in axs:
        ax.plot(poses[:,1],poses[:,2],c='#263faa',lw=.8);ax.scatter(*poses[0,1:3],marker='*',c='yellow',s=65);ax.set(xlabel='x (m)',ylabel='y (m)',aspect='equal')
    fig.suptitle(f'Geonji Plaza 083146 | {res:.2f} m grid | obstacles: {metrics.get("obstacle_source", "raw_or_static")} | no hole filling\nSurface evidence only: vehicle footprint and free-volume ray checks NOT applied',fontsize=14)
    fig.savefig(out/'area_review.png',dpi=150);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=Path('src/aac_2026/planning/geonji/config/geonji_drivable_area_static.yaml'));a=p.parse_args();main(a.config)
