"""Closed-lap topology check without filling unknown cells or relaxing the footprint."""
from pathlib import Path
import json,heapq,csv,hashlib
import numpy as np
from scipy import ndimage as ndi

BASE=Path('results/건지광장/drivable_area/083146_v3_static_only')
OUT=Path('results/건지광장/loop_check/083146_v1')

def encloses(mask,anchor_yx):
    y,x=anchor_yx
    if mask[y,x]:return False
    labels,_=ndi.label(~mask,structure=np.ones((3,3),int));label=labels[y,x]
    return bool(label and not(np.any(labels[0]==label) or np.any(labels[-1]==label) or np.any(labels[:,0]==label) or np.any(labels[:,-1]==label)))


def cut_loop(mask,clearance,anchor_yx,res):
    """Find a 4-neighbor closed path crossing an anchor-to-top cut once."""
    ay,ax=anchor_yx;h,w=mask.shape
    crossing=[y for y in range(ay+1,h) if mask[y,ax-1] and mask[y,ax]]
    crossing.sort(key=lambda y:min(clearance[y,ax-1],clearance[y,ax]),reverse=True)
    cost=1+(.35/(clearance+.03))**2
    tried=set()
    for y in crossing:
        if y//3 in tried:continue
        tried.add(y//3)
        source=y*w+ax-1;target=y*w+ax;distance=np.full(h*w,np.inf);prev=np.full(h*w,-1,int);distance[source]=0;heap=[(0.,source)]
        while heap:
            dist,u=heapq.heappop(heap)
            if dist!=distance[u]:continue
            if u==target:break
            uy,ux=divmod(u,w)
            for vy,vx in [(uy-1,ux),(uy+1,ux),(uy,ux-1),(uy,ux+1)]:
                if not(0<=vy<h and 0<=vx<w) or not mask[vy,vx]:continue
                if uy==vy and uy>=ay and min(ux,vx)==ax-1 and max(ux,vx)==ax:continue
                v=vy*w+vx;nd=dist+res*(cost[uy,ux]+cost[vy,vx])/2
                if nd<distance[v]:distance[v]=nd;prev[v]=u;heapq.heappush(heap,(nd,v))
        if not np.isfinite(distance[target]):continue
        chain=[target]
        while chain[-1]!=source:chain.append(int(prev[chain[-1]]))
        chain.reverse();chain.append(source);a=np.array([(v//w,v%w) for v in chain])
        assert np.all(np.abs(np.diff(a,axis=0)).sum(1)==1)
        assert mask[a[:,0],a[:,1]].all()
        return a
    raise ValueError('No valid 4-connected reference loop found across the radial cut')


def main():
    d=np.load(BASE/'area.npz');v=np.load(BASE/'vehicle_clearance.npz');res=float(d['resolution']);origin=d['origin'];clear=d['clearance'];surface=d['candidate'];vehicle=v['vehicle_centers'];anchor=np.array([0.,18.]);ij=np.floor((anchor-origin)/res).astype(int);ayx=(int(ij[1]),int(ij[0]))
    if d['candidate'][ayx]:raise ValueError('Reference anchor must remain inside the central excluded region')
    surface_loop=encloses(surface,ayx);vehicle_loop=encloses(vehicle,ayx);radius=float(v['radius']);body_radius=np.hypot(.56,.19)/2
    levels=np.unique(clear[surface]);low=0;high=len(levels)-1
    while low<high:
        mid=(low+high+1)//2
        if encloses(surface&(clear>=levels[mid]),ayx):low=mid
        else:high=mid-1
    threshold=float(levels[low]);next_threshold=float(levels[low+1]) if low+1<len(levels) else None
    OUT.mkdir(parents=True,exist_ok=False)
    result={'source_area':str(BASE),'anchor_xy_m':anchor.tolist(),'meaning':'One closed loop enclosing central plaza structure, not any small local cycle','surface_loop_exists_4_connected':surface_loop,'vehicle_center_loop_exists_4_connected':vehicle_loop,'vehicle_circle_radius_with_margin_m':radius,'vehicle_circle_radius_without_margin_m':float(body_radius),'vehicle_loop_without_5cm_margin':encloses(surface&(clear>=body_radius),ayx),'max_conservative_clearance_threshold_with_loop_m':threshold,'next_clearance_threshold_without_loop_m':next_threshold,'topology_method':'4-connected allowed grid, 8-connected complement; central complement component must not reach ROI exterior','vehicle_center_components':int(ndi.label(vehicle)[1]),'centerline_and_boundaries_ready':False,'controller_track_exported':False,'unknown_filled':False,'source_maps_modified':False,'limitations':['Result applies to existing grid and orientation-independent enclosing-circle footprint.','Clearance threshold is not a physical road-width measurement.','A yaw-dependent rectangle or better terrain evidence could change the result.','No live localization or real-vehicle feasibility certified.']}
    if not surface_loop:
        (OUT/'summary.json').write_text(json.dumps(result,indent=2)+'\n');return
    pixels=cut_loop(surface,clear,ayx,res);xy=origin+(pixels[:,::-1]+.5)*res
    vectors=xy-anchor;turn=np.arctan2((vectors[:-1,0]*vectors[1:,1]-vectors[:-1,1]*vectors[1:,0]),(vectors[:-1]*vectors[1:]).sum(1)).sum()/(2*np.pi)
    assert abs(abs(turn)-1)<1e-6
    # Validate entire segments, not just vertices, at 1/4-cell spacing.
    dense=np.concatenate([xy[:-1]+u*(xy[1:]-xy[:-1]) for u in np.linspace(0,1,5)])
    ids=np.floor((dense-origin)/res).astype(int);assert surface[ids[:,1],ids[:,0]].all()
    arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))];c=clear[pixels[:,0],pixels[:,1]];bad=~vehicle[pixels[:,0],pixels[:,1]]
    indexes=np.flatnonzero(bad);groups=np.split(indexes,np.flatnonzero(np.diff(arc[indexes])>1.)+1) if len(indexes) else []
    _,near=ndi.distance_transform_edt(surface,return_indices=True);rows=[]
    for g in groups:
        if not len(g):continue
        a,b=int(g[0]),int(g[-1]);ny,nx=near[:,pixels[g,0],pixels[g,1]];ob=d['obstacle'][ny,nx];f=d['floor_scans'][ny,nx];mid=g[np.argmin(c[g])]
        rows.append({'id':'B'+str(len(rows)+1).zfill(2),'start_index':a,'end_index':b,'path_span_m':float(arc[b]-arc[a]),'min_clearance_lower_bound_m':float(c[g].min()),'center_xy_m':xy[mid].tolist(),'roi_xy':[ (xy[a:b+1].min(0)-1.5).tolist(),(xy[a:b+1].max(0)+1.5).tolist()],'near_obstacle_path_vertices':int(ob.sum()),'near_unknown_no_floor_vertices':int((~ob&(f==0)).sum()),'near_unknown_floor1to2_vertices':int((~ob&(f>0)&(f<3)).sum()),'reason':'near_unknown' if (~ob).sum()>=ob.sum() else 'near_static_obstacle'})
    result.update({'reference_loop_length_m':float(arc[-1]),'reference_loop_winding_number':float(turn),'reference_loop_vertices':len(xy),'reference_loop_segment_samples_in_candidate':len(dense),'reference_loop_vertices_failing_vehicle_mask':int(bad.sum()),'reference_loop_bottleneck_groups':len(rows),'reference_loop_is_controller_input':False})
    np.savetxt(OUT/'reference_loop_DIAGNOSTIC_ONLY.csv',np.c_[arc,xy,c,bad.astype(int)],delimiter=',',header='s,x,y,clearance_lower_bound,vehicle_mask_reject',comments='')
    np.savez_compressed(OUT/'reference_loop.npz',xy=xy,pixels_yx=pixels,arc=arc,clearance=c,rejected=bad,plane=d['plane'])
    (OUT/'summary.json').write_text(json.dumps(result,indent=2)+'\n');(OUT/'bottlenecks.json').write_text(json.dumps(rows,indent=2)+'\n')
    with (OUT/'bottlenecks.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['id','x','y','reference_path_span_m','min_clearance_lower_bound_m','reason'])
        for r in rows:w.writerow([r['id'],*r['center_xy_m'],r['path_span_m'],r['min_clearance_lower_bound_m'],r['reason']])
    hashes={}
    for p in [BASE/'area.npz',BASE/'vehicle_clearance.npz',Path(__file__)]:
        with p.open('rb') as f:hashes[str(p)]=hashlib.file_digest(f,'sha256').hexdigest()
    (OUT/'provenance.json').write_text(json.dumps(hashes,indent=2)+'\n');plot(d,v,xy,rows)
    print(json.dumps(result),flush=True)


def plot(d,v,xy,rows):
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    origin=d['origin'];res=float(d['resolution']);h,w=d['candidate'].shape;extent=[origin[0],origin[0]+w*res,origin[1],origin[1]+h*res]
    code=np.zeros((h,w),np.uint8);code[d['candidate']]=1;code[d['obstacle']]=2;code[v['vehicle_centers']]=3
    fig,axs=plt.subplots(1,2,figsize=(17,9),layout='constrained')
    axs[0].imshow(d['candidate'],origin='lower',extent=extent,cmap=ListedColormap(['#646b75','#afd8b8']),interpolation='nearest');axs[0].plot(xy[:,0],xy[:,1],c='#1956a5',lw=1);axs[0].set_title('Surface-only reference loop exists\nDiagnostic grid path, not a smoothed/controller centerline')
    axs[1].imshow(code,origin='lower',extent=extent,cmap=ListedColormap(['#646b75','#b9d0bf','#d19067','#489560']),vmin=0,vmax=3,interpolation='nearest');axs[1].plot(xy[:,0],xy[:,1],c='#1956a5',lw=.8);axs[1].set_title('Current vehicle-center mask: NO enclosing closed loop\nDark green: vehicle centers / Gray: unknown / Orange: obstacles')
    for r in rows:
        a,b=r['start_index'],r['end_index'];axs[1].plot(xy[a:b+1,0],xy[a:b+1,1],c='#d92991',lw=2);x,y=r['center_xy_m'];axs[1].annotate(r['id'],(x,y),xytext=(4,6),textcoords='offset points',fontsize=9,bbox=dict(fc='#ffef80',alpha=.9,boxstyle='round,pad=.15'))
    for ax in axs:ax.set(xlabel='x (m)',ylabel='y (m)',aspect='equal');ax.scatter(0,18,marker='x',c='black',s=60)
    fig.savefig(OUT/'loop_feasibility.png',dpi=150);plt.close(fig)
    for r in rows:
        fig,ax=plt.subplots(figsize=(8,7),layout='constrained');ax.imshow(code,origin='lower',extent=extent,cmap=ListedColormap(['#646b75','#b9d0bf','#d19067','#489560']),vmin=0,vmax=3,interpolation='nearest');ax.plot(xy[:,0],xy[:,1],c='#1956a5',lw=1);a,b=r['start_index'],r['end_index'];ax.plot(xy[a:b+1,0],xy[a:b+1,1],c='#d92991',lw=2);lo,hi=np.array(r['roi_xy']);ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),aspect='equal',xlabel='x (m)',ylabel='y (m)',title=r['id']+' | '+r['reason']+'\nReference-path exclusion; not physical gap width');fig.savefig(OUT/(r['id']+'.png'),dpi=130);plt.close(fig)

if __name__=='__main__':main()
