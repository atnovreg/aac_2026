"""Conservative simulation track from Geonji v4 assumed-floor center mask."""
import json,hashlib,argparse
from pathlib import Path
import numpy as np
from scipy import ndimage as ndi
from scipy.spatial import cKDTree
from planning.geonji.check_geonji_loop import cut_loop
from planning.track_pipeline.geometry import (resample_closed,sample_grid,area,self_intersections,validate_strip)
from control.acados_mpcc.track import Track

SOURCE=Path('results/건지광장/drivable_area/083146_v4_assumed_floor')
OUT=Path('tracks/geonji_083146_assumed')

def box_clearance(mask,points,res,origin):
    boxes=origin+(np.argwhere(~np.pad(mask,1))[:,::-1]-.5)*res
    tree=cKDTree(boxes);nearest=tree.query(points)[0]
    # Any cell able to beat the nearest-center upper bound lies within this ball.
    groups=tree.query_ball_point(points,nearest+res/np.sqrt(2)+1e-9)
    return np.array([np.linalg.norm(np.maximum(np.abs(p-boxes[idx])-res/2,0),axis=1).min() for p,idx in zip(points,groups)])

def route_search_mask(mask,origin,res,inner,right_inner=False):
    selected=mask.copy()
    if inner:
        yy,xx=np.indices(mask.shape);x=origin[0]+(xx+.5)*res;y=origin[1]+(yy+.5)*res
        # Topological route-selection gate from the left ROI edge into the round
        # obstacle. It is not obstacle evidence and never modifies the area mask.
        selected[(x<-29.)&(y>=12.4)&(y<=13.0)]=False
    if right_inner:
        yy,xx=np.indices(mask.shape);x=origin[0]+(xx+.5)*res;y=origin[1]+(yy+.5)*res
        selected[(x>22.)&(y>=10.)&(y<=10.6)]=False
    return selected

def main(inner=False,right_inner=False,source=None,output=None):
    global OUT,SOURCE
    if source is not None:SOURCE=Path(source)
    if inner:OUT=Path('tracks/geonji_083146_inner')
    if right_inner:OUT=Path('tracks/geonji_083146_both_inner')
    if output is not None:OUT=Path(output)
    d=dict(np.load(SOURCE/'area.npz'));v=dict(np.load(SOURCE/'vehicle_clearance.npz'));mask=v['vehicle_centers'];res=float(d['resolution']);origin=d['origin']
    ij=np.floor((np.array([0.,18.])-origin)/res).astype(int)
    clearance=np.maximum(0,ndi.distance_transform_edt(np.pad(mask,1))[1:-1,1:-1]*res-res/np.sqrt(2))
    pixels=cut_loop(route_search_mask(mask,origin,res,inner,right_inner),clearance*.3,tuple(ij[::-1]),res)
    seed=origin+(pixels[:,::-1]+.5)*res
    if np.linalg.norm(seed[-1]-seed[0])<1e-8:seed=seed[:-1]
    if area(seed)<0:seed=seed[::-1]
    seed=np.roll(seed,-np.argmin(np.linalg.norm(seed-d['poses'][0,1:3],axis=1)),axis=0)
    s,xy,yaw,k,length=resample_closed(seed,.15,1.5)
    # Evaluate exactly the periodic B-spline that the acados track loader uses.
    provisional=Track(s,xy,np.ones(len(s)),np.ones(len(s)),length)
    ref=provisional.reference(s);normal=np.c_[-ref['tangent'][:,1],ref['tangent'][:,0]]
    cap=np.minimum.reduce([box_clearance(mask,xy,res,origin)-.03,np.full(len(s),1.),.65/np.maximum(abs(ref['curvature']),1e-6)])
    widths=ndi.minimum_filter1d(cap,size=9,mode='wrap')
    assert widths.min()>.15, widths.min()
    track=Track(s,xy,widths,widths,length)
    dense_s=np.linspace(0,length,int(np.ceil(length/.02)),endpoint=False);r=track.reference(dense_s);n=np.c_[-r['tangent'][:,1],r['tangent'][:,0]]
    # Validate the actual curved solver strips, including every cross section.
    points=r['center'][:,None,:]+n[:,None,:]*r['width_left'][:,None,None]*np.linspace(-1,1,101)[None,:,None]
    assert sample_grid(mask,points.reshape(-1,2),res,origin).all()
    left=xy+normal*widths[:,None];right=xy-normal*widths[:,None]
    metrics=validate_strip(xy,left,right,mask,res,origin)
    metrics.update(centre_self_crossings=self_intersections(xy),left_self_crossings=self_intersections(left),right_self_crossings=self_intersections(right))
    assert not any(metrics[key] for key in ['strip_samples_outside_usable','mutual_boundary_crossings','centre_self_crossings','left_self_crossings','right_self_crossings'])
    vec=r['center']-np.array([0,18]);following=np.roll(vec,-1,axis=0);winding=np.arctan2(vec[:,0]*following[:,1]-vec[:,1]*following[:,0],(vec*following).sum(1)).sum()/(2*np.pi);assert abs(winding-1)<1e-6
    metrics.update(geometry_valid=True,length_m=length,samples=len(s),min_half_width_m=float(widths.min()),max_half_width_m=float(widths.max()),max_abs_curvature_per_m=float(abs(r['curvature']).max()),solver_strip_samples_checked=int(points.shape[0]*points.shape[1]),solver_strip_outside=0,winding_number=float(winding),limits_semantics='vehicle-center permissible limits, footprint already accounted for',footprint_sides_m=json.loads((SOURCE/'vehicle_clearance.json').read_text())['rectangle_side_lengths_m'],footprint_margin_m=json.loads((SOURCE/'vehicle_clearance.json').read_text())['extra_margin_m'],additional_solver_body_margin_m=0,ready_for_vehicle=False,assumed_floor=True)
    OUT.mkdir(parents=True,exist_ok=False)
    def csv(name,a,header):np.savetxt(OUT/name,a,delimiter=',',header=header,comments='',fmt='%.10f')
    csv('track.csv',np.c_[s,xy,ref['yaw'],ref['curvature'],widths,widths],'s,x,y,yaw,curvature,width_left,width_right')
    csv('centerline.csv',np.c_[s,xy,ref['yaw'],ref['curvature']],'s,x,y,yaw,curvature')
    csv('left_boundary.csv',np.c_[s,left,widths],'s,x,y,half_width');csv('right_boundary.csv',np.c_[s,right,widths],'s,x,y,half_width')
    (OUT/'validation.json').write_text(json.dumps(metrics,indent=2)+'\n')
    config={'right_inner_gate':{'x_greater_than':22.,'y_range':[10.,10.6]} if right_inner else None,'route_choice':'both inner passages' if right_inner else 'left round obstacle inner passage' if inner else 'unconstrained clearance preference','search_gate':{'x_less_than':-29.,'y_range':[12.4,13.0]} if inner else None,'source':str(SOURCE),'seed_clearance_cost_scale':.3,'smoothing_sigma_m':1.5,'sample_spacing_m':.15,'limit_clearance_reserve_m':.03,'max_half_width_m':1.,'width_min_filter_samples':9,'assumption':'recorded-route +/-1m floor fill, static obstacles retained','source_hashes':{str(SOURCE/f):hashlib.sha256((SOURCE/f).read_bytes()).hexdigest() for f in ['area.npz','vehicle_clearance.npz']}}
    (OUT/'extraction.json').write_text(json.dumps(config,indent=2)+'\n')
    np.savez_compressed(OUT/'visualization.npz',center=xy,left=left,right=right,plane=d['plane'])
    (OUT/'README.md').write_text('# 건지광장 083146 시뮬레이션 트랙\n\n바닥 보완 v4 기반. 정적 장애물과 남은 미관측 영역 유지. 중앙부를 반시계 방향으로 감싸는 폐곡선. 중심선은 도로의 측량 중심이 아니라 여유를 확보한 기준 경로입니다.\n\n좌우 제한선은 validation.json에 기록된 차량 외접원과 여유를 이미 적용한 차량 중심 허용 영역 안에 있습니다. acados에서는 vehicle_width: 0으로 추가 차폭 차감을 끕니다. 동역학은 ORCA 1:43 그대로이므로 실제 차량 모델 검증은 아닙니다.\n\n폭은 금지 셀 사각형까지의 거리, 3cm 수치 여유, 곡률, 최대 편측 1m를 기준으로 보수적으로 제한합니다. 좌우 폭은 대칭이며 위치에 따라 변합니다. 물리적인 도로 경계가 아닙니다.\n')
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    code=np.zeros(mask.shape,int);code[d['observed_floor']]=1;code[d['assumed_floor']]=2;code[d['obstacle']]=3
    h,w=mask.shape;extent=[origin[0],origin[0]+w*res,origin[1],origin[1]+h*res]
    fig,ax=plt.subplots(figsize=(12,10),layout='constrained');ax.imshow(code,origin='lower',extent=extent,cmap=ListedColormap(['#abb0b8','#f2f6ed','#b4e8ef','#df976d']),vmin=0,vmax=3)
    for curve,color,label in [(xy,'#245ada','Centerline'),(left,'#108641','Left center limit'),(right,'#d041a2','Right center limit')]:
        closed=np.vstack([curve,curve[0]]);ax.plot(*closed.T,c=color,lw=1.2,label=label)
    ax.scatter(*xy[0],marker='*',s=100,c='red',label='Start CCW');ax.set(title=f'Geonji Plaza | assumed-floor simulation track | {length:.1f} m',xlabel='x (m)',ylabel='y (m)',aspect='equal');ax.legend();fig.savefig(OUT/'track_overlay.png',dpi=160);plt.close(fig)
    print(json.dumps(metrics),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--inner',action='store_true');p.add_argument('--both-inner',action='store_true');p.add_argument('--source');p.add_argument('--output');args=p.parse_args();main(args.inner or args.both_inner,args.both_inner,args.source,args.output)
