"""Simulation-only floor assumptions in a bounded recorded-trajectory corridor."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy import ndimage as ndi
from tools.vehicle_area_clearance import main as vehicle_clearance
from tools.check_geonji_loop import encloses,cut_loop

def fill_route_gaps(observed,obstacle,origin,res,route,radius):
    if radius<=0 or len(route)<2:raise ValueError('Positive radius and a trajectory required')
    yy,xx=np.indices(observed.shape);points=origin+np.c_[xx.ravel()+.5,yy.ravel()+.5]*res
    distance=np.full(len(points),np.inf)
    # Exact distance to polyline segments; never bridge between nonadjacent poses.
    for a,b in zip(route[:-1],route[1:]):
        delta=b-a;den=delta@delta
        t=np.clip((points-a)@delta/den,0,1) if den>0 else np.zeros(len(points))
        distance=np.minimum(distance,np.linalg.norm(points-(a+t[:,None]*delta),axis=1))
    distance=distance.reshape(observed.shape)
    assumed=(distance<=radius)&~obstacle&~observed
    return (observed|assumed)&~obstacle,assumed,distance

def main(source,out,radius):
    out.mkdir(parents=True,exist_ok=False)
    d=dict(np.load(source/'area.npz'));before=hashlib.sha256((source/'area.npz').read_bytes()).hexdigest()
    res=float(d['resolution']);observed=d['candidate'].copy()
    candidate,assumed,distance=fill_route_gaps(observed,d['obstacle'],d['origin'],res,d['poses'][:,1:3],radius)
    d.update(candidate=candidate,observed_floor=observed,assumed_floor=assumed,route_distance=distance)
    occupancy=np.full(candidate.shape,-1,np.int8);occupancy[candidate]=0;occupancy[d['obstacle']]=100;d['occupancy']=occupancy
    d['clearance']=np.maximum(0,ndi.distance_transform_edt(np.pad(candidate,1))[1:-1,1:-1]*res-res/np.sqrt(2));d['clearance'][~candidate]=0
    labels,_=ndi.label(candidate);sizes=np.bincount(labels.ravel());sizes[0]=0;d['main_component']=labels==sizes.argmax()
    np.savez_compressed(out/'area.npz',**d)
    vehicle_clearance(out,.56,.19,.05)
    v=dict(np.load(out/'vehicle_clearance.npz'));ij=np.floor((np.array([0.,18.])-d['origin'])/res).astype(int);anchor=tuple(ij[::-1]);connected=encloses(v['vehicle_centers'],anchor)
    summary={'source':str(source),'purpose':'simulation-only assumed floor; not new lidar observations','corridor_half_width_m':radius,'width_basis':'provisional 1m each side of recorded IMU XY polyline; not measured road boundary','assumed_floor_cells':int(assumed.sum()),'assumed_floor_area_m2':float(assumed.sum()*res**2),'static_obstacle_cells':int(d['obstacle'].sum()),'static_obstacles_unchanged':True,'vehicle_center_loop_exists':connected,'controller_track_exported':False,'source_area_sha256':before,'source_maps_modified':False}
    loop=None
    if connected:
        pixels=cut_loop(v['vehicle_centers'],d['clearance'],anchor,res);loop=d['origin']+(pixels[:,::-1]+.5)*res
        dense=np.concatenate([loop[:-1]+u*np.diff(loop,axis=0) for u in np.linspace(0,1,5)])
        ids=np.floor((dense-d['origin'])/res).astype(int);assert v['vehicle_centers'][ids[:,1],ids[:,0]].all()
        np.savetxt(out/'feasible_grid_loop_DIAGNOSTIC_ONLY.csv',loop,delimiter=',',header='x,y',comments='')
        summary['diagnostic_grid_loop_length_m']=float(np.linalg.norm(np.diff(loop,axis=0),axis=1).sum());summary['validated_segment_samples']=len(dense)
    assert not (assumed&d['obstacle']).any()
    assert np.array_equal(np.load(source/'area.npz')['obstacle'],d['obstacle'])
    assert hashlib.sha256((source/'area.npz').read_bytes()).hexdigest()==before
    (out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    (out/'README.md').write_text('# 건지광장 083146 — 시뮬레이션용 바닥 보완\n\n기록된 IMU XY 경로의 좌우 '+str(radius)+'m 이내 미관측 셀만 바닥으로 가정했습니다. 정적 장애물은 그대로 유지합니다. 관측 횟수와 원본 지도는 수정하지 않았습니다.\n\nassumed_floor는 추가 관측이 아닌 가정이며 observed_floor와 별도 저장됩니다. 중앙부 및 경로 밖 빈 영역은 일괄 보완하지 않습니다. 차량 56×19cm의 외접원 + 5cm 여유로 연결성을 확인했습니다. 차량 중심과 IMU의 외부 파라미터는 미확정입니다.\n\nreview.png: 초록=관측 바닥, 하늘색=가정 바닥, 주황=정적 장애물, 회색=미확인, 남색=기록 경로.\n\nfeasible_grid_loop_DIAGNOSTIC_ONLY.csv는 연결성 검사용 격자 경로이며 MPCC 입력 중심선이나 측정 경계가 아닙니다.\n')
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    code=np.zeros(candidate.shape,int);code[observed]=1;code[assumed]=2;code[d['obstacle']]=3
    h,w=code.shape;ox,oy=d['origin'];extent=[ox,ox+w*res,oy,oy+h*res]
    fig,ax=plt.subplots(figsize=(13,10),layout='constrained');ax.imshow(code,origin='lower',extent=extent,cmap=ListedColormap(['#626975','#50b86d','#63cce8','#e8834f']),vmin=0,vmax=3,interpolation='nearest');ax.plot(d['poses'][:,1],d['poses'][:,2],c='#243b86',lw=.8,label='Recorded IMU trajectory');ax.legend();ax.set(title=f'SIMULATION ASSUMPTION | route +/- {radius} m\nGreen: observed floor | Cyan: assumed floor | Orange: static obstacles | Gray: unknown\nVehicle-circle enclosing loop: {connected}',xlabel='x (m)',ylabel='y (m)',aspect='equal');fig.savefig(out/'review.png',dpi=150);plt.close(fig)
    print(json.dumps(summary),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=Path('results/건지광장/drivable_area/083146_v3_static_only'));p.add_argument('--out',type=Path,default=Path('results/건지광장/drivable_area/083146_v4_assumed_floor'));p.add_argument('--radius',type=float,default=1.);a=p.parse_args();main(a.source,a.out,a.radius)
