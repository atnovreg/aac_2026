"""Read-only audit of recorded-path exclusions; not a proof that the road is cut."""
from pathlib import Path
import json,csv,html
import numpy as np
from scipy import ndimage as ndi
from scipy.spatial import cKDTree

BASE=Path('results/건지광장/drivable_area/083146_v2_20cm')
OUT=Path('results/건지광장/gap_review/083146_v1')
NAMES={0:'candidate',1:'center_obstacle',2:'center_unknown_no_floor',3:'center_unknown_few_floor',4:'clearance_near_obstacle',5:'clearance_near_unknown'}
KOREAN={0:'차체 여유 충족',1:'중심 칸에 장애물 증거',2:'중심 칸에 기준 바닥 관측 없음',3:'중심 칸 바닥 관측 1~2스캔',4:'주변 장애물 칸에 가까움',5:'주변 미확인 칸에 가까움'}

def main():
 from scipy.spatial.transform import Rotation,Slerp
 from rosbags.rosbag1 import Reader
 from rosbags.typesys import Stores,get_typestore
 from sensors.bag_adapter.pointcloud_layout import OUTPUT_DTYPE
 d=np.load(BASE/'area.npz');v=np.load(BASE/'vehicle_clearance.npz');t=d['poses'];xy=t[:,1:3];res=float(d['resolution']);origin=d['origin'];shape=d['candidate'].shape;plane=d['plane'];radius=float(v['radius'])
 ij=np.floor((xy-origin)/res).astype(int);yi,xi=ij[:,1],ij[:,0];bad=~v['vehicle_centers'][yi,xi]
 _,near=ndi.distance_transform_edt(d['candidate'],return_indices=True);ny,nx=near[:,yi,xi]
 code=np.zeros(len(t),np.int8);ob=d['obstacle'][yi,xi];cand=d['candidate'][yi,xi];f=d['floor_scans'][yi,xi]
 code[bad&ob]=1;code[bad&~ob&~cand&(f==0)]=2;code[bad&~ob&~cand&(f>0)]=3
 code[bad&cand&d['obstacle'][ny,nx]]=4;code[bad&cand&~d['obstacle'][ny,nx]]=5
 # Circle-distance transform is conservative; examine exact disk-to-unavailable-cell-box distance at grid centers.
 uy,ux=np.nonzero(~d['candidate']);blocked=np.c_[origin[0]+(ux+.5)*res,origin[1]+(uy+.5)*res];tree=cKDTree(blocked)
 exact_pass=np.zeros(len(t),bool)
 for i in np.flatnonzero(bad&cand):
  center=origin+(ij[i]+.5)*res;ids=tree.query_ball_point(center,radius+res/np.sqrt(2))
  if ids:
   dist=np.linalg.norm(np.maximum(np.abs(blocked[ids]-center)-res/2,0),axis=1)
   exact_pass[i]=bool((dist>=radius).all())
  else:exact_pass[i]=True
 s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))];ids=np.flatnonzero(bad);groups=np.split(ids,np.flatnonzero(np.diff(s[ids])>.6)+1)
 vehicle_labels,_=ndi.label(v['vehicle_centers'])
 pose_components=vehicle_labels[yi,xi]
 rows=[]
 for num,g in enumerate(groups,1):
  a,b=int(g[0]),int(g[-1]);lo=xy[a:b+1].min(0)-1.5;hi=xy[a:b+1].max(0)+1.5
  reason_counts={NAMES[k]:int((code[g]==k).sum()) for k in range(1,6)};dominant=max(reason_counts,key=reason_counts.get)
  target_ij=np.c_[np.where(cand[g],nx[g],xi[g]),np.where(cand[g],ny[g],yi[g])]
  target=np.unique(target_ij[:,1]*shape[1]+target_ij[:,0])
  row={'id':num,'pose_start':a,'pose_end':b,'start_s':float(t[a,0]-t[0,0]),'end_s':float(t[b,0]-t[0,0]),'estimated_path_span_m':float(s[b]-s[a]),'excluded_poses':len(g),'center_xy':xy[g[len(g)//2]].tolist(),'roi_xy':[lo.tolist(),hi.tolist()],'reason_counts':reason_counts,'dominant_reason':dominant,'exact_disk_pass_despite_conservative_mask_poses':int(exact_pass[g].sum()),'limiting_cells':target.tolist(),'snapshots':[]}
  before=a-1;after=b+1
  if before<0 or after>=len(t):
   row['endpoint_connectivity']='endpoint_unavailable'
  else:
   row['endpoint_components']=[int(pose_components[before]),int(pose_components[after])]
   row['endpoint_connectivity']='same_grid_component' if pose_components[before]>0 and pose_components[before]==pose_components[after] else 'different_grid_components'
  rows.append(row)
 assert sum(r['excluded_poses'] for r in rows)==int(bad.sum())
 assert (code[bad]>0).all()
 # Only limiting cells get detailed per-scan evidence; ROI previews retain whole local frame.
 targets=np.unique(np.concatenate([r['limiting_cells'] for r in rows]));targetmask=np.zeros(shape[0]*shape[1],bool);targetmask[targets]=True
 total=np.zeros(shape[0]*shape[1],np.uint32);floor=np.zeros_like(total);obstacles=np.zeros_like(total);other=np.zeros_like(total)
 first=np.full(len(total),np.inf);last=np.full(len(total),-np.inf);ob_bins=np.zeros(len(total),np.uint8)
 best=[[None,None,None] for r in rows];times=t[:,0]-t[0,0];rot=Slerp(times,Rotation.from_quat(t[:,4:8]));store=get_typestore(Stores.ROS1_NOETIC)
 source=Path('results/dynamic_lio/20260913_inputs/indoor_front180_20260913_083146.bag')
 scans=0
 with Reader(source) as reader:
  cs=[c for c in reader.connections if c.msgtype.endswith('PointCloud2')]
  for conn,_,raw in reader.messages(connections=cs):
   msg=store.deserialize_ros1(raw,conn.msgtype);p=np.frombuffer(msg.data,dtype=OUTPUT_DTYPE);stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9-t[0,0];pt=stamp+p['time'].astype(float);keep=(pt>=0)&(pt<=times[-1]);pt=pt[keep];p=p[keep]
   if not len(pt):continue
   local=np.column_stack([p[k] for k in 'xyz']).astype(float)+[-.011,-.02329,.04412];world=rot(pt).apply(local)+np.column_stack([np.interp(pt,times,t[:,i]) for i in (1,2,3)])
   h=world[:,2]-world[:,:2]@plane[:2]-plane[2];cell=np.floor((world[:,:2]-origin)/res).astype(int);valid=(cell>=0).all(1)&(cell[:,0]<shape[1])&(cell[:,1]<shape[0]);flat=cell[valid,1]*shape[1]+cell[valid,0];hv=h[valid];hit=targetmask[flat];flat=flat[hit];hv=hv[hit]
   allids=np.unique(flat);total[allids]+=1
   fid=np.unique(flat[abs(hv)<=.1]);oid=np.unique(flat[(hv>=.15)&(hv<=1.5)]);xid=np.unique(flat[(abs(hv)>.1)&~((hv>=.15)&(hv<=1.5))]);floor[fid]+=1;obstacles[oid]+=1;other[xid]+=1
   first[oid]=np.minimum(first[oid],stamp);last[oid]=np.maximum(last[oid],stamp);bin6=min(5,max(0,int(stamp/times[-1]*6)));ob_bins[oid]|=1<<bin6
   bucket=min(2,max(0,int(stamp/times[-1]*3)))
   for ri,r in enumerate(rows):
    lo,hi=np.array(r['roi_xy']);sel=(world[:,:2]>=lo).all(1)&(world[:,:2]<=hi).all(1)&(h>-.4)&(h<2)
    count=int(sel.sum())
    if count and (best[ri][bucket] is None or count>best[ri][bucket]['count']):best[ri][bucket]={'t':float(stamp),'count':count,'points':world[sel].astype(np.float32)}
   scans+=1
 # Independent recount must agree exactly with extraction raw evidence at audited cells.
 assert np.array_equal(floor[targets],d['floor_scans'].ravel()[targets])
 assert np.array_equal(obstacles[targets],d['obstacle_scans'].ravel()[targets])
 for ri,r in enumerate(rows):
  cells=np.array(r['limiting_cells']);obs_cells=d['obstacle'].ravel()[cells];missing=~d['candidate'].ravel()[cells]&~obs_cells
  # Spread is evidence of repeated returns, not semantic confirmation of a fixed object.
  bins=np.array([int(x).bit_count() for x in ob_bins[cells]])
  repeated=obs_cells&(bins>=2)&((last[cells]-first[cells])>=10)
  r['limiting_cell_evidence']={'cells':len(cells),'obstacle_cells':int(obs_cells.sum()),'unknown_cells':int(missing.sum()),'no_raw_returns_in_height_or_any_other_band':int((missing&(total[cells]==0)).sum()),'returns_but_no_floor_band':int((missing&(total[cells]>0)&(floor[cells]==0)).sum()),'floor_only_1_or_2_scans':int((missing&(floor[cells]>0)&(floor[cells]<3)).sum()),'repeated_obstacle_cells_across_time_bins':int(repeated.sum()),'obstacle_cells_static_map_only':int((obs_cells&(obstacles[cells]<3)&d['static_obstacle'].ravel()[cells]).sum())}
  for b in best[ri]:r['snapshots'].append(None if b is None else {'elapsed_s':b['t'],'points_in_roi':b['count']})
  r['review_image']=f'gap_{r["id"]:02d}.png'
  plot_detail(r,best[ri],d,v)
 summary={'scope':'Recorded-path exclusions; not proof of a physical road cut or absence of alternative paths.','excluded_poses':int(bad.sum()),'total_poses':len(t),'groups':len(rows),'endpoint_connectivity_counts':{k:sum(r['endpoint_connectivity']==k for r in rows) for k in ['same_grid_component','different_grid_components','endpoint_unavailable']},'grouping_max_distance_between_excluded_samples_m':.6,'merged_intervals_do_not_modify_masks':True,'reason_counts':{NAMES[k]:int((code==k).sum()) for k in range(1,6)},'exact_disk_pass_despite_conservative_mask_poses':int(exact_pass.sum()),'audited_limiting_cells':len(targets),'raw_scans_recounted':scans,'raw_floor_obstacle_counts_match_extraction':True,'snapshot_selection':'Highest point count in each of 3 equal recording-time windows, cropped to ROI; not all frames and not guaranteed independent viewpoints.','vehicle_center_imu_transform_known':False,'changes_to_map_or_area':False}
 OUT.mkdir(exist_ok=True,parents=True);(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(OUT/'regions.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n')
 np.savez_compressed(OUT/'audit.npz',pose_reason=code,pose_exact_disk_pass=exact_pass,target_cells=targets,total_scans=total[targets],floor_scans=floor[targets],obstacle_scans=obstacles[targets],obstacle_time_bins=ob_bins[targets],obstacle_first_s=first[targets],obstacle_last_s=last[targets])
 with (OUT/'regions.csv').open('w') as f:
  w=csv.writer(f);w.writerow(['id','start_s','end_s','path_span_m','excluded_poses','x','y','dominant_reason','exact_disk_pass_poses'])
  for r in rows:w.writerow([r['id'],r['start_s'],r['end_s'],r['estimated_path_span_m'],r['excluded_poses'],*r['center_xy'],r['dominant_reason'],r['exact_disk_pass_despite_conservative_mask_poses']])
 plot_overview(rows,d,v);write_html(rows,summary)
 print(json.dumps(summary),flush=True)


def plot_overview(rows,d,v):
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib.colors import ListedColormap
 res=float(d['resolution']);o=d['origin'];shape=d['candidate'].shape;extent=[o[0],o[0]+shape[1]*res,o[1],o[1]+shape[0]*res]
 code=np.zeros(shape,np.uint8);code[d['candidate']]=1;code[d['obstacle']]=2;code[v['vehicle_centers']]=3
 fig,ax=plt.subplots(figsize=(14,11),layout='constrained');ax.imshow(code,origin='lower',extent=extent,cmap=ListedColormap(['#686b70','#bcd4c4','#da956f','#50a168']),vmin=0,vmax=3,interpolation='nearest');ax.plot(d['poses'][:,1],d['poses'][:,2],lw=.8,c='#344e9e')
 for r in rows:
  x,y=r['center_xy'];ax.annotate(str(r['id']),xy=(x,y),xytext=(6,6),textcoords='offset points',fontsize=10,color='black',bbox=dict(boxstyle='round,pad=.15',fc='yellow',alpha=.85));ax.scatter(x,y,c='red',s=12)
 ax.set(xlabel='x (m)',ylabel='y (m)',aspect='equal',title='Numbered recorded-path exclusion groups\nGray: unknown / Orange: obstacle evidence / Light green: surface / Dark green: vehicle centers')
 fig.savefig(OUT/'overview.png',dpi=150);plt.close(fig)


def plot_detail(r,snaps,d,v):
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib.colors import ListedColormap
 lo,hi=np.array(r['roi_xy']);o=d['origin'];res=float(d['resolution']);shape=d['candidate'].shape;plane=d['plane'];extent=[o[0],o[0]+shape[1]*res,o[1],o[1]+shape[0]*res]
 codes=np.zeros(shape,np.uint8);codes[d['candidate']]=1;codes[d['obstacle']]=2;codes[v['vehicle_centers']]=3
 fig,axs=plt.subplots(2,2,figsize=(12,10),layout='constrained');axs=axs.ravel();axs[0].imshow(codes,origin='lower',extent=extent,cmap=ListedColormap(['#686b70','#bcd4c4','#da956f','#50a168']),vmin=0,vmax=3,interpolation='nearest');axs[0].set_title('Surface / obstacle / unknown / vehicle centers',fontsize=10)
 for ax,b in zip(axs[1:],snaps):
  if b is None:ax.set_title('No local returns in this recording-time third')
  else:
   p=b['points'];h=p[:,2]-p[:,:2]@plane[:2]-plane[2];sc=ax.scatter(p[:,0],p[:,1],c=h,cmap='coolwarm',vmin=-.15,vmax=.6,s=3);ax.set_title(f'Raw scan at {b["t"]:.1f}s\n{b["count"]} local points',fontsize=10);fig.colorbar(sc,ax=ax,shrink=.7,label='Height above road plane (m)')
 for ax in axs:
  ax.plot(d['poses'][:,1],d['poses'][:,2],c='#394ba8',lw=.7);a,b=r['pose_start'],r['pose_end'];ax.plot(d['poses'][a:b+1,1],d['poses'][a:b+1,2],c='red',lw=2);ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),aspect='equal',xlabel='x (m)',ylabel='y (m)');ax.grid(alpha=.15)
 fig.suptitle(f'Gap {r["id"]:02d}: {r["dominant_reason"]} | path span {r["estimated_path_span_m"]:.1f} m\nRed: audited path interval; snapshots are samples, not free-space or semantic proof',fontsize=12)
 fig.savefig(OUT/r['review_image'],dpi=130,bbox_inches='tight');plt.close(fig)


def write_html(rows,summary):
 reverse={v:k for k,v in NAMES.items()}
 s='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>건지광장 끊긴 구간 검토</title><style>body{font:18px/1.7 sans-serif;background:#eef2f5;max-width:1400px;margin:30px auto;padding:20px}article,header{background:white;padding:25px;margin:20px 0}img{width:100%}summary{cursor:pointer;font-size:22px}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #ddd}strong{color:#b35422}</style><header><h1>건지광장 · 기록 궤적에서 제외된 구간</h1><p>번호는 실제 길이 막혔다는 판정이 아니라, 현재 차량 여유 마스크에서 제외된 기록 위치를 묶은 것이다. 다른 경로로 우회할 수 있는지는 별도 확인이 필요하다.</p><p>원본 지도·주행 영역은 수정하지 않았다. 빨간 선은 검토 구간이다. 각 확대 그림에는 전체 기록을 세 시간대로 나눠 해당 위치의 점이 가장 많이 보이는 단일 스캔을 하나씩 골랐다.</p>'''
 s+=f'<p>구간 전후가 같은 격자 연결 성분인 곳: {summary["endpoint_connectivity_counts"]["same_grid_component"]}개. 다른 연결 성분: {summary["endpoint_connectivity_counts"]["different_grid_components"]}개. 끝점 부족: {summary["endpoint_connectivity_counts"]["endpoint_unavailable"]}개. 같은 성분이면 격자상 우회 연결이 남아 있다는 뜻이며 실제 조향 가능 경로를 검증한 것은 아니다.</p>'
 s+=f'<p>총 {summary["excluded_poses"]}개 제외 자세 / {summary["groups"]}개 구간. 기존 거리 근사 대신 원과 셀 사각형의 거리를 정확히 계산하면 {summary["exact_disk_pass_despite_conservative_mask_poses"]}개 자세는 여유 조건을 만족한다. 아직 원본 마스크에는 적용하지 않았다.</p><a href="overview.png"><img src="overview.png"></a></header>'
 for r in rows:
  e=r['limiting_cell_evidence'];connection={'same_grid_component':'전후 같은 연결 성분: 격자상 우회 연결 있음','different_grid_components':'전후 다른 연결 성분: 현재 격자에서 연결 보완 필요','endpoint_unavailable':'시작/종료 끝점 부족'}[r['endpoint_connectivity']];s+=f'<article id="gap{r["id"]}"><h2>구간 {r["id"]:02d} · {KOREAN[reverse[r["dominant_reason"]]]}</h2><p><b>{connection}</b></p><p>{r["start_s"]:.1f}–{r["end_s"]:.1f}초 / 추정 경로 범위 {r["estimated_path_span_m"]:.1f}m / 제외 자세 {r["excluded_poses"]}개</p><p>가까운 제한 칸 {e["cells"]}개: 장애물 {e["obstacle_cells"]}, 미확인 {e["unknown_cells"]}. 미확인 중 원본 반환점 없음 {e["no_raw_returns_in_height_or_any_other_band"]}, 반환점은 있지만 기준 바닥점 없음 {e["returns_but_no_floor_band"]}, 바닥 관측 1–2스캔 {e["floor_only_1_or_2_scans"]}.</p><p>서로 다른 시간 구간에서 10초 이상 간격으로 장애물 높이 점이 관측된 칸: {e["repeated_obstacle_cells_across_time_bins"]}. <strong>반복 관측은 고정 구조물의 의미 분류를 확정하지 않는다.</strong></p><details><summary>원본 스캔 3장과 영역 확대 비교</summary><a href="{r["review_image"]}"><img loading="lazy" src="{r["review_image"]}"></a></details></article>'
 (OUT/'index.html').write_text(s+'</html>')

if __name__=='__main__':main()
