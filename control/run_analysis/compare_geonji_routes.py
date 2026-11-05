"""Compare three saved, identically configured RTI runs on Geonji v4."""
from pathlib import Path
import json,csv
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.font_manager import FontProperties
from planning.track_pipeline.geometry import sample_grid

BASE=Path('results/건지광장/comparison/20260915_three_routes')
SOURCE=Path('results/건지광장/drivable_area/083146_v4_assumed_floor')
CASES=[('01_outer','① 양쪽 바깥','#2674d9'),('02_left_inner','② 왼쪽 안쪽','#e3871d'),('03_both_inner','③ 양쪽 안쪽','#b637bc')]

def main():
 font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name();plt.rcParams.update({'font.family':font,'axes.unicode_minus':False})
 source=SOURCE;d=np.load(source/'area.npz');mask=np.load(source/'vehicle_clearance.npz')['vehicle_centers'];res=float(d['resolution']);origin=d['origin'];h,w=mask.shape;extent=[origin[0],origin[0]+w*res,origin[1],origin[1]+h*res]
 code=np.zeros(mask.shape,int);code[d['observed_floor']]=1;code[d['assumed_floor']]=2;code[d['obstacle']]=3;cmap=ListedColormap(['#c1c6cc','#f6f8f0','#b6e7ee','#e09b76'])
 rows=[];data=[];common=None;hashes=None
 for case,label,color in CASES:
  runs=sorted((BASE/case).glob('*/summary.json'));assert len(runs)==1,'Expected one comparable run per case'
  run=runs[0].parent;s=json.loads(runs[0].read_text());config=yaml.safe_load((run/'config.yaml').read_text());track=Path(config['track']);provenance=json.loads((run/'provenance.json').read_text())
  normalized={k:v for k,v in config.items() if k not in ['track','output']}
  if common is None:common=normalized;hashes=provenance['source_sha256']
  assert normalized==common and provenance['source_sha256']==hashes
  a=np.genfromtxt(run/'trajectory.csv',delimiter=',',names=True);dense=np.load(run/'simulation.npz')['dense'];geometry=json.loads((track/'validation.json').read_text());visual=np.load(track/'visualization.npz')
  inside=sample_grid(mask,dense[:,1:3],res,origin)
  row={'case':case,'label':label,'run':str(run),'track':str(track),'length_m':s['track_length_m'],'completed_lap':s['completed_lap'],'lap_time_s':s['simulation_time_s'],'max_offset_m':s['max_abs_lateral_offset_m'],'rms_offset_m':float(np.sqrt(np.mean(a['lateral_offset']**2))),'corridor_violation_m':s['max_corridor_violation_m'],'map_outside_samples':int((~inside).sum()),'min_half_width_m':geometry['min_half_width_m'],'max_curvature_per_m':geometry['max_abs_curvature_per_m'],'solver_mean_ms':s['solver_time_ms']['mean'],'solver_max_ms':s['solver_time_ms']['max'],'controller_mean_ms':s['controller_cycle_time_ms']['mean'],'controller_p95_ms':s['controller_cycle_time_ms']['p95'],'controller_max_ms':s['controller_cycle_time_ms']['max'],'deadline_misses':int((a['controller_cycle_time']>.02).sum()),'steps':len(a),'max_steering_deg':float(np.max(abs(a['steering']))*180/np.pi)}
  rows.append(row);data.append((label,color,a,visual))
 (BASE/'comparison.json').write_text(json.dumps({'common_configuration':common,'same_controller_sources':True,'cases':rows,'scope':'Single sequential run per route; desktop wall-clock timing is not a real-time worst-case guarantee. Same configured model, same assumed-floor map.'},indent=2,ensure_ascii=False)+'\n')
 with (BASE/'comparison.csv').open('w') as f:
  writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
 def background(ax):ax.imshow(code,origin='lower',extent=extent,cmap=cmap,vmin=0,vmax=3);ax.set(xlabel='x (m)',ylabel='y (m)',aspect='equal')
 fig,axs=plt.subplots(1,3,figsize=(18,7.5),layout='constrained')
 for ax,(label,color,a,v),row in zip(axs,data,rows):
  background(ax)
  for side in ['left','right']:
   curve=np.vstack([v[side],v[side][0]]);ax.plot(*curve.T,c='#747474',lw=.65)
  ax.plot(a['x'],a['y'],c=color,lw=1.6)
  ax.set_title(f"{label}\n{row['length_m']:.2f} m · {row['lap_time_s']:.2f} s\n최대 횡오차 {row['max_offset_m']*100:.1f} cm")
 fig.suptitle(('건지광장 — Slash 58276-74 + VESC 가정 모델' if common.get('model') == 'slash_58276_74' else '건지광장 3개 코스 비교 — 동일 RTI·ORCA 모델·지도')+'\n색 선: 시뮬레이션 주행 / 회색 선: 차량 중심 좌우 제한선',fontsize=15);fig.savefig(BASE/'three_routes.png',dpi=160);plt.close(fig)
 fig,axs=plt.subplots(1,2,figsize=(13,9),layout='constrained')
 for ax in axs:
  background(ax)
  for label,color,a,v in data:ax.plot(a['x'],a['y'],c=color,lw=1.8,label=label)
  ax.legend()
 axs[0].set(xlim=(-36,-24),ylim=(4,23),title='왼쪽 원형 장애물 통과');axs[1].set(xlim=(17,32),ylim=(-1,22),title='오른쪽 원형 장애물 통과');fig.savefig(BASE/'passage_comparison.png',dpi=160);plt.close(fig)
 fig,axs=plt.subplots(2,1,figsize=(12,7),layout='constrained')
 for label,color,a,v in data:
  axs[0].plot(a['progress'],a['vx'],c=color,label=label,lw=1);axs[1].plot(a['progress'],a['controller_cycle_time']*1000,c=color,label=label,lw=.6)
 axs[0].set(ylabel='전진 속도 (m/s)',title='속도와 전체 제어 계산 시간');axs[1].axhline(20,c='red',ls='--',label='50Hz 기준 20ms');axs[1].set(xlabel='각 코스의 진행 거리 (m)',ylabel='제어 계산 (ms)')
 for ax in axs:ax.legend();ax.grid(alpha=.2)
 fig.savefig(BASE/'speed_timing.png',dpi=150);plt.close(fig)
 lines=['# 건지광장 3가지 코스 비교','', '모두 같은 RTI/Gauss-Newton, 동일 차량 모델, 20ms 주기, 예측 길이·비용·제약으로 순차 실행했습니다. 시작 전 GC 정리도 동일합니다. 기존 SQP 결과와 혼합하지 않았습니다.','', '| 코스 | 길이(m) | 완주(s) | 최대 횡오차(cm) | 제어 평균/최대(ms) | 20ms 초과 |','|---|---:|---:|---:|---:|---:|']
 for r in rows:lines.append(f"| {r['label']} | {r['length_m']:.2f} | {r['lap_time_s']:.2f} | {r['max_offset_m']*100:.1f} | {r['controller_mean_ms']:.2f} / {r['controller_max_ms']:.2f} | {r['deadline_misses']} / {r['steps']} |")
 lines+=['','지도상 허용 영역 및 제한선 이탈 여부는 comparison.json에 기록했습니다. 경계는 각 트랙 validation.json의 차량 외접원과 여유를 이미 반영한 차량 중심 허용 범위입니다.','', '각 코스 한 번씩 실행한 결과입니다. 작은 시간 차이는 20ms 제어주기와 완주선 통과 판정의 영향을 받으며 반복실험으로 통계적 우위를 검증한 결과가 아닙니다. CPU 부하는 완전히 통제하지 않았으므로 계산 시간 차이를 코스 자체의 우열로 해석하지 않습니다. 미실측 평면 모델과 보완 바닥 기반이며 실차 성능 보장은 아닙니다.','', '그림: three_routes.png / passage_comparison.png / speed_timing.png','재생 로그와 생성 solver는 각 case 폴더에 보존했습니다.']
 (BASE/'README.md').write_text('\n'.join(lines)+'\n')
 print(json.dumps(rows,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
