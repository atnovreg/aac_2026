"""Independent map-domain check of dense simulated vehicle positions."""
import argparse,json
from pathlib import Path
import numpy as np
from tools.track_pipeline.geometry import sample_grid
from tools.acados_mpcc.track import Track

def main(run):
 track_path=Path(json.loads((run/'provenance.json').read_text())['track'])
 extraction=json.loads((track_path/'extraction.json').read_text())
 base=Path(extraction['source']);d=np.load(base/'area.npz');v=np.load(base/'vehicle_clearance.npz');a=np.load(run/'simulation.npz');dense=a['dense'];rows=a['trajectory'];xy=dense[:,1:3]
 center_ok=sample_grid(v['vehicle_centers'],xy,float(d['resolution']),d['origin'])
 # Resample all intersample segments to <=1cm for the map-domain check.
 parts=[xy[:1]]
 for start,end in zip(xy[:-1],xy[1:]):
  count=max(1,int(np.ceil(np.linalg.norm(end-start)/.01)));parts.append(start+np.arange(1,count+1)[:,None]/count*(end-start))
 fine=np.vstack(parts);fine_ok=sample_grid(v['vehicle_centers'],fine,float(d['resolution']),d['origin'])
 report={'dense_plant_samples':len(xy),'dense_samples_outside_vehicle_center_mask':int((~center_ok).sum()),'map_segment_samples':len(fine),'map_segment_samples_outside_vehicle_center_mask':int((~fine_ok).sum()),'solver_deadline_20ms_exceedances':int((rows[:,13]>.02).sum()),'controller_deadline_20ms_exceedances':int((rows[:,14]>.02).sum()),'steps':len(rows),'model':json.loads((run/'summary.json').read_text())['model'],'ground_assumptions':'v4 route +/-1m; static obstacles unchanged'}
 (run/'map_validation.json').write_text(json.dumps(report,indent=2)+'\n')
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib.colors import ListedColormap
 track=Track.from_csv(Path(json.loads((run/'provenance.json').read_text())['track'])/'track.csv');ref=track.reference(np.linspace(0,track.length,4000));normal=np.c_[-ref['tangent'][:,1],ref['tangent'][:,0]]
 code=np.zeros(d['candidate'].shape,int);code[d['observed_floor']]=1;code[d['assumed_floor']]=2;code[d['obstacle']]=3;h,w=code.shape;res=float(d['resolution']);o=d['origin'];extent=[o[0],o[0]+w*res,o[1],o[1]+h*res]
 fig,ax=plt.subplots(figsize=(12,10),layout='constrained');ax.imshow(code,origin='lower',extent=extent,cmap=ListedColormap(['#abb0b8','#f2f6ed','#b4e8ef','#df976d']),vmin=0,vmax=3)
 for sign,color,label in [(1,'#108641','Left center limit'),(-1,'#d041a2','Right center limit')]:
  boundary=ref['center']+sign*normal*ref['width_left'][:,None];ax.plot(*boundary.T,c=color,lw=1,label=label)
 ax.plot(*ref['center'].T,'--',c='#245ada',lw=.8,label='Reference centerline');ax.plot(rows[:,1],rows[:,2],c='#e32218',lw=1,label='acados simulated vehicle');summary=json.loads((run/'summary.json').read_text());ax.set(title=f"Geonji Plaza | simulation | lap completed: {summary['completed_lap']}\nAssumed-floor map; planar model, no bumps or sensor noise",xlabel='x (m)',ylabel='y (m)',aspect='equal');ax.legend();fig.savefig(run/'map_simulation_overlay.png',dpi=160);plt.close(fig)
 print(json.dumps(report),flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);main(p.parse_args().run)
