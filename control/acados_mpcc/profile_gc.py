import gc,json,time,yaml
from pathlib import Path
from control.acados_mpcc.controller import default_config
from control.acados_mpcc.simulation import run
cfg=default_config();cfg.update(yaml.safe_load(Path('src/aac_2026/control/acados_mpcc/config/acados_geonji_inner_rti_profile.yaml').read_text()))
events=[]
def callback(phase,info):events.append({'phase':phase,'time':time.perf_counter(),'cpu':time.thread_time(),**info})
gc.callbacks.append(callback)
try:out,summary=run(cfg)
finally:gc.callbacks.remove(callback)
(out/'gc_events.json').write_text(json.dumps(events,indent=2))
a=json.loads((out/'solver_statistics.json').read_text())
for row in sorted(a,key=lambda x:x['cycle_time_s'],reverse=True)[:3]:
 start=row['cycle_started_monotonic_s'];end=start+row['cycle_time_s'];print('TIMING_GC',row['step'],row['cycle_time_s'],[e for e in events if start<=e['time']<=end],flush=True)
