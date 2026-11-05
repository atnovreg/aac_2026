"""Metric map/track and fixed baseline assets, generated from existing results."""
from io import BytesIO
import json
import numpy as np
import yaml
from PIL import Image
from .config import TRACKS,flatten

def map_png(key='both_inner'):
    with np.load(TRACKS[key]['area']/'area.npz') as d:
        code=np.zeros(d['candidate'].shape,dtype=np.uint8)
        code[d['observed_floor']]=1;code[d['assumed_floor']]=2;code[d['obstacle']]=3
        colors=np.array([[217,222,228],[248,249,244],[177,222,227],[210,165,137]],dtype=np.uint8)
        image=Image.fromarray(colors[code][::-1]);buf=BytesIO();image.save(buf,format='PNG');return buf.getvalue()

def track_data(key):
    spec=TRACKS[key];track=spec['track']
    with np.load(track/'visualization.npz') as v:
        curves={k:v[k].tolist() for k in ['center','left','right']}
    with np.load(spec['area']/'area.npz') as d:
        h,w=d['candidate'].shape;res=float(d['resolution']);x,y=d['origin'];extent=[float(x),float(x+w*res),float(y),float(y+h*res)]
    runs=sorted(spec['baseline'].glob('*/summary.json'))
    if len(runs)!=1:raise ValueError(f'{key}: 코스 기준 실행은 정확히 하나여야 합니다.')
    run=runs[0].parent
    rows=np.load(run/'simulation.npz')['trajectory']
    summary=json.loads((run/'summary.json').read_text())
    summary['rms_lateral_offset_m']=float(np.sqrt(np.mean(rows[:,11]**2)))
    summary['controller_deadline_misses']=int((rows[:,14]>.02).sum())
    weights=flatten(yaml.safe_load((run/'config.yaml').read_text())['weights'])
    return dict(id=key,label=spec['label'],length_m=summary['track_length_m'],**curves,
                map={'url':f'/map.png?track={key}','extent':extent,'revision':spec['revision']},
                baseline={'trajectory':rows.tolist(),'summary':summary,'weights':weights})
