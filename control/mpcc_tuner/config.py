"""Validated experiment inputs. No solver import or user-supplied filesystem paths."""
from pathlib import Path
from system.f1tenth.workspace import find_workspace
import math
import copy
import yaml

ROOT=find_workspace(__file__)
SPEED_LIMITS={'min':1.,'max':26.8,'step':.1}  # Approximately 60 mph; a cap, not a speed target.
TUNER_PROFILE=yaml.safe_load((ROOT/'src/aac_2026/control/mpcc_tuner/config/mpcc_tuner.yaml').read_text())
BASELINE=ROOT/TUNER_PROFILE['baseline_directory']
ORIGINAL_AREA=ROOT/'results/건지광장/drivable_area/083146_v5_slash_footprint'
EDITED_MAP=ROOT/'results/건지광장/map_edit/20260917_inner_focus'
TRACKS={
    'edited_inner':dict(label='편집 지도 · 양쪽 안쪽',
        track=EDITED_MAP/'track_both_inner',area=EDITED_MAP/'drivable_area',
        baseline=ROOT/'results/건지광장/mpcc_tuner_baseline/20260919_edited_inner/04_edited_inner',
        controller='both_inner',revision='20260917_inner_focus'),
}
for key,label,case in [('outer','이전 지도 · 양쪽 바깥','01_outer'),
                       ('left_inner','이전 지도 · 왼쪽 안쪽','02_left_inner'),
                       ('both_inner','이전 지도 · 양쪽 안쪽','03_both_inner')]:
    TRACKS[key]=dict(label=label,track=ROOT/'tracks'/f'geonji_slash_{key}',
        area=ORIGINAL_AREA,baseline=BASELINE/case,controller=key,revision='083146_v5_slash_footprint')
FIELDS=[
 ('contour','중심선 오차','q꜀',.0001,100.,'log','main','크면 중심선에 더 가깝게 주행하려고 합니다.'),
 ('lag','진행 위치 오차','qₗ',1.,100000.,'log','main','차량과 가상 경로 진행 위치를 연결합니다.'),
 ('progress','진행 보상','qₚ',.0001,2.,'log','main','크면 더 빠른 경로 진행을 선호합니다.'),
 ('increment_steering','조향 변화 벌점','RΔδ',.001,100.,'log','main','크면 조향 변화가 부드러워지지만 대응이 늦어질 수 있습니다.'),
 ('increment_drive','구동 변화 벌점','RΔI',.00001,10.,'log','main','정규화된 구동 전류 변화의 벌점입니다.'),
 ('terminal_contour_multiplier','마지막 중심선 비용 배율','q꜀,N / q꜀',0.,100.,'linear','main','예측 마지막 상태의 중심선 가중치에 곱합니다.'),
 ('yaw_rate','요레이트 벌점','qᵣ',0.,.1,'linear','advanced','차체 회전 속도의 크기를 억제합니다.'),
 ('terminal_yaw_rate_multiplier','마지막 요레이트 배율','qᵣ,N / qᵣ',0.,100.,'linear','advanced','마지막 상태의 요레이트 가중치 배율입니다.'),
 ('input_drive','구동 입력 크기','Rᵢ',0.,1.,'linear','advanced','암페어가 아닌 전류 비율 자체의 벌점입니다.'),
 ('input_steering','조향각 크기','Rδ',0.,1.,'linear','advanced','필요한 코너 조향까지 억제할 수 있습니다.'),
 ('input_progress','가상 진행속도 크기','Rv',.0000001,.1,'log','advanced','진행 보상과 함께 작용합니다. 수식상 양수여야 합니다.'),
 ('increment_progress','가상 진행속도 변화','RΔv',.000001,1.,'log','advanced','가상 진행속도의 변화 벌점입니다.'),
]

def flatten(weights):
    result={k:float(v) for k,v in weights.items() if not isinstance(v,list)}
    for prefix in ['input','increment']:
        for name,v in zip(['drive','steering','progress'],weights[prefix]): result[f'{prefix}_{name}']=float(v)
    return result

def catalog():
    import json
    weights=flatten(yaml.safe_load((ROOT/'src/aac_2026/control/acados_mpcc/config/acados_mpcc.yaml').read_text())['weights'])
    return {'fields':[dict(key=k,label=l,symbol=s,min=lo,max=hi,scale=scale,group=group,description=d,default=weights[k]) for k,l,s,lo,hi,scale,group,d in FIELDS],
            'tracks':[{'id':k,'label':v['label'],'length_m':json.loads((v['track']/'validation.json').read_text())['length_m'],'map_revision':v['revision']} for k,v in TRACKS.items()],
            'defaults':{'track':'edited_inner','speed_max':4.,'weights':weights},'speed_limits':dict(SPEED_LIMITS),'baseline_label':'선택한 지도·코스 기준 · q꜀ 0.1 · 4 m/s','solver_profile':dict(TUNER_PROFILE)}

def validated_request(data):
    if not isinstance(data,dict) or set(data)!={'track','speed_max','weights'}: raise ValueError('track, speed_max, weights를 모두 입력해야 합니다.')
    if not isinstance(data['track'],str) or data['track'] not in TRACKS: raise ValueError('등록된 코스를 선택하세요.')
    def number(value,lo,hi,label):
        if type(value) not in (int,float) or not math.isfinite(value) or not lo<=value<=hi:
            raise ValueError(f'{label}: {lo:g}~{hi:g} 범위의 유한한 숫자를 입력하세요.')
        return float(value)
    speed=number(data['speed_max'],SPEED_LIMITS['min'],SPEED_LIMITS['max'],'최대 속도')
    values=data['weights']
    if not isinstance(values,dict) or set(values)!={f[0] for f in FIELDS}: raise ValueError('가중치 항목이 누락되었거나 알 수 없는 항목이 있습니다.')
    weights={k:number(values[k],lo,hi,label) for k,label,_,lo,hi,*_ in FIELDS}
    return {'track':data['track'],'speed_max':speed,'weights':weights}

def simulation_config(request,output):
    request=validated_request(request)
    cfg=yaml.safe_load((ROOT/'src/aac_2026/control/acados_mpcc/config/acados_mpcc.yaml').read_text())
    spec=TRACKS[request['track']]
    cfg.update(yaml.safe_load((ROOT/f"src/aac_2026/control/acados_mpcc/config/acados_slash_{spec['controller']}_rti.yaml").read_text()))
    cfg['track']=str(spec['track']);cfg['output']=str(output)
    cfg['speed_max']=request['speed_max'];cfg['progress_speed_max']=max(6.,request['speed_max']*1.5)
    # Damps the local QP update; does not alter vehicle, track, or requested cost weights.
    cfg['levenberg_marquardt']=TUNER_PROFILE['levenberg_marquardt']
    weights=copy.deepcopy(cfg['weights']);flat=request['weights']
    for key in list(weights):
        if key in ['input','increment']: weights[key]=[flat[f'{key}_{part}'] for part in ['drive','steering','progress']]
        else: weights[key]=flat[key]
    cfg['weights']=weights
    return cfg
