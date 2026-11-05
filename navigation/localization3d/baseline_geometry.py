#!/usr/bin/env python3
"""Audit actual Jetson virtual-plant positions against original zones and grid."""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,animation
from matplotlib.path import Path as MplPath
from matplotlib.patches import PathPatch,Patch,Rectangle
import numpy as np
from scipy.spatial import cKDTree
import shapely
from shapely.geometry import shape,LineString
from shapely.geometry.polygon import orient


def paint(ax,geometry,color):
    parts=list(geometry.geoms) if geometry.geom_type=='MultiPolygon' else [geometry]
    for p in parts:
        if p.is_empty: continue
        p=orient(p,sign=1)
        vertices=[];codes=[]
        for r in [p.exterior,*p.interiors]:
            xy=np.asarray(r.coords);vertices.extend(xy)
            codes.extend([MplPath.MOVETO]+[MplPath.LINETO]*(len(xy)-2)+[MplPath.CLOSEPOLY])
        ax.add_patch(PathPatch(MplPath(vertices,codes),facecolor=color,edgecolor='none'))


class GridDistance:
    def __init__(self,mask,origin,res):
        self.origin,self.res=np.asarray(origin),float(res)
        self.upper=self.origin+np.array(mask.shape[::-1])*self.res
        self.centres=self.origin+(np.argwhere(~mask)[:,::-1]+.5)*self.res
        self.tree=cKDTree(self.centres)

    def __call__(self,points):
        nearest,_=self.tree.query(points)
        groups=self.tree.query_ball_point(points,nearest+self.res/np.sqrt(2)+1e-10)
        result=[]
        for p,indices in zip(points,groups):
            delta=np.maximum(np.abs(self.centres[indices]-p)-self.res/2,0)
            result.append(max(0.,min(np.linalg.norm(delta,axis=1).min(),
                                     np.min(np.r_[p-self.origin,self.upper-p]))))
        return np.asarray(result)


def analytic_check():
    mask=np.ones((5,5),bool);mask[2,2]=False
    dist=GridDistance(mask,[0,0],1.)
    np.testing.assert_allclose(dist(np.array([[2.5,2.5],[1.5,2.5],[1.,1.],[-1,0]])),[0,.5,1,0])


def main(args):
    analytic_check()
    root=Path(__file__).resolve().parent
    payload=args.payload.resolve()
    latest=json.loads((payload/'latest_lap.json').read_text())
    run=payload/latest['relative_directory']
    summary=json.loads((run/'summary.json').read_text())
    experiment=json.loads((run/'experiment.json').read_text())
    for name,h in experiment['input_sha256'].items():
        assert hashlib.sha256((payload/'inputs'/name).read_bytes()).hexdigest()==h,name
    assert experiment['machine']=='aarch64'
    assert summary['completed_lap'] and summary['status']=='success'
    features=json.loads((payload/'inputs/zones_map_xy.geojson').read_text())['features']
    regions={f['properties']['zone']:shape(f['geometry']) for f in features}
    core,soft=regions['preferred'],regions['soft'];hard=core.union(soft)
    a=np.load(run/'simulation.npz')
    dense=a['dense'];rows=a['trajectory']
    track=np.genfromtxt(payload/'inputs/track.csv',delimiter=',',names=True)
    initial=np.array([track['x'][0],track['y'][0]])
    actual=np.vstack([initial,dense[:,1:3]])
    points=shapely.points(actual)
    inside_hard=shapely.covers(hard,points)
    inside_core=shapely.covers(core,points)
    outside=int((~inside_hard).sum())
    line=LineString(actual)
    with np.load(payload/'inputs/area_layers.npz') as d:
        distance=GridDistance(d['candidate'],d['origin'],float(d['resolution']))
        radius=float(d['body_circle_radius'])
    clear=distance(actual)
    midpoint=(actual[:-1]+actual[1:])/2
    chord_len=np.linalg.norm(np.diff(actual,axis=0),axis=1)
    lower=distance(midpoint)-chord_len/2
    reserve=lower-radius
    polygon_line_covered=hard.covers(line)
    soft_length=line.difference(core).length
    report=dict(status='PASS' if outside==0 and polygon_line_covered and reserve.min()>=0 else 'FAIL',
        completed_virtual_lap=True,run_directory=str(run),execution_machine=experiment['machine'],
        simulation_time_s=summary['simulation_time_s'],track_length_m=summary['track_length_m'],
        actual_progress_m=summary['physical_progress_m'],controller_steps=summary['steps'],
        speed_max_m_s=summary['max_actual_speed_m_s'],speed_cap_m_s=2.,
        rms_reference_tracking_error_m=summary['rms_lateral_offset_m'],
        max_abs_reference_tracking_error_m=summary['max_abs_lateral_offset_m'],
        dense_samples=len(actual),original_hard_outside_samples=outside,
        entire_recorded_polyline_inside_original_hard=bool(polygon_line_covered),
        preferred_sample_fraction=float(inside_core.mean()),
        actual_polyline_soft_length_m=float(soft_length),
        actual_polyline_soft_fraction=float(soft_length/line.length),
        minimum_grid_clearance_m=float(clear.min()),
        minimum_recorded_chord_clearance_lower_bound_m=float(lower.min()),
        body_and_provisional_margin_radius_m=radius,
        minimum_chord_reserve_after_body_and_margin_m=float(reserve.min()),
        controller_cycle_time_ms=summary['controller_cycle_time_ms'],
        controller_deadline_misses=summary['controller_deadline_misses'],
        nonzero_solver_status_count=summary['nonzero_solver_status_count'],
        vehicle_validation=False,
        scope='Jetson offline MPCC + virtual ODE plant. Polygon/grid checks apply to recorded poses and connecting chords; not real vehicle, continuous ODE proof or real-time guarantee.')
    out=root/'results';out.mkdir(exist_ok=True)
    (out/'audit.json').write_text(json.dumps(report,indent=2)+'\n')
    draw(out,track,rows,regions,report,args.animation)
    print(json.dumps(report,indent=2))
    if report['status']!='PASS': raise SystemExit(1)


def draw(out,track,rows,regions,report,make_animation):
    font=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name()
        plt.rcParams['axes.unicode_minus']=False
    fig,ax=plt.subplots(figsize=(10,8))
    ax.set_facecolor('#eeeeee')
    paint(ax,regions['preferred'].union(regions['soft']),'#ffc97a')
    paint(ax,regions['preferred'],'#d8efd5')
    ax.plot(track['x'],track['y'],'--',color='#1d60a4',lw=1,label='계획 주행선')
    travelled,=ax.plot(rows[:,1],rows[:,2],color='#c92338',lw=1.1,label='Jetson 모의 주행')
    marker,=ax.plot([],[],'o',color='#c92338',markersize=6)
    ax.set_aspect('equal');ax.grid(alpha=.2)
    ax.set_xlim(track['x'].min()-5,track['x'].max()+5)
    ax.set_ylim(track['y'].min()-5,track['y'].max()+5)
    ax.set_xlabel('지도 x [m]');ax.set_ylabel('지도 y [m]')
    ax.set_title('Jetson acados 가상 차량 · 30cm 소프트 완충 영역\n실차 주행·실시간 성능 검증 전')
    handles,labels=ax.get_legend_handles_labels()
    ax.legend(handles=[Patch(facecolor='#d8efd5'),Patch(facecolor='#ffc97a'),*handles],
              labels=['선호 주행 영역','소프트 완충 영역',*labels],loc='upper right',fontsize=9)
    label=ax.text(.02,.03,f"한 바퀴 {report['simulation_time_s']:.2f}초 · 하드 이탈 {report['original_hard_outside_samples']}개 표본",
                  transform=ax.transAxes,fontsize=10,bbox=dict(facecolor='white',alpha=.85,edgecolor='none'))
    fig.tight_layout();fig.savefig(out/'jetson_simulation.png',dpi=170)
    if make_animation:
        duration=rows[-1,0]
        times=np.linspace(rows[0,0],duration,int(np.ceil(duration/5*10)))
        indices=np.searchsorted(rows[:,0],times)
        def update(index):
            i=int(index)
            travelled.set_data(rows[:i+1,1],rows[:i+1,2])
            marker.set_data([rows[i,1]],[rows[i,2]])
            label.set_text(f'모의 시간 {rows[i,0]:.1f} / {duration:.1f}초 · 5배속 · 빨간 표식은 확대 표시')
            return travelled,marker,label
        ani=animation.FuncAnimation(fig,update,frames=indices,interval=100,blit=False)
        ani.save(out/'jetson_simulation_5x.gif',writer=animation.PillowWriter(fps=10),dpi=90)
    plt.close(fig)


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--payload',type=Path,default=Path(__file__).resolve().parent/'jetson')
    ap.add_argument('--animation',action='store_true')
    main(ap.parse_args())
