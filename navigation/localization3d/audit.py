"""Independent GT region audit and three-pose localization evaluation."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import shapely
from shapely.geometry import shape,LineString
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

HERE=Path(__file__).resolve().parent
BASE=HERE/'baseline'
# Avoid import-name collision with this independent audit.
import importlib.util
spec=importlib.util.spec_from_file_location('baseline_geometry_audit',HERE/'baseline_geometry.py')
geometry=importlib.util.module_from_spec(spec);spec.loader.exec_module(geometry)


def main(args):
    run=args.run.resolve()
    summary=json.loads((run/'summary.json').read_text())
    provenance=json.loads((run/'provenance.json').read_text())
    assert summary['completed_lap']
    for name,h in provenance['world']['input_sha256'].items():
        assert hashlib.sha256((HERE/'inputs'/name).read_bytes()).hexdigest()==h,name
    for name,h in provenance['experiment_sha256'].items():
        assert hashlib.sha256((run/'source'/name).read_bytes()).hexdigest()==h,name
    data=np.load(run/'loop.npz')
    t=data['time'];gt=data['gt'];odo=data['odometry'];est=data['estimate'];dense=data['dense']
    assert (np.diff(t)>0).all()
    # Controller inputs must be the same-time estimated poses, not copied GT.
    fb=data['feedback']
    np.testing.assert_allclose(fb[:,0],t[:len(fb)],atol=1e-10)
    np.testing.assert_allclose(fb[:,1:3],est[:len(fb),:2,3],atol=1e-10)
    assert not np.array_equal(fb[:,1:3],gt[:len(fb),:2,3])
    features=json.loads((HERE/'inputs/zones_map_xy.geojson').read_text())['features']
    zones={f['properties']['zone']:shape(f['geometry']) for f in features}
    hard=zones['preferred'].union(zones['soft'])
    xy=np.vstack([gt[0,:2,3],dense[:,1:3]])
    outside=int((~shapely.covers(hard,shapely.points(xy))).sum())
    covered=bool(hard.covers(LineString(xy)))
    with np.load(HERE/'inputs/area_layers.npz') as d:
        distance=geometry.GridDistance(d['candidate'],d['origin'],float(d['resolution']))
        radius=float(d['body_circle_radius'])
    chord=np.linalg.norm(np.diff(xy,axis=0),axis=1)
    reserve=distance((xy[:-1]+xy[1:])/2)-chord/2-radius
    error=np.linalg.norm(est[:,:3,3]-gt[:,:3,3],axis=1)
    rawerror=np.linalg.norm(odo[:,:3,3]-gt[:,:3,3],axis=1)
    attitude=Rotation.from_matrix(est[:,:3,:3]@gt[:,:3,:3].transpose(0,2,1)).magnitude()
    scans=json.loads((run/'scans.json').read_text())
    sample=np.load(sorted((run/'scans').glob('*.npz'))[0])['points']
    assert np.linalg.matrix_rank(sample-sample.mean(axis=0))==3
    assert np.ptp(sample[:,2])>.5
    report=dict(status='PASS' if outside==0 and covered and reserve.min()>=0 else 'FAIL',run=str(run),
        completed_lap=True,simulated_time_s=summary['simulation_time_s'],controller_steps=summary['steps'],
        estimated_state_feedback_verified=True,full_xyz_sensor_and_six_dof_estimation=True,
        estimated_position_rms_m=float(np.sqrt(np.mean(error**2))),estimated_position_p95_m=float(np.percentile(error,95)),
        estimated_position_max_m=float(error.max()),estimated_attitude_rms_deg=float(np.rad2deg(np.sqrt(np.mean(attitude**2)))),
        raw_odometry_final_error_m=float(rawerror[-1]),raw_odometry_rms_m=float(np.sqrt(np.mean(rawerror**2))),
        accepted_scans=sum(s['accepted'] for s in scans),total_scans=len(scans),
        original_hard_outside_gt_samples=outside,entire_gt_recorded_polyline_inside_original_hard=covered,
        minimum_gt_chord_grid_reserve_m=float(reserve.min()),
        registration_mean_ms=float(np.mean([s['registration_s'] for s in scans])*1000),
        registration_max_ms=float(max(s['registration_s'] for s in scans)*1000),
        controller_cycle_mean_ms=summary['controller_cycle_mean_ms'],controller_cycle_max_ms=summary['controller_cycle_max_ms'],
        overall_cycle_mean_ms=summary['overall_cycle_mean_ms'],overall_cycle_max_ms=float(data['cycle_s'].max()*1000),
        machine=provenance['machine'],
        scope='Numerical3Dsensorclosedloop; ideal known-voxel world and map, genericinstantaneousscan, simulatednoise/bias; NOT measuredMID360accuracy or realtime/vehiclevalidation.')
    out=HERE/'results';(out/'audit.json').write_text(json.dumps(report,indent=2)+'\n')
    plot(out,t,gt,odo,est,error,rawerror,zones,report)
    print(json.dumps(report,indent=2))
    if report['status']!='PASS':raise SystemExit(1)


def plot(out,t,gt,odo,est,error,rawerror,zones,report):
    font=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font));plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name()
    fig,axs=plt.subplots(1,2,figsize=(13,6))
    geometry.paint(axs[0],zones['preferred'].union(zones['soft']),'#ffd69d')
    geometry.paint(axs[0],zones['preferred'],'#e1efdd')
    for points,color,label in [(gt,'#2467e8','GT 정답'),(odo,'#ed8c16','오도메트리'),(est,'#168848','3D 스캔매칭')]:
        axs[0].plot(points[:,0,3],points[:,1,3],color=color,label=label,lw=1.2)
        axs[0].plot(points[-1,0,3],points[-1,1,3],'o',ms=9,color=color,fillstyle='none',mew=2)
    axs[0].set_aspect('equal');axs[0].set_xlabel('지도 x [m]');axs[0].set_ylabel('지도 y [m]');axs[0].legend()
    axs[0].set_title('센서 추정 상태로 MPCC 한 바퀴')
    axs[1].semilogy(t,np.maximum(rawerror,1e-5),color='#ed8c16',label='오도메트리 오차')
    axs[1].semilogy(t,np.maximum(error,1e-5),color='#168848',label='3D 정합 추정 오차')
    axs[1].set_xlabel('가상 시간 [s]');axs[1].set_ylabel('GT 대비 3차원 위치 오차 [m]');axs[1].legend()
    for ax in axs:ax.grid(alpha=.2)
    fig.suptitle('3D 가상 센서·6자유도 ICP 기준 실험 — 실제 MID-360 정확도 검증 전',fontsize=13)
    fig.tight_layout();fig.savefig(out/'three_pose_comparison.png',dpi=160);plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True)
    main(parser.parse_args())
