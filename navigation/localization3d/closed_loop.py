"""Sensor-estimated full-3D pose closes the numerical Slash/MPCC loop."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import platform
import os
import shutil
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from voxel_sensor import VoxelWorld,ray_grid
from estimator import StateEstimator,DeadReckoner
from measurements import road_pose,map_velocity,sensor_packet
from feedback import estimated_feedback
from control.acados_mpcc.track import Track
from control.acados_mpcc.controller import default_config
from control.acados_mpcc.simulation import integrate_plant,assess_pose
from control.acados_mpcc.slash_model import load_parameters

HERE=Path(__file__).resolve().parent
from system.f1tenth.workspace import find_workspace
WS=Path(os.environ.get('LOCALIZATION_WS',str(find_workspace(__file__))))
BASE=HERE
sys.path.insert(0,str(BASE))
from solver_margin import GuardedSoftController


def dump(path,value):
    path.write_text(json.dumps(value,indent=2,default=lambda v:v.tolist() if isinstance(v,np.ndarray) else v.item())+'\n')


def main(args):
    folder=HERE/'results'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    folder.mkdir(parents=True);(folder/'scans').mkdir()
    manifest=json.loads((HERE/'inputs/manifest.json').read_text())
    for name,h in manifest['input_sha256'].items():
        assert hashlib.sha256((HERE/'inputs'/name).read_bytes()).hexdigest()==h,name
    cfg=default_config()
    update=yaml.safe_load((WS/'src/aac_2026/system/f1tenth/config/selected_run.yaml').read_text())
    cfg['weights'].update(update.pop('weights',{}));cfg.update(update)
    cfg.update(track=str(HERE/'inputs'),soft_profile=str(HERE/'inputs/soft_profile.npz'),
               soft_weight=.03,solver_geometry_margin_m=.03,
               model='slash_58276_74',model_parameters=load_parameters(),
               speed_max=args.speed+.3,progress_speed_max=args.speed,initial_speed=1.,
               horizon=60,dt=.02,runtime_backend='python',vehicle_width=0.,safety_scaling=1.)
    (folder/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    provenance=dict(scope='3D instantaneous voxel range sensor + noisy wheel/3-axis IMU + six-DoF ICP; estimated-state MPCC; offline virtual plant only',
        machine=platform.machine(),python=platform.python_version(),seed=args.seed,
        sensor_period_s=.1,controller_period_s=.02,world=manifest,
        lidar_model='240azimuth x16elevation [-7,+52]deg,360degazimuth,0.5..30m,rangeStd.01m,dropout.02; notMID360fidelity',
        wheel_scale=1.02,wheel_noise_std_m_s=.01,gyro_bias_rad_s=[0,0,.001],gyro_noise_std_rad_s=.002,
        acceleration_used_for_translation=False,lateral_velocity_source='estimated scan-to-scan displacement with.4s lowpass',
        source_sha256={str(p.relative_to(WS)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [
            WS/'src/aac_2026/control/acados_mpcc/controller.py',WS/'src/aac_2026/control/acados_mpcc/simulation.py',
            WS/'src/aac_2026/control/acados_mpcc/slash_model.py',WS/'src/aac_2026/control/acados_mpcc/config/vehicles/slash_58276_74.yaml']},
        experiment_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*.py')},
        original_adapters_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [BASE/'soft_runtime.py',BASE/'solver_margin.py']})
    dump(folder/'provenance.json',provenance)
    (folder/'source').mkdir()
    for name in provenance['experiment_sha256']:shutil.copy2(HERE/name,folder/'source'/name)
    print('RUN',folder,flush=True)
    world=VoxelWorld.load(HERE/'inputs/map_geometry.npz');world.surfaces()
    plane=np.array(manifest['floor_plane']);track=Track.from_csv(HERE/'inputs/track.csv')
    start_progress=1.
    reference=track.reference(start_progress)
    truth=np.r_[reference['center'],reference['yaw'],1.,0.,0.,start_progress]
    gtpose=road_pose(truth,plane)
    prior=gtpose.copy();prior[:3,3]+=[.2,-.15,.08]
    prior[:3,:3]=Rotation.from_euler('xyz',[.01,-.01,.04]).as_matrix()@prior[:3,:3]
    mount=np.eye(4);mount[:3,3]=[.08,0,.2]
    dump(folder/'initialization.json',dict(coarse_prior=prior,lidar_to_body=mount,
        prior_definition='Declared simulation initial error +.2/-.15/+.08m andsmallRPYoffset; no ongoingGTlookup',
        gyro_frame='3Dbody',accelerometer_frame='3Dbody specific force m/s2',start_progress_m=start_progress))
    rng_lidar=np.random.default_rng(args.seed)
    rng_motion=np.random.default_rng(args.seed+1)
    directions=ray_grid()
    estimate=StateEstimator(world,prior,mount);odometry=DeadReckoner(prior)
    controller=None
    controls=[];feedback=[];dense=[];frames=[];measurements=[];scanlog=[];cycles=[]
    progress=start_progress;estimated_progress=start_progress;previous_yaw=float(reference['yaw'])
    previous_input=np.zeros(3)
    sensor=dict(wheel=1.02,gyro=np.zeros(3),acceleration=gtpose[:3,:3].T@np.array([0,0,9.80665]))
    estimate.velocity_map=estimate.pose[:3,0]*sensor['wheel']
    estimate.wheel=sensor['wheel']
    status='failed';failure=None;started=time.perf_counter()
    try:
        for step in range(args.steps):
            t=step*cfg['dt']
            cycle_start=time.perf_counter()
            gtpose=road_pose(truth,plane)
            if step%5==0:
                scan_start=time.perf_counter()
                packet=world.scan(gtpose@mount,directions,noise_std=.01,dropout=.02,rng=rng_lidar)
                generation_s=time.perf_counter()-scan_start
                match_start=time.perf_counter()
                result=estimate.correct(packet['points'],t)
                match_s=time.perf_counter()-match_start
                scanlog.append(dict(t=t,returns=len(packet['points']),accepted=result.accepted,reason=result.reason,
                    rmse=result.rmse,inlier_ratio=result.inlier_ratio,condition=result.condition,iterations=result.iterations,
                    generation_s=generation_s,registration_s=match_s))
                if step%50==0:
                    np.savez_compressed(folder/'scans'/f'{step:06d}.npz',points=packet['points'].astype(np.float32),
                        ranges=packet['ranges'].astype(np.float32),valid=packet['valid'],timestamp=t)
                if step==0 and not result.accepted:raise RuntimeError('Initial3Dregistration rejected: '+result.reason)
            frames.append((t,gtpose.copy(),odometry.pose.copy(),estimate.pose.copy()))
            state=estimated_feedback(estimate,track,estimated_progress,previous_yaw,t)
            estimated_progress,previous_yaw=state[6],state[2]
            if controller is None:controller=GuardedSoftController(track,cfg,folder/'generated')
            control,diagnostics=controller.solve(state,previous_input)
            feedback.append(np.r_[t,state]);controls.append(np.r_[t,control,diagnostics['cycle_time_s'],diagnostics['solver_time_s']])
            if diagnostics['status']:
                dump(folder/'solver_failure.json',dict(state=state,previous_input=previous_input,diagnostics=diagnostics))
                np.savez_compressed(folder/'failed_prediction.npz',prediction=controller.prediction,increments=controller.increments)
                raise RuntimeError(f"Solver status {diagnostics['status']}")
            solution=integrate_plant(truth,control,cfg,np.zeros(0))
            if not solution.success:raise RuntimeError('Virtual plant integration failed')
            for sample_t,point in zip(solution.sample_t,solution.y.T):
                progress,offset,violation=assess_pose(track,point,progress,0.)
                dense.append(np.r_[t+sample_t,point[:6],progress,offset,violation])
                if violation>cfg['constraint_tolerance']:
                    raise RuntimeError(f'GT runtime corridor violation {violation:.6f}m')
            oldpose=gtpose.copy();oldvelocity=map_velocity(truth,plane)
            truth=solution.y[:,-1].copy();truth[6]=progress
            newpose=road_pose(truth,plane);newvelocity=map_velocity(truth,plane)
            sensor=sensor_packet(oldpose,newpose,oldvelocity,newvelocity,cfg['dt'],rng_motion)
            measurements.append(np.r_[t+cfg['dt'],sensor['wheel'],sensor['gyro'],sensor['acceleration']])
            # Only these raw sensor quantities cross into either estimator.
            estimate.predict(sensor['wheel'],sensor['gyro'],cfg['dt'])
            odometry.predict(sensor['wheel'],sensor['gyro'],cfg['dt'])
            previous_input=control
            cycles.append(time.perf_counter()-cycle_start)
            if step%250==0:
                error=np.linalg.norm(estimate.pose[:3,3]-newpose[:3,3])
                print(f'Step{step}: progress{progress-start_progress:.2f}/{track.length:.2f}m, estimateError{error:.4f}m, returns{scanlog[-1]["returns"]}',flush=True)
            if progress>=start_progress+track.length:
                frames.append((t+cfg['dt'],newpose.copy(),odometry.pose.copy(),estimate.pose.copy()))
                status='completed';break
        if status!='completed':failure=f'No lap within {args.steps} steps'
    except Exception as error:
        failure=f'{type(error).__name__}: {error}'
        print('STOP',failure,flush=True)
    times=np.array([x[0] for x in frames])
    gt=np.array([x[1] for x in frames]);odo=np.array([x[2] for x in frames]);est=np.array([x[3] for x in frames])
    np.savez_compressed(folder/'loop.npz',time=times,gt=gt,odometry=odo,estimate=est,
        controls=np.array(controls),feedback=np.array(feedback),dense=np.array(dense),measurements=np.array(measurements),cycle_s=np.array(cycles))
    dump(folder/'scans.json',scanlog)
    errors=np.linalg.norm(est[:,:3,3]-gt[:,:3,3],axis=1) if len(times) else np.array([np.inf])
    odoerrors=np.linalg.norm(odo[:,:3,3]-gt[:,:3,3],axis=1) if len(times) else np.array([np.inf])
    summary=dict(status=status,failure=failure,completed_lap=status=='completed',
        simulation_time_s=float(dense[-1][0]) if len(dense) else 0.,steps=len(controls),
        physical_progress_m=float(progress-start_progress),track_length_m=track.length,
        position_error_rms_m=float(np.sqrt(np.mean(errors**2))),position_error_max_m=float(errors.max()),
        raw_odometry_error_final_m=float(odoerrors[-1]),scans=len(scanlog),accepted_scans=sum(x['accepted'] for x in scanlog),
        gt_max_runtime_corridor_violation_m=float(np.array(dense)[:,9].max()) if len(dense) else None,
        wall_time_s=time.perf_counter()-started,scope=provenance['scope'],
        controller_cycle_mean_ms=float(np.array(controls)[:,4].mean()*1000) if len(controls) else None,
        controller_cycle_max_ms=float(np.array(controls)[:,4].max()*1000) if len(controls) else None,
        overall_cycle_mean_ms=float(np.mean(cycles)*1000) if len(cycles) else None,
        sensor_motion_model='Instantaneous3Dscan; gyro/wheelprediction; recordedaccelerometer not integrated',
        ground_truth_used_by_controller=False)
    dump(folder/'summary.json',summary)
    dump(HERE/'results/latest.json',dict(directory=str(folder.relative_to(HERE)),summary=summary))
    print(json.dumps(summary,indent=2),flush=True)
    return 0 if status=='completed' or (args.probe and failure==f'No lap within {args.steps} steps') else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steps',type=int,default=10000)
    parser.add_argument('--speed',type=float,default=1.5)
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--probe',action='store_true')
    raise SystemExit(main(parser.parse_args()))
