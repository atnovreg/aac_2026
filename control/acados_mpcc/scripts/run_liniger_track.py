#!/usr/bin/env python3
"""Prepare an isolated copy of the existing Liniger MATLAB baseline and run it."""
import argparse
import datetime
import difflib
import json
from pathlib import Path
import re
import shutil
import subprocess


def quote(value):return str(value).replace("'","''")


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--baseline',type=Path,default=Path('/home/irdl4/Documents/01_Research/Personal/01_MPCC/mpcc_reference/alexliniger_MPCC'))
    p.add_argument('--track',type=Path,default=Path('tracks/indoor_final'))
    p.add_argument('--matlab',default='/home/irdl4/Applications/MATLAB/R2026a/bin/matlab')
    p.add_argument('--steps',type=int,default=2000)
    p.add_argument('--qp-iterations',type=int,default=100)
    p.add_argument('--progress-speed',type=float,default=None)
    p.add_argument('--prepare-only',action='store_true')
    args=p.parse_args();track=args.track.resolve()
    validation=json.loads((track/'validation.json').read_text())
    if not validation['geometry_valid']:p.error('Track geometry validation failed')
    if not (args.baseline/'Matlab'/'simulation.m').is_file():p.error('Liniger MATLAB baseline not found')
    run=track/'simulation'/datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    source=run/'Matlab';shutil.copytree(args.baseline/'Matlab',source)
    shutil.copy2(track/'track_liniger.mat',source/'Tracks'/'track_indoor.mat')
    qp=source/'QuadProgInterface.m'
    qp_original=qp.read_text()
    qp_modified=qp_original.replace("'MaxIterations',100",f"'MaxIterations',{args.qp_iterations}").replace('info.QPtime = QPtime;','info.QPtime = QPtime;\ninfo.raw_exitflag = exitflag;')
    qp.write_text(qp_modified)
    original=(source/'simulation.m').read_text()
    code=original.replace("MPC_vars.interface = 'hpipm';","MPC_vars.interface = 'quadprog';")
    code=code.replace('load Tracks/track2.mat','load Tracks/track_indoor.mat')
    code=code.replace('simN = 500;',f'simN = {args.steps};').replace('plotOn = 1;','plotOn = 0;').replace('n_cars = 5;','n_cars = 1;')
    code=code.replace('tl = traj.ppy.breaks(end);', '''tl = traj.ppy.breaks(end);
% Adapt only global coordinate/progress bounds to this metric track.
MPC_vars.bounds(1,:) = [min(track2.outer(1,:))-2,max(track2.outer(1,:))+2]*MPC_vars.Tx(1,1);
MPC_vars.bounds(2,:) = [min(track2.outer(2,:))-2,max(track2.outer(2,:))+2]*MPC_vars.Tx(2,2);
MPC_vars.bounds(7,:) = [0,tl+5]*MPC_vars.Tx(7,7);
''')
    if args.progress_speed is not None:
        if args.progress_speed<=0:p.error('--progress-speed must be positive')
        code=code.replace('MPC_vars.bounds(7,:) = [0,tl+5]*MPC_vars.Tx(7,7);',f'MPC_vars.bounds(7,:) = [0,tl+5]*MPC_vars.Tx(7,7);\nMPC_vars.bounds(10,2) = {args.progress_speed}*MPC_vars.Tu(3,3);')
    code=code.replace('qpTime_log = zeros(1,simN);','qpTime_log = zeros(1,simN);\nactual_log = zeros(nx,simN);\nprogress_log = zeros(1,simN);\ntravelled_progress = 0;\nprevious_theta = theta;')
    code=re.sub(r'(\[[^\n]+\] = optimizer_mpcc\([^\n]+;)',r"\1\n    if exitflag ~= 0, save('../failure.mat','i','x','u','x0','MPC_vars','TrackMPC','info','actual_log','progress_log','qpTime_log','X_log','U_log','uprev'); fprintf('Failure at i=%d, raw QP status=%d\\n',i,info.raw_exitflag); error('MPCC:QPFailure','QP solve failed; no successful lap claimed'); end",code)
    code=code.replace('    uprev = u(:,1);','''    uprev = u(:,1);
    actual_log(:,i) = x0;
    delta_progress = mod(theta-previous_theta+tl/2,tl)-tl/2;
    travelled_progress = travelled_progress + delta_progress;
    previous_theta = theta;
    progress_log(i) = travelled_progress;
    if mod(i,100)==0, fprintf('Step %d: progress %.2f / %.2f m, vx %.2f m/s\\n',i,travelled_progress,tl,x0(4)); end
''')
    code=code.replace('    B_log(:,i) = reshape(b,N*4,1);','''    B_log(:,i) = reshape(b,N*4,1);
    if travelled_progress >= tl
        simN=i;
        X_log=X_log(:,1:i); U_log=U_log(:,1:i); B_log=B_log(:,1:i);
        actual_log=actual_log(:,1:i); progress_log=progress_log(1:i); qpTime_log=qpTime_log(1:i);
        break;
    end
''')
    # Replace upstream plotting/statistics tail, which assumes >=2 lap wraps.
    code=code[:code.index('PlotLog( X_log')]
    output=quote(run)
    code+=f'''
save('{output}/simulation.mat','actual_log','X_log','U_log','B_log','qpTime_log','progress_log','track','track2','traj','tl','Ts','simN','MPC_vars','ModelParams');
summary=struct('completed_lap',travelled_progress>=tl,'track_length_m',tl,'progress_m',travelled_progress,'steps',simN,'simulation_time_s',simN*Ts,'mean_qp_time_s',mean(qpTime_log),'max_qp_time_s',max(qpTime_log),'model','Liniger ORCA 1:43','solver','quadprog','vehicle_validation',false);
fid=fopen('{output}/summary.json','w');fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));fclose(fid);
fig=figure('Visible','off');plot(track2.inner(1,:),track2.inner(2,:),'g',track2.outer(1,:),track2.outer(2,:),'g');hold on;plot(track.center(1,:),track.center(2,:),'b--');plot(actual_log(1,:),actual_log(2,:),'r');axis equal;grid on;xlabel('x [m]');ylabel('y [m]');title('Liniger MPCC on measured indoor track (ORCA 1:43 model)');legend('Left limit','Right limit','Centerline','Simulated vehicle');exportgraphics(fig,'{output}/simulation.png',Resolution=160);
assert(summary.completed_lap,'No full lap within step budget');
'''
    (source/'simulation.m').write_text(code)
    # Fix track preprocessor's uniform-width assumption for paired variable widths.
    border=source/'borderAdjustment.m';before=border.read_text()
    after=before.replace('c =  WidthCar/widthTrack;','c = WidthCar/norm([x2-x1,y2-y1]);').replace('d = -WidthCar/widthTrack;','d = -WidthCar/norm([x2-x1,y2-y1]);')
    border.write_text(after)
    diff=''.join(difflib.unified_diff(original.splitlines(True),code.splitlines(True),fromfile='upstream/simulation.m',tofile='run/simulation.m'))
    diff+=''.join(difflib.unified_diff(qp_original.splitlines(True),qp_modified.splitlines(True),fromfile='upstream/QuadProgInterface.m',tofile='run/QuadProgInterface.m'))
    diff+=''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='upstream/borderAdjustment.m',tofile='run/borderAdjustment.m'))
    (run/'integration.patch').write_text(diff)
    revision=subprocess.check_output(['git','-C',str(args.baseline),'rev-parse','HEAD'],text=True).strip()
    (run/'provenance.json').write_text(json.dumps({'baseline':str(args.baseline),'commit':revision,'track':str(track),'changes':['track input','quadprog solver','single vehicle','global XY/progress bounds','per-sample boundary shrink','logging and one-lap stop'],'model':'unmodified ORCA 1:43','qp_max_iterations':args.qp_iterations,'progress_speed_limit_m_s':args.progress_speed},indent=2)+'\n')
    print(run,flush=True)
    if not args.prepare_only:
        with (run/'matlab.log').open('w') as log:
            result=subprocess.run([args.matlab,'-batch',"set(groot,'defaultFigureVisible','off'); simulation"],cwd=source,stdout=log,stderr=subprocess.STDOUT)
        if result.returncode:
            failure={'completed_lap':False,'status':'failed','matlab_exit_code':result.returncode,'log':'matlab.log'}
            (run/'failure_summary.json').write_text(json.dumps(failure,indent=2)+'\n')
            raise SystemExit(f'MATLAB failed ({result.returncode}); inspect {run}/matlab.log')
        print((run/'summary.json').read_text())

if __name__=='__main__':main()
