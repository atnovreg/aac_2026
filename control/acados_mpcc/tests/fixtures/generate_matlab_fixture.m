function generate_matlab_fixture(baseline, destination)
% Independent oracle: evaluate the original MATLAB baseline, never a Python port.
% Private baseline helper bodies are copied verbatim to temporary wrappers;
% only the public function name / wrapper entry point changes.
if nargin < 1
    baseline = '/home/irdl4/Documents/01_Research/Personal/01_MPCC/mpcc_reference/alexliniger_MPCC/Matlab';
end
if nargin < 2
    destination = fullfile(fileparts(mfilename('fullpath')), 'matlab_orca_golden.json');
end
addpath(baseline);
temporary = tempname;
mkdir(temporary);
cleanup = onCleanup(@() cleanup_temporary(temporary));

source = fileread(fullfile(baseline, 'SimTimeStep.m'));
first = strfind(source, 'function xdot=fx_bicycle');
assert(isscalar(first), 'Expected exactly one original ODE helper');
ode_source = source(first:end);
ode_source = strrep(ode_source, 'function xdot=fx_bicycle', 'function xdot=original_fx_bicycle');
write_text(fullfile(temporary, 'original_fx_bicycle.m'), ode_source);

source = fileread(fullfile(baseline, 'getMPCmatrices.m'));
first = strfind(source, '% GENERATING Q');
assert(isscalar(first), 'Expected exactly one original cost-helper block');
wrapper = sprintf(['function out=original_cost(traj,mpc,model,x,stage_index)\n' ...
    'out.H=generateH(traj,mpc,model,x,stage_index);\n' ...
    'out.f=generatef(traj,mpc,model,x,stage_index);\n' ...
    '[out.ec,out.el]=getErrors(traj,mod(x(7),traj.ppx.breaks(end)),x(1),x(2));\n' ...
    '[out.grad_ec,out.grad_el]=getErrorGradient(traj,mod(x(7),traj.ppx.breaks(end)),model,x(1),x(2));\n' ...
    'end\n']);
write_text(fullfile(temporary, 'original_cost.m'), [wrapper, source(first:end)]);
addpath(temporary, '-begin');

model = getModelParams(1);
mpc = getMPC_vars('ORCA');
% A smooth exact polynomial reference avoids any Python/MATLAB spline mismatch.
% x_ref(s)=s, y_ref(s)=0.015*s^2 for 0<=s<=60.
traj.ppx = mkpp([0,60], [1,0]);
traj.ppy = mkpp([0,60], [.015,0,0]);
traj.dppx = mkpp([0,60], 1);
traj.dppy = mkpp([0,60], [.03,0]);
traj.ddppx = mkpp([0,60], 0);
traj.ddppy = mkpp([0,60], .03);

states = [1.2,.1,.2,1.0,0,0,1; ...
          5.15,.29,.3,1.8,.08,.7,5; ...
          9.85,1.65,-.8,3.0,-.1,-1.2,10; ...
          20.1,6.15,1.1,2.4,.15,1.4,20];
inputs = [.3,.04,1.3; .5,.11,1.7; .8,-.09,3.0; .4,.18,2.3];
delta_normalized = [.05,.07,.1; -.02,-.05,-.1; .1,.03,.2; -.04,-.06,.05];
samples = struct([]);
for i = 1:size(states,1)
    x = states(i,:)';
    u = inputs(i,:)';
    du = delta_normalized(i,:)';
    z = blkdiag(mpc.Tx,mpc.Tu)*[x;u];
    stage = original_cost(traj,mpc,model,x,1);
    terminal = original_cost(traj,mpc,model,x,mpc.N+1);
    [Ad,Bd,gd] = DiscretizedLinearizedModel(x,u,model,mpc.Ts);
    samples(i).state = x';
    samples(i).input = u';
    samples(i).delta_input_normalized = du';
    samples(i).delta_input_physical = (mpc.invTu*du)';
    samples(i).ode = original_fx_bicycle(0,x,u,model)';
    samples(i).ode45_next_state = SimTimeStep(x,u,mpc.Ts,model);
    samples(i).linearized_discrete_Ad = Ad;
    samples(i).linearized_discrete_Bd = Bd;
    samples(i).linearized_discrete_gd = gd';
    samples(i).e_contour = stage.ec;
    samples(i).e_lag = stage.el;
    samples(i).grad_e_contour = stage.grad_ec;
    samples(i).grad_e_lag = stage.grad_el;
    samples(i).H_stage = stage.H;
    samples(i).f_stage = stage.f';
    samples(i).H_terminal = terminal.H;
    samples(i).f_terminal = terminal.f';
    increment_cost = du'*diag([mpc.rdD,mpc.rdDelta,mpc.rdVtheta])*du;
    input_cost = u'*diag([mpc.rD,mpc.rDelta,mpc.rVtheta])*u-mpc.qVtheta*u(3);
    samples(i).normalized_increment_cost = increment_cost;
    samples(i).input_cost = input_cost;
    samples(i).nonlinear_stage_cost = mpc.qC*stage.ec^2+mpc.qL*stage.el^2+mpc.qOmega*x(6)^2+input_cost+increment_cost;
    samples(i).nonlinear_terminal_cost = mpc.qCNmult*mpc.qC*stage.ec^2+mpc.qL*stage.el^2+mpc.qOmegaNmult*mpc.qOmega*x(6)^2+input_cost;
    samples(i).qp_stage_cost_without_constant = .5*z'*stage.H*z+stage.f'*z+increment_cost;
    samples(i).qp_terminal_cost_without_constant = .5*z'*terminal.H*z+terminal.f'*z;
end

[status,commit] = system(sprintf('git -C "%s" rev-parse HEAD', baseline));
assert(status==0, 'Cannot record upstream revision');
result = struct();
result.source = 'Original alexliniger/MPCC MATLAB code evaluated by MATLAB; helper equations copied verbatim to temporary wrappers';
result.baseline_commit = strtrim(commit);
result.matlab_version = version;
result.model_name = 'ORCA 1:43';
result.state_order = {'x','y','phi','vx','vy','omega','theta'};
result.input_order = {'D','delta','vtheta'};
result.reference = struct('s_interval',[0,60],'x_polynomial_descending',[1,0],'y_polynomial_descending',[.015,0,0]);
result.model_parameters = model;
result.mpc_parameters = mpc;
result.notes = {'Stage/terminal nonlinear costs reconstructed from original getErrors output and original weights; QP costs retain the original constant omission and 1e-12 diagonal regularization.', ...
    'input denotes applied input for ODE, and previous input for the augmented-stage cost.', ...
    'Delta controls are normalized differences Tu*(u_k-u_{k-1}), not physical rates per second.', ...
    'All sample vx are >=1 m/s; baseline linearization low-speed branch is outside this oracle.', ...
    'ode45_next_state uses the unchanged SimTimeStep.m and default MATLAB ODE45 tolerances.'};
result.samples = samples;
write_text(destination, [jsonencode(result,PrettyPrint=true),newline]);
fprintf('Wrote %d original-MATLAB samples to %s\n',numel(samples),destination);
end

function write_text(path,value)
fid=fopen(path,'w');
assert(fid>=0,'Cannot open output');
cleanup=onCleanup(@() fclose(fid));
fprintf(fid,'%s',value);
end

function cleanup_temporary(path)
rmpath(path);
if isfolder(path), rmdir(path,'s'); end
end
