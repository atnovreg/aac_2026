"""ORCA MPCC: normalized input increments and sampled-sum costs.

Model/objective port from alexliniger/MPCC, Copyright 2018 ETH Zurich,
Kenneth Kuchera and Alexander Liniger, Apache-2.0. Original formulation is
retained; nonlinear RK4 prediction and acados SQP replace the MATLAB QP loop.
"""
from pathlib import Path
from tools.workspace import find_workspace
import time

import casadi as ca
import numpy as np
import yaml
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

from .model import INPUT_SCALE, configured_rhs, configured_rk4, increment_limits
from .actuator_model import layout as actuator_layout, augmented_step


def default_config():
    path = find_workspace(__file__) / 'config/acados_mpcc.yaml'
    return yaml.safe_load(path.read_text())


def state_bounds(track, config):
    center = track.reference(np.linspace(0, track.length, 500))['center']
    lower = np.r_[center.min(axis=0) - 3, -6*np.pi, 0., -2., -7., 0., -.1, -.35, 0.]
    upper = np.r_[center.max(axis=0) + 3, 6*np.pi, 4., 2., 7., track.length+5, 1., .35, config['progress_speed_max']]
    if config.get('model') == 'slash_58276_74':
        p = config['model_parameters']
        lower[3] = .2
        upper[3] = config.get('speed_max', 4.)
        lower[7] = -1.
        lower[8], upper[8] = -p['steering_limit'], p['steering_limit']
    dimensions = actuator_layout(config)
    if dimensions is not None:
        drive_n, steer_n, _ = dimensions
        limit = config['model_parameters']['steering_limit']
        lower = np.r_[lower, -1., -limit, np.full(drive_n, -1.), np.full(steer_n, -limit)]
        upper = np.r_[upper, 1., limit, np.ones(drive_n), np.full(steer_n, limit)]
    return lower, upper


def build_ocp(track, config, build_dir):
    """Return an unbuilt OCP plus physical units per normalized state."""
    ocp = AcadosOcp()
    model = AcadosModel()
    model.name = 'slash_mpcc' if config.get('model') == 'slash_58276_74' else 'orca_mpcc'
    dimensions = actuator_layout(config)
    nx = dimensions[2] if dimensions is not None else 10
    z = ca.MX.sym('z', nx)
    du = ca.MX.sym('du', 3)
    p = ca.MX.sym('strip', 4)  # center x,y and left normal x,y
    scale = np.array([3., 3., 2*np.pi, 4., 2., 7., track.length, 1., .35, 6.])
    if dimensions is not None:
        drive_n, steer_n, _ = dimensions
        scale = np.r_[scale, 1., .35, np.ones(drive_n), np.full(steer_n, .35)]
        scale[3] = max(4., config['speed_max'])
    physical = z * scale
    current_input = physical[7:10] + du * INPUT_SCALE
    state = physical[:7]
    h = config['dt'] / config['prediction_substeps']
    for _ in range(config['prediction_substeps']):
        k1 = configured_rhs(state, current_input, config, symbolic=True)
        k2 = configured_rhs(state + h/2*k1, current_input, config, symbolic=True)
        k3 = configured_rhs(state + h/2*k2, current_input, config, symbolic=True)
        k4 = configured_rhs(state + h*k3, current_input, config, symbolic=True)
        state = state + h/6*(k1+2*k2+2*k3+k4)
    model.x, model.u, model.p = z, du, p
    model.disc_dyn_expr = (augmented_step(physical, current_input, config, symbolic=True)
                           if dimensions is not None else ca.vertcat(state, current_input)) / scale

    center, tangent = track.symbolic_reference(physical[6])
    delta_xy = physical[:2] - center
    ec = tangent[1]*delta_xy[0] - tangent[0]*delta_xy[1]
    el = -ca.dot(tangent, delta_xy)
    y = ca.vertcat(ec, el, physical[5], physical[7:10])
    model.cost_y_expr = ca.vertcat(y, du)
    model.cost_y_expr_e = y
    model.con_h_expr = ca.dot(p[2:], physical[:2] - p[:2])
    model.con_h_expr_e = model.con_h_expr
    # Initial state is fixed: an additional corridor constraint is redundant.
    model.con_h_expr_0 = ca.MX.zeros(0, 1)
    ocp.model = model
    ocp.name = model.name
    ocp.code_gen_options.code_export_directory = str(Path(build_dir).resolve())
    ocp.parameter_values = np.array([0., 0., 0., 1.])

    weights = config['weights']
    stage_weights = np.r_[weights['contour'], weights['lag'], weights['yaw_rate'], weights['input'], weights['increment']]
    terminal_weights = np.r_[weights['contour']*weights['terminal_contour_multiplier'], weights['lag'], weights['yaw_rate']*weights['terminal_yaw_rate_multiplier'], weights['input']]
    ocp.cost.cost_type = ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.cost.W = np.diag(2*stage_weights)
    ocp.cost.W_e = np.diag(2*terminal_weights)
    ocp.cost.yref = np.zeros(9)
    ocp.cost.yref_e = np.zeros(6)
    # r*v^2 - q*v = r*(v-q/(2*r))^2 - q^2/(4*r).
    progress_target = weights['progress'] / (2*weights['input'][2])
    ocp.cost.yref[5] = ocp.cost.yref_e[5] = progress_target

    lower, upper = state_bounds(track, config)
    ocp.constraints.idxbx = ocp.constraints.idxbx_e = np.arange(nx)
    ocp.constraints.lbx = ocp.constraints.lbx_e = lower / scale
    ocp.constraints.ubx = ocp.constraints.ubx_e = upper / scale
    ocp.constraints.idxbu = np.arange(3)
    ocp.constraints.lbu = -increment_limits(config)
    ocp.constraints.ubu = increment_limits(config)
    ocp.constraints.lh = ocp.constraints.lh_e = np.array([-.5])
    ocp.constraints.uh = ocp.constraints.uh_e = np.array([.5])
    ocp.constraints.x0 = np.r_[0., 0., 0., 1., np.zeros(nx-4)] / scale

    options = ocp.solver_options
    options.N_horizon = config['horizon']
    options.tf = config['horizon'] * config['dt']
    options.cost_scaling = np.ones(config['horizon'] + 1)
    options.integrator_type = 'DISCRETE'
    options.nlp_solver_type = config['solver']
    options.hessian_approx = config['hessian']
    if config['hessian'] == 'EXACT':
        options.regularize_method = 'PROJECT'
    options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    options.qp_solver_cond_N = config['condensing_horizon']
    options.qp_solver_ric_alg = 0
    options.qp_solver_cond_ric_alg = 0
    options.qp_solver_iter_max = config['qp_max_iterations']
    options.qp_solver_tol_stat = config['qp_tolerance']
    options.qp_solver_tol_eq = config['qp_tolerance']
    options.qp_solver_tol_ineq = config['qp_tolerance']
    options.qp_solver_tol_comp = config['qp_tolerance']
    options.qp_solver_warm_start = 1
    options.hpipm_mode = 'ROBUST'
    options.nlp_solver_max_iter = config['nlp_max_iterations']
    options.eval_residual_at_max_iter = True
    options.tol = config['nlp_tolerance']
    options.levenberg_marquardt = config['levenberg_marquardt']
    options.globalization = 'MERIT_BACKTRACKING' if config['solver'] == 'SQP' else 'FIXED_STEP'
    options.globalization_full_step_dual = int(config['full_step_dual'])
    options.print_level = 0
    return ocp, scale


class Controller:
    def __init__(self, track, config, build_dir):
        self.track, self.config = track, config
        self.ocp, self.scale = build_ocp(track, config, build_dir)
        self.solver = AcadosOcpSolver(self.ocp, verbose=False)
        self.runtime_backend = config.get('runtime_backend', 'python')
        if self.runtime_backend not in ('python', 'c_batch'):
            raise ValueError(f"Unknown runtime backend: {self.runtime_backend}")
        self.c_batch = None
        if self.runtime_backend == 'c_batch':
            from .c_batch import CBatch
            self.c_batch = CBatch(self.solver)
        self.n = config['horizon']
        self.prediction = None
        self.increments = None

    def initialize(self, state, previous_input, actuator_state=None):
        """Centerline warm start; measured augmented state remains exact."""
        progress = state[6] + np.arange(self.n+1)*self.config['dt']*state[3]
        reference = self.track.reference(progress)
        yaw = np.unwrap(reference['yaw'])
        yaw += 2*np.pi*round((state[2]-yaw[0])/(2*np.pi))
        self.prediction = np.zeros((self.n+1, len(self.scale)))
        self.prediction[:, :2] = reference['center']
        self.prediction[:, 2] = yaw
        self.prediction[:, 3] = state[3]
        self.prediction[:, 6] = progress
        self.prediction[:, 7:10] = previous_input
        if len(self.scale) > 10:
            if actuator_state is None:
                raise ValueError('Actuator-aware control requires the current actuator state')
            self.prediction[:, 10:] = actuator_state
        self.prediction[0, :10] = np.r_[state, previous_input]
        self.increments = np.zeros((self.n, 3))

    def shift_prediction(self):
        terminal = self.prediction[-1].copy()
        self.prediction[:-1] = self.prediction[1:]
        if self.config.get('actuator_prediction') is not None:
            self.prediction[-1] = augmented_step(terminal, terminal[7:10], self.config)
        else:
            self.prediction[-1, :7] = configured_rk4(terminal[:7], terminal[7:], self.config)
            self.prediction[-1, 7:] = terminal[7:]
        self.increments[:-1] = self.increments[1:]
        self.increments[-1] = 0.

    def solve(self, state, previous_input, shift=True, actuator_state=None):
        start = time.perf_counter()
        profile = self.config.get('profile_timing', False)
        marks = []
        def mark(name):
            if profile:
                marks.append((name, time.perf_counter(), time.thread_time()))
        mark('start')
        if self.prediction is None:
            self.initialize(state, previous_input, actuator_state)
        elif shift:
            self.shift_prediction()
        self.prediction[0, :10] = np.r_[state, previous_input]
        if len(self.scale) > 10:
            memory = np.asarray(actuator_state, dtype=float)
            if memory.shape != (len(self.scale)-10,) or not np.isfinite(memory).all():
                raise ValueError('Invalid current actuator state')
            self.prediction[0, 10:] = memory
        reference = self.track.reference(self.prediction[:, 6])
        clearance = self.config['vehicle_width'] * self.config['safety_scaling'] / 2
        mark('prediction_and_reference')
        if self.c_batch is None:
            for i in range(self.n+1):
                self.solver.set(i, 'x', self.prediction[i] / self.scale)
                tangent = reference['tangent'][i]
                self.solver.set(i, 'p', np.r_[reference['center'][i], -tangent[1], tangent[0]])
                if i:
                    left = reference['width_left'][i] - clearance
                    right = reference['width_right'][i] - clearance
                    if min(left, right) <= 0:
                        raise ValueError('Track is too narrow for configured vehicle clearance')
                    self.solver.constraints_set(i, 'lh', np.array([-right]))
                    self.solver.constraints_set(i, 'uh', np.array([left]))
                if i < self.n:
                    self.solver.set(i, 'u', self.increments[i])
            self.solver.constraints_set(0, 'lbx', self.prediction[0] / self.scale)
            self.solver.constraints_set(0, 'ubx', self.prediction[0] / self.scale)
            mark('set_inputs')
            status = self.solver.solve()
            mark('solver_call')
            self.prediction = np.array([self.solver.get(i, 'x') * self.scale for i in range(self.n+1)])
            self.increments = np.array([self.solver.get(i, 'u') for i in range(self.n)])
            mark('get_outputs')
        else:
            left = reference['width_left'] - clearance
            right = reference['width_right'] - clearance
            if np.minimum(left[1:], right[1:]).min() <= 0:
                raise ValueError('Track is too narrow for configured vehicle clearance')
            tangent = reference['tangent']
            parameters = np.column_stack((reference['center'], -tangent[:, 1], tangent[:, 0]))
            lower_h = -right
            upper_h = left
            status, prediction, increments, c_timings = self.c_batch.step(
                self.prediction / self.scale, self.increments,
                parameters, lower_h, upper_h)
            self.prediction = prediction * self.scale
            self.increments = increments
            mark('c_batch')
        diagnostics = {
            'status': int(status),
            'solver_time_s': float(self.solver.get_stats('time_tot')),
            'sqp_iterations': int(self.solver.get_stats('sqp_iter')),
            'residuals': self.solver.get_residuals(recompute=True).tolist(),
            'cycle_time_s': time.perf_counter()-start,
        }
        if self.c_batch is not None:
            diagnostics['c_batch_phase_timing_s'] = dict(zip(
                ('set_inputs', 'solver_call', 'get_outputs'), map(float, c_timings)))
        mark('diagnostics')
        if profile:
            diagnostics['cycle_started_monotonic_s'] = start
            diagnostics['phase_timing_s'] = {
                name: {'wall': wall-marks[i-1][1], 'thread_cpu': cpu-marks[i-1][2]}
                for i, (name, wall, cpu) in enumerate(marks) if i
            }
        if status:
            diagnostics['iteration_statistics'] = self.solver.get_stats('statistics').tolist()
        return previous_input + INPUT_SCALE*self.increments[0], diagnostics
