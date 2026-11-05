"""Independent ODE plant and auditable physical lap/constraint measurements."""
from datetime import datetime, timezone
import hashlib
import gc
import json
from pathlib import Path
from system.f1tenth.workspace import find_workspace
import platform
import time
from types import SimpleNamespace

import numpy as np
from scipy.integrate import solve_ivp
import yaml

from .controller import Controller, state_bounds
from .model import INPUT_SCALE, configured_rhs, increment_limits, model_label
from .track import Track
from .actuator_model import layout as actuator_layout, response_coefficients, TICK


def integrate_plant(state, control, config, actuator_state=None):
    """Independent RK45 plant; delayed response uses 10ms held interval means."""
    dimensions = actuator_layout(config)
    if dimensions is None:
        times = np.linspace(0., config['dt'], 6)[1:]
        result = solve_ivp(lambda _, x: configured_rhs(x, control, config),
                           (0., config['dt']), state, rtol=1e-9, atol=1e-11, t_eval=times)
        result.sample_t = times
        result.actuator_states = np.zeros((len(times), 0))
        return result
    drive_n, steer_n, nx = dimensions
    memory = np.asarray(actuator_state, dtype=float).copy()
    if memory.shape != (nx-10,) or not np.isfinite(memory).all():
        raise ValueError('Delayed plant requires finite current actuator state')
    p = config['actuator_prediction']
    values, times, memories = [], [], []
    state = np.asarray(state).copy()
    for tick in range(round(config['dt']/TICK)):
        applied = np.asarray(control).copy()
        for j, (count, start, tau) in enumerate(((drive_n, 2, p['drive_tau_s']),
                                               (steer_n, 2+drive_n, p['steering_tau_s']))):
            target = memory[start] if count else control[j]
            endpoint_fraction, mean_fraction = response_coefficients(tau)
            applied[j] = memory[j]+(target-memory[j])*mean_fraction
            memory[j] += (target-memory[j])*endpoint_fraction
            if count:
                memory[start:start+count-1] = memory[start+1:start+count]
                memory[start+count-1] = control[j]
        local_times = np.linspace(0., TICK, 6)[1:]
        result = solve_ivp(lambda _, x: configured_rhs(x, applied, config),
                           (0., TICK), state, rtol=1e-9, atol=1e-11, t_eval=local_times)
        if not result.success:
            return result
        state = result.y[:, -1]
        values.extend(result.y.T)
        times.extend(tick*TICK+local_times)
        memories.extend([memory.copy() for _ in local_times])
    return SimpleNamespace(success=True, message='success', y=np.array(values).T,
                           sample_t=np.array(times), actuator_states=np.array(memories))


def assess_pose(track, state, previous_progress, clearance):
    progress = track.project(state[:2], previous_progress)
    reference = track.reference(progress)
    tangent = reference['tangent']
    offset = np.dot(state[:2]-reference['center'], [-tangent[1], tangent[0]])
    violation = max(offset-reference['width_left']+clearance,
                    -offset-reference['width_right']+clearance, 0.)
    return float(progress), float(offset), float(violation)


def input_increment_violation(control, previous_input, config=None):
    increment = (np.asarray(control)-np.asarray(previous_input)) / INPUT_SCALE
    return max(float(np.max(np.abs(increment)-increment_limits(config or {}))), 0.)


def _json_safe(value):
    """Keep diagnostic JSON readable even when numerical failure produces NaNs."""
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def _save_json(path, value):
    path.write_text(json.dumps(_json_safe(value), indent=2, allow_nan=False)+'\n')


def _provenance(track_path, config):
    root = find_workspace(__file__)
    source = Path(__file__).parent
    return {
        'model': model_label(config),
        'baseline_commit': '207a38999d83bc038213813c3928523fc39610d9',
        'acados': json.loads((root/'src/aac_2026/control/acados_mpcc/config/acados_versions.json').read_text()),
        'track': str(track_path.resolve()),
        'track_sha256': hashlib.sha256((track_path/'track.csv').read_bytes()).hexdigest(),
        'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in sorted(list(source.glob('*.py')) + list(source.glob('*.c')))},
        'platform': platform.platform(),
        'machine': platform.machine(),
        'python': platform.python_version(),
        'method_changes_from_matlab': [
            'Nonlinear RK4 prediction replaces frozen affine matrix-exponential prediction',
            f"acados {config['solver']} with {config['hessian']} Hessian replaces damped sequential MATLAB QPs",
            'Periodic track interpolation replaces MATLAB spline implementation',
            'Track strips refreshed once per control cycle; held fixed within each SQP solve',
            'Same numeric and symbolic dynamics; model-specific low-speed treatment documented in vehicle specification',
            'Progress state normalization uses track length; physical units unchanged',
            ('Independent high-accuracy RK45 plant with 10ms held interval-mean delayed inputs'
             if config.get('actuator_prediction') is not None else
             'Independent high-accuracy RK45 plant; original ode45 used default tolerances'),
        ],
        'solver': config['solver'],
        'runtime_backend': config.get('runtime_backend', 'python'),
        'vehicle_validation': False,
    }


class CancelledError(RuntimeError):
    """A cooperative stop that preserves the partial simulation artifacts."""


def run(config, observer=None, should_cancel=None):
    """Run the plant, optionally reporting copied observations between cycles.

    Observer and cancellation callbacks are outside measured control/simulation
    cycles. Observation failures stop the run and remain in its saved summary.
    """
    config = dict(config)
    if config.get('model') == 'slash_58276_74':
        from .slash_model import SPEC, load_parameters
        specification = yaml.safe_load(SPEC.read_text())
        config['model_parameters'] = load_parameters(config.get('model_parameters'))
        config['vehicle_model_revision'] = 'slash_spec_bounded_brake_v2'
        config['vehicle_specification'] = specification
    track_path = Path(config['track']).resolve()
    validation = json.loads((track_path/'validation.json').read_text())
    if not validation['geometry_valid']:
        raise ValueError('Input track has not passed geometry validation')
    track = Track.from_csv(track_path/'track.csv')
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    directory = Path(config['output']).resolve()/timestamp
    directory.mkdir(parents=True)
    (directory/'config.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    _save_json(directory/'provenance.json', _provenance(track_path, config))
    print(f'Run: {directory}', flush=True)
    reference = track.reference(0.)
    state = np.r_[reference['center'], reference['yaw'], config['initial_speed'], 0., 0., 0.]
    previous_input = np.zeros(3)
    dimensions = actuator_layout(config)
    actuator_state = np.zeros(dimensions[2]-10) if dimensions is not None else np.zeros(0)
    progress = 0.
    clearance = config['vehicle_width']*config['safety_scaling']/2
    physical_lower, physical_upper = state_bounds(track, config)
    records, dense_records, solver_records = [], [], []
    failure, completed_lap = None, False
    started = time.perf_counter()
    controller = None
    observed_count, observer_failed = 0, False
    last_observed_prediction = None

    def failure_record(error):
        return {'type': type(error).__name__, 'message': str(error), 'state': state.tolist(),
                'previous_input': previous_input.tolist(), 'progress_m': progress}

    def emit(event):
        nonlocal observer_failed
        try:
            observer(_json_safe(event))
        except Exception:
            observer_failed = True
            raise

    def emit_frame():
        nonlocal observed_count
        last = records[-1]
        emit({'type': 'frame', 'step': len(records), 'time_s': float(last[0]),
              'state': last[1:8].copy(), 'samples': np.array(records[observed_count:]),
              'prediction': last_observed_prediction,
              'progress_m': float(last[7])})
        observed_count = len(records)

    try:
        if observer is not None:
            emit({'type': 'started', 'directory': str(directory), 'track_length_m': track.length})
        if should_cancel is not None and should_cancel():
            raise CancelledError('Simulation cancelled before solver build')
        controller = Controller(track, config, directory/'generated')
        # Solver generation leaves short-lived cyclic Python objects. Collect at
        # startup so the first automatic full collection does not interrupt driving.
        collection_start = time.perf_counter()
        collected = gc.collect()
        _save_json(directory/'startup_memory_collection.json', {
            'collected_objects': collected,
            'wall_time_s': time.perf_counter()-collection_start,
            'automatic_gc_enabled': gc.isenabled(),
            'scope': 'after solver build, before first control sample',
        })
        for step in range(config['steps']):
            if should_cancel is not None and should_cancel():
                raise CancelledError(f'Simulation cancelled before step {step+1}')
            cycle_start = time.perf_counter()
            if dimensions is None:
                control, diagnostics = controller.solve(state, previous_input)
            else:
                control, diagnostics = controller.solve(state, previous_input, actuator_state=actuator_state)
            diagnostics['step'] = step
            solver_records.append(diagnostics)
            if diagnostics['status'] != 0:
                raise RuntimeError(f"acados status {diagnostics['status']} at step {step}; residuals {diagnostics['residuals']}")
            if not np.isfinite(control).all():
                raise RuntimeError(f'Nonfinite control at step {step}')
            diagnostics['input_increment_violation'] = input_increment_violation(control, previous_input, config)
            if diagnostics['input_increment_violation'] > config['constraint_tolerance']:
                raise RuntimeError(f'Normalized input increment constraint violated at step {step}')
            # Full nonlinear plant, separately integrated from OCP discrete RK4.
            dt = config['dt']
            solution = integrate_plant(state, control, config, actuator_state)
            if not solution.success or not np.isfinite(solution.y).all():
                raise RuntimeError(f'Plant integration failed at step {step}: {solution.message}')
            max_violation = 0.
            for j, plant_state in enumerate(solution.y.T):
                next_progress, offset, violation = assess_pose(track, plant_state, progress, clearance)
                if abs(next_progress-progress) > 1.:
                    raise RuntimeError('Physical projection discontinuity exceeds 1m within 4ms')
                progress = next_progress
                physical = np.r_[plant_state, control, solution.actuator_states[j]]
                # Virtual state is replaced by actual projection, as in baseline.
                physical[6] = progress
                bound_violation = max(float(np.max(physical_lower-physical)), float(np.max(physical-physical_upper)), 0.)
                max_violation = max(max_violation, violation, bound_violation)
                dense_records.append(np.r_[step*dt+solution.sample_t[j], plant_state[:6], progress, offset, violation, bound_violation])
            state = solution.y[:, -1]
            actuator_state = solution.actuator_states[-1].copy()
            state[6] = progress
            previous_input = control
            record = np.r_[(step+1)*dt, state, control, offset, max_violation,
                           diagnostics['solver_time_s'], diagnostics['cycle_time_s'], time.perf_counter()-cycle_start]
            records.append(record)
            if observer is not None:
                # Keep the last completed step's prediction: a subsequent failed
                # solve may overwrite Controller.prediction with invalid values.
                last_observed_prediction = (controller.prediction[:, :2].copy()
                                            if controller.prediction is not None else [])
            if max_violation > config['constraint_tolerance']:
                raise RuntimeError(f'Physical constraint violation {max_violation:.6g} at step {step}')
            if observer is not None and (step == 0 or (step+1) % 10 == 0):
                emit_frame()
            if step % 100 == 0:
                print(f'Step {step+1}: {progress:.2f}/{track.length:.2f}m, vx={state[3]:.2f}, '
                      f'SQP={diagnostics["sqp_iterations"]}, solver={diagnostics["solver_time_s"]*1000:.2f}ms', flush=True)
            if progress >= track.length:
                completed_lap = True
                break
        if not completed_lap:
            raise RuntimeError(f'No physical full lap within {config["steps"]} steps')
    except Exception as error:
        failure = failure_record(error)
        print(f'FAILED: {error}', flush=True)
        if controller is not None and controller.prediction is not None:
            np.savez_compressed(directory/'failed_prediction.npz', prediction=controller.prediction,
                                increments=controller.increments, measured_state=state,
                                previous_input=previous_input)
    if observer is not None and not observer_failed and len(records) > observed_count:
        try:
            emit_frame()
        except Exception as error:
            if failure is None:
                failure = failure_record(error)
            else:
                failure['observer_failure'] = failure_record(error)
    elapsed = time.perf_counter()-started
    rows = np.array(records).reshape((-1, 16))
    dense = np.array(dense_records).reshape((-1, 11))
    np.savetxt(directory/'trajectory.csv', rows, delimiter=',', comments='',
               header='t,x,y,yaw,vx,vy,yaw_rate,progress,' + ('current_fraction' if config.get('model') == 'slash_58276_74' else 'D') + ',steering,vtheta,lateral_offset,constraint_violation,solver_time,controller_cycle_time,simulation_cycle_time')
    np.savez_compressed(directory/'simulation.npz', trajectory=rows, dense=dense,
                        last_state=state, last_input=previous_input)
    # Preserve all numeric diagnostics, including NaN/Inf, without pickle.
    # JSON uses null for nonfinite values throughout the nested structure.
    raw_diagnostics = {name: np.asarray([item[name] for item in solver_records])
                       for name in ('status', 'step', 'sqp_iterations',
                                    'solver_time_s', 'cycle_time_s')}
    raw_diagnostics['residuals'] = np.asarray(
        [item['residuals'] for item in solver_records], dtype=float).reshape((-1, 4))
    np.savez_compressed(directory/'solver_diagnostics.npz', **raw_diagnostics)
    _save_json(directory/'solver_statistics.json', solver_records)
    summary = {
        'completed_lap': completed_lap and failure is None,
        'status': ('success' if completed_lap and failure is None else
                   'cancelled' if failure and failure['type'] == 'CancelledError' else 'failed'),
        'model': model_label(config), 'solver': config['solver'],
        'track_length_m': track.length, 'physical_progress_m': float(progress),
        'steps': len(rows), 'simulation_time_s': len(rows)*config['dt'],
        'wall_time_including_build_s': elapsed,
        'vehicle_validation': False,
        'actuator_prediction': config.get('actuator_prediction'),
        'actuator_parameters_measured': False,
        'timing_scope': f"{platform.machine()} CPU, {config.get('runtime_backend', 'python')} runtime backend; "
                        'offline simulation controller calculation, excluding plant and ROS I/O',
        'dense_validation_samples': len(dense),
        'max_corridor_violation_m': float(dense[:, 9].max()) if len(dense) else None,
        'max_state_input_bound_violation': float(dense[:, 10].max()) if len(dense) else None,
        'nonzero_solver_status_count': sum(item['status'] != 0 for item in solver_records),
        'max_normalized_input_increment_violation': max((item.get('input_increment_violation', 0.) for item in solver_records), default=0.),
        'failure': failure,
        'rms_lateral_offset_m': float(np.sqrt(np.mean(rows[:, 11]**2))) if len(rows) else None,
        'controller_deadline_misses': int(np.count_nonzero(rows[:, 14] > config['dt'])) if len(rows) else 0,
    }
    if len(rows):
        for label, index in [('solver', 13), ('controller_cycle', 14), ('simulation_cycle', 15)]:
            timings = rows[:, index]
            summary[label+'_time_ms'] = {'mean': float(timings.mean()*1000), 'p95': float(np.percentile(timings, 95)*1000), 'max': float(timings.max()*1000)}
        summary['max_abs_lateral_offset_m'] = float(np.abs(rows[:, 11]).max())
        summary['max_actual_speed_m_s'] = float(rows[:, 4].max())
    # The machine-readable result must survive optional visualization failures.
    _save_json(directory/'summary.json', summary)
    if len(rows):
        try:
            _plot(directory, track, rows, summary)
        except Exception as error:
            summary['plot_error'] = {'type': type(error).__name__, 'message': str(error)}
            _save_json(directory/'summary.json', summary)
            print(f'Plot artifact failed: {error}', flush=True)
    if observer is not None and not observer_failed:
        try:
            emit({'type': 'finished', 'summary': summary, 'directory': str(directory)})
        except Exception as error:
            summary['completed_lap'] = False
            summary['status'] = 'failed'
            if summary['failure'] is None:
                summary['failure'] = failure_record(error)
            else:
                summary['failure']['observer_failure'] = failure_record(error)
            _save_json(directory/'summary.json', summary)
    print(json.dumps(_json_safe(summary), indent=2, allow_nan=False), flush=True)
    return directory, summary


def _plot(directory, track, rows, summary):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    reference = track.reference(np.linspace(0., track.length, 1500))
    center, tangent = reference['center'], reference['tangent']
    normal = np.c_[-tangent[:, 1], tangent[:, 0]]
    left = center + normal*reference['width_left'][:, None]
    right = center - normal*reference['width_right'][:, None]
    figure, axes = plt.subplots(1, 2, figsize=(13, 6), layout='constrained')
    axes[0].plot(left[:, 0], left[:, 1], color='gray', label='Track limits')
    axes[0].plot(right[:, 0], right[:, 1], color='gray')
    axes[0].plot(center[:, 0], center[:, 1], 'b--', lw=.8, label='Centerline')
    axes[0].plot(rows[:, 1], rows[:, 2], 'r', lw=1., label=summary['model'])
    axes[0].set_aspect('equal'); axes[0].set_xlabel('x (m)'); axes[0].set_ylabel('y (m)')
    axes[0].legend(); axes[0].grid(alpha=.2)
    axes[1].plot(rows[:, 0], rows[:, 4], label='Actual vx')
    axes[1].plot(rows[:, 0], rows[:, 10], '--', label='Virtual progress speed')
    axes[1].set_xlabel('Simulation time (s)'); axes[1].set_ylabel('Speed (m/s)')
    axes[1].legend(); axes[1].grid(alpha=.2)
    figure.suptitle(f'acados {summary["solver"]} — {summary["model"]} — {summary["status"]}')
    figure.savefig(directory/'simulation.png', dpi=160)
    plt.close(figure)
