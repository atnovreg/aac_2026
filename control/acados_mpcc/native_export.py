"""Generate native MPCC artifacts while keeping Python out of the runtime loop."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path

import casadi as ca
import numpy as np
from scipy.integrate import solve_ivp
import yaml

from .controller import Controller, default_config
from .model import INPUT_SCALE, configured_rhs, configured_rk4, increment_limits
from .slash_model import load_parameters
from .track import Track
from .actuator_model import augmented_step, layout as actuator_layout


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _export_track(track, output):
    spline = track._spline
    knots = np.asarray(spline.t, dtype=float)
    coefficients = np.asarray(spline.c, dtype=float)
    if coefficients.shape != (len(knots) - spline.k - 1, 2):
        raise ValueError('Unexpected periodic spline coefficient shape')
    path = output / 'track_native.txt'
    with path.open('w') as stream:
        stream.write('MPCC_TRACK_V1\n')
        stream.write(f'{track.length:.17g} {spline.k} {len(knots)} '
                     f'{len(coefficients)} {len(track.s)}\n')
        for knot in knots:
            stream.write(f'{knot:.17g}\n')
        for x, y in coefficients:
            stream.write(f'{x:.17g} {y:.17g}\n')
        for progress, left, right in zip(track.s, track.width_left,
                                         track.width_right):
            stream.write(f'{progress:.17g} {left:.17g} {right:.17g}\n')
    return path


def _generate_plant(config, output, dt, suffix):
    state = ca.MX.sym('state', 7)
    command = ca.MX.sym('command', 3)
    next_state = state
    substeps = int(config['prediction_substeps'])
    h = dt / substeps
    for _ in range(substeps):
        k1 = configured_rhs(next_state, command, config, symbolic=True)
        k2 = configured_rhs(next_state + h * k1 / 2, command, config,
                            symbolic=True)
        k3 = configured_rhs(next_state + h * k2 / 2, command, config,
                            symbolic=True)
        k4 = configured_rhs(next_state + h * k3, command, config,
                            symbolic=True)
        next_state = next_state + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6
    function = ca.Function(f'slash_plant_step_{suffix}', [state, command],
                           [next_state])
    previous_dir = Path.cwd()
    try:
        os.chdir(output)
        function.generate(f'plant_step_{suffix}.c', {'with_header': True})
    finally:
        os.chdir(previous_dir)


def _generate_prediction(config, output, dt, suffix):
    dimensions = actuator_layout(config)
    if dimensions is None:
        return
    state = ca.MX.sym('state', dimensions[2])
    command = ca.MX.sym('command', 3)
    result = augmented_step(state, command, config, dt, symbolic=True)
    function = ca.Function(f'slash_prediction_step_{suffix}', [state, command], [result])
    previous_dir = Path.cwd()
    try:
        os.chdir(output)
        function.generate(f'prediction_step_{suffix}.c', {'with_header': True})
    finally:
        os.chdir(previous_dir)


def _write_native_fixtures(track, config, output):
    progress = np.r_[0., 1e-10, -1e-10, track.length - 1e-10,
                     track.length + 1e-10,
                     np.linspace(0., track.length, 101, endpoint=False)]
    reference = track.reference(progress)
    track_path = output / 'track_reference.csv'
    with track_path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('progress', 'center_x', 'center_y', 'tangent_x',
                         'tangent_y', 'width_left', 'width_right'))
        for index, value in enumerate(progress):
            writer.writerow((float(value), *map(float, reference['center'][index]),
                             *map(float, reference['tangent'][index]),
                             float(reference['width_left'][index]),
                             float(reference['width_right'][index])))
    plant_path = output / 'plant_reference.csv'
    with plant_path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow([*(f'x_{i}' for i in range(7)),
                         *(f'u_{i}' for i in range(3)),
                         *(f'next_x_{i}' for i in range(7))])
        for fraction, speed, command in (
            (0., 1., (0., 0., 1.)),
            (.37, 2., (.1, .02, 2.)),
            (.91, 3., (-.2, -.03, 3.)),
        ):
            s = fraction * track.length
            route = track.reference(s)
            state = np.array([*route['center'], route['yaw'], speed,
                              .02, .01, s])
            control = np.array(command)
            next_state = configured_rk4(state, control, dict(config, dt=.01))
            writer.writerow([*map(float, state), *map(float, control),
                             *map(float, next_state)])
    return track_path, plant_path


def _write_replay(controller, track, config, output):
    reference = track.reference(0.)
    state = np.r_[reference['center'], reference['yaw'],
                  config['initial_speed'], 0., 0., 0.]
    previous_input = np.zeros(3)
    memory = np.zeros(len(controller.scale)-10)
    names = ['state_x', 'state_y', 'state_yaw', 'state_vx', 'state_vy',
             'state_yaw_rate', 'state_progress', 'previous_current',
             'previous_steering', 'previous_vtheta', 'command_current',
             'command_steering', 'command_vtheta', 'status']
    names[10:10] = [f'actuator_state_{i}' for i in range(len(memory))]
    limit = int(config.get('native_replay_steps', config['steps']))
    path = output / 'python_replay.csv'
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        for _ in range(limit):
            command, diagnostics = controller.solve(state, previous_input, actuator_state=memory)
            writer.writerow([*map(float, state), *map(float, previous_input), *map(float, memory),
                             *map(float, command), diagnostics['status']])
            if diagnostics['status'] or state[6] >= track.length:
                break
            if len(memory):
                physical = augmented_step(np.r_[state, previous_input, memory], command, config)
                state, memory = physical[:7], physical[10:]
                previous_input = np.asarray(command)
                continue
            solution = solve_ivp(
                lambda _, value: configured_rhs(value, command, config),
                (0., config['dt']), state, rtol=1e-9, atol=1e-11,
            )
            if not solution.success or not np.isfinite(solution.y).all():
                raise RuntimeError('Python reference plant integration failed')
            state = solution.y[:, -1]
            previous_input = np.asarray(command)
    return path


def _write_runtime_params(controller, config, output):
    def line(name, values):
        return name + ' ' + ' '.join(f'{float(value):.17g}' for value in values)
    path = output / 'runtime_params.txt'
    dimensions = actuator_layout(config)
    lines = [
        'MPCC_RUNTIME_V2' if dimensions is not None else 'MPCC_RUNTIME_V1',
        f"horizon {int(config['horizon'])}",
        line('prediction_dt', [config['dt']]),
        line('preview_s', [config['horizon'] * config['dt']]),
        line('state_scale', controller.scale),
        line('input_scale', INPUT_SCALE),
        line('increment_limit', increment_limits(config)),
        line('vehicle_width', [config['vehicle_width']]),
        line('safety_scaling', [config['safety_scaling']]),
        line('initial_speed', [config['initial_speed']]),
        line('speed_max', [config['speed_max']]),
    ]
    if dimensions is not None:
        p = config['actuator_prediction']
        lines.extend((line('actuator_delay_steps', dimensions[:2]),
                      line('actuator_tau_s', [p['drive_tau_s'], p['steering_tau_s']])))
    path.write_text('\n'.join(lines) + '\n')
    return path


def export_native(config: dict, output_dir: Path) -> dict:
    """Build solver and frozen native inputs for this exact route/config."""
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    track_path = Path(config['track']) / 'track.csv'
    track = Track.from_csv(track_path)
    track_artifact = _export_track(track, output)
    _generate_plant(config, output, .01, '10ms')
    _generate_plant(config, output, config['dt'], '20ms')
    _generate_prediction(config, output, .01, '10ms')
    _generate_prediction(config, output, config['dt'], '20ms')
    track_fixture, plant_fixture = _write_native_fixtures(track, config, output)
    controller = Controller(track, config, output / 'generated')
    runtime_params = _write_runtime_params(controller, config, output)
    replay = _write_replay(controller, track, config, output)
    config_artifact = output / 'config.yaml'
    config_artifact.write_text(yaml.safe_dump(config, sort_keys=False))
    sources = (Path(__file__), Path(__file__).with_name('controller.py'),
               Path(__file__).with_name('track.py'),
               Path(__file__).with_name('model.py'),
               Path(__file__).with_name('slash_model.py'),
               Path(__file__).with_name('c_batch_bridge.c'),
               Path(__file__).with_name('actuator_model.py'))
    manifest = {
        'artifact_format': 'MPCC_NATIVE_V1',
        'horizon': int(config['horizon']),
        'state_dimension': len(controller.scale),
        'actuator_prediction': config.get('actuator_prediction'),
        'dt': float(config['dt']),
        'prediction_time_s': float(config['horizon'] * config['dt']),
        'route_length_m': track.length,
        'runtime_backend': config.get('runtime_backend', 'python'),
        'runtime_params_sha256': _sha256(runtime_params),
        'config_sha256': _sha256(config_artifact),
        'track_csv_sha256': _sha256(track_path),
        'track_native_sha256': _sha256(track_artifact),
        'track_reference_sha256': _sha256(track_fixture),
        'plant_reference_sha256': _sha256(plant_fixture),
        'plant_c_sha256': _sha256(output / 'plant_step_10ms.c'),
        'plant_header_sha256': _sha256(output / 'plant_step_10ms.h'),
        'plant_20ms_c_sha256': _sha256(output / 'plant_step_20ms.c'),
        'plant_20ms_header_sha256': _sha256(output / 'plant_step_20ms.h'),
        'solver_library_sha256': _sha256(output / 'generated' /
                                        'libacados_ocp_solver_slash_mpcc.so'),
        'replay_sha256': _sha256(replay),
        'source_sha256': {path.name: _sha256(path) for path in sources},
    }
    if actuator_layout(config) is not None:
        manifest['prediction_source_sha256'] = {
            f'prediction_step_{suffix}.{extension}': _sha256(output/f'prediction_step_{suffix}.{extension}')
            for suffix in ('10ms', '20ms') for extension in ('c', 'h')}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    config = default_config()
    updates = yaml.safe_load(args.config.read_text())
    if 'weights' in updates:
        config['weights'].update(updates.pop('weights'))
    config.update(updates)
    if config.get('model') == 'slash_58276_74':
        config['model_parameters'] = load_parameters(config.get('model_parameters'))
    print(json.dumps(export_native(config, args.output), indent=2))


if __name__ == '__main__':
    main()
