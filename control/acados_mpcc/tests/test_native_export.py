"""Native artifacts must preserve the Python track and Slash dynamics."""
import ctypes
import csv
import hashlib
import importlib
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np
import yaml

from control.acados_mpcc.controller import default_config
from control.acados_mpcc.model import configured_rk4, increment_limits
from control.acados_mpcc.slash_model import load_parameters
from control.acados_mpcc.track import Track


class NativeExportTests(unittest.TestCase):
    def test_export_preserves_track_and_ten_millisecond_plant(self):
        module = 'control.acados_mpcc.native_export'
        self.assertIsNotNone(importlib.util.find_spec(module),
                             'native exporter module is missing')
        export_native = importlib.import_module(module).export_native
        updates = yaml.safe_load(Path(
            'results/건지광장/jetson_mpcc/20261002_c_batch_run.yaml').read_text())
        config = default_config()
        config['weights'].update(updates.pop('weights'))
        config.update(updates)
        config['model_parameters'] = load_parameters(config.get('model_parameters'))
        config['horizon'] = 4
        config['condensing_horizon'] = 4
        config['native_replay_steps'] = 2
        track = Track.from_csv(Path(config['track']) / 'track.csv')

        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            manifest = export_native(config, output)
            self.assertEqual(manifest['horizon'], 4)
            self.assertEqual(manifest['prediction_time_s'], 0.08)
            self.assertEqual(manifest['config_sha256'], hashlib.sha256(
                (output / 'config.yaml').read_bytes()).hexdigest())
            self.assertEqual(manifest['plant_header_sha256'], hashlib.sha256(
                (output / 'plant_step_10ms.h').read_bytes()).hexdigest())
            runtime_lines = (output / 'runtime_params.txt').read_text().splitlines()
            self.assertEqual(runtime_lines[0], 'MPCC_RUNTIME_V1')
            parameters = {row.split()[0]: row.split()[1:] for row in runtime_lines[1:]}
            self.assertEqual(int(parameters['horizon'][0]), 4)
            self.assertEqual(float(parameters['prediction_dt'][0]), .02)
            np.testing.assert_allclose([float(v) for v in parameters['increment_limit']],
                                       increment_limits(config), rtol=0, atol=1e-15)
            self.assertEqual(manifest['runtime_params_sha256'], hashlib.sha256(
                (output / 'runtime_params.txt').read_bytes()).hexdigest())
            self.assertTrue((output / 'generated/acados_solver_slash_mpcc.h').is_file())
            self.assertTrue((output / 'generated/libacados_ocp_solver_slash_mpcc.so').is_file())

            lines = (output / 'track_native.txt').read_text().splitlines()
            self.assertEqual(lines[0], 'MPCC_TRACK_V1')
            length, degree, knot_count, coeff_count, width_count = lines[1].split()
            self.assertAlmostEqual(float(length), track.length, places=10)
            self.assertEqual(int(degree), 3)
            self.assertEqual(int(knot_count), int(coeff_count) + 4)
            self.assertEqual(int(width_count), len(track.s))
            self.assertEqual(len(lines), 2 + int(knot_count) + int(coeff_count) + int(width_count))
            self.assertTrue(all(np.isfinite(float(v)) for row in lines[2:]
                                for v in row.split()))

            with (output / 'track_reference.csv').open() as stream:
                track_fixture = list(csv.DictReader(stream))
            self.assertGreaterEqual(len(track_fixture), 105)
            self.assertEqual(float(track_fixture[0]['progress']), 0.)
            for row in track_fixture:
                progress = float(row['progress'])
                reference = track.reference(progress)
                np.testing.assert_allclose(
                    [float(row[key]) for key in ('center_x', 'center_y',
                                                  'tangent_x', 'tangent_y',
                                                  'width_left', 'width_right')],
                    [*reference['center'], *reference['tangent'],
                     reference['width_left'], reference['width_right']],
                    rtol=0, atol=1e-12)

            with (output / 'plant_reference.csv').open() as stream:
                plant_fixture = list(csv.DictReader(stream))
            self.assertGreaterEqual(len(plant_fixture), 3)
            for row in plant_fixture:
                x = np.array([float(row[f'x_{i}']) for i in range(7)])
                u = np.array([float(row[f'u_{i}']) for i in range(3)])
                expected = np.array([float(row[f'next_x_{i}']) for i in range(7)])
                np.testing.assert_allclose(
                    configured_rk4(x, u, dict(config, dt=0.01)), expected,
                    rtol=0, atol=1e-12)

            with (output / 'python_replay.csv').open() as stream:
                replay = list(csv.DictReader(stream))
            self.assertEqual(len(replay), 2)
            self.assertEqual([int(row['status']) for row in replay], [0, 0])
            self.assertIn('state_x', replay[0])
            self.assertIn('command_steering', replay[0])

            library = output / 'libplant_test.so'
            subprocess.run(['cc', '-O2', '-shared', '-fPIC',
                            str(output / 'plant_step_10ms.c'), '-lm', '-o', str(library)],
                           check=True, capture_output=True, text=True)
            function = ctypes.CDLL(str(library)).slash_plant_step_10ms
            pointer = ctypes.POINTER(ctypes.c_double)
            function.argtypes = [ctypes.POINTER(pointer), ctypes.POINTER(pointer),
                                 ctypes.POINTER(ctypes.c_int), pointer, ctypes.c_int]
            function.restype = ctypes.c_int
            state = np.array([0.1, -3.3, 0.03, 1.0, 0.02, 0.01, 0.0], dtype='d')
            command = np.array([0.1, 0.02, 1.0], dtype='d')
            actual = np.zeros(7, dtype='d')
            args = (pointer * 2)(state.ctypes.data_as(pointer),
                                 command.ctypes.data_as(pointer))
            results = (pointer * 1)(actual.ctypes.data_as(pointer))
            work_int = (ctypes.c_int * 256)()
            work_real = (ctypes.c_double * 4096)()
            self.assertEqual(function(args, results, work_int, work_real, 0), 0)
            expected = configured_rk4(state, command, dict(config, dt=0.01))
            np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-10)

            library_20 = output / 'libplant_20ms_test.so'
            subprocess.run(['cc', '-O2', '-shared', '-fPIC',
                            str(output / 'plant_step_20ms.c'), '-lm', '-o', str(library_20)],
                           check=True, capture_output=True, text=True)
            function_20 = ctypes.CDLL(str(library_20)).slash_plant_step_20ms
            function_20.argtypes = function.argtypes
            function_20.restype = ctypes.c_int
            actual.fill(0)
            self.assertEqual(function_20(args, results, work_int, work_real, 0), 0)
            np.testing.assert_allclose(actual, configured_rk4(state, command, config),
                                       rtol=0, atol=1e-10)


if __name__ == '__main__':
    unittest.main()
