"""Failure diagnostics survive nonfinite solver values and optional plot errors."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from control.acados_mpcc import simulation
from control.acados_mpcc.controller import default_config


class _FailureController:
    def __init__(self, *_):
        self.prediction = np.full((2, 10), np.nan)
        self.increments = np.full((1, 3), np.nan)

    def solve(self, *_):
        return np.zeros(3), {
            'status': 4, 'solver_time_s': float('nan'), 'sqp_iterations': 1,
            'residuals': [float('nan'), float('inf'), 0., 0.],
            'cycle_time_s': float('inf'),
        }


class _SuccessController(_FailureController):
    def solve(self, *_):
        return np.zeros(3), {
            'status': 0, 'solver_time_s': .001, 'sqp_iterations': 1,
            'residuals': [0.] * 4, 'cycle_time_s': .002,
        }


class _ShortTrack:
    """Synthetic threshold for testing artifact handling, not controller dynamics."""
    length = .01

    def reference(self, progress):
        shape = np.asarray(progress).shape
        return {'center': np.zeros(shape+(2,)), 'yaw': np.zeros(shape),
                'tangent': np.broadcast_to([1., 0.], shape+(2,)),
                'width_left': np.ones(shape), 'width_right': np.ones(shape)}

    def project(self, xy, previous_progress):
        return float(xy[0])


class FailureLoggingTests(unittest.TestCase):
    @staticmethod
    def run_quietly(config):
        with redirect_stdout(io.StringIO()):
            return simulation.run(config)

    def test_solver_setup_failure_is_recorded_without_diagnostics(self):
        with tempfile.TemporaryDirectory() as output:
            config = default_config()
            config.update(output=output, steps=1)
            with patch.object(simulation, 'Controller', side_effect=RuntimeError('setup test failure')):
                directory, result = self.run_quietly(config)
            self.assertFalse(result['completed_lap'])
            self.assertEqual(result['failure']['message'], 'setup test failure')
            self.assertTrue((directory/'summary.json').is_file())
            with np.load(directory/'solver_diagnostics.npz') as raw:
                self.assertEqual(raw['residuals'].shape, (0, 4))

    def test_solver_nan_diagnostics_do_not_discard_failure_summary(self):
        with tempfile.TemporaryDirectory() as output:
            config = default_config()
            config.update(output=output, steps=1)
            with patch.object(simulation, 'Controller', _FailureController):
                directory, result = self.run_quietly(config)
            saved = json.loads((directory/'summary.json').read_text())
            self.assertEqual(saved['status'], 'failed')
            self.assertFalse(result['completed_lap'])
            self.assertEqual(saved['nonzero_solver_status_count'], 1)
            diagnostics = json.loads((directory/'solver_statistics.json').read_text())[0]
            self.assertIsNone(diagnostics['solver_time_s'])
            self.assertIsNone(diagnostics['cycle_time_s'])
            self.assertEqual(diagnostics['residuals'], [None, None, 0., 0.])
            with np.load(directory/'solver_diagnostics.npz') as raw:
                self.assertTrue(np.isnan(raw['solver_time_s'][0]))
                self.assertTrue(np.isinf(raw['residuals'][0, 1]))
            with np.load(directory/'failed_prediction.npz') as raw:
                self.assertTrue(np.isnan(raw['prediction']).all())

    def test_plot_failure_preserves_completed_simulation_summary(self):
        with tempfile.TemporaryDirectory() as output:
            config = default_config()
            config.update(output=output, steps=1)
            with patch.object(simulation, 'Controller', _SuccessController), \
                 patch.object(simulation.Track, 'from_csv', return_value=_ShortTrack()), \
                 patch.object(simulation, '_plot', side_effect=RuntimeError('plot test failure')):
                directory, result = self.run_quietly(config)
            saved = json.loads((directory/'summary.json').read_text())
            self.assertTrue(result['completed_lap'])
            self.assertEqual(saved['status'], 'success')
            self.assertEqual(saved['plot_error']['message'], 'plot test failure')

    def test_nested_nonfinite_json_is_sanitized(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'nested.json'
            simulation._save_json(path, {'data': [np.array([np.nan, np.inf]),
                                                  {'value': np.float64(-np.inf)}]})
            self.assertEqual(json.loads(path.read_text()),
                             {'data': [[None, None], {'value': None}]})


if __name__ == '__main__':
    unittest.main()
