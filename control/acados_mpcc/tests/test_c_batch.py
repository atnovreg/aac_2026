"""The batched C solver path must reproduce the Python acados calls."""
import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml

from control.acados_mpcc.controller import Controller, default_config
from control.acados_mpcc.slash_model import load_parameters
from control.acados_mpcc.track import Track


class CBatchEquivalenceTests(unittest.TestCase):
    def test_two_control_steps_match_python_backend(self):
        updates = yaml.safe_load(Path(
            'results/건지광장/jetson_mpcc/20261002_selected_run.yaml').read_text())
        config = default_config()
        config['weights'].update(updates.pop('weights'))
        config.update(updates)
        config['model_parameters'] = load_parameters(config.get('model_parameters'))
        config['horizon'] = 4
        config['condensing_horizon'] = 4
        track = Track.from_csv(Path(config['track']) / 'track.csv')
        reference = track.reference(0.)
        state = np.r_[reference['center'], reference['yaw'], 1., 0., 0., 0.]
        previous_input = np.zeros(3)

        with tempfile.TemporaryDirectory() as directory:
            python_config = dict(config, runtime_backend='python')
            c_config = dict(config, runtime_backend='c_batch')
            python = Controller(track, python_config, Path(directory) / 'python')
            c_batch = Controller(track, c_config, Path(directory) / 'c_batch')
            self.assertIsNotNone(c_batch.c_batch)
            for _ in range(2):
                expected, expected_diag = python.solve(state, previous_input)
                actual, actual_diag = c_batch.solve(state, previous_input)
                self.assertEqual(expected_diag['status'], actual_diag['status'])
                np.testing.assert_allclose(actual, expected, rtol=1e-7, atol=1e-8)
                np.testing.assert_allclose(c_batch.prediction, python.prediction,
                                           rtol=1e-7, atol=1e-8)
                np.testing.assert_allclose(c_batch.increments, python.increments,
                                           rtol=1e-7, atol=1e-8)
                previous_input = expected


if __name__ == '__main__':
    unittest.main()
