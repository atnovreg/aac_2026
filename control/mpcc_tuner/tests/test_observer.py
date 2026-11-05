"""Live observations preserve simulation evidence and cancellation semantics."""
from contextlib import redirect_stdout
import copy
import io
import json
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from tools.acados_mpcc import simulation
from tools.acados_mpcc.controller import default_config


class StraightTrack:
    length = .245

    def reference(self, progress):
        shape = np.asarray(progress).shape
        return {'center': np.zeros(shape+(2,)), 'yaw': np.zeros(shape),
                'tangent': np.broadcast_to([1., 0.], shape+(2,)),
                'width_left': np.ones(shape), 'width_right': np.ones(shape)}

    def project(self, xy, previous_progress):
        return float(xy[0])


class FixedController:
    def __init__(self, *_):
        self.prediction = np.zeros((3, 10))
        self.increments = np.zeros((2, 3))

    def solve(self, state, previous):
        self.prediction[:] = np.r_[state, previous]
        self.prediction[:, 0] += np.arange(3)*.02
        return np.zeros(3), {'status': 0, 'solver_time_s': .001,
                            'sqp_iterations': 1, 'residuals': [0.]*4,
                            'cycle_time_s': .002}


class ObserverTests(unittest.TestCase):
    def run_case(self, output, observer=None, should_cancel=None, controller=FixedController,
                 clock=None, steps=30):
        config = default_config()
        config.update(output=output, steps=steps)
        # Isolate observer behavior from code generation, actual geometry and wall
        # clock jitter; retain the real RK45 plant, logging and validation loop.
        def straight_rhs(state, control, config):
            return np.array([1., 0., 0., 0., 0., 0., 1.])
        with patch.object(simulation, 'Controller', controller), \
             patch.object(simulation.Track, 'from_csv', return_value=StraightTrack()), \
             patch.object(simulation, 'configured_rhs', side_effect=straight_rhs), \
             patch.object(simulation, '_plot'), redirect_stdout(io.StringIO()):
            if clock:
                with patch.object(simulation.time, 'perf_counter', side_effect=lambda: clock[0]):
                    return simulation.run(config, observer=observer, should_cancel=should_cancel)
            return simulation.run(config, observer=observer, should_cancel=should_cancel)

    def test_frames_include_every_completed_sample_once_without_changing_trajectory(self):
        events = []
        with tempfile.TemporaryDirectory() as output:
            plain_dir, plain = self.run_case(output)
            seen_dir, seen = self.run_case(output, observer=lambda event: events.append(copy.deepcopy(event)))
            with np.load(plain_dir/'simulation.npz') as a, np.load(seen_dir/'simulation.npz') as b:
                np.testing.assert_array_equal(a['trajectory'][:, :15], b['trajectory'][:, :15])
                np.testing.assert_array_equal(a['dense'], b['dense'])
                rows = b['trajectory'].copy()
            frames = [event for event in events if event['type'] == 'frame']
            self.assertEqual([frame['step'] for frame in frames], [1, 10, 13])
            np.testing.assert_array_equal(np.concatenate([frame['samples'] for frame in frames]), rows)
            self.assertEqual(events[0]['type'], 'started')
            self.assertEqual(events[-1]['type'], 'finished')
            self.assertEqual(events[-1]['summary'], seen)
            self.assertTrue(plain['completed_lap'] and seen['completed_lap'])
            self.assertEqual(seen['rms_lateral_offset_m'], 0.)
            self.assertEqual(seen['controller_deadline_misses'], 0)
            # Predictions belong to the start of the reported last step.
            self.assertAlmostEqual(frames[-1]['prediction'][0][0], .24)
            self.assertAlmostEqual(frames[-1]['state'][0], .26)

    def test_cancel_flushes_partial_samples_and_saves_cancelled_summary(self):
        events, calls = [], [0]
        class CountController(FixedController):
            def solve(self, state, previous):
                calls[0] += 1
                return super().solve(state, previous)
        with tempfile.TemporaryDirectory() as output:
            directory, result = self.run_case(output, observer=events.append,
                                             should_cancel=lambda: calls[0] >= 4,
                                             controller=CountController)
            self.assertEqual(result['status'], 'cancelled')
            self.assertFalse(result['completed_lap'])
            self.assertEqual(result['failure']['type'], 'CancelledError')
            self.assertEqual(result['steps'], 4)
            self.assertEqual(calls[0], 4)
            self.assertEqual(json.loads((directory/'summary.json').read_text()), result)
            frames = [event for event in events if event['type'] == 'frame']
            self.assertEqual([frame['step'] for frame in frames], [1, 4])
            self.assertEqual(sum(len(frame['samples']) for frame in frames), 4)
            self.assertEqual(events[-1]['type'], 'finished')

    def test_observer_time_is_outside_recorded_control_and_simulation_cycle(self):
        clock = [0.]
        def observer(event):
            clock[0] += 100.
        with tempfile.TemporaryDirectory() as output:
            directory, result = self.run_case(output, observer=observer, clock=clock)
            with np.load(directory/'simulation.npz') as data:
                self.assertTrue(np.all(data['trajectory'][:, 15] == 0.))
                self.assertTrue(np.all(data['trajectory'][:, 14] == .002))
            self.assertTrue(result['completed_lap'])

    def test_frame_callback_failure_preserves_partial_failed_evidence(self):
        def observer(event):
            if event['type'] == 'frame':
                raise OSError('observer storage unavailable')
        with tempfile.TemporaryDirectory() as output:
            directory, result = self.run_case(output, observer=observer)
            self.assertFalse(result['completed_lap'])
            self.assertEqual(result['status'], 'failed')
            self.assertEqual(result['steps'], 1)
            self.assertEqual(result['failure']['message'], 'observer storage unavailable')
            self.assertTrue((directory/'simulation.npz').is_file())
            self.assertEqual(json.loads((directory/'summary.json').read_text()), result)

    def test_finished_callback_failure_cannot_report_success(self):
        def observer(event):
            if event['type'] == 'finished':
                raise OSError('finished event write failed')
        with tempfile.TemporaryDirectory() as output:
            directory, result = self.run_case(output, observer=observer)
            self.assertFalse(result['completed_lap'])
            self.assertEqual(result['status'], 'failed')
            self.assertEqual(result['failure']['message'], 'finished event write failed')
            self.assertEqual(json.loads((directory/'summary.json').read_text()), result)

    def test_failure_before_batch_boundary_flushes_only_completed_predictions(self):
        class FailFifthController(FixedController):
            calls = 0
            def solve(self, state, previous):
                self.calls += 1
                control, diagnostics = super().solve(state, previous)
                if self.calls == 5:
                    diagnostics['status'] = 4
                    self.prediction[:] = np.nan
                return control, diagnostics
        events = []
        with tempfile.TemporaryDirectory() as output:
            _, result = self.run_case(output, observer=events.append, controller=FailFifthController)
            self.assertEqual(result['steps'], 4)
            frames = [event for event in events if event['type'] == 'frame']
            self.assertEqual([frame['step'] for frame in frames], [1, 4])
            self.assertTrue(np.isfinite(frames[-1]['prediction']).all())
            self.assertAlmostEqual(frames[-1]['prediction'][0][0], .06)


if __name__ == '__main__':
    unittest.main()
