"""Catch instantaneous-drive prediction and delay queue ordering errors."""
import tempfile
import unittest
from pathlib import Path

import casadi as ca
import numpy as np

from control.acados_mpcc.controller import build_ocp, default_config
from control.acados_mpcc.slash_model import load_parameters
from control.acados_mpcc.track import Track


class ActuatorPredictionTests(unittest.TestCase):
    def config(self):
        cfg = default_config()
        cfg.update(model='slash_58276_74', model_parameters=load_parameters(),
                   dt=.03, speed_max=60/3.6, progress_speed_max=60/3.6,
                   actuator_prediction=dict(drive_delay_s=.02, steering_delay_s=.04,
                                            drive_tau_s=.06, steering_tau_s=.08))
        return cfg

    def build(self, cfg):
        track = Track.from_csv('tracks/geonji_slash_both_inner/track.csv')
        with tempfile.TemporaryDirectory() as folder:
            ocp, scale = build_ocp(track, cfg, Path(folder))
        return ocp, scale

    def test_drive_command_is_delayed_and_endpoint_differs_from_mean(self):
        ocp, scale = self.build(self.config())
        self.assertEqual(len(scale), 18,
                         'prediction must retain actuator endpoints and pending commands')
        fun = ca.Function('delayed_step_test', [ocp.model.x, ocp.model.u],
                          [ocp.model.disc_dyn_expr])
        physical = np.zeros(18)
        physical[3] = 1.
        result = np.asarray(fun(physical/scale, [1., 0., 0.])).ravel()*scale
        # Two zero-input 10ms ticks precede the first nonzero drive response.
        self.assertAlmostEqual(result[10], .15351827510938598, places=12)
        np.testing.assert_allclose(result[12:14], [1., 1.], atol=1e-12)
        np.testing.assert_allclose(result[14:18], [0., 0., 0., 0.], atol=1e-12)
        self.assertLess(result[3], 1.01, 'pending input must not act immediately')
        self.assertAlmostEqual(float(ocp.constraints.ubx[3]*scale[3]), 60/3.6)

    def test_existing_pending_steering_reaches_plant_before_new_command(self):
        ocp, scale = self.build(self.config())
        self.assertEqual(len(scale), 18)
        fun = ca.Function('pending_step_test', [ocp.model.x, ocp.model.u],
                          [ocp.model.disc_dyn_expr])
        physical = np.zeros(18)
        physical[3] = 3.
        physical[14:18] = [.1, .2, .3, .4]
        result = np.asarray(fun(physical/scale, [0., -.2/.35, 0.])).ravel()*scale
        # Analytical cascade of held .1, .2, .3 targets, each lasting 10ms.
        self.assertAlmostEqual(result[11], .06514130355530276, places=12)
        np.testing.assert_allclose(result[14:18], [.4, -.2, -.2, -.2], atol=1e-12)
        self.assertGreater(result[5], 0., 'old positive steering must affect yaw')

    def test_fractional_delay_is_rejected(self):
        cfg = self.config()
        cfg['actuator_prediction']['drive_delay_s'] = .015
        with self.assertRaises(ValueError):
            self.build(cfg)

    def test_python_plant_obeys_same_pending_command_delay(self):
        from control.acados_mpcc import simulation
        self.assertTrue(hasattr(simulation, 'integrate_plant'),
                        'Python simulation must integrate delayed applied inputs')
        state = np.array([0., 0., 0., 1., 0., 0., 0.])
        result = simulation.integrate_plant(state, np.array([1., 0., 1.]),
                                           self.config(), np.zeros(8))
        self.assertTrue(result.success)
        # A new drive command remains zero for its first two 10ms ticks.
        self.assertLess(result.y[3, 9], 1.)
        self.assertAlmostEqual(result.actuator_states[-1][0], .15351827510938598)
        self.assertAlmostEqual(result.sample_t[-1], .03)

    def test_prediction_integrator_rejects_fractional_substep_count(self):
        cfg = self.config()
        cfg['actuator_prediction_substeps'] = 1.5
        with self.assertRaises(ValueError):
            self.build(cfg)

    def test_two_substep_prediction_matches_independent_rk45_over_speed_range(self):
        from control.acados_mpcc.actuator_model import augmented_step
        from control.acados_mpcc.simulation import integrate_plant
        cfg = self.config()
        cfg['actuator_prediction_substeps'] = 2
        for speed in (1., 8., 16.):
            x = np.zeros(18)
            x[3] = speed
            x[4:6] = [.1, .2]
            x[10:12] = [.4, -.05]
            x[12:18] = [.3, -.2, -.1, .05, .1, .15]
            command = np.array([-.3, .15, speed])
            predicted = augmented_step(x, command, cfg)
            independent = integrate_plant(x[:7], command, cfg, x[10:])
            np.testing.assert_allclose(predicted[:7], independent.y[:, -1], atol=2e-6, rtol=0)


if __name__ == '__main__':
    unittest.main()
