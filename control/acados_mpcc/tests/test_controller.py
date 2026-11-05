"""Catch changes in objective units, terminal terms and actuator increments."""
import tempfile
import unittest
from pathlib import Path

import casadi as ca
import numpy as np

from control.acados_mpcc.controller import Controller, build_ocp, default_config
from control.acados_mpcc.track import Track


class ControllerFormulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.track = Track.from_csv(Path('tracks/indoor_final/track.csv'))
        cls.config = default_config()
        cls.ocp, cls.scale = build_ocp(cls.track, cls.config, Path(cls.directory.name))

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_sampled_cost_and_progress_reward_match_baseline(self):
        ref = self.track.reference(5.)
        tangent = ref['tangent']
        normal = np.array([-tangent[1], tangent[0]])
        xy = ref['center'] + .12 * normal + .03 * tangent
        physical = np.r_[xy, ref['yaw'], 1.4, .1, .7, 5., .4, -.1, 2.]
        increment = np.array([.2, -.3, .1])
        residual = ca.Function('stage_cost', [self.ocp.model.x, self.ocp.model.u], [self.ocp.model.cost_y_expr])
        y = np.asarray(residual(physical / self.scale, increment)).ravel() - self.ocp.cost.yref
        value = .5 * y @ self.ocp.cost.W @ y - 1.0
        # qV=.02, rV=.0001: completing the square adds exactly 1.
        expected = (.1 * .12**2 + 1000 * .03**2 + 1e-5 * .7**2
                    + 1e-4 * (.4**2 + .1**2 + 2.**2) - .02 * 2.
                    + .01 * .2**2 + 1. * .3**2 + .001 * .1**2)
        self.assertAlmostEqual(value, expected, places=7)
        np.testing.assert_array_equal(self.ocp.solver_options.cost_scaling, np.ones(41))

    def test_terminal_multipliers_and_last_input_reward(self):
        ref = self.track.reference(5.)
        t = ref['tangent']
        xy = ref['center'] + .12 * np.array([-t[1], t[0]]) + .03 * t
        physical = np.r_[xy, ref['yaw'], 1.4, .1, .7, 5., .4, -.1, 2.]
        fun = ca.Function('terminal_cost', [self.ocp.model.x], [self.ocp.model.cost_y_expr_e])
        y = np.asarray(fun(physical / self.scale)).ravel() - self.ocp.cost.yref_e
        value = .5 * y @ self.ocp.cost.W_e @ y - 1.
        expected = 1. * .12**2 + 1000 * .03**2 + 1e-4 * .7**2 + 1e-4 * (.4**2 + .1**2 + 4.) - .04
        self.assertAlmostEqual(value, expected, places=7)

    def test_increment_is_normalized_step_change_not_rate(self):
        physical = np.array([0., 0., 0., 1., 0., 0., 5., .3, 0., 1.])
        increment = np.array([.1, .2, .3])
        step = ca.Function('aug_step', [self.ocp.model.x, self.ocp.model.u], [self.ocp.model.disc_dyn_expr])
        result = np.asarray(step(physical / self.scale, increment)).ravel() * self.scale
        np.testing.assert_allclose(result[7:], [.4, .07, 2.8], atol=1e-12)
        self.assertAlmostEqual(result[6], 5.056, places=12)

    def test_shift_propagates_physical_tail_with_progress(self):
        controller = Controller.__new__(Controller)
        controller.config = self.config
        controller.n = 40
        # Analytic straight-line equilibrium at vx=1: no force or yaw motion.
        duty = (.0518 + .00035) / (.287 - .0545)
        controller.prediction = np.tile([0., 0., 0., 1., 0., 0., 0., duty, 0., 1.], (41, 1))
        controller.prediction[:, 0] = controller.prediction[:, 6] = np.arange(41)*.02
        controller.increments = np.zeros((40, 3))
        controller.shift_prediction()
        self.assertAlmostEqual(controller.prediction[-1, 0], .82, places=12)
        self.assertAlmostEqual(controller.prediction[-1, 6], .82, places=12)


if __name__ == '__main__':
    unittest.main()
