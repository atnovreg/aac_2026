"""Numerical parity and closed-track geometry behavior, independent of acados."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from control.acados_mpcc.model import orca_rhs, rk4_step, symbolic_orca_rhs
from control.acados_mpcc.track import Track


FIXTURES = Path(__file__).parent / "fixtures" / "matlab_orca_golden.json"


class ModelTests(unittest.TestCase):
    def test_original_matlab_derivatives_and_integrated_step(self):
        """Catch sign, tire, mass and RK4 errors against independently run MATLAB."""
        fixtures = json.loads(FIXTURES.read_text())
        for sample in fixtures["samples"]:
            with self.subTest(state=sample["state"]):
                x, u = np.asarray(sample["state"]), np.asarray(sample["input"])
                np.testing.assert_allclose(orca_rhs(x, u), sample["ode"], atol=1e-11)
                np.testing.assert_allclose(
                    rk4_step(x, u), sample["ode45_next_state"], atol=2e-6, rtol=2e-6
                )

    def test_stationary_input_rhs_and_progress(self):
        """Zero-speed numeric dynamics remain finite and retain commanded progress."""
        derivative = orca_rhs(np.zeros(7), np.array([0.5, 0.0, 1.25]))
        np.testing.assert_allclose(
            derivative, [0, 0, 0, (0.287 * 0.5 - 0.0518) / 0.041, 0, 0, 1.25]
        )

    @unittest.skipUnless(importlib.util.find_spec("casadi"), "CasADi environment required")
    def test_symbolic_model_matches_matlab_including_reverse_motion(self):
        """Numeric and generated model retain original abs(vx) tire-slip convention."""
        import casadi as ca

        x, u = ca.MX.sym("x", 7), ca.MX.sym("u", 3)
        f = ca.Function("orca_test", [x, u], [symbolic_orca_rhs(x, u)])
        fixtures = json.loads(FIXTURES.read_text())
        for sample in fixtures["samples"]:
            np.testing.assert_allclose(
                np.asarray(f(sample["state"], sample["input"])).ravel(),
                sample["ode"], atol=1e-11,
            )
        for vx in [-1.2, 0.0, 0.03]:
            state = np.array([1, 2, 0.5, vx, 0.1, -0.2, 5])
            control = np.array([0.2, 0.1, 1.3])
            np.testing.assert_allclose(
                np.asarray(f(state, control)).ravel(), orca_rhs(state, control), atol=1e-12
            )


class TrackTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "track.csv"
        radius = 3.0
        self.length = 2 * np.pi * radius
        angle = np.arange(120) * 2 * np.pi / 120
        rows = np.column_stack((radius * angle, 10 + radius * np.cos(angle),
                                -7 + radius * np.sin(angle), angle + np.pi / 2,
                                np.full(120, 1 / radius), np.full(120, 0.6),
                                np.full(120, 0.9)))
        np.savetxt(self.path, rows, delimiter=",", comments="",
                   header="s,x,y,yaw,curvature,width_left,width_right")
        self.path.with_name("validation.json").write_text(json.dumps({"length_m": self.length}))

    def test_circle_reference_is_periodic_with_correct_normal_geometry(self):
        """Catch origin scaling, seam gaps, nonunit tangent and curvature sign errors."""
        track = Track.from_csv(self.path)
        s = np.array([-0.1, 0.0, 0.1, self.length - 0.1, self.length, self.length + 0.1])
        ref = track.reference(s)
        angle = s / 3
        np.testing.assert_allclose(ref["center"],
                                   np.column_stack((10 + 3 * np.cos(angle), -7 + 3 * np.sin(angle))),
                                   atol=2e-7)
        np.testing.assert_allclose(ref["tangent"],
                                   np.column_stack((-np.sin(angle), np.cos(angle))), atol=2e-6)
        np.testing.assert_allclose(ref["curvature"], 1 / 3, atol=1e-4)
        np.testing.assert_allclose(ref["width_left"], 0.6, atol=1e-12)
        np.testing.assert_allclose(ref["width_right"], 0.9, atol=1e-12)
        np.testing.assert_allclose(track.reference(0)["center"], [13, -7], atol=1e-12)

    def test_projection_unwraps_actual_position_in_both_directions(self):
        """Crossing the seam must add/subtract progress rather than creating a fake lap."""
        track = Track.from_csv(self.path)
        for expected_s, previous_s in [(self.length + 0.04, self.length - 0.03),
                                       (-0.04, 0.03), (2 * self.length + 0.03, 2 * self.length - 0.02)]:
            angle = expected_s / 3
            xy = np.array([10 + 3.2 * np.cos(angle), -7 + 3.2 * np.sin(angle)])
            self.assertAlmostEqual(track.project(xy, previous_s=previous_s), expected_s, delta=2e-6)
        self.assertAlmostEqual(track.project([10, -4]), 0.25 * self.length, delta=2e-6)

    @unittest.skipUnless(importlib.util.find_spec("casadi"), "CasADi environment required")
    def test_symbolic_spline_matches_numeric_and_has_seam_derivatives(self):
        """Generated expressions must use the same periodic curve and continuous derivative."""
        import casadi as ca

        track = Track.from_csv(self.path)
        s = ca.MX.sym("s")
        center, tangent = track.symbolic_reference(s)
        fn = ca.Function("track_test", [s], [center, tangent, ca.jacobian(center, s),
                                            ca.jacobian(tangent, s)])
        for position in [-self.length - 0.1, -1e-8, 0, 1e-8, 1.42, self.length - 1e-8,
                         self.length, self.length + 1e-8, 3 * self.length + 0.1]:
            c, t, dc, dt = [np.asarray(v).ravel() for v in fn(position)]
            expected = track.reference(position)
            np.testing.assert_allclose(c, expected["center"], atol=1e-11)
            np.testing.assert_allclose(t, expected["tangent"], atol=1e-11)
            np.testing.assert_allclose(dc, expected["tangent"], atol=1e-6)
            np.testing.assert_allclose(dt, [-np.cos(position / 3) / 3,
                                           -np.sin(position / 3) / 3], atol=1e-4)
        # Test actual C generation, so unsupported symbolic nodes cannot reach acados.
        generator = ca.CodeGenerator("track_test.c")
        generator.add(fn)
        generator.generate(self.directory.name + "/")


if __name__ == "__main__":
    unittest.main()
