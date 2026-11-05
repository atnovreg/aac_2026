"""Periodic measured-track geometry shared by Python and generated C solvers."""

import json
from pathlib import Path

import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.optimize import minimize_scalar


class Track:
    """Closed centerline in the original map XY frame, with paired half-widths.

    A single periodic cubic B-spline represents the center in both SciPy and
    CasADi. Tangent/yaw/curvature are derived from that same curve rather than
    interpolated independently. Widths use periodic linear interpolation to
    avoid cubic overshoot beyond the exported limits.
    """

    def __init__(self, s, xy, width_left, width_right, length):
        self.s = np.asarray(s, dtype=float)
        self.xy = np.asarray(xy, dtype=float)
        self.width_left = np.asarray(width_left, dtype=float)
        self.width_right = np.asarray(width_right, dtype=float)
        self.length = float(length)
        n = len(self.s)
        if n < 4 or self.xy.shape != (n, 2):
            raise ValueError("A closed track requires at least four XY samples")
        if self.width_left.shape != (n,) or self.width_right.shape != (n,):
            raise ValueError("Each center sample requires left and right widths")
        if not all(np.all(np.isfinite(a)) for a in
                   (self.s, self.xy, self.width_left, self.width_right, self.length)):
            raise ValueError("Track values must be finite")
        if abs(self.s[0]) > 1e-8 or np.any(np.diff(self.s) <= 0) or self.length <= self.s[-1]:
            raise ValueError("Track progress must start at zero and increase below the closed length")
        if np.any(self.width_left <= 0) or np.any(self.width_right <= 0):
            raise ValueError("Track widths must be positive")
        self._closed_s = np.r_[self.s, self.length]
        self._closed_xy = np.vstack((self.xy, self.xy[0]))
        self._spline = make_interp_spline(self._closed_s, self._closed_xy,
                                          k=3, bc_type="periodic")
        self._symbolic_function = None

    @classmethod
    def from_csv(cls, path):
        path = Path(path)
        data = np.genfromtxt(path, delimiter=",", names=True)
        fields = ("s", "x", "y", "yaw", "curvature", "width_left", "width_right")
        if not data.dtype.names or not set(fields).issubset(data.dtype.names):
            raise ValueError("Track CSV requires s,x,y,yaw,curvature,width_left,width_right")
        data = np.atleast_1d(data)
        if len(data) < 4 or not all(np.all(np.isfinite(data[key])) for key in fields):
            raise ValueError("Track CSV requires at least four finite samples")
        xy = np.column_stack((data["x"], data["y"]))
        repeated_endpoint = np.linalg.norm(xy[-1] - xy[0]) < 1e-8
        validation = path.with_name("validation.json")
        if validation.exists():
            length = float(json.loads(validation.read_text())["length_m"])
        elif repeated_endpoint:
            length = float(data["s"][-1])
        else:
            length = float(data["s"][-1] + np.linalg.norm(xy[-1] - xy[0]))
        if repeated_endpoint:
            if not np.isclose(data["s"][-1], length, atol=1e-6):
                raise ValueError("Repeated endpoint does not agree with track length")
            data, xy = data[:-1], xy[:-1]
        return cls(data["s"], xy, data["width_left"], data["width_right"], length)

    def reference(self, s):
        """Return center/tangent (...,2), yaw/curvature/widths (...) for scalar or array s."""
        wrapped = np.mod(np.asarray(s, dtype=float), self.length)
        center = self._spline(wrapped)
        derivative = self._spline(wrapped, nu=1)
        second = self._spline(wrapped, nu=2)
        speed = np.linalg.norm(derivative, axis=-1)
        tangent = derivative / speed[..., None]
        return {
            "center": center,
            "tangent": tangent,
            "yaw": np.arctan2(tangent[..., 1], tangent[..., 0]),
            "curvature": (derivative[..., 0] * second[..., 1]
                          - derivative[..., 1] * second[..., 0]) / speed**3,
            "width_left": np.interp(wrapped, self._closed_s, np.r_[self.width_left, self.width_left[0]]),
            "width_right": np.interp(wrapped, self._closed_s, np.r_[self.width_right, self.width_right[0]]),
        }

    def project(self, xy, previous_s=None):
        """Project actual position onto the curve; previous_s keeps progress continuous.

        Initial projection searches the whole track. Later projections search a
        local arc window, avoiding false jumps to an adjacent section of a loop.
        Reverse movement remains negative; this function never forces progress.
        """
        point = np.asarray(xy, dtype=float)
        if point.shape != (2,) or not np.all(np.isfinite(point)):
            raise ValueError("Projection requires one finite XY position")
        if previous_s is not None and not np.isfinite(previous_s):
            raise ValueError("Previous progress must be finite")
        candidates = np.arange(len(self.s))
        if previous_s is not None:
            distance_s = (self.s - previous_s + self.length / 2) % self.length - self.length / 2
            window = min(self.length / 4, max(1.0, self.length * 0.03))
            candidates = candidates[np.abs(distance_s) <= window]
            if not len(candidates):
                candidates = np.array([np.argmin(np.abs(distance_s))])
        start = self._closed_xy[candidates]
        direction = self._closed_xy[candidates + 1] - start
        fractions = np.clip(np.sum((point - start) * direction, axis=1)
                            / np.maximum(np.sum(direction**2, axis=1), 1e-20), 0, 1)
        squared = np.sum((start + fractions[:, None] * direction - point)**2, axis=1)
        index = candidates[np.argmin(squared)]
        spacing = self._closed_s[index + 1] - self._closed_s[index]
        lo, hi = self._closed_s[index] - spacing, self._closed_s[index + 1] + spacing
        result = minimize_scalar(
            lambda progress: float(np.sum((self._spline(progress % self.length) - point)**2)),
            bounds=(lo, hi), method="bounded", options={"xatol": 1e-11},
        )
        progress = float(result.x % self.length)
        if min(progress, self.length - progress) < 1e-8:
            progress = 0.0
        if previous_s is not None:
            progress += self.length * round((previous_s - progress) / self.length)
        return progress

    def symbolic_reference(self, s):
        """Return (center2, unit_tangent2) as differentiable, C-generatable MX graphs."""
        import casadi as ca

        if self._symbolic_function is None:
            progress = ca.MX.sym("track_progress")
            wrapped = progress - self.length * ca.floor(progress / self.length)
            # Keep the wrapped interval strictly inside the knot domain. CasADi
            # 3.7.2's second derivative at the domain's first knot is doubled;
            # three identical periods avoid that endpoint convention entirely.
            extended_s = np.r_[self.s - self.length, self.s, self.s + self.length,
                               2 * self.length]
            extended_xy = np.vstack((np.tile(self.xy, (3, 1)), self.xy[0]))
            extended = make_interp_spline(extended_s, extended_xy, k=3, bc_type="periodic")
            curve = ca.bspline(wrapped, ca.DM(extended.c.ravel()),
                               [extended.t.tolist()], [extended.k], 2, {})
            derivative = ca.jacobian(curve, progress)
            tangent = derivative / ca.sqrt(ca.dot(derivative, derivative))
            self._symbolic_function = ca.Function("periodic_track_reference", [progress],
                                                 [curve, tangent])
        return tuple(self._symbolic_function(s))
