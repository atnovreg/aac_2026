"""3D likelihood-field global initialization for a vehicle on one floor."""
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial.transform import Rotation
import yaml


class ParticleInitializer:
    MODE_LIMIT = 16
    PARTICLES_PER_MODE = 32
    GRID_M = .125
    COARSE_STEP_M = .25
    COARSE_YAW_RAD = np.deg2rad(10.)
    MAX_DISTANCE_M = .5
    BASE_HEIGHT_M = .055  # AGX axle midpoint height, see system/f1tenth/config/agx/README.md
    MIN_MARGIN = .06
    MAX_COST = .55

    @classmethod
    def from_map(cls, directory, surface_centres):
        directory = Path(directory)
        with (directory / 'occupancy.yaml').open() as stream:
            resolution = float(yaml.safe_load(stream)['resolution'])
        with np.load(directory / 'map_evidence.npz') as evidence:
            return cls(surface_centres, evidence['usable'], evidence['origin'],
                       resolution, evidence['plane'])

    def __init__(self, map_points, usable, origin, resolution, floor):
        self.usable = np.asarray(usable, dtype=bool)
        self.origin = np.asarray(origin, dtype=float)
        self.resolution = resolution
        self.floor = np.asarray(floor, dtype=float)
        cells = np.argwhere(self.usable)[:, ::-1]
        if len(cells) == 0:
            raise ValueError('Global localization requires usable map cells')
        xy = self.origin + (cells + .5) * resolution
        # Use the available cell nearest each coarse bin centre. This keeps
        # the seed inside the map without shifting all seeds to one bin edge.
        bins = np.floor(xy / self.COARSE_STEP_M).astype(int)
        error = np.linalg.norm(xy - (bins + .5) * self.COARSE_STEP_M, axis=1)
        order = np.argsort(error, kind='stable')
        _, first = np.unique(bins[order], axis=0, return_index=True)
        self.seed_xy = xy[order[first]]

        self.grid_origin = np.floor(map_points.min(axis=0) / self.GRID_M) * self.GRID_M
        voxels = np.floor((map_points - self.grid_origin) / self.GRID_M).astype(int)
        shape = voxels.max(axis=0) + 1
        if np.prod(shape, dtype=np.int64) > 10_000_000:
            raise ValueError('Global localization map is too large for the indoor 3D field')
        free = np.ones(shape, dtype=bool)
        free[tuple(voxels.T)] = False
        distance = distance_transform_edt(free, sampling=self.GRID_M)
        # Clip far and unmatched returns so people and unmapped objects have finite influence.
        self.field = np.minimum(distance / self.MAX_DISTANCE_M, 1.).astype(np.float32) ** 2
        self.rng = np.random.default_rng(0)
        self.reset()

    def reset(self):
        self.particles = None
        self.losses = None
        self.last_odom = None
        self.winner = None
        self.confirmations = 0
        self.search_again = False

    @staticmethod
    def separated(a, b):
        yaw_delta = (a[..., 2] - b[2] + np.pi) % (2 * np.pi) - np.pi
        return (np.linalg.norm(a[..., :2] - b[:2], axis=-1) >= 1.) | (abs(yaw_delta) >= np.deg2rad(35.))

    def poses(self, planar, tilt):
        positions = planar[:, :2]
        angles = np.column_stack((np.broadcast_to(tilt, (len(planar), 2)), planar[:, 2]))
        out = np.broadcast_to(np.eye(4), (len(planar), 4, 4)).copy()
        out[:, :2, 3] = positions
        out[:, 2, 3] = positions @ self.floor[:2] + self.floor[2] + self.BASE_HEIGHT_M
        out[:, :3, :3] = Rotation.from_euler('xyz', angles).as_matrix()
        return out

    def cost(self, poses, points):
        mapped = np.einsum('nj,pij->pni', points, poses[:, :3, :3]) + poses[:, None, :3, 3]
        indices = np.floor((mapped - self.grid_origin) / self.GRID_M).astype(int)
        valid = ((indices >= 0) & (indices < self.field.shape)).all(axis=2)
        values = np.ones(indices.shape[:2])
        values[valid] = self.field[tuple(indices[valid].T)]
        return values.mean(axis=1)

    def initialize(self, points, tilt):
        # The first scan searches all usable XY cells and a full turn of headings.
        all_states, all_costs = [], []
        for yaw in np.arange(-np.pi, np.pi - 1e-8, self.COARSE_YAW_RAD):
            for xy in np.array_split(self.seed_xy, (len(self.seed_xy) + 255) // 256):
                states = np.column_stack((xy, np.full(len(xy), yaw)))
                all_states.append(states)
                all_costs.append(self.cost(self.poses(states, tilt), points))
        states = np.concatenate(all_states)
        costs = np.concatenate(all_costs)
        selected = []
        remaining = np.isfinite(costs) & (costs <= self.MAX_COST)
        while remaining.any() and len(selected) <= self.MODE_LIMIT:
            best = np.argmin(np.where(remaining, costs, np.inf))
            selected.append(best)
            remaining &= self.separated(states, states[best])
        if not selected:
            return False
        self.search_again = (len(selected) > self.MODE_LIMIT
                             and costs[selected[-1]] - costs[selected[0]] < self.MIN_MARGIN)
        starts = states[selected[:self.MODE_LIMIT]]
        perturbations = self.rng.normal(size=(len(starts), self.PARTICLES_PER_MODE, 3))
        perturbations *= [.14, .14, .09]
        perturbations[:, 0] = 0.
        self.particles = starts[:, None, :] + perturbations
        return True

    def update(self, odom_from_base, points_base):
        """Return a confirmed map-from-base transform for the input scan, or None."""
        # Ground returns alone cannot distinguish position along a corridor.
        mask = ((points_base[:, 2] > .18) & (points_base[:, 2] < 2.35)
                & (np.linalg.norm(points_base[:, :2], axis=1) > .45))
        points = points_base[mask]
        if len(points) < 100:
            return None, 'insufficient non-ground lidar returns'
        points = points[np.linspace(0, len(points)-1, min(len(points), 256), dtype=int)]
        tilt = Rotation.from_matrix(odom_from_base[:3, :3]).as_euler('xyz')[:2]
        first = self.last_odom is None
        if not first:
            delta = np.linalg.solve(self.last_odom, odom_from_base)
            delta_yaw = np.arctan2(delta[1, 0], delta[0, 0])
            if np.linalg.norm(delta[:2, 3]) < .18 and abs(delta_yaw) < .18:
                return None, 'waiting for motion to distinguish location hypotheses'
        if first or self.search_again:
            self.reset()
            if not self.initialize(points, tilt):
                return None, 'no supported global lidar hypotheses'
        else:
            yaw = self.particles[..., 2]
            dx, dy = delta[:2, 3]
            self.particles[..., 0] += np.cos(yaw)*dx - np.sin(yaw)*dy
            self.particles[..., 1] += np.sin(yaw)*dx + np.cos(yaw)*dy
            self.particles[..., 2] += delta_yaw
        self.last_odom = odom_from_base.copy()

        shape = self.particles.shape[:2]
        states = self.particles.reshape(-1, 3)
        costs = self.cost(self.poses(states, tilt), points)
        index = np.floor((states[:, :2] - self.origin) / self.resolution).astype(int)
        valid = ((index >= 0) & (index < self.usable.shape[::-1])).all(axis=1)
        valid[valid] = self.usable[index[valid, 1], index[valid, 0]]
        costs[~valid] = 1.
        costs = costs.reshape(shape)
        best_index = costs.argmin(axis=1)
        modes = np.arange(shape[0])
        best = self.particles[modes, best_index].copy()
        measurement = costs[modes, best_index]
        self.losses = measurement if self.losses is None else .45*self.losses + .55*measurement
        winner = self.losses.argmin()
        alternatives = self.separated(best, best[winner])
        runner_up = self.losses[alternatives].min(initial=np.inf)
        margin = runner_up - self.losses[winner]

        # Resample separately inside each mode, so a plausible second corridor
        # cannot be erased by stochastic resampling of the entire population.
        for i, row in enumerate(costs):
            weights = np.exp(-12. * (row - row.min()))
            samples = self.rng.choice(shape[1], size=shape[1]-1, p=weights/weights.sum())
            noise = self.rng.normal(size=(len(samples), 3)) * [.06, .06, .035]
            self.particles[i, 1:] = self.particles[i, samples] + noise
            self.particles[i, 0] = best[i]
        if measurement.min() > self.MAX_COST:
            self.search_again = True
        if (not self.search_again and measurement[winner] <= self.MAX_COST
                and margin >= self.MIN_MARGIN):
            self.confirmations = self.confirmations + 1 if winner == self.winner else 1
            self.winner = winner
            if self.confirmations >= 3:
                return self.poses(best[winner:winner+1], tilt)[0], 'unique lidar hypothesis'
        else:
            self.winner, self.confirmations = None, 0
        return None, (f'{shape[0]} lidar hypotheses, best cost {measurement[winner]:.3f}, '
                      f'margin {margin:.3f}, confirmations {self.confirmations}/3')
