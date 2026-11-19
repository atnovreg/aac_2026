"""Fixed-map ICP using synchronized, deskewed FAST-LIO IMU-frame scans."""
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml

from planning.track_pipeline.projection import read_pcd, xyz
from .estimator import register


# Distances beyond this radius cannot enter the scan-match equations.
CORRESPONDENCE_RADIUS_M = .6


def stamp_is_fresh(stamp_ns, now_ns, max_age_s):
    # The bag's discrete /clock can lag a scan slightly between ticks.
    return stamp_ns > 0 and now_ns > 0 and -50_000_000 <= now_ns - stamp_ns <= max_age_s * 1e9


class MapSurfaces:
    """Plane centres and normals from the prior, in its original coordinates."""

    def __init__(self, directory):
        directory = Path(directory)
        with np.load(directory / 'map_evidence.npz') as evidence:
            plane = evidence['plane']
        cloud = read_pcd(directory / 'map.pcd')
        # The offline comparison uses a 7.5 cm voxel grid over the floor and walls.
        points = xyz(cloud[::max(1, len(cloud) // 1_800_000)])
        heights = points[:, 2] - (points[:, :2] @ plane[:2] + plane[2])
        points = points[(heights > -.10) & (heights < 2.55)]
        voxels = np.floor(points / .075).astype(np.int32)
        _, indices = np.unique(voxels, axis=0, return_inverse=True)
        indices = indices.reshape(-1)
        counts = np.bincount(indices)
        centres = np.column_stack([
            np.bincount(indices, weights=axis) / counts for axis in points.T
        ])
        if len(centres) < 14:
            raise ValueError("Prior map has too few populated voxels")
        neighbours = cKDTree(centres).query(centres, k=14)[1]
        offsets = centres[neighbours]
        offsets -= offsets.mean(axis=1, keepdims=True)
        eigenvalues, axes = np.linalg.eigh(np.einsum('nki,nkj->nij', offsets, offsets))
        planar = (eigenvalues[:, 0] < .11 * eigenvalues[:, 1]) & (eigenvalues[:, 1] > .0005)
        self.centres = centres[planar]
        self.normals = axes[planar, :, 0]
        if len(self.centres) < 100:
            raise ValueError("Prior map has too few planar features for registration")
        self.tree = cKDTree(self.centres)

    def nearest_faces(self, points):
        distances, indices = self.tree.query(points, distance_upper_bound=CORRESPONDENCE_RADIUS_M)
        # cKDTree uses an out-of-range index for an unmatched point. Its infinite
        # distance excludes the placeholder centre and normal from registration.
        indices[~np.isfinite(distances)] = 0
        return self.centres[indices], self.normals[indices], distances


def initial_pose(values):
    values = np.asarray(values, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError("initial_base_pose requires x,y,z,roll,pitch,yaw (metres/radians)")
    transform = np.eye(4)
    transform[:3, 3] = values[:3]
    transform[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    return transform


def imu_from_lidar(config_path):
    with open(config_path) as stream:
        mapping = yaml.safe_load(stream)['/**']['ros__parameters']['mapping']
    if mapping['extrinsic_est_en']:
        raise ValueError("Static IMU-to-LiDAR calibration requires mapping.extrinsic_est_en=false")
    transform = np.eye(4)
    transform[:3, :3] = np.asarray(mapping['extrinsic_R']).reshape(3, 3)
    transform[:3, 3] = mapping['extrinsic_T']
    rot = transform[:3, :3]
    if (not np.isfinite(transform).all() or not np.allclose(rot.T @ rot, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rot), 1)):
        raise ValueError("FAST-LIO extrinsics must be a rigid transform")
    return transform


class MapTracker:
    def __init__(self, world, seed, base_from_imu):
        self.world = world
        self.initial_map_from_imu = seed @ base_from_imu
        self.imu_from_base = np.linalg.inv(base_from_imu)
        self.map_from_odom = None
        self.localized = False
        self.previous_odom_base = None
        self.previous_stamp_ns = None
        self.requires_restart = False

    def update(self, stamp_ns, odom_from_imu, points):
        if self.requires_restart:
            return None, 'time or FAST-LIO origin changed; restart with a new seed'
        odom_base = odom_from_imu @ self.imu_from_base
        previous = self.previous_odom_base
        dt = (stamp_ns - self.previous_stamp_ns) * 1e-9 if previous is not None else None
        if dt is not None and dt <= 0:
            if dt < -.5:
                self.requires_restart = True
            return None, 'out-of-order or rewound input time'
        if self.map_from_odom is None:
            self.map_from_odom = self.initial_map_from_imu @ np.linalg.inv(odom_from_imu)

        twist = None
        if previous is not None:
            # Fixed old-to-new-map alignment corrections must never become velocity.
            # Differentiate the actual base origin in the continuous FAST-LIO frame.
            body_rotation = odom_base[:3, :3]
            linear = body_rotation.T @ (odom_base[:3, 3] - previous[:3, 3]) / dt
            relative = previous[:3, :3].T @ body_rotation
            angular = Rotation.from_matrix(relative).as_rotvec() / dt
            # An origin reset invalidates the fixed-map prior. Limits here detect
            # obvious discontinuities, well outside the vehicle's indoor motion.
            if np.linalg.norm(linear) > 25 or np.linalg.norm(angular) > 15:
                self.requires_restart = True
                return None, 'FAST-LIO pose discontinuity; restart with a new seed'
            if dt <= .5:
                twist = linear, angular

        self.previous_stamp_ns = stamp_ns
        self.previous_odom_base = odom_base
        ranges = np.einsum('ij,ij->i', points, points)
        points = points[(ranges > .45**2) & (ranges < 14**2)]
        # Give the approximate starting pose twice the refinement budget. In
        # tracking, FAST-LIO already predicts the motion to the new scan.
        result = register(self.world, points, self.map_from_odom @ odom_from_imu,
                          max_iterations=8 if self.localized else 16,
                          max_distance=CORRESPONDENCE_RADIUS_M)
        if not result.accepted:
            return None, result.reason
        self.localized = True
        self.map_from_odom = result.pose @ np.linalg.inv(odom_from_imu)
        if twist is None:
            return None, 'waiting for consecutive poses for twist'
        return (result.pose @ self.imu_from_base, *twist), 'accepted'
