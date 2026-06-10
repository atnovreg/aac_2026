"""Pure helpers for lane-mask Chamfer refinement."""

import cv2
import numpy as np


def deterministic_subsample(points, max_points):
    """Uniformly subsample an ordered point array without randomness."""
    if len(points) <= max_points:
        return points
    indices = np.floor(
        np.arange(max_points, dtype=np.float64)
        * len(points) / max_points
    ).astype(np.int64)
    return points[indices]


def chamfer_score(
        distance_transform, homography, points_xy,
        min_coverage, outside_penalty):
    """Return penalized mean EDT score and in-map projection coverage."""
    if len(points_xy) == 0:
        return float('inf'), 0.0

    projected = cv2.perspectiveTransform(
        points_xy, homography).reshape(-1, 2)
    height, width = distance_transform.shape[:2]
    valid = (
        np.isfinite(projected).all(axis=1)
        & (projected[:, 0] >= 0) & (projected[:, 0] < width)
        & (projected[:, 1] >= 0) & (projected[:, 1] < height)
    )
    coverage = float(np.count_nonzero(valid) / len(projected))
    if coverage < min_coverage:
        return float('inf'), coverage

    distances = np.full(len(projected), outside_penalty, dtype=np.float32)
    px = projected[valid, 0].astype(np.int32)
    py = projected[valid, 1].astype(np.int32)
    distances[valid] = distance_transform[py, px]
    return float(distances.mean()), coverage


def centered_rigid_correction(params, center_xy):
    """Build a rigid 2-D correction rotating around center_xy."""
    dx, dy, dtheta = params
    center = np.asarray(center_xy, dtype=np.float64)
    cos_t, sin_t = np.cos(dtheta), np.sin(dtheta)
    rotation = np.array(
        [[cos_t, -sin_t], [sin_t, cos_t]], dtype=np.float64)
    translation = center + np.array([dx, dy]) - rotation @ center

    correction = np.eye(3, dtype=np.float64)
    correction[:2, :2] = rotation
    correction[:2, 2] = translation
    return correction
