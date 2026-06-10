"""Tests for deterministic and bounded Chamfer helpers."""

import math

import numpy as np

from opencv_node.chamfer import (
    centered_rigid_correction,
    chamfer_score,
    deterministic_subsample,
)


def test_deterministic_subsample_is_repeatable():
    """The same ordered points always produce the same subset."""
    points = np.arange(200).reshape(100, 2)

    first = deterministic_subsample(points, 20)
    second = deterministic_subsample(points, 20)

    assert np.array_equal(first, second)
    assert len(first) == 20


def test_chamfer_score_rejects_low_projection_coverage():
    """A transform cannot improve its score by moving points off the map."""
    distance_transform = np.zeros((10, 10), dtype=np.float32)
    points = np.array(
        [[[1.0, 1.0]], [[2.0, 2.0]], [[3.0, 3.0]], [[4.0, 4.0]]],
        dtype=np.float32,
    )
    homography = np.array(
        [[1.0, 0.0, 8.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )

    score, coverage = chamfer_score(
        distance_transform, homography, points, 0.8, 30.0)

    assert math.isinf(score)
    assert coverage == 0.25


def test_centered_rotation_keeps_vehicle_center_fixed():
    """Rotation is applied around the projected vehicle center."""
    center = np.array([100.0, 200.0])
    correction = centered_rigid_correction(
        [0.0, 0.0, math.radians(10.0)], center)
    homogeneous_center = np.array([center[0], center[1], 1.0])

    corrected = correction @ homogeneous_center

    assert np.allclose(corrected[:2], center)
