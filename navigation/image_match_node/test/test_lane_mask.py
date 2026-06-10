"""Tests for lane-mask feature semantics."""

import cv2
import numpy as np

from opencv_node.lane_mask import (
    classify_lane_keypoints,
    partition_features,
    resolve_morphology_parameters,
)


def test_classify_lane_keypoints_uses_existing_keypoints():
    """Keypoints near the lane mask receive a lane semantic label."""
    mask = np.zeros((100, 200), dtype=np.uint8)
    mask[:, 50] = 255
    keypoints = [
        cv2.KeyPoint(50.0, 20.0, 3.0),
        cv2.KeyPoint(52.0, 40.0, 3.0),
        cv2.KeyPoint(60.0, 60.0, 3.0),
    ]

    labels = classify_lane_keypoints(keypoints, mask, 0.02)

    assert labels.tolist() == [True, True, False]


def test_morphology_parameters_scale_with_resolution():
    """Ratio parameters produce larger cleanup values for larger images."""
    bev_kernel, bev_area = resolve_morphology_parameters(
        (480, 800), 5, 20, 0.006, 0.00005)
    map_kernel, map_area = resolve_morphology_parameters(
        (2292, 3054), 5, 20, 0.006, 0.00005)

    assert bev_kernel == 3
    assert map_kernel == 15
    assert bev_area == 19
    assert map_area == 350


def test_partitioned_features_are_disjoint():
    """Each detected keypoint belongs to exactly one semantic group."""
    keypoints = [
        cv2.KeyPoint(10.0, 10.0, 3.0),
        cv2.KeyPoint(20.0, 20.0, 3.0),
        cv2.KeyPoint(30.0, 30.0, 3.0),
    ]
    descriptors = np.arange(12, dtype=np.float32).reshape(3, 4)
    labels = np.array([True, False, True])

    lane_kp, lane_desc, non_lane_kp, non_lane_desc = partition_features(
        keypoints, descriptors, labels)

    assert [keypoint.pt for keypoint in lane_kp] == [
        keypoints[0].pt, keypoints[2].pt]
    assert [keypoint.pt for keypoint in non_lane_kp] == [keypoints[1].pt]
    assert np.array_equal(lane_desc, descriptors[[0, 2]])
    assert np.array_equal(non_lane_desc, descriptors[[1]])
