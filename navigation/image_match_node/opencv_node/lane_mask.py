"""Classical lane-mask preprocessing shared by map and BEV matching."""

import cv2
import numpy as np


def remove_small_components(mask, min_area):
    """Remove connected components smaller than min_area pixels."""
    if min_area <= 0:
        return mask
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    cleaned = np.zeros_like(mask)
    for label in range(1, num_labels):
        if stats[label, cv2.CC_STAT_AREA] >= min_area:
            cleaned[labels == label] = 255
    return cleaned


def resolve_morphology_parameters(
        image_shape, kernel_size, min_area,
        kernel_size_ratio=0.0, min_area_ratio=0.0):
    """Resolve morphology parameters for the current image resolution."""
    height, width = image_shape[:2]
    if kernel_size_ratio > 0.0:
        kernel_size = max(
            1, int(round(min(height, width) * kernel_size_ratio)))
    kernel_size = max(1, int(kernel_size))
    if kernel_size % 2 == 0:
        kernel_size += 1

    if min_area_ratio > 0.0:
        min_area = max(
            1, int(round(height * width * min_area_ratio)))
    return kernel_size, max(0, int(min_area))


def extract_lane_mask(
        bgr, hsv_lower, hsv_upper, roi_y_min_ratio, roi_y_max_ratio,
        kernel_size, open_iterations, close_iterations, min_area,
        kernel_size_ratio=0.0, min_area_ratio=0.0):
    """Extract lane paint using the classic_road_mask_ros2 pipeline."""
    height, width = bgr.shape[:2]
    kernel_size, min_area = resolve_morphology_parameters(
        bgr.shape,
        kernel_size,
        min_area,
        kernel_size_ratio,
        min_area_ratio,
    )
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.asarray(hsv_lower, dtype=np.uint8),
        np.asarray(hsv_upper, dtype=np.uint8),
    )

    roi = np.zeros((height, width), dtype=np.uint8)
    y0 = int(round(height * roi_y_min_ratio))
    y1 = int(round(height * roi_y_max_ratio))
    roi[y0:y1, :] = 255
    mask = cv2.bitwise_and(mask, roi)

    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    if open_iterations > 0:
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, kernel, iterations=open_iterations)
    if close_iterations > 0:
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, kernel, iterations=close_iterations)
    return remove_small_components(mask, min_area)


def make_lane_feature_mask(lane_mask, dilation_ratio):
    """Expand lane pixels so descriptors can include paint boundaries."""
    if dilation_ratio <= 0.0:
        return lane_mask
    radius = max(
        1, int(round(min(lane_mask.shape[:2]) * dilation_ratio)))
    kernel_size = radius * 2 + 1
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    return cv2.dilate(lane_mask, kernel)


def classify_lane_keypoints(keypoints, lane_mask, dilation_ratio):
    """Classify already-detected keypoints using an expanded lane mask."""
    labels = np.zeros(len(keypoints), dtype=bool)
    if not keypoints or not lane_mask.any():
        return labels

    feature_mask = make_lane_feature_mask(lane_mask, dilation_ratio)
    height, width = feature_mask.shape[:2]
    points = np.rint([keypoint.pt for keypoint in keypoints]).astype(np.int32)
    valid = (
        (points[:, 0] >= 0) & (points[:, 0] < width)
        & (points[:, 1] >= 0) & (points[:, 1] < height)
    )
    labels[valid] = (
        feature_mask[points[valid, 1], points[valid, 0]] > 0)
    return labels


def partition_features(keypoints, descriptors, lane_labels):
    """Split one detector result into disjoint lane/non-lane groups."""
    if descriptors is None or len(keypoints) == 0:
        return [], None, [], None
    lane_indices = np.flatnonzero(lane_labels)
    non_lane_indices = np.flatnonzero(~lane_labels)
    lane_keypoints = [keypoints[index] for index in lane_indices]
    non_lane_keypoints = [
        keypoints[index] for index in non_lane_indices]
    lane_descriptors = (
        descriptors[lane_indices] if len(lane_indices) else None)
    non_lane_descriptors = (
        descriptors[non_lane_indices] if len(non_lane_indices) else None)
    return (
        lane_keypoints,
        lane_descriptors,
        non_lane_keypoints,
        non_lane_descriptors,
    )
