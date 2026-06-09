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


def extract_lane_mask(
        bgr, hsv_lower, hsv_upper, roi_y_min_ratio, roi_y_max_ratio,
        kernel_size, open_iterations, close_iterations, min_area):
    """Extract lane paint using the classic_road_mask_ros2 pipeline."""
    height, width = bgr.shape[:2]
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
