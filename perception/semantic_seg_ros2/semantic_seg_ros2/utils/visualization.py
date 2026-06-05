"""Semantic segmentation mask visualization helpers."""

from __future__ import annotations

import cv2
import numpy as np


DEFAULT_PALETTE_BGR = np.array(
    [
        (35, 35, 35),
        (70, 190, 80),
        (0, 220, 255),
        (70, 70, 190),
        (200, 120, 80),
        (180, 80, 180),
        (90, 180, 210),
        (240, 220, 90),
    ],
    dtype=np.uint8,
)


def parse_class_ids(values: list[int] | tuple[int, ...] | None) -> list[int]:
    if values is None:
        return []
    return [int(value) for value in values]


def mask_from_class_ids(labels: np.ndarray, class_ids: list[int]) -> np.ndarray:
    if not class_ids:
        return np.zeros(labels.shape, dtype=np.uint8)
    mask = np.isin(labels, np.array(class_ids, dtype=labels.dtype))
    return mask.astype(np.uint8) * 255


def colorize_labels(labels: np.ndarray, palette: np.ndarray | None = None) -> np.ndarray:
    palette = DEFAULT_PALETTE_BGR if palette is None else palette
    color = np.zeros((*labels.shape, 3), dtype=np.uint8)
    for class_id in np.unique(labels):
        color[labels == class_id] = palette[int(class_id) % len(palette)]
    return color


def make_overlay(
    bgr: np.ndarray,
    labels: np.ndarray,
    drivable_mask: np.ndarray,
    lane_mask: np.ndarray,
    alpha: float,
) -> np.ndarray:
    seg_color = colorize_labels(labels)
    overlay = (
        bgr.astype(np.float32) * (1.0 - alpha)
        + seg_color.astype(np.float32) * alpha
    ).astype(np.uint8)

    if np.any(drivable_mask):
        contours, _ = cv2.findContours(drivable_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay, contours, -1, (80, 220, 80), 2)
    if np.any(lane_mask):
        overlay[lane_mask > 0] = (0, 240, 255)
    return overlay
