#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Extract inner/outer track lane lines from a CAD-like PNG image and export matched wide-format CSV.

Definition used here:
- left_line   = inner red lane boundary
- right_line  = outer red lane boundary
- center_line = midpoint between left_line and right_line at the same seq

This script is intended for the provided track drawing image. It uses color-based red-line
segmentation, so DXF/DWG extraction is recommended if exact CAD coordinates are required.

Example:
    python extract_track_lines_wide.py \
        --input input_track_image.png \
        --output-csv track_lines_wide.csv \
        --preview extracted_lines_preview.png \
        --samples 500 \
        --scale-width-mm 16000 \
        --scale-height-mm 11000
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Tuple

import cv2
import numpy as np


def mask_red_lines(bgr: np.ndarray) -> np.ndarray:
    """Return a binary mask for red CAD lane lines."""
    # The source image uses thin bright red CAD lines on a white background.
    # We combine BGR and HSV thresholds to avoid picking black annotations or cyan dashes.
    b, g, r = cv2.split(bgr)
    bgr_mask = (r > 140) & (g < 170) & (b < 170) & ((r.astype(np.int16) - g.astype(np.int16)) > 35)

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    hsv_mask = (((h <= 10) | (h >= 170)) & (s > 40) & (v > 120))

    mask = (bgr_mask | hsv_mask).astype(np.uint8) * 255

    # Connect small gaps in the very thin CAD line.
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)
    return mask


def signed_area(points: np.ndarray) -> float:
    """Signed area of a closed polyline."""
    x = points[:, 0]
    y = points[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def contour_area_abs(points: np.ndarray) -> float:
    return abs(cv2.contourArea(points.astype(np.float32).reshape(-1, 1, 2)))


def extract_two_main_contours(mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Extract outer and inner red closed contours from the red mask."""
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if len(contours) < 2:
        raise RuntimeError(f"Could not find two red lane contours. Found {len(contours)} contour(s).")

    # Keep reasonably large contours only; text/dimensions, if captured, are much smaller.
    contour_pts = [c.reshape(-1, 2).astype(np.float64) for c in contours]
    contour_pts = [p for p in contour_pts if len(p) > 500]
    if len(contour_pts) < 2:
        raise RuntimeError("Red contours were found, but fewer than two large lane contours remained.")

    contour_pts.sort(key=contour_area_abs, reverse=True)
    outer = contour_pts[0]
    outer_area = contour_area_abs(outer)

    # The CAD image often has double red strokes around each lane boundary.
    # After thresholding, RETR_LIST returns several nested contours. The outer lane
    # group has the largest areas; the inner lane group is the next large group with
    # a much smaller area. Pick the largest contour whose area is clearly smaller
    # than the outer group.
    inner_candidates = [p for p in contour_pts[1:] if contour_area_abs(p) < 0.75 * outer_area]
    if not inner_candidates:
        inner_candidates = contour_pts[1:]
    inner = inner_candidates[0]

    # Ensure the larger-area contour is right_line / outer, smaller-area contour is left_line / inner.
    if contour_area_abs(inner) > contour_area_abs(outer):
        outer, inner = inner, outer
    return outer, inner


def rotate_start_near_reference(points: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Rotate closed polyline so index 0 is nearest to a reference point."""
    idx = int(np.argmin(np.linalg.norm(points - reference[None, :], axis=1)))
    return np.concatenate([points[idx:], points[:idx]], axis=0)


def ensure_same_orientation(a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Make two closed polylines run in the same orientation."""
    if np.sign(signed_area(a)) != np.sign(signed_area(b)):
        b = b[::-1].copy()
    return a, b


def resample_closed_polyline(points: np.ndarray, n: int) -> np.ndarray:
    """Uniformly resample a closed polyline by arc length."""
    if n < 3:
        raise ValueError("samples must be at least 3")

    pts = points.astype(np.float64)
    pts_closed = np.vstack([pts, pts[0]])
    seg = np.linalg.norm(np.diff(pts_closed, axis=0), axis=1)
    keep = seg > 1e-6
    pts_closed = pts_closed[np.r_[True, keep]]
    seg = np.linalg.norm(np.diff(pts_closed, axis=0), axis=1)

    cumulative = np.r_[0.0, np.cumsum(seg)]
    total = cumulative[-1]
    targets = np.linspace(0.0, total, n, endpoint=False)

    out = np.empty((n, 2), dtype=np.float64)
    for i, t in enumerate(targets):
        j = int(np.searchsorted(cumulative, t, side="right") - 1)
        j = min(j, len(seg) - 1)
        local = (t - cumulative[j]) / max(seg[j], 1e-12)
        out[i] = pts_closed[j] * (1.0 - local) + pts_closed[j + 1] * local
    return out


def align_and_sample(inner: np.ndarray, outer: np.ndarray, samples: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return matched inner(left), center, outer(right) samples."""
    outer, inner = ensure_same_orientation(outer, inner)

    # Use the bottom-most point of the outer line as a stable start reference.
    # The inner line starts from the nearest point to that outer reference, which makes seq roughly correspond.
    outer_ref = outer[np.argmax(outer[:, 1])]
    outer = rotate_start_near_reference(outer, outer_ref)
    inner = rotate_start_near_reference(inner, outer_ref)

    right = resample_closed_polyline(outer, samples)  # outer boundary
    left = resample_closed_polyline(inner, samples)   # inner boundary

    # If the first-pair distance is unexpectedly large, try reversing the inner line and keep the better alignment.
    inner_rev = rotate_start_near_reference(inner[::-1].copy(), outer_ref)
    left_rev = resample_closed_polyline(inner_rev, samples)
    if np.mean(np.linalg.norm(right - left_rev, axis=1)) < np.mean(np.linalg.norm(right - left, axis=1)):
        left = left_rev

    center = (left + right) / 2.0
    return left, center, right


def pixel_to_mm(points: np.ndarray, image_shape: Tuple[int, int, int], scale_width_mm: float, scale_height_mm: float) -> np.ndarray:
    """Convert image pixel coordinates to approximate drawing coordinates in mm.

    x_mm increases to the right. y_mm increases upward by default, so image y is inverted.
    """
    h, w = image_shape[:2]
    x_mm = points[:, 0] / (w - 1) * scale_width_mm
    y_mm = (1.0 - points[:, 1] / (h - 1)) * scale_height_mm
    return np.column_stack([x_mm, y_mm])


def save_wide_csv(path: Path, left: np.ndarray, center: np.ndarray, right: np.ndarray, image_shape, scale_width_mm: float, scale_height_mm: float) -> None:
    left_mm = pixel_to_mm(left, image_shape, scale_width_mm, scale_height_mm)
    center_mm = pixel_to_mm(center, image_shape, scale_width_mm, scale_height_mm)
    right_mm = pixel_to_mm(right, image_shape, scale_width_mm, scale_height_mm)

    header = [
        "seq",
        "left_x_px", "left_y_px", "left_x_mm", "left_y_mm",
        "center_x_px", "center_y_px", "center_x_mm", "center_y_mm",
        "right_x_px", "right_y_px", "right_x_mm", "right_y_mm",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i in range(len(left)):
            writer.writerow([
                i,
                f"{left[i,0]:.3f}", f"{left[i,1]:.3f}", f"{left_mm[i,0]:.3f}", f"{left_mm[i,1]:.3f}",
                f"{center[i,0]:.3f}", f"{center[i,1]:.3f}", f"{center_mm[i,0]:.3f}", f"{center_mm[i,1]:.3f}",
                f"{right[i,0]:.3f}", f"{right[i,1]:.3f}", f"{right_mm[i,0]:.3f}", f"{right_mm[i,1]:.3f}",
            ])


def save_preview(path: Path, bgr: np.ndarray, left: np.ndarray, center: np.ndarray, right: np.ndarray) -> None:
    preview = bgr.copy()

    # Fade background for readability.
    white = np.full_like(preview, 255)
    preview = cv2.addWeighted(preview, 0.35, white, 0.65, 0)

    def draw_line(points: np.ndarray, color: Tuple[int, int, int], thickness: int) -> None:
        pts = np.round(points).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(preview, [pts], isClosed=True, color=color, thickness=thickness, lineType=cv2.LINE_AA)

    # BGR colors: inner/left=blue, center=green, outer/right=red.
    draw_line(left, (200, 80, 20), 3)
    draw_line(center, (30, 150, 30), 3)
    draw_line(right, (30, 30, 220), 3)

    cv2.imwrite(str(path), preview)


def main() -> None:
    parser = argparse.ArgumentParser(description="Extract matched left/center/right track lines into wide-format CSV.")
    parser.add_argument("--input", required=True, help="Input PNG/JPG drawing image")
    parser.add_argument("--output-csv", default="track_lines_wide.csv", help="Output wide-format CSV")
    parser.add_argument("--preview", default="extracted_lines_preview.png", help="Output preview image")
    parser.add_argument("--samples", type=int, default=500, help="Number of matched samples per line")
    parser.add_argument("--scale-width-mm", type=float, default=16000.0, help="Approximate drawing width in mm")
    parser.add_argument("--scale-height-mm", type=float, default=11000.0, help="Approximate drawing height in mm")
    args = parser.parse_args()

    input_path = Path(args.input)
    bgr = cv2.imread(str(input_path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(f"Could not read image: {input_path}")

    mask = mask_red_lines(bgr)
    outer, inner = extract_two_main_contours(mask)
    left, center, right = align_and_sample(inner, outer, args.samples)

    save_wide_csv(Path(args.output_csv), left, center, right, bgr.shape, args.scale_width_mm, args.scale_height_mm)
    save_preview(Path(args.preview), bgr, left, center, right)

    print(f"Saved CSV: {args.output_csv}")
    print(f"Saved preview: {args.preview}")
    print("Definition: left_line=inner, center_line=midpoint, right_line=outer")


if __name__ == "__main__":
    main()
