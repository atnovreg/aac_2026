#!/usr/bin/env python3
"""
Extract matched left / center / right track lines from a textured track image.

Conventions used here:
- left_line  = inner boundary of the road track
- right_line = outer boundary of the road track
- center_line = point-wise midpoint between left_line and right_line
- wide CSV format: one row per seq, so left/center/right share the same seq index.

This script is designed for the provided toy track texture image where the road is
dark asphalt and the central area/grass are brighter.
"""

import argparse
import os
import cv2
import numpy as np
import pandas as pd


def contour_signed_area(poly: np.ndarray) -> float:
    x = poly[:, 0]
    y = poly[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def rotate_to_anchor(poly: np.ndarray, anchor: np.ndarray) -> np.ndarray:
    idx = int(np.argmin(np.sum((poly - anchor) ** 2, axis=1)))
    return np.vstack([poly[idx:], poly[:idx]])


def resample_closed(poly: np.ndarray, n: int) -> np.ndarray:
    pts = np.asarray(poly, dtype=float)
    if np.linalg.norm(pts[0] - pts[-1]) > 1e-9:
        pts = np.vstack([pts, pts[0]])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.r_[0.0, np.cumsum(seg)]
    total = float(cum[-1])
    if total <= 0:
        raise RuntimeError("Contour length is zero.")
    targets = np.linspace(0.0, total, n, endpoint=False)
    x = np.interp(targets, cum, pts[:, 0])
    y = np.interp(targets, cum, pts[:, 1])
    return np.c_[x, y]


def periodic_smooth(pts: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return pts
    if window % 2 == 0:
        window += 1
    pad = window // 2
    kernel = np.ones(window, dtype=float) / float(window)
    out = []
    for dim in range(2):
        arr = np.r_[pts[-pad:, dim], pts[:, dim], pts[:pad, dim]]
        out.append(np.convolve(arr, kernel, mode="valid"))
    return np.c_[out[0], out[1]]


def extract_track_lines(
    input_path: str,
    output_csv: str,
    preview_path: str | None,
    samples: int,
    scale_width_mm: float,
    scale_height_mm: float,
    dark_threshold: int,
    smooth_window: int,
) -> pd.DataFrame:
    image = cv2.imread(input_path)
    if image is None:
        raise FileNotFoundError(input_path)

    height, width = image.shape[:2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 1) Segment dark asphalt road.
    mask = (gray < dark_threshold).astype("uint8") * 255
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)),
        iterations=2,
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        iterations=1,
    )

    # 2) Keep the largest connected component as the track road.
    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if num < 2:
        raise RuntimeError("No dark road component found. Try increasing --dark-threshold.")
    road_label = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    road = (labels == road_label).astype("uint8") * 255
    road = cv2.morphologyEx(
        road,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)),
        iterations=2,
    )

    # 3) Extract outer road contour and largest inner-hole contour.
    contours, hierarchy = cv2.findContours(road, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None or len(contours) < 2:
        raise RuntimeError("Could not find both outer and inner contours.")

    info = []
    for idx, contour in enumerate(contours):
        parent = int(hierarchy[0, idx, 3])
        area = abs(cv2.contourArea(contour))
        info.append((idx, parent, area))

    outer_idx = max(
        [idx for idx, parent, _ in info if parent == -1],
        key=lambda idx: abs(cv2.contourArea(contours[idx])),
    )
    children = [idx for idx, parent, _ in info if parent == outer_idx]
    if not children:
        raise RuntimeError("Outer contour has no inner hole contour.")
    inner_idx = max(children, key=lambda idx: abs(cv2.contourArea(contours[idx])))

    right_poly = contours[outer_idx][:, 0, :].astype(float)  # outer = right_line
    left_poly = contours[inner_idx][:, 0, :].astype(float)   # inner = left_line

    # 4) Match direction and anchor both contours near the left-middle start area.
    if np.sign(contour_signed_area(right_poly)) != np.sign(contour_signed_area(left_poly)):
        left_poly = left_poly[::-1]

    anchor_y = float((right_poly[:, 1].mean() + left_poly[:, 1].mean()) / 2.0)
    right_poly = rotate_to_anchor(right_poly, np.array([right_poly[:, 0].min(), anchor_y]))
    left_poly = rotate_to_anchor(left_poly, np.array([left_poly[:, 0].min(), anchor_y]))

    right = resample_closed(right_poly, samples)
    left = resample_closed(left_poly, samples)
    right = periodic_smooth(right, smooth_window)
    left = periodic_smooth(left, smooth_window)
    center = (left + right) / 2.0

    sx = scale_width_mm / float(width)
    sy = scale_height_mm / float(height)
    rows = []
    for seq in range(samples):
        lx, ly = left[seq]
        cx, cy = center[seq]
        rx, ry = right[seq]
        lane_width_px = float(np.linalg.norm(right[seq] - left[seq]))
        lane_width_mm = float(
            np.linalg.norm(np.array([rx * sx, ry * sy]) - np.array([lx * sx, ly * sy]))
        )
        rows.append(
            {
                "seq": seq,
                "left_x_px": lx,
                "left_y_px": ly,
                "center_x_px": cx,
                "center_y_px": cy,
                "right_x_px": rx,
                "right_y_px": ry,
                "left_x_mm": lx * sx,
                "left_y_mm": ly * sy,
                "center_x_mm": cx * sx,
                "center_y_mm": cy * sy,
                "right_x_mm": rx * sx,
                "right_y_mm": ry * sy,
                "lane_width_px": lane_width_px,
                "lane_width_mm": lane_width_mm,
            }
        )

    df = pd.DataFrame(rows)
    os.makedirs(os.path.dirname(os.path.abspath(output_csv)) or ".", exist_ok=True)
    df.to_csv(output_csv, index=False)

    if preview_path:
        preview = image.copy()

        def draw_poly(points: np.ndarray, color: tuple[int, int, int], thickness: int) -> None:
            pts = np.round(points).astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(preview, [pts], True, color, thickness, cv2.LINE_AA)

        # BGR colors
        draw_poly(left, (255, 0, 0), 4)      # blue
        draw_poly(center, (0, 255, 255), 3)  # yellow
        draw_poly(right, (0, 0, 255), 4)     # red

        for seq in range(0, samples, max(1, samples // 10)):
            for points, color in [(left, (255, 0, 0)), (center, (0, 255, 255)), (right, (0, 0, 255))]:
                x, y = np.round(points[seq]).astype(int)
                cv2.circle(preview, (x, y), 5, color, -1, cv2.LINE_AA)
            x, y = np.round(center[seq]).astype(int)
            cv2.putText(preview, str(seq), (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(preview, str(seq), (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)

        legend = [
            ("left_line = inner boundary", (255, 0, 0)),
            ("center_line = midpoint", (0, 255, 255)),
            ("right_line = outer boundary", (0, 0, 255)),
        ]
        y = 30
        for text, color in legend:
            cv2.rectangle(preview, (20, y - 15), (42, y + 5), color, -1)
            cv2.putText(preview, text, (50, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(preview, text, (50, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 1, cv2.LINE_AA)
            y += 28

        os.makedirs(os.path.dirname(os.path.abspath(preview_path)) or ".", exist_ok=True)
        cv2.imwrite(preview_path, preview)

    return df


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Input track texture image path")
    parser.add_argument("--output-csv", default="track_texture_lines_wide.csv")
    parser.add_argument("--preview", default="track_texture_lines_preview.png")
    parser.add_argument("--samples", type=int, default=500)
    parser.add_argument("--scale-width-mm", type=float, default=16000.0)
    parser.add_argument("--scale-height-mm", type=float, default=11000.0)
    parser.add_argument("--dark-threshold", type=int, default=95)
    parser.add_argument("--smooth-window", type=int, default=5)
    args = parser.parse_args()

    df = extract_track_lines(
        input_path=args.input,
        output_csv=args.output_csv,
        preview_path=args.preview,
        samples=args.samples,
        scale_width_mm=args.scale_width_mm,
        scale_height_mm=args.scale_height_mm,
        dark_threshold=args.dark_threshold,
        smooth_window=args.smooth_window,
    )
    print(f"Saved: {args.output_csv}")
    if args.preview:
        print(f"Saved: {args.preview}")
    print(f"Rows: {len(df)}")
    print(f"Seq range: {int(df['seq'].min())} - {int(df['seq'].max())}")
    print("Columns:", ", ".join(df.columns))


if __name__ == "__main__":
    main()
