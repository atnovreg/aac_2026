#!/usr/bin/env python3
"""
Extract parking-lot marking line segments from the toy track texture.

This is intentionally separate from extract_texture_track_lines_wide.py so the
closed-loop driving centerline CSV remains untouched. The output is segment
oriented: one row is one parking marking line, with pixel, drawing-mm, and map
coordinates for both endpoints.
"""

import argparse
import os
from dataclasses import dataclass

import cv2
import numpy as np
import pandas as pd


@dataclass
class AxisSegment:
    orientation: str
    line: float
    start: float
    end: float
    length: float
    evidence_count: int = 1


def parse_rgb(value: str) -> tuple[int, int, int]:
    parts = [int(part.strip()) for part in value.split(",")]
    if len(parts) != 3 or any(part < 0 or part > 255 for part in parts):
        raise argparse.ArgumentTypeError("RGB value must look like '108,110,109'.")
    return tuple(parts)


def mm_to_world(
    x_mm: float,
    y_mm: float,
    scale_width_mm: float,
    scale_height_mm: float,
    map_x_min: float,
    map_x_max: float,
    map_y_min: float,
    map_y_max: float,
    map_swap_xy: bool,
    map_flip_x: bool,
    map_flip_y: bool,
) -> tuple[float, float]:
    u = x_mm / scale_width_mm
    v = y_mm / scale_height_mm

    xn = v if map_swap_xy else u
    yn = u if map_swap_xy else v
    if map_flip_x:
        xn = 1.0 - xn
    if map_flip_y:
        yn = 1.0 - yn

    return (
        map_x_min + xn * (map_x_max - map_x_min),
        map_y_min + yn * (map_y_max - map_y_min),
    )


def px_to_mm(
    x_px: float,
    y_px: float,
    image_width: int,
    image_height: int,
    scale_width_mm: float,
    scale_height_mm: float,
) -> tuple[float, float]:
    return (
        x_px * scale_width_mm / float(image_width),
        y_px * scale_height_mm / float(image_height),
    )


def build_parking_surface_mask(
    image_bgr: np.ndarray,
    parking_gray_rgb: tuple[int, int, int],
    gray_distance: float,
    close_kernel: int,
    open_kernel: int,
) -> np.ndarray:
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    target = np.array(parking_gray_rgb, dtype=np.int16)
    dist = np.linalg.norm(image_rgb.astype(np.int16) - target, axis=2)
    mask = (dist <= gray_distance).astype("uint8") * 255

    if close_kernel > 1:
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_kernel, close_kernel)),
            iterations=2,
        )
    if open_kernel > 1:
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_kernel, open_kernel)),
            iterations=1,
        )

    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if num < 2:
        raise RuntimeError("No parking-lot gray surface found. Try increasing --gray-distance.")

    main_label = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return (labels == main_label).astype("uint8") * 255


def detect_axis_segments(
    image_bgr: np.ndarray,
    parking_surface: np.ndarray,
    roi_dilation_kernel: int,
    roi_dilation_iterations: int,
    white_saturation_max: int,
    white_value_min: int,
    hough_threshold: int,
    min_line_length: float,
    max_line_gap: float,
    axis_angle_tolerance_deg: float,
) -> tuple[list[AxisSegment], np.ndarray, np.ndarray]:
    roi = cv2.dilate(
        parking_surface,
        cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (roi_dilation_kernel, roi_dilation_kernel)),
        iterations=roi_dilation_iterations,
    )

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    _, saturation, value = cv2.split(hsv)
    white = ((saturation <= white_saturation_max) & (value >= white_value_min)).astype("uint8") * 255
    white = cv2.bitwise_and(white, roi)
    white = cv2.morphologyEx(
        white,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
        iterations=1,
    )

    edges = cv2.Canny(white, 50, 150)
    hough = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180.0,
        threshold=hough_threshold,
        minLineLength=int(round(min_line_length)),
        maxLineGap=int(round(max_line_gap)),
    )

    segments: list[AxisSegment] = []
    if hough is None:
        return segments, white, roi

    for x1, y1, x2, y2 in hough[:, 0, :].astype(float):
        dx = x2 - x1
        dy = y2 - y1
        length = float(np.hypot(dx, dy))
        if length < min_line_length:
            continue

        angle = (np.degrees(np.arctan2(dy, dx)) + 180.0) % 180.0
        horizontal_error = min(abs(angle), abs(angle - 180.0))
        vertical_error = abs(angle - 90.0)

        if horizontal_error <= axis_angle_tolerance_deg:
            start, end = sorted((x1, x2))
            segments.append(AxisSegment("horizontal", (y1 + y2) * 0.5, start, end, end - start))
        elif vertical_error <= axis_angle_tolerance_deg:
            start, end = sorted((y1, y2))
            segments.append(AxisSegment("vertical", (x1 + x2) * 0.5, start, end, end - start))

    return segments, white, roi


def merge_axis_segments(
    raw_segments: list[AxisSegment],
    merge_distance_px: float,
    merge_gap_px: float,
    min_merged_length_px: float,
) -> list[AxisSegment]:
    merged: list[AxisSegment] = []

    for orientation in ("horizontal", "vertical"):
        candidates = sorted(
            [seg for seg in raw_segments if seg.orientation == orientation],
            key=lambda seg: (seg.line, seg.start, seg.end),
        )

        clusters: list[list[AxisSegment]] = []
        for segment in candidates:
            matched_cluster = None
            for cluster in clusters:
                line_values = [item.line for item in cluster]
                start_values = [item.start for item in cluster]
                end_values = [item.end for item in cluster]
                cluster_line = float(np.mean(line_values))
                cluster_start = min(start_values)
                cluster_end = max(end_values)
                overlaps_or_touches = segment.start <= cluster_end + merge_gap_px and (
                    segment.end + merge_gap_px >= cluster_start)
                if abs(segment.line - cluster_line) <= merge_distance_px and overlaps_or_touches:
                    matched_cluster = cluster
                    break

            if matched_cluster is None:
                clusters.append([segment])
            else:
                matched_cluster.append(segment)

        for cluster in clusters:
            weights = np.array([max(item.length, 1.0) for item in cluster], dtype=float)
            line = float(np.average([item.line for item in cluster], weights=weights))
            start = float(min(item.start for item in cluster))
            end = float(max(item.end for item in cluster))
            length = end - start
            if length >= min_merged_length_px:
                merged.append(AxisSegment(
                    orientation=orientation,
                    line=line,
                    start=start,
                    end=end,
                    length=length,
                    evidence_count=len(cluster),
                ))

    return sorted(
        merged,
        key=lambda seg: (
            0 if seg.orientation == "horizontal" else 1,
            seg.line,
            seg.start,
            seg.end,
        ),
    )


def endpoints_for_segment(segment: AxisSegment) -> tuple[float, float, float, float]:
    if segment.orientation == "horizontal":
        return segment.start, segment.line, segment.end, segment.line
    return segment.line, segment.start, segment.line, segment.end


def infer_line_role(segment: AxisSegment, cx_px: float, cy_px: float) -> str:
    if segment.orientation == "horizontal":
        return "top_slot_header" if cy_px < 520.0 else "bottom_slot_edge"
    if segment.length >= 250.0:
        return "parking_lot_boundary"
    if cx_px > 1250.0:
        return "right_approach_marking"
    if cx_px > 1120.0 and 450.0 <= cy_px <= 680.0:
        return "entry_direction_marking"
    if cy_px < 560.0:
        return "top_slot_separator"
    if cy_px > 620.0:
        return "bottom_slot_separator"
    return "parking_marking"


def segments_to_dataframe(
    segments: list[AxisSegment],
    image_width: int,
    image_height: int,
    scale_width_mm: float,
    scale_height_mm: float,
    map_x_min: float,
    map_x_max: float,
    map_y_min: float,
    map_y_max: float,
    map_swap_xy: bool,
    map_flip_x: bool,
    map_flip_y: bool,
) -> pd.DataFrame:
    rows = []
    for line_id, segment in enumerate(segments):
        x1_px, y1_px, x2_px, y2_px = endpoints_for_segment(segment)
        cx_px = (x1_px + x2_px) * 0.5
        cy_px = (y1_px + y2_px) * 0.5
        x1_mm, y1_mm = px_to_mm(
            x1_px, y1_px, image_width, image_height, scale_width_mm, scale_height_mm)
        x2_mm, y2_mm = px_to_mm(
            x2_px, y2_px, image_width, image_height, scale_width_mm, scale_height_mm)
        cx_mm, cy_mm = px_to_mm(
            cx_px, cy_px, image_width, image_height, scale_width_mm, scale_height_mm)
        x1_m, y1_m = mm_to_world(
            x1_mm, y1_mm, scale_width_mm, scale_height_mm,
            map_x_min, map_x_max, map_y_min, map_y_max,
            map_swap_xy, map_flip_x, map_flip_y)
        x2_m, y2_m = mm_to_world(
            x2_mm, y2_mm, scale_width_mm, scale_height_mm,
            map_x_min, map_x_max, map_y_min, map_y_max,
            map_swap_xy, map_flip_x, map_flip_y)
        cx_m, cy_m = mm_to_world(
            cx_mm, cy_mm, scale_width_mm, scale_height_mm,
            map_x_min, map_x_max, map_y_min, map_y_max,
            map_swap_xy, map_flip_x, map_flip_y)
        length_px = float(np.hypot(x2_px - x1_px, y2_px - y1_px))
        length_mm = float(np.hypot(x2_mm - x1_mm, y2_mm - y1_mm))
        length_m = float(np.hypot(x2_m - x1_m, y2_m - y1_m))
        angle_deg = float(np.degrees(np.arctan2(y2_px - y1_px, x2_px - x1_px)))
        line_role = infer_line_role(segment, cx_px, cy_px)

        rows.append({
            "line_id": line_id,
            "line_type": "parking_marking",
            "line_role": line_role,
            "orientation": segment.orientation,
            "x1_px": x1_px,
            "y1_px": y1_px,
            "x2_px": x2_px,
            "y2_px": y2_px,
            "center_x_px": cx_px,
            "center_y_px": cy_px,
            "x1_mm": x1_mm,
            "y1_mm": y1_mm,
            "x2_mm": x2_mm,
            "y2_mm": y2_mm,
            "center_x_mm": cx_mm,
            "center_y_mm": cy_mm,
            "x1_m": x1_m,
            "y1_m": y1_m,
            "x2_m": x2_m,
            "y2_m": y2_m,
            "center_x_m": cx_m,
            "center_y_m": cy_m,
            "length_px": length_px,
            "length_mm": length_mm,
            "length_m": length_m,
            "angle_deg": angle_deg,
            "evidence_count": segment.evidence_count,
        })
    return pd.DataFrame(rows)


def draw_preview(
    image_bgr: np.ndarray,
    roi: np.ndarray,
    white: np.ndarray,
    segments: list[AxisSegment],
    preview_path: str,
) -> None:
    preview = image_bgr.copy()
    green_overlay = np.zeros_like(preview)
    green_overlay[:, :, 1] = 120
    preview = np.where(
        roi[:, :, None] > 0,
        cv2.addWeighted(preview, 0.72, green_overlay, 0.28, 0),
        preview,
    )

    white_overlay = np.zeros_like(preview)
    white_overlay[:, :] = (220, 220, 220)
    preview = np.where(
        white[:, :, None] > 0,
        cv2.addWeighted(preview, 0.35, white_overlay, 0.65, 0),
        preview,
    )

    for line_id, segment in enumerate(segments):
        x1, y1, x2, y2 = endpoints_for_segment(segment)
        color = (0, 255, 255) if segment.orientation == "horizontal" else (255, 0, 255)
        p1 = (int(round(x1)), int(round(y1)))
        p2 = (int(round(x2)), int(round(y2)))
        cv2.line(preview, p1, p2, color, 3, cv2.LINE_AA)
        cx = int(round((x1 + x2) * 0.5))
        cy = int(round((y1 + y2) * 0.5))
        cv2.circle(preview, (cx, cy), 4, color, -1, cv2.LINE_AA)
        cv2.putText(
            preview,
            str(line_id),
            (cx + 6, cy - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            preview,
            str(line_id),
            (cx + 6, cy - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    legend = [
        ("parking ROI", (0, 120, 0)),
        ("horizontal marking", (0, 255, 255)),
        ("vertical marking", (255, 0, 255)),
    ]
    y = 28
    for text, color in legend:
        cv2.rectangle(preview, (18, y - 15), (40, y + 5), color, -1)
        cv2.putText(preview, text, (48, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(preview, text, (48, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA)
        y += 26

    os.makedirs(os.path.dirname(os.path.abspath(preview_path)) or ".", exist_ok=True)
    cv2.imwrite(preview_path, preview)


def extract_parking_lines(
    input_path: str,
    output_csv: str,
    preview_path: str | None,
    parking_gray_rgb: tuple[int, int, int],
    gray_distance: float,
    scale_width_mm: float,
    scale_height_mm: float,
    map_x_min: float,
    map_x_max: float,
    map_y_min: float,
    map_y_max: float,
    map_swap_xy: bool,
    map_flip_x: bool,
    map_flip_y: bool,
    white_saturation_max: int,
    white_value_min: int,
    hough_threshold: int,
    min_line_length: float,
    max_line_gap: float,
    axis_angle_tolerance_deg: float,
    merge_distance_px: float,
    merge_gap_px: float,
    min_merged_length_px: float,
) -> pd.DataFrame:
    image = cv2.imread(input_path)
    if image is None:
        raise FileNotFoundError(input_path)

    height, width = image.shape[:2]
    parking_surface = build_parking_surface_mask(
        image,
        parking_gray_rgb=parking_gray_rgb,
        gray_distance=gray_distance,
        close_kernel=17,
        open_kernel=7,
    )
    raw_segments, white, roi = detect_axis_segments(
        image,
        parking_surface=parking_surface,
        roi_dilation_kernel=17,
        roi_dilation_iterations=2,
        white_saturation_max=white_saturation_max,
        white_value_min=white_value_min,
        hough_threshold=hough_threshold,
        min_line_length=min_line_length,
        max_line_gap=max_line_gap,
        axis_angle_tolerance_deg=axis_angle_tolerance_deg,
    )
    merged_segments = merge_axis_segments(
        raw_segments,
        merge_distance_px=merge_distance_px,
        merge_gap_px=merge_gap_px,
        min_merged_length_px=min_merged_length_px,
    )

    df = segments_to_dataframe(
        merged_segments,
        image_width=width,
        image_height=height,
        scale_width_mm=scale_width_mm,
        scale_height_mm=scale_height_mm,
        map_x_min=map_x_min,
        map_x_max=map_x_max,
        map_y_min=map_y_min,
        map_y_max=map_y_max,
        map_swap_xy=map_swap_xy,
        map_flip_x=map_flip_x,
        map_flip_y=map_flip_y,
    )

    os.makedirs(os.path.dirname(os.path.abspath(output_csv)) or ".", exist_ok=True)
    df.to_csv(output_csv, index=False)

    if preview_path:
        draw_preview(image, roi, white, merged_segments, preview_path)

    return df


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="input_track_texture.png")
    parser.add_argument("--output-csv", default="parking_lot_lines.csv")
    parser.add_argument("--preview", default="parking_lot_lines_preview.png")
    parser.add_argument("--parking-gray-rgb", type=parse_rgb, default=(108, 110, 109))
    parser.add_argument("--gray-distance", type=float, default=18.0)
    parser.add_argument("--scale-width-mm", type=float, default=16000.0)
    parser.add_argument("--scale-height-mm", type=float, default=11000.0)
    parser.add_argument("--map-x-min", type=float, default=-5.5)
    parser.add_argument("--map-x-max", type=float, default=5.5)
    parser.add_argument("--map-y-min", type=float, default=-8.0)
    parser.add_argument("--map-y-max", type=float, default=8.0)
    parser.add_argument("--map-swap-xy", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--map-flip-x", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--map-flip-y", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--white-saturation-max", type=int, default=45)
    parser.add_argument("--white-value-min", type=int, default=185)
    parser.add_argument("--hough-threshold", type=int, default=35)
    parser.add_argument("--min-line-length", type=float, default=60.0)
    parser.add_argument("--max-line-gap", type=float, default=8.0)
    parser.add_argument("--axis-angle-tolerance-deg", type=float, default=4.0)
    parser.add_argument("--merge-distance-px", type=float, default=10.0)
    parser.add_argument("--merge-gap-px", type=float, default=25.0)
    parser.add_argument("--min-merged-length-px", type=float, default=65.0)
    args = parser.parse_args()

    df = extract_parking_lines(
        input_path=args.input,
        output_csv=args.output_csv,
        preview_path=args.preview,
        parking_gray_rgb=args.parking_gray_rgb,
        gray_distance=args.gray_distance,
        scale_width_mm=args.scale_width_mm,
        scale_height_mm=args.scale_height_mm,
        map_x_min=args.map_x_min,
        map_x_max=args.map_x_max,
        map_y_min=args.map_y_min,
        map_y_max=args.map_y_max,
        map_swap_xy=args.map_swap_xy,
        map_flip_x=args.map_flip_x,
        map_flip_y=args.map_flip_y,
        white_saturation_max=args.white_saturation_max,
        white_value_min=args.white_value_min,
        hough_threshold=args.hough_threshold,
        min_line_length=args.min_line_length,
        max_line_gap=args.max_line_gap,
        axis_angle_tolerance_deg=args.axis_angle_tolerance_deg,
        merge_distance_px=args.merge_distance_px,
        merge_gap_px=args.merge_gap_px,
        min_merged_length_px=args.min_merged_length_px,
    )

    print(f"Saved: {args.output_csv}")
    if args.preview:
        print(f"Saved: {args.preview}")
    print(f"Rows: {len(df)}")
    if not df.empty:
        print("Orientation counts:")
        print(df["orientation"].value_counts().to_string())
        print(
            "World bounds: "
            f"x=[{df[['x1_m', 'x2_m']].min().min():.3f}, {df[['x1_m', 'x2_m']].max().max():.3f}], "
            f"y=[{df[['y1_m', 'y2_m']].min().min():.3f}, {df[['y1_m', 'y2_m']].max().max():.3f}]"
        )
    print("Columns:", ", ".join(df.columns))


if __name__ == "__main__":
    main()
