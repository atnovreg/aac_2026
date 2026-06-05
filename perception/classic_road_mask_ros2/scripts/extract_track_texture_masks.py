#!/usr/bin/env python3
"""Extract drivable-road and lane-marking masks from the Gazebo track texture."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


DEFAULT_TEXTURE = (
    Path(__file__).resolve().parents[3]
    / "system"
    / "simple_gazebo_track"
    / "models"
    / "simple_track"
    / "materials"
    / "textures"
    / "track.png"
)


def hsv_triplet(value: str) -> tuple[int, int, int]:
    parts = [int(part.strip()) for part in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("HSV values must be H,S,V")
    h, s, v = parts
    if not (0 <= h <= 179 and 0 <= s <= 255 and 0 <= v <= 255):
        raise argparse.ArgumentTypeError("HSV ranges are H=0..179, S/V=0..255")
    return h, s, v


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create mask images from simple_track/materials/textures/track.png.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--texture", default=str(DEFAULT_TEXTURE))
    parser.add_argument("--output-dir", default="/tmp/aac_track_masks")
    parser.add_argument("--world-size-x", type=float, default=12.0)
    parser.add_argument("--world-size-y", type=float, default=16.0)
    parser.add_argument("--road-hsv-lower", type=hsv_triplet, default=(0, 0, 25))
    parser.add_argument("--road-hsv-upper", type=hsv_triplet, default=(179, 90, 125))
    parser.add_argument("--lane-hsv-lower", type=hsv_triplet, default=(0, 0, 165))
    parser.add_argument("--lane-hsv-upper", type=hsv_triplet, default=(179, 80, 255))
    parser.add_argument("--kernel-size", type=int, default=5)
    parser.add_argument("--min-road-area", type=int, default=1000)
    parser.add_argument("--min-lane-area", type=int, default=20)
    parser.add_argument("--alpha", type=float, default=0.45)
    args = parser.parse_args(argv)
    args.kernel_size = max(1, args.kernel_size)
    if args.kernel_size % 2 == 0:
        args.kernel_size += 1
    args.alpha = min(1.0, max(0.0, args.alpha))
    return args


def remove_small_components(mask: np.ndarray, min_area: int) -> np.ndarray:
    if min_area <= 0:
        return mask
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    cleaned = np.zeros_like(mask)
    for label in range(1, num_labels):
        if stats[label, cv2.CC_STAT_AREA] >= min_area:
            cleaned[labels == label] = 255
    return cleaned


def clean_mask(mask: np.ndarray, kernel_size: int, min_area: int) -> np.ndarray:
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return remove_small_components(mask, min_area)


def make_overlay(
    bgr: np.ndarray,
    drivable_mask: np.ndarray,
    lane_mask: np.ndarray,
    alpha: float,
) -> np.ndarray:
    overlay = bgr.copy()
    road_color = np.array([80, 190, 80], dtype=np.uint8)
    lane_color = np.array([0, 220, 255], dtype=np.uint8)

    road_pixels = drivable_mask > 0
    lane_pixels = lane_mask > 0
    if road_pixels.any():
        overlay[road_pixels] = (
            overlay[road_pixels].astype(np.float32) * (1.0 - alpha)
            + road_color.astype(np.float32) * alpha
        ).astype(np.uint8)
    if lane_pixels.any():
        overlay[lane_pixels] = (
            overlay[lane_pixels].astype(np.float32) * 0.25
            + lane_color.astype(np.float32) * 0.75
        ).astype(np.uint8)
    return overlay


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv)
    texture_path = Path(args.texture).expanduser()
    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    bgr = cv2.imread(str(texture_path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError(f"Failed to read texture: {texture_path}")

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    road_surface = cv2.inRange(
        hsv,
        np.array(args.road_hsv_lower, dtype=np.uint8),
        np.array(args.road_hsv_upper, dtype=np.uint8),
    )
    lane_mask = cv2.inRange(
        hsv,
        np.array(args.lane_hsv_lower, dtype=np.uint8),
        np.array(args.lane_hsv_upper, dtype=np.uint8),
    )
    road_surface = clean_mask(road_surface, args.kernel_size, args.min_road_area)
    lane_mask = clean_mask(lane_mask, max(3, args.kernel_size), args.min_lane_area)
    drivable_mask = cv2.bitwise_or(road_surface, lane_mask)
    overlay = make_overlay(bgr, drivable_mask, lane_mask, args.alpha)

    cv2.imwrite(str(output_dir / "drivable_mask.png"), drivable_mask)
    cv2.imwrite(str(output_dir / "road_surface_mask.png"), road_surface)
    cv2.imwrite(str(output_dir / "lane_markings_mask.png"), lane_mask)
    cv2.imwrite(str(output_dir / "overlay.png"), overlay)

    height, width = bgr.shape[:2]
    metadata = {
        "texture": str(texture_path),
        "width_px": width,
        "height_px": height,
        "world_size_x_m": args.world_size_x,
        "world_size_y_m": args.world_size_y,
        "meters_per_pixel_x": args.world_size_x / width,
        "meters_per_pixel_y": args.world_size_y / height,
        "pixel_to_world": {
            "x_m": "(u / width_px - 0.5) * world_size_x_m",
            "y_m": "(0.5 - v / height_px) * world_size_y_m",
            "note": "u is image column from left, v is image row from top.",
        },
        "road_hsv_lower": args.road_hsv_lower,
        "road_hsv_upper": args.road_hsv_upper,
        "lane_hsv_lower": args.lane_hsv_lower,
        "lane_hsv_upper": args.lane_hsv_upper,
        "drivable_pixels": int(np.count_nonzero(drivable_mask)),
        "road_surface_pixels": int(np.count_nonzero(road_surface)),
        "lane_pixels": int(np.count_nonzero(lane_mask)),
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")

    print(json.dumps(metadata, indent=2), flush=True)
    print(f"Saved masks to {output_dir}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
