#!/usr/bin/env python3
"""Extract drivable-road and lane-marking masks from a ROS2 image topic."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String


def hsv_triplet(value: str) -> tuple[int, int, int]:
    parts = [int(part.strip()) for part in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("HSV values must be H,S,V")
    h, s, v = parts
    if not (0 <= h <= 179 and 0 <= s <= 255 and 0 <= v <= 255):
        raise argparse.ArgumentTypeError("HSV ranges are H=0..179, S/V=0..255")
    return h, s, v


def parse_args(argv: Iterable[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Publish drivable-road and lane-marking masks from camera images.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--image-topic", default="/camera_front/image_raw")
    parser.add_argument("--drivable-topic", default="/road_mask/drivable")
    parser.add_argument("--lane-topic", default="/road_mask/lane_markings")
    parser.add_argument("--overlay-topic", default="/road_mask/overlay")
    parser.add_argument("--stats-topic", default="/road_mask/stats")
    parser.add_argument("--queue-size", type=int, default=1)
    parser.add_argument("--reliable", action="store_true")
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--min-interval", type=float, default=0.0)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--roi-y-min-ratio", type=float, default=0.25)
    parser.add_argument("--roi-y-max-ratio", type=float, default=1.0)
    parser.add_argument("--road-hsv-lower", type=hsv_triplet, default=(0, 0, 25))
    parser.add_argument("--road-hsv-upper", type=hsv_triplet, default=(179, 90, 125))
    parser.add_argument("--lane-hsv-lower", type=hsv_triplet, default=(0, 0, 165))
    parser.add_argument("--lane-hsv-upper", type=hsv_triplet, default=(179, 80, 255))
    parser.add_argument("--kernel-size", type=int, default=5)
    parser.add_argument("--open-iterations", type=int, default=1)
    parser.add_argument("--close-iterations", type=int, default=2)
    parser.add_argument("--min-road-area", type=int, default=800)
    parser.add_argument("--min-lane-area", type=int, default=40)
    parser.add_argument("--alpha", type=float, default=0.45)
    parser.add_argument("--save-output-dir", default=None)
    parser.add_argument("--save-every", type=int, default=0)
    parser.add_argument(
        "--image-file",
        default=None,
        help="Run a one-shot non-ROS smoke test on a BGR/RGB image file.",
    )
    args, ros_args = parser.parse_known_args(argv)
    args.frame_stride = max(1, args.frame_stride)
    args.queue_size = max(1, args.queue_size)
    args.kernel_size = max(1, args.kernel_size)
    if args.kernel_size % 2 == 0:
        args.kernel_size += 1
    args.roi_y_min_ratio = min(1.0, max(0.0, args.roi_y_min_ratio))
    args.roi_y_max_ratio = min(1.0, max(args.roi_y_min_ratio, args.roi_y_max_ratio))
    args.alpha = min(1.0, max(0.0, args.alpha))
    return args, ros_args


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    channels_by_encoding = {
        "bgr8": 3,
        "rgb8": 3,
        "bgra8": 4,
        "rgba8": 4,
        "mono8": 1,
        "8uc1": 1,
        "8uc3": 3,
        "8uc4": 4,
    }
    channels = channels_by_encoding.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    data = np.frombuffer(msg.data, dtype=np.uint8)
    if channels == 1:
        rows = data.reshape((msg.height, msg.step))
        gray = rows[:, : msg.width].copy()
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    row_pixels = msg.width * channels
    rows = data.reshape((msg.height, msg.step))
    image = rows[:, :row_pixels].reshape((msg.height, msg.width, channels)).copy()

    if encoding in ("bgr8", "8uc3"):
        return image
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if encoding in ("rgba8", "8uc4"):
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def cv2_to_image_msg(array: np.ndarray, encoding: str, header) -> Image:
    if not array.flags["C_CONTIGUOUS"]:
        array = np.ascontiguousarray(array)

    msg = Image()
    msg.header = header
    msg.height = int(array.shape[0])
    msg.width = int(array.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    channels = 1 if array.ndim == 2 else int(array.shape[2])
    msg.step = int(msg.width * channels * array.dtype.itemsize)
    msg.data = array.tobytes()
    return msg


def remove_small_components(mask: np.ndarray, min_area: int) -> np.ndarray:
    if min_area <= 0:
        return mask
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    cleaned = np.zeros_like(mask)
    for label in range(1, num_labels):
        if stats[label, cv2.CC_STAT_AREA] >= min_area:
            cleaned[labels == label] = 255
    return cleaned


def roi_mask(height: int, width: int, y_min_ratio: float, y_max_ratio: float) -> np.ndarray:
    mask = np.zeros((height, width), dtype=np.uint8)
    y0 = int(round(height * y_min_ratio))
    y1 = int(round(height * y_max_ratio))
    mask[y0:y1, :] = 255
    return mask


def clean_mask(
    mask: np.ndarray,
    kernel_size: int,
    open_iterations: int,
    close_iterations: int,
    min_area: int,
) -> np.ndarray:
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    if open_iterations > 0:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=open_iterations)
    if close_iterations > 0:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=close_iterations)
    return remove_small_components(mask, min_area)


def extract_masks(
    bgr: np.ndarray,
    args: argparse.Namespace,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    height, width = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    roi = roi_mask(height, width, args.roi_y_min_ratio, args.roi_y_max_ratio)

    road_mask = cv2.inRange(
        hsv,
        np.array(args.road_hsv_lower, dtype=np.uint8),
        np.array(args.road_hsv_upper, dtype=np.uint8),
    )
    lane_mask = cv2.inRange(
        hsv,
        np.array(args.lane_hsv_lower, dtype=np.uint8),
        np.array(args.lane_hsv_upper, dtype=np.uint8),
    )

    road_mask = cv2.bitwise_and(road_mask, roi)
    lane_mask = cv2.bitwise_and(lane_mask, roi)
    road_mask = clean_mask(
        road_mask,
        args.kernel_size,
        args.open_iterations,
        args.close_iterations,
        args.min_road_area,
    )
    lane_mask = clean_mask(
        lane_mask,
        max(3, args.kernel_size),
        0,
        1,
        args.min_lane_area,
    )

    drivable_mask = cv2.bitwise_or(road_mask, lane_mask)
    overlay = make_overlay(bgr, drivable_mask, lane_mask, args.alpha)
    stats = {
        "width": width,
        "height": height,
        "drivable_pixels": int(np.count_nonzero(drivable_mask)),
        "lane_pixels": int(np.count_nonzero(lane_mask)),
        "drivable_ratio": float(np.count_nonzero(drivable_mask) / (height * width)),
        "lane_ratio": float(np.count_nonzero(lane_mask) / (height * width)),
    }
    return drivable_mask, lane_mask, overlay, stats


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


def stats_json(header, image_topic: str, stats: dict, elapsed_ms: float) -> str:
    payload = {
        "header": {
            "stamp": {
                "sec": int(header.stamp.sec),
                "nanosec": int(header.stamp.nanosec),
            },
            "frame_id": header.frame_id,
        },
        "image_topic": image_topic,
        "elapsed_ms": elapsed_ms,
        **stats,
    }
    return json.dumps(payload, separators=(",", ":"))


class RoadLaneMaskNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("road_lane_mask_wrapper")
        self.args = args
        self.received = 0
        self.processed = 0
        self.last_process_time = 0.0
        self.stop_requested = False

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=args.queue_size,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=(
                QoSReliabilityPolicy.RELIABLE
                if args.reliable
                else QoSReliabilityPolicy.BEST_EFFORT
            ),
        )

        self.drivable_pub = self.create_publisher(Image, args.drivable_topic, 1)
        self.lane_pub = self.create_publisher(Image, args.lane_topic, 1)
        self.overlay_pub = self.create_publisher(Image, args.overlay_topic, 1)
        self.stats_pub = self.create_publisher(String, args.stats_topic, 10)
        self.sub = self.create_subscription(Image, args.image_topic, self.on_image, qos)

        self.output_dir = Path(args.save_output_dir) if args.save_output_dir else None
        if self.output_dir is not None:
            self.output_dir.mkdir(parents=True, exist_ok=True)

        self.get_logger().info(
            f"Listening on {args.image_topic}; road HSV={args.road_hsv_lower}-{args.road_hsv_upper}, "
            f"lane HSV={args.lane_hsv_lower}-{args.lane_hsv_upper}"
        )
        self.get_logger().info(
            f"Publishing drivable={args.drivable_topic}, lane={args.lane_topic}, "
            f"overlay={args.overlay_topic}, stats={args.stats_topic}"
        )

    def on_image(self, msg: Image) -> None:
        self.received += 1
        if self.received % self.args.frame_stride != 0:
            return

        now = time.monotonic()
        if self.args.min_interval > 0 and now - self.last_process_time < self.args.min_interval:
            return

        start = time.perf_counter()
        try:
            bgr = image_msg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warning(f"Failed to convert image: {exc}")
            return

        drivable, lane, overlay, stats = extract_masks(bgr, self.args)
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        self.last_process_time = time.monotonic()
        self.processed += 1

        drivable_msg = cv2_to_image_msg(drivable, "mono8", msg.header)
        self.drivable_pub.publish(drivable_msg)

        lane_msg = cv2_to_image_msg(lane, "mono8", msg.header)
        self.lane_pub.publish(lane_msg)

        overlay_msg = cv2_to_image_msg(overlay, "bgr8", msg.header)
        self.overlay_pub.publish(overlay_msg)

        self.stats_pub.publish(
            String(data=stats_json(msg.header, self.args.image_topic, stats, elapsed_ms))
        )

        if self.output_dir and self.args.save_every > 0:
            if self.processed % self.args.save_every == 0:
                stem = f"frame_{self.processed:06d}"
                cv2.imwrite(str(self.output_dir / f"{stem}_drivable.png"), drivable)
                cv2.imwrite(str(self.output_dir / f"{stem}_lane.png"), lane)
                cv2.imwrite(str(self.output_dir / f"{stem}_overlay.png"), overlay)

        self.get_logger().info(
            f"frame={self.processed} drivable={stats['drivable_ratio']:.3f} "
            f"lane={stats['lane_ratio']:.3f} elapsed={elapsed_ms:.1f}ms"
        )

        if self.args.max_frames > 0 and self.processed >= self.args.max_frames:
            self.get_logger().info("Reached --max-frames; shutting down.")
            self.stop_requested = True


def run_image_file_smoke(args: argparse.Namespace) -> int:
    bgr = cv2.imread(args.image_file, cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError(f"Failed to read image: {args.image_file}")

    start = time.perf_counter()
    drivable, lane, overlay, stats = extract_masks(bgr, args)
    elapsed_ms = (time.perf_counter() - start) * 1000.0
    stats["elapsed_ms"] = elapsed_ms

    if args.save_output_dir:
        output_dir = Path(args.save_output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_dir / "smoke_drivable.png"), drivable)
        cv2.imwrite(str(output_dir / "smoke_lane.png"), lane)
        cv2.imwrite(str(output_dir / "smoke_overlay.png"), overlay)

    print(json.dumps(stats, indent=2), flush=True)
    return 0


def main(argv: Iterable[str] | None = None) -> int:
    args, ros_args = parse_args(argv)
    if args.image_file:
        return run_image_file_smoke(args)

    rclpy.init(args=ros_args)
    node = RoadLaneMaskNode(args)
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
