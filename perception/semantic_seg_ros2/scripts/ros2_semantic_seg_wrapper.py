#!/usr/bin/env python3
"""Run a semantic segmentation backend on a ROS2 image topic."""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(PACKAGE_ROOT))

try:
    import rclpy  # noqa: E402
    from rclpy.node import Node  # noqa: E402
    from rclpy.qos import (  # noqa: E402
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from sensor_msgs.msg import Image  # noqa: E402
    from std_msgs.msg import String  # noqa: E402
except ModuleNotFoundError:
    rclpy = None
    Node = object
    QoSDurabilityPolicy = None
    QoSHistoryPolicy = None
    QoSProfile = None
    QoSReliabilityPolicy = None
    Image = object
    String = None

from semantic_seg_ros2.backends import create_backend  # noqa: E402
from semantic_seg_ros2.config import deep_update, load_config  # noqa: E402
from semantic_seg_ros2.utils.ros_image import cv2_to_image_msg, image_msg_to_bgr  # noqa: E402
from semantic_seg_ros2.utils.visualization import (  # noqa: E402
    make_overlay,
    mask_from_class_ids,
    parse_class_ids,
)


def comma_ints(value: str | None) -> list[int] | None:
    if value is None:
        return None
    if value.strip() == "":
        return []
    return [int(part.strip()) for part in value.split(",")]


def parse_args(argv: Iterable[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Common ROS2 wrapper for PIDNet-S, BiSeNetV2, or TorchScript segmentation.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--config", default=None)
    parser.add_argument("--backend", default=None, help="Override backend from config.")
    parser.add_argument("--image-topic", default="/camera_front/image_raw")
    parser.add_argument("--labels-topic", default="/semantic_seg/labels")
    parser.add_argument("--drivable-topic", default="/semantic_seg/drivable")
    parser.add_argument("--lane-topic", default="/semantic_seg/lane_markings")
    parser.add_argument("--overlay-topic", default="/semantic_seg/overlay")
    parser.add_argument("--stats-topic", default="/semantic_seg/stats")
    parser.add_argument("--queue-size", type=int, default=1)
    parser.add_argument("--reliable", action="store_true")
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--min-interval", type=float, default=0.0)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--alpha", type=float, default=0.45)
    parser.add_argument("--drivable-class-ids", type=comma_ints, default=None)
    parser.add_argument("--lane-class-ids", type=comma_ints, default=None)
    parser.add_argument(
        "--allow-random-weights",
        action="store_true",
        default=None,
        help="Allow model startup when the configured checkpoint is missing. For smoke tests only.",
    )
    parser.add_argument("--save-output-dir", default=None)
    parser.add_argument("--save-every", type=int, default=0)
    parser.add_argument("--image-file", default=None, help="Run one-shot non-ROS inference.")
    args, ros_args = parser.parse_known_args(argv)

    args.frame_stride = max(1, args.frame_stride)
    args.queue_size = max(1, args.queue_size)
    args.alpha = min(1.0, max(0.0, args.alpha))
    return args, ros_args


def build_runtime_config(args: argparse.Namespace) -> dict:
    config = load_config(args.config)
    config.setdefault("model", {})
    config.setdefault("output", {})

    model_overrides = {}
    if args.allow_random_weights is not None:
        model_overrides["allow_random_weights"] = args.allow_random_weights

    overrides = {
        "backend": args.backend,
        "model": model_overrides,
        "output": {
            "drivable_class_ids": args.drivable_class_ids,
            "lane_class_ids": args.lane_class_ids,
        },
    }
    clean_overrides = {
        key: value
        for key, value in overrides.items()
        if value is not None and not (isinstance(value, dict) and all(v is None for v in value.values()))
    }
    if "output" in clean_overrides:
        clean_overrides["output"] = {
            key: value for key, value in clean_overrides["output"].items() if value is not None
        }
    if "model" in clean_overrides and not clean_overrides["model"]:
        del clean_overrides["model"]
    config = deep_update(config, clean_overrides)

    if not config.get("backend"):
        raise ValueError("--backend or config field 'backend' is required.")
    return config


def prediction_stats(
    header,
    image_topic: str,
    backend_name: str,
    labels: np.ndarray,
    drivable_mask: np.ndarray,
    lane_mask: np.ndarray,
    logits_shape,
    elapsed_ms: float,
    total_elapsed_ms: float,
) -> str:
    unique, counts = np.unique(labels, return_counts=True)
    class_pixels = {str(int(k)): int(v) for k, v in zip(unique, counts)}
    payload = {
        "header": {
            "stamp": {
                "sec": int(header.stamp.sec),
                "nanosec": int(header.stamp.nanosec),
            },
            "frame_id": header.frame_id,
        },
        "image_topic": image_topic,
        "backend": backend_name,
        "width": int(labels.shape[1]),
        "height": int(labels.shape[0]),
        "logits_shape": logits_shape,
        "inference_ms": elapsed_ms,
        "total_ms": total_elapsed_ms,
        "fps": 1000.0 / total_elapsed_ms if total_elapsed_ms > 0 else 0.0,
        "drivable_pixels": int(np.count_nonzero(drivable_mask)),
        "lane_pixels": int(np.count_nonzero(lane_mask)),
        "class_pixels": class_pixels,
    }
    return json.dumps(payload, separators=(",", ":"))


class SemanticSegNode(Node):
    def __init__(self, args: argparse.Namespace, config: dict, backend):
        super().__init__("semantic_seg_wrapper")
        self.args = args
        self.config = config
        self.backend = backend
        output_config = config.get("output", {})
        self.drivable_class_ids = parse_class_ids(output_config.get("drivable_class_ids", [0]))
        self.lane_class_ids = parse_class_ids(output_config.get("lane_class_ids", []))

        self.received = 0
        self.processed = 0
        self.last_process_time = 0.0
        self.stop_requested = False
        self.running = True
        self.latest = None
        self.latest_seq = 0
        self.condition = threading.Condition()

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

        self.labels_pub = self.create_publisher(Image, args.labels_topic, 1)
        self.drivable_pub = self.create_publisher(Image, args.drivable_topic, 1)
        self.lane_pub = self.create_publisher(Image, args.lane_topic, 1)
        self.overlay_pub = self.create_publisher(Image, args.overlay_topic, 1)
        self.stats_pub = self.create_publisher(String, args.stats_topic, 10)
        self.sub = self.create_subscription(Image, args.image_topic, self.on_image, qos)

        self.output_dir = Path(args.save_output_dir) if args.save_output_dir else None
        if self.output_dir is not None:
            self.output_dir.mkdir(parents=True, exist_ok=True)

        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()

        self.get_logger().info(
            f"Listening on {args.image_topic}; backend={config['backend']}, "
            f"drivable_ids={self.drivable_class_ids}, lane_ids={self.lane_class_ids}"
        )
        self.get_logger().info(
            f"Publishing labels={args.labels_topic}, drivable={args.drivable_topic}, "
            f"lane={args.lane_topic}, overlay={args.overlay_topic}, stats={args.stats_topic}"
        )

    def on_image(self, msg: Image) -> None:
        self.received += 1
        if self.received % self.args.frame_stride != 0:
            return

        try:
            bgr = image_msg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warning(f"Failed to convert image: {exc}")
            return

        with self.condition:
            self.latest_seq += 1
            self.latest = (self.latest_seq, msg.header, bgr)
            self.condition.notify()

    def worker_loop(self) -> None:
        while self.running:
            with self.condition:
                while self.running and self.latest is None:
                    self.condition.wait(timeout=0.2)
                if not self.running:
                    break
                seq, header, bgr = self.latest
                self.latest = None

            now = time.monotonic()
            if self.args.min_interval > 0 and now - self.last_process_time < self.args.min_interval:
                continue

            total_start = time.perf_counter()
            try:
                prediction = self.backend.infer(bgr)
            except Exception as exc:
                self.get_logger().error(f"Inference failed: {exc}")
                continue

            labels = prediction.labels.astype(np.uint8)
            drivable = mask_from_class_ids(labels, self.drivable_class_ids)
            lane = mask_from_class_ids(labels, self.lane_class_ids)
            overlay = make_overlay(bgr, labels, drivable, lane, self.args.alpha)
            total_elapsed_ms = (time.perf_counter() - total_start) * 1000.0
            self.last_process_time = time.monotonic()
            self.processed += 1

            self.labels_pub.publish(cv2_to_image_msg(labels, "mono8", header))
            self.drivable_pub.publish(cv2_to_image_msg(drivable, "mono8", header))
            self.lane_pub.publish(cv2_to_image_msg(lane, "mono8", header))
            self.overlay_pub.publish(cv2_to_image_msg(overlay, "bgr8", header))
            self.stats_pub.publish(
                String(
                    data=prediction_stats(
                        header,
                        self.args.image_topic,
                        self.config["backend"],
                        labels,
                        drivable,
                        lane,
                        prediction.logits_shape,
                        prediction.elapsed_ms,
                        total_elapsed_ms,
                    )
                )
            )

            if self.output_dir and self.args.save_every > 0:
                if self.processed % self.args.save_every == 0:
                    stem = f"frame_{self.processed:06d}"
                    cv2.imwrite(str(self.output_dir / f"{stem}_labels.png"), labels)
                    cv2.imwrite(str(self.output_dir / f"{stem}_drivable.png"), drivable)
                    cv2.imwrite(str(self.output_dir / f"{stem}_lane.png"), lane)
                    cv2.imwrite(str(self.output_dir / f"{stem}_overlay.png"), overlay)

            self.get_logger().info(
                f"seq={seq} frame={self.processed} infer={prediction.elapsed_ms:.1f}ms "
                f"total={total_elapsed_ms:.1f}ms fps={1000.0 / total_elapsed_ms:.2f}"
            )

            if self.args.max_frames > 0 and self.processed >= self.args.max_frames:
                self.get_logger().info("Reached --max-frames; shutting down.")
                self.stop_requested = True
                self.running = False
                with self.condition:
                    self.condition.notify_all()

    def shutdown_worker(self) -> None:
        self.running = False
        with self.condition:
            self.condition.notify_all()
        self.worker.join(timeout=5.0)


def run_image_file(args: argparse.Namespace, config: dict, backend) -> int:
    bgr = cv2.imread(args.image_file, cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError(f"Failed to read image: {args.image_file}")

    output_config = config.get("output", {})
    drivable_class_ids = parse_class_ids(output_config.get("drivable_class_ids", [0]))
    lane_class_ids = parse_class_ids(output_config.get("lane_class_ids", []))

    total_start = time.perf_counter()
    prediction = backend.infer(bgr)
    labels = prediction.labels.astype(np.uint8)
    drivable = mask_from_class_ids(labels, drivable_class_ids)
    lane = mask_from_class_ids(labels, lane_class_ids)
    overlay = make_overlay(bgr, labels, drivable, lane, args.alpha)
    total_elapsed_ms = (time.perf_counter() - total_start) * 1000.0

    if args.save_output_dir:
        output_dir = Path(args.save_output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_dir / "smoke_labels.png"), labels)
        cv2.imwrite(str(output_dir / "smoke_drivable.png"), drivable)
        cv2.imwrite(str(output_dir / "smoke_lane.png"), lane)
        cv2.imwrite(str(output_dir / "smoke_overlay.png"), overlay)

    print(
        json.dumps(
            {
                "backend": config["backend"],
                "image_file": args.image_file,
                "width": int(labels.shape[1]),
                "height": int(labels.shape[0]),
                "logits_shape": prediction.logits_shape,
                "inference_ms": prediction.elapsed_ms,
                "total_ms": total_elapsed_ms,
                "fps": 1000.0 / total_elapsed_ms if total_elapsed_ms > 0 else 0.0,
                "drivable_pixels": int(np.count_nonzero(drivable)),
                "lane_pixels": int(np.count_nonzero(lane)),
            },
            indent=2,
        ),
        flush=True,
    )
    return 0


def main(argv: Iterable[str] | None = None) -> int:
    args, ros_args = parse_args(argv)
    config = build_runtime_config(args)

    if not args.image_file and rclpy is None:
        raise RuntimeError(
            "ROS2 Python modules are not available. Run "
            "'source /opt/ros/$ROS_DISTRO/setup.bash' before ROS topic mode."
        )

    backend = create_backend(config["backend"], config.get("model", {}))

    if args.image_file:
        return run_image_file(args, config, backend)

    rclpy.init(args=ros_args)
    node = SemanticSegNode(args, config, backend)
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted.")
    finally:
        node.shutdown_worker()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
