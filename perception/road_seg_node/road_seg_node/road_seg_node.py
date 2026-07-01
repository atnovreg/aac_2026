#!/usr/bin/env python3
"""ROS2 node for realtime road semantic segmentation.

The node subscribes to an RGB camera topic, runs a YOLO semantic segmentation
model, and publishes:
  - a binary road mask
  - an overlay image
  - the raw semantic class map
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
from ultralytics import YOLO


@dataclass(frozen=True)
class RoadSegConfig:
    task: str
    model_path: str
    image_topic: str
    mask_topic: str
    overlay_topic: str
    class_map_topic: str
    stats_topic: str
    device: str
    imgsz: int
    road_class_names: tuple[str, ...]
    road_class_ids: tuple[int, ...]
    overlay_alpha: float
    process_every_n: int
    publish_class_map: bool
    publish_stats: bool


class RoadSegNode(Node):
    """Semantic segmentation node that extracts road/drivable pixels."""

    def __init__(self) -> None:
        super().__init__("road_seg_node")
        self.bridge = CvBridge()
        self.frame_count = 0
        self.processed_count = 0
        self.last_log_time = time.monotonic()
        self.latest_msg: Image | None = None
        self.latest_msg_lock = threading.Lock()
        self.frame_event = threading.Event()
        self.stop_event = threading.Event()

        self.config = self._load_config()
        self.model = YOLO(self.config.model_path, task=self.config.task)
        self.road_ids = self._resolve_road_class_ids()

        self.mask_pub = self.create_publisher(Image, self.config.mask_topic, 10)
        self.overlay_pub = self.create_publisher(Image, self.config.overlay_topic, 10)
        self.class_map_pub = self.create_publisher(Image, self.config.class_map_topic, 10)
        self.stats_pub = self.create_publisher(String, self.config.stats_topic, 10)
        self.image_sub = self.create_subscription(Image, self.config.image_topic, self._on_image, 10)
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

        self.get_logger().info(f"Loaded model: {self.config.model_path}")
        self.get_logger().info(f"YOLO task: {self.config.task}")
        self.get_logger().info(f"Subscribing: {self.config.image_topic}")
        self.get_logger().info(f"Publishing road mask: {self.config.mask_topic}")
        self.get_logger().info(f"Publishing overlay: {self.config.overlay_topic}")
        self.get_logger().info(f"Road class ids: {self.road_ids}")

    def _load_config(self) -> RoadSegConfig:
        self.declare_parameter("task", "semantic")
        self.declare_parameter("model_path", "/ros_ws/src/aac_2026/models/yolo26n-sem.pt")
        self.declare_parameter("image_topic", "/camera_front/image_raw")
        self.declare_parameter("mask_topic", "/road_seg/mask")
        self.declare_parameter("overlay_topic", "/road_seg/overlay")
        self.declare_parameter("class_map_topic", "/road_seg/class_map")
        self.declare_parameter("stats_topic", "/road_seg/stats")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("imgsz", 1024)
        self.declare_parameter("road_class_names", "road")
        self.declare_parameter("road_class_ids", "")
        self.declare_parameter("overlay_alpha", 0.45)
        self.declare_parameter("process_every_n", 1)
        self.declare_parameter("publish_class_map", True)
        self.declare_parameter("publish_stats", True)

        road_class_names = self._csv_to_names(self.get_parameter("road_class_names").value)
        road_class_ids = self._csv_to_ints(self.get_parameter("road_class_ids").value)
        task = str(self.get_parameter("task").value).strip().lower()
        if task not in ("semantic", "segment"):
            self.get_logger().warn(f"Unsupported task '{task}'; using semantic.")
            task = "semantic"

        process_every_n = int(self.get_parameter("process_every_n").value)
        if process_every_n < 1:
            self.get_logger().warn("process_every_n must be >= 1; using 1.")
            process_every_n = 1

        overlay_alpha = float(self.get_parameter("overlay_alpha").value)
        overlay_alpha = min(1.0, max(0.0, overlay_alpha))

        return RoadSegConfig(
            task=task,
            model_path=str(self.get_parameter("model_path").value),
            image_topic=str(self.get_parameter("image_topic").value),
            mask_topic=str(self.get_parameter("mask_topic").value),
            overlay_topic=str(self.get_parameter("overlay_topic").value),
            class_map_topic=str(self.get_parameter("class_map_topic").value),
            stats_topic=str(self.get_parameter("stats_topic").value),
            device=str(self.get_parameter("device").value),
            imgsz=int(self.get_parameter("imgsz").value),
            road_class_names=road_class_names,
            road_class_ids=road_class_ids,
            overlay_alpha=overlay_alpha,
            process_every_n=process_every_n,
            publish_class_map=bool(self.get_parameter("publish_class_map").value),
            publish_stats=bool(self.get_parameter("publish_stats").value),
        )

    @staticmethod
    def _csv_to_names(value: object) -> tuple[str, ...]:
        return tuple(item.strip().lower() for item in str(value).split(",") if item.strip())

    @staticmethod
    def _csv_to_ints(value: object) -> tuple[int, ...]:
        ids: list[int] = []
        for item in str(value).split(","):
            item = item.strip()
            if item:
                ids.append(int(item))
        return tuple(ids)

    def _resolve_road_class_ids(self) -> tuple[int, ...]:
        if self.config.road_class_ids:
            return self.config.road_class_ids

        model_names = getattr(self.model, "names", {})
        names_by_id = model_names.items() if isinstance(model_names, dict) else enumerate(model_names)
        resolved = [
            int(class_id)
            for class_id, class_name in names_by_id
            if str(class_name).strip().lower() in self.config.road_class_names
        ]

        if resolved:
            return tuple(sorted(set(resolved)))

        if self.config.task == "semantic":
            self.get_logger().warn(
                "Could not resolve road_class_names from model.names; falling back to Cityscapes road id 0. "
                "Set road_class_ids explicitly if your model uses another label."
            )
            return (0,)

        self.get_logger().warn(
            "Could not resolve road_class_names from model.names for segment mode. "
            "The mask will be empty unless road_class_ids or matching class names are provided."
        )
        return ()

    def _on_image(self, msg: Image) -> None:
        self.frame_count += 1
        if self.frame_count % self.config.process_every_n:
            return

        with self.latest_msg_lock:
            self.latest_msg = msg
            self.frame_event.set()

    def _worker_loop(self) -> None:
        while rclpy.ok() and not self.stop_event.is_set():
            if not self.frame_event.wait(timeout=0.1):
                continue

            with self.latest_msg_lock:
                msg = self.latest_msg
                self.latest_msg = None
                self.frame_event.clear()

            if msg is not None:
                self._process_image(msg)

    def _process_image(self, msg: Image) -> None:
        started_at = time.perf_counter()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert input image: {exc}")
            return

        inference_started_at = time.perf_counter()
        try:
            result = self.model(
                frame,
                imgsz=self.config.imgsz,
                device=self.config.device,
                verbose=False,
            )[0]
        except Exception as exc:
            self.get_logger().error(f"Semantic segmentation inference failed: {exc}")
            return
        inference_ms = (time.perf_counter() - inference_started_at) * 1000.0

        class_map = None
        if self.config.task == "semantic":
            class_map = self._extract_class_map(result, frame.shape[:2])
            if class_map is None:
                return
            road_mask = self._make_road_mask(class_map)
        else:
            road_mask = self._extract_segment_mask(result, frame.shape[:2])
            if road_mask is None:
                return

        header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id)

        self._publish_mono_image(self.mask_pub, road_mask, header)
        if self.config.publish_class_map and class_map is not None:
            self._publish_mono_image(self.class_map_pub, class_map, header)

        overlay = self._make_overlay(frame, road_mask)
        self._publish_bgr_image(self.overlay_pub, overlay, header)

        self.processed_count += 1
        self._publish_stats(road_mask, started_at, inference_ms)
        self._log_fps()

    def _extract_class_map(self, result: object, image_shape: tuple[int, int]) -> np.ndarray | None:
        semantic_mask = getattr(result, "semantic_mask", None)
        if semantic_mask is None:
            self.get_logger().error(
                "Model result has no semantic_mask. Use a semantic checkpoint such as yolo26n-sem.pt."
            )
            return None

        class_map = semantic_mask.data
        if hasattr(class_map, "detach"):
            class_map = class_map.detach().cpu().numpy()

        class_map = np.asarray(class_map)
        if class_map.ndim != 2:
            self.get_logger().error(f"Expected class map with shape (H, W), got {class_map.shape}.")
            return None

        class_map = class_map.astype(np.uint8, copy=False)
        height, width = image_shape
        if class_map.shape != (height, width):
            class_map = cv2.resize(class_map, (width, height), interpolation=cv2.INTER_NEAREST)

        return class_map

    def _extract_segment_mask(self, result: object, image_shape: tuple[int, int]) -> np.ndarray | None:
        height, width = image_shape
        output_mask = np.zeros((height, width), dtype=np.uint8)
        if not self.road_ids:
            return output_mask

        masks = getattr(result, "masks", None)
        boxes = getattr(result, "boxes", None)
        if masks is None or boxes is None:
            return output_mask

        mask_data = getattr(masks, "data", None)
        class_data = getattr(boxes, "cls", None)
        if mask_data is None or class_data is None:
            return output_mask

        if hasattr(mask_data, "detach"):
            mask_data = mask_data.detach().cpu().numpy()
        if hasattr(class_data, "detach"):
            class_data = class_data.detach().cpu().numpy()

        mask_data = np.asarray(mask_data)
        class_data = np.asarray(class_data).astype(np.int32, copy=False)
        if mask_data.ndim != 3:
            self.get_logger().error(f"Expected segment masks with shape (N, H, W), got {mask_data.shape}.")
            return None

        selected = np.isin(class_data, self.road_ids)
        if not np.any(selected):
            return output_mask

        merged = np.any(mask_data[selected] > 0.5, axis=0).astype(np.uint8) * 255
        if merged.shape != (height, width):
            merged = cv2.resize(merged, (width, height), interpolation=cv2.INTER_NEAREST)
        return merged

    def _make_road_mask(self, class_map: np.ndarray) -> np.ndarray:
        return np.isin(class_map, self.road_ids).astype(np.uint8) * 255

    def _make_overlay(self, frame: np.ndarray, road_mask: np.ndarray) -> np.ndarray:
        road_layer = np.zeros_like(frame)
        road_layer[:, :, 1] = road_mask
        return cv2.addWeighted(frame, 1.0, road_layer, self.config.overlay_alpha, 0.0)

    def _publish_mono_image(self, publisher: object, image: np.ndarray, header: Header) -> None:
        msg = self.bridge.cv2_to_imgmsg(image, encoding="mono8")
        msg.header = header
        self._safe_publish(publisher, msg)

    def _publish_bgr_image(self, publisher: object, image: np.ndarray, header: Header) -> None:
        msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        msg.header = header
        self._safe_publish(publisher, msg)

    def _safe_publish(self, publisher: object, msg: object) -> None:
        if self.stop_event.is_set() or not rclpy.ok():
            return
        try:
            publisher.publish(msg)
        except Exception as exc:
            if not self.stop_event.is_set() and rclpy.ok():
                self.get_logger().error(f"Failed to publish {type(msg).__name__}: {exc}")

    def _log_fps(self) -> None:
        now = time.monotonic()
        elapsed = now - self.last_log_time
        if elapsed < 5.0:
            return

        fps = self.processed_count / elapsed
        self.get_logger().info(f"Processed FPS: {fps:.2f}")
        self.processed_count = 0
        self.last_log_time = now

    def _publish_stats(self, road_mask: np.ndarray, started_at: float, inference_ms: float) -> None:
        if not self.config.publish_stats:
            return

        total_ms = (time.perf_counter() - started_at) * 1000.0
        road_pixels = int(np.count_nonzero(road_mask))
        total_pixels = int(road_mask.size)
        stats = {
            "total_ms": round(total_ms, 3),
            "inference_ms": round(inference_ms, 3),
            "road_pixels": road_pixels,
            "total_pixels": total_pixels,
            "road_ratio": round(road_pixels / total_pixels, 6) if total_pixels else 0.0,
            "device": self.config.device,
            "imgsz": self.config.imgsz,
        }
        self._safe_publish(self.stats_pub, String(data=json.dumps(stats)))

    def destroy_node(self) -> bool:
        self.stop_event.set()
        self.frame_event.set()
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RoadSegNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
