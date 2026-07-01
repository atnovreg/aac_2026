#!/usr/bin/env python3
"""ROS2 wrapper for YOLOP drivable-area and lane-line segmentation."""

from __future__ import annotations

import importlib
import json
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any

import cv2
import numpy as np
import rclpy
import torch
import torch.nn.functional as F
import torchvision.transforms as transforms
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String


NORMALIZE = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
TRANSFORM = transforms.Compose([transforms.ToTensor(), NORMALIZE])


@dataclass(frozen=True)
class YolopConfig:
    yolop_dir: str
    weights_path: str
    image_topic: str
    drivable_mask_topic: str
    lane_mask_topic: str
    overlay_topic: str
    stats_topic: str
    detections_topic: str
    device: str
    img_size: int
    conf_thres: float
    iou_thres: float
    overlay_alpha: float
    process_every_n: int
    queue_size: int
    reliable: bool
    half_precision: bool
    publish_overlay: bool
    publish_detections: bool
    draw_boxes: bool


@dataclass(frozen=True)
class YolopModules:
    cfg: Any
    get_net: Any
    letterbox_for_img: Any
    non_max_suppression: Any
    plot_one_box: Any
    scale_coords: Any
    select_device: Any
    time_synchronized: Any


class YolopNode(Node):
    """Run YOLOP on incoming camera frames and publish masks."""

    def __init__(self) -> None:
        super().__init__("yolop_node")
        self.bridge = CvBridge()
        self.frame_count = 0
        self.processed_count = 0
        self.last_log_time = time.monotonic()
        self.latest_msg: Image | None = None
        self.latest_msg_lock = threading.Lock()
        self.frame_event = threading.Event()
        self.stop_event = threading.Event()

        self.config = self._load_config()
        self.modules = self._load_yolop_modules(self.config.yolop_dir)
        self.device = self._select_device(self.config.device)
        self.half = self.device.type != "cpu" and self.config.half_precision
        self.model = self._load_model()
        self.names = self.model.module.names if hasattr(self.model, "module") else self.model.names

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=self.config.queue_size,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=(
                QoSReliabilityPolicy.RELIABLE
                if self.config.reliable
                else QoSReliabilityPolicy.BEST_EFFORT
            ),
        )

        self.drivable_pub = self.create_publisher(Image, self.config.drivable_mask_topic, 10)
        self.lane_pub = self.create_publisher(Image, self.config.lane_mask_topic, 10)
        self.overlay_pub = self.create_publisher(Image, self.config.overlay_topic, 10)
        self.stats_pub = self.create_publisher(String, self.config.stats_topic, 10)
        self.detections_pub = self.create_publisher(String, self.config.detections_topic, 10)
        self.image_sub = self.create_subscription(Image, self.config.image_topic, self._on_image, qos)

        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

        self.get_logger().info(f"Loaded YOLOP weights: {self.config.weights_path}")
        self.get_logger().info(f"YOLOP directory: {self.config.yolop_dir}")
        self.get_logger().info(f"Device: {self.device}; half_precision={self.half}")
        self.get_logger().info(f"Subscribing: {self.config.image_topic}")
        self.get_logger().info(f"Publishing lane mask: {self.config.lane_mask_topic}")
        self.get_logger().info(f"Publishing drivable mask: {self.config.drivable_mask_topic}")

    def _load_config(self) -> YolopConfig:
        self.declare_parameter("yolop_dir", "/ros_ws/src/aac_2026/perception/YOLOP")
        self.declare_parameter("weights_path", "/ros_ws/src/aac_2026/perception/YOLOP/weights/End-to-end.pth")
        self.declare_parameter("image_topic", "/camera_front/image_raw")
        self.declare_parameter("drivable_mask_topic", "/yolop/drivable_mask")
        self.declare_parameter("lane_mask_topic", "/yolop/lane_mask")
        self.declare_parameter("overlay_topic", "/yolop/overlay")
        self.declare_parameter("stats_topic", "/yolop/stats")
        self.declare_parameter("detections_topic", "/yolop/detections")
        self.declare_parameter("device", "0")
        self.declare_parameter("img_size", 640)
        self.declare_parameter("conf_thres", 0.25)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("overlay_alpha", 0.55)
        self.declare_parameter("process_every_n", 1)
        self.declare_parameter("queue_size", 1)
        self.declare_parameter("reliable", False)
        self.declare_parameter("half_precision", True)
        self.declare_parameter("publish_overlay", True)
        self.declare_parameter("publish_detections", True)
        self.declare_parameter("draw_boxes", True)

        process_every_n = max(1, int(self.get_parameter("process_every_n").value))
        queue_size = max(1, int(self.get_parameter("queue_size").value))
        img_size = max(32, int(self.get_parameter("img_size").value))
        overlay_alpha = float(self.get_parameter("overlay_alpha").value)
        overlay_alpha = min(1.0, max(0.0, overlay_alpha))

        return YolopConfig(
            yolop_dir=str(self.get_parameter("yolop_dir").value),
            weights_path=str(self.get_parameter("weights_path").value),
            image_topic=str(self.get_parameter("image_topic").value),
            drivable_mask_topic=str(self.get_parameter("drivable_mask_topic").value),
            lane_mask_topic=str(self.get_parameter("lane_mask_topic").value),
            overlay_topic=str(self.get_parameter("overlay_topic").value),
            stats_topic=str(self.get_parameter("stats_topic").value),
            detections_topic=str(self.get_parameter("detections_topic").value),
            device=str(self.get_parameter("device").value),
            img_size=img_size,
            conf_thres=float(self.get_parameter("conf_thres").value),
            iou_thres=float(self.get_parameter("iou_thres").value),
            overlay_alpha=overlay_alpha,
            process_every_n=process_every_n,
            queue_size=queue_size,
            reliable=bool(self.get_parameter("reliable").value),
            half_precision=bool(self.get_parameter("half_precision").value),
            publish_overlay=bool(self.get_parameter("publish_overlay").value),
            publish_detections=bool(self.get_parameter("publish_detections").value),
            draw_boxes=bool(self.get_parameter("draw_boxes").value),
        )

    def _load_yolop_modules(self, yolop_dir: str) -> YolopModules:
        root = Path(yolop_dir).expanduser().resolve()
        if not root.exists():
            raise RuntimeError(f"YOLOP directory does not exist: {root}")
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))

        cfg_module = importlib.import_module("lib.config")
        models_module = importlib.import_module("lib.models")
        augmentations_module = importlib.import_module("lib.utils.augmentations")
        general_module = importlib.import_module("lib.core.general")
        plot_module = importlib.import_module("lib.utils.plot")
        utils_module = importlib.import_module("lib.utils.utils")

        return YolopModules(
            cfg=cfg_module.cfg,
            get_net=models_module.get_net,
            letterbox_for_img=augmentations_module.letterbox_for_img,
            non_max_suppression=general_module.non_max_suppression,
            plot_one_box=plot_module.plot_one_box,
            scale_coords=general_module.scale_coords,
            select_device=utils_module.select_device,
            time_synchronized=utils_module.time_synchronized,
        )

    def _select_device(self, device: str) -> torch.device:
        normalized = device.strip().lower()
        if normalized == "cuda":
            normalized = "0"
        elif normalized.startswith("cuda:"):
            normalized = normalized.split(":", 1)[1]
        return self.modules.select_device(None, normalized)

    def _load_model(self) -> torch.nn.Module:
        weights_path = Path(self.config.weights_path).expanduser()
        if not weights_path.exists():
            raise RuntimeError(f"YOLOP weights file does not exist: {weights_path}")

        model = self.modules.get_net(self.modules.cfg)
        checkpoint = torch.load(str(weights_path), map_location=self.device, weights_only=False)
        model.load_state_dict(checkpoint["state_dict"])
        model = model.to(self.device)
        if self.half:
            model.half()
        model.eval()

        with torch.no_grad():
            warmup = torch.zeros((1, 3, self.config.img_size, self.config.img_size), device=self.device)
            _ = model(warmup.half() if self.half else warmup)

        return model

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

        try:
            result = self._run_inference(frame)
        except Exception as exc:
            self.get_logger().error(f"YOLOP inference failed: {exc}")
            return

        header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id)
        self._publish_mono_image(self.drivable_pub, result["drivable_mask"], header)
        self._publish_mono_image(self.lane_pub, result["lane_mask"], header)

        if self.config.publish_overlay:
            overlay = self._make_overlay(frame, result["drivable_mask"], result["lane_mask"])
            if self.config.draw_boxes:
                self._draw_detections(overlay, result["detections"])
            self._publish_bgr_image(self.overlay_pub, overlay, header)

        if self.config.publish_detections:
            self._safe_publish(self.detections_pub, String(data=json.dumps(result["detections"])))

        total_ms = (time.perf_counter() - started_at) * 1000.0
        stats = self._make_stats(result, total_ms)
        self._safe_publish(self.stats_pub, String(data=json.dumps(stats)))
        self.processed_count += 1
        self._log_fps()

    def _run_inference(self, frame: np.ndarray) -> dict[str, Any]:
        image, _, pad = self.modules.letterbox_for_img(frame, new_shape=self.config.img_size, auto=True)
        tensor = TRANSFORM(image).to(self.device)
        tensor = tensor.half() if self.half else tensor.float()
        if tensor.ndimension() == 3:
            tensor = tensor.unsqueeze(0)

        with torch.no_grad():
            t1 = self.modules.time_synchronized()
            det_out, da_seg_out, ll_seg_out = self.model(tensor)
            t2 = self.modules.time_synchronized()

            inf_out, _ = det_out
            det_pred = self.modules.non_max_suppression(
                inf_out,
                conf_thres=self.config.conf_thres,
                iou_thres=self.config.iou_thres,
                classes=None,
                agnostic=False,
            )
            t3 = self.modules.time_synchronized()

        _, _, height, width = tensor.shape
        original_h, original_w = frame.shape[:2]
        pad_w, pad_h = pad
        pad_w = int(pad_w)
        pad_h = int(pad_h)

        drivable_mask = self._extract_segmentation_mask(
            da_seg_out,
            height,
            width,
            pad_h,
            pad_w,
            original_h,
            original_w,
        )
        lane_mask = self._extract_segmentation_mask(
            ll_seg_out,
            height,
            width,
            pad_h,
            pad_w,
            original_h,
            original_w,
        )
        detections = self._extract_detections(det_pred[0], tensor.shape[2:], frame.shape)

        return {
            "drivable_mask": drivable_mask,
            "lane_mask": lane_mask,
            "detections": detections,
            "inference_ms": (t2 - t1) * 1000.0,
            "nms_ms": (t3 - t2) * 1000.0,
        }

    def _extract_segmentation_mask(
        self,
        prediction: torch.Tensor,
        height: int,
        width: int,
        pad_h: int,
        pad_w: int,
        original_h: int,
        original_w: int,
    ) -> np.ndarray:
        cropped = prediction[:, :, pad_h : height - pad_h, pad_w : width - pad_w]
        resized = F.interpolate(
            cropped,
            size=(original_h, original_w),
            mode="bilinear",
            align_corners=False,
        )
        mask = torch.argmax(resized, dim=1).squeeze().cpu().numpy().astype(np.uint8)
        return mask * 255

    def _extract_detections(
        self,
        det: torch.Tensor,
        model_shape: tuple[int, int],
        frame_shape: tuple[int, ...],
    ) -> list[dict[str, Any]]:
        detections: list[dict[str, Any]] = []
        if det is None or not len(det):
            return detections

        det[:, :4] = self.modules.scale_coords(model_shape, det[:, :4], frame_shape).round()
        for *xyxy, conf, cls in reversed(det):
            class_id = int(cls)
            box = [int(value.item()) for value in xyxy]
            class_name = self.names[class_id] if class_id < len(self.names) else str(class_id)
            detections.append(
                {
                    "class_id": class_id,
                    "class_name": class_name,
                    "confidence": round(float(conf), 4),
                    "xyxy": box,
                }
            )
        return detections

    def _make_overlay(
        self,
        frame: np.ndarray,
        drivable_mask: np.ndarray,
        lane_mask: np.ndarray,
    ) -> np.ndarray:
        overlay = frame.copy()
        drivable_pixels = drivable_mask > 0
        lane_pixels = lane_mask > 0

        if drivable_pixels.any():
            green = np.array([0, 255, 0], dtype=np.float32)
            overlay[drivable_pixels] = (
                overlay[drivable_pixels].astype(np.float32) * (1.0 - self.config.overlay_alpha)
                + green * self.config.overlay_alpha
            ).astype(np.uint8)

        if lane_pixels.any():
            red = np.array([0, 0, 255], dtype=np.float32)
            overlay[lane_pixels] = (
                overlay[lane_pixels].astype(np.float32) * 0.35
                + red * 0.65
            ).astype(np.uint8)

        return overlay

    def _draw_detections(self, overlay: np.ndarray, detections: list[dict[str, Any]]) -> None:
        for detection in detections:
            self.modules.plot_one_box(
                detection["xyxy"],
                overlay,
                label=f"{detection['class_name']} {detection['confidence']:.2f}",
                color=[0, 255, 255],
                line_thickness=2,
            )

    def _make_stats(self, result: dict[str, Any], total_ms: float) -> dict[str, Any]:
        drivable_mask = result["drivable_mask"]
        lane_mask = result["lane_mask"]
        total_pixels = int(lane_mask.size)
        return {
            "total_ms": round(total_ms, 3),
            "inference_ms": round(result["inference_ms"], 3),
            "nms_ms": round(result["nms_ms"], 3),
            "drivable_pixels": int(np.count_nonzero(drivable_mask)),
            "lane_pixels": int(np.count_nonzero(lane_mask)),
            "total_pixels": total_pixels,
            "drivable_ratio": round(float(np.count_nonzero(drivable_mask) / total_pixels), 6)
            if total_pixels
            else 0.0,
            "lane_ratio": round(float(np.count_nonzero(lane_mask) / total_pixels), 6)
            if total_pixels
            else 0.0,
            "detections": len(result["detections"]),
            "device": str(self.device),
            "img_size": self.config.img_size,
        }

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

    def destroy_node(self) -> bool:
        self.stop_event.set()
        self.frame_event.set()
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YolopNode()
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
