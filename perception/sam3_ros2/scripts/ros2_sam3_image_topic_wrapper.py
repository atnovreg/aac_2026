#!/usr/bin/env python3
"""Run SAM3 image segmentation from a ROS2 image topic."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from contextlib import nullcontext
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np
import torch
from huggingface_hub import hf_hub_download
from PIL import Image as PILImage


SAM3_REPO = Path(__file__).resolve().parents[2] / "sam3"
SAM3_CHECKPOINTS = SAM3_REPO / "checkpoints"
if SAM3_REPO.exists():
    sys.path.insert(0, str(SAM3_REPO))

import rclpy  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402


MASK_COLORS_RGB = np.array(
    [
        (255, 64, 32),
        (64, 180, 255),
        (120, 220, 90),
        (255, 220, 64),
        (220, 80, 220),
        (64, 220, 200),
        (255, 140, 40),
        (160, 120, 255),
    ],
    dtype=np.uint8,
)


def parse_args(argv: Iterable[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Subscribe to a ROS2 image topic and publish SAM3 masks/overlays.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--image-topic", default="/camera_front/image_raw")
    parser.add_argument("--prompt", default="vehicle")
    parser.add_argument("--checkpoint", default=None, help="Path to sam3.pt")
    parser.add_argument("--hf-repo", default="facebook/sam3")
    parser.add_argument("--hf-filename", default="sam3.pt")
    parser.add_argument(
        "--allow-download",
        action="store_true",
        help="Allow Hugging Face download when checkpoint is not already cached.",
    )
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--resolution", type=int, default=1008)
    parser.add_argument("--confidence-threshold", type=float, default=0.5)
    parser.add_argument("--overlay-topic", default="/sam3/overlay")
    parser.add_argument("--mask-topic", default="/sam3/mask")
    parser.add_argument("--detections-topic", default="/sam3/detections")
    parser.add_argument("--queue-size", type=int, default=1)
    parser.add_argument("--reliable", action="store_true", help="Use reliable QoS.")
    parser.add_argument(
        "--frame-stride",
        type=int,
        default=1,
        help="Run inference on every Nth received frame.",
    )
    parser.add_argument(
        "--min-interval",
        type=float,
        default=0.0,
        help="Minimum seconds between processed frames.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Exit after this many processed frames. 0 means run forever.",
    )
    parser.add_argument("--alpha", type=float, default=0.55)
    parser.add_argument(
        "--save-output-dir",
        default=None,
        help="Optional directory for saved overlay/mask debug images.",
    )
    parser.add_argument(
        "--save-every",
        type=int,
        default=0,
        help="Save every Nth processed frame when --save-output-dir is set.",
    )
    parser.add_argument(
        "--empty-cache",
        action="store_true",
        help="Call torch.cuda.empty_cache() after each frame.",
    )
    parser.add_argument(
        "--image-file",
        default=None,
        help="Run a one-shot non-ROS smoke test on an image file.",
    )
    args, ros_args = parser.parse_known_args(argv)
    args.frame_stride = max(1, args.frame_stride)
    args.queue_size = max(1, args.queue_size)
    args.alpha = min(1.0, max(0.0, args.alpha))
    return args, ros_args


def resolve_checkpoint(args: argparse.Namespace) -> str:
    if args.checkpoint:
        checkpoint = Path(args.checkpoint).expanduser()
        if not checkpoint.exists():
            raise FileNotFoundError(f"SAM3 checkpoint not found: {checkpoint}")
        return str(checkpoint)

    # Prefer the pre-downloaded local copy under perception/sam3/checkpoints/
    local = SAM3_CHECKPOINTS / args.hf_filename
    if local.exists():
        return str(local)

    return hf_hub_download(
        repo_id=args.hf_repo,
        filename=args.hf_filename,
        local_files_only=not args.allow_download,
    )


def autocast_context(device: str):
    if device.startswith("cuda"):
        return torch.autocast(device_type="cuda", dtype=torch.bfloat16)
    return nullcontext()


def build_processor(args: argparse.Namespace) -> Any:
    from sam3.model.sam3_image_processor import Sam3Processor
    from sam3.model_builder import build_sam3_image_model

    if args.device.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested, but torch.cuda.is_available() is False.")

    os.environ.setdefault("TORCHINDUCTOR_CACHE_DIR", "/tmp/torchinductor_cache_sam3")
    os.environ.setdefault("USE_PERFLIB", "1")

    checkpoint = resolve_checkpoint(args)
    print(f"[sam3-ros2] Loading checkpoint: {checkpoint}", flush=True)
    with autocast_context(args.device):
        model = build_sam3_image_model(
            checkpoint_path=checkpoint,
            load_from_HF=False,
            compile=False,
            device=args.device,
            eval_mode=True,
        )
    return Sam3Processor(
        model,
        resolution=args.resolution,
        device=args.device,
        confidence_threshold=args.confidence_threshold,
    )


def run_sam3_inference(
    processor: Any,
    rgb_image: np.ndarray,
    prompt: str,
    device: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    pil_image = PILImage.fromarray(rgb_image)
    with torch.inference_mode(), autocast_context(device):
        state = processor.set_image(pil_image)
        output = processor.set_text_prompt(prompt=prompt, state=state)
        if device.startswith("cuda"):
            torch.cuda.synchronize()

    masks = output["masks"].detach().cpu().numpy().astype(bool)
    if masks.ndim == 4:
        masks = masks[:, 0]
    boxes = output["boxes"].detach().float().cpu().numpy()
    scores = output["scores"].detach().float().cpu().numpy()
    return masks, boxes, scores


def make_overlay(
    rgb_image: np.ndarray,
    masks: np.ndarray,
    boxes: np.ndarray,
    scores: np.ndarray,
    alpha: float,
) -> np.ndarray:
    overlay = rgb_image.copy()
    for idx, mask in enumerate(masks):
        color = MASK_COLORS_RGB[idx % len(MASK_COLORS_RGB)]
        mask_bool = mask.astype(bool)
        if not mask_bool.any():
            continue
        blended = (
            overlay[mask_bool].astype(np.float32) * (1.0 - alpha)
            + color.astype(np.float32) * alpha
        )
        overlay[mask_bool] = blended.astype(np.uint8)

    bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
    for idx, box in enumerate(boxes):
        x0, y0, x1, y1 = np.round(box).astype(int).tolist()
        color_rgb = MASK_COLORS_RGB[idx % len(MASK_COLORS_RGB)]
        color_bgr = (int(color_rgb[2]), int(color_rgb[1]), int(color_rgb[0]))
        cv2.rectangle(bgr, (x0, y0), (x1, y1), color_bgr, 2)
        label = f"{idx}:{scores[idx]:.2f}"
        cv2.putText(
            bgr,
            label,
            (x0, max(16, y0 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color_bgr,
            2,
            cv2.LINE_AA,
        )
    return bgr


def combined_mask_image(masks: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    if masks.size == 0:
        return np.zeros(shape, dtype=np.uint8)
    return (masks.any(axis=0).astype(np.uint8) * 255)


def detections_json(
    header,
    prompt: str,
    image_topic: str,
    masks: np.ndarray,
    boxes: np.ndarray,
    scores: np.ndarray,
    elapsed_ms: float,
) -> str:
    detections = []
    for idx, (box, score) in enumerate(zip(boxes, scores)):
        mask_pixels = int(masks[idx].sum()) if idx < len(masks) else 0
        detections.append(
            {
                "id": idx,
                "score": float(score),
                "box_xyxy": [float(x) for x in box.tolist()],
                "mask_pixels": mask_pixels,
            }
        )
    payload = {
        "header": {
            "stamp": {
                "sec": int(header.stamp.sec),
                "nanosec": int(header.stamp.nanosec),
            },
            "frame_id": header.frame_id,
        },
        "prompt": prompt,
        "image_topic": image_topic,
        "num_detections": len(detections),
        "elapsed_ms": elapsed_ms,
        "detections": detections,
    }
    return json.dumps(payload, separators=(",", ":"))


class Sam3ImageTopicNode(Node):
    def __init__(self, args: argparse.Namespace, processor: Any):
        super().__init__("sam3_image_topic_wrapper")
        self.args = args
        self.processor = processor
        self.bridge = CvBridge()
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

        self.overlay_pub = self.create_publisher(Image, args.overlay_topic, 1)
        self.mask_pub = self.create_publisher(Image, args.mask_topic, 1)
        self.detections_pub = self.create_publisher(String, args.detections_topic, 10)
        self.sub = self.create_subscription(Image, args.image_topic, self.on_image, qos)

        self.output_dir = Path(args.save_output_dir) if args.save_output_dir else None
        if self.output_dir is not None:
            self.output_dir.mkdir(parents=True, exist_ok=True)

        self.get_logger().info(
            f"Listening on {args.image_topic}; prompt='{args.prompt}', "
            f"threshold={args.confidence_threshold}, device={args.device}"
        )
        self.get_logger().info(
            f"Publishing overlay={args.overlay_topic}, mask={args.mask_topic}, "
            f"detections={args.detections_topic}"
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
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Failed to convert image: {exc}")
            return

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        masks, boxes, scores = run_sam3_inference(
            self.processor,
            rgb,
            self.args.prompt,
            self.args.device,
        )
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        self.last_process_time = time.monotonic()
        self.processed += 1

        overlay = make_overlay(rgb, masks, boxes, scores, self.args.alpha)
        mask = combined_mask_image(masks, rgb.shape[:2])

        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        overlay_msg.header = msg.header
        self.overlay_pub.publish(overlay_msg)

        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        self.detections_pub.publish(
            String(
                data=detections_json(
                    msg.header,
                    self.args.prompt,
                    self.args.image_topic,
                    masks,
                    boxes,
                    scores,
                    elapsed_ms,
                )
            )
        )

        if self.output_dir and self.args.save_every > 0:
            if self.processed % self.args.save_every == 0:
                stem = f"frame_{self.processed:06d}"
                cv2.imwrite(str(self.output_dir / f"{stem}_overlay.png"), overlay)
                cv2.imwrite(str(self.output_dir / f"{stem}_mask.png"), mask)

        if self.args.empty_cache and self.args.device.startswith("cuda"):
            torch.cuda.empty_cache()

        self.get_logger().info(
            f"frame={self.processed} detections={len(scores)} "
            f"elapsed={elapsed_ms:.1f}ms"
        )

        if self.args.max_frames > 0 and self.processed >= self.args.max_frames:
            self.get_logger().info("Reached --max-frames; shutting down.")
            self.stop_requested = True


def run_image_file_smoke(args: argparse.Namespace) -> int:
    processor = build_processor(args)
    image = PILImage.open(args.image_file).convert("RGB")
    rgb = np.array(image)
    start = time.perf_counter()
    masks, boxes, scores = run_sam3_inference(
        processor,
        rgb,
        args.prompt,
        args.device,
    )
    elapsed_ms = (time.perf_counter() - start) * 1000.0
    overlay = make_overlay(rgb, masks, boxes, scores, args.alpha)
    mask = combined_mask_image(masks, rgb.shape[:2])

    if args.save_output_dir:
        output_dir = Path(args.save_output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_dir / "smoke_overlay.png"), overlay)
        cv2.imwrite(str(output_dir / "smoke_mask.png"), mask)

    print(
        json.dumps(
            {
                "image_file": args.image_file,
                "prompt": args.prompt,
                "num_detections": int(len(scores)),
                "scores": [float(score) for score in scores.tolist()],
                "boxes": boxes.tolist(),
                "elapsed_ms": elapsed_ms,
            },
            indent=2,
        ),
        flush=True,
    )
    return 0


def main(argv: Iterable[str] | None = None) -> int:
    args, ros_args = parse_args(argv)
    if args.image_file:
        return run_image_file_smoke(args)

    processor = build_processor(args)
    rclpy.init(args=ros_args)
    node = Sam3ImageTopicNode(args, processor)
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
