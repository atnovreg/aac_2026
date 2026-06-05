"""ONNX Runtime backend for exported semantic segmentation models."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .base import SemanticPrediction


class OnnxRuntimeBackend:
    name = "onnxruntime"

    def __init__(self, config: dict[str, Any]):
        try:
            import onnxruntime as ort
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "ONNX Runtime is not installed. Install it in the Docker container with "
                "'python3 -m pip install --user onnxruntime-gpu'."
            ) from exc

        self.config = config
        self.device = str(config.get("device", "cuda:0"))
        self.input_size = config.get("input_size", [1024, 2048])
        self.input_layout = str(config.get("input_layout", "nchw")).lower()
        self.output_layout = str(config.get("output_layout", "nchw")).lower()
        self.color_order = str(config.get("color_order", "rgb")).lower()
        self.mean = config.get("mean")
        self.std = config.get("std")
        self.class_names = list(config.get("class_names", []))

        checkpoint = config.get("checkpoint") or config.get("model_path")
        if not checkpoint:
            raise ValueError("ONNX Runtime backend requires 'checkpoint' or 'model_path'.")
        path = Path(checkpoint).expanduser()
        if not path.exists():
            raise FileNotFoundError(
                "ONNX checkpoint is missing: "
                f"{path}. Run scripts/download_semantic_seg_assets.py --pidnet-onnx "
                "or edit the config to point at an existing ONNX file."
            )

        providers = config.get("providers")
        if providers:
            session_providers = list(providers)
        else:
            available = set(ort.get_available_providers())
            session_providers = []
            if bool(config.get("use_tensorrt", False)) and "TensorrtExecutionProvider" in available:
                session_providers.append("TensorrtExecutionProvider")
            if self.device.startswith("cuda") and "CUDAExecutionProvider" in available:
                session_providers.append("CUDAExecutionProvider")
            session_providers.append("CPUExecutionProvider")

        options = ort.SessionOptions()
        options.enable_mem_pattern = False
        self.session = ort.InferenceSession(str(path), sess_options=options, providers=session_providers)
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[int(config.get("output_index", 0))].name
        print(
            "[semantic-seg] ONNX Runtime loaded "
            f"{path} with providers={self.session.get_providers()}",
            flush=True,
        )

    def _preprocess(self, bgr: np.ndarray) -> tuple[np.ndarray, tuple[int, int], tuple[int, int]]:
        original_h, original_w = bgr.shape[:2]
        target_h, target_w = int(self.input_size[0]), int(self.input_size[1])
        image = cv2.resize(bgr, (target_w, target_h), interpolation=cv2.INTER_LINEAR)

        if self.color_order == "rgb":
            image = image[:, :, ::-1]
        elif self.color_order != "bgr":
            raise ValueError(f"Unsupported color_order: {self.color_order}")

        image = image.astype(np.float32) / 255.0
        if self.mean is not None:
            image -= np.array(self.mean, dtype=np.float32)
        if self.std is not None:
            image /= np.array(self.std, dtype=np.float32)

        if self.input_layout == "nchw":
            tensor = image.transpose(2, 0, 1)[None, :, :, :]
        elif self.input_layout == "nhwc":
            tensor = image[None, :, :, :]
        else:
            raise ValueError(f"Unsupported input_layout: {self.input_layout}")
        return np.ascontiguousarray(tensor, dtype=np.float32), (target_h, target_w), (original_h, original_w)

    def _labels_from_logits(
        self,
        logits: np.ndarray,
        model_input_size: tuple[int, int],
        original_size: tuple[int, int],
    ) -> np.ndarray:
        if logits.ndim == 4:
            if self.output_layout == "nchw":
                class_logits = logits[0]
            elif self.output_layout == "nhwc":
                class_logits = logits[0].transpose(2, 0, 1)
            else:
                raise ValueError(f"Unsupported output_layout: {self.output_layout}")

            input_h, input_w = model_input_size
            original_h, original_w = original_size
            if original_h * original_w <= input_h * input_w:
                target_h, target_w = original_h, original_w
            else:
                target_h, target_w = input_h, input_w
            resized = np.empty((class_logits.shape[0], target_h, target_w), dtype=np.float32)
            for class_idx, class_map in enumerate(class_logits):
                resized[class_idx] = cv2.resize(
                    class_map,
                    (target_w, target_h),
                    interpolation=cv2.INTER_LINEAR,
                )
            labels = np.argmax(resized, axis=0).astype(np.uint8)
        elif logits.ndim == 3:
            labels = logits[0].astype(np.uint8)
        elif logits.ndim == 2:
            labels = logits.astype(np.uint8)
        else:
            raise RuntimeError(f"Unsupported ONNX output shape: {logits.shape}")

        original_h, original_w = original_size
        if labels.shape != (original_h, original_w):
            labels = cv2.resize(labels, (original_w, original_h), interpolation=cv2.INTER_NEAREST)
        return labels

    def infer(self, bgr_image: np.ndarray) -> SemanticPrediction:
        start = time.perf_counter()
        tensor, input_size, original_size = self._preprocess(bgr_image)
        output = self.session.run([self.output_name], {self.input_name: tensor})[0]
        labels = self._labels_from_logits(output, input_size, original_size)
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        return SemanticPrediction(labels=labels, logits_shape=tuple(output.shape), elapsed_ms=elapsed_ms)
