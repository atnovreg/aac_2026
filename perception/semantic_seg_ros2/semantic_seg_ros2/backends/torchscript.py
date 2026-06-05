"""TorchScript backend for exported semantic segmentation models."""

from __future__ import annotations

import time
from typing import Any

import numpy as np
import torch

from .base import SemanticPrediction
from .torch_common import first_logits, logits_to_labels, normalize_bgr_to_tensor


class TorchScriptBackend:
    name = "torchscript"

    def __init__(self, config: dict[str, Any]):
        self.config = config
        self.device = str(config.get("device", "cuda:0"))
        self.input_size = config.get("input_size", [512, 1024])
        self.mean = list(config.get("mean", [0.485, 0.456, 0.406]))
        self.std = list(config.get("std", [0.229, 0.224, 0.225]))
        self.align_corners = bool(config.get("align_corners", False))
        self.divisor = int(config.get("divisor", 32))
        self.class_names = list(config.get("class_names", []))

        checkpoint = config.get("checkpoint")
        if not checkpoint:
            raise ValueError("TorchScript backend requires 'checkpoint'.")
        self.model = torch.jit.load(checkpoint, map_location=self.device)
        self.model.eval()

    @torch.inference_mode()
    def infer(self, bgr_image: np.ndarray) -> SemanticPrediction:
        start = time.perf_counter()
        tensor, input_size, original_size = normalize_bgr_to_tensor(
            bgr_image,
            self.mean,
            self.std,
            self.device,
            self.input_size,
            divisor=self.divisor,
        )
        output = self.model(tensor)
        logits = first_logits(output)
        labels = logits_to_labels(logits, input_size, original_size, self.align_corners)
        if self.device.startswith("cuda"):
            torch.cuda.synchronize()
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        return SemanticPrediction(labels=labels, logits_shape=tuple(logits.shape), elapsed_ms=elapsed_ms)
