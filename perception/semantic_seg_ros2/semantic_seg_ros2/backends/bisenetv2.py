"""BiSeNetV2 backend adapter."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import numpy as np
import torch

from .base import SemanticPrediction
from .torch_common import (
    add_source_path,
    first_logits,
    load_matching_state,
    logits_to_labels,
    normalize_bgr_to_tensor,
)


class BiSeNetV2Backend:
    name = "bisenetv2"

    def __init__(self, config: dict[str, Any]):
        self.config = config
        self.device = str(config.get("device", "cuda:0"))
        self.model_type = str(config.get("model_type", "bisenetv2"))
        self.num_classes = int(config.get("num_classes", 19))
        self.input_size = config.get("input_size", [512, 1024])
        self.mean = list(config.get("mean", [0.3257, 0.3690, 0.3223]))
        self.std = list(config.get("std", [0.2112, 0.2148, 0.2115]))
        self.align_corners = bool(config.get("align_corners", False))
        self.class_names = list(config.get("class_names", []))

        if self.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA device requested, but torch.cuda.is_available() is False.")

        add_source_path(config.get("source_path"))
        if bool(config.get("skip_backbone_pretrain", True)):
            import lib.models.bisenetv2 as bisenetv2_module

            bisenetv2_module.BiSeNetV2.load_pretrain = lambda self: None

        from lib.models import model_factory

        self.model = model_factory[self.model_type](self.num_classes, aux_mode="eval")
        checkpoint = config.get("checkpoint")
        allow_random = bool(config.get("allow_random_weights", False))
        if checkpoint and Path(checkpoint).expanduser().exists():
            loaded = load_matching_state(self.model, checkpoint)
            print(f"[semantic-seg] BiSeNetV2 loaded {loaded} checkpoint tensors.", flush=True)
        elif checkpoint:
            if not allow_random:
                raise FileNotFoundError(
                    "BiSeNetV2 checkpoint is missing: "
                    f"{checkpoint}. Download the checkpoint or run with "
                    "--allow-random-weights for smoke tests only."
                )
            print(
                f"[semantic-seg] BiSeNetV2 checkpoint missing, using random weights for smoke test: {checkpoint}",
                flush=True,
            )
        else:
            print("[semantic-seg] BiSeNetV2 running with random weights.", flush=True)
        self.model.to(self.device)
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
            divisor=32,
        )
        output = self.model(tensor)
        logits = first_logits(output)
        labels = logits_to_labels(logits, input_size, original_size, self.align_corners)
        if self.device.startswith("cuda"):
            torch.cuda.synchronize()
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        return SemanticPrediction(labels=labels, logits_shape=tuple(logits.shape), elapsed_ms=elapsed_ms)
