"""Model backend registry."""

from __future__ import annotations

from typing import Any

from .base import SemanticSegmentationBackend
from .bisenetv2 import BiSeNetV2Backend
from .onnxruntime_backend import OnnxRuntimeBackend
from .pidnet import PIDNetBackend
from .torchscript import TorchScriptBackend


BACKENDS = {
    "pidnet_s": PIDNetBackend,
    "pidnet": PIDNetBackend,
    "bisenetv2": BiSeNetV2Backend,
    "onnx": OnnxRuntimeBackend,
    "onnxruntime": OnnxRuntimeBackend,
    "torchscript": TorchScriptBackend,
}


def create_backend(name: str, config: dict[str, Any]) -> SemanticSegmentationBackend:
    key = name.lower()
    if key not in BACKENDS:
        known = ", ".join(sorted(BACKENDS))
        raise ValueError(f"Unknown backend '{name}'. Known backends: {known}")
    return BACKENDS[key](config)
