"""Common backend interface for semantic segmentation models."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass
class SemanticPrediction:
    labels: np.ndarray
    logits_shape: tuple[int, ...] | None
    elapsed_ms: float


class SemanticSegmentationBackend(Protocol):
    name: str
    class_names: list[str]

    def infer(self, bgr_image: np.ndarray) -> SemanticPrediction:
        """Return per-pixel class labels for a BGR uint8 image."""
