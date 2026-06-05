"""Shared PyTorch preprocessing and checkpoint helpers."""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
import torch.nn.functional as F


def add_source_path(source_path: str | None) -> None:
    if not source_path:
        return
    path = Path(source_path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Model source path does not exist: {path}")
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)


def load_checkpoint_state(checkpoint_path: str) -> dict[str, torch.Tensor]:
    path = Path(checkpoint_path).expanduser()
    if not path.exists():
        raise FileNotFoundError(f"Checkpoint does not exist: {path}")
    state = torch.load(path, map_location="cpu")
    if isinstance(state, dict):
        for key in ("state_dict", "model", "net"):
            if key in state and isinstance(state[key], dict):
                state = state[key]
                break
    if not isinstance(state, dict):
        raise RuntimeError(f"Unsupported checkpoint format: {path}")
    return state


def strip_prefixes(state: dict[str, torch.Tensor]) -> dict[str, torch.Tensor]:
    prefixes = ("module.", "model.")
    cleaned = {}
    for key, value in state.items():
        new_key = key
        changed = True
        while changed:
            changed = False
            for prefix in prefixes:
                if new_key.startswith(prefix):
                    new_key = new_key[len(prefix) :]
                    changed = True
        cleaned[new_key] = value
    return cleaned


def load_matching_state(model: torch.nn.Module, checkpoint_path: str) -> int:
    state = strip_prefixes(load_checkpoint_state(checkpoint_path))
    model_state = model.state_dict()
    matched = {
        key: value
        for key, value in state.items()
        if key in model_state and tuple(value.shape) == tuple(model_state[key].shape)
    }
    model_state.update(matched)
    model.load_state_dict(model_state, strict=False)
    return len(matched)


def normalize_bgr_to_tensor(
    bgr: np.ndarray,
    mean: list[float],
    std: list[float],
    device: str,
    input_size: list[int] | tuple[int, int] | None,
    divisor: int,
) -> tuple[torch.Tensor, tuple[int, int], tuple[int, int]]:
    original_h, original_w = bgr.shape[:2]
    if input_size:
        target_h, target_w = int(input_size[0]), int(input_size[1])
        image = cv2.resize(bgr, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
    else:
        image = bgr
        target_h, target_w = original_h, original_w

    if divisor > 1:
        padded_h = int(math.ceil(target_h / divisor) * divisor)
        padded_w = int(math.ceil(target_w / divisor) * divisor)
        if padded_h != target_h or padded_w != target_w:
            padded = np.zeros((padded_h, padded_w, 3), dtype=image.dtype)
            padded[:target_h, :target_w] = image
            image = padded
    else:
        padded_h, padded_w = target_h, target_w

    rgb = image[:, :, ::-1].astype(np.float32) / 255.0
    rgb -= np.array(mean, dtype=np.float32)
    rgb /= np.array(std, dtype=np.float32)
    tensor = torch.from_numpy(rgb.transpose(2, 0, 1).copy()).unsqueeze(0).to(device)
    return tensor, (target_h, target_w), (original_h, original_w)


def logits_to_labels(
    logits: torch.Tensor,
    model_input_size: tuple[int, int],
    original_size: tuple[int, int],
    align_corners: bool,
) -> np.ndarray:
    logits = F.interpolate(
        logits,
        size=model_input_size,
        mode="bilinear",
        align_corners=align_corners,
    )
    labels = torch.argmax(logits, dim=1).squeeze(0).detach().cpu().numpy().astype(np.uint8)
    original_h, original_w = original_size
    if labels.shape != (original_h, original_w):
        labels = cv2.resize(labels, (original_w, original_h), interpolation=cv2.INTER_NEAREST)
    return labels


def first_logits(output: Any) -> torch.Tensor:
    if isinstance(output, torch.Tensor):
        return output
    if isinstance(output, (list, tuple)):
        for item in output:
            if isinstance(item, torch.Tensor) and item.ndim == 4:
                return item
    raise RuntimeError(f"Could not find logits tensor in model output type {type(output)!r}")
