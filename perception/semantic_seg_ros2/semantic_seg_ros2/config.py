"""Config loading helpers."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def load_config(path: str | None) -> dict[str, Any]:
    if path is None:
        return {}

    config_path = Path(path).expanduser()
    text = config_path.read_text()
    if config_path.suffix.lower() == ".json":
        return json.loads(text)

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError(
            "YAML config requested, but PyYAML is not installed. "
            "Use a .json config or install PyYAML."
        ) from exc

    data = yaml.safe_load(text)
    return data or {}


def deep_update(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_update(result[key], value)
        else:
            result[key] = value
    return result
