"""Locate the data/build workspace without assuming a source directory depth."""
from pathlib import Path


def find_workspace(start):
    """Find the nearest marked ancestor of a source file or directory."""
    path = Path(start).resolve()
    for candidate in (path, *path.parents):
        if (candidate / ".f1tenth-workspace").is_file():
            return candidate
    raise FileNotFoundError(f"No .f1tenth-workspace marker above {path}")
