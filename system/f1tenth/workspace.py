"""Locate the data/build workspace without assuming a source directory depth."""
from pathlib import Path


def find_workspace(start):
    """Find the nearest marked ancestor of a source file or directory."""
    path = Path(start).resolve()
    for candidate in (path, *path.parents):
        if (candidate / ".f1tenth-workspace").is_file():
            return candidate
    raise FileNotFoundError(f"No .f1tenth-workspace marker above {path}")


def resolve_data_path(workspace, recorded_path):
    """Read saved /ws paths and historical bags/ names through canonical bag/."""
    path = Path(recorded_path)
    if path.is_relative_to('/ws'):
        path = path.relative_to('/ws')
    elif path.is_absolute():
        return path
    if path.parts and path.parts[0] == 'bags':
        path = Path('bag', *path.parts[1:])
    return Path(workspace)/path
