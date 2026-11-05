#!/usr/bin/env python3
"""Recreate source aliases after cloning AAC into a ROS workspace's src/."""
import argparse
import json
import os
from pathlib import Path


def ensure_links(links):
    """Validate the entire operation first; preserve every conflicting file."""
    for alias, target in links.items():
        if not target.exists():
            raise FileNotFoundError(target)
        if alias.exists() or alias.is_symlink():
            if not alias.is_symlink() or alias.resolve() != target.resolve():
                raise FileExistsError(f'Preserving existing path: {alias}')
    for alias, target in links.items():
        if not alias.is_symlink():
            alias.parent.mkdir(parents=True, exist_ok=True)
            alias.symlink_to(os.path.relpath(target, alias.parent),
                             target_is_directory=target.is_dir())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', type=Path,
                        help='Existing local directory containing bags/tracks/results/vendor')
    parser.add_argument('--reuse-python', action='store_true',
                        help='Also link existing local Python environments; same machine only')
    args = parser.parse_args()
    aac = Path(__file__).resolve().parents[2]
    if aac.parent.name != 'src':
        parser.error('Place this repository under <ros_workspace>/src/ first.')
    workspace = aac.parent.parent
    entries = json.loads((Path(__file__).parent/'workspace_links.json').read_text())
    links = {workspace/key: aac/value for key, value in entries.items()}
    if args.reuse_python and not args.assets:
        parser.error('--reuse-python requires --assets')
    if args.assets:
        assets = args.assets.resolve(strict=True)
        for name in ['bags', 'tracks', 'results', 'vendor/acados']:
            links[workspace/name] = assets/name
        if args.reuse_python:
            for name in ['.venv-acados', '.venv-dynamic-lio']:
                links[workspace/name] = assets/name
    ensure_links(links)
    marker = workspace/'.f1tenth-workspace'
    if not marker.exists():
        marker.write_text('AAC workspace: source aliases configured by operations/f1tenth/setup_workspace.py\n')
    print(f'Configured {len(links)} source/asset aliases under {workspace}')


if __name__ == '__main__':
    main()
