#!/usr/bin/env python3
"""Initialize AAC data locations without recreating a parallel source tree."""
import argparse
import os
from pathlib import Path


def ensure_links(links):
    """Preflight every target; replace only empty directories or matching links."""
    empty = []
    for alias, target in links.items():
        if not target.exists():
            raise FileNotFoundError(target)
        if alias.is_symlink():
            if alias.resolve() != target.resolve():
                raise FileExistsError(f'Preserving existing path: {alias}')
        elif alias.exists():
            if alias.is_dir() and not any(alias.iterdir()):
                empty.append(alias)
            else:
                raise FileExistsError(f'Preserving existing path: {alias}')
    for alias in empty:
        alias.rmdir()
    for alias, target in links.items():
        if not alias.is_symlink():
            alias.parent.mkdir(parents=True, exist_ok=True)
            alias.symlink_to(os.path.relpath(target, alias.parent),
                             target_is_directory=target.is_dir())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', type=Path,
                        help='Existing local data: bag (or historical bags), tracks, results, vendor')
    parser.add_argument('--reuse-python', action='store_true',
                        help='Reuse existing Python environments on this same machine only')
    args = parser.parse_args()
    aac = Path(__file__).resolve().parents[2]
    if aac.parent.name != 'src':
        parser.error('Place this repository under <ros_workspace>/src/ first.')
    workspace = aac.parent.parent
    if args.reuse_python and not args.assets:
        parser.error('--reuse-python requires --assets')
    links = {}
    dockerignore = aac/'docker/f1tenth/context.dockerignore'
    if dockerignore.is_file():
        links[workspace/'.dockerignore'] = dockerignore
    if args.assets:
        assets = args.assets.resolve(strict=True)
        recordings = assets/'bag'
        if not recordings.is_dir():
            recordings = assets/'bags'
        links[workspace/'bag'] = recordings
        for name in ['tracks', 'results', 'vendor/acados']:
            links[workspace/name] = assets/name
        if args.reuse_python:
            for name in ['.venv-acados', '.venv-dynamic-lio']:
                links[workspace/name] = assets/name
    ensure_links(links)
    for name in ['bag', 'tracks', 'results']:
        (workspace/name).mkdir(exist_ok=True)
    marker = workspace/'.f1tenth-workspace'
    if not marker.exists():
        marker.write_text('AAC workspace: functional sources under src/aac_2026; recordings under bag/\n')
    print(f'Initialized AAC workspace at {workspace}; no legacy source aliases created')


if __name__ == '__main__':
    main()
