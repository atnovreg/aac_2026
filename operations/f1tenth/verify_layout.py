#!/usr/bin/env python3
"""Check the current AAC layout, independently of old migration snapshots."""
import argparse
from pathlib import Path

REQUIRED_FILES = (
    'control/acados_mpcc/controller.py',
    'control/acados_mpcc/config/acados_mpcc.yaml',
    'control/mpcc_cpp_virtual/package.xml',
    'control/mpcc_tuner/server.py',
    'navigation/fastlio/FAST_LIO/package.xml',
    'navigation/localization3d/closed_loop.py',
    'navigation/localization3d/inputs/map_geometry.npz',
    'planning/track_pipeline/__main__.py',
    'sensors/livox_ros_driver2/src/livox_ros_driver2.cpp',
    'sensors/livox_ros_driver2/package.xml',
    'sensors/realsense-ros/realsense2_camera/package.xml',
    'system/f1tenth/config/agx/profile.yaml',
    'system/f1tenth_gazebo_replay/Replay.cc',
    'docker/f1tenth/compose.yaml',
    'operations/f1tenth/run.sh',
)


def check_layout(aac, source_only=False):
    errors = [f'Missing source: {name}' for name in REQUIRED_FILES if not (aac/name).is_file()]
    workspace = aac.parent.parent
    if aac.parent.name != 'src':
        errors.append('Place AAC under <workspace>/src/aac_2026')
    if not (workspace/'.f1tenth-workspace').is_file():
        errors.append('Run operations/f1tenth/setup_workspace.py first')
    for name in ['tools', 'scripts', 'config', 'requirements', 'bags', 'compose.yaml']:
        path = workspace/name
        if path.exists() or path.is_symlink():
            errors.append(f'Legacy duplicate workspace entry: {path}')
    if not source_only:
        for name in ['bag', 'tracks', 'results']:
            if not (workspace/name).is_dir():
                errors.append(f'Missing or broken data directory: {workspace/name}')
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-only', action='store_true', help='Skip local data directories')
    args = parser.parse_args()
    errors = check_layout(Path(__file__).resolve().parents[2], args.source_only)
    if errors:
        raise SystemExit('\n'.join(errors))
    print('PASS: AAC source layout' + ('' if args.source_only else ' and local data directories'))
    print('Scope: layout only; dependencies, tests and physical hardware require separate checks.')


if __name__ == '__main__':
    main()
