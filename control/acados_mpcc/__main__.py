import argparse
from pathlib import Path
import yaml

from .controller import default_config
from .simulation import run


def main():
    parser = argparse.ArgumentParser(description='Run an offline ORCA MPCC with acados on a measured track')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--track', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--solver', choices=['SQP', 'SQP_RTI'])
    parser.add_argument('--steps', type=int)
    parser.add_argument('--progress-speed', type=float)
    args = parser.parse_args()
    config = default_config()
    if args.config:
        updates = yaml.safe_load(args.config.read_text())
        if 'weights' in updates:
            config['weights'].update(updates.pop('weights'))
        config.update(updates)
    for key in ('track', 'output', 'solver', 'steps'):
        value = getattr(args, key)
        if value is not None:
            config[key] = str(value) if isinstance(value, Path) else value
    if args.progress_speed is not None:
        config['progress_speed_max'] = args.progress_speed
    for key in ('dt', 'steps', 'horizon', 'initial_speed', 'progress_speed_max', 'prediction_substeps'):
        if config[key] <= 0:
            parser.error(f'{key} must be positive')
    _, summary = run(config)
    if not summary['completed_lap']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
