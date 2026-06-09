"""Prepare the Hugging Face cache used by EfficientLoFTR."""

import argparse
import os
from pathlib import Path


def _repo_root():
    path = Path(__file__).resolve()
    for parent in path.parents:
        if (parent / 'requirements.txt').exists() and (parent / 'docker').exists():
            return parent
    return Path('/ros_ws/src/aac_2026')


def main(args=None):
    """Download and verify the EfficientLoFTR Hugging Face model cache."""
    parser = argparse.ArgumentParser(
        description='Prepare EfficientLoFTR model cache under aac_2026/.hf_home.')
    parser.add_argument(
        '--model-id',
        default='zju-community/efficientloftr',
        help='Hugging Face model id.',
    )
    parser.add_argument(
        '--hf-home',
        default=str(_repo_root() / '.hf_home'),
        help='HF_HOME directory. Defaults to the aac_2026/.hf_home cache.',
    )
    parser.add_argument(
        '--local-files-only',
        action='store_true',
        help='Do not download; only verify that the cache already exists.',
    )
    parser.add_argument(
        '--device',
        default='cpu',
        help='Device used for a light model load check, e.g. cpu or cuda.',
    )
    parsed = parser.parse_args(args=args)

    hf_home = Path(parsed.hf_home).expanduser().resolve()
    cache_dir = hf_home / 'hub'
    hf_home.mkdir(parents=True, exist_ok=True)
    os.environ['HF_HOME'] = str(hf_home)
    os.environ.setdefault('HF_HUB_DISABLE_TELEMETRY', '1')

    from transformers import AutoImageProcessor, AutoModelForKeypointMatching

    print(f'[efficientloftr-cache] HF_HOME={hf_home}')
    print(f'[efficientloftr-cache] cache_dir={cache_dir}')
    print(f'[efficientloftr-cache] model_id={parsed.model_id}')
    processor = AutoImageProcessor.from_pretrained(
        parsed.model_id,
        cache_dir=str(cache_dir),
        local_files_only=parsed.local_files_only,
        use_fast=False,
    )
    model = AutoModelForKeypointMatching.from_pretrained(
        parsed.model_id,
        cache_dir=str(cache_dir),
        local_files_only=parsed.local_files_only,
    )
    model = model.eval().to(parsed.device)
    print(f'[efficientloftr-cache] processor={type(processor).__name__}')
    print(f'[efficientloftr-cache] model={type(model).__name__}')
    print('[efficientloftr-cache] ready')


if __name__ == '__main__':
    main()
