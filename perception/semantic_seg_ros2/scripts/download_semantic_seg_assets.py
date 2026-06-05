#!/usr/bin/env python3
"""Download semantic segmentation checkpoints used by the AAC wrappers."""

from __future__ import annotations

import argparse
import shutil
import sys
import zipfile
from pathlib import Path
from urllib.request import Request, urlopen


REPO_ROOT = Path(__file__).resolve().parents[3]

BISENETV2_URL = "https://github.com/CoinCheung/BiSeNet/releases/download/0.0.0/model_final_v2_city.pth"
BISENETV2_TARGET = REPO_ROOT / "perception/BiSeNet/res/model_final_v2_city.pth"
BISENETV2_LINK = REPO_ROOT / "perception/BiSeNet/res/model_final.pth"

PIDNET_ONNX_URL = (
    "https://qaihub-public-assets.s3.us-west-2.amazonaws.com/"
    "qai-hub-models/models/pidnet/releases/v0.55.0/pidnet-onnx-float.zip"
)
PIDNET_ONNX_DIR = REPO_ROOT / "perception/PIDNet/pretrained_models/qualcomm_pidnet_onnx_float"


def download(url: str, target: Path, force: bool) -> None:
    if target.exists() and target.stat().st_size > 0 and not force:
        print(f"[assets] exists: {target}")
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(target.suffix + ".tmp")
    print(f"[assets] downloading {url}")
    request = Request(url, headers={"User-Agent": "aac-semantic-seg/1.0"})
    with urlopen(request, timeout=120) as response, tmp.open("wb") as out:
        shutil.copyfileobj(response, out)
    tmp.replace(target)
    print(f"[assets] wrote {target} ({target.stat().st_size} bytes)")


def download_bisenetv2(force: bool) -> None:
    download(BISENETV2_URL, BISENETV2_TARGET, force)
    if BISENETV2_LINK.exists() or BISENETV2_LINK.is_symlink():
        BISENETV2_LINK.unlink()
    BISENETV2_LINK.symlink_to(BISENETV2_TARGET.name)
    print(f"[assets] linked {BISENETV2_LINK} -> {BISENETV2_TARGET.name}")


def download_pidnet_onnx(force: bool) -> None:
    zip_path = PIDNET_ONNX_DIR / "pidnet-onnx-float.zip"
    download(PIDNET_ONNX_URL, zip_path, force)
    expected = PIDNET_ONNX_DIR / "pidnet.onnx"
    if expected.exists() and not force:
        print(f"[assets] exists: {expected}")
        return
    with zipfile.ZipFile(zip_path) as archive:
        for member in archive.infolist():
            if member.is_dir():
                continue
            source = member.filename
            name = Path(source).name
            if name not in {"pidnet.onnx", "pidnet.data", "labels.txt", "metadata.json"}:
                continue
            target = PIDNET_ONNX_DIR / name
            with archive.open(member) as src, target.open("wb") as dst:
                shutil.copyfileobj(src, dst)
            print(f"[assets] extracted {target} ({target.stat().st_size} bytes)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--all", action="store_true", help="Download all supported assets.")
    parser.add_argument("--bisenetv2", action="store_true", help="Download BiSeNetV2 Cityscapes weight.")
    parser.add_argument("--pidnet-onnx", action="store_true", help="Download Qualcomm PIDNet ONNX export.")
    parser.add_argument("--force", action="store_true", help="Re-download even when files already exist.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not (args.all or args.bisenetv2 or args.pidnet_onnx):
        print("Nothing selected. Use --all, --bisenetv2, or --pidnet-onnx.", file=sys.stderr)
        return 2
    if args.all or args.bisenetv2:
        download_bisenetv2(args.force)
    if args.all or args.pidnet_onnx:
        download_pidnet_onnx(args.force)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
