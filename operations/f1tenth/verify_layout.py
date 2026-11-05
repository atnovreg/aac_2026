#!/usr/bin/env python3
"""Verify imported source hashes, original AAC files, and local source/data links."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-only', action='store_true',
                        help='Skip workstation asset links and original-source availability checks')
    args = parser.parse_args()
    aac = Path(__file__).resolve().parents[2]
    here = Path(__file__).resolve().parent
    manifest = json.loads((here/'import_manifest.json').read_text())
    failures = []
    imported_checked = 0
    for row in manifest['files']:
        if args.source_only and row.get('publish_source') is False:
            continue
        imported_checked += 1
        path = aac/row['destination']
        if not path.is_file():
            failures.append(f'Missing imported file: {path}')
            continue
        if hashlib.sha256(path.read_bytes()).hexdigest() != row.get('installed_sha256'):
            failures.append(f'Imported file changed since verification: {path}')
        source = row['source']
        if not args.source_only and not source.startswith('ssh://'):
            original = Path(source)
            if not original.is_file() or hashlib.sha256(original.read_bytes()).hexdigest() != row['source_sha256']:
                failures.append(f'Original source missing or changed: {source}')
    baseline = json.loads((here/'original_aac_hashes.json').read_text())
    backups = {'README.md': 'operations/f1tenth/before/README.md',
               '.gitignore': 'operations/f1tenth/before/.gitignore'}
    backups.update(manifest.get('original_aac_backups', {}))
    for rel, expected in baseline.items():
        # Host-path updates retain the byte-exact baseline in a recorded backup.
        source = aac/backups.get(rel, rel)
        if not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest() != expected:
            failures.append(f'Original AAC file not preserved: {rel}')
    aliases = json.loads((here/'workspace_links.json').read_text())
    for alias, target in aliases.items():
        path = aac.parent.parent/alias
        if not path.is_symlink() or path.resolve() != (aac/target).resolve() or not path.exists():
            failures.append(f'Broken or conflicting source alias: {alias}')
    if not args.source_only:
        for link in manifest['local_links']:
            path, target = Path(link['path']), Path(link['target'])
            if not path.is_symlink() or path.resolve() != target.resolve() or not path.exists():
                failures.append(f'Broken local data/archive alias: {path}')
    for message in failures:
        print(message)
    if failures:
        raise SystemExit(f'FAIL: {len(failures)} preservation/layout checks')
    print(f"PASS: {imported_checked} imported files, {len(baseline)} original AAC files, "
          f'{len(aliases)} portable source aliases')
    if not args.source_only:
        print(f"PASS: {len(manifest['local_links'])} local source/data/archive links; originals preserved")
    print('Scope: file/layout verification; hardware/AGX operation is not verified.')


if __name__ == '__main__':
    main()
