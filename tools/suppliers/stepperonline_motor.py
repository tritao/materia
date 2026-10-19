#!/usr/bin/env python3
"""Prepare hash-pinned, public motor reference assets. Generation itself is offline."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = Path(__file__).with_name('stepperonline-23hs22.json')


def verify(content, asset):
    if hashlib.sha256(content).hexdigest() != asset['sha256']:
        raise ValueError(f"Reference changed: review {asset['path']} before updating its pin")
    if asset['kind'] == 'step' and not content.startswith(b'ISO-10303-21;'):
        raise ValueError('Expected a STEP reference, not a download/login page')
    if asset['kind'] == 'svg' and b'<svg' not in content[:1000]:
        raise ValueError('Expected the supplier dimension drawing')


def prepare(directory, offline=False):
    manifest = json.loads(MANIFEST.read_text())
    for asset in manifest['assets']:
        path = directory / asset['path']
        if path.exists():
            verify(path.read_bytes(), asset)
            continue
        if offline:
            raise ValueError(f'Missing pinned motor reference: {path}')
        request = urllib.request.Request(asset['url'], headers={'User-Agent': 'MateriaSupplierCatalog/1.0'})
        with urllib.request.urlopen(request, timeout=60) as response:
            content = response.read(20_000_001)
        if len(content) > 20_000_000:
            raise ValueError('Motor reference exceeds download budget')
        verify(content, asset)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + '.tmp')
        temporary.write_bytes(content)
        temporary.replace(path)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, default=ROOT / 'build/suppliers/stepperonline')
    parser.add_argument('--offline', action='store_true')
    args = parser.parse_args()
    manifest = prepare(args.directory, args.offline)
    print(f"Verified {len(manifest['assets'])} pinned {manifest['model']} references in {args.directory}")


if __name__ == '__main__':
    main()
