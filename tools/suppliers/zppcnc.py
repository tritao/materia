#!/usr/bin/env python3
"""Fetch hash-pinned public ZPP reference files. No CAD files enter project history."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from urllib.request import Request, urlopen

REFERENCE = Path(__file__).with_name("zppcnc-reference.json")

def digest(data):
    return hashlib.sha256(data).hexdigest()

def fetch(path, url, expected, offline):
    if path.exists():
        data = path.read_bytes()
    elif offline:
        raise ValueError(f"Missing pinned download: {path}")
    else:
        with urlopen(Request(url, headers={"User-Agent": "MateriaSupplierImporter/1.0"}), timeout=60) as response:
            data = response.read(32 * 1024 * 1024 + 1)
        if len(data) > 32 * 1024 * 1024:
            raise ValueError("Supplier download exceeds size limit")
    if digest(data) != expected:
        raise ValueError(f"Supplier hash mismatch: {path.name}")
    if not path.exists():
        path.write_bytes(data)
    return data

def prepare(directory, offline=False):
    reference = json.loads(REFERENCE.read_text())
    directory.mkdir(parents=True, exist_ok=True)
    for asset in reference["assets"]:
        archive = directory / asset["archive"]
        fetch(archive, asset["url"], asset["archive_sha256"], offline)
        step = directory / asset["step"]
        if step.exists():
            data = step.read_bytes()
        else:
            # Select one reviewed member; archive paths never become output paths.
            data = subprocess.run(["7z", "x", "-so", str(archive), asset["step"]],
                                  check=True, stdout=subprocess.PIPE).stdout
        if digest(data) != asset["step_sha256"] or not data.startswith(b"ISO-10303-21;"):
            raise ValueError(f"STEP content differs from review: {asset['model']}")
        if not step.exists():
            step.write_bytes(data)
        print(f"Verified {asset['model']}: {len(data)} bytes")
    for catalog in reference["catalogs"]:
        fetch(directory / catalog["file"], catalog["url"], catalog["sha256"], offline)
    print("ZPP CAD and catalog references ready")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare", action="store_true", required=True)
    parser.add_argument("--directory", type=Path, default=Path("build/suppliers/zppcnc"))
    parser.add_argument("--offline", action="store_true")
    args = parser.parse_args()
    prepare(args.directory, args.offline)
