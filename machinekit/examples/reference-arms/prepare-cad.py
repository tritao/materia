#!/usr/bin/env python3
"""Fetch and safely extract pinned official CAD archives into ignored build/cad/."""
import hashlib
import json
from pathlib import Path
import shutil
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parent


def main():
    spec = json.loads((ROOT / "cad-sources.json").read_text())
    output = ROOT / "build/cad"
    output.mkdir(parents=True, exist_ok=True)
    audit = []
    for archive in spec["archives"]:
        path = output / (archive["id"] + ".zip")
        if not path.exists():
            temporary = path.with_suffix(".zip.partial")
            with urllib.request.urlopen(archive["url"], timeout=120) as source, temporary.open("wb") as target:
                shutil.copyfileobj(source, target)
            temporary.replace(path)
        if hashlib.sha256(path.read_bytes()).hexdigest() != archive["sha256"]:
            raise ValueError(f"CAD archive hash changed: {path}")
        destination = output / archive["id"]
        files = {}
        with zipfile.ZipFile(path) as source:
            for member in source.infolist():
                if member.is_dir():
                    continue
                target = destination / member.filename
                if not target.resolve().is_relative_to(destination.resolve()):
                    raise ValueError("Unsafe CAD archive path")
                data = source.read(member)
                target.parent.mkdir(parents=True, exist_ok=True)
                if not target.exists() or target.read_bytes() != data:
                    target.write_bytes(data)
                files[member.filename] = hashlib.sha256(data).hexdigest()
        audit.append({"id": archive["id"], "archiveSha256": archive["sha256"], "files": files})
        (output / "sources.json").write_text(json.dumps(audit, indent=2) + "\n")
        print(f"{archive['id']}: {len(files)} verified CAD files")


if __name__ == "__main__":
    main()
