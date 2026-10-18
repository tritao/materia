#!/usr/bin/env python3
"""Fetch pinned supplier reference CAD for local design inspection; no models are bundled."""
import hashlib
from pathlib import Path
import urllib.request

ASSETS = {
    "HGH15CA": "3bc285a6e79d23be1a5d6408382785361402575449bf40175d2319b27a55c9d8",
    "HGR15R": "2897086393c72de3ca2a1af658da9e471bfbe38e234605e89dbb8497283e4351",
}
ROOT = Path(__file__).resolve().parent / "build" / "supplier"


def main():
    ROOT.mkdir(parents=True, exist_ok=True)
    for name, digest in ASSETS.items():
        path = ROOT / (name + ".step")
        url = "https://www.matis.sk/data/CAD_modely/05_Kolejnicova_vedeni/HG/" + name + ".STP"
        data = path.read_bytes() if path.exists() else urllib.request.urlopen(url, timeout=60).read()
        if hashlib.sha256(data).hexdigest() != digest:
            raise SystemExit("Supplier CAD changed; review before accepting: " + url)
        if not data.startswith(b"ISO-10303-21;"):
            raise SystemExit("Not a STEP model: " + url)
        path.write_bytes(data)
        print(path)


if __name__ == "__main__":
    main()
