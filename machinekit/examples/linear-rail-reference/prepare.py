#!/usr/bin/env python3
"""Fetch the pinned HG reference CAD for local design inspection; no models are bundled.

Fills the one HG reference directory that rail and carriage drawings read,
build/suppliers/vallder/hg-reference (or MATERIA_HG_REFERENCE), via tools/suppliers/hg.py.
"""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools" / "suppliers"))
import hg  # noqa: E402


def main():
    output = Path(os.environ.get("MATERIA_HG_REFERENCE") or ROOT / "build" / "suppliers" / "vallder" / "hg-reference")
    hg.prepare(output)
    print(output)


if __name__ == "__main__":
    main()
