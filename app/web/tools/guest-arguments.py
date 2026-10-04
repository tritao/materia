#!/usr/bin/env python3
"""Compatibility entry for the workspace's shared manifest walker."""
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).resolve().parents[3] / "tools/web/guest-arguments.py"), run_name="__main__")
