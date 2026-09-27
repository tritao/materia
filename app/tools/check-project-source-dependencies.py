#!/usr/bin/env python3
"""Keep the project-source test target on the app's dependency graph."""

import json
from pathlib import Path


app_dir = Path(__file__).resolve().parents[1]
app = json.loads((app_dir / "haxeon.json").read_text())
source = json.loads((app_dir / "haxeon.project-source.json").read_text())
if source["dependencies"] != app["dependencies"]:
    raise SystemExit("app/haxeon.project-source.json dependencies differ from app/haxeon.json")
print("Project-source dependencies match app")
