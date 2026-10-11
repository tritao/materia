#!/usr/bin/env python3
"""Keep the project-source test target on the app's dependency graph."""

import json
from pathlib import Path


app_dir = Path(__file__).resolve().parents[1]
source_path = app_dir / "tests/project-source/haxeon.json"
source = json.loads(source_path.read_text())
app_dependency = source["dependencies"].get("app", {})
if (source_path.parent / app_dependency.get("path", "")).resolve() != app_dir:
    raise SystemExit("Project-source tests must inherit the app package")
print("Project-source dependencies inherit app")
