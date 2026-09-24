#!/usr/bin/env python3
"""Keep compiled example-script identity tied to its source bytes."""

import hashlib
import pathlib
import re
import sys

app = pathlib.Path(__file__).resolve().parents[1]
source = app / "src/examples/TwoRobotSetupScript.hx"
manifest = app / "src/ScriptSourceManifest.hx"
source_text = manifest.read_text()
match = re.search(r'TWO_ROBOT_SHA256 = "([0-9a-f]{64})"', source_text)
if match is None:
    raise SystemExit("Missing two-robot script source identity")
actual = hashlib.sha256(source.read_bytes()).hexdigest()
if sys.argv[1:] == ["--write"]:
    manifest.write_text(source_text[: match.start(1)] + actual + source_text[match.end(1) :])
    print("Generated example script source identity")
    raise SystemExit(0)
if sys.argv[1:]:
    raise SystemExit("Usage: check-script-identity.py [--write]")
if actual != match.group(1):
    raise SystemExit("Two-robot source identity is stale; run check-script-identity.py --write")
print("Example script source identity verified")
