#!/usr/bin/env bash
set -euo pipefail

app_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
repo_dir=$(cd "$app_dir/.." && pwd)
command -v xvfb-run >/dev/null || { echo "xvfb-run is required" >&2; exit 2; }

output=$(MATERIA_KEEP_INSPECTOR_WORKSPACE=1 "$repo_dir/haxeon/scripts/haxeon" run \
  --project "$app_dir/tests/scripted-inspector/haxeon.json")
printf '%s\n' "$output"
workspace=$(printf '%s\n' "$output" | sed -n 's/^Materia scripted inspector workspace: //p' | tail -1)
case "$workspace" in
  "$app_dir"/tests/scripted-inspector/build/scripted-setup-[0-9]*/workspace.json) ;;
  *) echo "Unexpected scripted workspace path: $workspace" >&2; exit 1 ;;
esac
test -f "$workspace"
cleanup() {
  test ! -f "$workspace" || unlink "$workspace"
  rmdir "$(dirname "$workspace")" 2>/dev/null || true
}
trap cleanup EXIT

capture_dir="$app_dir/tests/build/scripted-inspector-visual"
mkdir -p "$capture_dir"
xvfb-run -a -s '-screen 0 1600x1000x24' env REFERENCE_EDITOR_WORKSPACE="$workspace" \
  "$repo_dir/haxeon/scripts/haxeon" run --project "$app_dir/haxeon.json" -- \
  --setup-script=materia.examples.two-robot --capture-dir="$capture_dir" --frames=12

python - "$capture_dir" <<'PY'
import json
import pathlib
import struct
import sys

capture = pathlib.Path(sys.argv[1])
state = json.loads((capture / "app-state.json").read_text())
tree = (capture / "ui-tree.txt").read_text()
with (capture / "frame.png").open("rb") as image:
    header = image.read(24)
assert header[:8] == b"\x89PNG\r\n\x1a\n"
assert struct.unpack(">II", header[16:24]) == (1320, 900)
assert state["scene"]["selectedId"] == "moving-obstacle"
assert [item["id"] for item in state["scene"]["objects"]] == ["moving-obstacle"]
for label in ("Reload script", "Enable overrides", "Apply", "Run", "Pause", "Reset"):
    assert f'label="{label}"' in tree, label
assert 'label="materia/robot-b"' in tree
print(f"Scripted inspector desktop capture verified: {capture / 'frame.png'}")
PY
