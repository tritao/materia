#!/usr/bin/env bash
# Replays every viewport state in app/tests/depth-precision/ through the real
# editor and fails when a view's depth step at its focus exceeds the camera's
# target. Add a case by saving "Copy view state" output there (use a project
# path relative to the repository root). Results land in app/tests/build/depth-precision/.
set -euo pipefail

app_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
repo_dir=$(cd "$app_dir/.." && pwd)
command -v xvfb-run >/dev/null || { echo "xvfb-run is required" >&2; exit 2; }
cd "$repo_dir"

failed=0
for state in "$app_dir"/tests/depth-precision/*.json; do
  name=$(basename "$state" .json)
  out="$app_dir/tests/build/depth-precision/$name"
  mkdir -p "$out"
  echo "depth-precision: $name"
  if ! xvfb-run -a -s '-screen 0 1600x1000x24' "$repo_dir/haxeon/scripts/haxeon" run \
    --project "$app_dir/tests/haxeon.depth-capture.json" -- --state="$state" --out="$out" 2>&1 \
    | tee "$out/run.log" | grep -E '^depth-capture'; then
    echo "depth-precision: $name FAILED (see $out/run.log)" >&2
    failed=1
  fi
done
exit "$failed"
