#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
log_file=$(mktemp)
trap 'rm "$log_file"' EXIT
for case_name in frozen-failure frozen-joint-failure; do
  if "$repo_root/haxeon/scripts/haxeon" build --project="$repo_root/machinekit/tests/haxeon-$case_name.json" >"$log_file" 2>&1; then
    echo "$case_name accepted mutation" >&2
    exit 1
  fi
  if ! grep -q 'Final anonymous field .* cannot be assigned' "$log_file"; then
    tail -20 "$log_file" >&2
    exit 1
  fi
done
echo "Frozen mechanical record mutation rejected"
