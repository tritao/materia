#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "$script_dir/../.." && pwd)"
exec "$repo_dir/haxeon/scripts/haxeon" run \
  --project "$script_dir/haxeon.json" -- materia.examples.two-robot --verify
