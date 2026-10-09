#!/usr/bin/env bash
# Build immutable assets and a small index; the browser fetches model data only after selection.
set -euo pipefail
app_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
materia_dir=$(dirname "$app_dir")
output=${1:-"$app_dir/build/web/site/examples"}
"$materia_dir/haxeon/scripts/haxeon" build --project="$app_dir/web/examples/haxeon.json"
export HAXEON_HOME="$materia_dir/haxeon"
export MATERIA_INSTALL_ROOT="$materia_dir"
native_paths=""
for directory in "$app_dir"/build/host/native/*/ "$app_dir"/web/examples/build/host/native/*/; do native_paths="$native_paths:${directory%/}"; done
export LD_LIBRARY_PATH="$materia_dir/haxeon/out:$materia_dir/haxeon/.tools/hashlink$native_paths${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd "$materia_dir"
exec "$materia_dir/haxeon/.tools/hashlink/hl" "$app_dir/web/examples/build/host/main.hl" "$output" "${@:2}"
