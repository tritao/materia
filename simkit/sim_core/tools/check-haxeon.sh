#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
repo_dir=$(cd "$module_dir/../.." && pwd)
if [[ -n "${HAXEON_DIR:-}" ]]; then
    haxeon_dir=$HAXEON_DIR
else
    haxeon_dir="$(dirname "$repo_dir")/realtime-haxe"
    if [[ ! -x "$haxeon_dir/.tools/haxe/haxe" &&
        -x "$repo_dir/../../realtime-haxe/.tools/haxe/haxe" ]]; then
        haxeon_dir="$repo_dir/../../realtime-haxe"
    fi
fi
haxe_bin=${HAXEON_HAXE_BIN:-"$haxeon_dir/.tools/haxe/haxe"}
output=$(mktemp --suffix=.hl)
trap 'rm -f "$output"' EXIT

"$haxe_bin" --cwd "$haxeon_dir" -cp src --run compiler.tools.HaxeonCompiler \
    --output="$output" \
    --entry=SimBindingCompile \
    --root="$module_dir/tests/haxeon" \
    --root="$module_dir/bindings/haxe" \
    --root="$repo_dir/modules/scene/bindings/haxe" \
    --ffi-interface="$repo_dir/modules/scene/bindings/nativekit-scene.hxi" \
    --ffi-interface="$module_dir/bindings/nativekit-sim.hxi" \
    --ffi-projection="$module_dir/bindings/nativekit-sim.hxmap" \
    "$module_dir/tests/haxeon/SimBindingCompile.hx"

echo "check-haxeon: compiled simulation bindings"
