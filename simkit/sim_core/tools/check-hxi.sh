#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
repo_dir=$(cd "$module_dir/../.." && pwd)
if [[ -n "${HAXEON_DIR:-}" ]]; then
    haxeon_dir=$HAXEON_DIR
else
    haxeon_dir="$(dirname "$repo_dir")/realtime-haxe"
    if [[ ! -x "$haxeon_dir/scripts/haxeon-ffi-audit" &&
        -x "$repo_dir/../../realtime-haxe/scripts/haxeon-ffi-audit" ]]; then
        haxeon_dir="$repo_dir/../../realtime-haxe"
    fi
fi
output=${1:-"$module_dir/bindings/nativekit-sim.hxi"}

"$haxeon_dir/scripts/haxeon-ffi-audit" \
    --target=x86_64-linux-gnu \
    --target=x86_64-w64-windows-gnu \
    --target=x86_64-apple-darwin \
    --target=arm64-apple-darwin \
    --profile=portable-abi64 \
    --library=nativekit_sim_core \
    --interface=NativeKitSim \
    --depends=NativeKitScene \
    --dependency-hxi="$repo_dir/modules/scene/bindings/nativekit-scene.hxi" \
    --dependency-hxi="$repo_dir/bindings/haxe/nativekit.hxi" \
    --include="$module_dir/include" \
    --include="$repo_dir/modules/scene/include" \
    --include="$repo_dir/include" \
    --exclude-header="$repo_dir/modules/scene/include/nativekit_scene.h" \
    --exclude-header="$repo_dir/include/nativekit.h" \
    --source-label=modules/sim_core/bindings/nativekit_sim_import.h \
    --output="$output" \
    "$module_dir/bindings/nativekit_sim_import.h"

echo "check-hxi: wrote $output"
