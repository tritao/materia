#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
scenekit_dir=$(dirname "$module_dir")
materia_dir=$(dirname "$scenekit_dir")
nativekit_dir=${NATIVEKIT_DIR:-"$materia_dir/nativekit"}
haxeon_dir=${HAXEON_DIR:-"$materia_dir/haxeon"}
output=${1:-"$module_dir/bindings/nativekit-scene.hxi"}

"$haxeon_dir/scripts/haxeon-ffi-audit" \
    --target=x86_64-linux-gnu \
    --target=x86_64-w64-windows-gnu \
    --target=x86_64-apple-darwin \
    --target=arm64-apple-darwin \
    --profile=portable-abi64 \
    --library=nativekit_scene \
    --interface=NativeKitScene \
    --include="$module_dir/include" \
    --include="$nativekit_dir/include" \
    --exclude-header="$nativekit_dir/include/nativekit.h" \
    --source-label=scene/bindings/nativekit_scene_import.h \
    --output="$output" \
    "$module_dir/bindings/nativekit_scene_import.h"

echo "check-hxi: wrote $output"
