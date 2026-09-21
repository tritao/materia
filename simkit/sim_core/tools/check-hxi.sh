#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
simkit_dir=$(dirname "$module_dir")
materia_dir=$(dirname "$simkit_dir")
scenekit_dir=${SCENEKIT_DIR:-"$materia_dir/scenekit"}
nativekit_dir=${NATIVEKIT_DIR:-"$materia_dir/nativekit"}
haxeon_dir=${HAXEON_DIR:-"$materia_dir/haxeon"}
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
    --dependency-hxi="$scenekit_dir/scene/bindings/nativekit-scene.hxi" \
    --dependency-hxi="$nativekit_dir/bindings/haxe/nativekit.hxi" \
    --include="$module_dir/include" \
    --include="$scenekit_dir/scene/include" \
    --include="$nativekit_dir/include" \
    --exclude-header="$scenekit_dir/scene/include/nativekit_scene.h" \
    --exclude-header="$nativekit_dir/include/nativekit.h" \
    --source-label=sim_core/bindings/nativekit_sim_import.h \
    --output="$output" \
    "$module_dir/bindings/nativekit_sim_import.h"

echo "check-hxi: wrote $output"
