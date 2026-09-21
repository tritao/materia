#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
scenekit_dir=$(dirname "$module_dir")
materia_dir=$(dirname "$scenekit_dir")
nativekit_dir=${NATIVEKIT_DIR:-"$materia_dir/nativekit"}
haxeon_dir=${HAXEON_DIR:-"$materia_dir/haxeon"}
output=${1:-"$module_dir/bindings/nativekit-scene-interaction.hxi"}

"$haxeon_dir/scripts/haxeon-ffi-audit" \
    --target=x86_64-linux-gnu \
    --target=x86_64-w64-windows-gnu \
    --target=x86_64-apple-darwin \
    --target=arm64-apple-darwin \
    --profile=portable-abi64 \
    --library=nativekit_scene_interaction \
    --interface=NativeKitSceneInteraction \
    --depends=NativeKitSceneRender \
    --depends=NativeKitScene \
    --depends=NativeKitGpu \
    --dependency-hxi="$scenekit_dir/scene/bindings/nativekit-scene.hxi" \
    --dependency-hxi="$nativekit_dir/bindings/haxe/nativekit.hxi" \
    --dependency-hxi="$nativekit_dir/modules/gpu/bindings/nativekit-gpu.hxi" \
    --dependency-hxi="$scenekit_dir/scene_render/bindings/nativekit-scene-render.hxi" \
    --include="$module_dir/include" \
    --include="$scenekit_dir/scene_render/include" \
    --include="$scenekit_dir/scene/include" \
    --include="$nativekit_dir/modules/gpu/include" \
    --include="$nativekit_dir/include" \
    --exclude-header="$scenekit_dir/scene_render/include/nativekit_scene_render.h" \
    --exclude-header="$scenekit_dir/scene/include/nativekit_scene.h" \
    --exclude-header="$nativekit_dir/modules/gpu/include/nativekit_gpu.h" \
    --exclude-header="$nativekit_dir/include/nativekit_graphics.h" \
    --exclude-header="$nativekit_dir/include/nativekit.h" \
    --source-label=scene_interaction/bindings/nativekit_scene_interaction_import.h \
    --output="$output" \
    "$module_dir/bindings/nativekit_scene_interaction_import.h"

echo "check-hxi: wrote $output"
