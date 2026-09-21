#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
scenekit_dir=$(dirname "$module_dir")
materia_dir=$(dirname "$scenekit_dir")
nativekit_dir=${NATIVEKIT_DIR:-"$materia_dir/nativekit"}
haxeon_dir=${HAXEON_DIR:-"$materia_dir/haxeon"}
nativekit_build=${NATIVEKIT_BUILD:-"$scenekit_dir/build"}
target=${NATIVEKIT_HAXE_TARGET:-x86_64-linux-gnu}
haxe_bin=${HAXEON_HAXE_BIN:-"$haxeon_dir/.tools/haxe/haxe"}
hashlink_bin=${HAXEON_HASHLINK_BIN:-"$haxeon_dir/.tools/hashlink/hl"}
haxeon_runtime_dir=${HAXEON_RUNTIME_DIR:-"$haxeon_dir/out"}
generated_dir=${NATIVEKIT_SCENE_HAXEON_OUTPUT_DIR:-"$scenekit_dir/out/scene-haxeon"}
mkdir -p "$generated_dir"

scene_header="$scenekit_dir/scene/include/nativekit_scene.h"
render_header="$scenekit_dir/scene_render/include/nativekit_scene_render.h"
interaction_header="$scenekit_dir/scene_interaction/include/nativekit_scene_interaction.h"
nativekit_import_header="$nativekit_dir/bindings/haxe/nativekit_import.h"
gpu_import_header="$nativekit_dir/modules/gpu/bindings/nativekit_gpu_import.h"
nativekit_header="$nativekit_dir/include/nativekit.h"
graphics_header="$nativekit_dir/include/nativekit_graphics.h"
gpu_header="$nativekit_dir/modules/gpu/include/nativekit_gpu.h"

import_hxi() {
	(
		cd "$haxeon_dir"
		"$haxe_bin" --cwd "$haxeon_dir" -cp src --run FfiImportMain "$@"
	)
}

import_hxi \
	--target="$target" \
	--library=nativekit \
	--interface=NativeKit \
	--include="$nativekit_dir/include" \
	--output="$generated_dir/nativekit.hxi" \
	--source-label=bindings/haxe/nativekit_import.h \
	"$nativekit_import_header"

import_hxi \
	--target="$target" \
	--library=nativekit_gpu \
	--interface=NativeKitGpu \
	--depends=NativeKit \
	--include="$nativekit_dir/modules/gpu/include" \
	--include="$nativekit_dir/modules/gpu/bindings" \
	--include="$nativekit_dir/include" \
	--exclude-header="$nativekit_header" \
	--exclude-header="$graphics_header" \
	--output="$generated_dir/nativekit-gpu.hxi" \
	--source-label=modules/gpu/bindings/nativekit_gpu_import.h \
	"$gpu_import_header"

import_hxi \
	--target="$target" \
	--library=nativekit_scene \
	--interface=NativeKitScene \
	--include="$scenekit_dir/scene/include" \
	--include="$nativekit_dir/include" \
	--exclude-header="$nativekit_header" \
	--output="$generated_dir/nativekit-scene.hxi" \
	--source-label=scene/include/nativekit_scene.h \
	"$scene_header"

import_hxi \
	--target="$target" \
	--library=nativekit_scene_render \
	--interface=NativeKitSceneRender \
	--depends=NativeKitScene \
	--depends=NativeKitGpu \
	--include="$scenekit_dir/scene_render/include" \
	--include="$scenekit_dir/scene/include" \
	--include="$nativekit_dir/modules/gpu/include" \
	--include="$nativekit_dir/include" \
	--exclude-header="$scene_header" \
	--exclude-header="$gpu_header" \
	--exclude-header="$graphics_header" \
	--exclude-header="$nativekit_header" \
	--output="$generated_dir/nativekit-scene-render.hxi" \
	--source-label=scene_render/include/nativekit_scene_render.h \
	"$render_header"

import_hxi \
	--target="$target" \
	--library=nativekit_scene_interaction \
	--interface=NativeKitSceneInteraction \
	--depends=NativeKitSceneRender \
	--depends=NativeKitScene \
	--depends=NativeKitGpu \
	--include="$scenekit_dir/scene_interaction/include" \
	--include="$scenekit_dir/scene_render/include" \
	--include="$scenekit_dir/scene/include" \
	--include="$nativekit_dir/modules/gpu/include" \
	--include="$nativekit_dir/include" \
	--exclude-header="$render_header" \
	--exclude-header="$scene_header" \
	--exclude-header="$gpu_header" \
	--exclude-header="$graphics_header" \
	--exclude-header="$nativekit_header" \
	--output="$generated_dir/nativekit-scene-interaction.hxi" \
	--source-label=scene_interaction/bindings/nativekit_scene_interaction_import.h \
	"$interaction_header"

output="$generated_dir/nativekit-scene.hx.hl"
(
	cd "$haxeon_dir"
	"$haxe_bin" -cp "$haxeon_dir/src" -cp "$module_dir/tests/haxeon" --run HxiNativeKitSceneMain \
		"$output" "$generated_dir/nativekit-scene.hxi" "$generated_dir/nativekit-scene-render.hxi" \
		"$generated_dir/nativekit-scene-interaction.hxi" "$nativekit_dir" "$scenekit_dir" \
		"$generated_dir/nativekit.hxi" "$generated_dir/nativekit-gpu.hxi"
)

set +e
LD_LIBRARY_PATH="$haxeon_runtime_dir:$haxeon_dir/.tools/hashlink:$nativekit_build/scene_interaction:$nativekit_build/scene_render:$nativekit_build/scene:$nativekit_build/nativekit/modules/gpu:$nativekit_build/nativekit:${LD_LIBRARY_PATH:-}" \
	xvfb-run -a env LIBGL_ALWAYS_SOFTWARE=1 "$hashlink_bin" "$output"
status=$?
set -e
if [[ $status -ne 42 ]]; then
	echo "NativeKit scene Haxeon smoke test returned $status, expected 42" >&2
	exit 1
fi
