#!/usr/bin/env bash
set -euo pipefail
[[ $# == 1 ]] || { echo "usage: app/web/tools/generate-wasm-hxi.sh OUTPUT_DIR" >&2; exit 2; }
materia_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
haxeon_dir=${HAXEON_DIR:-"$materia_dir/haxeon"}
mkdir -p "$1"
output_dir=$(cd "$1" && pwd)
generators=(
	"haxeon/packages/platform/tools/audit-haxeon-abi.sh:nativekit.hxi:--output="
	"haxeon/packages/platform/tools/update-haxeon-net-hxi.sh:nativekit-net.hxi:--output="
	"haxeon/packages/gpu/tools/check-hxi.sh:nativekit-gpu.hxi"
	"haxeon/packages/ui/tools/check-hxi.sh:nativekit-ui.hxi"
	"scenekit/scene/tools/check-hxi.sh:nativekit-scene.hxi"
	"scenekit/scene_render/tools/check-hxi.sh:nativekit-scene-render.hxi"
	"simkit/sim_core/tools/check-hxi.sh:nativekit-sim.hxi"
	"simkit/sim_mujoco/tools/check-hxi.sh:nativekit-sim-mujoco.hxi"
	"robotkit/runtime/tools/check-hxi.sh:robotkit-runtime.hxi"
	"robotkit/runtime/tools/check-simkit-hxi.sh:robotkit-simkit.hxi"
	"robotkit/policy/tools/check-hxi.sh:robotkit-policy.hxi"
	"robotkit/inference/tools/check-hxi.sh:robotkit-inference.hxi"
	"visionkit/native/tools/check-hxi.sh:visionkit.hxi"
	"animkit/native/tools/check-hxi.sh:animkit.hxi"
	"stockkit/core/tools/check-hxi.sh:stockkit.hxi"
	"trajectorykit/native/tools/check-hxi.sh:trajectory-core.hxi"
	"kinematicskit/native/tools/check-hxi.sh:kinematicskit.hxi"
	"motionkit/native/tools/check-hxi.sh:motionkit.hxi"
)
"$materia_dir/tools/web/generate-wasm-hxi.sh" "$output_dir" "${generators[@]}"

# CadKit's ABI uses only fixed-width scalars and generated layouts, so it is
# imported for one target and published under the portable profile, like its
# desktop interface (cadkit/scripts/generate-haxeon-hxi).
cadkit_import=$(mktemp)
"$haxeon_dir/scripts/haxeon-ffi-import" --target=wasm32-unknown-emscripten --library=cadkit-core \
	--interface=CadKit --source-label=core/include/cadkit.h --output="$cadkit_import" \
	"$materia_dir/cadkit/core/include/cadkit.h" >/dev/null
sed -e "1s/for wasm32-unknown-emscripten/for portable-abi32/" \
	-e '2s/@target("wasm32-unknown-emscripten")/@target("portable-abi32")/' \
	"$cadkit_import" > "$output_dir/CadKit.hxi"
rm -f -- "${cadkit_import:?}"
echo "generate-wasm-hxi: CadKit.hxi"
