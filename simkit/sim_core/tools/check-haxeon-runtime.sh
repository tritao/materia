#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
simkit_dir=$(dirname "$module_dir")
materia_dir=$(dirname "$simkit_dir")
scenekit_dir=${SCENEKIT_DIR:-"$materia_dir/scenekit"}
haxeon_dir=${HAXEON_DIR:-"$materia_dir/haxeon"}
haxe_bin=${HAXEON_HAXE_BIN:-"$haxeon_dir/.tools/haxe/haxe"}
hashlink_bin=${HAXEON_HASHLINK_BIN:-"$haxeon_dir/.tools/hashlink/hl"}
native_build=${NKSIM_SHARED_BUILD_DIR:-"$simkit_dir/build"}
output=$(mktemp --suffix=.hl)
trap 'rm -f "$output"' EXIT

if [[ ! -x "$haxe_bin" || ! -x "$hashlink_bin" ]]; then
    echo "check-haxeon-runtime: missing Haxeon or HashLink toolchain" >&2
    exit 1
fi

"$haxe_bin" --cwd "$haxeon_dir" -cp src --run compiler.tools.HaxeonCompiler \
    --output="$output" \
    --entry=SimBindingRuntime \
    --root="$module_dir/tests/haxeon" \
    --root="$module_dir/bindings/haxe" \
    --root="$scenekit_dir/scene/bindings/haxe" \
    --ffi-interface="$scenekit_dir/scene/bindings/nativekit-scene.hxi" \
    --ffi-interface="$module_dir/bindings/nativekit-sim.hxi" \
    --ffi-projection="$module_dir/bindings/nativekit-sim.hxmap" \
    "$module_dir/tests/haxeon/SimBindingRuntime.hx"

library_path="$native_build/sim_core:$native_build/scenekit/scene:$native_build/nativekit:$haxeon_dir/out:$haxeon_dir/.tools/hashlink"
if [[ "$(uname -s)" == "Darwin" ]]; then
    export DYLD_LIBRARY_PATH="$library_path${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
else
    export LD_LIBRARY_PATH="$library_path${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

set +e
"$hashlink_bin" "$output"
status=$?
set -e
if [[ $status -ne 42 ]]; then
    echo "check-haxeon-runtime: fixture failed with exit $status" >&2
    exit 1
fi
echo "check-haxeon-runtime: executed simulation bindings"
