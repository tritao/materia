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
hashlink_bin=${HAXEON_HASHLINK_BIN:-"$haxeon_dir/.tools/hashlink/hl"}
native_build=${NKSIM_MUJOCO_SHARED_BUILD_DIR:-"$repo_dir/../../nativekit-builds/sim-mujoco-vendored"}
output=$(mktemp --suffix=.hl)
trap 'rm -f "$output"' EXIT

if [[ ! -x "$haxe_bin" || ! -x "$hashlink_bin" ]]; then
    echo "check-haxeon-runtime: missing Haxeon or HashLink toolchain" >&2
    exit 1
fi

"$haxe_bin" --cwd "$haxeon_dir" -cp src --run compiler.tools.HaxeonCompiler \
    --output="$output" \
    --entry=MujocoBindingRuntime \
    --root="$module_dir/tests/haxeon" \
    --root="$module_dir/bindings/haxe" \
    --root="$repo_dir/modules/sim_core/bindings/haxe" \
    --root="$repo_dir/modules/scene/bindings/haxe" \
    --ffi-interface="$repo_dir/modules/scene/bindings/nativekit-scene.hxi" \
    --ffi-interface="$repo_dir/modules/sim_core/bindings/nativekit-sim.hxi" \
    --ffi-interface="$module_dir/bindings/nativekit-sim-mujoco.hxi" \
    --ffi-projection="$repo_dir/modules/sim_core/bindings/nativekit-sim.hxmap" \
    "$module_dir/tests/haxeon/MujocoBindingRuntime.hx"

library_path="$native_build:$native_build/modules/scene:$native_build/modules/sim_core:$native_build/modules/sim_mujoco:$haxeon_dir/out:$haxeon_dir/.tools/hashlink"
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
echo "check-haxeon-runtime: executed MuJoCo simulation bindings"
