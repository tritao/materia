#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
simkit_dir=$(dirname "$module_dir")
materia_dir=$(dirname "$simkit_dir")
scenekit_dir=${SCENEKIT_DIR:-"$materia_dir/scenekit"}
haxeon_dir=${HAXEON_DIR:-"$materia_dir/haxeon"}
haxe_bin=${HAXEON_HAXE_BIN:-"$haxeon_dir/.tools/haxe/haxe"}
output=$(mktemp --suffix=.hl)
trap 'rm -f "$output"' EXIT

"$haxe_bin" --cwd "$haxeon_dir" -cp src --run compiler.tools.HaxeonCompiler \
    --output="$output" \
    --entry=MujocoBindingCompile \
    --root="$module_dir/tests/haxeon" \
    --root="$module_dir/bindings/haxe" \
    --root="$simkit_dir/sim_core/bindings/haxe" \
    --root="$scenekit_dir/scene/bindings/haxe" \
    --ffi-interface="$scenekit_dir/scene/bindings/nativekit-scene.hxi" \
    --ffi-interface="$simkit_dir/sim_core/bindings/nativekit-sim.hxi" \
    --ffi-interface="$module_dir/bindings/nativekit-sim-mujoco.hxi" \
    --ffi-projection="$simkit_dir/sim_core/bindings/nativekit-sim.hxmap" \
    "$module_dir/tests/haxeon/MujocoBindingCompile.hx"

echo "check-haxeon: compiled MuJoCo simulation bindings"
