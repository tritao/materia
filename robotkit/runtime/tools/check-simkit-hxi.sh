#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
robotkit_dir=$(dirname "$module_dir")
materia_dir=$(dirname "$robotkit_dir")
haxeon_dir=${HAXEON_DIR:-"$materia_dir/haxeon"}
simkit_dir=${SIMKIT_DIR:-"$materia_dir/simkit"}
scenekit_dir=${SCENEKIT_DIR:-"$materia_dir/scenekit"}
nativekit_dir=${NATIVEKIT_DIR:-"$materia_dir/nativekit"}
output=${1:-"$module_dir/bindings/robotkit-simkit.hxi"}

"$haxeon_dir/scripts/haxeon-ffi-audit" \
    --target=x86_64-linux-gnu \
    --target=x86_64-w64-windows-gnu \
    --target=x86_64-apple-darwin \
    --target=arm64-apple-darwin \
    --profile=portable-abi64 \
    --library=robotkit_runtime \
    --interface=RobotKitSimKit \
    --depends=RobotKitRuntime \
    --depends=NativeKitSim \
    --dependency-hxi="$module_dir/bindings/robotkit-runtime.hxi" \
    --dependency-hxi="$nativekit_dir/bindings/haxe/nativekit.hxi" \
    --dependency-hxi="$scenekit_dir/scene/bindings/nativekit-scene.hxi" \
    --dependency-hxi="$simkit_dir/sim_core/bindings/nativekit-sim.hxi" \
    --include="$module_dir/include" \
    --include="$simkit_dir/sim_core/include" \
    --include="$scenekit_dir/scene/include" \
    --include="$nativekit_dir/include" \
    --exclude-header="$module_dir/include/robotkit_runtime.h" \
    --exclude-header="$simkit_dir/sim_core/include/nativekit_sim.h" \
    --exclude-header="$simkit_dir/sim_core/include/nativekit_sim_session.h" \
    --exclude-header="$scenekit_dir/scene/include/nativekit_scene.h" \
    --exclude-header="$nativekit_dir/include/nativekit.h" \
    --source-label=runtime/bindings/robotkit_simkit_import.h \
    --output="$output" \
    "$module_dir/bindings/robotkit_simkit_import.h"

echo "check-hxi: wrote $output"
