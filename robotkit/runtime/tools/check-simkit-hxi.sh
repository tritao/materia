#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
robotkit_dir=$(dirname "$module_dir")
materia_dir=$(dirname "$robotkit_dir")
haxeon_dir=${HAXEON_DIR:-"$materia_dir/haxeon"}
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
    --dependency-hxi="$module_dir/bindings/robotkit-runtime.hxi" \
    --include="$module_dir/include" \
    --exclude-header="$module_dir/include/robotkit_runtime.h" \
    --source-label=runtime/bindings/robotkit_simkit_import.h \
    --output="$output" \
    "$module_dir/bindings/robotkit_simkit_import.h"

echo "check-hxi: wrote $output"
