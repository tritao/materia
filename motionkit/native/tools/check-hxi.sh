#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
materia_dir=$(dirname "$(dirname "$module_dir")")
haxeon_dir=${HAXEON_DIR:-"$materia_dir/haxeon"}
output=${1:-"$module_dir/bindings/motionkit.hxi"}

"$haxeon_dir/scripts/haxeon-ffi-audit" \
    --target=x86_64-linux-gnu \
    --target=x86_64-w64-windows-gnu \
    --target=x86_64-apple-darwin \
    --target=arm64-apple-darwin \
    --profile=portable-abi64 \
    --library=motionkit_core \
    --interface=MotionKitNative \
    --include="$module_dir/include" \
    --include="$materia_dir/trajectorykit/native/include" \
    --depends=TrajectoryCore \
    --dependency-hxi="$materia_dir/trajectorykit/native/bindings/trajectory-core.hxi" \
    --exclude-header="$materia_dir/trajectorykit/native/include/trajectory_core.h" \
    --source-label=native/bindings/motionkit_import.h \
    --output="$output" \
    "$module_dir/bindings/motionkit_import.h"

echo "check-hxi: wrote $output"
