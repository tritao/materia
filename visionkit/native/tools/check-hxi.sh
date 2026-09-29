#!/usr/bin/env bash
set -euo pipefail
module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
materia_dir=$(dirname "$(dirname "$module_dir")")
haxeon_dir=${HAXEON_DIR:-"$materia_dir/haxeon"}
output=${1:-"$module_dir/bindings/visionkit.hxi"}
"$haxeon_dir/scripts/haxeon-ffi-audit" \
    --target=x86_64-linux-gnu \
    --target=x86_64-w64-windows-gnu \
    --target=x86_64-apple-darwin \
    --target=arm64-apple-darwin \
    --profile=portable-abi64 \
    --library=visionkit_core \
    --interface=VisionKitNative \
    --include="$module_dir/include" \
    --source-label=native/bindings/visionkit_import.h \
    --output="$output" \
    "$module_dir/bindings/visionkit_import.h"
