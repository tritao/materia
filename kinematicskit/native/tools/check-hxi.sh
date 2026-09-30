#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
materia_dir=$(dirname "$(dirname "$module_dir")")
haxeon_dir=${HAXEON_DIR:-"$materia_dir/haxeon"}
output=${1:-"$module_dir/bindings/kinematicskit.hxi"}

"$haxeon_dir/scripts/haxeon-ffi-audit" \
    --target=x86_64-linux-gnu \
    --target=x86_64-w64-windows-gnu \
    --target=x86_64-apple-darwin \
    --target=arm64-apple-darwin \
    --profile=portable-abi64 \
    --library=kinematicskit_core \
    --interface=KinematicsKitNative \
    --include="$module_dir/include" \
    --source-label=native/bindings/kinematicskit_import.h \
    --output="$output" \
    "$module_dir/bindings/kinematicskit_import.h"

echo "check-hxi: wrote $output"
