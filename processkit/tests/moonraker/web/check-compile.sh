#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "$0")/../../../.." && pwd)
mkdir -p "$root/build/rf4-web-client"
cd "$root/haxeon"
.tools/haxe/haxe -cp src --run compiler.tools.HaxeonCompiler \
  --target=wasm32 --entry=MoonrakerWebCompile --output=../build/rf4-web-client/client.wasm \
  --root=../processkit/tests/moonraker/web --root=../processkit/haxe --root=packages/platform/src \
  --ffi-interface=packages/platform/bindings/nativekit-wasm.hxi \
  --ffi-projection=packages/platform/bindings/nativekit.hxmap \
  --ffi-interface=packages/platform/bindings/nativekit-net-wasm.hxi \
  --ffi-projection=packages/platform/bindings/nativekit-net.hxmap \
  ../processkit/tests/moonraker/web/MoonrakerWebCompile.hx
