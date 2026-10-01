#!/usr/bin/env bash
# Builds the reference editor for the browser into app/build/web/site.
#
#   1. Generates the wasm32 FFI interfaces of every native kit.
#   2. Compiles the editor to a Haxeon guest module (entry app.MainWeb), wasm-gc by default or
#      wasm32 with MATERIA_WEB_TARGET=wasm32.
#   3. Links the Emscripten host (NativeKit, UIKit, SceneKit) and exports every C
#      function the guest imports from those libraries.
#   4. Assembles the page, both modules and the fonts.
#
# Serve the result over HTTP: python3 -m http.server --directory app/build/web/site 8080
set -euo pipefail

app_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
materia_dir=$(dirname "$app_dir")
haxeon_dir=${HAXEON_DIR:-"$materia_dir/haxeon"}
emsdk_dir=${EMSDK_DIR:-"$materia_dir/nativekit/.tools/emsdk"}
build_dir=${MATERIA_WEB_BUILD_DIR:-"$app_dir/build/web"}
build_type=${CMAKE_BUILD_TYPE:-Release}
# wasm-gc keeps Haxe values as Wasm GC objects; wasm32 keeps them in linear memory with Haxeon's own collector
# and needs no Wasm GC support, at several times the frame cost. The host is the same.
guest_target=${MATERIA_WEB_TARGET:-wasm-gc}
site_dir="$build_dir/site"
guest="$build_dir/materia_guest.wasm"

# Linear memory: the Emscripten host heap ends at host_limit and the guest's
# managed heap fills the rest. Both sides must agree, so both are derived here.
page_size=65536
host_limit=${MATERIA_WEB_HOST_HEAP_BYTES:-134217728}
memory_size=${MATERIA_WEB_MEMORY_BYTES:-805306368}
if (( host_limit % page_size != 0 || memory_size % page_size != 0 || host_limit >= memory_size )); then
	echo "build.sh: memory sizes must be 64 KiB multiples with the host heap below the total" >&2
	exit 2
fi

if [[ ! -f "$emsdk_dir/emsdk_env.sh" ]]; then
	echo "build.sh: Emscripten is not installed; run nativekit/tools/setup-web.sh" >&2
	exit 1
fi
if [[ ! -x "$haxeon_dir/.tools/hashlink/hl" ]]; then
	echo "build.sh: the Haxeon toolchain is missing; run haxeon/scripts/bootstrap-tools.sh" >&2
	exit 1
fi
mkdir -p "$build_dir" "$site_dir/assets"

echo "== wasm32 FFI interfaces"
"$app_dir/web/tools/generate-wasm-hxi.sh" "$build_dir/hxi"

echo "== Haxeon $guest_target guest"
contract="$build_dir/memory_contract.json"
cat > "$contract" <<JSON
{
  "name": "nativekit-haxeon-linear-memory",
  "version": 1,
  "address_model": "wasm32",
  "page_size": $page_size,
  "host_base": 0,
  "host_limit": $host_limit,
  "guest_base": $host_limit,
  "guest_limit": $memory_size,
  "memory_size": $memory_size
}
JSON
# The compiler runs as HashLink bytecode; rebuild it when its sources change.
compiler="$build_dir/haxeon-compiler.hl"
if [[ ! -f "$compiler" ]] || [[ -n $(find "$haxeon_dir/src" -name '*.hx' -newer "$compiler" -print -quit) ]]; then
	"$haxeon_dir/.tools/haxe/haxe" --cwd "$haxeon_dir" -cp src -hl "$compiler" -main compiler.tools.HaxeonCompiler
fi
mapfile -t guest_arguments < <(python3 "$app_dir/web/tools/guest-arguments.py" "$app_dir/haxeon.json" "$build_dir/hxi")
(cd "$haxeon_dir" && LD_LIBRARY_PATH="$haxeon_dir/out:$haxeon_dir/.tools/hashlink${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
	HAXEON_WASM_LEGACY_EXCEPTIONS=1 \
	"$haxeon_dir/.tools/hashlink/hl" "$compiler" --target="$guest_target" --output="$guest" --entry=app.MainWeb \
	--wasm-import-memory --wasm-memory-contract="$contract" \
	"${guest_arguments[@]}") > "$build_dir/guest-compile.log" 2>&1 || true
grep -E '^compiled' "$build_dir/guest-compile.log" || true
if [[ ! -f "$guest" || "$guest" -ot "$contract" ]]; then
	grep -vE '^(loading|compiler|driver) ' "$build_dir/guest-compile.log" | tail -20 >&2
	echo "build.sh: the guest did not compile; see $build_dir/guest-compile.log" >&2
	exit 1
fi

echo "== Emscripten host"
# Export the guest's imports from the libraries the host links; haxeon-host.js
# gives the guest's remaining imports stubs that throw when called.
exports="$build_dir/exports.json"
node - "$guest" "$exports" <<'NODE'
const fs = require("fs");
const [guestPath, exportsPath] = process.argv.slice(2);
const linked = new Set(["nativekit", "nativekit_gpu", "nativekit_ui", "nativekit_scene", "nativekit_scene_render"]);
const module = new WebAssembly.Module(fs.readFileSync(guestPath));
const names = new Set(["_main", "_nk_last_error", "_nkgpu_last_error", "_nkui_haxeon_memory_contract_status",
  "_nkui_haxeon_memory_contract_version", "_nkui_haxeon_memory_contract_page_size",
  "_nkui_haxeon_memory_contract_host_base", "_nkui_haxeon_memory_contract_host_limit",
  "_nkui_haxeon_memory_contract_guest_base", "_nkui_haxeon_memory_contract_guest_limit",
  "_nkui_haxeon_memory_contract_memory_size"]);
for (const entry of WebAssembly.Module.imports(module))
  if (entry.kind === "function" && linked.has(entry.module)) names.add("_" + entry.name);
fs.writeFileSync(exportsPath, JSON.stringify([...names].sort()));
NODE
source "$emsdk_dir/emsdk_env.sh" >/dev/null 2>&1
emcmake cmake -S "$app_dir/web" -B "$build_dir/host" -G Ninja -DCMAKE_BUILD_TYPE="$build_type" \
	-DMATERIA_WEB_GUEST_WASM="$guest" -DMATERIA_WEB_EXPORTS_FILE="$exports" \
	-DNK_WASM_HOST_HEAP_LIMIT="$host_limit" -DMATERIA_WEB_GUEST_MEMORY_LIMIT="$memory_size" >/dev/null
cmake --build "$build_dir/host" --target materia_web
# Instantiation stops at the first mismatched import; report them all here.
node "$app_dir/web/tools/check-imports.js" "$guest" "$build_dir/host/materia_web.wasm" "$build_dir/host/materia_web.js"

echo "== Site"
cp "$build_dir/host/materia_web.js" "$build_dir/host/materia_web.wasm" "$guest" "$site_dir/"
cp "$app_dir/web/index.html" "$app_dir/web/materia.js" "$haxeon_dir/stdlib/haxeon/wasm/haxeon-host.js" "$site_dir/"
fonts="$materia_dir/uikit/vendor/skribidi/example/data"
cp "$fonts/IBMPlexSans-Regular.ttf" "$fonts/NotoEmoji-Regular.ttf" "$site_dir/assets/"
echo "Built $site_dir"
echo "Serve it with: python3 -m http.server --directory \"$site_dir\" 8080"
