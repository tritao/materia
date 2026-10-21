#!/usr/bin/env bash
# Compiles the docs runtime (src/DocsRuntime.hx with CncKit and ToolpathKit) to Wasm GC for the site's
# live widgets. The result is committed as docs-runtime.wasm with docs-runtime.inputs, a hash of what it was
# built from (inputs.py), because the site's own build has no Haxe toolchain. Run after changing the runtime,
# those kits or the Haxeon pin; CI fails while the hash is stale.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
haxeon="$repo/haxeon"
out="$here/build"
mkdir -p "$out"
compiler="$out/haxeon-compiler.hl"
if [[ ! -f "$compiler" ]] || [[ -n $(find "$haxeon/src" -name '*.hx' -newer "$compiler" -print -quit) ]]; then
	"$haxeon/.tools/haxe/haxe" --cwd "$haxeon" -cp src -hl "$compiler" -main compiler.tools.HaxeonCompiler
fi
kits=("$repo/cnckit/haxe" "$repo/toolpathkit/haxe")
sources=("$here/src/DocsRuntime.hx")
for kit in "${kits[@]}"; do
	while IFS= read -r file; do sources+=("$file"); done < <(find "$kit" -name '*.hx' | sort)
done
roots=(--root="$here/src")
for kit in "${kits[@]}"; do roots+=(--root="$kit"); done
rm -f "$out/docs-runtime.wasm"
(cd "$haxeon" && LD_LIBRARY_PATH="$haxeon/out:$haxeon/.tools/hashlink${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
	HAXEON_WASM_LEGACY_EXCEPTIONS=1 \
	"$haxeon/.tools/hashlink/hl" "$compiler" --target=wasm-gc --output="$out/docs-runtime.wasm" --entry=DocsRuntime \
	"${roots[@]}" "${sources[@]}") | grep -vE '^(loading|compiler|driver) ' || true
[[ -f "$out/docs-runtime.wasm" ]] || { echo "docs runtime did not compile" >&2; exit 1; }
cp "$out/docs-runtime.wasm" "$here/docs-runtime.wasm"
python3 "$here/inputs.py" > "$here/docs-runtime.inputs"
node --experimental-strip-types "$here/check.mjs"
echo "docs-runtime.wasm: $(stat -c %s "$here/docs-runtime.wasm") bytes, $(gzip -9c "$here/docs-runtime.wasm" | wc -c) gzipped"
