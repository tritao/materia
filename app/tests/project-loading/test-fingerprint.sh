#!/usr/bin/env bash
# The fingerprint a project build reports (MateriaProjectModuleBuild writes it, from what the compile read) must change for
# every kind of input and for nothing else, and must not depend on whether the compile was clean or incremental.
set -euo pipefail
repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
fixture=$(mktemp -d /tmp/materia-fingerprint.XXXXXX)
trap 'rm -rf -- "$fixture"' EXIT
export XDG_CACHE_HOME="$fixture/cache"
mkdir -p "$fixture/a" "$fixture/b" "$fixture/app/tools" "$fixture/app/build/host/native/kit" "$fixture/cache"
cp "$repo_root"/app/tools/*.hx "$fixture/app/tools/"
cat > "$fixture/haxeon.json" <<'JSON'
{"version":1,"package":{"name":"fingerprint-fixture"},"entry":"GraphPreview","sourceRoots":["a","b"],"ffi":{"interfaces":["fx.hxi"]},"target":"host"}
JSON
cat > "$fixture/b/GraphPreview.hx" <<'JSON'
class GraphPreview {public static function preview():haxe.io.Bytes return haxe.io.Bytes.ofString(GraphDependency.value());}
JSON
cat > "$fixture/b/GraphDependency.hx" <<'JSON'
class GraphDependency {public static function value():String return "first" + (haxeon.build.CodeIdentity.ofModule("GraphDependency") == null ? "" : "");}
JSON
cp "$repo_root/haxeon/tests/ffi/host_error.hxi" "$fixture/fx.hxi"
echo one > "$fixture/declared.txt"
echo native > "$fixture/app/build/host/native/kit/libkit.hdll"
haxe="$repo_root/haxeon/.tools/haxe/haxe"
"$haxe" --cwd "$repo_root/haxeon" -cp src -cp "$repo_root/app/tools" -main MateriaProjectModuleBuild -hl "$fixture/build.hl"
n=0
fingerprint() { # prints the fingerprint the build reports
  rm -f "$fixture/out.record.json"
  "$repo_root/haxeon/scripts/hl" "$fixture/build.hl" "$fixture/haxeon.json" GraphPreview preview "$fixture/out" \
    "$repo_root/haxeon" false "$fixture/app/tools" "$fixture/declared.txt" > "$fixture/build.log" 2>&1 \
    || { cat "$fixture/build.log"; exit 1; }
  grep -o '"fingerprint":"[0-9a-f]*"' "$fixture/out.record.json"
}
same() { n=$((n+1)); test "$1" = "$2" || { echo "FAIL: $3 changed the fingerprint"; exit 1; }; }
differs() { n=$((n+1)); test "$1" != "$2" || { echo "FAIL: $3 did not change the fingerprint"; exit 1; }; }
first=$(fingerprint)
# A program that reads code identities contains a module the compiler generates; the record must still be confirmable.
n=$((n+1)); grep -q '"recordable":true' "$fixture/out.record.json" || { echo "FAIL: the record is not confirmable"; exit 1; }
same "$first" "$(fingerprint)" "an unchanged repeat"
clean=$(HAXEON_COMPILER_SERVER=0 fingerprint)
same "$first" "$clean" "a clean (one-shot) compile instead of the incremental one"
echo 'class Unrelated {public static function unused():Int return 1;}' > "$fixture/b/Unrelated.hx"
same "$first" "$(fingerprint)" "a module nothing imports"
rm "$fixture/b/Unrelated.hx"
echo 'class GraphDependency {public static function value():String return "changed" + (haxeon.build.CodeIdentity.ofModule("GraphDependency") == null ? "" : "");}' > "$fixture/b/GraphDependency.hx"
changed=$(fingerprint)
differs "$first" "$changed" "editing an imported module"
echo 'class GraphDependency {public static function value():String return "first" + (haxeon.build.CodeIdentity.ofModule("GraphDependency") == null ? "" : "");}' > "$fixture/b/GraphDependency.hx"
same "$first" "$(fingerprint)" "restoring the module"
echo 'class GraphDependency {public static function value():String return "shadow" + (haxeon.build.CodeIdentity.ofModule("GraphDependency") == null ? "" : "");}' > "$fixture/a/GraphDependency.hx"
differs "$first" "$(fingerprint)" "a module in an earlier root shadowing an imported one"
rm "$fixture/a/GraphDependency.hx"
echo one-edited > "$fixture/declared.txt"
differs "$first" "$(fingerprint)" "editing a declared input file"
echo one > "$fixture/declared.txt"
sed -i 's/"target":"host"/"target":"host","defines":["fingerprint-probe"]/' "$fixture/haxeon.json"
differs "$first" "$(fingerprint)" "editing the project manifest"
sed -i 's/,"defines":\["fingerprint-probe"\]//' "$fixture/haxeon.json"
same "$first" "$(fingerprint)" "restoring the manifest"
sed -i 's/^}$/\textern fn another(value: i32) -> i32 @symbol("another_call");\n}/' "$fixture/fx.hxi"
differs "$first" "$(fingerprint)" "editing an FFI interface"
cp "$repo_root/haxeon/tests/ffi/host_error.hxi" "$fixture/fx.hxi"
same "$first" "$(fingerprint)" "restoring the FFI interface"
echo native-changed > "$fixture/app/build/host/native/kit/libkit.hdll"
differs "$first" "$(fingerprint)" "a native library with other content"
echo native > "$fixture/app/build/host/native/kit/libkit.hdll"
same "$first" "$(fingerprint)" "restoring the native library"
echo nativf > "$fixture/app/build/host/native/kit/libkit.hdll"
differs "$first" "$(fingerprint)" "a native library with the same size and other content"
echo native > "$fixture/app/build/host/native/kit/libkit.hdll"
touch -d '2001-01-01' "$fixture/app/build/host/native/kit/libkit.hdll"
same "$first" "$(fingerprint)" "a native library rebuilt with identical content (new mtime)"
differs "$first" "$(HAXEON_INLINE=0 fingerprint)" "a compiler option"
echo "Fingerprint: $n checks (stable, clean = incremental, unrelated module ignored, module edit, shadowing module, declared input, FFI interface, manifest, native library content, rebuild with the same content, compiler option)."
