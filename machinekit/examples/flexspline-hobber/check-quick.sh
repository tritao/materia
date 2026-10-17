#!/usr/bin/env bash
# Small physical smoke checks. Reuses existing CAD/MuJoCo libraries read-only;
# rebuilds only RobotKit's matching runtime. Full reference/ISO checks stay separate.
set -euo pipefail
root=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$root"
evidence="$root/build/flexspline-quick"
mkdir -p "$evidence/logs"
cache=${HOBBER_NATIVE_CACHE:-"$root/app/build/host/native"}
if [[ ! -f "$cache/app/libnativekit_sim_mujoco.so" ]]; then
  cache=${HOBBER_NATIVE_CACHE:-"$root/../machine-tending/app/build/host/native"}
fi
[[ -f "$cache/app/libnativekit_sim_mujoco.so" ]] || { echo 'Set HOBBER_NATIVE_CACHE to an existing app/build/host/native directory.' >&2; exit 2; }
stock=${HOBBER_STOCK_NATIVE:-"$root/build/flexspline-hb3/native"}
[[ -f "$stock/libstockkit_core.so" ]] || { echo 'Set HOBBER_STOCK_NATIVE to the current StockKit native library directory.' >&2; exit 2; }
export HAXEON_COMPILER_SERVER=0
export CADKIT_OCCT_DIR=${CADKIT_OCCT_DIR:-"$HOME/dev/materia-cache/occt-f1dc4efb-release"}
export CMAKE_BUILD_PARALLEL_LEVEL=1
ulimit -s 65536
printf 'Reusing native dependencies from %s; rebuilding RobotKit only.\n' "$cache"
# Cache sources stay untouched; the matching private RobotRuntime layout is rebuilt locally.
cmake -S machinekit/tests/hobber/quick-native -B "$evidence/runtime" -G Ninja \
  -DCACHED="$cache/app" -DCMAKE_BUILD_TYPE=Release > "$evidence/logs/configure.log" 2>&1
cmake --build "$evidence/runtime" --target robotkit_runtime -j1 > "$evidence/logs/native.log" 2>&1
export LD_LIBRARY_PATH="$evidence/runtime/robotkit/runtime:$stock"
for provider in "$cache"/*; do [[ ! -d "$provider" ]] || LD_LIBRARY_PATH+=":$provider"; done
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$CADKIT_OCCT_DIR/lib:$root/haxeon/.tools/hashlink:$root/haxeon/out"
{
  git rev-parse HEAD
  git diff --binary HEAD | sha256sum
  git -C haxeon rev-parse HEAD
  git -C haxeon diff --binary HEAD | sha256sum
  sha256sum machinekit/examples/flexspline-hobber/check-quick.sh machinekit/tests/hobber/quick-native/CMakeLists.txt
  printf 'Native cache: %s\nStock: %s\n' "$cache" "$stock"
  sha256sum "$evidence/runtime/robotkit/runtime/librobotkit_runtime.so" "$stock/libstockkit_core.so" "$cache/app/libnativekit_sim_core.so" "$cache/app/libnativekit_sim_mujoco.so"
} > "$evidence/provenance.txt"
run_capped() {
  systemd-run --user --scope --quiet --collect -p MemoryMax=6G -p MemorySwapMax=0 \
    --unit="hobber-quick-$1-$$" -- "${@:2}"
}
case_name=${1:-all}
case "$case_name" in all|force|errors|app) ;; *) echo "Usage: $0 [all|force|errors|app]" >&2; exit 2;; esac
failed=0
for test_case in force errors app; do
  [[ "$case_name" == all || "$case_name" == "$test_case" ]] || continue
  case "$test_case" in
    force) manifest=stockkit/tests/hobbing-forces/haxeon.json; binary=stockkit/tests/hobbing-forces/build/host/main.hl; args=(quick);;
    errors) manifest=stockkit/tests/hobbing-errors/haxeon.json; binary=stockkit/tests/hobbing-errors/build/host/main.hl; args=(physical);;
    app) manifest=app/haxeon.project-source.json; binary=app/build/host/main.hl; args=();;
  esac
  printf 'Running %s check...\n' "$test_case"
  if run_capped "compile-$test_case" ./haxeon/scripts/haxeon build --compiler-only --project="$manifest" > "$evidence/logs/$test_case-build.log" 2>&1 \
    && run_capped "$test_case" /usr/bin/time -v -o "$evidence/logs/$test_case-time.log" \
      env PROJECT_SOURCE_ONLY=flexspline-hobber ./haxeon/.tools/hashlink/hl "$binary" "${args[@]}" > "$evidence/logs/$test_case.log" 2>&1; then
    printf '%s: PASS\n' "$test_case"
    sed -n '/QUICK_FORCE /p; /QUICK_ERRORS /p; /HOBBER_ACCEPTANCE /p' "$evidence/logs/$test_case.log"
  else
    printf '%s: FAIL (see %s/logs/%s*.log)\n' "$test_case" "$evidence" "$test_case"
    tail -n 10 "$evidence/logs/$test_case.log" 2>/dev/null || true
    failed=1
  fi
done
exit "$failed"
