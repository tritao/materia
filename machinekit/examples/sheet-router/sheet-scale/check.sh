#!/usr/bin/env bash
set -euo pipefail
router_repo=$(pwd -P)
router_output="$router_repo/build/sheet-router-work/sr9"
router_heavy=${MATERIA_HEAVY_RUN:-/home/joao/dev/materia-cache/tools/heavy-run}
mkdir -p "$router_output"
export HAXEON_COMPILER_SERVER=0
for router_mode in compact dense; do
  router_uniform=ON
  [[ "$router_mode" != dense ]] || router_uniform=OFF
  "$router_heavy" --mem 3G --need 1G --label "router-sheet-$router_mode" bash -c '
    cmake -S stockkit/core -B "$1" -DCMAKE_BUILD_TYPE=Release -DSK_UNIFORM_TILES="$2" &&
    cmake --build "$1" --parallel 2 &&
    ctest --test-dir "$1" -E stockkit_sheet_tests --output-on-failure
  ' bash "$router_output/$router_mode" "$router_uniform"
  router_args=()
  [[ "$router_mode" != dense ]] || router_args+=(--small)
  "$router_heavy" --mem 3G --need 1G --label "router-sheet-$router_mode-measure" \
    "$router_output/$router_mode/stockkit_sheet_tests" "${router_args[@]}" | tee "$router_output/$router_mode-results.log"
done
python3 - "$router_output" <<'PY'
import pathlib,sys
root=pathlib.Path(sys.argv[1])
fixtures=lambda p:[line for line in p.read_text().splitlines() if line.startswith('fixture ')]
assert fixtures(root/'compact-results.log')==fixtures(root/'dense-results.log'),'dense/compact fixture mismatch'
PY
"$router_heavy" --mem 3G --need 1G --label router-sheet-program-build haxeon/scripts/haxeon build --compiler-only --project projectkit/tests/haxeon.json
"$router_heavy" --mem 3G --need 1G --label router-sheet-preview-build haxeon/scripts/haxeon build --compiler-only --project stockkit/tests/sheet-scale/haxeon.json
"$router_heavy" --mem 6G --need 1G --label router-sheet-app-build haxeon/scripts/haxeon build --compiler-only --project app/tests/sheet-machining/haxeon.json
export LD_LIBRARY_PATH="$router_repo/haxeon/out:$router_repo/haxeon/.tools/hashlink:$router_output/compact:$router_repo/stockkit/tests/build/host/native/stockkit${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
for router_provider in "$router_repo/machinekit/tests/build/host/native"/*; do
  [[ ! -d "$router_provider" ]] || LD_LIBRARY_PATH+=":$router_provider"
done
(
  cd "$router_repo/projectkit/tests"
  "$router_heavy" --mem 3G --need 1G --label router-sheet-program-tests "$router_repo/haxeon/.tools/hashlink/hl" build/host/main.hl
)
"$router_heavy" --mem 3G --need 1G --label router-sheet-preview-tests env LD_PRELOAD="$router_output/compact/libstockkit_core.so" \
  haxeon/.tools/hashlink/hl "$router_output/preview/host/main.hl"
"$router_heavy" --mem 3G --need 1G --label router-sheet-app-tests \
  haxeon/.tools/hashlink/hl "$router_output/artifact-app/host/main.hl"
