#!/usr/bin/env bash
set -euo pipefail
router_repo=$(pwd -P)
router_heavy=${MATERIA_HEAVY_RUN:-/home/joao/dev/materia-cache/tools/heavy-run}
export HAXEON_COMPILER_SERVER=0
"$router_heavy" --mem 3G --need 1G --label router-sheet-cam-build haxeon/scripts/haxeon build --compiler-only --project camkit/tests/sheets/haxeon.json
"$router_heavy" --mem 3G --need 1G --label router-sheet-legacy-build haxeon/scripts/haxeon build --compiler-only --project camkit/tests/sheets/haxeon.regression.json
"$router_heavy" --mem 6G --need 1G --label router-sheet-cabinet-build haxeon/scripts/haxeon build --compiler-only --project machinekit/examples/sheet-router/haxeon.sheet-ops.json
export LD_LIBRARY_PATH="$router_repo/haxeon/out:$router_repo/haxeon/.tools/hashlink:$router_repo/build/sheet-router-work/sr9/compact:$router_repo/stockkit/tests/build/host/native/stockkit${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
for router_provider in "$router_repo/machinekit/tests/build/host/native"/*; do
  [[ ! -d "$router_provider" ]] || LD_LIBRARY_PATH+=":$router_provider"
done
# Reuse the SR9 core. Build it with sheet-scale/check.sh if this working directory was cleaned.
export LD_PRELOAD="$router_repo/build/sheet-router-work/sr9/compact/libstockkit_core.so"
"$router_heavy" --mem 3G --need 1G --label router-sheet-cam-tests haxeon/.tools/hashlink/hl build/sheet-router-work/sr11/core/host/main.hl
"$router_heavy" --mem 3G --need 1G --label router-sheet-legacy-tests haxeon/.tools/hashlink/hl build/sheet-router-work/sr11/legacy/host/main.hl
"$router_heavy" --mem 6G --need 1G --label router-sheet-cabinet-tests haxeon/.tools/hashlink/hl build/sheet-router-work/sr11/router/host/main.hl build/sheet-router-work/sr11/cabinet-validation.json
