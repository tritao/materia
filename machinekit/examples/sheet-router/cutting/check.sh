#!/usr/bin/env bash
set -euo pipefail
router_repo=$(pwd -P)
router_heavy=${MATERIA_HEAVY_RUN:-/home/joao/dev/materia-cache/tools/heavy-run}
export HAXEON_COMPILER_SERVER=0
export LD_LIBRARY_PATH="$router_repo/haxeon/out:$router_repo/haxeon/.tools/hashlink:$router_repo/build/sheet-router-work/sr9/compact:$router_repo/stockkit/tests/build/host/native/stockkit${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
for router_provider in "$router_repo/machinekit/tests/build/host/native"/*; do
 [[ ! -d "$router_provider" ]] || LD_LIBRARY_PATH+=":$router_provider"
done
export LD_PRELOAD="$router_repo/build/sheet-router-work/sr9/compact/libstockkit_core.so"
mkdir -p build/sheet-router-work/sr13
for router_case in cutting sizing force-vacuum force-sheet; do
 "$router_heavy" --mem 6G --need 1G --label "router-$router_case-build" haxeon/scripts/haxeon build --compiler-only --project "machinekit/examples/sheet-router/haxeon.$router_case.json"
 if [[ "$router_case" == sizing ]]; then export SHEET_ROUTER_SIZING_OUTPUT=build/sheet-router-work/sr13/sizing; fi
 "$router_heavy" --mem 6G --need 1G --label "router-$router_case-check" haxeon/.tools/hashlink/hl "build/sheet-router-work/sr13/$router_case/host/main.hl" "build/sheet-router-work/sr13/$router_case.json"
done
