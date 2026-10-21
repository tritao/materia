#!/usr/bin/env bash
set -euo pipefail
router_repo=$(pwd -P)
router_heavy=${MATERIA_HEAVY_RUN:-/home/joao/dev/materia-cache/tools/heavy-run}
export HAXEON_COMPILER_SERVER=0
mkdir -p build/sheet-router-work/sr12
"$router_heavy" --mem 3G --need 1G --label router-sheet-nesting-build haxeon/scripts/haxeon build --compiler-only --project camkit/tests/nesting/haxeon.json
"$router_heavy" --mem 3G --need 1G --label router-sheet-schema-build haxeon/scripts/haxeon build --compiler-only --project manufacturingkit/tests/haxeon.json
export LD_LIBRARY_PATH="$router_repo/haxeon/out:$router_repo/haxeon/.tools/hashlink${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
for router_provider in "$router_repo/machinekit/tests/build/host/native"/*; do
  [[ ! -d "$router_provider" ]] || LD_LIBRARY_PATH+=":$router_provider"
done
"$router_heavy" --mem 3G --need 1G --label router-sheet-nesting-tests haxeon/.tools/hashlink/hl build/sheet-router-work/sr12/core/host/main.hl build/sheet-router-work/sr12/nesting-validation.json
"$router_heavy" --mem 3G --need 1G --label router-sheet-schema-tests haxeon/.tools/hashlink/hl manufacturingkit/tests/build/host/main.hl
