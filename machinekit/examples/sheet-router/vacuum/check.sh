#!/usr/bin/env bash
set -euo pipefail
router_repo=$(pwd -P)
router_output="$router_repo/build/sheet-router-work/sr8"
router_heavy=${MATERIA_HEAVY_RUN:-/home/joao/dev/materia-cache/tools/heavy-run}
mkdir -p "$router_output"
export HAXEON_COMPILER_SERVER=0
"$router_heavy" --mem 6G --need 1G --label stock-components-build haxeon/scripts/haxeon build --compiler-only --project stockkit/tests/components/haxeon.json --output "$router_output/core.hl"
"$router_heavy" --mem 6G --need 1G --label router-vacuum-build haxeon/scripts/haxeon build --compiler-only --project machinekit/examples/sheet-router/haxeon.vacuum.json --output "$router_output/vacuum.hl"
export LD_LIBRARY_PATH="$router_repo/haxeon/out:$router_repo/haxeon/.tools/hashlink:$router_repo/stockkit/tests/build/host/native/stockkit${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
for router_provider in "$router_repo/machinekit/tests/build/host/native"/*; do
  [[ ! -d "$router_provider" ]] || LD_LIBRARY_PATH+=":$router_provider"
done
"$router_heavy" --mem 3G --need 1G --label stock-components-native haxeon/.tools/hashlink/hl "$router_output/core.hl"
"$router_heavy" --mem 3G --need 1G --label router-vacuum-native haxeon/.tools/hashlink/hl "$router_output/vacuum.hl"
