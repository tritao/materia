#!/usr/bin/env bash
set -euo pipefail
router_repo=$(pwd -P)
router_heavy=${MATERIA_HEAVY_RUN:-/home/joao/dev/materia-cache/tools/heavy-run}
export HAXEON_COMPILER_SERVER=0
"$router_heavy" --mem 6G --need 1G --label router-electrical-build haxeon/scripts/haxeon build --compiler-only --project machinekit/examples/sheet-router/haxeon.electrical.json
router_libs="$router_repo/haxeon/out:$router_repo/haxeon/.tools/hashlink"
for router_provider in "$router_repo/machinekit/tests/build/host/native"/*; do
 [[ ! -d "$router_provider" ]] || router_libs+=":$router_provider"
done
"$router_heavy" --mem 6G --need 1G --label router-electrical-export env LD_LIBRARY_PATH="$router_libs" haxeon/.tools/hashlink/hl build/sheet-router-work/sr17/host/main.hl build/sheet-router-work/sr17/model-channels.json
python3 machinekit/examples/sheet-router/electronics/schematic.py build/sheet-router-work/sr17/model-channels.json
python3 machinekit/examples/sheet-router/electronics/check.py
