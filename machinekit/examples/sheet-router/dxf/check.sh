#!/usr/bin/env bash
set -euo pipefail
router_repo=$(pwd -P)
router_heavy=${MATERIA_HEAVY_RUN:-/home/joao/dev/materia-cache/tools/heavy-run}
export HAXEON_COMPILER_SERVER=0
"$router_heavy" --mem 3G --need 1G --label router-dxf-build haxeon/scripts/haxeon build --compiler-only --project cadkit/haxe/tests/dxf-import/haxeon.json
"$router_heavy" --mem 3G --need 1G --label router-dxf-writer-build haxeon/scripts/haxeon build --compiler-only --project cadkit/haxe/tests/haxeon-drawing.json
export LD_LIBRARY_PATH="$router_repo/haxeon/out:$router_repo/haxeon/.tools/hashlink${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$router_heavy" --mem 3G --need 1G --label router-dxf-tests haxeon/.tools/hashlink/hl build/sheet-router-work/sr10/host/main.hl
"$router_heavy" --mem 3G --need 1G --label router-dxf-writer-tests haxeon/.tools/hashlink/hl cadkit/haxe/tests/build/drawing/host/main.hl
