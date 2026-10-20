#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 1 ]]; then
  echo "Usage: $0 <retained-sr6-manifest-directory-with-y2600.json-and-y3150.json>" >&2
  exit 2
fi
router_repo=$(pwd -P)
router_inputs=$(realpath "$1")
router_output="$router_repo/build/sheet-router-work/sr7"
router_heavy=${MATERIA_HEAVY_RUN:-/home/joao/dev/materia-cache/tools/heavy-run}
router_ccx=${SHEET_ROUTER_CCX:-"$router_repo/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/bin/ccx"}
router_sampler=${SHEET_ROUTER_PLANNER_SAMPLE:-"$router_output/planner-sample"}
[[ -x "$router_sampler" && -x "$router_ccx" ]] || { echo "Run the ATC check first to install the pinned planner; existing CCX is required" >&2; exit 2; }
mkdir -p "$router_output"
export HAXEON_COMPILER_SERVER=0
"$router_heavy" --mem 6G --need 1G --label router-atc-mass-build haxeon/scripts/haxeon build --compiler-only --project machinekit/examples/sheet-router/haxeon.atc-mass.json --output "$router_output/mass.hl"
export LD_LIBRARY_PATH="$router_repo/haxeon/out:$router_repo/haxeon/.tools/hashlink${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
for router_provider in "$router_repo/machinekit/tests/build/host/native"/*; do
  [[ ! -d "$router_provider" ]] || LD_LIBRARY_PATH+=":$router_provider"
done
"$router_heavy" --mem 3G --need 1G --label router-atc-mass haxeon/.tools/hashlink/hl "$router_output/mass.hl"
export OPENBLAS_NUM_THREADS=1
export LD_LIBRARY_PATH="$(dirname "$router_ccx")/../lib/x86_64-linux-gnu"
"$router_heavy" --mem 6G --need 1G --label router-atc-structure python3 machinekit/examples/sheet-router/atc/structural_check.py \
  --inputs "$router_inputs" --mass "$router_output" --fabrication machinekit/examples/sheet-router/fabrication \
  --ccx "$router_ccx" --sampler "$router_sampler" --output "$router_output/structural"
