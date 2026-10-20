#!/usr/bin/env bash
set -euo pipefail
# Run from the repository root. Inputs are exported model packets, the prepared
# fabrication directory and the retained pre-review weld-detail JSON directory.
if [[ $# != 4 ]]; then
  echo "Usage: $0 <packets> <fabrication> <previous-weld-details> <build-output>" >&2
  exit 2
fi
router_repo=$(pwd -P)
router_packets=$1
router_fabrication=$2
router_previous=$3
router_output=$(realpath -m "$4")
case "$router_output" in "$router_repo"/build/*) ;; *) echo "Output must be under this checkout's build/" >&2; exit 2;; esac
if [[ -d "$router_output" ]] && [[ -n $(find "$router_output" -type l -print -quit) ]]; then
  echo "Refusing an output tree containing symlinks" >&2
  exit 2
fi
router_heavy=${MATERIA_HEAVY_RUN:-/home/joao/dev/materia-cache/tools/heavy-run}
router_ccx=${SHEET_ROUTER_CCX:-machinekit/examples/bench-mill/build/fem/tools/calculix/usr/bin/ccx}
router_sampler=${SHEET_ROUTER_PLANNER_SAMPLE:-build/sheet-router-evidence/unpacked/sr6-qualified/planner-sample}
router_python=machinekit/examples/sheet-router/fem
[[ -x "$router_ccx" && -x "$router_sampler" && -x "$router_heavy" ]] || { echo "Existing CCX, planner sampler and heavy-run are required; no native rebuild is performed" >&2; exit 2; }
export OPENBLAS_NUM_THREADS=1
export LD_LIBRARY_PATH="$(dirname "$router_ccx")/../lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
mkdir -p "$router_output"
"$router_heavy" --mem 6G --need 1G --label router-weld-envelope python3 "$router_python/qualify_welds.py" \
  --packets "$router_packets" --fabrication "$router_fabrication" --ccx "$router_ccx" --output "$router_output/reactions"
python3 "$router_python/weld_report.py" --results "$router_output/reactions" --fabrication "$router_fabrication" \
  --packets "$router_packets" --output "$router_output/static-qualification.json"
"$router_heavy" --mem 5G --need 1G --label router-weld-mesh python3 "$router_python/weld_convergence.py" \
  --packets "$router_packets" --fabrication "$router_fabrication" --ccx "$router_ccx" --output "$router_output/coarse" \
  --envelope "$router_output/static-qualification.json"
python3 "$router_python/weld_report.py" --results "$router_output/coarse" --fabrication "$router_fabrication" \
  --packets "$router_packets" --output "$router_output/coarse-qualification.json"
"$router_heavy" --mem 3G --need 1G --label router-weld-motion python3 "$router_python/weld_motion_checks.py" \
  --results "$router_output/reactions" --fabrication "$router_fabrication" --packets "$router_packets" \
  --sampler "$router_sampler" --output "$router_output"
python3 "$router_python/weld_qualification.py" --static "$router_output/static-qualification.json" \
  --coarse "$router_output/coarse-qualification.json" --fabrication "$router_fabrication" --dynamic "$router_output" \
  --packets "$router_packets" --previous "$router_previous" --output "$router_output/published"
