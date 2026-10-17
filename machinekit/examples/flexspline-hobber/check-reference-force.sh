#!/usr/bin/env bash
# Gate a separate force tolerance, run the full stroke, save per-pass radial history.
set -euo pipefail
root=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$root"
evidence=${HOBBER_FORCE_EVIDENCE:-"$root/build/flexspline-reference-force-fast"}
mkdir -p "$evidence"
case_name=${1:-run}
[[ "$case_name" == run || "$case_name" == benchmark ]] || { echo 'Usage: check-reference-force.sh [run|benchmark]' >&2; exit 2; }
cache=${HOBBER_NATIVE_CACHE:-"$root/../machine-tending/app/build/host/native"}
stock=${HOBBER_STOCK_NATIVE:-"$root/build/flexspline-force-parallel/native"}
if [[ -z "${HOBBER_STOCK_NATIVE:-}" ]]; then
  cmake -S stockkit/core -B "$stock" -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON > "$evidence/native-build.log" 2>&1
  cmake --build "$stock" --target stockkit_core -j2 >> "$evidence/native-build.log" 2>&1
fi
export HAXEON_COMPILER_SERVER=0
export CADKIT_OCCT_DIR=${CADKIT_OCCT_DIR:-"$HOME/dev/materia-cache/occt-f1dc4efb-release"}
export LD_LIBRARY_PATH="$stock:$CADKIT_OCCT_DIR/lib:$root/haxeon/.tools/hashlink:$root/haxeon/out"
for provider in "$cache"/*; do [[ ! -d "$provider" ]] || LD_LIBRARY_PATH+=":$provider"; done
export LD_LIBRARY_PATH
ulimit -s 65536
./haxeon/scripts/haxeon build --compiler-only --project=stockkit/tests/hobbing-forces/haxeon.json > "$evidence/build.log" 2>&1
cp stockkit/tests/hobbing-forces/build/host/main.hl "$evidence/force-main.hl"
if [[ "$case_name" == run ]]; then
  ./haxeon/scripts/haxeon build --compiler-only --project=machinekit/tests/hobber/haxeon.json >> "$evidence/build.log" 2>&1
  cp machinekit/tests/hobber/build/host/main.hl "$evidence/ratings-main.hl"
fi
{
  git rev-parse HEAD
  git diff --binary HEAD | sha256sum
  sha256sum "$evidence/force-main.hl" "$stock/libstockkit_core.so"
  [[ ! -f "$evidence/ratings-main.hl" ]] || sha256sum "$evidence/ratings-main.hl"
} > "$evidence/provenance.txt"
run_capped() {
  systemd-run --user --scope --quiet --collect -p MemoryMax=4G -p MemorySwapMax=0 \
    --unit="hobber-force-$1-$$" -- "${@:2}"
}
run_capped calibration ./haxeon/.tools/hashlink/hl "$evidence/force-main.hl" speed | tee "$evidence/calibration.log"
HOBBER_FORCE_TOLERANCE_MM=$(python3 - "$evidence/calibration.log" "$evidence/calibration.json" "${HOBBER_FORCE_TOLERANCE_MM:-}" "${HOBBER_FORCE_PARITY_LOG:-}" <<'PY'
import json,pathlib,sys
lines=pathlib.Path(sys.argv[1]).read_text().splitlines()
accept=next(json.loads(l.removeprefix('FORCE_SPEED_ACCEPTANCE ')) for l in lines if l.startswith('FORCE_SPEED_ACCEPTANCE '))
rows=[json.loads(l.removeprefix('FORCE_TOLERANCE ')) for l in lines if l.startswith('FORCE_TOLERANCE ')]
selected=float(sys.argv[3]) if sys.argv[3] else accept['recommendedToleranceMm']
if selected!=accept['fineToleranceMm']:
    matches=[r for r in rows if r['candidate']['toleranceMm']==selected]
    if len(matches)!=2 or not all(r['passesOnePercent'] for r in matches):
        raise SystemExit('Requested force tolerance did not pass both short-cut convergence checks')
if sys.argv[4]:
    oldlines=pathlib.Path(sys.argv[4]).read_text().splitlines()
    old={d['fixture']:d for l in oldlines if l.startswith('FORCE_THREADS ') for d in [json.loads(l.removeprefix('FORCE_THREADS '))]}
    new={d['fixture']:d for l in lines if l.startswith('FORCE_THREADS ') for d in [json.loads(l.removeprefix('FORCE_THREADS '))]}
    if len(old)!=2 or old.keys()!=new.keys() or any(old[k]['stockHash']!=new[k]['stockHash'] or old[k]['loadHash']!=new[k]['loadHash'] for k in old):
        raise SystemExit('Cached sampler / parallel geometry differs from pre-change stock or per-pass loads')
    accept['oldSamplerStockAndLoadsIdentical']=True
accept['selectedToleranceMm']=selected
pathlib.Path(sys.argv[2]).write_text(json.dumps({'acceptance':accept,'trials':rows},indent=2)+'\n')
print(selected)
PY
)
export HOBBER_FORCE_TOLERANCE_MM
[[ "$case_name" == run ]] || exit 0
export HOBBER_FORCE_THREADS=${HOBBER_FORCE_THREADS:-4}
export HOBBER_FORCE_HISTORY="$evidence/passes.csv"
cut_status=0
run_capped full /usr/bin/time -v -o "$evidence/time.log" \
  ./haxeon/.tools/hashlink/hl "$evidence/force-main.hl" reference | tee "$evidence/force.log" || cut_status=$?
sed -n 's/^REFERENCE_FORCE //p' "$evidence/force.log" > "$evidence/loads.json"
[[ -s "$evidence/loads.json" ]] || exit 1
python3 machinekit/examples/flexspline-hobber/force-band-study.py "$evidence/loads.json" --output "$evidence/bands.json" | tee "$evidence/bands.log"
[[ "$cut_status" == 0 ]] || exit "$cut_status"
python3 machinekit/examples/flexspline-hobber/finish-force-validation.py --fast-directory "$evidence"
