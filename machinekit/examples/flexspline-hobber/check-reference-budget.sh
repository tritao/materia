#!/usr/bin/env bash
# Fine physical HB11 source studies. Reuse native providers and completed cases.
set -euo pipefail
root=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$root"
mode=${1:-all}
default_evidence="$root/build/flexspline-reference-budget-fast"
[[ "$mode" != parallel ]] || default_evidence="$root/build/flexspline-reference-budget-parallel"
evidence=${HOBBER_BUDGET_EVIDENCE:-"$default_evidence"}
force=${HOBBER_FORCE_EVIDENCE:-"$root/build/flexspline-reference-force-fast"}
default_stock="$evidence/native"
[[ "$mode" != parallel ]] || default_stock="$root/build/flexspline-clipping-speed-v2/native"
if [[ "$mode" == parallel && -f "$evidence/native-upgrade-validation.json" ]]; then
  default_stock=$(python3 - "$evidence/native-upgrade-validation.json" <<'PYNATIVE'
import json, sys
print(json.load(open(sys.argv[1]))['newNativeDirectory'])
PYNATIVE
  )
fi
stock=${HOBBER_STOCK_NATIVE:-"$default_stock"}
cache=${HOBBER_NATIVE_CACHE:-"$root/../machine-tending/app/build/host/native"}
mkdir -p "$evidence"
exec 9>"$evidence/run.lock"
flock -n 9 || { echo 'Reference budget is already running in this evidence directory.' >&2; exit 2; }
if [[ "$mode" != parallel && -z "${HOBBER_STOCK_NATIVE:-}" ]]; then
  cmake -S stockkit/core -B "$stock" -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON > "$evidence/native-build.log" 2>&1
  cmake --build "$stock" --target stockkit_core -j2 >> "$evidence/native-build.log" 2>&1
fi
export HAXEON_COMPILER_SERVER=0
export CADKIT_OCCT_DIR=${CADKIT_OCCT_DIR:-"$HOME/dev/materia-cache/occt-f1dc4efb-release"}
export LD_LIBRARY_PATH="$stock:$CADKIT_OCCT_DIR/lib:$root/haxeon/.tools/hashlink:$root/haxeon/out"
for provider in "$cache"/*; do [[ ! -d "$provider" ]] || LD_LIBRARY_PATH+=":$provider"; done
export LD_LIBRARY_PATH
ulimit -s 65536
if [[ "$mode" == parallel ]]; then
  legacy=${HOBBER_BUDGET_LEGACY:-"$root/build/flexspline-reference-budget-fast"}
  face=${HOBBER_BUDGET_FACE:-"$root/build/flexspline-face-end-check"}
  threads=${HOBBER_BUDGET_THREADS:-4}
  workers=${HOBBER_BUDGET_WORKERS:-4}
  if [[ ! -f "$evidence/main.hl" ]]; then
    [[ -f "$stock/libstockkit_core.so" && -f "$legacy/inputs.json" ]] || { echo 'Validated native library and legacy inputs required.' >&2; exit 2; }
    ./haxeon/scripts/haxeon build --compiler-only --project=machinekit/tests/hobber/haxeon.json > "$evidence/build.log" 2>&1
    cp machinekit/tests/hobber/build/host/main.hl "$evidence/main.hl"
    ln "$legacy/inputs.json" "$evidence/inputs.json" || cp "$legacy/inputs.json" "$evidence/inputs.json"
    python3 - "$legacy" "$evidence" "$threads" "$workers" <<'PYSETTINGS'
import json, sys
from pathlib import Path
legacy, output = map(Path, sys.argv[1:3])
s = json.loads((legacy / 'settings.json').read_text())
s.update(threads=int(sys.argv[3]), workers=int(sys.argv[4]), faceEndpoints=True)
(output / 'settings.json').write_text(json.dumps(s, indent=2) + '\n')
PYSETTINGS
    { git rev-parse HEAD; sha256sum "$evidence/main.hl" "$evidence/inputs.json" "$stock/libstockkit_core.so"; } > "$evidence/provenance.txt"
  fi
  legacy_pid=()
  [[ -z "${HOBBER_BUDGET_LEGACY_PID:-}" ]] || legacy_pid=(--legacy-pid "$HOBBER_BUDGET_LEGACY_PID")
  exec systemd-run --user --scope --quiet --collect -p MemoryMax=6G -p MemorySwapMax=0 \
    --unit="flexspline-hb11-parallel-$$" python3 machinekit/examples/flexspline-hobber/run-reference-budget.py "$evidence" \
    --legacy-directory "$legacy" --face-directory "$face" --native "$stock" \
    --workers "$workers" --threads "$threads" "${legacy_pid[@]}"
fi
c++ -std=c++20 -O3 -Irobotkit/runtime/include -Irobotkit/runtime/generated \
  machinekit/tests/hobber/native/reference-sync.cpp -o "$evidence/sync-exporter"
python3 machinekit/examples/flexspline-hobber/prepare-reference-budget.py \
  --force-directory "$force" --output-directory "$evidence" --sync-exporter "$evidence/sync-exporter" > "$evidence/prepare.log"
./haxeon/scripts/haxeon build --compiler-only --project=machinekit/tests/hobber/haxeon.json > "$evidence/build.log" 2>&1
cp machinekit/tests/hobber/build/host/main.hl "$evidence/main.hl"
{
  git rev-parse HEAD
  git diff --binary HEAD | sha256sum
  sha256sum "$evidence/main.hl" "$evidence/sync-exporter" "$evidence/inputs.json" "$stock/libstockkit_core.so"
} > "$evidence/provenance.txt"
rm -f "$evidence/$mode-exit-status.txt"
status=0
systemd-run --user --scope --quiet --collect -p MemoryMax=6G -p MemorySwapMax=0 \
  --unit="flexspline-hb11-$mode-$$" /usr/bin/time -v -o "$evidence/$mode-time.log" \
  ./haxeon/.tools/hashlink/hl "$evidence/main.hl" reference-budget "$evidence/inputs.json" "$evidence" "$mode" \
  > "$evidence/$mode.log" 2>&1 || status=$?
printf '%s\n' "$status" > "$evidence/$mode-exit-status.txt"
tail -n 8 "$evidence/$mode.log"
exit "$status"
