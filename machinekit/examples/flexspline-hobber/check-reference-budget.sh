#!/usr/bin/env bash
# Fine physical HB11 source studies. Reuse native providers; never touch HB4.
set -euo pipefail
root=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$root"
mode=${1:-all}
evidence=${HOBBER_BUDGET_EVIDENCE:-"$root/build/flexspline-reference-budget-fast"}
force=${HOBBER_FORCE_EVIDENCE:-"$root/build/flexspline-reference-force-fast"}
stock=${HOBBER_STOCK_NATIVE:-"$evidence/native"}
cache=${HOBBER_NATIVE_CACHE:-"$root/../machine-tending/app/build/host/native"}
mkdir -p "$evidence"
exec 9>"$evidence/run.lock"
flock -n 9 || { echo 'Reference budget is already running in this evidence directory.' >&2; exit 2; }
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
