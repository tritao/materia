#!/usr/bin/env bash
# Full ideal reference stroke, followed by installed static continuous-rating checks.
set -euo pipefail
root=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$root"
evidence="$root/build/flexspline-reference-force"
mkdir -p "$evidence"
cache=${HOBBER_NATIVE_CACHE:-"$root/../machine-tending/app/build/host/native"}
stock=${HOBBER_STOCK_NATIVE:-"$root/build/flexspline-hb3/native"}
export HAXEON_COMPILER_SERVER=0
export CADKIT_OCCT_DIR=${CADKIT_OCCT_DIR:-"$HOME/dev/materia-cache/occt-f1dc4efb-release"}
export LD_LIBRARY_PATH="$stock:$CADKIT_OCCT_DIR/lib:$root/haxeon/.tools/hashlink:$root/haxeon/out"
for provider in "$cache"/*; do [[ ! -d "$provider" ]] || LD_LIBRARY_PATH+=":$provider"; done
export LD_LIBRARY_PATH
ulimit -s 65536
for manifest in stockkit/tests/hobbing-forces/haxeon.json machinekit/tests/hobber/haxeon.json; do
  ./haxeon/scripts/haxeon build --compiler-only --project="$manifest" >> "$evidence/build.log" 2>&1
done
{
  git rev-parse HEAD
  sha256sum stockkit/tests/hobbing-forces/build/host/main.hl machinekit/tests/hobber/build/host/main.hl "$stock/libstockkit_core.so"
} > "$evidence/provenance.txt"
cut_status=0
systemd-run --user --scope --quiet --collect -p MemoryMax=4G -p MemorySwapMax=0 \
  --unit="hobber-reference-force-$$" -- /usr/bin/time -v -o "$evidence/time.log" \
  ./haxeon/.tools/hashlink/hl stockkit/tests/hobbing-forces/build/host/main.hl reference | tee "$evidence/force.log" || cut_status=$?
sed -n 's/^REFERENCE_FORCE //p' "$evidence/force.log" > "$evidence/loads.json"
[[ -s "$evidence/loads.json" ]] || exit 1
ratings_status=0
./haxeon/.tools/hashlink/hl machinekit/tests/hobber/build/host/main.hl reference-loads "$evidence/loads.json" | tee "$evidence/ratings.log" || ratings_status=$?
[[ "$cut_status" == 0 && "$ratings_status" == 0 ]]
