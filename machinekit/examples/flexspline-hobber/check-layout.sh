#!/usr/bin/env bash
# Exact layout and mount checks. Reuses native providers read-only; no full cutting run.
set -euo pipefail
root=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$root"
mode=${1:-all}
case "$mode" in all|initial) ;; *) echo "Usage: $0 [all|initial]" >&2; exit 2;; esac
layout_evidence=${HOBBER_LAYOUT_EVIDENCE:-"$root/build/flexspline-layout"}
cache=${HOBBER_NATIVE_CACHE:-"$root/../machine-tending/app/build/host/native"}
stock=${HOBBER_STOCK_NATIVE:-"$root/build/flexspline-shared-sweep/native-pair"}
export CADKIT_OCCT_DIR=${CADKIT_OCCT_DIR:-"$HOME/dev/materia-cache/occt-f1dc4efb-release"}
export LD_LIBRARY_PATH="$stock:$CADKIT_OCCT_DIR/lib:$root/haxeon/.tools/hashlink:$root/haxeon/out"
for provider in "$cache"/*; do [[ ! -d "$provider" ]] || LD_LIBRARY_PATH+=":$provider"; done
export HAXEON_COMPILER_SERVER=0
ulimit -s 65536
mkdir -p "$layout_evidence"
./haxeon/scripts/haxeon build --compiler-only --project=machinekit/tests/hobber/haxeon.json \
  --output="$layout_evidence/main.hl" > "$layout_evidence/build.log" 2>&1
{
  git rev-parse HEAD
  git diff --binary HEAD -- machinekit | sha256sum
  sha256sum "$layout_evidence/main.hl" machinekit/haxe/src/machinekit/hobbing/HobberSaddle.hx
} > "$layout_evidence/provenance.txt"
haxeon/.tools/hashlink/hl "$layout_evidence/main.hl" > "$layout_evidence/mechanical.log" 2>&1
cat "$layout_evidence/mechanical.log"
export HOBBER_INSTALLATION_EVIDENCE="$layout_evidence"
haxeon/.tools/hashlink/hl "$layout_evidence/main.hl" installation > "$layout_evidence/installation.log" 2>&1
cat "$layout_evidence/installation.log"
for configuration in coarse reference; do
  mkdir -p "$layout_evidence/$configuration"
  export HOBBER_INTERFERENCE_EVIDENCE="$layout_evidence/$configuration"
  args=(interferences)
  [[ "$configuration" != reference ]] || args+=(reference)
  [[ "$mode" != all ]] || args+=(sweep)
  haxeon/.tools/hashlink/hl "$layout_evidence/main.hl" "${args[@]}" > "$layout_evidence/$configuration/run.log" 2>&1
  tail -n 1 "$layout_evidence/$configuration/run.log"
done
