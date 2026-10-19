#!/usr/bin/env bash
# CAD tool checks only; does not run a reference cut or rebuild native providers.
set -euo pipefail
root=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$root"
mode=${1:-quick}
case "$mode" in quick|reference) ;; *) echo "Usage: $0 [quick|reference]" >&2; exit 2;; esac
hob_evidence=${HOB_CHECK_EVIDENCE:-"$root/build/flexspline-hob-detail"}
cache=${HOBBER_NATIVE_CACHE:-"$root/../machine-tending/app/build/host/native"}
export CADKIT_OCCT_DIR=${CADKIT_OCCT_DIR:-"$HOME/dev/materia-cache/occt-f1dc4efb-release"}
export LD_LIBRARY_PATH="$CADKIT_OCCT_DIR/lib:$root/haxeon/.tools/hashlink:$root/haxeon/out"
for provider in "$cache"/*; do [[ ! -d "$provider" ]] || LD_LIBRARY_PATH+=":$provider"; done
export HAXEON_COMPILER_SERVER=0
ulimit -s 65536
mkdir -p "$hob_evidence"
./haxeon/scripts/haxeon build --compiler-only --project=machinekit/tests/hobbing/haxeon.json > "$hob_evidence/build.log" 2>&1
./haxeon/.tools/hashlink/hl machinekit/tests/hobbing/build/host/main.hl > "$hob_evidence/parts.log" 2>&1
./haxeon/.tools/hashlink/hl machinekit/tests/hobbing/build/host/main.hl hob-mesh > "$hob_evidence/mesh.log" 2>&1
args=(hob-detail short keyed two-start rake)
[[ "$mode" != reference ]] || args=(hob-detail reference)
./haxeon/.tools/hashlink/hl machinekit/tests/hobbing/build/host/main.hl "${args[@]}" > "$hob_evidence/$mode.log" 2>&1
cat "$hob_evidence/parts.log"
cat "$hob_evidence/mesh.log"
cat "$hob_evidence/$mode.log"
