#!/usr/bin/env bash
# Diagnostic boundary refinement study; no full-circle/reference acceptance claim.
set -euo pipefail
root=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$root"
evidence=${HB4_BOUNDARY_EVIDENCE:-"$root/build/flexspline-hb4-boundary-check"}
native=${HB4_PROBE_NATIVE:-"$root/build/flexspline-shared-sweep/native-pair"}
input=${HB4_BOUNDARY_INPUT:-"$root/stockkit/tests/metrology/fixtures/hb4-boundary.json"}
[[ -f "$native/libstockkit_core.so" ]] || { echo 'Set HB4_PROBE_NATIVE to an existing StockKit native provider.' >&2; exit 2; }
mkdir -p "$evidence"
exec 9>"$evidence/run.lock"
flock -n 9 || { echo 'Boundary diagnostic already running in this evidence directory.' >&2; exit 2; }
HAXEON_COMPILER_SERVER=0 ./haxeon/scripts/haxeon build --compiler-only --project=stockkit/tests/metrology/haxeon.json > "$evidence/build.log" 2>&1
cp stockkit/tests/metrology/build/host/main.hl "$evidence/main.hl"
export HB4_BOUNDARY_EVIDENCE="$evidence" HB4_BOUNDARY_INPUT="$input"
export LD_LIBRARY_PATH="$native:$root/haxeon/.tools/hashlink:$root/haxeon/out${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
python3 - "$evidence" "$native" "$input" <<'PYMETA'
import hashlib,json,subprocess,sys
from pathlib import Path
out,native,fixture=map(Path,sys.argv[1:])
files=[out/'main.hl',native/'libstockkit_core.so',fixture]
(out/'execution.json').write_text(json.dumps({'sourceCommit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
 'threads':4,'sweepToleranceMm':0.00005,'refinements':list(range(6)),'fullReferenceAcceptance':False,
 'sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}},indent=2)+'\n')
for name in ['result.json','levels.json','exit-status.txt']:
 (out/name).unlink(missing_ok=True)
PYMETA
ulimit -s 65536
status=0
/usr/bin/time -v -o "$evidence/time.log" haxeon/.tools/hashlink/hl "$evidence/main.hl" boundary-probe > "$evidence/run.log" 2>&1 || status=$?
printf '%s\n' "$status" > "$evidence/exit-status.txt"
cat "$evidence/run.log"
exit "$status"
