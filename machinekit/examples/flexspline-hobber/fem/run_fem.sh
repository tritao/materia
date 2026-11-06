#!/bin/bash
# Re-run the hobber FE checks end to end (about 40 minutes on a loaded machine; use the repository's heavy-run wrapper when it is shared) and refresh
# results/hobber-fem-results.json. Then paste the cradle and saddle numbers into machinekit/haxe/src/machinekit/hobbing/HobberFeResults.hx (the routine
# check `HobberFrameChecks.spindleDrive` throws when the cradle geometry no longer matches the signature stored there).
#   usage: run_fem.sh [work dir]      env: HEAVY="heavy-run --slots 4 --need 6G --label fem --" to queue the python steps
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../../../.." && pwd)
work=${1:-${TMPDIR:-/tmp}/hobber-fem}
mkdir -p "$work/export"
export TMPDIR="$work"
# 1. the model's STEP parts, holes, masses and the servo group (read-only on the model)
"$root/haxeon/scripts/haxeon" run --project "$here/haxeon.json" -- "$work/export"
cd "$here"
# 2. the cradle on the seat: the design with its rest block (final) and the joint / support / coupling brackets
${HEAVY:-} python3 cradle_fem.py "$work/export" results/cradle-final.json final
# 3. the X carriage and the Z saddle
${HEAVY:-} python3 carriage_fem.py "$work/export" results/carriage.json 14 10
${HEAVY:-} python3 saddle_fem.py "$work/export" results/saddle.json 14 10
# 4. the hand model against the variants (and its refit: add `fit`), then the report
python3 cradle_handmodel.py "$work/export"
python3 hobber_fem.py report > /dev/null
python3 -m unittest discover -s tests -p 'test_*.py'
