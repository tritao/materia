#!/usr/bin/env bash
set -euo pipefail
router_repo=$(pwd -P)
router_output="$router_repo/build/sheet-router-work/sr7"
router_heavy=${MATERIA_HEAVY_RUN:-/home/joao/dev/materia-cache/tools/heavy-run}
router_sampler=${SHEET_ROUTER_PLANNER_SAMPLE:-"$router_output/planner-sample"}
mkdir -p "$router_output"
if [[ ! -x "$router_sampler" ]]; then
  [[ ! -e "$router_sampler" && ! -L "$router_sampler" ]] || { echo "Refusing to overwrite planner sampler" >&2; exit 2; }
  "$router_heavy" --mem 3G --need 1G --label router-atc-sampler python3 - "$router_repo/build/sheet-router-evidence/sr7-atc.tar.gz" "$router_output" <<'PYTOOL'
import os,sys,tarfile,pathlib
root=pathlib.Path(sys.argv[2]);(root/'native').mkdir(exist_ok=True)
with tarfile.open(sys.argv[1],'r:gz') as archive:
    for name in ('planner-sample','native/libmotionkit_core.so.0','native/libtrajectory_core.so.0'):
        entry=archive.getmember('sr7-atc/'+name)
        if not entry.isfile():raise ValueError('Planner dependency is not a regular file')
        target=root/name
        if target.exists() or target.is_symlink():raise ValueError('Refusing to overwrite planner dependency')
        with target.open('xb') as out:out.write(archive.extractfile(entry).read())
        os.chmod(target,0o755)
PYTOOL
fi
export HAXEON_COMPILER_SERVER=0
export SHEET_ROUTER_PLANNER_SAMPLE="$router_sampler"
"$router_heavy" --mem 6G --need 1G --label router-atc-build haxeon/scripts/haxeon build --compiler-only \
  --project machinekit/examples/sheet-router/haxeon.atc.json --output "$router_output/full.hl"
export LD_LIBRARY_PATH="$router_repo/haxeon/out:$router_repo/haxeon/.tools/hashlink${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
for router_provider in "$router_repo/machinekit/tests/build/host/native"/*; do
  [[ ! -d "$router_provider" ]] || LD_LIBRARY_PATH+=":$router_provider"
done
"$router_heavy" --mem 3G --need 1G --label router-atc-policy haxeon/.tools/hashlink/hl "$router_output/full.hl" --policy-only
"$router_heavy" --mem 5G --need 1G --label router-atc-clearance haxeon/.tools/hashlink/hl "$router_output/full.hl" --sequence-only
