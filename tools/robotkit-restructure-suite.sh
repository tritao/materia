#!/usr/bin/env bash
# Phase gate for robotkit/RESTRUCTURE_PLAN.md. Run from any directory.
# Usage: robotkit-restructure-suite.sh BUILD_ROOT [all|workspace|compile|robotkit|consumers|native]
# Every phase requires all stages; separate calls let interrupted gates resume.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
build_root=${1:-/tmp/robotkit-restructure-checks}
stage=${2:-all}
case "$stage" in all|workspace|compile|robotkit|consumers|native) ;;
    *) printf 'Unknown gate stage: %s\n' "$stage" >&2; exit 2 ;;
esac
: "${CADKIT_OCCT_DIR:?Set CADKIT_OCCT_DIR to an installed copy of the pinned OCCT build}"
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-4}
mkdir -p "$build_root"
build_root=$(cd "$build_root" && pwd)
cd "$root"

# CAD is needed by consumers in the workspace, the worker demo and the welder.
cmake -S cadkit -B "$build_root/cad" -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DCADKIT_OCCT_DIR="$CADKIT_OCCT_DIR" -DCADKIT_BUILD_TESTS=ON
cmake --build "$build_root/cad"
export LD_LIBRARY_PATH="$build_root/cad/core:$CADKIT_OCCT_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

if [[ "$stage" == all || "$stage" == workspace ]]; then
python3 tools/check-robotkit-packages.py
trajectorykit/native/tools/check-hxi.sh "$build_root/trajectory-core.hxi"
cmake -E compare_files "$build_root/trajectory-core.hxi" trajectorykit/native/bindings/trajectory-core.hxi
motionkit/native/tools/check-hxi.sh "$build_root/motionkit.hxi"
cmake -E compare_files "$build_root/motionkit.hxi" motionkit/native/bindings/motionkit.hxi

# This is the maintained successor to the untracked x7-suite.sh.
haxeon/scripts/haxeon workspace test --workspace materia.workspace.json \
    --jobs 4 --compilers 2 --no-test-cache --skip-tag peer
fi

if [[ "$stage" == all || "$stage" == compile ]]; then
python3 tools/check-compile-all.py --jobs 2 --scratch "$build_root/compile" \
    --json "$build_root/compile-results.json"
fi

if [[ "$stage" == all || "$stage" == robotkit ]]; then
# The TCP harness owns the peers needed by the workspace's peer-tagged project.
CADKIT_BUILD_ROOT="$build_root/cad" ROBOTKIT_BUILD_ROOT="$build_root/native" robotkit/tests/run-all.sh
fi

if [[ "$stage" == all || "$stage" == consumers ]]; then
haxeon/scripts/haxeon run --project robotkit/robotd/haxeon.json -- --in-memory
haxeon/scripts/haxeon run --project robotkit/tools/humanoid/tests/haxeon.json
haxeon/scripts/haxeon run --project machinekit/examples/robot-welder/haxeon.json
# VisionKit's RobotKit perception tests are in no workspace project.
haxeon/scripts/haxeon run --project visionkit/tests/haxeon-robotkit.json
haxeon/scripts/haxeon build --project app/haxeon.json
app/run-built.sh --snapshot --worker-demo=rack-to-table --worker-demo-step=1000 \
    > "$build_root/worker-demo.log"
python3 - "$build_root/worker-demo.log" <<'PY'
import json, sys
records = [json.loads(line) for line in open(sys.argv[1]) if line.startswith('{')]
worker = next(record for record in records if record.get('workerDemo') == 'rack-to-table')
assert worker['jobDone'] and worker['jobFailure'] is None, worker
print('worker demo passed')
PY
fi

if [[ "$stage" == all || "$stage" == native ]]; then
cmake -S motionkit/native -B "$build_root/motion-release" -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_root/motion-release"
ctest --test-dir "$build_root/motion-release" --output-on-failure -j 1
cmake --build "$build_root/motion-release" --target motionkit_runtime_fixtures
"$build_root/motion-release/motionkit_runtime_fixtures" > "$build_root/runtime-fixtures.hpp"
cmake -E compare_files "$build_root/runtime-fixtures.hpp" robotkit/runtime/tests/runtime_trajectory_fixtures.hpp
ctest --test-dir "$build_root/cad" --output-on-failure -j 1
fi
