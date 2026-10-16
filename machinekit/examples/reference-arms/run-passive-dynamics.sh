#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$task_root"
# SimulationHarness needs the scene/physics libraries from an existing app build.
task_physics_paths=""
for task_dir in "$task_root"/app/build/host/native/*/; do
  [[ -d "$task_dir" ]] && task_physics_paths="$task_physics_paths:${task_dir%/}"
done
export LD_LIBRARY_PATH="${task_physics_paths#:}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
mkdir -p machinekit/examples/reference-arms/build/passive-dynamics
bash machinekit/examples/reference-arms/run-built.sh --passive-dynamics 2>&1 |
  tee machinekit/examples/reference-arms/build/passive-dynamics/runtime.log
