#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$task_root"
task_libraries="$task_root/haxeon/.tools/hashlink:$task_root/haxeon/out"
for task_native in "$task_root/machinekit/examples/reference-arms/build/host/native/"*; do
  [[ -d "$task_native" ]] && task_libraries="$task_libraries:$task_native"
done
export LD_LIBRARY_PATH="$task_libraries${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$task_root/haxeon/.tools/hashlink/hl" machinekit/examples/reference-arms/build/host/main.hl "$@"
