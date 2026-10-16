#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$task_root"
export LD_LIBRARY_PATH="$task_root/haxeon/.tools/hashlink:$task_root/haxeon/out:$task_root/machinekit/examples/zeroerr-arm/build/host/native/cadkit:$task_root/machinekit/examples/zeroerr-arm/build/host/native/collisionkit-native${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$task_root/haxeon/.tools/hashlink/hl" machinekit/examples/zeroerr-arm/build/host/main.hl "$@"
