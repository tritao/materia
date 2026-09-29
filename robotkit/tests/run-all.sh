#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
robotkit_dir="$repo_dir/robotkit"
haxeon="$repo_dir/haxeon/scripts/haxeon"
cadkit_build="${CADKIT_BUILD_ROOT:-$repo_dir/cadkit/build/debug}"
cadkit_library="$cadkit_build/core/libcadkit-core.so"
mujoco_source="$repo_dir/simkit/vendor/mujoco"

say() {
  printf '\n== %s ==\n' "$1"
}

run() {
  printf '+ '
  printf '%q ' "$@"
  printf '\n'
  "$@"
}

build_cadkit_core() {
  say "CadKit core"
  if [[ ! -f "$cadkit_library" ]]; then
    run cmake -S "$repo_dir/cadkit" -B "$cadkit_build" -GNinja \
      -DCMAKE_BUILD_TYPE=Debug -DCADKIT_BUILD_TESTS=OFF
    run cmake --build "$cadkit_build" --target cadkit-core
  else
    printf 'Using existing %s\n' "$cadkit_library"
  fi
}

ensure_mujoco() {
  if [[ -f "$mujoco_source/CMakeLists.txt" ]]; then
    return 0
  fi

  say "MuJoCo prerequisite"
  if ! git -C "$repo_dir" submodule update --init simkit/vendor/mujoco; then
    printf 'MuJoCo is unavailable; its project will be skipped.\n' >&2
    return 1
  fi
  [[ -f "$mujoco_source/CMakeLists.txt" ]]
}

say "Haxe world tests"
run python3 "$robotkit_dir/tools/recording/generate_schemas.py" check \
  --lock "$robotkit_dir/schema/rkf1.lock.json" \
  --output "$robotkit_dir/haxe/robotkit/world/RecordingSchemas.hx"
run "$haxeon" run --project "$robotkit_dir/tests/haxeon.json"

if [[ "${ROBOTKIT_TEST_SENSORKIT_SPIKE:-0}" == "1" ]]; then
  say "SensorKit camera spike"
  run "$haxeon" run --project "$robotkit_dir/tests/spike/haxeon.json"
fi

say "Device protocol host and MCU tests"
run cargo test --manifest-path "$robotkit_dir/device_protocol/Cargo.toml" --features std
run cargo test --manifest-path "$robotkit_dir/device_virtual/Cargo.toml"
run "$robotkit_dir/device_protocol/tools/check-mcu-build.sh"

say "RobotKit native tests"
run cmake -S "$robotkit_dir" -B "$robotkit_dir/build" -GNinja \
  -DCMAKE_BUILD_TYPE=Debug
run cmake --build "$robotkit_dir/build" -j "${CMAKE_BUILD_PARALLEL_LEVEL:-6}"
run ctest --test-dir "$robotkit_dir/build" --output-on-failure

build_cadkit_core

say "CAD bridge tests"
cadkit_ld="$(dirname "$cadkit_library")"
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  export LD_LIBRARY_PATH="$cadkit_ld:$LD_LIBRARY_PATH"
else
  export LD_LIBRARY_PATH="$cadkit_ld"
fi
run "$haxeon" run --project "$robotkit_dir/cadbridge/tests/haxeon.json"

if ensure_mujoco; then
  say "MuJoCo wall-finishing scenario"
  run "$haxeon" run --project "$robotkit_dir/tests/mujoco/haxeon.json"
  say "MuJoCo end-effector proximity"
  run "$haxeon" run --project "$robotkit_dir/cadbridge/tests/mujoco/haxeon.json"
else
  printf '\n== MuJoCo wall-finishing scenario ==\n'
  printf 'SKIP: simkit/vendor/mujoco is unavailable.\n'
fi

say "World TCP integration"
run python3 -m tools.wire.rkf1 check --source "$robotkit_dir/haxe/robotkit/protocol" --lock "$robotkit_dir/schema/rkf1.lock.json"
run python3 -m unittest tools.wire.tests.test_rkf1
run "$haxeon" run --project "$robotkit_dir/tests/integration/haxeon.json" -- --outbound-scheduler
run "$robotkit_dir/tests/world-tcp.sh"
run env ROBOTKIT_TEST_SUBSCRIPTIONS=1 "$robotkit_dir/tests/world-tcp.sh"
run env ROBOTKIT_TEST_SESSIONS=1 "$robotkit_dir/tests/world-tcp.sh"
run env ROBOTKIT_TEST_LEASE_TIMEOUT=1 "$robotkit_dir/tests/world-tcp.sh"
run env ROBOTKIT_TEST_BULK=1 "$robotkit_dir/tests/world-tcp.sh"
run env ROBOTKIT_TEST_PERCEPTION=1 "$robotkit_dir/tests/world-tcp.sh"
run env ROBOTKIT_TEST_PERCEPTION_STALL=1 "$robotkit_dir/tests/world-tcp.sh"
run env ROBOTKIT_TEST_MALFORMED_HELLO=1 "$robotkit_dir/tests/world-tcp.sh"

printf '\nAll available RobotKit suites passed.\n'
