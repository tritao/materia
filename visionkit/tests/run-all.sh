#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
visionkit_build=${VISIONKIT_TEST_BUILD_DIR:-"$repo_dir/visionkit/tests/build-native"}
if [[ ! -f "$repo_dir/animkit/native/vendor/stb/stb_image.h" ]]; then
  printf 'stb submodule is required for the VisionKit CLI test\n' >&2
  exit 2
fi
cmake -S "$repo_dir/visionkit/native" -B "$visionkit_build" \
  -DVK_BUILD_TESTS=ON -DVK_BUILD_CALIBRATE=ON
cmake --build "$visionkit_build" -j "${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
ctest --test-dir "$visionkit_build" --output-on-failure
export LD_LIBRARY_PATH="$visionkit_build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$repo_dir/haxeon/scripts/haxeon" run --project "$repo_dir/visionkit/tests/haxeon.json"
"$repo_dir/haxeon/scripts/haxeon" run --project "$repo_dir/visionkit/tests/haxeon-robotkit.json"
