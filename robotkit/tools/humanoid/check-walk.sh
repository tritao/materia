#!/usr/bin/env bash
# H4 acceptance gate (robotkit/plans/HUMANOID.md): Unitree's pretrained G1
# walking policy through the RobotKit runtime in MuJoCo. Runs the humanoid
# test project (stand, 1 m/s for 30 s, turns, pushes, MCAP record and replay)
# on the checked-in 12-joint G1 fixture. With a directory from fetch-g1.sh it
# also runs the same policy on the 29-joint Menagerie G1, which has other
# inertias, an IMU of its own and arms and a waist the policy does not know.
#
# Usage: check-walk.sh [g1-directory]
set -euo pipefail

g1=${1:+$(cd "$1" && pwd)}
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd "$script_dir/../../.." && pwd)
haxeon="$repo_dir/haxeon/scripts/haxeon"

"$haxeon" run --project "$script_dir/tests/haxeon.json"

if [[ -n "$g1" ]]; then
  spec="$script_dir/policies/unitree-g1/policy-menagerie.json"
  model="$g1/model/robot.json"
  for scenario in "10 0 0 0" "10 0.5 0 0" "30 1.0 0 0"; do
    echo "Menagerie G1: seconds vx vy wz = $scenario"
    # shellcheck disable=SC2086
    "$haxeon" run --project "$script_dir/haxeon.json" -- walk "$model" "$spec" $scenario | grep -E '^ran|PASS|FAIL'
  done
fi
