#!/usr/bin/env bash
# Sim-to-sim conformance gate (robotkit/plans/HUMANOID.md, HU-D5): the same
# MJCF file and control sequence in plain MuJoCo and, imported, in RobotKit's
# Simulation must give the same trajectory.
#
# 1. The checked-in biped fixture, driven by joint torques for 1.5 s through
#    landing, standing and toppling.
# 2. With a G1 directory from fetch-g1.sh, Unitree G1's MJX scene started in
#    its "home" keyframe with its position servos held still and swinging
#    +-0.1 rad, for 2 s: servos alone cannot balance it, so this covers
#    standing, toppling and hitting the ground. Both sides solve to
#    convergence (MuJoCo's default 100 and 50 iterations). The scene's own 5
#    and 8 leave solves whose result depends on constraint order, which
#    differs between the two compiled models; they agree to about 1e-4 rad.
#
# Usage: check-conformance.sh <native-build-dir> [g1-directory]
# The native build is robotd/native-mujoco's, holding robotkit_mjcf_import
# and robotkit_mjcf_reference.
set -euo pipefail

build=${1:?usage: check-conformance.sh <native-build-dir> [g1-directory]}
g1=${2:-}
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
robotkit_dir=$(cd "$script_dir/../.." && pwd)
repo_dir=$(dirname "$robotkit_dir")
tools=$(dirname "$(find "$build" -name robotkit_mjcf_reference -type f | head -1)")
work=$(mktemp -d "${TMPDIR:-/tmp}/robotkit-conformance.XXXXXX")
trap 'rm -rf "$work"' EXIT

conformance() {
  "$repo_dir/haxeon/scripts/haxeon" run --project "$script_dir/haxeon.json" -- conformance "$@" \
    | grep -E '^t=|worst|PASS|FAIL'
}

fixture="$robotkit_dir/tests/fixtures/mjcf/conformance.xml"
"$tools/robotkit_mjcf_import" "$fixture" "$work/biped" > /dev/null
"$tools/robotkit_mjcf_reference" "$fixture" 0.01 150 6 "$work/biped.csv"
conformance "$work/biped/robot.json" "$work/biped.csv" 0.01 5 6 2 0 0 1e-6 1e-6

if [[ -n "$g1" ]]; then
  scene="$g1/menagerie/unitree_g1/scene_mjx.xml"
  "$tools/robotkit_mjcf_import" "$scene" "$work/g1" > /dev/null
  for amplitude in 0 0.1; do
    "$tools/robotkit_mjcf_reference" "$scene" 0.02 100 "$amplitude" "$work/g1-$amplitude.csv" \
      home 100 50
    conformance "$work/g1/robot.json" "$work/g1-$amplitude.csv" 0.02 5 "$amplitude" 2 100 50 \
      1e-5 1e-5 "$work/g1/poses.json" home
  done
fi
