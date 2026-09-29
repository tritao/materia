#!/usr/bin/env bash
# Fetches the Unitree G1 description from MuJoCo Menagerie at a pinned commit
# and imports its MJX variant, the one MuJoCo Playground trains and transfers
# to hardware, into a RobotModel. It imports scene_mjx.xml rather than
# g1_mjx.xml: the robot's geoms collide only through the foot-floor contact
# pairs that the scene declares.
#
# The model is not vendored: it stays under the licence in its LICENSE file
# (BSD-3-Clause, Unitree Robotics). Output goes to the directory given as the
# first argument, default robotkit/build/humanoid/g1.
#
# Usage: fetch-g1.sh [output-directory] [path-to-robotkit_mjcf_import]
set -euo pipefail

menagerie_commit=c96a32d28fb5da84da38c1da4d749e7a13212855
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
robotkit_dir=$(cd "$script_dir/../.." && pwd)
output=${1:-"$robotkit_dir/build/humanoid/g1"}
importer=${2:-robotkit_mjcf_import}
source_dir="$output/menagerie"

if [[ ! -d "$source_dir/.git" ]]; then
  rm -rf "$source_dir"
  git clone --quiet --filter=blob:none --no-checkout \
    https://github.com/google-deepmind/mujoco_menagerie "$source_dir"
  git -C "$source_dir" sparse-checkout set unitree_g1
fi
git -C "$source_dir" -c advice.detachedHead=false checkout --quiet "$menagerie_commit"

"$importer" "$source_dir/unitree_g1/scene_mjx.xml" "$output/model"
echo "G1 RobotModel: $output/model/robot.json, keyframes: $output/model/poses.json"
echo "Licence: $source_dir/unitree_g1/LICENSE"
