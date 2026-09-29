#!/usr/bin/env bash
# Fetches Unitree's own G1 description (12 leg joints, upper body fixed) from
# unitree_rl_gym at a pinned commit and imports its MuJoCo scene into a
# RobotModel. This is the model the pretrained G1 walking policy in
# policies/unitree-g1 was trained on and that Unitree's sim-to-sim script
# (deploy/deploy_mujoco) runs it in, so it is the fair model to judge the
# policy on. The larger Menagerie G1 (fetch-g1.sh) has 29 joints and other
# inertias.
#
# Nothing is vendored: the description stays under the BSD-3-Clause licence in
# the repository's LICENSE. Output goes to the directory given as the first
# argument, default robotkit/build/humanoid/unitree-g1.
#
# Usage: fetch-unitree-g1.sh [output-directory] [path-to-robotkit_mjcf_import]
set -euo pipefail

commit=276801e46c5d433564f24658bac64f254b7d2d4b
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
robotkit_dir=$(cd "$script_dir/../.." && pwd)
output=${1:-"$robotkit_dir/build/humanoid/unitree-g1"}
importer=${2:-robotkit_mjcf_import}
source_dir="$output/unitree_rl_gym"

if [[ ! -d "$source_dir/.git" ]]; then
  rm -rf "$source_dir"
  git clone --quiet --filter=blob:none --no-checkout \
    https://github.com/unitreerobotics/unitree_rl_gym "$source_dir"
  git -C "$source_dir" sparse-checkout set resources/robots/g1_description deploy/deploy_mujoco
fi
git -C "$source_dir" -c advice.detachedHead=false checkout --quiet "$commit"

"$importer" "$source_dir/resources/robots/g1_description/scene.xml" "$output/model"
echo "G1 (12 DoF) RobotModel: $output/model/robot.json"
echo "Licence: $source_dir/LICENSE"
