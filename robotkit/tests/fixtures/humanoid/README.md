# Humanoid fixtures

`unitree-g1-12dof.robot.json` is `robotkit_mjcf_import` of Unitree's G1 scene
(`resources/robots/g1_description/scene.xml`, 12 leg joints, upper body fixed)
from unitree_rl_gym commit 276801e, copyright Unitree Robotics, BSD-3-Clause;
the licence is `tools/humanoid/policies/unitree-g1/LICENSE`. Only the imported
model is here, without the STL visuals: the simulation uses its collision
primitives. `tools/humanoid/fetch-unitree-g1.sh` regenerates it, with meshes.
It is the model the pretrained G1 walking policy was trained and is
demonstrated on.
