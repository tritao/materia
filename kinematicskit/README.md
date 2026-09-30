# KinematicsKit

One kinematics core for Materia: a compiled kinematic forest, forward
kinematics and Jacobians, and (from K1) tasks, limits and solvers. It has no
dependencies. Authored models compile into it in their own kits: CadKit
assemblies (`cadkit.modeling.AssemblyKinematics`) and RobotKit robots
(`robotkit.kinematics.RobotKinematics`).

The plan and its progress log are in `plans/KINEMATICS.md`.

Run the tests:

```
../haxeon/scripts/haxeon run --project tests/haxeon.json
```
