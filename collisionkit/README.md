# CollisionKit

A collision world for Materia cells: bodies the caller registers and poses,
shapes attached to them (primitives, convex sets, triangle meshes, height
fields), rules on which pairs are checked and why, and queries for
distances, violations of per-class margins (with inflation for motion
bounds), and batched checks over many pose sets. The world knows bodies,
not models: one world holds several robots, mechanisms and the environment.

- `haxe/`: the pure half, `collisionkit.CollisionWorld` and its types.
  Callers (MotionKit validation, path search, planners) use only these.
- `native/`: `collisionkit.native.NativeCollisionWorld` on coal, through the
  C ABI in `native/include/collisionkit.h`. coal is vendored in
  `native/vendor/coal` (see `native/THIRD_PARTY.md`).

Rigid, adjacent and closure pairs of a model come from KinematicsKit
(`kinematicskit.KinematicBodyPairs`); the caller turns them into rules.

The plan and its progress log are in `plans/COLLISION.md`.

Run the tests:

```
cmake -S native -B <build> -G Ninja && cmake --build <build> && ctest --test-dir <build>
../haxeon/scripts/haxeon run --project native/tests/haxeon.json
```
