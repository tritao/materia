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

## Layers

- `CollisionWorld`: the interface queries run against. Bodies, shapes, rules and margins are set one call at a time.
- `CollisionDescription`: the same content as plain data (`DescribedBody`, `DescribedObject`, `DescribedRule`,
  `DescribedArticulation`), plus margin classes, held-contact policy for grasped parts, and the assumptions the geometry
  makes. Geometry sources add to it; `append` and `copy` combine and duplicate descriptions.
- `CollisionDescription.build(world)` puts a description into a world and returns a `CollisionBuild`: the world's ids for
  every described body and object, the layout errors found at the references, and the operations that change a live
  build (`transferBody` for a grasp or placement, contact windows). The build keeps its own copy of the description.
- `collisionkit.kinematics.ModelBodies` binds a KinematicsKit model to a description and poses its bodies from a
  kinematic state.
- `CollisionBounds` gives a shape's bounds after a pose and its reach from its own origin; `CollisionPose` converts to
  and from KinematicsKit's `Transform`.

Rigid, adjacent and closure pairs of a model come from KinematicsKit
(`kinematicskit.KinematicBodyPairs`); the caller turns them into rules.

The plan and its progress log are in `plans/COLLISION.md`.

Run the tests:

```
cmake -S native -B <build> -G Ninja && cmake --build <build> && ctest --test-dir <build>
../haxeon/scripts/haxeon run --project native/tests/haxeon.json
```
