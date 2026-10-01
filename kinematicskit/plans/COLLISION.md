# Collision — checking, avoidance and free-space planning (plan)

Builds on KinematicsKit K6 (one group, one solver, one path search) and
KK-D20 (coal, vendored from `tritao/coal`, branch `materia`). Collision
checking had been planned as "a future plan" by MotionKit Lane D (D4) and
Lane C (OMPL listed as out of scope). This is that plan.

## Where things stand (2026-10-01)

- **coal** is a submodule at `kinematicskit/native/vendor/coal`. It builds as
  `coal::core` (Eigen only) and is exercised only by a standalone smoke test.
  The build has primitives, convex sets, BVH meshes, height fields and the
  broadphase managers. It has no octrees, no mesh-file loading and no qhull,
  so coal cannot compute convex hulls: hulls come as point sets from
  CadKit's pure-Haxe `ConvexHullVertices`.
- **RobotKit geometry:**
  - `CollisionShape` holds a box, sphere, capsule or cylinder with a pose in
    its link's frame. `Link.collisionGeometry` is only a mesh reference.
  - URDF import keeps primitives; a `<mesh>` stays a reference and warns.
    MJCF import keeps primitives too.
  - Links can also get convex hulls, passed beside the model
    (`Simulation.addRobotAtPose(linkCollisionHulls)`).
  - Tools have their own vocabulary, `ToolCollisionShape`: none, a box, a
    cylinder, or hull pieces with padding.
  - Self-collision exclusion exists only inside SimKit's MuJoCo backend
    (`add_self_collision_excludes`: parent-child pairs, and pairs that
    overlap at rest).
- **CAD geometry:**
  - `Shape.tessellate` gives float64 triangle meshes.
  - `ConvexHullVertices` gives hulls of at most 64 points.
  - `AssemblyState.worldPose` places parts, and `AssemblyPhysicalPart`
    carries a part's `collisionHull`.
  - Scene objects with `collisionEnabled` only become boxes.
- **Pure-Haxe checks today:**
  - `ToolClearanceChecker` (separating-axis test, convex tool against
    oriented boxes, used by `WorkPatchPlanner`);
  - `RobotLinkBounds` (bounding spheres, for the editor);
  - `HumanBodyProxy` (capsules).
- **Terrain:** `HeightMap` (a vertex grid lowered by digging) feeds no
  collision or physics backend.
- **Validation:**
  - `mk_validation_report` checks position, velocity, acceleration, jerk,
    continuity and task space; collision is not among them.
  - `ProgramCompiler.finish` runs `checkTaskSpace` after timing.

No single collision-world model exists. `ApplicationSimulation` assembling
a SimKit session is the closest thing to one.

## Decisions

- **CL-D1 — The collision world is in kinematicskit, bound to its bodies.**
  This revises KK-D9, which put collision behind a validator interface
  outside the kit. Avoidance (C4) needs distance rows whose Jacobians come
  from the kit's snapshot. So shapes attach to bodies of a
  `KinematicModel` (or to the world) with an offset, and one call poses
  them all from a `KinematicState` (q plus root poses).
  - Geometry sources stay outside the kit: RobotKit, CadKit through
    cadbridge, the scene, terrain. Each turns its own model into shapes.
  - The kit holds one shape vocabulary:
    - primitives: box, sphere, capsule, cylinder, plane, half-space;
    - convex point sets;
    - triangle meshes;
    - height fields.
- **CL-D2 — One interface, a native backend first.** The Haxe interface is
  `CollisionWorld`. Its operations:
  - add, remove and move objects;
  - pose them from a state;
  - check collision;
  - report the distance and closest points for pairs within a query
    distance.

  `NativeCollisionWorld` implements it on coal through the
  kinematicskit-native C ABI (KK-D20). Planning (C6) and validation (C3)
  use only the interface, so they stay pure Haxe (KK-D14).
  - Without native code (editor, browser), nothing implements the
    interface yet. Checks are then reported as unchecked, never as passed.
  - A pure-Haxe backend (spheres, capsules and boxes, from
    `ToolClearanceChecker` and `RobotLinkBounds`) can come later. It is not
    on this plan's path.
- **CL-D3 — Allowed pairs are a matrix with recorded reasons.** It is
  MoveIt's allowed-collision matrix, built in this order:
  1. groups (each robot's links, its tool, held parts, the environment);
  2. defaults: bodies joined by one joint, and pairs that overlap at the
     group's reference configuration (MuJoCo's rule, which also catches
     links that cannot collide). Static-versus-static pairs are never
     checked;
  3. explicit allow or deny overrides from the cell.

  Every allowed pair records why (adjacent, overlaps at reference,
  declared), so a report can say why a contact was not counted.
  Process contact is allowed per program segment, never globally:
  - a torch touching its seam;
  - a gripper closing on its part;
  - a bucket in the soil.
- **CL-D4 — Two distances per pair class.**
  - The **safety margin** `ds`: clearance below it is a violation in
    validation.
  - The **influence distance** `di` (greater than `ds`): pairs closer than
    it get avoidance rows in C4.

  Defaults:
  - robot to environment: `ds` 10 mm, `di` 50 mm;
  - self: `ds` 5 mm, `di` 30 mm;
  - allowed process contact: no margin.

  The cell overrides them. The report records the margins it used.
- **CL-D5 — Path checks are conservative, not just sampled.** Between two
  checked configurations, no point on a body moves further than a bound:
  - the bound is the sum over the joints from the root of |Δq_j| times the
    body's farthest distance from joint j's axis (plus |Δp| for prismatic
    joints and moving roots). Δq_j is the joint's total travel over the
    segment, not the difference of its ends, so a timed path that
    overshoots between samples is still bounded;
  - with motion bounds `Ba` and `Bb` for the pair, clearance along the
    segment is at least `(d0 + d1 − Ba − Bb)/2`, so the segment is clear
    when that is at or above `ds`;
  - otherwise the segment is bisected.

  This is the same argument FCL and coal use for conservative advancement.
  With it, validation can claim "no contact closer than `ds`" for the whole
  timed path, not only at samples. The report marks the check
  `MK_METHOD_EXACT` only when the bound closed every segment. Should
  bisection hit its depth limit, the check is marked sampled and the
  segment is named in the report.
- **CL-D6 — The scene changes while a program runs.** Objects can be added,
  removed and moved between queries. Held parts re-attach from the world to
  a tool body at a grasp, and back at a place. A height field updates a
  region of cells (coal `HeightField::updateHeights`) as digging lowers
  `HeightMap`. Every object keeps a stable id across updates, so allowed
  pairs and reports survive changes.
- **CL-D7 — Free-space planning is our own RRT-Connect, with OMPL as the
  reference.** OMPL is not vendored, for two reasons: it needs Boost
  (serialization, filesystem, program_options), and the planner must run
  without native code (KK-D14).
  - MotionKit gets a `MotionPlanner` interface. Its first implementation is
    bidirectional RRT-Connect in joint space, followed by shortcutting.
  - Edges are checked with CL-D5's bound.
  - The planner hands joint waypoints to the existing timing, so the
    result is validated like any other path.
  - OMPL's planners are the reference, and its benchmark set
    (planning time, path length, success rate) is the comparison.
  - Asymptotically optimal or constrained planning (OMPL's RRT*, BIT* or
    constrained state spaces) can come in later as an optional native
    backend behind the same interface, if needed.

## Steps

Each step is its own commit with all suites green.

- **C0 — This plan.**
- **C1 — Native collision world.**
  - Link `coal::core` into `kinematicskit_core`, beside ProxQP.
  - C ABI:
    - `kk_collision_world_create` and `_destroy`, against a model handle;
    - add a shape, as a primitive, convex points, a mesh (float64 vertices
      plus indices) or a height field, attached to a body or to the
      world, with an offset;
    - remove and move objects, and update a height-field region;
    - allow or deny pairs;
    - `kk_collision_world_update` from q and root poses;
    - `kk_collision_world_check`, returning whether anything collides,
      with the first pairs;
    - `kk_collision_world_distances(query distance)`, returning per pair
      the ids, signed distance, closest points (world) and normal.
  - Broadphase: every checked pair, filtered by world bounding boxes. A
    coal broadphase manager comes in if scenes grow large enough to need
    it.
  - In Haxe: the `CollisionWorld` interface (pure kit) and
    `NativeCollisionWorld` (native package).
  - Tests: C++ (shapes against each other, a mesh, a height field, poses
    following FK, allowed pairs) and Haxe (the same through the wrapper).
  - Convex shapes are built from points with no qhull. Confirm that coal's
    support function works without neighbour lists (or build the lists
    from `ConvexHullVertices`' faces).
- **C2 — Geometry from the cell.**
  - RobotKit builds a collision group from a `RobotModel`:
    - link primitives;
    - link hulls;
    - the tool's `ToolCollisionShape`;
    - CAD-sourced link meshes and hulls through cadbridge (the
      `EndEffectorCollision` path).
  - Default allowed pairs come from adjacency and from overlap at the
    reference configuration.
  - Environment: scene boxes, CAD parts and fixtures as meshes or hulls at
    `AssemblyState.worldPose`, and `HeightMap` as a height field.
  - Out of scope here: loading URDF `<mesh>` files. They stay references
    until a mesh importer exists.
- **C3 — Validation.**
  - Add `MK_CHECK_COLLISION` to `mk_validation_report`, with
    `mk_report_set_collision`, `ValidationReport.setCollision` and a
    guarantee field.
  - After `checkTaskSpace`, `ProgramCompiler.finish` checks the timed path
    with CL-D5. On failure the report gives the time, the pair, the
    clearance and the margin, and compilation throws as it does for task
    space.
  - Process contact windows come from the program's segments.
  - Without a world, the check stays `UNCHECKED`.
- **C4 — Avoidance (Lane D D4).** Pairs within `di` become inequality rows
  in the velocity-damper form
  `n·(J_a − J_b)·Δ ≥ −ξ·(d − ds)/(di − ds)·dt`, using closest points and
  the snapshot's point Jacobians.
  - The rows go into `DifferentialIk`: the native QP takes general
    inequalities, as mink does.
  - `PrioritizedSolver` takes them as a second active set beside its box
    limits. This is new: today it only pins DOFs.
  - The servo and IK then stay clear instead of only finding out
    afterwards.
- **C5 — The path search avoids collisions.**
  - `RedundancyResolver` and `PathConfigurationSelector` drop candidates
    that collide.
  - Their edges are checked with CL-D5.
  - An optional clearance cost lets the search prefer room over minimal
    motion.
- **C6 — Free-space planning (CL-D7).**
  - The `MotionPlanner` interface in MotionKit.
  - RRT-Connect with shortcutting in pure Haxe, over a `CollisionWorld`.
  - `ProgramCompiler` plans a joint move that asks for it (a planned
    MoveJ) and then times it as usual.
  - Tests:
    - narrow passages;
    - start or goal in collision (named, not planned);
    - determinism from a seed;
    - a planned move through validation.
  - An OMPL benchmark scene is noted for an out-of-tree comparison.

## Out of scope

- Continuous collision on swept volumes. CL-D5's bound gives the same
  guarantee for rigid links with less machinery.
- Dynamics-level contact. That stays with SimKit and MuJoCo.
- Octrees and point clouds from sensors. coal supports them through
  octomap, and they come when perception needs them.
- Loading mesh files (URDF `<mesh>`).

## Progress log

### C1 — Native collision world (2026-10-01)

Done as planned, with these differences:
- **Height-field distances.** coal answers only collision tests on height
  fields (`HeightFieldShapeDistancer` throws "not implemented"). The world
  compares the cells near the other object instead: two triangular prisms
  per cell, down to the field's minimum height, split as coal splits them
  for collision, with coal's convex distance on each. That gives signed
  distances (a penetration depth is measured within one cell), and a
  height field against a mesh also works (convex against mesh). Two height
  fields cannot be compared.
- **Convex without qhull.** `PointConvex` builds a coal convex from points
  alone. With no neighbour lists, coal's support function scans every
  point, which is fine for `ConvexHullVertices`' at most 64. Coplanar
  point sets are refused.
- **Pair statuses** in precedence order:
  1. a declared rule;
  2. static (both on the world);
  3. rigid: the same body, or joined only through fixed joints, so a tool
     on a fixed flange counts as part of the last link;
  4. adjacent: one movable joint apart, through fixed joints on either
     side;
  5. overlapping at reference;
  6. checked.

  A pair that would be checked but cannot be compared is `Unsupported`, and
  queries fail (`KK_ERROR_UNSUPPORTED`) until it is allowed or removed.
- **Interfaces:**
  - C ABI: `kk_collision_*` in `kinematicskit.h`.
  - Kit (pure Haxe): `CollisionWorld`, `CollisionGeometry`,
    `CollisionPairStatus`, `CollisionPairRule`, `CollisionPair`,
    `CollisionDistance`.
  - Native: `NativeCollisionWorld`.
- **Tests:**
  - `tests/cpp/collision_world.cpp`, standalone CTest:
    - statuses;
    - distances with closest points and normals;
    - FK posing;
    - re-attaching;
    - rules and reference overlap;
    - a mesh, a convex block, a half-space;
    - a height field, including updates and against a mesh;
    - unsupported pairs;
    - argument checks.
  - `native/tests`:
    - distances follow the Haxe snapshot to 1e-9 on random trees;
    - a grasped part rides on the tool;
    - raised terrain collides.
  - The native suite passes (369 assertions), as does the pure kit (202)
    and MotionKit (9541), which now links coal through
    `kinematicskit-native`.
- **Build.** coal is linked into `kinematicskit_core`, so every native kit
  build compiles it once (about a minute). The library has no new dynamic
  dependencies, and its dependency records list no Boost or assimp header.
