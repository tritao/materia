# Collision — checking, avoidance and free-space planning (plan)

Builds on KinematicsKit K6 (one group, one solver, one path search) and
KK-D20 (coal, vendored from `tritao/coal`, branch `materia`). Collision
checking had been planned as "a future plan" by MotionKit Lane D (D4) and
Lane C (OMPL listed as out of scope). This is that plan.

Revised on 2026-10-01, after CL1: the world moves out of the kit into its
own package, and three errors are fixed (see the revision log). The plan
moved from `kinematicskit/plans/` to `collisionkit/plans/` with CL2.

## Where things stand (2026-10-01, main's clearance code 2026-10-09)

- **coal** is a submodule at `collisionkit/native/vendor/coal` (since CL2;
  it was at `kinematicskit/native/vendor/coal`).
  - CL1 linked it into `kinematicskit_core`, so every package that built
    kinematicskit-native compiled it. Since CL2 only collisionkit-native
    does.
  - The build has primitives, convex sets, BVH meshes, height fields and
    the broadphase managers. It has no octrees, no mesh-file loading and
    no qhull. Hulls come as point sets from CadKit's pure-Haxe
    `ConvexHullVertices`.
  - Found in CL1 and since:
    - no distance queries on height fields (implemented in our fork in
      CL2b, CL-D13);
    - meshes are surfaces: a shape wholly inside one is not in contact;
    - `HeightField::updateHeights` clamps heights at the field's minimum;
    - no conservative advancement (FCL has it, coal does not);
    - a swept-sphere radius inflates primitives and convex sets.
- **Models:** RobotKit and CadKit compile separate `KinematicModel`s
  (`RobotKinematics.compile`, `AssemblyKinematics.compile`). A cell has
  several: each robot, each CAD mechanism.
- **RobotKit geometry:**
  - `CollisionShape` holds a box, sphere, capsule or cylinder with a pose in
    its link's frame. `Link.collisionGeometry` is only a mesh reference.
  - Each shape has a contact policy for simulation (`ShapeContact`: layers,
    pairs only, pairs and environment), and robots declare contact pairs.
  - URDF import keeps primitives; a `<mesh>` stays a reference and warns.
    MJCF import keeps primitives too.
  - Links can also get convex hulls, passed beside the model
    (`Simulation.addRobotAtPose(linkCollisionHulls)`).
  - Tools have their own vocabulary, `ToolCollisionShape`: none, a box, a
    cylinder, or hull pieces with padding.
  - Self-collision exclusion exists only inside SimKit's MuJoCo backend
    (`add_self_collision_excludes`): within one articulation,
    parent-child pairs and pairs that overlap at rest.
- **CAD geometry:**
  - `Shape.tessellate` gives float64 triangle meshes, inside curved faces
    by up to the linear deflection.
  - `ConvexHullVertices` gives hulls of at most 64 points, or an enclosing
    26-DOP (`enclosingFromMesh`).
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
- **Programs and timing:**
  - `MotionOp` has no grasp, place or process operation. A gripper is a
    `SetOutput`; process events are `MK_EVENT_PROCESS` commands.
  - MoveJ is a Ruckig state-to-state move, rest to rest, with time
    synchronization only (`MK_SYNCHRONIZATION_TIME`). Its path in joint
    space is not a straight line.
  - Cartesian moves are timed along `JointPathSamples`.
- **Validation:**
  - `mk_validation_report` checks position, velocity, acceleration, jerk,
    continuity and task space; collision is not among them.
  - Each check has one `joint` slot and a method (exact or sampled); the
    report has a list of assumptions.
  - `ProgramCompiler.finish` runs `checkTaskSpace` after timing.
- **Solvers:** `DifferentialIk`'s native QP takes box bounds only
  (`lower ≤ Δ ≤ upper`). `PrioritizedSolver` keeps an active set of pinned
  DOFs.

`ApplicationSimulation` assembling a SimKit session is the closest thing
to a cell-wide collision model.

**Main's clearance code** (arrived with the 2026-10-09 rebase, from the
process-path plan, `motionkit/plans/PROCESS_PATH_PLANNING.md`):
- `ArmClearance` (robotkit autonomy): convex hulls per link, from the
  simulation hulls, assembly parts and deposited weld hulls
  (`MissionPlayer`). Pure-Haxe GJK (`ConvexDistance`) over all pairs, with
  a sphere and box broad phase. Neighbour links touching at a reference are
  excluded; tool contact has its own margin. Queries:
  - `violation(q, contact, wanted, displacements)`: the first pair closer
    than the margin plus each body's inflation;
  - `closest`;
  - `sweep`: sampled along a straight joint line.
- `ClearanceMotionBounds` and `ClearanceMotionEnvelope`: CL-D5's motion
  bound on the compiled kinematic forest, with couplings, prismatic
  joints and moving work frames.
- `JointCurveClearance`, implementing `JointPathClearanceProof`: a
  conservative proof on refined quintic paths. Per span, a query at the
  midpoint with the margin raised by the envelope's bound, then bisection
  to depth 20.
- `LazyCollisionLadder` and `StructuredLadder`: the ladder search that
  prunes colliding candidates and edges and retries (PP-D5). This is CL6
  for process paths.
- `TrajectoryClearance`: sampled checks (10 ms, 0.02 rad) for timed
  trajectories without a certificate: joint moves, and generated entry,
  exit and hold moves.
- No clearance entry in the validation report (now in trajectorykit,
  `trajectory_core.h`); failures throw. Main added `MK_CHECK_METHOD_BOUND`
  for task-space certificates.
- PP10 plans to implement the clearance seams on collisionkit once CL2–CL4
  are on main, and to delete `ArmClearance` from process planning.

## Decisions

- **CL-D1 — The collision world is its own package, collisionkit, posed
  from body poses.** (Revised 2026-10-01. The first version put the world
  in kinematicskit, bound to one `KinematicModel`.)
  - The world knows bodies, not models. The caller registers bodies, poses
    them (seven numbers each, usually from a kit snapshot) and attaches
    objects to them with offsets. One world holds a whole cell: several
    robots and CAD mechanisms, each its own model, and the environment.
  - kinematicskit keeps what needs a model's structure, and does not
    depend on collisionkit:
    - rigid, adjacent and closure pairs (CL-D3);
    - motion bounds for path checks (CL-D5);
    - the avoidance task, which turns pair results (bodies, closest
      points, normal, distance) into rows with the snapshot's point
      Jacobians (CL5).
  - Geometry sources stay outside both: RobotKit, CadKit through
    cadbridge, the scene, terrain. Each turns its own model into shapes.
  - One shape vocabulary:
    - primitives: box, sphere, capsule, cylinder, half-space;
    - convex point sets;
    - triangle meshes;
    - height fields.

    Primitives and convex sets can be inflated by a radius (coal's
    swept-sphere radius), which carries tool padding exactly.
  - Only packages that want collision depend on collisionkit, and only
    those that use its native library compile coal.
- **CL-D2 — One interface, a native backend, and native code wherever
  collision is wanted.**
  - collisionkit's pure half holds the `CollisionWorld` interface and its
    types. Validation (CL4), the path search (CL6) and the planner (CL7)
    use only the interface, so they stay pure Haxe.
  - Its native half, `NativeCollisionWorld`, implements it on coal through
    collisionkit's own C ABI.
  - Without a backend, checks are reported as unchecked, never as passed.
  - Reach comes from the native library, not from a second backend: the
    desktop editor links it, and the browser build compiles it into its
    Emscripten host (coal needs only Eigen). A pure-Haxe backend (spheres,
    capsules and boxes, from `ToolClearanceChecker` and `RobotLinkBounds`)
    comes only if a real need appears.
- **CL-D3 — Allowed pairs are rules on bodies, with recorded reasons.** It
  is MoveIt's allowed-collision matrix.
  - Rules name pairs of world bodies, so an object attached later (a
    grasped part) follows its body's rules without new calls. Rules on
    object pairs exist too, for process contact (a torch and its seam).
  - Every rule records why: static, rigid, adjacent, closure, overlaps at
    reference, declared, process contact. A report can say why a contact
    was not counted.
  - Defaults, built by the caller:
    1. static: bodies the caller marks as fixed in this cell (the world, a
       fixed robot's base and what is rigid to it). Static pairs are never
       checked;
    2. per model, from the kit: rigid (joined only through fixed joints),
       adjacent (one movable joint apart, through fixed joints on either
       side) and closure pairs (bodies joined by a closure, KK-D4);
    3. overlap at reference: pairs among one articulation's own bodies
       that overlap at its reference configuration. This is MoveIt SRDF's
       "Default" reason; SimKit does the same within an articulation. It
       never applies across articulations or against the environment: an
       overlap there is a layout error, and it is reported.
  - Declared allow or check rules from the cell override the defaults.
  - Process contact is allowed per window, never globally (CL-D9).
  - SimKit takes its exclusions from the same rules instead of computing
    its own (CL3b), so simulation and checking agree on which pairs may
    touch; it may exclude more, never fewer (CL-D10).
- **CL-D4 — Two distances per pair class.**
  - The **safety margin** `ds`: clearance below it is a violation in
    validation.
  - The **influence distance** `di` (greater than `ds`): pairs closer than
    it get avoidance rows in CL5.
  - Each body belongs to a group the caller declares: a robot's links, its
    tool, held parts, the environment. Pair classes and defaults:
    - self (within one robot, its tool and held parts): `ds` 5 mm, `di`
      30 mm;
    - robot to environment: `ds` 10 mm, `di` 50 mm;
    - robot to robot: `ds` 10 mm, `di` 50 mm;
    - inside a process contact window: no margin.
  - A window covers its approach and retract too, with a margin of its own
    (CL-D9). Otherwise a torch approaching its seam would breach `ds`
    before the window opens.
  - The world answers one query distance at a time. Callers query at the
    largest `di` and apply each class's margins to the results.
  - The cell overrides the defaults. The report records the margins it
    used.
- **CL-D5 — Path checks are conservative, not just sampled.**
  (Implementation revised 2026-10-09: main's `ClearanceMotionEnvelope`
  and `JointCurveClearance` are this decision's implementation; see
  CL-D12. They use one midpoint query per span with the margin raised by
  `Ba + Bb`, an equally sound form of the two-endpoint bound below. No
  second proof is built. Certificates are reported with main's
  `MK_CHECK_METHOD_BOUND`.) Between two checked configurations, no point on a body moves further than a bound
  `B`, the sum of:
  - for each joint between the root and the body, `R_j · T_j`:
    - `T_j` is the joint's total travel over the segment, not the
      difference of its ends, so a timed path that overshoots between
      samples is still bounded. A coupled joint's travel is its DOF's
      times `|ratio|`. A timed path is piecewise polynomial, so `T_j` is
      exact from its extrema; the peak speed times the duration is a
      cheaper bound;
    - `R_j` bounds the distance from joint j's axis to any point of the
      body, over every configuration the segment can pass through. The
      distance at either end is not enough: it changes as the joints
      between j and the body move (a base yaw while the arm unfolds). The
      default holds for every configuration: the sum of the offsets
      between consecutive joints from j to the body, plus the body's
      radius about its frame, including inflation and attached objects.
      A prismatic joint has `R_j = 1`;
  - for a root that moves without joints (a pose set from outside): its
    translation, plus its rotation angle times the body's farthest
    distance from the root's origin.

  With bounds `Ba` and `Bb` for a pair, clearance along the segment is at
  least `(d0 + d1 − Ba − Bb)/2`. The segment is clear when that is at or
  above `ds`; otherwise it is bisected.
  - Distances count as lower bounds: a pair not returned within the query
    distance counts as at the query distance, and the solver's tolerance
    is subtracted.
  - The bound depends only on each joint's travel, not on the path's
    shape. A segment closed without bisection is clear for any motion
    between its ends whose per-joint travel stays within `T_j`. Bisection
    evaluates the actual path.
  - This is Schwarzer, Saha and Latombe's adaptive collision checking for
    articulated robots (2005). FCL's conservative advancement is the
    rigid-body case. coal has neither, so the bound is computed in Haxe,
    in the kit, which knows the model's structure.

  With it, validation can claim "no contact closer than `ds`" for the whole
  timed path, not only at samples, within CL-D8. The report marks the
  check `MK_CHECK_METHOD_EXACT` only when the bound closed every segment.
  Should bisection hit its depth limit, the check is marked sampled and the
  segment is named in the report.
- **CL-D6 — The scene changes while a program runs.**
  - Objects can be added, removed and moved between queries. Held parts
    re-attach from the world (or a fixture's body) to a tool body at a
    grasp, and back at a place.
  - Validation replays these changes in program time order, and never
    bisects across one (CL-D9).
  - A height field's grid is replaced as digging lowers `HeightMap`. A
    region update can come if grids get large.
  - coal clamps heights at the field's minimum, so the world refuses
    heights below it, and terrain sources set the minimum below the
    deepest possible dig.
  - Every body and object keeps a stable id across changes, so rules and
    reports survive them.
- **CL-D7 — Free-space planning is our own RRT-Connect, with OMPL as the
  reference.** (Revised 2026-10-09.)
  - **Why not an existing planner.**

    | Option | Why not now |
    | --- | --- |
    | OMPL | It needs Boost, which we kept out with coal. Its RRT-Connect alone is a few hundred lines; its value is its other planners and its benchmark set, which we do not need yet. |
    | VAMP | It needs robot-specific code generated ahead of time, with robots made of spheres. That does not fit cells that change at runtime or CAD meshes in coal. |
    | MoveIt, Tesseract, MPLib | They bring ROS or FCL + OMPL + Pinocchio, which duplicates what we have. |
    | cuRobo | NVIDIA licence and CUDA. |

  - **Why Haxe, and where native code goes.** Reach is not the reason: a
    planner needs a collision backend, and that is native everywhere
    (CL-D2), the browser included. Haxe is for integration: the kit's
    models, redundancy parameterizations, MotionKit timing and phase
    synchronization, and the existing suites. The cost that dominates
    RRT-Connect is collision checking, which is native. So the hot path
    stays native:
    - edges and configuration lists are checked in batched native calls,
      with CL-D5's bisection done natively (CL2 adds the call). That
      avoids one FFI call, and one array copy, per configuration;
    - tree nodes are stored flat in preallocated arrays, not one array
      per configuration;
    - nearest-neighbour search becomes a k-d tree once trees grow (a
      linear scan scales badly).
  - **Measured, not assumed.** Our planner is benchmarked against OMPL's
    RRT-Connect on the same scenes, with the same validity checker
    (out of tree). If the Haxe tree code is a meaningful share of
    planning time, nearest-neighbour search (or the whole loop) moves to
    C++, with that measurement as the reason.
  - MotionKit gets a `MotionPlanner` interface. Its first implementation is
    bidirectional RRT-Connect in joint space, followed by shortcutting.
  - Edges are checked with CL-D5's bound, with a small slack over `ds`.
  - A planned path runs along the edges that were checked. Each edge is a
    rest-to-rest move with Ruckig's phase synchronization, which is a
    straight line in joint space. MotionKit's generator accepts only time
    synchronization today; CL7 adds phase. Blending through waypoints
    comes later, and validation (CL4b) re-checks whatever is timed.
  - OMPL's planners are the reference, and its benchmark set (planning
    time, path length, success rate) is the comparison.
  - Asymptotically optimal or constrained planning (OMPL's RRT*, BIT* or
    constrained state spaces) can come in later as an optional native
    backend behind the same interface, if needed. Recheck OMPL's Boost
    requirements then: they were not verified for its current release.
- **CL-D8 — "Exact" means exact for the declared geometry.**
  - The report lists what the geometry approximates, in its assumptions:
    - the tessellation deflection of CAD meshes (tessellated curved faces
      lie inside the true surface by up to it);
    - the kind of each hull (enclosing or not);
    - inflation;
    - the distance solver's tolerance.

    The margin absorbs these, or the report says it does not.
  - coal meshes are surfaces: a shape wholly inside one reads as clear. A
    path checked with CL-D5 from a clear start cannot cross a surface
    unnoticed, but a check of one configuration (a start, a goal, a path
    search candidate) can be fooled. So solids are convex pieces or hulls.
    Meshes are for surfaces (floors, walls, sheet), or for closed meshes
    with an inside test.
- **CL-D9 — Process contact and attachments are declared with the
  program.**
  - Whoever builds the program declares, per operation or event:
    - attach or detach: which object goes to which body, at which offset;
    - contact windows: the pair, the segments the window covers (approach,
      contact, retract) and the margin on approach and retract.

    That is processkit for process devices, and the cell for grasps.
  - Typical windows:
    - a torch on its seam;
    - fingers closing on a part;
    - a part leaving or reaching its fixture;
    - a bucket in the soil.
  - MotionKit validation (CL4b) consumes the declarations. Without them,
    every contact is a collision.

- **CL-D10 — RobotKit's contact settings are simulation-only, except
  contact pairs.** (Decided 2026-10-09.)
  - `ShapeContact` (`Layers`, `PairsOnly`, `PairsAndEnvironment`) and the
    blueprint's `selfCollision` tune simulated contact for stability and
    cost. Clearance ignores them: a `PairsOnly` finger pad is still
    checked against a fixture. The report records that a
    simulation-only setting was ignored.
  - A `ContactPair` says two shapes are meant to touch (fingers closing
    on each other). It becomes an allow rule on that object pair, with
    the reason "declared contact". Checking it would always fail.
  - Intended contact with the environment (a grasp, a foot on the floor,
    a torch on its seam) is not a simulation setting: it is a CL-D9
    window, declared with the program.
  - Rules flow one way, from these rules to MuJoCo as excludes. The
    simulation may switch off more contacts than the rules allow, never
    fewer.
- **CL-D11 — Convex decomposition is V-HACD 4, with enclosure checked by
  us.** (Decided 2026-10-09.)
  - Non-convex solids (links, tools, parts, fixtures) become sets of
    convex pieces (CL-D8), instead of one hull or a surface mesh.
  - V-HACD 4: BSD-3, one header, no libraries. It is archived upstream
    (its README points to CoACD), which only means frozen. It is a plain
    submodule of upstream `kmammou/v-hacd` at tag `v4.1.0`, as the repo
    keeps every dependency it does not change. A `tritao` fork comes only
    if a fix is needed, and creating one needs the user's go-ahead.
  - CoACD is rejected for now. Its pieces are better, but its build pulls
    in Boost, OpenVDB, zlib and spdlog: Boost is what coal was trimmed to
    avoid, and OpenVDB would weigh on the browser build. Using it later
    means trimming it as coal was.
  - Neither library promises that the pieces enclose the input. After
    decomposition we measure how far the tessellated mesh sticks out of
    the pieces' union, and inflate the pieces by that distance (coal's
    inflation radius), so the union encloses it. The inflation and the
    tessellation deflection go into the report's assumptions (CL-D8).
  - Decomposition runs when a CAD part changes, not at planning time.
    The pieces are cached with the part.

- **CL-D12 — Main's clearance design stays; collisionkit replaces its
  geometry engine.** (Decided 2026-10-09.)
  - Main's process-path work already has the proof
    (`JointCurveClearance`), the motion bound (`ClearanceMotionEnvelope`)
    and the collision-aware ladder (`LazyCollisionLadder`). What it lacks
    is the geometry: `ArmClearance` is convex hulls only, Haxe GJK over
    every pair, without primitives, meshes, height fields or pair rules.
  - An interface is extracted from what `ArmClearance` exposes: violations
    with per-body inflation, the closest pair, straight-line joint sweeps,
    the motion envelope, the tool pose. `ClearanceViolation` moves out of
    robotkit to sit with it. `ArmClearance` and a collisionkit adapter both
    implement it.
  - Allowed: extracting interfaces, moving types, adding adapters and new
    code paths, all without changing behaviour. Not allowed: changing the
    proof, the margins or the ladder, or deleting `ArmClearance`. Switching
    process planning over and deleting `ArmClearance` is main's PP10, and
    is decided with the process-path work.

- **CL-D13 — Fix coal in our fork when it limits us; upstream the fix.**
  (Decided 2026-10-09.)
  - Gaps go into our `tritao/coal` fork (branch `materia`), not into
    workarounds in collisionkit. Patches stay small and upstreamable to
    `coal-library/coal`, so the fork's diff stays small.
  - Add: height-field distance queries. coal's
    `HeightFieldShapeDistancer` still throws "not implemented" (checked on
    upstream `devel`; v3.0.4 is the latest release). It is implemented in
    coal: the height field's BV tree is traversed with distance lower
    bounds, cells are compared as coal's own prisms
    (`buildConvexTriangles`, already used for collision), and the signed
    distance, closest points and normal come back through coal's normal
    distance API. Height field against mesh goes through the same prisms
    (prism against mesh already works).
  - Later, only if V-HACD decomposition proves too coarse for some part: an
    opt-in "closed mesh" mode. With no intersection, a winding-number inside
    test makes the distance negative, so a shape wholly inside a closed CAD
    mesh no longer reads as clear (CL-D8).
  - Not added to coal:
    - a hull builder: V-HACD's pieces carry their triangles, and coal's
      `Convex<Triangle>` constructor builds neighbours from them; CadKit's
      point-only hulls (at most 64 points) are fine with the linear scan;
    - mesh inflation: per-body inflation is extra margin per pair
      (`required + δa + δb`), as main's proof does it;
    - motion bounds or continuous collision: `ClearanceMotionEnvelope`
      owns that;
    - batching, pair rules or broadphase changes: those live in
      collisionkit;
    - height field against height field: it stays `Unsupported`.

## Steps

Each step is its own commit with all suites green.

- **CL0 — This plan.** Done (9ca6aed52); revised after CL1 and on
  2026-10-09.
- **CL1 — Native collision world in the kit.** Done (311788aa9), see the
  progress log. CL2 moves it out of the kit.
- **CL2 — collisionkit.** Done, see the progress log.
  - New package:
    - a pure half: `CollisionWorld`, its types and the rule types;
    - a native half: its C ABI, `NativeCollisionWorld`, and coal, moved
      from `kinematicskit/native/vendor/coal` with its third-party notes.
  - `kinematicskit_core` drops coal.
  - The world is posed from body poses. Bodies get the world's own ids, and
    the world keeps no model copy and no FK.
  - Rules:
    - on body pairs, with reasons;
    - on object pairs, for process contact;
    - static bodies;
    - declared overrides.
  - Overlap at reference works over a set of bodies the caller gives (one
    articulation). This fixes CL1, which also allowed robot-environment
    pairs.
  - Inflation for primitives and convex sets.
  - Results name the bodies as well as the objects. A query over chosen
    pairs lets CL-D5 bisect one pair without recomputing the rest.
  - Violation queries as main's proof uses them (CL-D12): the first pair
    closer than its margin plus each body's inflation (`required + δa +
    δb`), and the closest pair with names.
  - Batched checks (CL-D7, CL-D12): one call takes many pose sets, each
    with its per-body inflation, and returns the first failure.
    `JointCurveClearance` makes thousands of queries per path (about 3,900
    per direction on G17), so one FFI call each would cost more than the
    geometry.
  - Height updates below the field's minimum are refused.
  - In kinematicskit (pure Haxe): rigid, adjacent and closure pairs of a
    `KinematicModel`, with reasons.
  - This plan moves to `collisionkit/plans/`, and KINEMATICS.md and
    MotionKit's plans follow.
  - Tests:
    - CL1's C++ and Haxe tests carried over, posed from the snapshot's
      poses;
    - two models in one world;
    - a held part following its body's rules;
    - overlap at reference leaving the environment checked;
    - inflation against an exact distance;
    - refused heights.
- **CL2b — coal: height-field distance (CL-D13).** Done, see the progress
  log.
  - Implemented in the fork, with coal's own tests for a box, sphere,
    capsule, convex and mesh against a height field, including penetration
    and the closest points.
  - collisionkit's world then uses coal's height-field distance directly:
    the cell-prism workaround (`field_distance`) is deleted, and its tests
    keep passing unchanged.
  - An upstream PR to `coal-library/coal` comes later, not in this run,
    when the user says so.
- **CL3 — Geometry from the cell**, in two commits. CL3a can run
  unattended; CL3b reaches into SimKit and is done with review.
- **CL3a — Robot, tool and environment geometry; decomposition.** Done,
  see the progress log.
  - RobotKit builds a robot's bodies, groups and shapes from a
    `RobotModel`:
    - link primitives;
    - link hulls;
    - the tool's `ToolCollisionShape`, with hull padding as inflation;
    - CAD-sourced link meshes and hulls through cadbridge (the
      `EndEffectorCollision` path).
  - Default rules come from the kit's pairs and from overlap at reference.
    Each robot's reference configuration is named (its home if the model
    has one, else zero) and recorded.
  - RobotKit's contact settings are mapped per CL-D10: `ShapeContact` and
    `selfCollision` are ignored and recorded, and each `ContactPair` becomes
    an allow rule with the reason "declared contact".
  - Environment:
    - scene boxes;
    - CAD parts and fixtures at `AssemblyState.worldPose`, as hulls
      (solids) or meshes (surfaces, CL-D8);
    - CAD mechanisms as bodies posed from their own model;
    - `HeightMap` as a height field, its minimum below the deepest dig.
  - Tests:
    - one world with a robot, a CAD mechanism and terrain;
    - a fixture through the robot at its reference, reported as a layout
      error;
    - the CL-D10 mapping: a `PairsOnly` shape still checked, a contact
      pair allowed with its reason;
    - a decomposed non-convex part whose pieces enclose its tessellation
      after inflation, with the inflation recorded.
  - Convex decomposition (CL-D11): V-HACD 4 as an upstream submodule at
    `v4.1.0`, built into cadbridge's native code (or CadKit's, wherever
    the tessellation is), with the enclosure measurement and inflation,
    and the pieces cached with the part.
  - The geometry also covers what `ArmClearance` gets today (simulation
    hulls, assembly parts, deposited weld hulls, via `MissionPlayer`), so
    CL4a can compare the two worlds on the same scenes.
  - Out of scope here: loading URDF `<mesh>` files. They stay references
    until a mesh importer exists.
- **CL3b — One collision description shared with SimKit.** Done, see the
  progress log.
  - The cell's description (shapes per body, allowed pairs with their
    reasons) is consumed by both the coal world and the SimKit session.
  - MuJoCo gets the exclusions as excludes instead of recomputing them
    (`add_self_collision_excludes`), so the planner and the simulation
    cannot drift apart (CL-D10: the simulation may exclude more, never
    fewer).
  - The decomposed pieces feed MuJoCo too.
  - Test, a cross-check: at sampled configurations, every MuJoCo contact
    is also a collision in the coal world (meshes aside, which MuJoCo
    treats as hulls).
- **CL4a — The clearance interface and the collisionkit adapter**
  (CL-D12). Done, see the progress log.
  - Extract the interface from `ArmClearance`'s queries and move
    `ClearanceViolation` beside it. `StructuredJointPathPlanner`,
    `JointCurveClearance`, `LazyCollisionLadder`'s callbacks and the
    processkit runners take the interface instead of `ArmClearance`.
    Behaviour does not change.
  - A collisionkit adapter implements it from CL3a's cell geometry and
    CL2's batched, inflated queries. `ClearanceMotionEnvelope` gets its
    reach from the same geometry.
  - Parity tests on main's weld and rail scenes (G17 included): the two
    implementations agree on violations within the solver tolerance, and
    `JointCurveClearance` certifies the same paths with either. Every
    disagreement is explained in the log (geometry, hull versus
    primitive, tolerance).
  - Not here: switching process planning to the adapter, or deleting
    `ArmClearance` (main's PP10).
- **CL4b — Clearance in the validation report; conservative joint moves.**
  Done, see the progress log.
  - Add `MK_CHECK_COLLISION` to `mk_validation_report` (trajectorykit,
    `trajectory_core.h`), with its setter and `ValidationReport`'s.
  - A check's single `joint` slot cannot name a pair, so the report gains
    the pair (object ids and names) and the segment. `MK_CHECK_COUNT`
    grows, so the report's layout changes: it is versioned by
    `struct_size`, and the bindings are regenerated.
  - `ProgramCompiler` records what it already proves: a certificate from
    `JointCurveClearance` as `MK_CHECK_METHOD_BOUND`, a
    `TrajectoryClearance` pass as sampled, the closest pair and margin
    either way. Failures still throw, and are recorded first.
  - Joint moves and generated entry, exit and hold moves get the same
    proof on their straight joint lines (`ClearanceMotionEnvelope` with
    bisection), recorded as a bound when it closes. Where it does not
    close but the sampled check passes, the report says sampled and names
    the segment; it does not throw, so existing programs behave as before.
  - Contact windows and attachments (CL-D9) replay in time order; pair
    classes keep their margins (CL-D4).
  - Without a clearance world, the check stays `UNCHECKED`.
  - Tests:
    - a passing and a failing cell, recorded in the report;
    - a joint move whose sampled check misses a contact between samples,
      caught by the bound;
    - bisection at its depth limit reported as sampled;
    - a grasp and a place replayed;
    - a torch window with its approach margin.
- **CL5 — Avoidance (Lane D D4).** Done, see the progress log.
  - The kit's avoidance task takes pairs within `di`. CL1's normal `n`
    points from `a` toward `b`, so the distance changes at
    `n·(J_b − J_a)·q̇`. Each pair gives the velocity-damper row
    `n·(J_b − J_a)·Δ ≥ −ξ·(d − ds)/(di − ds)·dt`.
    - The Jacobians are point Jacobians at the two closest points.
    - `ξ` is a speed.
    - A side outside the solved model (the environment, another robot)
      has a zero Jacobian.
  - `DifferentialIk`'s native QP gains general rows (ProxQP's `C`, `l`,
    `u`); today it takes box bounds only.
  - Below `ds` a row demands separation, which can conflict with step
    limits. Rows get a small relaxation of their bound, as in mink, and the
    solution reports the rows it relaxed.
  - The servo and IK then stay clear instead of only finding out
    afterwards.
  - `PrioritizedSolver` comes later. Its collision rows rank above the hard
    tasks, and it reports a target they block. It only matters where a
    backend exists.
  - Tests:
    - the servo stopping short of an obstacle and sliding along it;
    - a pair against another robot;
    - recovery from inside `ds`.
- **CL6 — The path search avoids collisions.** Mostly done on main for
  process paths: `LazyCollisionLadder` prunes colliding candidates and
  edges and retries (PP-D5), and reaches collisionkit through CL4a's
  interface. `PathConfigurationSelector` is deleted, and PP9 deletes the
  `RedundancyResolver` fallback. What remains, later and with the
  process-path work:
  - an optional clearance cost, so the search prefers room over minimal
    motion.
- **CL7 — Free-space planning (CL-D7).** Done, see the progress log.
  - The `MotionPlanner` interface in MotionKit.
  - RRT-Connect with shortcutting in Haxe, over a `CollisionWorld`, with
    its own seeded random generator. Edges go through CL2's batched
    checks; nodes are stored flat; nearest-neighbour search is a k-d tree
    once trees grow (CL-D7).
  - Phase synchronization in MotionKit's generator, so a planned edge is a
    straight line in joint space.
  - `ProgramCompiler` plans a joint move that asks for it (a planned
    MoveJ), times each edge rest to rest, and validates the result (CL4b).
  - Main's PP7 already reserves a use: a blocked process entry with no
    clear k-best start goes through the planner.
  - Tests:
    - narrow passages;
    - start or goal in collision (named, not planned);
    - determinism from a seed;
    - the timed path following the checked edges;
    - a planned move through validation.
  - An out-of-tree benchmark against OMPL's RRT-Connect on the same
    scenes and checker. OMPL builds from source into a scratch prefix
    against the system's Boost (1.83's development packages are
    installed); nothing of it enters the repo. Time-boxed: if OMPL does not
    build within about an hour, the log says so and the benchmark waits.
    It records planning time, path length, success rate, and the
    share of time outside collision checks (CL-D7 decides from it whether
    any planner code moves to C++).
- **CL8 — Collision in the editor and the browser** (any time after CL3;
  steps written 2026-10-09, after PP10).
  - The desktop editor links collisionkit's native library and shows
    colliding and near pairs while a cell is laid out or a robot jogged.
  - The browser build compiles the library into its Emscripten host
    (linked since PP10).
  - **What the editor has to show it with** (survey 2026-10-09):
    - Scene objects (`EditorSceneObject`) are posed at their geometry
      centre, in metres, Z-up. Only boxes exist as collision proxies (the
      record's extents; `CadCollisionBounds` for CAD parts).
    - A generated project's parts are `project:<occurrence>` scene objects.
      Their hulls are in CAD units in the part frame
      (`AssemblyPhysicalPart.collisionHull`, or a component's multi-piece
      `collisionHulls`). `AssemblySimulationBridge.toRobotModel` prefers the
      multi-piece hulls, but `SimulationAssemblyParts` uses the single hull.
    - Outside simulation there is no robot joint state. The editor's "jog" is
      the project assembly: an inspector joint edit or an IK drag. Both
      reach the scene through `EditorScene.setAssemblyOccurrenceTransforms`.
      A drag preview publishes on every pointer move.
    - While a simulation runs, robot links are posed by `Simulation.linkPose`
      with `AssemblyRobot` hulls (MissionPlayer's clearance already uses
      them).
  - **CL-D14 — one editor collision world, rebuilt rarely, posed often.**
    - `SceneCollision` (app) describes the scene in one `CollisionDescription`:
      - every assembly occurrence is a body with its hull pieces;
      - each assembly is an articulation with the loaded state as its
        reference, so parts that touch by design are allowed, with that
        reason;
      - rigidly joined occurrences, and parent/child across a joint, follow
        CL-D3's rigid and adjacent rules;
      - every other `collisionEnabled` object is a movable body with its box
        (CAD parts with their collision bounds).
    - The world is rebuilt only when the collision content changes
      (objects added or removed, a shape or the collision flag changed;
      `SceneModel.physicsRecordsChanged` is the existing test). A pose
      change only re-poses bodies and queries again.
    - The query runs at most once per scene revision, when the overlay is on
      and the view asks: `colliding(0)` and `distances(near)`. The near
      distance is a setting, 10 mm by default.
    - One function gives an occurrence's collision pieces (multi-piece hulls
      when the component has them, else its hull). The editor and
      `SimulationAssemblyParts` both use it, which ends the disagreement
      above.
  - **CL8a — the editor's collision world.**
    - `SceneCollision`, the occurrence-pieces function, and
      `SimulationAssemblyParts` moved onto it.
    - Headless tests:
      - two overlapping boxes collide; moving one 5 mm apart makes the pair
        near; 50 mm apart, clear;
      - a project's parts at load report nothing (touching by design is
        allowed);
      - a joint edit that drives a link into the base reports that pair;
      - a pose-only change does not rebuild the world.
  - **CL8b — show it.**
    - Colliding objects are drawn with a red material override, near ones
      amber (`SceneView.setMaterial`, per view, nothing persistent).
    - A 2D overlay draws each pair's closest points and a line between
      them, with the distance, for the closest pairs (at most 16).
    - The viewport toolbar shows the count ("2 collisions, 1 near").
    - A setting `editors/3d/collision/show` (Editor Settings dialog), the
      near distance `editors/3d/collision/near_mm`, and a command
      `scene.toggle-collisions` in the viewport options menu.
    - Tests check the material overrides, the overlay key and the count
      headlessly (`SceneEditingTests` already builds a `SceneView` and a
      viewport without a GPU).
  - **CL8c — while simulating.**
    - Robot links (`AssemblyRobot` hulls at `Simulation.linkPose`) and the
      parts the simulation moves join the same world. Pairs show as in
      CL8b, as clearance next to MuJoCo's own contacts.
    - A test drives a simulated arm into a box and sees the pair.
  - **CL8d — the browser.**
    - The web build runs CL8a's tests' scene in its smoke test (`app/web/test.sh`),
      and a downloadable example shows its pairs.
    - Generated projects can't be compiled in the browser (no external
      commands), and CAD-shape hulls need the OCCT browser build; both are
      recorded as the browser's limits, not worked around.

## Order and gates (2026-10-09)

Working order:
1. CL2;
2. CL2b;
3. merge onto local `main` (no push);
4. CL3a;
5. CL4a;
6. CL4b;
7. CL5;
8. CL3b;
9. CL7, with its OMPL benchmark last.

CL6's remainder, main's PP10 and CL8 wait for review.

Gates, for work without review:
- **Each step is one commit, with its suites green:**
  - the standalone C++ tests;
  - the native and pure kit;
  - collisionkit's own suites, from CL2 on;
  - MotionKit;
  - RobotKit;
  - processkit's weld planning, from CL4a on.
- **No existing test assertion or expected value changes.** A step that
  would change one stops, and the log explains why; nothing is
  rebaselined. Without a clearance world every new check reports
  unchecked, which is what keeps this possible: only new tests carry
  worlds.
- **Known failure on main:** the pure kit's "DLS allocates nothing per
  iteration (-40.8 bytes)" fails identically on a clean `origin/main`. It
  is recorded, not fixed here.
- **Stop and write up, instead of deciding alone, on any of:**
  - a new dependency beyond those this plan names (V-HACD, and OMPL
    out of tree);
  - creating a fork;
  - changes to main's process-path logic beyond CL-D12's allowance;
  - deleting `ArmClearance`.
- **No pushes,** the coal fork included: its commits stay local. The
  parent's submodule pin may point at a local, unpushed coal commit
  overnight; pushing the fork and opening the upstream PR wait for the
  user, like every other push.
- OCCT is never rebuilt: set
  `MATERIA_CACHE_DIR=/home/joao/dev/materia-cache`. The shared checkout is
  touched only to move local `main`.

## Out of scope

- Continuous collision on swept volumes. CL-D5's bound gives the same
  guarantee for rigid links with less machinery.
- Dynamics-level contact. That stays with SimKit and MuJoCo.
- Octrees and point clouds from sensors. coal supports them through
  octomap, and they come when perception needs them.
- Loading mesh files (URDF `<mesh>`).
- A pure-Haxe collision backend, until a need appears (CL-D2).

## Progress log

### Plan revision (2026-10-01, after CL1)

A review of the plan against CL1 and the code around it changed:
- **Where the world lives** (CL-D1, CL-D2). In a kit, bound to one model,
  a cell's other models could only be world objects. Moving one rebuilt
  every pair, and two of them were never checked against each other (both
  static). Every user of `motionkit-robot` compiled coal. The world moves
  to collisionkit and is posed from body poses (CL2).
- **Three errors:**
  - the avoidance row had `n·(J_a − J_b)`. With CL1's normal from `a` to
    `b`, that bounds separation, not approach (CL5);
  - CL-D5 used the body's distance from a joint's axis without requiring
    it to hold over the whole segment. Measured at an end, it can be too
    small. It also missed a moving root's rotation and coupled joints;
  - overlap at reference applied to every pair, so a fixture through the
    robot at its reference would be allowed for good. It now applies
    within one articulation (CL-D3).
- **Gaps closed:**
  - meshes are surfaces (CL-D8);
  - margins per pair class, and the approach to process contact (CL-D4);
  - where windows and attachments come from (CL-D9);
  - planned moves following the checked edges (CL-D7);
  - general rows in the QP (CL5);
  - height clamping (CL-D6).
- **Corrections:**
  - the overlap rule is SimKit's, not MuJoCo's (MuJoCo itself only
    filters parent-child pairs);
  - coal has no conservative advancement;
  - the method is `MK_CHECK_METHOD_EXACT`.
- **Steps renamed** from C0–C6 to CL0–CL8. MotionKit's Lane C and other
  plans also number their steps C*n*.

### CL1 — Native collision world (2026-10-01)

Done as planned, with these differences (CL2 moves this into
collisionkit):
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
  5. overlapping at reference (over every pair; CL2 restricts it);
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

### Rebase onto main and planner revision (2026-10-09)

- **The branch is rebased onto main `1decbfc20`**, 956 commits on from
  where it started. CL0 and CL1 kept their content. Their commits are now
  9ca6aed52 and 311788aa9.
  - `motionkit/plans/README.md` keeps main's process-path row next to the
    collision rows.
  - `kinematicskit.hxi` is regenerated from the merged header.
  - CL1's C++ test packs main's model format 2 (joint terms), in
    ec135609a.
- **Main's process-path plan already depends on this one.** PP10 adopts
  collisionkit once CL2–CL4 are on main, and `ArmClearance` serves its
  clearance interface until then.
- **CL1 stays off main until CL2.** Landing it alone would make every
  native kit build on main compile coal, for a design CL2 replaces.
- **CL-D7 revised:** the planner is Haxe for integration, not for reach.
  Collision is native everywhere it is wanted. The hot path is native
  (batched checks, added to CL2), and the OMPL benchmark decides whether
  any planner code moves to C++.
- **CL3 gains** convex decomposition and one collision description shared
  with SimKit's MuJoCo backend, with a cross-check test.
- **Checks after the rebase:**
  - the standalone C++ tests pass;
  - the native kit suite passes (713 assertions);
  - MotionKit passes (1224712 assertions), and so does RobotKit (5433):
    both now compile coal through `kinematicskit-native`.
  - The pure kit suite fails "DLS allocates nothing per iteration
    (-40.8 bytes)". A clean export of `origin/main` fails identically, so
    this is main's, not this branch's.

### Decisions on CL3 (2026-10-09)

- CL-D10: RobotKit's contact settings are simulation-only; contact pairs
  become allow rules with their reason.
- CL-D11: convex decomposition is V-HACD 4 from a `tritao` fork, with
  enclosure measured and fixed by inflation; CoACD is rejected because
  its build needs Boost and OpenVDB (checked in its CMakeLists).
- CL3 splits into CL3a (geometry, mapping, decomposition) and CL3b (the
  description shared with SimKit).
- CL4 and CL6 start with a reconciliation against main's clearance code,
  which arrived with the rebase.

### Reconciliation with main's clearance code (2026-10-09)

- **Main already has CL-D5 and most of CL6 for process paths:**
  - `ClearanceMotionEnvelope` and `JointCurveClearance` give the bound and
    the proof;
  - `LazyCollisionLadder` gives the pruning.

  What main lacks is the geometry engine, and that is what collisionkit
  provides. CL-D12 records the rule: main's design stays, and collisionkit
  replaces `ArmClearance`'s geometry behind an extracted interface.
- **Step changes:**
  - CL2 gains inflated, batched violation queries.
  - CL4 splits:
    - CL4a, the interface and adapter, with parity tests;
    - CL4b, the report entry, and conservative checks for joint moves and
      generated moves.

    A conservative check that does not close never throws where the
    sampled one passes.
  - CL6 shrinks to an optional clearance cost.
  - CL7 notes PP7's entry hook.
- **V-HACD** is an upstream submodule at `v4.1.0`, not a fork (CL-D11).
- **The OMPL benchmark** builds OMPL out of tree against the system Boost.
- **"Order and gates"** records the working order and the rules for
  unattended work.

### CL2 — collisionkit (2026-10-09)

Done as planned, with these choices:
- **Package.** `collisionkit/`:
  - pure half (`collisionkit`): `CollisionWorld`, `CollisionGeometry`,
    `CollisionPose`, `CollisionPair`, `CollisionDistance`,
    `CollisionViolation`, `CollisionMargins`, `CollisionPairStatus`,
    `CollisionPairRule`; no dependencies;
  - native half (`collisionkit-native`): the C ABI `ck_*` in
    `native/include/collisionkit.h`, `NativeCollisionWorld`, coal moved
    to `native/vendor/coal` with its notes (`native/THIRD_PARTY.md`). The
    submodule keeps its old name in `.gitmodules`; only its path moved.
  - kinematicskit-native is back to main's: no coal, no collision code, no
    `KK_ERROR_UNSUPPORTED`. Main's standalone coal smoke test moved with
    coal.
- **Bodies.** The world's own, numbered from 0, never removed; body -1 is
  the world. Each has a group (CL-D4's pair classes) and can be marked
  static. `ck_set_body_poses` poses a run of bodies from seven doubles
  each. The world keeps no model and no FK.
- **Statuses,** in precedence order: an object rule; a body rule (an allow
  rule records its reason: rigid, adjacent, closure, declared, process
  contact, declared contact); static (both bodies static); rigid (one
  body); overlapping at reference; checked. Unsupported pairs still make
  queries fail.
- **Overlap at reference** takes the bodies of one articulation and marks
  only pairs with both bodies in it, replacing earlier marks among them.
  Re-attaching an object drops its marks.
- **Inflation** (`ck_set_inflation`) is coal's swept-sphere radius on
  primitives and convex sets; meshes and height fields refuse it
  (`CK_ERROR_UNSUPPORTED`). Distances to inflated shapes are exact to
  1e-9 in the tests (sphere against box, convex block and capsule).
- **Violation queries** (CL-D12): margins come with each query as a square
  table over body groups (row ≤ column read), plus an optional inflation
  per body, so `required = margin(ga, gb) + δa + δb`. `ck_violation`
  returns the first checked pair in id order (signed distance and
  required); `ck_closest` the closest checked pair;
  `ck_violation_batch` checks many pose sets (every body's pose per set,
  with per-set inflation) in one call and returns the first failing set.
- **Results name bodies:** pair rows are four ints (objects a and b,
  bodies a and b). `ck_pair_distances` measures chosen pairs whatever
  their status, each in the order given.
- **Heights** below a field's minimum are refused, at creation and on
  update (coal would clamp them).
- **Kit pairs:** `kinematicskit.KinematicBodyPairs.of(model)` gives every
  rigid, adjacent and closure body pair with its relation
  (`BodyPairRelation`); `rigidGroups` the fixed-joint groups. The caller
  maps them to body rules.
- **Tests:**
  - `native/tests/cpp/collision_world.cpp` (standalone, CTest; CL1's carried
    over, posed by hand-computed arm poses): rules with reasons, static
    bodies, a held part following the tool's rules, overlap at reference
    within the arm with the environment overlap still reported, inflation,
    violations with group margins and inflation, the closest pair, batched
    sets, chosen pairs, refused heights, arguments.
  - `native/tests` (Haxe, with the pure kit as a test dependency): kit body
    pairs and their rules; distances following the kit snapshot to 1e-9 on
    random trees (CL1's); statuses, grasping, terrain (CL1's, posed from
    the snapshot); two models in one world; inflation against exact
    distances; violations and batches from a snapshot sweep; refused
    heights.
  - CL1's Haxe test marked terrain overlapping the arm at reference as
    allowed; under CL-D3 it is a layout error, so the carried-over test now
    expects it to stay checked. That test was CL1's own (never on main).
- **The pure kit suite** is unchanged. A body-pair test placed in it,
  anywhere before the allocation test, moved main's known allocation
  failure from DLS (-40.8 bytes) to LM (+40.96 bytes): the measurement
  depends on heap state. So the kit's pairs are tested in collisionkit's
  suite instead, and the pure kit keeps failing exactly as main does.
- **Suites:**
  - standalone C++ (CTest, `collisionkit/native`): 2 of 2 pass
    (`ck_coal_smoke`, `ck_collision_world`);
  - collisionkit native: passed (350 assertions);
  - native kit: passed (695 assertions: main's, CL1's 18 moved out);
  - pure kit: only main's known "DLS allocates nothing per iteration
    (-40.8 bytes)";
  - MotionKit: passed (1224712 assertions), no longer compiling coal;
  - RobotKit: passed (5433 assertions).

### CL2b — coal: height-field distance (2026-10-09)

- **In our coal fork** (branch `materia`, local commit 85cb6397, not
  pushed; the upstream PR waits for the user):
  - `HeightFieldShapeDistancer` is implemented. The height field's BV tree
    is traversed with lower bounds from each subtree's cell bounds against
    the shape's bounds, both in the field's frame (coal's old traversal
    node ignored the field's transform). Leaves are the two prisms of
    `buildConvexTriangles`, so distances are signed, with closest points
    and the normal from the field toward the shape. With signed distances
    asked for, overlapping bounds give no lower bound, so every cell they
    cover is compared.
  - A mesh against a height field (every mesh BV type, both argument
    orders) goes through the same prisms, convex against mesh.
  - Bounds use AABBs computed from the shape (`computeBV<AABB>`), which
    accept a swept-sphere radius; coal's generic OBBRSS fitting throws on
    one.
  - Tests: `test/hfield_distance.cpp`, registered in coal's
    `test/CMakeLists.txt`; built standalone here (Boost.Test header-only,
    against `coal_core`): 4 cases, 210 of 210 assertions pass. A box,
    sphere, capsule, convex and mesh against flat and sloped fields under
    an arbitrary placement, clear and penetrating, both argument orders,
    closest points on the field's surface, a swept-sphere radius,
    agreement with collision, and a shape beside the field.
- **collisionkit** uses coal's distance for every height-field pair; the
  cell-prism code (`field_distance`) is deleted. A height-field pair is
  checked by distance (coal has no height-field collision against a mesh),
  so support is decided by coal's distance table. CL2's tests pass
  unchanged.
- **Pin:** the parent pins coal at the local 85cb6397. A clone without this
  worktree's submodule objects cannot check it out until the fork is
  pushed.
- **Suites:** standalone C++ 2 of 2; collisionkit native passed (350
  assertions); native kit passed (695); pure kit only main's known DLS
  failure; MotionKit passed (1224712); RobotKit passed (5433).

### Merge onto local main (2026-10-09)

Local `main` was fast-forwarded to CL2b (5ef09380b) in the shared checkout,
without recursing into submodules; nothing was pushed. The shared checkout
keeps an untracked `kinematicskit/native/vendor/coal` directory from
before the move, and its `collisionkit/native/vendor/coal` is not checked
out (the pinned coal commit is local to this worktree).

### CL3a — Geometry from the cell (2026-10-09)

Done as planned, with these choices:
- **A description first.** `collisionkit.CollisionDescription` holds the
  cell as data: bodies (group, fixed, pose), objects (geometry, inflation,
  what they approximate), body and object rules with reasons,
  articulations with their reference, and the assumptions (CL-D8). Its
  `build(world)` adds everything, marks each articulation's overlaps at
  its reference, and returns the ids and the layout errors (pairs
  colliding at the reference between an articulation and anything
  outside it). CL3b feeds the same description to SimKit.
- **Models.** `collisionkit.kinematics.ModelBodies` puts a
  `KinematicModel` in a description: one body per model body, the kit's
  rigid, adjacent and closure pairs as rules, bodies rigid to a root fixed
  when the model is bolted down, posed at the reference; followers (a
  tool in its own group) take their body's rules. `place` and `poses`
  pose them from a state; `state(q)` keeps the cell placement. This gives
  collisionkit's pure half a dependency on the pure kit (kinematicskit
  still does not depend on collisionkit).
- **RobotKit** (`robotkit.collision.RobotCollision`, robotkit-autonomy):
  link primitives, link hulls (simulation or CAD-sourced), the tool on a
  follower body in the tool group (box, cylinder, or hulls inflated by
  their padding), contact pairs as "declared contact" rules, and
  `ShapeContact`, the self-collision opt-out and mesh references recorded
  as ignored (CL-D10). RobotModel has no home configuration, so the
  reference is the caller's or zero within limits, and its name is
  recorded. `describeClearanceBodies` adds `ArmClearance`'s inputs
  (`ClearanceBodyData`: link hulls, tool hulls, parts and weld beads on
  links) to the same bodies, for CL4a.
- **CAD** (`cadbridge.CadCollision`): mechanisms through
  `AssemblySimulationBridge.toRobotModel` and the robot builder, with the
  CAD link hulls; parts and fixtures at their world pose as an enclosing
  hull, a surface mesh, or decomposed.
- **Terrain** (`processkit.collision.HeightMapCollision`): a `HeightMap`
  becomes a height field (rows flipped to coal's layout, centred on the
  map) down to a floor below the deepest dig; `heights` gives updates.
- **Scene boxes:** `CollisionDescription.addFixed`; the app's scene records
  are not wired here (that is the editor's, CL8).
- **Decomposition** is in collisionkit's native library, not cadbridge or
  CadKit: cadbridge has no native code, CadKit's would tie it to OCCT, and
  a mesh is all it needs. `ck_decompose` runs V-HACD 4 (submodule at
  v4.1.0, synchronous) and measures enclosure: exact point-to-polytope
  distances at samples over every triangle no farther apart than a spacing
  (0.5 % of the bounding diagonal by default); `inflation = measured +
  spacing`, since the distance to the union is 1-Lipschitz. The pure
  `ConvexDecomposer` interface keeps cadbridge off the native library;
  `NativeConvexDecomposer` implements it. `CadDecompositionCache` keeps
  the pieces per part, keyed by the tessellation's contents.
- **Tests:**
  - `collisionkit/native/tests/cpp/decomposition.cpp` (CTest): an
    L-shaped prism splits into 4 pieces; inflated by 12.8 mm, every one of
    random surface samples lies inside them (checked through the world's
    distances); a box is one piece; arguments.
  - `robotkit/cadbridge/tests/collision` (new, 30 assertions): one world
    with a robot (primitives, a hull, finger pads, a box tool), a CAD
    slider posed from its own model and terrain from a height map, with
    distances following both models and a dig; a fixture through the
    forearm reported as a layout error and still checked; the CL-D10
    mapping (a `PairsOnly` pad checked against a fixture, the contact pair
    allowed as declared contact, the settings and the self-collision
    opt-out recorded); a fused L-shaped CAD bracket decomposed into 5
    pieces inflated by 1.0 mm (measured 0.29 mm, spacing 0.71 mm) that
    enclose its tessellation, decomposed once through the cache; and
    `ArmClearance`'s inputs landing on the right bodies.
- **Not here:** URDF `<mesh>` loading (references are recorded); the
  tessellation deflection is recorded, not added to the inflation.
- **Suites:** standalone C++ 3 of 3 (`ck_coal_smoke`, `ck_collision_world`,
  `ck_decomposition`); collisionkit native passed (350); cell tests passed
  (30); native kit passed (695); pure kit only main's known DLS failure;
  MotionKit passed (1224712); RobotKit passed (5433).

### CL4a — The clearance interface and the collisionkit adapter (2026-10-09)

- **Interface:** `robotkit.manipulation.ClearanceWorld`, extracted from what
  callers use of `ArmClearance`: `violation` (with per-body displacement),
  `closest`, `sweep`, `closestSweep`, `motionEnvelope`,
  `displacementBounds`, `tcpDisplacementBound`, `auditDisplacement`,
  `withGroup`, `pairCount`, `bodyCount`, `movingNames`, and `group()` for
  the arm. haxeon interfaces take no fields and no default values, so the
  arm is a method (`JointCurveClearance` reads `world.group()` instead of
  `world.arm`), the interface's optional arguments carry no defaults, and
  `ArmClearance.withGroup` returns the interface.
- **`ClearanceViolation`** moved out of `ArmClearance`'s module into its
  own, beside the interface, in robotkit-autonomy. Not out of robotkit:
  the interface needs `KinematicGroup` and `ClearanceMotionEnvelope`, which
  are robotkit's, and `ArmClearance` must implement it without robotkit
  depending on MotionKit.
- **Callers take the interface:** `StructuredJointPathPlanner`,
  `JointCurveClearance`, `LazyCollisionLadder`, `TrajectoryClearance`, the
  path planner interfaces, and processkit's weld and probe planners and
  runners. Types only; nothing else changed in them. Constructors stay
  `ArmClearance` (app, examples, tests), and process planning is not
  switched over (PP10).
- **Adapter:** `robotkit.collision.CollisionClearance` takes
  `ArmClearance`'s inputs and a world factory. It decides the checked
  pairs exactly as `ArmClearance` does (one link; both fixed; neighbouring
  or drive-coupled links touching within 2 mm at the reference, with the
  same `ConvexDistance` test), so the parity tests compare engines, not
  rules. Each hull is a convex object on its own collisionkit body, since
  the proof inflates per hull; the four groups (moving, moving tool,
  fixed, fixed tool) carry the margins, with `contactMargin` between tool
  and fixed groups under `contact`. Displacements are the world's per-body
  inflation, sweeps without a per-sample contact policy are one batched
  call, and the motion envelope uses the same hull corners. It uses
  `ArmClearance`'s body data rather than CL3a's description: the
  description puts a link's hulls on one body, and the proof inflates each
  hull by its own bound. `RobotCollision.describeClearanceBodies` maps the
  same data onto a cell description for later use.
- **coal fix (fork, local commit 7b57ae92):** the rail and G17 scenes have
  hulls of up to 64 points. coal's GJK dispatch picked its hill-climbing
  support for convex sets over 32 points even without a neighbour graph,
  and crashed on our point-only hulls (CL1–CL3 tests used at most 8
  points). Its generic support function already checked for neighbours;
  the GJK and contact-patch dispatch now do too, with a coal test
  (`test/convex_without_neighbors.cpp`: crashes without the fix, 5 of 5
  with it) and a 60-point convex in collisionkit's C++ test. A fork fix
  per CL-D13, upstreamable, not pushed.
- **Parity** (`machinekit/tests/clearance-parity`, 4184 assertions):
  - main's weld cell, open and with the wall: 301 configurations, 1244
    violation queries each (plain, contact, and inflated by the
    displacement bounds), the closest pairs within 1.9e-7 m, 40 sweeps and
    40 straight-path `JointCurveClearance` proofs (23 and 18 certified)
    identical; the weld planner with the adapter plans the same weld
    ("along the wire", roll 0, 96 poses; beside the wall "along the wire
    from twice as far", roll pi/4, 1802 poses);
  - the rail (3 m linear track with the arm) and the G17 track welder
    (arm, torch, 2.6 m weldment and table): 201 configurations each, the
    closest pairs within 1.1e-11 m, sweeps and 40 proofs each (39 and 23
    certified) identical;
  - no disagreement at all, so none needed explaining; the suite would
    accept and print boundary cases (a distance within 1e-6 m of its
    required clearance) and fail on any other.
  - Not run: the full G17 track-weld plan with the adapter (a 15 s to
    300 s benchmark of main's process-path work). Its scene, sweeps and
    proofs are covered above; the whole plan waits for PP10's switch-over.
- **Defaults through the interface:** haxeon fills a default argument at
  the call site from the static type, and interfaces cannot declare
  defaults. The first MotionKit run failed ("A clearance sweep needs
  matching joint values and a positive step"): a sweep called through
  `ClearanceWorld` passed null for the 0.02 step. `ArmClearance` and the
  adapter now take those arguments as optional and apply the same
  defaults in the body (contact false, step 0.02, endpoints unchecked), so
  every caller gets what it got before.
- **Suites:** standalone C++ 3 of 3; collisionkit native passed (350);
  cell tests passed (30); native kit passed (695); pure kit only main's
  known DLS failure; weld planning passed (609, weld pass path 19, the
  shared welding motion factory); parity passed (4184); MotionKit passed
  (1224712); RobotKit passed (5433); processkit's full suite passed; the
  app compiles. The weld planning and parity suites need SimKit's
  libraries on `LD_LIBRARY_PATH` (`robotkit/tests/build/host/native/
  robotkit-world-tests`), as their projects declare no native build.

### CL4b — Clearance in the validation report; conservative joint moves (2026-10-09)

- **The report** (`trajectory_core.h`) gains a planner-owned
  `collision` check and a `collision_pair` (object ids, names, the
  segment), with `mk_report_set_collision`; `mk_validate` leaves it
  unchecked. `ValidationReport` exposes `collision`, `collisionPair` and
  `setCollision`, `hasFailure` counts it, and `ValidationGuarantees` has a
  `collision` guarantee. The bindings are regenerated.
- **Not as `checks[MK_CHECK_COLLISION]`.** The plan had `MK_CHECK_COUNT`
  grow. `motionkit/native/tests/generator.cpp` asserts that `mk_validate`
  decides every entry of `checks` except task space, and a planner-owned
  collision entry there would stay unchecked and fail it; changing that
  assertion is against the gates. So the check sits beside `checks`, in the
  same report: its layout still changes and is versioned by `struct_size`
  as planned. Any binary built against the old header must be rebuilt (the
  weld planning suite first failed with "runtime.submitPlan failed with
  RobotKit status -10" until RobotKit's natives were rebuilt).
- **Object ids** are UINT32_MAX: `ClearanceWorld` names bodies only (its
  violations are `ArmClearance`'s, by name).
- **`ProgramCompiler.finish`** records, after the plan is created:
  - a refined process path certified by `JointCurveClearance`: a bound,
    with the closest pair over 8 times along the motion;
  - any other motion (a joint move, a generated entry, exit or hold):
    `TrajectoryClearanceProof`. Over a time interval of half-length h, joint
    j moves at most `v_j·h` from the midpoint, where `v_j` bounds its speed
    from the motion's own polynomials (Σ k|c_k|d^(k-1) per segment); the
    world's displacement bounds turn that into per-body inflation, and the
    interval is clear when the midpoint is, inflated; otherwise it is
    bisected, to `clearanceDepthLimit` (16). Every midpoint is also checked
    as it is. A bound that closes is recorded as a bound; one that does not
    is recorded as sampled (the sampled check passed), naming the first
    open interval, and does not throw; a real violation it finds between
    samples is recorded and throws.
  - The sampled checks that already throw (`TrajectoryClearance`, the
    refined-curve proof) still throw before a plan, and so a report,
    exists; their messages name the pair.
- **Contact policy:** with a per-configuration contact policy the bound
  uses the planner's neighbourhood guard (as `JointCurveClearance` does);
  without one it uses the stricter margins.
- **Changes (CL-D9):** `ClearanceChange` (attach, detach, open and close a
  window) and `ClearanceScene` in robotkit; `ClearanceEvent` (op, time,
  change) and `ProgramCompiler.clearanceEvents` in MotionKit. The proof
  replays an op's events in time order through the world, never bisecting
  across one; the scene resets when a program starts and carries across
  ops. `CollisionClearance` is a scene: attaching moves a hull onto a link
  where it is (rigid with that link's hulls), detaching onto the arm's
  root, and a window gives its pair two groups of their own, with the
  window's margin between them, or allows the pair (process contact) when
  the margin is null. An op with events skips the sampled check, which
  cannot replay them; the proof checks every midpoint as it is instead.
  `ArmClearance` cannot replay changes, and events with it are refused.
- **Tests** (`motionkit/tests/clearance-validation`, 20 assertions, on
  main's weld arm):
  - a clear joint move recorded as a bound with its closest pair
    (neck/table 0.552 m against 0.005 m), unchecked without a world, and a
    move into the table refused naming the pair;
  - a 1 mm bead on the outstretched arm (0.90 m out) swept across a 0.5 mm
    plate placed in the widest gap of the sampled check's own samples
    (0.0132 rad apart): the sampled check passes and the bound finds the
    contact at 0.754 s;
  - with the depth limit at 0, the same clear move is recorded as sampled
    over its open interval;
  - a part grasped at the torch tip (overlapping it), carried and placed,
    with the torch leaving it through a contact window: refused without
    the events, clear with them;
  - a torch ending 2 mm from its seam: refused under the 5 mm margin,
    clear inside a window with a 1 mm approach margin (recorded as a bound,
    1.98 mm against 1 mm).
- **Suites:** collisionkit standalone C++ 3 of 3; MotionKit standalone
  C++ 16 of 16 (the generator assertion untouched); collisionkit native
  passed (350); cell tests passed (30); native kit passed (695); pure kit
  only main's known DLS failure; RobotKit passed (5433); weld planning
  passed (609, weld pass path 19, the shared welding motion factory);
  processkit's full suite passed; parity passed (4184); clearance
  validation passed (20); MotionKit passed (1224712).

### CL5 — Avoidance (2026-10-09)

- **Kit** (pure): `AvoidancePair` (bodies of the solved model or -1, closest
  points, normal from a to b, distance, ds, di) and `AvoidanceRows`: each
  pair within di gives `n·(J_b − J_a)·Δ ≥ −ξ·(d − ds)/(di − ds)·dt` from
  the snapshot's point Jacobians at the closest points over the layout's
  DOF columns. A moving root's columns get no row entries yet.
- **Native QP:** `kk_qp_set_rows` keeps general rows on the QP handle and
  `kk_qp_solve` honours them (none set: unchanged); `kk_qp_relaxed` reads
  which rows needed relaxing. haxeon's FFI takes at most 16 arguments per
  C call, which ruled out one solve call carrying the rows. With rows, the
  QP first solves them hard (ProxQP inequality rows); if rows and limits
  admit no step, every row gets a slack s ≥ 0 penalized at 10^6 times the
  Hessian's largest diagonal, and the rows whose slack is in use are
  reported (mink relaxes its collision rows similarly).
- **`DifferentialIk.step`** takes `avoidance` pairs and `xi`, and its step
  reports the pairs that gave rows and those relaxed; if the QP fails with
  rows the step is zero, never an unchecked damped step.
- **`ManipulatorServo`** gains an `avoidance` provider (pairs at the state
  of the group's model) and `avoidanceSpeed`, and its step reports avoided
  and relaxed pairs.
- **From the world:** `ModelBodies.avoidancePairs` turns a built world's
  distances into pairs for that model (followers such as the tool map to
  their model body; other articulations and the environment are -1).
- `PrioritizedSolver` collision rows stay for later, as planned.
- **Tests:**
  - collisionkit native (14 new assertions, 364 in all): a two-link arm
    reaching behind a wall goes 99 mm into it without rows and stops at
    the 10 mm margin with them; an arm reaching for another robot's arm
    stops short of it (that side has no Jacobian); starting 4 mm from a
    wall, holding its place, the arm backs out to the margin (a bent pose:
    stretched out the tip cannot move along the wall's normal); rows that
    zero velocity limits cannot honour are relaxed and named while the
    limits hold exactly, and rows that can hold are not;
  - clearance validation (3 new assertions): `ManipulatorServo` jogging
    main's weld arm down and sideways stops the torch 5.005 mm above the
    table (margin 5 mm) and slides 0.75 m along it.
- **Suites:** collisionkit standalone C++ 3 of 3; MotionKit standalone C++
  16 of 16; collisionkit native passed (364); cell tests passed (30); native
  kit passed (695); pure kit only main's known DLS failure; RobotKit
  passed (5433); weld planning passed (609); processkit's full suite
  passed; parity passed (4184); clearance validation passed (23);
  MotionKit passed (1224712).

### CL3b — One collision description shared with SimKit (2026-10-09)

- **SimKit:** `nksim_world_set_exclusions` declares exactly which pairs of
  an articulation's bodies never collide; the MuJoCo backend then skips its
  own rule (parent and child, or overlapping at rest) for pairs within the
  declared set, and keeps it for every other pair. It is C-only (behind
  `NKSIM_HAXEON_IMPORT`, as SimKit's other C-only calls): handle arrays do
  not cross haxeon's FFI, and RobotKit's runtime is the caller. The
  committed SimKit bindings were already behind main's header in unrelated
  places; they are left as they are.
- **RobotKit:** `rk_simulation_robot_desc` gains an appended tail
  (`link_exclude_count`, `link_excludes`, up to 4096 pairs) and the flag
  `RK_SIMULATION_ROBOT_EXPLICIT_EXCLUDES`; the runtime passes them to SimKit
  for the robot's link bodies. `Simulation.addRobotAtPose` takes
  `linkExcludes` (link index pairs). Without it nothing changes.
- **From the description:** `RobotCollision.simulationExcludes(description,
  build, robot)` gives the link pairs to exclude: those with some object
  pair allowed in the built world (or carrying no objects), the tool
  counting as its flange link. Excludes are per link and rules per object,
  so a link pair with one allowed object pair is excluded whole: the
  simulation may exclude more, never fewer (CL-D10).
  `RobotCollision.simulationHulls` gives the description's convex objects
  on links (link hulls, CAD hulls, decomposed pieces) as the simulation's
  link hulls, without inflation (MuJoCo has no swept-sphere radius).
- **Test** (`robotkit/tests/collision-mujoco`, MuJoCo backend, 59
  assertions): an arm with link boxes and a hull, and two environment
  boxes, in one description. Folded back, the forearm reaches over the
  base: SimKit's own rule excludes that pair (its bounding-radius test
  calls them overlapping at rest), while the description checks it; with
  the shared excludes MuJoCo reports the contact, and coal does too. Over
  60 sampled configurations (a fresh simulation each, one step), every one
  of 52 MuJoCo contacts is a coal collision.
- **Worktree setup:** the MuJoCo native build (`robotd/native-mujoco`)
  needs haxeon's nested NativeKit submodules `libwebsockets` and `sokol`;
  they were cloned from the main checkout.
- **Not here:** RobotKit's other simulation paths (the app's
  `AssemblyRobot`, `MissionPlayer`) do not yet build a description; they
  keep SimKit's own rule until they do.
- **Suites:** collisionkit standalone C++ 3 of 3; MotionKit standalone C++
  16 of 16; collisionkit native passed (364); cell tests passed (30); native
  kit passed (695); pure kit only main's known DLS failure; RobotKit passed
  (5433); weld planning passed (609); processkit's full suite passed;
  parity passed (4184); clearance validation passed (23); the new MuJoCo
  cross-check passed (59); CadBridge's MuJoCo cup and vacuum test passed
  (the backend changed under it); MotionKit passed (1224712).

### CL7 — Free-space planning (2026-10-09)

- **Interfaces** (MotionKit, `motionkit.planner`): `MotionPlanner`
  (`plan(start, goal)`), `MotionPlan` (waypoints or the failure, with
  iterations, nodes, edge checks, time in checks and in all),
  `PlannerSpace` (bounds, `invalid(q)` naming what collides, and batched
  `edgesClear`).
- **RRT-Connect** (`RrtConnect`, pure Haxe): seeded (xorshift), extend by at
  most `step`, connect by checking all of a connection's straight steps in
  one batch and keeping the clear prefix, then shortcutting in batches of
  eight. Trees store nodes flat (`PlannerTree`: one coordinate array, one
  parent array) and find neighbours with a k-d tree rebuilt as the tree
  doubles past 64 nodes, plus a scan of the nodes added since.
- **The space** (`motionkit.robot.CollisionPlannerSpace`): a model's
  bodies in a collision description. An edge is clear when its midpoint is
  clear with each body inflated by its bound over half the edge's per-joint
  travel; otherwise it is halved to `depthLimit` (14), every level for every
  pending edge in one native call (`ck_violation_sets`, new: a flag per pose
  set), plus one for the midpoints as they are. The bound is
  `kinematicskit.MotionReach`: per body and DOF, the offsets from each
  joint to the body (every joint on the way, with a prismatic joint's
  largest travel) and the revolute joints' share of the body's radius
  (`CollisionDescription.bodyRadius`), configuration-independent. It is
  looser than main's `ClearanceMotionEnvelope` over a joint box, but needs
  only the kit's model, which is what the OMPL benchmark's checker shares.
- **Phase synchronization:** `MK_SYNCHRONIZATION_PHASE` in MotionKit's
  generator (Ruckig's phase synchronization), and
  `Trajectory.generateStateToState(phase)`: a rest-to-rest edge is a
  straight line in joint space.
- **Planned joint moves:** `MotionOptions.planned` and
  `ProgramCompiler.motionPlanner`. A planned MoveJ is the planner's
  waypoints, each edge timed rest to rest with phase synchronization and
  joined into the op's one motion, which validation (CL4b) then checks
  again. Not in motor space yet. PP7's blocked process entry can use the
  same planner; that wiring is with the process-path work.
- **Tests** (`motionkit/tests/planner`, 12 assertions):
  - a 10 cm ball on x and y slides through a 16 cm gap in a wall (the
    straight way is blocked): found, every edge rechecked clear, crossing
    in the gap (5 waypoints, 60 iterations, 3.3 ms, 98 % in checks);
  - a start or goal inside the wall is named ("start in collision:
    ball / wall-high ..."), not planned;
  - the same seed plans the same path, another seed another;
  - main's weld arm turning 1.2 rad past a post its torch would sweep
    through (the direct sweep collides): a planned MoveJ of 3 edges whose
    timed path stays on the planned edges (within 1e-6 rad at 200 times) and
    passes validation as a bound.
- **OMPL benchmark** (out of tree, nothing of it in the repo): OMPL 1.7.0
  built from source into a scratch prefix against the system Boost 1.83 in
  1 m 43 s. A C++ harness rebuilds each scene `PlannerTests` exports (with
  `PLANNER_BENCH_DIR`): the same packed kit model (native FK), collisionkit
  world, margins and edge check (the same `MotionReach` bisection over
  batched calls), so OMPL's RRT-Connect (same range) uses our checker;
  OMPL's partial shortcutting runs afterwards with as many rounds. 20 runs
  per scene, medians:

  | Scene | Planner | Success | Time | Path length | Edge checks | In checks |
  | --- | --- | --- | --- | --- | --- | --- |
  | maze (2 DOF, narrow gap) | ours | 20/20 | 2.92 ms | 2.082 | 368 | 96.2 % |
  | maze | OMPL | 20/20 | 1.24 ms | 1.985 (raw 2.761) | 147 | 27.6 % |
  | weld arm and post (6 DOF) | ours | 20/20 | 4.66 ms | 1.483 | 108 | 99.2 % |
  | weld arm and post | OMPL | 20/20 | 2.20 ms | 1.371 (raw 2.012) | 61 | 61.0 % |

  **CL-D7's decision from it:** our planner spends 1–4 % of its time
  outside collision checks, so its Haxe tree code is not a meaningful share,
  and none of it moves to C++. OMPL is about twice as fast on these scenes
  because it checks fewer edges (it connects one step at a time and stops
  at the first blocked step, where we check a whole connection in one
  batch) and each check costs less (FK in C++; ours runs the kit's FK in
  Haxe and copies pose arrays across the FFI, about 7.6 µs an edge against
  2.3 µs). If planning time matters, those are the two places: checking a
  connection's steps lazily, and native FK in the check (or a native
  `PlannerSpace`). Path lengths are within 8 %.
- **Suites:** collisionkit standalone C++ 3 of 3; MotionKit standalone C++
  16 of 16; collisionkit native passed (364); cell tests passed (30); native
  kit passed (695); pure kit only main's known DLS failure; RobotKit passed
  (5433); weld planning passed (609); processkit's full suite passed;
  parity passed (4184); clearance validation passed (23); planner passed
  (12); the MuJoCo cross-check passed (59); MotionKit passed (1224712).
- **The order's eight steps are done** (CL2, the merge, CL3a, CL4a, CL4b,
  CL5, CL3b, CL7). CL6's remainder, main's PP10 and CL8 wait for review, as
  "Order and gates" says. Nothing is pushed; the coal fork's commits
  (85cb6397, 7b57ae92) are local.
- **Local `main`** stays at CL2b (5ef09380b), where the order merged it.
  Fast-forwarding it to CL7 would overwrite uncommitted edits another
  session has in the shared checkout (`app/src/MissionPlayer.hx`,
  `robotkit/sim/haxe/robotkit/runtime/Simulation.hx`, both also changed
  here), so the branch `collision` holds CL3a to CL7 until that is sorted.

### PP10: process planning on collisionkit (2026-10-09)

- **Local `main` and the pushes.** Local `main` was fast-forwarded to CL7
  (f3da28dea) once the other session's two files were set aside and
  restored; the coal fork (`tritao/coal` `materia`, 7b57ae92) and then the
  parent (`tritao/materia` `main`, f3da28dea) were pushed with the user's
  permission. The upstream coal PR still waits for the user.
- **`ArmClearance` is deleted.** Every user builds
  `robotkit.collision.CollisionClearance` with a `NativeCollisionWorld`
  factory: the app's mission player, the robot-welder and machine-tending
  examples, MachineKit's track and physical checks, and the RobotKit,
  MotionKit and ProcessKit tests. `ClearanceBodyData` has its own module in
  `robotkit.manipulation`; `MARGIN`, `CONTACT_MARGIN` and `TOUCH` live on
  `CollisionClearance`. Projects that construct the world depend on
  `collisionkit-native`, and the web host links `collisionkit_core`.
- **The parity suite is retired** with the class it compared against. Its
  last run (above) was exact on the weld cell, rail and G17 scenes.
- **Two behaviours changed with the switch:**
  - A clearance with no checked pairs returns from `sweep` at once (the
    native batch refuses a world without bodies).
  - `sweep` asks the world in one batch, so it no longer calls `violation`
    per sample. One test fake (`EntryTrajectoryClearance` in
    `WeldPlanningTests`) overrode only `violation`, so it now samples its own
    sweep. No assertion changed.
- **Benchmarks** (PP8's runner, same host, back to back; not quiet-host
  timing). Every quality record is identical to the `ArmClearance` build of
  f3da28dea:

| Case (one run each) | `ArmClearance` at f3da28dea | collisionkit | Quality |
|---|---:|---:|---|
| G17 track weld, MuJoCo | 9.177 s | 7.475 s | identical: 232.5 s cycle, 0.778094423 rad margin, 4.998 mm leg |
| Robot welder, MuJoCo, runs 1–4 | 1.578 / 1.910 / 4.645 / 9.400 s | 0.720 / 0.735 / 1.413 / 2.072 s | identical: ten seams, 99.9 s |
| Robot welder, test backend, runs 1–4 | 1.699 / 1.776 / 4.596 / 9.318 s | 0.774 / 0.620 / 1.555 / 2.146 s | identical: ten seams, 99.9 s |
| Gantry welder, MuJoCo, runs 1–4 | 27.799 / 104.381 / 89.996 / 261.273 s | 4.094 / 14.529 / 11.330 / 33.406 s | identical: ten seams, 159.8 s |

  G17's continuous geometry certificate takes 0.168 s over six sections
  (`PROCESS_PATH_GEOMETRY_CLEARANCE`). The gantry welder's 159.8 s cycle
  (171.3 s in the PP8 log) is the same on both builds, so it moved before
  this step.
- **Suites:** RobotKit (5433), cell (30), clearance validation (23), planner
  (12), MuJoCo cross-check (59), weld planning (609), processkit's full
  suite, contact-search motion (120), MotionKit (1224712), MachineKit's
  track and physical checks, the robot-welder, machine-tending, track-arm
  and gantry-welder examples, the app's mobile welder and mobile mission
  (both backends, 10 seams, clear stow), and the web build (433 guest
  imports bound).
- **Two older failures, fixed after PP10.** Both failed the same way on
  f3da28dea and stopped their suites early:
  - MachineKit's `CoreXyDriveTests` ("the step rate is each motor's
    ceiling"): the device rounded a step interval of 1.0000000029 ticks,
    f32 noise at the 40 kHz ceiling, up to two ticks, and the planning cap
    mirrors it (10824b00b);
  - the app suite's worker document tests ("Could not open source file"):
    the Start-page tests left the working directory at the repository root
    (57dddf9ba).
  With them fixed, MachineKit's suite ("MachineKit smoke passed") and the
  whole app suite pass with PP10.

### CL8a: the editor's collision world (2026-10-09)

- `app.SceneCollision` (CL-D14) describes the open scene in one
  collisionkit world:
  - a generated project's occurrences, with their pieces;
  - CL-D3's rigid and adjacent rules from `AssemblyBodies`;
  - the assembly as an articulation referenced at its as-designed state
    (default joint values);
  - every other collision-enabled object as a movable box, built as the
    simulation builds it (a CAD part's collision bounds).
- It rebuilds only when what collides changes (ids, kinds, extents, CAD
  bounds, the collision flag, the project). A move or a joint edit
  re-poses the bodies. `query(scene, project, near)` answers once per scene
  revision: one pair per two scene objects, closest first, with the closest
  points.
- `AssemblySimulationBridge.collisionPieces` is the one source of an
  occurrence's pieces: the component's authored hulls, else the measured
  hull. `toRobotModel`, `SimulationAssemblyParts` (now `pieces`, not
  `vertices`), the mission clearance (one body per piece), grounded weld
  work and the editor all use it, so the simulation and the robot model no
  longer disagree.
- **Tests:**
  - `SceneCollisionTests` (app suite): overlapping boxes collide 20 mm deep;
    5 mm apart they are near, with the closest points spanning the gap; 50 mm
    apart they are clear; moving never rebuilds; turning collision off on
    one rebuilds once.
  - `checkSceneCollision` (project-source, `arm` and the default run) on the
    robot-arm example:
    - nothing collides as designed, and the one near pair is the workpiece
      2 mm over the table;
    - j2 1.5 / j3 -0.78 puts the hand into the pedestal;
    - j2 2.18 / j3 2.5 puts the suction cup into the table;
    - back at zero, nothing collides;
    - one build for all of it.
- **Suites:** the app suite; CadBridge (173); project-source `arm`,
  `scene-collision` and `mobile-welder-mission` (both backends, 10 seams,
  clear stow); the PP8 runner's gantry and robot welders, whose quality
  records are identical to PP10's.

### CL8b: showing it (2026-10-09)

- While editing, the viewport tints objects in a colliding pair red and
  objects only near amber. These are `SceneView.setMaterial` overrides in
  each render view, nothing persistent; the selected and hovered objects
  keep their own highlight.
- It joins the closest points of the 16 closest pairs, with marks at both
  ends.
- The viewport toolbar says what was found, for example
  "1 collision, 1 near · Hand / Pedestal 31 mm deep".
  - **Change from the plan:** the distance is on that line, not beside each
    drawn line, because the viewport has no text drawing (`Canvas.drawText`
    needs a `TextLayout`, which no overlay uses yet).
- **Settings:** `editors/3d/collision/show` (on by default) and
  `editors/3d/collision/near_mm` (10 mm) are in the Editor Settings dialog.
  `scene.toggle-collisions` is in the viewport options menu.
- While a simulation runs nothing is shown (CL8c).
- **Cost:** the viewport repaints only when the query result changes. On the
  generated arm (38 objects), a drag step's joint edit plus query takes about
  0.7–0.9 ms after main's allocation work (rebased onto 1b952cd54). Before
  that work it was 37 ms for the edit and about 3 ms more for the query.
- **Tests (`SceneCollisionTests`):**
  - the tints: all four objects in a colliding pair and a near pair, the
    selected one left out, none without pairs;
  - the toolbar line;
  - the viewport's repaint key changes with new pairs and not with the same
    result;
  - in a headless editor: the setting off shows nothing, the command turns it
    back on, and a 0 mm near distance shows only collisions.
- **Suites:** the app suite; the project-source `scene-collision` check; the
  web build (438 guest imports bound).

