# Collision — checking, avoidance and free-space planning (plan)

Builds on KinematicsKit K6 (one group, one solver, one path search) and
KK-D20 (coal, vendored from `tritao/coal`, branch `materia`). Collision
checking had been planned as "a future plan" by MotionKit Lane D (D4) and
Lane C (OMPL listed as out of scope). This is that plan.

Revised on 2026-10-01, after CL1: the world moves out of the kit into its
own package, and three errors are fixed (see the revision log). The plan
moves to `collisionkit/plans/` with CL2.

## Where things stand (2026-10-01)

- **coal** is a submodule at `kinematicskit/native/vendor/coal`.
  - CL1 linked it into `kinematicskit_core`, so every package that builds
    kinematicskit-native compiles it (about a minute): `motionkit-robot`
    and its users, such as processkit, toolpathkit's motion package, the
    RobotKit suites and the robot-arm example.
  - The build has primitives, convex sets, BVH meshes, height fields and
    the broadphase managers. It has no octrees, no mesh-file loading and
    no qhull. Hulls come as point sets from CadKit's pure-Haxe
    `ConvexHullVertices`.
  - Found in CL1 and since:
    - no distance queries on height fields;
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

CL1's world is the only collision world. `ApplicationSimulation`
assembling a SimKit session is the closest thing to a cell-wide one.

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
- **CL-D5 — Path checks are conservative, not just sampled.** Between two
  checked configurations, no point on a body moves further than a bound
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
    comes later, and validation (CL4) re-checks whatever is timed.
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
  - MotionKit validation (CL4) consumes the declarations. Without them,
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
  - V-HACD 4.0: BSD-3, one header, no libraries. It is archived upstream
    (its README points to CoACD), so it is vendored as a `tritao` fork,
    like coal, and fixed there if needed.
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

## Steps

Each step is its own commit with all suites green.

- **CL0 — This plan.** Done (9ca6aed52); revised after CL1 and on
  2026-10-09.
- **CL1 — Native collision world in the kit.** Done (311788aa9), see the
  progress log. CL2 moves it out of the kit.
- **CL2 — collisionkit.**
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
  - Batched checks for CL6 and CL7 (CL-D7): one call takes many pose sets
    (a configuration list, or a segment's two ends with their motion
    bounds) and returns the first failure, with CL-D5's bisection done
    natively.
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
- **CL3 — Geometry from the cell**, in two commits. CL3a can run
  unattended; CL3b reaches into SimKit and is done with review.
- **CL3a — Robot, tool and environment geometry; decomposition.**
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
  - Convex decomposition (CL-D11): V-HACD 4 vendored from a `tritao` fork
    into cadbridge's native code (or CadKit's, wherever the tessellation
    is), the enclosure measurement and inflation, and the pieces cached
    with the part. Creating the fork publishes a repository, so it needs
    the user's go-ahead.
  - Out of scope here: loading URDF `<mesh>` files. They stay references
    until a mesh importer exists.
- **CL3b — One collision description shared with SimKit.**
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
- **CL4 — Validation.**
  - **First, reconcile with main's clearance code** (written up in this
    plan, no code). Main's process-path work added `TrajectoryClearance`,
    `JointCurveClearance` and `LazyCollisionLadder` (motionkit-robot),
    `JointPathClearanceProof` (motionkit planner) and `ArmClearance`
    (robotkit autonomy), and its PP10 adopts collisionkit after CL2–CL4.
    Decide what CL4 and CL6 reuse, replace or drop, and whether
    `JointPathClearanceProof` already is CL-D5's proof. CL4 and CL6
    below were written before that code existed.
  - Add `MK_CHECK_COLLISION` to `mk_validation_report`, with
    `mk_report_set_collision` and `ValidationReport.setCollision`.
  - A check's single `joint` slot cannot name a pair, so the report gains
    the pair (object ids and names) and the segment. `MK_CHECK_COUNT`
    grows, so the report's layout changes: it is versioned by
    `struct_size`, and the bindings are regenerated.
  - After `checkTaskSpace`, `ProgramCompiler.finish` checks the timed path
    with CL-D5:
    - CL-D9's declarations replay in time order;
    - each pair class has its margins (CL-D4);
    - contact windows apply.
  - On failure the report gives the time, the pair, the clearance and the
    margin, and compilation throws as it does for task space.
  - Exact or sampled as CL-D5 says; the approximations go into the
    assumptions (CL-D8).
  - Without a world, the check stays `UNCHECKED`.
  - Tests:
    - a passing and a failing cell;
    - an overshoot between samples caught;
    - bisection at its depth limit reported as sampled;
    - a grasp and a place replayed;
    - a torch window with its approach margin.
- **CL5 — Avoidance (Lane D D4).**
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
- **CL6 — The path search avoids collisions.** Rewrite after the
  reconciliation in CL4: main's process paths now use a ladder search
  (`StructuredLadder`, `LazyCollisionLadder`), and main's PP plan calls
  CL6 and its PP5 the same job.
  - `RedundancyResolver` and `PathConfigurationSelector` drop candidates
    that collide (solids per CL-D8).
  - Their edges are checked with CL-D5.
  - An optional clearance cost lets the search prefer room over minimal
    motion.
  - The search only prunes; CL4 stays the guarantee.
- **CL7 — Free-space planning (CL-D7).**
  - The `MotionPlanner` interface in MotionKit.
  - RRT-Connect with shortcutting in Haxe, over a `CollisionWorld`, with
    its own seeded random generator. Edges go through CL2's batched
    checks; nodes are stored flat; nearest-neighbour search is a k-d tree
    once trees grow (CL-D7).
  - Phase synchronization in MotionKit's generator, so a planned edge is a
    straight line in joint space.
  - `ProgramCompiler` plans a joint move that asks for it (a planned
    MoveJ), times each edge rest to rest, and validates the result (CL4).
  - Tests:
    - narrow passages;
    - start or goal in collision (named, not planned);
    - determinism from a seed;
    - the timed path following the checked edges;
    - a planned move through validation.
  - An out-of-tree benchmark against OMPL's RRT-Connect on the same
    scenes and checker: planning time, path length, success rate, and the
    share of time outside collision checks (CL-D7 decides from it whether
    any planner code moves to C++).
- **CL8 — Collision in the editor and the browser** (any time after CL3).
  - The desktop editor links collisionkit's native library and shows
    colliding and near pairs while a cell is laid out or a robot jogged.
  - The browser build compiles the library into its Emscripten host.

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
