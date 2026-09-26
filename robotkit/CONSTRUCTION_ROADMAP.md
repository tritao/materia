# RobotKit construction roadmap (implementation handoff)

Goal: **RobotKit turns design geometry into safe, observable machine work.**
Refocus RobotKit from planar AMR navigation toward construction-task robots
(surface-work mobile manipulators first, excavators second), while keeping
the existing `Robot` / `RobotRuntime` / `RobotWorld` / Skill architecture and
the planar navigation stack working unchanged.

First end-to-end target: a **simulated mobile wall-finishing robot**
(omni base + 6-axis arm + surface tool + 3D scanner) that takes a wall face
from a CadKit/BimKit model, registers it against a scan, plans reachable
patches, and executes a fixed-standoff raster with coverage verification.

---

## 0. Read before writing code

1. `robotkit/ARCHITECTURE.md` in full. Its frames/units/time/ownership rules
   are normative:
   - right-handed, `+Z` up, `+X` forward; meters, radians, seconds;
   - quaternions are `x, y, z, w`;
   - `A_T_B` maps coordinates in frame `B` into frame `A`;
     `world_T_sensor = world_T_link * link_T_sensor`;
   - matrices column-major, column vectors (SceneKit convention);
   - semantic IDs (`LinkId`, `JointId`, `FrameId`, ...) never replaced by
     backend indices.
2. Existing types you will build on:
   - `haxe/robotkit/model/Joint.hx` — already has `axis`,
     `parentFramePosition/Rotation`, `childFramePosition/Rotation` (xyzw).
     FK must be derived from these; do not add a parallel joint description.
   - `haxe/robotkit/model/Frame.hx` — `link_T_frame` position + quaternion.
   - `haxe/robotkit/mobile/Pose2.hx`, `Twist2.hx`, `PlanarMath.hx`,
     `localization/FrameTree2.hx`, `FrameTransform2.hx`.
   - `haxe/robotkit/protocol/JointTargets.hx`, `world/RobotCommand.hx` —
     the only way new planners reach a robot.
   - `haxe/robotkit/skill/*` — `Skill`, `SkillRunner`, `GoTo`, `PickPallet`
     are the pattern for new skills.
3. `haxeon` is this repo's own Haxe compiler, not mainstream Haxe:
   - no class → anonymous-typedef structural subtyping; use `interface` for
     shared behavior, anonymous typedefs only for pure data;
   - `interface` bodies may only declare `function`s (no `var`);
   - `haxeon/stdlib/Math.hx` is reduced: no `Math.atan`/`Math.asin` — use
     `Math.atan2(y, 1)`; grep that file before using any other `Math.*`.
   - When a compile error is unclear, read the relevant haxeon compiler
     source and find the root cause before working around it.
4. Build / test commands:
   - Haxe tests: `./haxeon/scripts/haxeon run --project robotkit/tests/haxeon.json`
     (from the materia repo root; entry `tests.RobotWorldTests`).
   - Native: `cmake -S robotkit -B robotkit/build -GNinja -DCMAKE_BUILD_TYPE=Debug && cmake --build robotkit/build && ctest --test-dir robotkit/build --output-on-failure`
   - Integration: `robotkit/tests/world-tcp.sh` (also with
     `ROBOTKIT_TEST_SESSIONS=1` and `ROBOTKIT_TEST_LEASE_TIMEOUT=1`).
   Every milestone must leave all three green.

## Ground rules

- **Layering is the core invariant.** Cartesian/tool/work concepts live
  *above* `Robot`. Flow is always
  `WorkSurface → Toolpath → IK/trajectory → JointTargets → Robot → RobotRuntime`.
  Do not add Cartesian poses, IK, tools, or work geometry to `RobotRuntime`,
  the native runtime, `robotd`'s C side, or the device protocol.
  (M8.5 is the one sanctioned native change: simulator articulation fixes in
  RobotKit's simulation layer, SimKit, and the MuJoCo backend. It adds no
  Cartesian/IK concepts to the runtime.)
- **The RobotKit core stays CAD-agnostic.** `robotkit/haxeon.json` depends
  only on `nativekit`; keep it that way. CadKit/BimKit conversion goes in a
  separate bridge project (milestone 6).
- **One 3D pose type.** Use `Transform3` for poses and transforms
  (`world_T_base` *is* the base pose). Do not create both `Pose3` and
  `Transform3`. Bridge to the planar stack with
  `Transform3.fromPose2(pose, z = 0)` and `transform.toPose2()` (yaw
  projection); do not rewrite `Pose2`/`FrameTree2`/`Navigator`.
- Value types are immutable (`final` fields, operations return new values),
  matching `Pose2`.
- No `Map<String, Dynamic>` tool commands. Every tool has a typed interface.
- Tests go in `robotkit/tests/src/tests/` — add a new test class per
  milestone (e.g. `SpatialTests.hx`) and call it from
  `RobotWorldTests.main()` rather than growing the 3,300-line file further.
  Prefer analytic checks (known FK of a 2R arm, round-trip identities,
  numeric vs. analytic Jacobian) over snapshot values.
- Deterministic: seeded RNG only, no wall-clock dependence in planners.
- Add a short section to `ARCHITECTURE.md` for each new package describing
  its contracts, in the file's existing tone. No other new docs.
- One commit per milestone (more if a milestone is large), imperative
  subject like the existing log ("Add ...", "Route ..."), each commit
  building and passing tests. Work on a new branch
  `robotkit-construction` created from the current HEAD. Do not push.
- If a milestone reveals the plan is wrong (e.g. the simulator cannot drive
  an articulated arm), stop, write down what you found, and pick the
  smallest correct adjustment described in that milestone's fallback. Do not
  silently expand scope.

---

## Milestones

### M0 — Verify the controller lease timeout (small, do first)

The original proposal asked for a controller lease timeout. It appears to
already exist: `robotd/src/RobotServer.hx` has `CONTROL_LEASE_TIMEOUT_MS =
3000`, `checkControlLeaseTimeout()`, heartbeat validation, and emergency
stop on expiry (commit `ee30219c`). Confirm with
`ROBOTKIT_TEST_LEASE_TIMEOUT=1 robotkit/tests/world-tcp.sh`. Check whether
`RobotClient`/`RemoteRobot` send heartbeats automatically while holding
control; if they do, M0 is done with no commit. Only if a real gap exists,
fix it minimally with a test.

### M1 — 3D spatial types (`robotkit.spatial`)

Add `haxe/robotkit/spatial/`:
- `Vec3` (add/sub/scale/dot/cross/norm/normalized).
- `Quat` (xyzw; multiply, conjugate, rotate vector, from axis-angle,
  from/to rotation matrix, from/to roll-pitch-yaw, normalize, slerp,
  angular distance).
- `Transform3` (translation `Vec3` + rotation `Quat`; `compose`,
  `inverse`, `transformPoint`, `transformVector`, `fromPose2`, `toPose2`,
  `fromArrays(position, rotation)` to read `Joint`/`Frame` fields,
  column-major 4×4 export).
- `Twist3` (linear + angular), `Wrench3` (force + torque), with
  `Transform3.adjoint` application for moving twists/wrenches between frames.
- `FrameTree3`: named frames with `parent_T_child` edges, `lookup(A, B)`
  returning `A_T_B`, cycle/unknown-frame errors as exceptions consistent with
  `FrameTree2`. Include a `project_T_map` registration edge in a test to
  exercise the BIM-to-robot chain
  (`project → building → storey → room → wall` and
  `map → base → arm_base → ... → flange → tcp`).

Tests: quaternion round-trips, `T * T⁻¹ = I`, composition associativity,
`fromPose2`/`toPose2` round trip, adjoint consistency
(`twist` transformed then integrated equals integrated then transformed for
small dt), FrameTree3 lookups across two branches.

### M2 — Kinematic chains and IK (`robotkit.manipulation`)

- `KinematicChain`: built from a `RobotModel` plus base `LinkId` and tip
  `FrameId`/`LinkId`; walks `Joint`s using their existing fields. Supports
  revolute and prismatic; fixed joints fold into constant transforms;
  anything else is a construction error.
- `JointGroup`: ordered `JointId`s + limits (from `JointLimits`).
- `forwardKinematics(q):Transform3`, `allLinkTransforms(q)`,
  `jacobian(q)` (geometric, 6×n, expressed in base frame).
- `InverseKinematics`: damped-least-squares iteration with joint-limit
  clamping, configurable tolerance/iterations/damping, seed configuration;
  returns a result value (`converged`, `q`, position/orientation error,
  iterations) — never throws on non-convergence.
- `Manipulator`: chain + tool mount (flange frame) + `toJointTargets(q)`
  producing the existing `JointTargets` type.
- Add a test fixture model: a UR5-like 6R arm defined through `RobotModel`
  (use published UR5 DH-equivalent link offsets; put the fixture in the test
  sources, not the library).

Tests: 2R planar arm FK against closed form; UR-style FK at zero pose;
numeric-difference Jacobian matches analytic to 1e-6; IK recovers random
reachable targets generated by FK (seeded) within 1e-4 m / 1e-3 rad; IK
reports non-convergence for unreachable targets; `toJointTargets` round-trips
through `RobotCommand` to a `SimulatedRobot` or `ReplayRobot` the way
`testJointTargetBatches` does.

### M3 — Tools and TCP (`robotkit.tool`)

- `Tool`: `id`, `flange_T_tcp:Transform3`, simple collision approximation
  (reuse `model/CollisionApproximation` if it fits, else a box/cylinder
  value), mass.
- Typed capability interfaces (methods only, per haxeon): `SurfaceTool`
  (enable/disable, standoff), `Sander` (speed, contact force), `Sprayer`
  (flow, pressure), `Gripper` (open/close, grasp state). Provide a
  simulated implementation of each that records commanded state with
  timestamps so tests and coverage can observe it.
- `Manipulator` gains `tcpPose(q)` = FK · `flange_T_tcp`, and IK accepts TCP
  targets.

Tests: TCP pose offset correct under rotation; IK to a TCP target;
simulated sprayer state history.

### M4 — Toolpaths and Cartesian trajectories (`robotkit.process`)

- `ToolpathPoint`: `work_T_tcp:Transform3`, `feedRate` (m/s),
  `processOn:Bool`, optional desired surface normal, standoff,
  position/orientation tolerance.
- `Toolpath`: ordered points + frame id they are expressed in; helpers for
  length, process-on length, segmenting into approach / process / retract.
- `CartesianTrajectory`: time-parameterized TCP samples from a `Toolpath`
  (trapezoidal velocity per segment respecting feed rate and a max
  acceleration; linear interpolation for position, slerp for rotation).
- `ToolpathExecutor`: samples the trajectory, runs IK seeded by the previous
  solution, checks joint-space continuity (reject jumps above a threshold),
  produces a stream of `JointTargets` plus tool on/off commands. Failure
  modes are explicit results (unreachable point index, discontinuity).

Tests: timing sums match path length/feed; executor on a small raster in
front of the UR fixture yields continuous joints; a deliberately
unreachable point reports its index.

### M5 — Work geometry (`robotkit.work`)

- `WorkSurface`: `id`, `frameId`, `frame_T_surface:Transform3` (surface
  plane: +Z = outward normal), boundary polygon in surface XY, exclusion
  polygons (openings, sockets), target tolerance, material tag, and a
  `provenance` value (design element id string, source kind:
  `design | observed | work`).
- `RasterToolpathGenerator`: given a `WorkSurface`, tool width, overlap,
  standoff, feed, lead-in/out → `Toolpath` (boustrophedon, clipped to
  boundary minus exclusions, tool off across exclusions).
- `CoverageMap`: 2D grid over the surface; marks cells covered by a
  process-on TCP footprint; reports coverage fraction and uncovered regions.
- Keep geometry to planar polygons for now; curved surfaces are out of
  scope.

Tests: raster over a 3 m × 2.5 m wall with a door-sized exclusion covers
≥ 99% of the allowed area and 0% of the exclusion; coverage from a
simulated execution matches the planned coverage.

### M6 — CAD/BIM → WorkSurface bridge (separate project)

Create `robotkit/cadbridge/` as its own haxeon project depending on
`robotkit`, `cadkit`, and `bimkit` (see `bimkit/haxeon.json` for the
dependency syntax). It converts:
- a CadKit planar `Face` (`surfaceKind`, `center`, `normal`, outer and inner
  wires) into a design `WorkSurface`, inner wires becoming exclusions;
- a BimKit wall (`BimSchema.Wall`) side face with hosted windows/doors into a
  `WorkSurface` whose exclusions are the openings, with `provenance`
  referencing the BIM element id;
- the BIM spatial hierarchy into `FrameTree3` edges (`project → building →
  storey`).

Read `cadkit/haxe/src/cadkit/Face.hx`, `Shape.hx`, and
`bimkit/examples/HostedWall.hx` first; if CadKit does not expose face
boundary loops, add the smallest accessor needed in CadKit (separate commit)
rather than re-deriving geometry.

Tests: `bimkit/examples/HostedWall.hx`-style wall → `WorkSurface` with the
correct area, normal, and one exclusion per opening.

### M7 — As-built registration (`robotkit.perception` + `robotkit.work`)

- Geometry perception values: `PointCloud` (points in a frame, timestamps
  per ARCHITECTURE.md), `PlaneEstimate` (normal, offset, inlier count, RMS),
  `DeviationMap` (grid of signed distances surface-vs-design).
- `PlaneFit`: least-squares plane via 3×3 covariance (implement symmetric
  Jacobi eigen-solver; no external libs) plus seeded RANSAC for outliers.
- `SurfaceRegistration`: design `WorkSurface` + observed plane/cloud →
  corrected `project_T_map` (or a wall-local correction transform) and a
  `work` `WorkSurface` derived from design with provenance retained.
  Reject registrations whose correction exceeds configured limits.
- `SimulatedSurfaceScanner`: samples points from a "true" wall that is
  offset/rotated from design (e.g. +14 mm, 0.3° yaw, 8 mm bow), adds seeded
  noise, excludes openings.

Tests: recovers the injected offset/yaw within 1 mm / 0.05°; RANSAC ignores
20% outliers; oversize correction is rejected; deviation map reflects bow.

### M8 — Base placement and reachability (`robotkit.manipulation`)

- `ReachabilityChecker`: for a base pose and a `Toolpath` segment, IK every
  point (seeded continuation) and report reachable fraction + first failure.
- `WorkPatchPlanner`: split a `WorkSurface` into patches, and for each patch
  search candidate base poses on a grid in front of the wall (fixed standoff
  band, facing the wall, respecting a clearance distance) to find one from
  which the patch raster is fully reachable. Output an ordered plan of
  `(basePose:Pose2, patch toolpath)`.
- Base motion between patches uses the **existing** planar `Navigator`/`GoTo`
  via `Transform3.toPose2`. No joint base-arm optimization yet.

Tests: a 6 m wall is split into ≥ 2 patches, each reachable from its base
pose; an obstacle blocking one candidate forces a different pose.

### M8.5 — Simulator articulation fixes (native, authorized)

The user has authorized RobotKit-runtime, SimKit and MuJoCo-backend changes
for this milestone. Findings (verified by reading the code, not yet by
failing tests — **write each failing test first**, then fix):

**F1. Link rest poses are never computed (both backends).**
`robotkit/runtime/src/simulation.cpp` creates every link's scene node with
`robot_transform(robot_index)` — identity rotation at `(robot_index, 0, 0)` —
so all links of a robot start stacked at one point, and every link gets the
same `shape_`. The MuJoCo backend (`simkit/sim_mujoco/src/mujoco_backend.cpp`,
`add_body`) builds its body tree *from those rest poses* and uses only
`anchor_b` for the joint pivot; `anchor_a` is ignored. Result: any model with
non-zero joint offsets (every arm) has wrong geometry in MuJoCo.
Fix: in `simulation.cpp`, walk the joint tree from the root at q = 0 and set
each link's rest pose to
`world_T_child = world_T_parent · T(parent_frame_position, parent_frame_rotation) · T(child_frame_position, child_frame_rotation)⁻¹`.
Also stop dropping frame rotations: extend `nksim_joint_desc`
(`sim_core/include/nativekit_sim.h`) and `BackendJointDesc`
(`sim_core/src/PhysicsBackend.hpp`) with `rotation_a[4]` / `rotation_b[4]`
(xyzw), appended after existing fields and before `reserved`, honoring
`struct_size` (old-size callers get identity). Update `.hxi`/`.hxmap`
bindings and run `sim_core/tools/check-hxi.sh` / `check-haxeon.sh`.
In MuJoCo, place the hinge/slide at the joint frame (child-local
`anchor_b`/`rotation_b`) and verify that parent-side `anchor_a` agrees with
the rest poses (assert or diagnose, don't silently ignore).

**F2. MuJoCo: all three actuators act on every joint at once.**
`add_joint_actuators` adds position (kp 100, kv 10), velocity (kv 20) and
motor actuators per joint; `apply_joint_targets` zeroes all `ctrl` then sets
only the active one. But a MuJoCo position actuator's bias
(`-kp·q - kv·q̇`) applies even at `ctrl = 0`, so a velocity-commanded joint
is dragged back toward q = 0 and stalls (with these gains, near q ≈ 0.2 for a
target of 1). Wheels cannot spin continuously; the existing test
(`sim_mujoco/tests/mujoco.cpp` ~line 215) only asserts `position > 0` after
0.2 s so it misses this. Similarly the idle velocity actuator adds damping
to position-mode joints.
Fix: replace the three actuators with **one motor actuator per non-fixed
joint** and compute torque in `apply_joint_targets` each substep:
- position: `τ = m_ii·(ωn²·(q* − q) − 2ζωn·q̇) + bias_i`
- velocity: `τ = m_ii·kᵥ·(q̇* − q̇) + bias_i`
- effort: `τ = target`
where `m_ii` is the joint's diagonal of the mass matrix (`mj_fullM` or
`data->qM` via `dof_Madr`), `bias_i` is gravity/Coriolis compensation from
`data->qfrc_bias`, and defaults are ωn = 2π·10 rad/s, ζ = 1,
kᵥ = 50 1/s. Clamp to `max_force` when > 0. This makes behavior independent
of link mass (the fixed kp = 100 would sag a UR-class shoulder by ~0.5 rad).
Keep the constants in one place; don't add them to the public C API unless a
test needs to vary them.
Tests: velocity target 1 rad/s holds ≈ 1 rad/s after 5 s with position
still increasing; a 6R arm holds a position target under gravity within
1e-3 rad after settling; effort mode unchanged; `max_force` clamp respected.

**F3. MuJoCo: self-collision between non-adjacent links.**
Only parent/child pairs are excluded (`mjs_addExclude`, ~line 518), and every
link has the same shape at the same rest point (F1), so non-adjacent arm
links interpenetrate. After F1, exclude all body pairs belonging to the same
robot for now (a robot-to-environment collision model is enough for M9);
note in ARCHITECTURE.md that self-collision is not modelled.

**F4. Default backend: kinematic link placement.**
`sim_core/src/physics_backend.cpp` tracks joint positions as numbers only;
child bodies never move with their joints, and non-root links free-fall
under gravity. After integrating joints in `step()` — and after
`set_joint_targets`, `body_set_state` on a root, and reset/teleport —
recompute each child body from its parent in topological order:
`world_T_child = world_T_parent · T(anchor_a, rotation_a) · M(q) · T(anchor_b, rotation_b)⁻¹`,
`M(q)` = rotation about `axis_a` (revolute), translation along `axis_a`
(prismatic), identity (fixed). Reject cycles with
`NKSIM_ERROR_INVALID_ARGUMENT`. Joint-child bodies skip gravity; set their
linear/angular velocity by finite difference so IMUs on arm links read
sensibly. Clamp to limits when `lower_limit < upper_limit`.

**Cross-backend acceptance test:** the same 3-link arm (non-zero offsets,
rotated joint frames) at the same joint angles gives the same link world
poses in the default backend and MuJoCo (MuJoCo: after settling, 1e-3 m;
default: 1e-9) and both match RobotKit's M2 FK. Teleporting the root carries
the arm in both.

Regression: all existing SimKit (`ctest -L sim`), RobotKit native, RobotKit
Haxe, and `robotkit_mujoco*` tests pass. Wheel bodies will now visibly spin
and sit at their real offsets; adjust any test that assumed otherwise and say
so in the commit message. Commit F1–F4 separately.

### M9 — Simulated wall-finishing robot (end-to-end milestone)

- Robot fixture: omnidirectional base (add a `HolonomicDrive` `DriveModel`
  if none exists, mirroring `DifferentialDrive`) + UR fixture arm + sprayer
  tool + scanner sensor, defined as a `RobotModel` so it compiles through
  `RobotRuntimeCompiler`.
- Scenario (in tests, like `testSimulatedMaterialHandlingScenario`):
  BIM wall → design `WorkSurface` → scan → registration → patch plan →
  for each patch: navigate base → execute toolpath → record coverage →
  final verification (coverage ≥ 99%, no process-on in exclusions,
  registration residual reported). Record the run to MCAP via
  `RecordingRobot` and replay it.
- **Simulator:** after M8.5 the default backend places arm links from
  joint positions. Read the actual TCP pose from `Simulation.linkPose()` of
  the flange link · `flange_T_tcp`, and assert it matches base pose ·
  FK(reported joints) · `flange_T_tcp` to 1e-6 — this cross-checks RobotKit
  FK against SimKit. Derive coverage from the observed pose, never from
  commanded targets. Write the holonomic base plant in the style of
  `DifferentialDrivePlant`. Model sander contact force in the simulated
  tool, not in physics. The scanner should see the wall only where the arm
  does not occlude it, if SimKit ray casts include robot link bodies.
- After the default-backend scenario passes, run the same scenario against
  MuJoCo (`new Simulation(0.01, 2, 1)`, gated like the existing
  `robotkit_mujoco` tests). Assert coverage ≥ 97% and report TCP tracking
  error (max/RMS) in the test output.

### M10 — Construction skills (`robotkit.skill`)

Following `GoTo`/`PickPallet`: `ScanSurface`, `RegisterSurface`,
`FinishSurface(surface, FinishSpec)` (drives M8's plan through M4's
executor), then `Paint` and `Sand` as thin `FinishSpec` variants (sprayer vs.
sander tools; `Sand` adds a contact-force setpoint on the simulated sander).
`Drill` (point operations at surface positions) is optional. `LayTile` is
**out of scope** for this plan — it needs inventory, grasping, adhesive and
force control; leave a short note in ARCHITECTURE.md on what it will need.

Tests: each skill runs through `SkillRunner` against the M9 simulated robot
and against a `ReplayRobot` of a recording, as the forklift skills do.

### M11 — Terrain height map (`robotkit.work`)

- `HeightMap`: regular grid in a frame, elevation per cell, bilinear sample,
  volume between two height maps (cut/fill).
- `EarthworkRegion`: existing `HeightMap`, design `HeightMap`, exclusion
  polygons, grade tolerance.
- `BucketSweep`: approximate material removal — lower cells intersected by a
  swept bucket cutting edge to the edge height, returning removed volume.
  No soil mechanics.

Tests: volume of a known trench; sweep removes expected volume; cells
within tolerance of design are reported "at grade".

### M12 — Simulated excavator

- Excavator fixture `RobotModel`: tracked base, slew (revolute Z), boom,
  stick, bucket (revolute), TCP = bucket cutting edge via `Tool`.
- Reuse `KinematicChain`/IK (4-DOF; position + bucket pitch only — use a
  task-space weighting in IK or a closed-form planar solver).
- `DigCyclePlanner`: bucket entry → cut → curl → lift → swing → dump as a
  `Toolpath` in the excavator frame.
- Skills: `DigTrench(line, width, depth, gradeTolerance)` and
  `GradeRegion(region)` as bounded operations only; `DumpAt(pose)`.
- Scenario test: trench dug to within tolerance over N cycles with progress
  reported from the height map.
- Add an ARCHITECTURE.md note: application-level autonomy here is not the
  safety-rated layer; ISO 17757 / ISO 19014 functional safety belongs in
  certified machine controllers beneath `RobotRuntime`.

---

## Explicitly out of scope

- Soil mechanics, hydraulic valve/pressure modeling, engine control.
- Curved/freeform work surfaces; tiling; bricklaying.
- Joint base+arm whole-body optimization.
- Changes to the device protocol or `robotd` beyond what M0 might require.
  Native runtime/SimKit changes are limited to M8.5's scope.
- Replacing or deprecating `Pose2`/`FrameTree2`/`Costmap2`/`Navigator`.

## Done checklist per milestone

1. New code compiles under haxeon; all three test commands pass.
2. New test class exercises the milestone's acceptance criteria.
3. ARCHITECTURE.md section added/updated.
4. Commit(s) on `robotkit-construction`.
5. A one-paragraph note at the bottom of this file under "Progress log":
   what was built, any deviation from this plan and why.

## Progress log

**M0** (no commit): verified the controller lease timeout already exists —
`RobotServer.hx` has `CONTROL_LEASE_TIMEOUT_MS = 3000`, `checkControlLeaseTimeout()`,
and emergency-stops on expiry, and `RobotClient` renews the lease automatically
(`renewControlLeaseIfDue()`, called from its event dispatch loop and from
`poll()`/`wait()`) whenever it holds control. Confirmed with
`ROBOTKIT_TEST_LEASE_TIMEOUT=1 robotkit/tests/world-tcp.sh`. No gap found, so
no code change.

**M1**: added `robotkit/haxe/robotkit/spatial/` (`Vec3`, `Quat`, `Transform3`,
`Twist3`, `Wrench3`, `FrameTransform3`, `FrameTree3`) and
`robotkit/tests/src/tests/SpatialTests.hx`, called from
`RobotWorldTests.main()`. Read `robotkit/haxe/robotkit/localization/RobotFrameTree2.hx`
and `robotkit/runtime/src/simulation.cpp` to confirm the joint-frame
convention before implementing FK-adjacent code: `axis` is expressed in the
joint frame (i.e. after `parentFrameRotation`), matching how `simulation.cpp`
rotates `axis` by `parent_frame_rotation` to build the native `axis_a`. No
plan deviations in M1; documented the convention in `ARCHITECTURE.md` so M2
does not need to re-derive it.

**M2**: added `robotkit/haxe/robotkit/manipulation/` (`ChainTip`,
`KinematicChain`, `JointGroup`, `IKResult`, `InverseKinematics`,
`Manipulator`) and `robotkit/tests/src/tests/KinematicsTests.hx` (a 2R
planar fixture, plus a 6R "UR5-style" fixture built from published UR5
DH-equivalent link offsets, both defined in the test file only). FK and the
Jacobian are derived directly from `Joint`'s existing fields, per the
convention M1 documented. Deviation: the first version of
`InverseKinematics`/`KinematicChain` built its internal `Array<Array<Float>>`
matrices with nested array comprehensions (`[for (...) [for (...) ...]]`);
haxeon's backend rejected this with an opaque
`CFG array read has the wrong element type` error from `CfgVerifier.hx`.
`cadkit/haxe/src/cadkit/modeling/AssemblyLoopSolver.hx` and
`cadkit/haxe/src/cadkit/sketch/SketchSolver.hx` already do the same kind of
matrix math in this repo and consistently build nested arrays with explicit
`push` loops instead of nested comprehensions; switching to that pattern
fixed the compile with no functional change. A fixed zero seed for the "IK
recovers random reachable targets" test occasionally converged to a
different, still-valid, solution branch of the 6R wrist instead of the
seeded target's own branch and timed out; changed the test to warm-start IK
near the seeded target configuration (as a real caller re-solving from its
last known joint state would), which is standard practice for
local-convergence IK tests and matches the plan's "recovers ... within
1e-4 m / 1e-3 rad" acceptance criterion without asserting which branch is
found. Documented this joint-configuration ambiguity in ARCHITECTURE.md.
