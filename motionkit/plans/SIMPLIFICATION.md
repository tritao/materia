# MotionKit simplification plan

MotionKit's core decisions still hold: numerics are native and Haxe orchestrates (D1), transmissions
live in the model (D2), the execution session is native (D3), and the canonical trajectory is
piecewise polynomial (D4). Ruckig times free motion and TOPP-RA or the affine jerk route times paths,
all through one certification and lowering chain.

What has gone wrong is accretion. In almost every layer, two or three generations of the same
mechanism are live at once. D1's rule, that a replaced planner "is then deleted, not kept in
parallel", was not enforced. This plan removes the older generations. It adds no capability.

The survey was made on 2026-10-22 on branch `jerk-timing`, which lands before this plan starts. Its
file and line references are to that branch.

## Ground rules

- **Deletion first.** Each phase names what it deletes. Report the line delta per phase.
- **No compatibility layers or shims** (see the repo-wide no-legacy-compat rule). Callers move in the
  same commit as the API they use.
- **No behaviour change unless stated.**
  - Cycle times, PlanCheck findings and certification results stay the same.
  - Where a deleted route was the one actually in use, the phase states the difference and measures it.
- **Every commit stays green** on:
  - the MotionKit suite (`motionkit/tests`) and the native CTests;
  - the RobotKit integration suite;
  - the ProcessKit tests;
  - the toolpathkit motion tests;
  - the app cases that run motion: CNC, print, hobbing, mission, welding, tending, screwdriving.
- **The order is S0 → S1 → S2 → S3 → S4 → S5.** S1 and S2 are independent and may run in parallel
  worktrees.

## S0. Landing gates for jerk-timing (owned by that branch)

These are added to J5/J6 so the branch does not land with two hold designs:

1. **Production uses the J5 design.**
   - `ManipulatorMotion` passes `PublishedPathClockSettings` to its executors.
   - The app players (`CncProgramPlayer`, `PrintProgramPlayer`) stop passing `PathControlSettings`.
   - Hold becomes `executor.hold`, override becomes `executor.setFeedOverride` (capped at 1), and
     resume becomes `restartHeld` with the normal planner.
2. **Delete the superseded on-demand route:**
   - `PathControlSettings`, `PathStopPreparation`, `PathContinuationCompilation`;
   - `ContinuedProgramCompilation` and `ResumedProgramCompilation`, unless Held restart reuses them;
   - `ProgramCompiler.stop/resume/retimeOnPath`, `ProgramPlanner.fromPathResume/fromPathOverride`;
   - `PlanExecutor.holdFromSource/replaceFromSource/retainPrefixes`;
   - `ManipulatorMotion.tryPathHold/tryPathOverride/updatePathHold`;
   - the `speedScale` plumbing through the compiler.
3. **Delete `mk_path_change_speed`**, which is new on the branch and used only by `PlannerTests`.
4. **`path_time_filter.hpp`** is included only by its own test. Either J3 wires it in, or it is deleted
   until J3 needs it.

## S1. One path-planning route

**Problem.** `ProgramCompiler.defaultJointPathPlanner` returns null for external axes, work frames,
7-DOF and non-6-DOF arms. Those groups then go through `KinematicsSolver.solvePath`, which means the
`RedundancyResolver` beam search or `PathRequest.followPointByPoint`. Every planning feature therefore
exists twice, or is missing on one route.

Separately, `structured_ladder.cpp` `small_ladder` runs the structured DP, then solves the same graph
again with Descartes and returns the Descartes route.

**Change.**
- Commit the PP6 deletions. They are uncommitted in worktree `gantries` (branch `process-paths`) and
  include `ExternalAxesParameterization`. Then finish PP9.
- `jointPathPlanner` becomes non-null. Every group goes through `StructuredJointPathPlanner`.
  `CandidateProblem` already carries external axes (`ExternalAxisGrid`) and numeric branch families
  (`NumericBranchIk`).
- Narrow `JointPathPlanner` to section planning, `planSections(paths, request) -> curves`.
  - Clearance moves to a `ClearanceChecker` owned by the compiler, so `AxisJointPathPlanner` no longer
    stubs 6 of the 9 methods.
  - The planner exposes the coarse route cost that welding needs, so `WeldingPlanRunner` stops
    rebuilding ladder internals.
- **Native:** a persistent ladder handle (candidates, costs, `block_edge`/`disable_state`, re-search).
  Component splitting and reach pruning move from `StructuredLadder.hx` and `CandidateProblem.hx` into
  it, and Haxe keeps only collision verdicts.

**Delete:**
- `KinematicsSolver.solvePath`, `RedundantPathSolver`, `RedundancyResolver`, `RedundancyPoses`,
  `RedundancyCurve`, `CubicRedundancyCurve`, `RedundancySpline`, `RedundancyParameterization`,
  `SwivelParameterization`, `PathSolution` and `PathRequest.followPointByPoint`;
- `EaikKinematics.solvePath`;
- `processkit` `WeldPathPlanner`, its tests and `WeldPlanning.planner`. Its constants (`LIFT`,
  `AIR_MARGIN`, `STEP`, `CONTACT_ZONE`, `LIMIT_CLEARANCE`) move to a weld process constants class;
- Descartes:
  - the `descartes_light` vendor tree;
  - the `motionkit_descartes_core` target;
  - OpenMP;
  - `configuration.cpp`, `mk_select_configurations` and backend 3;
- the unused ladder and candidate exports: `mk_search_ladder`, `mk_search_ladder_filtered`,
  `mk_*_candidate_count`, `mk_sample_*_candidates[_compact]`.

**7-DOF swivel (decided 2026-10-23).** The swivel becomes one more lattice dimension, handled the same
way as an external axis in PP6.
- The swivel is one periodic coordinate per sample.
- PP6's per-sample centred grid and its de-quantizing band refinement carry over unchanged.
- Branches at each fixed swivel value come from `NumericBranchIk` over
  `KinematicGroup.solve(... atSwivel)`.
- Centre the grid on the preferred swivel: the posture preference, elbow-up by default. Do not centre
  it on the continuation value; PP6 found that continuation-centred free spin jumped at singularities.
- No production machine has a 7-DOF arm, so the gate is the existing 7-axis MotionKit cases
  (`ProgramTests`, `ToolFreedomTests`), ported and compared against their old results.
- A closed-form SRS (stereo-SEW) per-swivel IK waits for a real 7-DOF arm. It slots in behind the same
  branch interface.
- Depends on the PP6 commit, which carries the band refinement.

**Excavator (decided 2026-10-23).** This follows the deletions as the last PP9 item. It does not block
them, because `ToolpathPlanRunner` uses RobotKit IK and `MoveJ(JointTarget)`, never `solvePath`.

Problem:
- The machine is a swing plus a planar boom/stick/bucket chain. Its heading is fixed by the TCP's
  bearing from the swing axis; only the attack angle in the swing plane is free.
- Today the code works around this:
  - `DigCyclePlanner.poseAt` hand-sets yaw to `atan2(y, x)`.
  - The swing is densified "to stay on the manifold".
  - `ToolpathPlanRunner` emits exact-stop `MoveJ` per waypoint, with joint speeds from an
    `angularDistance * 0.1` blend.
  - Between entry and cut depth the bucket edge therefore follows a joint-interpolated curve, not the
    straight sweep that `BucketSweep` applies to the terrain.

Change:
- **Orientation policy.** Add a policy for swing-plane machines: the caller supplies the attack angle
  about the swing-plane normal, and the heading follows from position. It is generic for excavators,
  backhoes and loader cranes.
- **IK family.** Add an analytic family for a swing plus a planar 3R chain (about two branches) beside
  `CartesianAnalyticIk`.
- **Callers.** `DigCyclePlanner` authors position plus attack angle, and `ToolpathPlanRunner` emits
  `FollowPath`.
- **Delete** `poseAt`'s hand-set yaw, the swing densification and the per-point speed blend.

Gate:
- `ExcavatorTests` dig, grade and dump pass.
- The cut follows the straight sweep that the terrain update applies.
- Cycle times are reported against the current `MoveJ` runner.

**Proof:**
- ProcessKit weld/surface/handling, tending reach, the RobotKit construction skills (Paint), the
  excavator and the 7-DOF arm tests pass.
- Ladder costs and routes are unchanged where the structured planner was already used.
- For groups that move from the legacy route, record the cycle-time and route-cost differences.

## S2. One way to move a machine

**Problem.**
- There are four pipelines into the runtime: `ManipulatorMotion` (the production path),
  `MotionSystem` moves, homing through `RuntimeHomingDriver`, and `ServoSession`/`ServoPlan`.
- Three app and processkit sites also bypass all of them: `MachineDigitalProgramPlayer`,
  `ContactProbeRunner` and `RobotMotionPlayer`.
- Tags come from four id spaces: a static program tag from 1e6, a per-stream motion tag from 1, a
  static homing tag from 2e8, and `ServoPlan.nextId`.
- About eight objects track what has been sent, committed or executed.
- `MotionSystem`'s move/path/jog API is used only by tests. Production uses it only for homing.

**Change.**
- **Axis moves, jog, linear moves and path moves** lower through `ProgramCompiler`
  (`AxisKinematics`) into `ExecutionPlan`s and run on `PlanExecutor`. Jog uses Ruckig through the same
  plan type.
- **Homing** becomes its own owner, `MachineHoming`: the homing cycle plus the runtime driver. It
  submits through the same queue, and homing side control is one typed capability object instead of
  `Std.isOfType` downcasts.
- **One per-robot `PublicationQueue`** owns tag allocation, queue room, windows over a queue of plans,
  one submission builder and one progress ledger (plan id, tag, origin, committed, executed). It also
  decides who owns the command channel.
  - `PlanExecutor` stops being one-plan-per-stream, so `ManipulatorMotion`'s executor double buffer and
    `prefixSources` go away.
- **RobotKit exposes the queue protocol** as a `PlanQueue`: publish a window and get a token; observe
  the token, local time, committed horizon and room. MotionKit stops reading `RK_MAX_*` constants and
  snapshot queue fields directly.
- **Servo chunks and the three bypassing sites** submit through `PublicationQueue`.
- **Certification has one channel.**
  - `PlanCheckResult` becomes a row of `ValidationReport`, and `ExecutionPlan.checked` is removed.
  - `PlanCertification` is the only place checks run, with a small entry point for servo chunks.
  - There is one summary per motion owner.

**Delete:**
- `PathPlanner`, the second TOPP-RA plus degree-1 resampling pipeline;
- `AxisPlanner`, if its moves are covered by the compiler;
- `TrajectoryStream.motionSubmission`, `submitSmoothReplacement`, `MotionRequestCapture` replay and
  `MotionSystem`'s own buffer queue;
- `MotionLimits`, `PathPlanningOptions` and `AxisTarget`, if `MotionSystem` keeps no direct API.

**Proof:**
- All homing app cases, jog and the MotionKit session and stream tests pass.
- The tag and ledger invariants have their own test: one queue, monotonic tags, no gap at plan cuts.

## S3. One path-timing abstraction and one limits type

**Problem: native.**
- `PathTimingStream` holds both TOPP-RA and affine state, and `step` branches three ways.
- An undocumented closed-form trapezoid runs under `TOPPRA` for two-sample rest-to-rest straights.
- `mk_time_path`, the 5 timing-session calls and `mk_time_path_reference` are reachable only from
  tests and `machinekit/examples/sheet-router/fem/planner_sample.cpp`. Only they reach the 24-attempt
  stretch/soften retry ladder.
- There are two jerk-limited profile engines (our own S-curve code and Ruckig) and two clock cursors.
- Stop, resume and retime each carry their own search ladder.

**Problem: Haxe.**
- `PathTimingBackend` is a facade. Callers discover capabilities through `isOfType` and
  `affineMap()!=null && boundsJerk()`, while retiming bypasses the backends through `NativeJointPath`.
- The distance↔time map has three representations: `TimedPath` closures, `TimedPathSeries` and
  `PathTimingSnapshot`.
- Shaping has three entry points.

**Problem: limits.** There are seven overlapping limit types. `PathTimingLimits` is built from
`ValidationLimits` and then re-encoded into the same `mk_limits`.

**Change.**
- **Native:** a `WindowBackend` interface (prepare / next / commit) with TOPP-RA and affine
  implementations. The trapezoid becomes an explicit method or is removed.
- **Native:** stop, resume and retime become `revise(anchor, feed)`, with stop as feed 0, sharing one
  bridge search.
- **Native:** one on-clock S-curve engine, chosen by measurement. Ruckig stays a test oracle if it is
  not the engine. One clock cursor over `mk_time_stage`.
- **Haxe:** one `PathTimer` interface with explicit capabilities (prepare, retime support, the request
  carrying shaping). Toppra and Affine become strategies.
- **Haxe:** `PathTimingSnapshot`, renamed `PiecewiseTimeLaw`, becomes the only time-law type.
  `TimedPath` becomes `{trajectory, law, bindings}` with no closures.
- **Limits:** `PathTimingRequest {limits:ValidationLimits, speedCaps, startSpeed, endSpeed}` lowers
  `ValidationLimits` directly. `PlanningLimits` produces `ValidationLimits`, which removes the
  compiler's parallel jerk arrays. `MotionOptions` absorbs `MotionLimits`.
- **Policy constants** (grid density, margins, retry ladders, bisection depth, window length) move into
  one named policy struct, each with its stated derivation.
  - Investigate the double reserve: the 0.8 `interpolation_margin` also applies to the stream's
    interval-proof constraints.
  - Do not coarsen the 32-per-span grid without a per-span error estimate that also sets the margin.
- The Haxe `AffinePathTiming` reads the native stage and pass limits instead of copying them.

**Delete:**
- `mk_time_path`, the timing session API, `mk_time_path_reference` (it becomes a test helper over the
  stream), the stretch/soften ladder and `adjustments`. Port the router FEM sample to the stream first;
- `mk_timing_report.boundary_lowerings`, `ToppraPathTiming.boundaryLowerings` and the always-zero
  `PrintingBenchmark` assertion;
- `PathTimingBackend` and `SimplePathTiming` (moved to test support if tests still need them);
- `TimedPathSeries` bisection, `PathTimeLaw.shaped`, the Haxe-stage `PathTimeLaw` constructor and
  `NativeJointPath.lower/changeSpeed`;
- `PathTimingLimits`.

**Proof:**
- Cycle times on the printer, router, Paint, weld and CNC fixtures match before and after within
  arithmetic noise, with any intended difference stated.
- Planning CPU and peak memory are not worse.

## S4. Split `path.cpp` and share the math

After S3, split the remaining file along its seams. The 5,352 lines on the branch should come down to
roughly 3,500 across these files:

| File | Contents |
|---|---|
| `path_registry.cpp` | Handles, `Law`, source context, the path/law/source/binding ABI. The four `source_*` getters become one snapshot read. |
| `time_stage.hpp` | Stage evaluation and inversion, `stages_from_profile`, anchors. Replaces the four jet evaluators. |
| `polynomial.hpp` | Algebra, Hermite (one, not `hermite` and `small_hermite`), compose, roots. `path.cpp:358-421` is a verbatim copy of `trajectorykit/native/src/merge.cpp`, so this header is shared with trajectorykit and also used by `validation.cpp`'s root finder where possible. |
| `path_lowering.cpp` | `SpanCache`, `append_interval`, `lower_path_samples`. `prepare_stopped_affine`'s private degree-2 lowering goes through it. |
| `toppra_timing.cpp` | TOPP-RA backend. |
| `affine_jerk_timing.cpp` | Affine backend. |
| `path_shaping.cpp` | One convolution rule for `mk_shape_trajectory` and the time-law shaper. |
| `path_revision.cpp` | `revise`. |
| `timing_stream.cpp` | Stream and preview objects over `WindowBackend`. |

Also:
- **Trajectory operations** (couplings, advance, merge, shape) move to trajectorykit.
- **Diagnostics:** the 30 `getenv` and ~50 `fprintf(stderr)` calls go behind one cached diagnostics
  sink.
- **Duplicated math:** pick one side for quintic hull, De Casteljau and control-difference math. It is
  currently in both `quintic_geometry.cpp` and Haxe `QuinticPolynomial`, and `QuinticBlend` uses both.
- **Delete** `mk_eaik_forward` and any other ABI with no Haxe caller left after S1–S3.
- **haxeon.json `native.cmake.inputs`** lists every source and header, so the fingerprint sees edits.

## S5. Package and kinematics boundaries

**Haxe packages.**
- A `motionkit.timing` package takes `JointPathSamples`, `JointPathPolynomial`, `NativeJointPath`,
  `PathTimeLaw`, `TimedPath`, the stream and snapshot, backends, `BindingConstraint`,
  `PathPieceMetadata`, `ShaperImpulses` and `InputShaper`.
- `planner` keeps RRT only, and `path` is pure geometry with no timing or trajectory imports. This
  removes the `path`↔`planner`↔`trajectory` cycle.
- `PlanDiagnostic`, `PlanCheckResult` and `PlanSlip` leave `trajectory`.

**Dependencies.**
- `motionkit-robot` drops `robotkit-policy` (ONNX). `CommandRejection` moves to robotkit core, and
  `ServoSession` reuses `VelocityReference`'s stale/deadline logic.
- It drops `machinekit` and `cadbridge`. `MachineKitRobotCompiler` moves to MachineKit test support,
  or to cadbridge if a production user appears:
  - its second screw-ratio equation and re-check go;
  - it calls `DriveLoads` for step resolution;
  - the drive (currently hard-coded 24 V / 0.5 A / DM542 / 16 µsteps) is supplied by the caller.
- It depends on robotkit core and autonomy instead of the umbrella.

**Kinematics.**
- One cached kinematic family per `KinematicGroup` (FK, branches, differential, family tag), owned
  below MotionKit.
  - `KinematicsSolver` becomes a view over it, and ProcessKit queries the tag instead of `isOfType`.
  - `CartesianAnalyticIk` and `AxisKinematics` become one Cartesian family.
  - `BranchIk.of` stops choosing by try/catch.
- The coupling graph is compiled once (KinematicsKit KK-D5) and consumed by `AxisKinematics`,
  `MotionAxisCouplings`, `CouplingProjection`, RobotKit `SegmentArrays` and timing. Today there are
  about six implementations.
- `ManipulatorKinematics`' own damped-least-squares moves to KinematicsKit `DifferentialIk`, as
  KINEMATICS.md planned.
- One `Pose3`. MotionKit, RobotKit mobile and VisionKit each have their own.

**Geometry vocabulary.**
- Delete `ArcSegment` (a subset of `CircularSegment`).
- Primitives gain `angularRate`, `positionErrorBound` and `distanceTo`, which removes the `isOfType`
  switches in `PathPieceMetadata`, `PoseGeometry` and `GeometricPath`.
- Later, `PoseLine`/`PoseArc` become `PoseGeometry` over 3-D primitives plus an orientation policy.
- `MotionOp.FollowPath` drops the `frameId` and `events` that `PosePath` already carries.
- Delete `program/MotionPath`, `Pose`, `PathDerivativeEstimate`/`estimatePathDerivatives`.
- `ExecutionPlan.create` and `generateStateToState` take a `StartState` and a limits type instead of
  parallel arrays.

**Cohesion.**
- `ProgramCompiler` gets an immutable `CompilerSettings` in place of its 18 mutable fields copied by
  hand in `copySettings`. The op interpreter, path lowering and joint-move lowering split into separate
  units, and env-var profiling becomes a diagnostics hook.
- `ManipulatorMotion`'s barrier evaluation moves to a `ProgramBlocks` cursor, and the slip hook
  becomes a simulation observer.
- `CartesianAdmittance` and `HandlingPlanRunner` leave the planning package.

**Docs.**
- The MotionKit README timing section becomes a description of the design rather than a log.
- `PROCESS_PATH_PLANNING.md` is condensed to its decisions and remaining work.
- Completed lane plans move under `plans/done/`.
