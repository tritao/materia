# Lane B — Arm processes: toolpaths as plans, synchronized events, ProcessKit (handoff)

**Goal:** arms move through the same validated, runtime-owned plans as axis
machines, and process commands (spray, flow, dispense) fire at planned places
on the path. Today arm toolpaths bypass all of that:
- `ToolpathExecutor` runs IK point by point;
- `FinishSurface` sends `RobotCommand.JointTargets` every update tick
  (`robotkit/haxe/robotkit/skill/FinishSurface.hx:249`);
- the tool is switched from software with a step index as its timestamp
  (`:247`).

By the end of this lane, the simulated wall-finishing robot (M9) paints
through `MotionProgram` → plans → runtime, with spray events synchronized to
path position, on-path hold and resume, and ProcessKit handling interruption
and recovery.

Read first:
1. `motionkit/plans/CONTRACTS.md`, especially C1 (events), C2
   (`MotionProgram`), C3 (lowering and task-space validation), C4
   (kinematics) and the cross-lane rules. §P0 must be on `main`.
2. `motionkit/IMPLEMENTATION_PLAN.md`: decisions D1–D4, Ground rules, and
   the P7–P9d log (plan submission, committed horizon, HOLD, `ends_at_rest`,
   replacement retry in `MotionSystem`).
3. `robotkit/ARCHITECTURE.md`: "Kinematic chains and IK", "Tools and TCP",
   "Toolpaths and Cartesian trajectories", "Simulated wall-finishing robot
   (M9)", "Construction skills (M10)", "Simulated excavator (M12)".
4. Code:
   - `robotkit/haxe/robotkit/process/*`
   - `robotkit/haxe/robotkit/skill/FinishSurface.hx`, `DigTrench.hx`,
     `GradeRegion.hx`, `DumpAt.hx`
   - `robotkit/haxe/robotkit/tool/*`
   - `robotkit/tests/src/tests/WallFinishingScenarioTests.hx`,
     `ExcavatorTests.hx`, `ProcessTests.hx`
   - `motionkit/robot/haxe/motionkit/robot/MotionSystem.hx`

Work in `../materia-lane-b` on branch `lane-b-arm-process`, from `main` after
§P0. Same rules as the other lanes: failing test first, one commit per item,
merge to `main` after each green item, log at the end, and stop if the plan
turns out to be wrong. Lane A needs **B2 on `main` early** for its event work
(A8), so prioritise it.

## Lane decisions

- **LB-D1 — One manipulator motion facade.**
  `motionkit.robot.ManipulatorMotion` is to arms what `MotionSystem` is to
  axis machines:
  - it compiles `MotionProgram`s, submits plans, and tracks progress, hold,
    resume, abort and events;
  - it reuses `MotionSystem`'s plan submission, retry and streaming. Extract
    the shared parts into an internal `PlanExecutor` class rather than copying
    them.
- **LB-D2 — Keep `robotkit.process` as a compatibility facade.** `Toolpath`
  and `ToolpathPoint` stay as authoring types and convert to
  `FollowPath(path, events)`. The per-tick `ToolpathExecutor` execution path
  is deleted once every skill has migrated (B6).
- **LB-D3 — Timing goes through one lowering entry point.** Arm paths are
  timed through `motionkit.planner.PathTimingBackend` (contract C3, landed in
  §P0), using `SimplePathTiming` until Lane C's TOPP-RA backend lands. B8
  switches the backend. Nothing else changes.

---

## B1 — Cartesian process paths with orientation

Do (pure `motionkit`):
- `motionkit.path.PosePath`: an ordered pose path (position + xyzw) in an
  explicit frame.
  - Primitives: `PoseLine` (straight position, orientation per policy) and
    `PoseArc`.
  - Per-point position and orientation tolerances.
  - An orientation policy per C3: `Fixed`, `Interpolated`,
    `Cone(axis, halfAngle)` and `FreeAboutTool`.
  - Arc-length parameterisation of position. A combined path parameter
    handles pure reorientation segments: use a rotation-to-length weight
    documented in metres per radian, never a unitless sum.
- `PathEvent`s are attached by distance (C1).
- Conversion `robotkit.process.Toolpath` → `PosePath` + events, in
  `motionkit.robot` since it needs RobotKit types:
  - `processOn` transitions become events on a caller-named channel;
  - feed rates become per-segment speed limits.

Tests:
- length and pose evaluation on lines, arcs and pure reorientation;
- tolerance and policy round trip;
- a `Toolpath` with three `processOn` spans produces six events at the right
  distances.

## B2 — Events through plans and the runtime (C1), with simulated tool adapters

Do:
- **Plan and ABI:**
  - `ExecutionPlan` (native and Haxe) carries `TimedEvent`s;
  - the C plan submission gains a bounded event array
    (`RK_MAX_PLAN_EVENTS`, e.g. 256) via `struct_size` versioning;
  - bump `RK_API_VERSION` and regenerate the `.hxi`;
  - `RK_PLAN_CAPABILITY_EVENTS` (reserved in §P0) is now supported by
    queue-executing endpoints.
- **Channel declarations:**
  - in the runtime blueprint: id, kind, safe value;
  - supplied by the simulation robot description and `deployment.json`;
  - a plan referencing an undeclared channel is rejected at submit.
- **Runtime:**
  - Events queue alongside segments.
  - An event fires when the **trajectory clock** crosses its time.
  - HOLD applies `SafeWhileHeld`. RESUME restores `RestoreOnResume`.
  - STOP, ABORT, fault and e-stop discard unfired events and set every
    channel to its safe value.
  - Replacement discards events at or after `replace_after`.
- **Fired-event record:**
  - a bounded ring buffer read through `rk_robot_runtime_poll_events` (with an
    overflow flag), plus its Haxe wrapper;
  - each record: plan id, channel, value, scheduled trajectory time, applied
    owner time, and cause (`scheduled`, `hold_safe`, `resume_restore`,
    `stop_safe`);
  - **every** channel change goes through this record, synthetic safe values
    included, so there is exactly one path to the tools;
  - recording (MCAP) captures the records, as a schema bump with the old
    readers kept.
- **Simulated tools:** a `ChannelToolAdapter` in `robotkit.tool` maps
  channels to `Sprayer` / `SurfaceTool` / `Sander` calls. It passes the
  record's scheduled time as the tool call's `timestampNs`. The existing
  simulated tools consume it unchanged.

Tests (native and Haxe):
- events fire at the right trajectory time with and without a mid-path HOLD;
- STOP sets the safe values;
- a replacement drops later events;
- an undeclared channel is rejected;
- ring overflow is flagged;
- the MCAP round trip works;
- the Haxe adapter drives `SimulatedSprayer` at the scheduled times.

## B3 — `MotionProgram` compiler v1 (C2) with IK along paths

Do (`motionkit.robot.ProgramCompiler`):
- Compile `MoveJ`, `MoveL`, `FollowPath`, `Dwell`, `SetOutput` and
  `WaitInput` into plan blocks. `WaitInput` and `Dwell` end a block, and
  `SetOutput` becomes an event at the end of the previous motion.
- **`MoveJ`:** uses Ruckig (`mk_generate_state_to_state`). A `PoseTarget`
  resolves through `KinematicsSolver.sampleCandidates` (C4) and picks the
  candidate closest to the start configuration, in joint distance weighted by
  joint velocity limits.
- **`MoveL` / `FollowPath`:**
  - Densify the `PosePath` at a stated Cartesian resolution.
  - Solve IK seeded from the previous point.
  - **Reject** a joint jump above a configurable bound, with a diagnostic
    naming the path distance. Never "fix" it silently.
  - Build q(s), time it through `PathTimingBackend` (LB-D3; the simple
    backend first), lower it to segments via the C3 entry point, and fill
    the task-space slot (C3) by sampling the lowered trajectory through
    forward kinematics.
- **Blending:** `ExactStop` everywhere in v1. `ToleranceBlend` is accepted
  but treated as exact stop, with a logged note. Real blending arrives with
  Lane C's timing.
- **Diagnostics** name the op index and path distance: unreachable pose, IK
  discontinuity, task-space tolerance exceeded, joint limit.

Tests:
- `MoveJ` to a pose on the 6R fixture;
- a straight `MoveL` stays within the task-space tolerance at the stated
  sampling resolution;
- a path through a wrist singularity is rejected with a diagnostic, not
  executed;
- a `WaitInput` splits the program into two blocks;
- events carried through `FollowPath` land at the right trajectory times
  after lowering.

## B4 — `ManipulatorMotion` facade (LB-D1)

Do:
- Extract `PlanExecutor` from `MotionSystem`: plan submission, the
  replacement retry, streaming with `ends_at_rest`, and progress tracking.
  `MotionSystem`'s tests must pass unchanged.
- `ManipulatorMotion`:
  - `run(program)`;
  - `hold()`, `resume()`, `abort()`;
  - `progress()`: block, op, path distance;
  - `firedEvents()`;
  - completion and failure status, with the compiler or runtime diagnostic;
  - barrier handling: it evaluates `WaitInput` against a caller-supplied
    input source, and the timeout fails the program cleanly.
- It requires plan support at construction, like `MotionSystem`.

Tests:
- a program with two blocks and a barrier completes;
- a hold in the middle of a `FollowPath` stays on the path and delays events;
- abort sets the safe values;
- a barrier timeout fails cleanly.

## B5 — Wall finishing through plans (M9 acceptance)

Do:
- `FinishSurface` (and `Paint` / `Sand`) builds a `MotionProgram` per patch:
  - approach as `MoveJ`;
  - process as `FollowPath` with spray events from `processOn`;
  - retract as `MoveL`.
- It runs through `ManipulatorMotion` and drives the tool through
  `ChannelToolAdapter`.
- Remove the per-tick `JointTargets` loop and the step-index timestamps.

Tests:
- `WallFinishingScenarioTests` and the MuJoCo runner reach their existing
  coverage acceptance (≥ 97% in MuJoCo, per the M9 criteria);
- spray on/off happens within one owner cycle of the planned path positions;
- a new scenario holds mid-patch, resumes, and still passes coverage with no
  paint applied during the hold.

## B6 — Excavator and remaining toolpath skills; delete per-tick execution

Do:
- Migrate `DigTrench`, `GradeRegion` and `DumpAt` (and any other
  `ToolpathExecutor` user) the same way.
- Keep `BucketSweep` material accounting driven by the executed trajectory.
  It should read the runtime's executed pose, not the submitted targets.
- Then delete `ToolpathExecutor`'s per-tick execution API, plus
  `ToolpathExecutionStep` and `ToolpathExecutionResult` if they become
  unused. `CartesianTrajectory` goes too if nothing uses it after B3.

Tests: `ExcavatorTests` and `ConstructionSkillTests` pass with their current
acceptance bounds. `ProcessTests` are re-expressed on B1/B3.

## B7 — ProcessKit v1: painting and dispensing

Do: a new haxeon project `processkit/` (package `processkit`) depending on
`motionkit`, `motionkit-robot` and `robotkit`.
- `ProcessRecipe`: speed range, standoff, orientation policy, pass spacing,
  flow per speed (for dispensing, quantity per distance), trigger lead.
- `ProcessDevice` interface: prepare, ready check, fault query, safe state.
  It is backed by `ChannelToolAdapter` in simulation.
- `ProcessRun` state machine, with transitions logged:
  - preparation → ready → active → controlled interruption → recovery →
    completion.
  - **Interrupted-process recovery** doesn't resume at the same trajectory
    time. It re-approaches to the path distance where processing stopped,
    minus a recipe back-off, and restarts with events from that distance.
- **Feed changes:** an explicit per-recipe policy, `Adapt` (flow ∝ speed via
  events or an analog channel), `Pause` or `Reject`, per C1.6.
- Compiles to `MotionProgram`. Never submits plans directly.

Tests:
- painting a patch with a mid-pass fault, then recovery with overlap
  coverage within tolerance and no double coverage beyond the back-off;
- a dispense bead with a feed override under `Adapt` keeps quantity per
  distance within 2%;
- `Reject` refuses the override.

## B8 — Switch to TOPP-RA timing (after Lane C's C2 is on `main`)

Do:
- Implement `PathTimingBackend` with Lane C's native path timing.
- Enable `ToleranceBlend` where Lane C's timing supports it.
- Re-run B3–B7 acceptance.

Tests:
- identical geometry, with cycle time no worse than the simple backend on
  the M9 scenario;
- task-space and joint validation still pass.

## Out of scope for this lane

- Collision-aware free-space planning (OMPL);
- configuration selection over a whole path (Descartes-style ladder graph);
- seam tracking, weaving, force control and real process hardware.

## Progress log

### B1 — Add Cartesian process paths

Added framed pose lines and three-point arcs with per-point tolerances,
orientation policy, feed caps, and a rotation-to-length weight in metres per
radian for pure reorientation. Added a RobotKit Toolpath conversion that
preserves incoming-move feeds and emits process transitions at path distance
on a caller-named channel. Tests cover line, arc and rotation lengths,
evaluation, tolerance propagation, and three process spans producing six
events. The test was added before implementation. Commit: the commit
containing this entry.

MotionKit Haxe passed 5,873 assertions, RobotKit Haxe passed 4,437
assertions, MotionKit native passed 3 tests, RobotKit native passed 12 tests,
both FFI audits passed, and TCP integration passed in default, session and
lease-timeout modes.

### B2 — Events through plans and runtime

Added bounded timed events to native and Haxe execution plans, versioned
RobotKit submissions, declared process channels, and trajectory-clock event
delivery. HOLD makes configured channels safe and RESUME restores their last
fired values; STOP, ABORT, faults, and e-stop discard pending events and make
channels safe. A bounded fired-event ring feeds tool adapters and MCAP v5
recording, while earlier recording versions remain readable. Simulated
sprayer tests check scheduled timestamps, and native tests cover replacement,
undeclared channels, ring overflow, and safety transitions. Commit: the
commit containing this entry.

Reverified on the merged B5 tree: MotionKit Haxe passed 6,459 assertions,
RobotKit Haxe passed 4,539 assertions, MotionKit native CTest passed 4/4,
RobotKit native CTest passed 17/17, both FFI audits passed, and TCP integration
passed in default, session, and lease-timeout modes.

### B3 — MotionProgram compiler

Changed the C2 `FollowPath` field from `MotionPath` to `PosePath` in the shared
contract. `MotionPath` exposed only `length()` and also admitted geometric
paths without poses, so the arm compiler had to cast at runtime to obtain
framed pose samples. `PosePath` is the required typed input for arm path IK;
new line, arc, and spline primitives can implement its `PosePrimitive` API.

Added `ProgramCompiler` with exact-stop plan blocks and host-side dwell/input
barriers. Joint moves use Ruckig and pose candidates are ranked by joint
velocity-weighted distance. Cartesian moves are sampled, solved with seeded
IK, timed through `PathTimingBackend`, checked against joint bounds and the
authored task-space tolerance, and lowered to plans carrying timed process
events. A branch jump is rejected with the operation and path distance.
Tests cover a 6R pose target, Cartesian line, path events with lead time,
barrier splitting, joint bounds, a synthetic wrist branch jump, and a free
tool-axis rotation. `ToleranceBlend` records its exact-stop fallback.

Fixed Haxeon's nested array comprehension typing and lowering in its own
submodule commit so the MotionKit timing fixture can keep its nested array
expression. Haxeon regression and full test script passed. Commit: the
commit containing this entry.

Reverified on the merged B5 tree: MotionKit Haxe passed 6,459 assertions,
RobotKit Haxe passed 4,539 assertions, MotionKit native CTest passed 4/4,
RobotKit native CTest passed 17/17, both FFI audits passed, and TCP integration
passed in default, session, and lease-timeout modes.

### B4 — ManipulatorMotion facade

Extracted bounded trajectory-plan chunk submission, owner-clock progress
conversion, and smooth replacement retry from `MotionSystem` into
`PlanExecutor`. The executor also streams validated `ExecutionPlan` blocks,
including their timed process events and final `ends_at_rest` marker.
`ManipulatorMotion` compiles and runs programs, holds and resumes native
path time, aborts to declared safe outputs, exposes block/op/path-distance
progress and fired events, and handles dwell and input barriers with timeout
and diagnostic status. Program blocks retain the sampled distance/time map
used for path progress. Tests cover capability rejection, compiler errors,
two-block completion, timeout, FollowPath hold and delayed events, and safe
output after abort. Commit: the commit containing this entry.

MotionKit and RobotKit Haxe, both native suites, both FFI audits, and TCP
integration in default, session, and lease-timeout modes passed.

### B5 — Wall finishing through plans

`FinishSurface`, `Paint`, and `Sand` now run patch programs through
`ManipulatorMotion`: `MoveJ` approach, `FollowPath` process with timed channel
events, and `MoveL` retract. `ChannelToolAdapter` applies fired process records
to the simulated sprayer, and coverage uses observed joints and FK. The M9
scenario holds and resumes mid-patch, checks scheduled spray timing against
the owner clock, and confirms no paint during the hold. The synthetic DH arm
has only placeholder bounds boxes; its inferred self-contact trapped the slow
MuJoCo approach, so this fixture disables self collision while retaining
environment collision. Bounded plan chunks carry the preceding chord velocity
at linear continuation anchors. Sequential plans retain commanded endpoints,
and stopped velocity-controlled joints reanchor from observation.

After merging main, MotionKit Haxe passed 6,459 assertions, RobotKit Haxe
passed 4,537 assertions, and the MuJoCo M9 runner passed 64 assertions.
Default and MuJoCo coverage were both 99.60%, with zero opening coverage.
MotionKit native CTest passed 4/4, RobotKit native CTest passed 17/17, both
FFI audits passed, and TCP integration passed in default, session, and
lease-timeout modes. Commit: the commit containing this entry.

Lane C's `CornerBlender` can be evaluated for `ToleranceBlend` in B8 after
the TOPP-RA backend swap. B3's exact-stop fallback remains in place.

### B6 — Excavator toolpaths through plans

`DigTrench`, `GradeRegion`, and `DumpAt` now use a shared
`ToolpathPlanRunner` boundary backed by `ManipulatorMotion`. The four-joint
excavator follows validated joint moves through its authored TCP waypoints.
Its orientation manifold has an IK branch change across some straight TCP
segments, so a continuous Cartesian `FollowPath` cannot meet the authored
2 mm tolerance there. The cutting move is tracked by program operation;
`BucketSweep` reads observed joints and FK for the executed cut endpoints
and depth, with a 1 mm edge allowance for owner-cycle sampling. The trench
reaches grade in three cycles and removes 0.56 m³ within its existing 10%
volume bound. `DumpAt` and `GradeRegion` pass their existing acceptance.
The per-tick `ToolpathExecutor` API, its step/result/failure types, and
`CartesianTrajectory` were removed. Process and work tests now exercise
B1 path conversion and distance-based events.

MotionKit Haxe passed 6,459 assertions, RobotKit Haxe passed 4,522
assertions, and MuJoCo M9 passed 64 assertions with 99.60% coverage and zero
opening coverage. MotionKit native CTest passed 4/4, RobotKit native CTest
passed 17/17, both FFI audits passed, and TCP integration passed in default,
session, and lease-timeout modes. Commit: the commit containing this entry.
