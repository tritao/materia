# Motion stack: shared contracts (Phase 0)

The motion architecture work continues in three parallel lanes after the
completed `motionkit/IMPLEMENTATION_PLAN.md` (P1–P11, merged in `6592268c`):

- **Lane A — virtual device** (`LANE_A_VIRTUAL_DEVICE.md`): RKD6 scheduled
  protocol, virtual MCU, step generation, simulation loop.
- **Lane B — arm processes** (`LANE_B_ARM_PROCESS.md`): arm toolpaths as
  validated plans, path-synchronized process events, ProcessKit.
- **Lane C — planning math** (`LANE_C_PLANNING.md`): TOPP-RA path timing, IK
  solver interface and OPW, CncKit.

All three lanes are virtual: there is no hardware yet. Every design must still
be hardware-ready, meaning real hardware only adds a board layer and timing
measurements, not a redesign.

This file fixes the contracts that more than one lane touches. **Implement
§P0 first, in one session, and merge it to `main` before any lane starts.**
The lanes then work in parallel against these contracts. Changing a contract
later requires stopping, writing the reason in the changing lane's Progress
log, and updating this file in the same commit.

---

## C1 — Event model: process events tied to path progress

**Problem.** Process commands (spray on/off, flow, dispense rate, scanner
trigger) must happen at a *place on the path*, not at a wall-clock time. Today
`FinishSurface` switches the tool from software when a step is submitted and
passes the step index as its "timestamp"
(`robotkit/haxe/robotkit/skill/FinishSurface.hx:247`).

**Contract.**

1. **Authoring: path events** (Haxe, pure `motionkit`, package
   `motionkit.event`):
   - `PathEvent`:
     - `distance`: metres of arc length along the authored path, or the path
       parameter for joint paths;
     - `channel`: a stable string id, such as `"sprayer.flow"`;
     - `value`: `EventValue` — `Digital(Bool)`, `Analog(Float)` (SI units) or
       `Process(command:String, argument:Float)`;
     - `leadSeconds` (≥ 0): fire this much *earlier* than the path position,
       to compensate a known actuation latency such as spray trigger delay;
     - `holdPolicy`: `HoldPolicy` — `Keep`, `SafeWhileHeld` or
       `RestoreOnResume`.
   - Events sit on a path as a sorted list. They are data only.
2. **Lowering: timed events.** Once timing is known, the planner converts
   `distance` to path time with the path's time law, subtracts `leadSeconds`
   (clamped at the plan start) and produces a `TimedEvent`:
   - `timeNs`: plan-relative, **in path time**, i.e. the same clock as the
     trajectory segments;
   - `channel`, `value`, `holdPolicy`.

   `TimedEvent`s travel inside the `ExecutionPlan`. A plan with events
   requires a new capability bit, `RK_PLAN_CAPABILITY_EVENTS` (value 2).
3. **Execution semantics** (runtime, and later the device):
   - An event fires when the runtime's **trajectory clock** crosses
     `timeNs`. It never fires on wall time. HOLD slows the trajectory clock,
     so events are naturally delayed and stay attached to the path.
   - **On HOLD:**
     - `SafeWhileHeld` channels go to their declared safe value when the hold
       begins.
     - `RestoreOnResume` channels are set back to their last fired value when
       RESUME starts.
     - `Keep` channels are left unchanged.
   - **On STOP, ABORT, fault or e-stop:** unfired events are discarded, and
     every channel goes to its declared safe value.
   - **Replacement:** events at or after `replace_after` are discarded together
     with the segments. Events in the committed region are immutable, exactly
     like motion.
   - **Record:** each fired event is recorded with its plan id, the trajectory
     time it was scheduled for, and the owner-cycle time it was applied.
     Scheduled time is exact; application lags by at most one owner cycle
     host-side, and by less on the device once RKD6 carries events.
4. **Channels.** The deployment declares each channel's id, kind and safe
   value, in `deployment.json` or the simulation robot description. Every
   channel needs a safe value. A plan that references an undeclared channel is
   rejected at submit.
5. **Barriers are not events.** Waiting on feedback (`WaitReady`,
   `WaitInput`) splits motion into separate plans at the `MotionProgram`
   level (C2). It is never a timed event in the middle of a trajectory.
6. **Feed changes.** A feed override changes the trajectory clock rate, so
   path-time events move with it automatically. Whether the *process* adapts
   (for example flow proportional to speed) is decided by ProcessKit
   (Lane B). The runtime does not decide it.

**Ownership:**
- Lane B implements events in `ExecutionPlan`, the runtime queue, firing, the
  record, and the simulated tool adapters.
- Lane A carries them on the RKD6 wire, but only after Lane B's runtime part
  is on `main`.
- §P0 lands the data types and the reserved capability bit only.

## C2 — `MotionProgram`: controller-independent requested motion

**Contract** (Haxe, pure `motionkit`, package `motionkit.program`). A
`MotionProgram` is an ordered list of `MotionOp`:

| Op | Meaning |
| --- | --- |
| `MoveJ(target, limits, blend)` | Constrains the endpoint only. `target` is `JointTarget(Array<Float>)` or `PoseTarget(pose, frameId, configurationHint)`. |
| `MoveL(pose, frameId, feed, blend)` | Straight tool-point line with orientation interpolation (C3 policy). |
| `MoveC(via, end, frameId, feed, blend)` | Circular arc through `via`. |
| `FollowPath(path, frameId, timing, events)` | An authored path (lines, arcs, splines, or a process path from Lane B) with its `PathEvent`s. |
| `Dwell(seconds)` | Rest in place; it ends the plan block. |
| `SetOutput(channel, value)` | An event at the end of the previous motion. It does not end the block. |
| `WaitInput(channel, predicate, timeoutSeconds)` | A barrier. It ends the plan block, and the next block starts from rest. |

Rules:
- `blend` is `ExactStop | ToleranceBlend(metres)`, and only applies between
  consecutive moves in the same block.
- Frames are explicit ids, units are SI, and orientation is xyzw.
- **Compilation is split:**
  - pure `motionkit` validates the program structure;
  - `motionkit.robot` compiles it against a robot. It uses IK through the C4
    interface, timing through the C3 lowering rule, and produces one or more
    `ExecutionPlan`s per block. Streamed plans set `ends_at_rest = false`
    except for the last plan of each block.
- **Jogging is not a program op.** Jog is live control through `MotionSystem`.
- **Hold, stop and abort are not program ops.** They are session commands.
- CncKit (Lane C) and ProcessKit (Lane B) compile *into* `MotionProgram`.
  Neither submits plans directly.

**Ownership:**
- §P0 lands the pure data types plus structural validation.
- Lane B writes the first compiler, for `MoveJ` / `MoveL` / `FollowPath`.
- Lane C extends it for `MoveC` and CNC needs.

## C3 — Path trajectories reach the runtime as polynomial segments

**Contract.**
- **q(s) + s(t) is a planning-time form only.** Lane C's `PathTrajectory` (a
  joint path q(s), with s the path parameter, plus a time law s(t)) is never
  sent to the runtime.
- **Lowering.** `PathTrajectory` is converted to the existing canonical form:
  - piecewise polynomial segments (degree ≤ 5, shared boundaries, integer
    nanosecond knots);
  - fitted as cubic or quintic Hermite from exact q, q̇ and q̈ at the knots;
  - with adaptive knot insertion until the joint deviation from the exact
    q(s(t)) is below a stated tolerance.

  The runtime, the validator and RKD6 are unchanged.
- **Validation after lowering:**
  - Joint-space: the existing `mk_validate` on the lowered segments.
  - Task-space: the reserved `MK_CHECK_TASK_SPACE` slot is filled by the
    planner layer, which has forward kinematics (native `mk_validate` does
    not). It samples at a stated resolution and computes the tool-point
    deviation from the authored path. The report records the method
    (`sampled`) and the resolution, since exact extrema are not available in
    task space.

  Add `mk_report_set_task_space(report, status, worst, time, tolerance,
  resolution_ns)`, or the Haxe equivalent, so the slot is filled in one place.
- **Orientation policy** for Cartesian paths:
  - `Fixed`;
  - `Interpolated` (slerp by path fraction);
  - `Cone(axis, halfAngle)`: bounded deviation;
  - `FreeAboutTool`: rotation about the tool axis is unconstrained.

  The task-space check compares against the policy, not a single pose.
- **Events:** events use s(t) for their distance-to-time conversion (C1)
  *before* lowering. Lowering does not move events.
- **The single timing entry point** (pure `motionkit`, landed in §P0):

  ```haxe
  interface PathTimingBackend {
    /** Times joint path q(s) under limits and optional per-span speed
        caps, lowers it (C3) and returns the trajectory plus a
        distance-to-time map for event lowering (C1). */
    function time(path:JointPathSamples, limits:PathTimingLimits):TimedPath;
  }
  ```

  - `JointPathSamples`: monotonic `s` values, with `q`, `q'` and `q''` at
    each sample.
  - `PathTimingLimits`: per-joint velocity and acceleration limits, speed
    caps per `s` span, and start and end path speeds.
  - `TimedPath`: the lowered `Trajectory`, a `distanceToTime(s)` function,
    and the binding-constraint report (may be empty).
- **Until Lane C lands TOPP-RA:** Lane B may time arm paths with the existing
  simple method (per-segment trapezoid in path distance, then IK samples, then
  degree-1 or Hermite segments). It must go through the same lowering and
  validation entry point, so TOPP-RA can later be swapped in behind it.

## C4 — Kinematics solver interface

**Contract** (Haxe, pure `motionkit`, package `motionkit.kinematics`; native
backends behind it).

```haxe
interface KinematicsSolver {
  function jointCount():Int;
  /** Tool pose for joint vector q, in the solver's base frame. */
  function forward(q:Array<Float>):Pose3;
  /** One solution nearest `seed`, or null. */
  function solvePose(target:Pose3, seed:Array<Float>, tolerance:IkTolerance):Null<Array<Float>>;
  /** Up to `maxCount` distinct, joint-limit-valid solutions. Analytic
      solvers enumerate; numerical ones sample seeds deterministically. */
  function sampleCandidates(target:Pose3, maxCount:Int, tolerance:IkTolerance):Array<Array<Float>>;
  /** Joint velocity for a tool twist at q (least squares, damped). */
  function solveDifferential(q:Array<Float>, twist:Twist6):Null<Array<Float>>;
}
```

- `Pose3` and `Twist6` are pure MotionKit value types (position + xyzw
  quaternion; linear + angular). `motionkit.robot` converts them to and from
  RobotKit `Transform3` / `Twist3`. MotionKit must not import RobotKit.
- `sampleCandidates` replaces any promise of "all solutions". Redundant arms
  return samples.
- **Backends:**
  - an adapter over RobotKit's existing damped least-squares
    `Manipulator.solveIkForTcp` (in `motionkit.robot`), built by Lane B or §P0;
  - OPW analytic IK, native (Lane C).

  Callers depend on the interface only.

## C5 — Device core and board-layer boundary

**Contract** (Rust, `robotkit/device_protocol`).
- **The crate stays `#![no_std]`, with no heap.** The protocol, session,
  watchdog, clock mapping, segment queue and evaluator, event queue, stop
  logic and step generation all live in this crate. They are generic over a
  board trait. The existing `Device` trait (`src/runtime.rs`) is extended
  into the board layer:
  - outputs: joint or actuator targets, step pulses, digital and analog
    channels;
  - inputs: the device clock and state;
  - `stop_all`.
- **Board layers:**
  - **virtual**, in the same crate behind a `std`-only feature or in a
    sibling crate: a simulated clock with configurable drift and offset,
    simulated link, virtual stepper, and virtual channels;
  - **`boards/nucleo-g474re`**, later.

  CI must build the core for `thumbv7em-none-eabihf` (compile only), so
  anything that could never run on the MCU is caught early.
- **In-process virtual device.** The virtual device is also built as a
  `staticlib` with a small C ABI (`rkd_virtual_*`). A C++
  `VirtualDeviceEndpoint` can then run it *in-process* under the SimKit clock,
  deterministically, with a simulated link between the runtime's host link and
  the device core. The PTY process binaries remain for cross-process serial
  tests.
- **Numerics on the device:**
  - The G474 has a single-precision FPU, so device-side evaluation uses `f32`
    with segment-local time τ.
  - The host device compiler converts segments to their `f32` wire form and
    **re-validates the converted trajectory** at the device's time resolution
    (`executor_time_resolution_ns`) and within the deployment's position error
    budget (`target_error`). That is "validate what executes" at the last
    lowering step.

---

## Cross-lane rules

- **One worktree and branch per lane**, created from `main` *after* §P0
  merges:
  - `../materia-lane-a` on `lane-a-virtual-device`;
  - `../materia-lane-b` on `lane-b-arm-process`;
  - `../materia-lane-c` on `lane-c-planning`.

  Use the worktree setup in `motionkit/IMPLEMENTATION_PLAN.md` §Ground rules:
  - clone haxeon from the main checkout and check out the pinned commit;
  - copy `haxeon/.tools/` and `haxeon/out/haxeon_runtime.hdll`;
  - initialise `nativekit` and `simkit/vendor/mujoco` as needed.
- **Merge to `main` at the end of each lane item** that passes every suite, so
  the other lanes pick changes up early.
  - Before merging, merge `main` into the lane branch and re-run every suite.
  - Don't let a lane run more than a few items ahead of `main`.
- **ABI version bumps** (`RK_API_VERSION`, `MK_API_VERSION`, RKD protocol
  version) conflict across lanes. On conflict, take `max(both) + 1`,
  regenerate the `.hxi` with the lane's `check-hxi.sh`, and re-run the FFI
  audits.
- **Shared files touched by several lanes:**
  - `robotkit/runtime/src/runtime.cpp`: Lanes A and B;
  - `robotkit/runtime/include/robotkit_runtime.h`: Lanes A and B;
  - `motionkit/robot/.../MotionSystem.hx`: Lanes B and C;
  - `robotkit/haxe/robotkit/world/*`: Lanes A and B.

  Keep changes there small and in their own commits, which makes conflicts
  cheap.
- **haxeon.** A haxeon fix is its own submodule commit with a haxeon test.
  Before merging a lane that moved the pin, the pinned commit must exist in
  the main checkout's haxeon. Fetch it by **full SHA**:
  `git -C <main>/haxeon fetch <lane>/haxeon <full-sha>`. If `main` also moved
  haxeon, make a haxeon merge commit first.
- **Test commands:** as in `motionkit/IMPLEMENTATION_PLAN.md`. Every item
  leaves every suite green, including both FFI audits and `world-tcp.sh` in
  default, session and lease-timeout modes.
- **No hardware.** Nothing may require a physical device, serial port or
  network beyond localhost.

---

## §P0 — Land the shared contract types (one session, before the lanes)

Work on branch `motion-phase0` in `../materia-motion-phase0`, which already
exists and holds these plan files. Follow the ground rules of
`motionkit/IMPLEMENTATION_PLAN.md`: failing test first, one commit per item,
stage explicit paths, never push, and log to the Progress log at the end of
this file.

- **P0.1 — Event types.**
  - `motionkit.event`: `PathEvent`, `TimedEvent`, `EventValue`, `HoldPolicy`,
    and `ChannelDeclaration` (id, kind, safe value). Immutable, validated
    constructors: finite values, `distance ≥ 0`, `leadSeconds ≥ 0`, non-empty
    channel ids.
  - Reserve `RK_PLAN_CAPABILITY_EVENTS = 2` in `robotkit_runtime.h` with a
    comment pointing at C1. The runtime must reject plans requiring it (it
    can't execute events yet). Test the rejection.
- **P0.2 — `MotionProgram` types.**
  - `motionkit.program`: `MotionProgram`, `MotionOp` (enum per C2),
    `MoveTarget`, `Blend`, and structural validation with a clear error. The
    validation covers:
    - non-empty programs;
    - frame ids present;
    - finite feeds greater than 0;
    - blends only between moves;
    - `WaitInput` timeouts greater than 0.
  - No compiler.
- **P0.3 — Kinematics interface.**
  - `motionkit.kinematics`: `KinematicsSolver`, `IkTolerance`, `Pose3`,
    `Twist6` per C4.
  - `motionkit.robot.ManipulatorKinematics`: an adapter over RobotKit
    `Manipulator` (forward via `tcpPose`, `solvePose` via `solveIkForTcp`,
    `solveDifferential` via the chain Jacobian with damped least squares, and
    `sampleCandidates` from a deterministic seeded grid of seeds, deduplicated
    by joint distance).
  - Tests:
    - forward/solve round-trip on the existing 6R wall-finishing arm fixture;
    - deterministic candidate sampling (same inputs give identical output);
    - differential solve matches a finite-difference check.
- **P0.3b — Path timing entry point.**
  - `motionkit.planner.PathTimingBackend`, `JointPathSamples`,
    `PathTimingLimits` and `TimedPath` per C3.
  - A reference backend `SimplePathTiming`:
    - a trapezoid in path distance with per-span caps;
    - the tightest joint-derived speed and acceleration limit along the path
      (using `q'` and `q''`);
    - degree-1 lowering through `Trajectory.fromPositionSamples` at a stated
      sample period, with the exact `distanceToTime`.
  - Tests:
    - a straight joint path respects the joint limits;
    - `distanceToTime` is monotonic and hits the endpoints exactly.
- **P0.4 — Task-space report slot writer.** Add `mk_report_set_task_space`
  per C3 (native, C ABI, `MK_API_VERSION` bump) and its Haxe wrapper method.
  Regenerate the binding. Tests: the report round-trips status, worst value,
  tolerance and resolution; unset remains `unchecked`.
- **P0.5 — Verify `main`.** Run the full suite list on the merged tree.
  Record the result and assertion counts in the log. If anything fails, fix
  it in its own commit before merging P0.

Then merge `motion-phase0` to `main` (the owner does this or approves it),
and the three lanes start.

## Progress log

### P0.1 — Add process-event contract types

Added immutable path/timed event and channel-declaration values with finite,
non-negative and non-empty validation, including safe-value kind checks.
Reserved runtime capability bit 2 for events, regenerated the RobotKit FFI
binding, and verified that plans requiring the reserved bit are rejected until
event execution lands. The constructor and runtime tests failed before the
implementation and now pass. Commit: the commit containing this entry.

The pinned NativeKit commit was absent from its configured remote, so the
worktree dependency was fetched from the shared checkout at the exact recorded
SHA. MotionKit and RobotKit Haxe suites, all 12 native tests, the RobotKit FFI
audit, and TCP integration in default, session and lease-timeout modes passed.

### P0.3 — Add the kinematics solver contract

Added pure MotionKit pose, twist and IK-tolerance values plus the
`KinematicsSolver` interface. `ManipulatorKinematics` adapts RobotKit TCP FK
and IK, samples candidates from a deterministic joint-limit grid with
joint-distance deduplication, and solves TCP differential motion with a damped
least-squares chain Jacobian shifted from flange to tool point. Tests cover a
6R forward/solve round trip, repeatable candidates and a central-difference
twist check; they failed before the implementation and now pass. P0.3 landed
before P0.2 because the latter's typed `MotionOp` variants depend on `Pose3`.
Commit: the commit containing this entry.

MotionKit passed 4,995 assertions, RobotKit passed 4,437 aggregate assertions,
all 12 native tests passed, and TCP integration passed in default, session and
lease-timeout modes.

### P0.2 — Add validated motion programs

Added the controller-independent `MotionProgram` and typed operations for
joint, linear, circular and authored-path motion, dwell, output and input
barriers. Added joint/pose move targets, exact/tolerance blends, serializable
input predicates and a small path interface that current and future authored
paths can implement. Structural validation reports the operation index and
checks non-empty programs and joint targets, framed poses, positive finite
feeds/dwell/timeouts, sorted in-range events, typed values and blends only
between consecutive moves. Tests were written first and now pass. Commit: the
commit containing this entry.

MotionKit passed 5,004 assertions, RobotKit passed 4,437 aggregate assertions,
all 12 native tests passed, and TCP integration passed in default, session and
lease-timeout modes. The session integration's first run missed the latched
e-stop during reconnect; an immediate unchanged rerun passed, as it did before
this item.

### P0.3b — Add the path-timing entry point

Added the pure MotionKit `PathTimingBackend`, sampled joint-path and limit
values, and `TimedPath` result with an exact distance-to-time map. The
`SimplePathTiming` reference backend derives conservative path-speed and
path-acceleration bounds from each joint's q' and q'', applies authored
per-span caps and endpoint speeds, and lowers the resulting trapezoids to a
degree-1 trajectory at a declared sample period. Its report identifies the
constraint that set each span's speed bound. Tests were written first and now
cover joint limits, cap enforcement, monotonic endpoint-exact event timing and
invalid inputs. Commit: the commit containing this entry.

MotionKit passed 5,857 assertions, RobotKit passed 4,437 aggregate assertions,
all 12 native tests passed, both FFI audits passed, and TCP integration passed
in default, session and lease-timeout modes on the combined branch tree.

### P0.4 — Add the task-space report writer

Added `mk_report_set_task_space` and the Haxe `ValidationReport.setTaskSpace`
wrapper. The task-space check now records sampled method, worst deviation,
time, tolerance, signed margin and sampling resolution; an untouched slot
remains unchecked with zero resolution. Bumped `MK_API_VERSION` to 6 and
regenerated the Haxe binding. Native and Haxe tests cover unset state and
round-tripping the sampled result fields. Commit: the commit containing this
entry.

MotionKit passed 5,864 assertions, RobotKit passed 4,437 aggregate assertions,
all 12 native tests passed, both FFI audits passed, and TCP integration passed
in default, session and lease-timeout modes.
