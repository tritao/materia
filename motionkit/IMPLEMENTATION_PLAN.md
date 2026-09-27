# MotionKit architecture: implementation plan (handoff)

This plan takes MotionKit and the RobotKit runtime from where they are today to
the first increments of the target motion architecture. It fixes four
architectural decisions up front, then lists self-contained work items. Each
item states the problem, where the code is, what to change, and how to prove it.

`robotkit/MOTION_ARCHITECTURE_GAP_MAP.md` records what exists today and why
each item is needed. Read it first. It refers to the architecture proposal by
section number. That proposal is background only: this file is the contract,
and where they differ, this file wins.

---

## 0. Read before writing code

1. `robotkit/MOTION_ARCHITECTURE_GAP_MAP.md` (current state).
2. `motionkit/README.md`, then all of `motionkit/haxe/motionkit/` (about 2.5k
   lines). `MotionSystem.hx` is the heart: hold and resume by re-timing,
   chunk streaming, splices, jogs.
3. `robotkit/ARCHITECTURE.md`: "Core contracts", "Ownership", "Joint command
   batches", "Deployment boundary". Its frames, units and time rules are
   normative (SI units, `A_T_B`, xyzw quaternions).
4. `robotkit/CONSTRUCTION_ROADMAP.md` §0 and "Ground rules". These two
   still apply:
   - The native runtime, robotd's C side and the device protocol stay
     **joint-space only**: no Cartesian poses, IK, tools or work geometry.
   - haxeon quirks:
     - no class → anonymous-type structural subtyping;
     - interfaces declare only functions;
     - reduced `Math`, so grep `haxeon/stdlib/Math.hx` before using a
       `Math.*` member.
     - If haxeon lacks something, fix haxeon (separate commit, with a haxeon
       test) rather than working around it in kit code. Read the compiler
       source to find the root cause of an unclear error first.
5. The runtime C API conventions in `robotkit/runtime/include/robotkit_runtime.h`:
   - opaque handles;
   - `struct_size`-versioned structs;
   - `RK_ERROR_*` codes;
   - `RK_API_VERSION` bumps;
   - FFI bindings regenerated with `robotkit/runtime/tools/check-hxi.sh`
     (a wrapper around `haxeon/scripts/haxeon-ffi-audit`). Never hand-edit
     a `.hxi`.

## Test commands (from the repo root)

- MotionKit (Haxe, native-backed):
  `./haxeon/scripts/haxeon run --project motionkit/tests/haxeon.json`
- RobotKit Haxe: `./haxeon/scripts/haxeon run --project robotkit/tests/haxeon.json`
- RobotKit native:
  `cmake -S robotkit -B robotkit/build -GNinja -DCMAKE_BUILD_TYPE=Debug && cmake --build robotkit/build -j 6 && ctest --test-dir robotkit/build --output-on-failure`
- Integration: `robotkit/tests/world-tcp.sh`, also with
  `ROBOTKIT_TEST_SESSIONS=1` and `ROBOTKIT_TEST_LEASE_TIMEOUT=1`.
- From P3 onward, MotionKit native:
  `cmake -S motionkit/native -B motionkit/native/build -GNinja -DCMAKE_BUILD_TYPE=Debug && cmake --build motionkit/native/build && ctest --test-dir motionkit/native/build --output-on-failure`

Every item must leave every suite above green.

## Ground rules

- **Worktree, not the shared checkout.** `/home/joao/dev/materia` is shared by
  concurrent sessions, and its branch can change under you. Set up a worktree:
  1. `git worktree add ../materia-motion-architecture -b motion-architecture main`.
  2. `git clone /home/joao/dev/materia/haxeon ../materia-motion-architecture/haxeon`,
     then check out the commit pinned by `git ls-tree main haxeon`. The pinned
     haxeon commit is often local and unpushed, so `git submodule update` fails.
  3. Copy `haxeon/.tools/` and `haxeon/out/haxeon_runtime.hdll` from the main
     checkout; they are gitignored.
  4. Run `git submodule update --init nativekit simkit/vendor/mujoco` as
     needed.
- One commit per item (more if the item says so), imperative subject in the
  log's style, each commit building and passing tests. Stage explicit paths
  only. Never stage build output, and never push.
- Write the failing test first for every behaviour change.
- Deterministic code only: no wall clock in planners, seeded RNG only.
- Do not spawn sub-agents.
- Append a "Progress log" entry to the end of this file for each item: what
  changed, the commit, and anything surprising.
- **If an item shows this plan is wrong, stop.** Write down what you found in
  the log and make the smallest correct adjustment. Do not silently expand
  scope.
- Do not touch `exosuit/`, or the scenekit shader files that are modified in
  the main checkout. They belong to other work.

---

## Architectural decisions

These are settled. Implement them; do not re-open them without writing the
reason in the Progress log and stopping.

### D1 — Numerical motion code is native C++; Haxe owns API and orchestration

MotionKit gets a native library, `motionkit/native` (C++17, C ABI, haxeon FFI
bindings). Trajectory representation, evaluation, limit validation,
state-to-state generation (Ruckig) and, later, path timing (TOPP-RA), IK
backends (OPW) and QP solvers live there. Haxe keeps the public API, the
paths as authored data, orchestration (`MotionSystem`), and the MachineKit
bridge.

Why:

- Every library the architecture reuses is C++.
- The executor (`robotkit/runtime`) is C++ and must evaluate trajectories
  exactly as the planner and validator do. Sharing one evaluator makes
  "validate what the backend executes" true by construction instead of by
  test.
- Future device compilers live next to the runtime.
- haxeon's reduced `Math` makes numerical Haxe code costly.

Existing Haxe planners (`TrapezoidalPlanner`, `LineLookaheadPlanner`,
`JogProfile`, `TimeScaling`) stay in use until a native replacement lands.
Each is then **deleted, not kept in parallel**. This plan replaces
`TrapezoidalPlanner`, `JogProfile` and `TimeScaling`. `LineLookaheadPlanner`
is replaced later by native path timing, which is out of scope here.

Dependency direction: the C++ `robotkit_runtime` links `motionkit_core`. The
reverse never happens: `motionkit/native` depends on nothing in the repo.

### D2 — Transmissions live in the RobotKit model

The authoritative `RobotModel` gains actuators with explicit transmissions
(joint ↔ actuator mapping: ratio, offset, and coupling for several actuators
on one joint). Consequences:

- A dual-drive gantry becomes **one joint with two actuators**.
- The runtime converts joint targets into actuator targets at the endpoint
  boundary.
- Device channels map to actuators, not joints.
- MotionKit's `MotionAxisBlueprint.jointScales/jointOffsets` are then derived
  from the model, and later removed. MotionKit axes remain task coordinates
  only.

Why: an axis's gear or lead-screw ratio is a fact about the machine, which
the device, simulation and diagnostics all need. Today it lives only in a
MotionKit convenience object. Whatever planning layer is used, the runtime
must enforce limits in both joint and actuator units.

This plan does the model and compiler part (P11). The runtime and device part
is design-only here, because it changes the device layout and fingerprint.

### D3 — `ExecutionSession` is a native runtime object

The execution session lives in the C++ `RobotRuntime`, next to the queue it
owns. It covers plan identity, revision checks, committed horizon,
replace-after, hold/resume/controlled stop/abort, and progress. Haxe gets a
thin handle, and `MotionSystem` becomes a client of it. robotd and in-process
callers then share one implementation. Path-preserving hold and resume move
from host-side re-timing (`TimeScaling`) into the runtime, which already
slows its trajectory clock for STOP.

Why:

- The runtime already owns the queue, tags, splices and the path-following
  stop.
- A Haxe session would duplicate that state across a process boundary (robotd).
- Hold latency would depend on host scheduling.

### D4 — The canonical joint trajectory is piecewise polynomial

`JointTrajectory` becomes a sequence of segments. Each segment has:

- start time `t0_ns` (int64 nanoseconds, matching the runtime clock);
- duration `duration_ns`;
- degree `d ≤ 5`;
- for each joint, coefficients `c[0..d]` of `p(τ) = Σ c_k τ^k`, with τ in
  seconds from the segment start.

Segment boundaries are shared across joints. Consequences:

- Position samples in the existing runtime chunk path are the degree-1
  special case, so their position interpolation maps onto it without a
  behaviour change. Haxe `JointTrajectory.sample` separately interpolates
  authored velocity and acceleration arrays; those derivative values are not
  equivalent to derivatives of the degree-1 position polynomial.
- Trapezoids are exact at degree 2 and Ruckig at degree 3; degree 5 is
  reserved for quintic blends.
- Position, velocity, acceleration and jerk are all analytic, so validation
  can find exact extrema instead of sampling.
- `PathTrajectory` (q(s) plus s(t)) is added later as a second canonical
  form. It is out of scope here.
- The future RKD scheduled protocol carries these segments directly, since
  polynomial segments are its first encoding.

Why: a sampled-only representation can't express jerk-limited output
exactly, can't be validated for acceleration or jerk, and would force
resampling at every layer.

---

## Work items

Suggested order is as listed. P1–P2 do not depend on the decisions and can
land first.

## P1 — Split MotionKit's pure core from its RobotKit adapter

Problem: `motionkit/haxeon.json` depends on `robotkit` and `machinekit`. Only
three files cause this:

- `motionkit/haxe/motionkit/MotionSystem.hx`
- `motionkit/haxe/motionkit/axis/MotionSystemBlueprint.hx`
- `motionkit/haxe/motionkit/MachineKitRobotCompiler.hx`

Do:
- Create a haxeon project `motionkit/robot/haxeon.json` (package name
  `motionkit-robot`) with source root `haxe`. Its dependencies are `motionkit`,
  `robotkit` and `machinekit`.
- Move the three files into it under the Haxe package `motionkit.robot`
  (`motionkit.robot.MotionSystem`, `motionkit.robot.MotionSystemBlueprint`,
  `motionkit.robot.MachineKitRobotCompiler`). Use `git mv` so history follows.
- Remove `robotkit` and `machinekit` from `motionkit/haxeon.json`.
- Point `motionkit/tests/haxeon.json` at the new project and fix imports.
- Update the README examples and the gap map's §1.

Tests: all existing MotionKit tests pass unchanged, apart from imports. Add
a check (a small script or test step) that `grep -r "import robotkit\|import machinekit" motionkit/haxe`
finds nothing.
Acceptance: `motionkit` builds with no RobotKit or MachineKit dependency.

## P2 — Runtime rejects trajectory chunks its endpoint cannot execute

Problem: the runtime accepts `RK_COMMAND_TRAJECTORY_CHUNK` even when
`endpoint_->supports_trajectory_queue()` is false (as for
`DeviceSerialEndpoint`). Only the Haxe adapter checks the capability. The
plan's rule is that backends reject unsupported guarantees.

Do: in `robotkit/runtime/src/runtime.cpp` (the submit and validation paths),
return `RK_ERROR_UNSUPPORTED` for a trajectory chunk when the endpoint lacks
queue support, before anything is queued. Document it in the header comment
of `rk_robot_runtime_submit_trajectory`.

Tests: a native test with an `InMemoryRobot`-style endpoint whose queue
support is false. Submitting a chunk returns `RK_ERROR_UNSUPPORTED`, the
queue stays empty, and no fault is latched.

## P3 — `motionkit/native` library with the polynomial trajectory (D1, D4)

Do:
- `motionkit/native/CMakeLists.txt`: a library `motionkit_core`
  (shared/static option matching `RK_BUILD_SHARED`), with ctest tests. It has
  no dependency on anything else in the repo.
- Public C header `motionkit/native/include/motionkit.h`, following the
  RobotKit C conventions:
  - opaque handles, `struct_size`, error codes (`MK_ERROR_*`) and
    `MK_API_VERSION`;
  - no exceptions across the ABI.
- C++ implementation behind it. C++ callers such as the runtime may use a
  C++ header `motionkit.hpp`; the C ABI exists for Haxe.
- Trajectory API:
  - `mk_trajectory_create(joint_count)`;
  - `mk_trajectory_append_segment(t, duration_ns, degree, coefficients[joint][degree+1])`,
    which rejects non-finite values, zero or negative durations, degree > 5,
    and a start that isn't the previous segment's end;
  - `mk_trajectory_evaluate(t, time_ns, out position/velocity/acceleration/jerk)`;
  - `mk_trajectory_duration_ns`, `mk_trajectory_segment_count`,
    `mk_trajectory_destroy`;
- `mk_trajectory_from_samples(times_ns, positions)`, which builds degree-1
  segments equivalent to today's linear position interpolation. Its velocity
  is each segment's chord velocity and its acceleration and jerk are zero.
- Continuity report: the maximum C0, C1 and C2 jump at each boundary,
  reported rather than enforced.
- Bindings:
  - `motionkit/native/bindings/motionkit_import.h`;
  - `motionkit/native/tools/check-hxi.sh`, modelled on
    `robotkit/runtime/tools/check-hxi.sh` with library `motionkit_core` and
    interface `MotionKitNative`;
  - the generated `.hxi` registered under `"ffi"` in `motionkit/haxeon.json`.
- Haxe wrapper `motionkit.trajectory.Trajectory`, which owns a native handle
  and provides `evaluate`, `durationSeconds`, `jointCount` and
  `fromJointTrajectory(JointTrajectory)` (degree 1). The existing sampled
  `JointTrajectory` stays for now; this conversion preserves sampled positions
  and times, not its independently stored velocity and acceleration arrays.
- Add `motionkit/native` to `robotkit/robotd/native/CMakeLists.txt` (and to
  the native `inputs` lists in the haxeon.json files that build it), so Haxe
  tests load the library.

Tests (native):
- evaluating a hand-built cubic matches the analytic values;
- `from_samples` positions match Haxe `JointTrajectory.sample` on the same
  data to 1e-12; native velocity equals each chord slope, and native
  acceleration and jerk are zero within each segment;
- boundary lookup at exact knot times is right-continuous and deterministic;
- the rejection cases above.

Haxe: a round-trip test through the wrapper.

## P4 — Limit validation and `ExecutionPlan` (native)

Do:
- `mk_validate(trajectory, limits, out report)`. `limits` holds, per joint:
  position lower/upper, and max absolute velocity, acceleration and jerk.
  Zero means "not claimed", which is recorded as unchecked, not as passed.
- Extrema must be exact:
  - For each segment and derivative order, find the critical points (roots
    of the next derivative in the open interval) plus the endpoints.
  - Solve in closed form up to cubic.
  - For quartic, bracket sign changes on 64 subintervals and refine by
    bisection plus Newton to 1e-12 s.
- `ValidationReport` records, for each check (position, velocity,
  acceleration, jerk, continuity):
  - `passed | failed | unchecked`;
  - the worst value;
  - its joint, time and limit.

  It also records the model revision, calibration revision and trajectory
  revision it was computed for, plus a list of unresolved assumptions
  (strings).
- Limit comparisons use an explicit tolerance, not zero slack. For derivative
  order *n*, use `max(1e-9 × abs(limit), max_abs_derivative(n+1) × 0.5 ns)`;
  continuity compares two rounded sides, so its quantization bound uses 1 ns.
  Store the signed limit margin and tolerance alongside each worst value in
  `ValidationReport`. Analytic extrema and genuinely out-of-limit moves still
  fail. This is the user-approved P5 nanosecond-boundary correction.
- `ExecutionPlan` (native object, Haxe wrapper) contains:
  - `planId` (u64);
  - `modelRevision` and `calibrationRevision`;
  - the trajectory;
  - start-state assumption: joint positions, velocities and accelerations
    with tolerances;
  - required capabilities: a bit set for now, with only `TIMED_TRAJECTORY`
    defined;
  - `planningAuthority` (`MATERIA` only for now; `BACKEND` reserved);
  - the validation report.

  Creating a plan runs validation. A plan whose report has any `failed`
  check cannot be created (error `MK_ERROR_LIMIT`, report still returned).
- Calibration revision: add `calibration_revision` (u64, default 0) to
  `rk_robot_blueprint` and the snapshot, next to `revision`, via the
  `struct_size` pattern, and bump `RK_API_VERSION`. Also add
  `calibrationRevision` to the Haxe `RobotRuntimeCompiler.compile` inputs.
  Nothing produces a nonzero value yet; this reserves the identity.

Tests:
- a cubic whose velocity peaks mid-segment above the limit fails, at the
  exact analytic time and value;
- the same trajectory at a lower peak passes;
- an unclaimed jerk limit shows `unchecked`;
- a position extremum between samples is caught;
- plan creation is refused on failure.

## P5 — Ruckig state-to-state generator (native)

Do:
- Add Ruckig Community as a git submodule at
  `motionkit/native/vendor/ruckig`, pinned to a release tag. Record the tag
  and its MIT licence in `motionkit/native/THIRD_PARTY.md`.
  - Audit the files actually compiled. Do not build or link its cloud client:
    no network code enters the build.
  - If there is no network access, stop and log it. Do not vendor by copy.
- `mk_generate_state_to_state(current pos/vel/acc, target pos/vel/acc,
  limits vel/acc/jerk, synchronization = time, out trajectory)`.
  - Convert Ruckig's per-DOF profiles (brake pre-trajectory plus seven
    constant-jerk phases) into degree-3 segments on the union of all DOFs'
    phase boundaries. Drop sub-nanosecond slivers deterministically.
  - The result must be the degree-3 `Trajectory` from P3.
- Errors: map Ruckig result codes to `MK_ERROR_*`, and put the Ruckig code in
  an out-parameter.

Tests (native), each also run through `mk_validate` with all limits claimed:
- The converted trajectory matches unmodified Ruckig's own `at_time` at 1 kHz
  within 1e-9 for position, velocity and acceleration. Within ±1 ns of a
  rounded phase boundary, use the corresponding derivative quantization
  bound for that component.
- Nonzero initial velocity.
- Nonzero initial acceleration.
- Reversal (initial velocity away from the target).
- Retarget mid-motion, starting from a state evaluated on a previous
  trajectory.
- Stop: target velocity and acceleration zero, target position free. Use
  Ruckig's velocity control interface for "stop as fast as allowed".
- A start state above the velocity limit (brake phase).
- Travel-limit extrema: a move whose overshoot would cross the position
  limit is reported `failed` by validation, not clipped.

## P6 — Runtime executes polynomial segments (D4)

Problem: the runtime queue (`ControlState::trajectory`, a deque of
`RuntimeTrajectoryPoint`) stores linear samples.

Do:
- Link `robotkit_runtime` to `motionkit_core`.
- Change the queue's element to a segment:
  - start time in the chunk's time base, plus duration, degree and
    coefficients;
  - the chunk tag;
  - evaluation through `motionkit.hpp`, so the runtime has no evaluator of
    its own.
- Convert existing `RK_COMMAND_TRAJECTORY_CHUNK` points to degree-1 segments
  on submit. Their external behaviour must not change.
- Add `RK_COMMAND_TRAJECTORY_SEGMENTS` with a `rk_trajectory_segment_chunk`
  struct carrying the same `tag/splice_tag/splice_time_ns`. Bound it by
  segments and by total coefficients. Keep `RK_MAX_TRAJECTORY_QUEUE_POINTS`
  semantics as a limit on queued segments, and document it.
- Submit-time checks:
  - replace the chord-velocity check with `mk_validate` over the new
    segments plus the junction to the queue's end;
  - check position and velocity limits exactly as now;
  - check acceleration when `max_acceleration` is set;
  - a violation still latches a fault and returns `RK_ERROR_LIMIT`.
- Path-following STOP: use the analytic path velocity and acceleration from
  the segments instead of chord differences. Keep the same acceleration
  budget rule.
- Bump `RK_API_VERSION`, regenerate the `.hxi`, and add `TrajectoryChunk`
  support for segments in Haxe (`robotkit.world`).

Tests:
- All existing runtime and MotionKit queue tests pass unchanged. This is
  the degree-1 equivalence check.
- A Ruckig trajectory submitted as segments is executed; the sampled runtime
  targets match `mk_trajectory_evaluate` exactly (same code path).
- A splice into a segment queue cuts mid-segment correctly.
- A segment chunk that exceeds the acceleration limit is rejected.

## P7 — `ExecutionSession` in the runtime: identity, revisions, committed horizon (D3)

Do:
- C API `rk_robot_runtime_submit_plan(runtime, plan_submission)`.
  - The submission carries plan id, model and calibration revision,
    required-capability bits, start-state assumption, the segments, and an
    optional `replace_after` (plan id plus time).
  - The runtime rejects the submission:
    - with `RK_ERROR_MODEL_MISMATCH` if the revisions do not match the
      blueprint;
    - with `RK_ERROR_UNSUPPORTED` if a required capability is missing;
    - with `RK_ERROR_INVALID_STATE` if the start state is outside tolerance
      of the queue end, or of the current state when idle.
  - Validation reuses P6.
- **Committed horizon.**
  - The runtime defines `committed_until_ns` as the owner-clock time before
    which queued motion may no longer change. It equals the current
    trajectory time plus a configured `commit_lead_ns` (blueprint field;
    default two owner periods).
  - The snapshot publishes `committed_until_ns`, the active plan id and the
    queue end time.
  - A `replace_after` earlier than `committed_until_ns` is **rejected with
    an error**, not silently dropped as splices are today.
  - Existing splice semantics stay for the old chunk command. Document the
    difference.
- Session state in the snapshot: `idle | executing | holding | held |
  stopping | faulted`.
- Haxe: `robotkit.world.ExecutionPlanSubmission` and snapshot fields. Keep
  it a thin mapping.

Tests:
- A stale model revision is rejected, with nothing queued.
- A mismatched start state is rejected.
- A replacement before the committed horizon is rejected, and a later one is
  applied.
- Committed motion is byte-identical before and after an accepted
  replacement. Evaluate the committed region at 1 kHz before and after.
- Replay:
  - record a session with the existing MCAP recorder (`RecordingRobot`);
  - replay it through `ReplayRobot`;
  - confirm the published progress sequence matches.

## P8 — Native hold / resume / controlled stop / abort (D3)

Problem: only `STOP` and `EMERGENCY_STOP` exist in the runtime. Hold and
resume are simulated host-side by `MotionSystem` with `TimeScaling`.

Do:
- Add `RK_COMMAND_HOLD` and `RK_COMMAND_RESUME`.
  - HOLD slows the trajectory clock rate to 0 along the path using the same
    acceleration budget as STOP, but **keeps** the queue.
  - RESUME ramps the rate back to 1 within the same budget.
  - Both are joint-acceleration-limited using the analytic segment
    derivatives.
- STOP keeps its current meaning: a controlled stop along the path, after
  which the queue is cleared.
- Add an explicit ABORT distinct from e-stop: an immediate controlled stop
  via the straight ramp, then the queue is cleared, with no safety latch.
  Otherwise document why STOP already covers it. Decide by reading the
  current STOP fallback, and record the decision.
- Underflow: when the queue ends while the path is still moving (nonzero
  velocity at the last segment end), finish with the acceleration-limited
  straight ramp instead of holding the last point. Record a fault reason
  `trajectory_underflow`, which is not latched.
- A hold that runs out of queued path finishes on the ramp, as STOP does
  today.

Tests:
- Hold mid-move: the path is preserved (positions stay on the planned
  trajectory's image), limits hold, and the rate reaches 0.
- Resume returns to rate 1 and finishes at the planned end pose.
- A hold during acceleration or deceleration phases stays within limits
  (mirror the existing MotionKit hold tests).
- Underflow at speed is ramped and flagged.
- The existing MotionKit hold and resume tests pass once P9 switches over.

## P9 — MotionSystem on sessions and Ruckig; delete the replaced Haxe planners

Do (in `motionkit/robot/…/MotionSystem.hx`):
- When the runtime supports plans, submit `ExecutionPlan`s through P7 and
  use the native HOLD and RESUME from P8. Remove the host-side re-timing path
  for queue backends.
- `moveAxes`, `queueAxes` and `home` plan with Ruckig (P5), starting from
  the runtime-reported current state.
- An immediate axis move, jog or home while executing degree ≥ 2 segments
  **retargets from the moving state** with
  `replace_after = committed_until`. `moveAxes`, `jog` and `home` return a
  plan instead of null in that case. Update the README's description of this
  behaviour. `moveLinear` and path moves keep stop-first behaviour until
  native path timing exists.
- `movePath` and `queuePath` keep `LineLookaheadPlanner` (converted to
  degree-1 segments) until native path timing exists.
- **Derivative sources.** Never derive a retarget start state or hold lead
  from a degree-1 trajectory's analytic derivatives.
  - Axis moves, jogs and homing use Ruckig (degree 3), whose derivatives are
    exact.
  - `LineLookaheadPlanner` output stays degree 1 until native path timing
    exists. Line phases could be emitted as exact degree-2 segments, but arcs
    cannot.
  - For those trajectories, retargeting while moving keeps today's stop-first
    behaviour. The hold lead uses the planner's authored velocities, kept
    alongside the native trajectory, or a conservative bound from the chord
    velocity.
  - Retargeting from a moving state applies only when the active segments
    are degree ≥ 2.
- Fallback for backends without queue or plan support (serial device
  today):
  - keep position-target streaming;
  - evaluate the native trajectory each fixed step;
  - hold and resume become "stop and replan with Ruckig" there. Document
    this as the degraded mode.
- Delete `TrapezoidalPlanner`, `JogProfile`, `TimeScaling` and
  `TimeScaledTrajectory` once nothing uses them. Keep `MotionLimits`.
  Tests that assert those classes' specific behaviour are replaced by
  equivalent behaviour tests on `MotionSystem`.

Tests:
- The existing MotionSystem suite passes, with assertions updated only
  where the documented behaviour intentionally changed (retarget instead of
  stop-then-move).
- New tests:
  - retarget mid-move stays within jerk limits (validation report);
  - direction reversal while jogging;
  - a degree-1 path move keeps stop-first replacement and has enough hold
    lead to stop within joint acceleration limits;
  - a dual-motor axis stays in proportion under retargeting.

Commits: at least three — switch execution; switch planning; delete the
old planners.

## P10 — Trajectories and plans over robotd

Problem:
- robotd defines `TrajectoryRequest` but never handles it.
- `RemoteRobot` throws on `TrajectoryChunk` (`robotkit/haxe/robotkit/world/RemoteRobot.hx:107`).
- The network `RobotCapabilities` message has no queue field.

As a result, buffered motion works only in-process.

Do:
- Add a capability field for the trajectory queue and plan support.
- Implement plan submission (P7 shape) and the HOLD, RESUME and ABORT
  commands through `RobotClient`, `RemoteRobot` and `RobotServer`. Only the
  control-lease owner may submit.
- Forward runtime error codes to the client as faults with the code.
- Carry the new snapshot fields (session state, active plan id,
  `committed_until`).

Tests:
- `world-tcp.sh` gains a scenario in which a remote client submits a
  Ruckig plan, holds, resumes and completes.
- An observer session's submission is refused.
- Lease expiry mid-plan still triggers the emergency stop.

## P11 — Transmissions in the RobotKit model (D2, model and compiler only)

Do:
- `robotkit.model`:
  - replace `Actuator {name, maxEffort, maxRate}` with an actuator that has
    `id`, `maxEffort`, `maxRate` (actuator units) and a `Transmission`:
    `SimpleTransmission {jointId, ratio, offset}`, where
    `joint = offset + actuator / ratio` (documented, SI units);
  - allow several actuators on one joint (dual drive, each with its own
    transmission);
  - bump the `RobotModel` schema to v4, and have `RobotModelCodec` read v3
    by converting its one-to-one actuators (ratio 1, offset 0).
- `MachineKitRobotCompiler.compileLinearAxis`: emit the lead-screw
  transmission (ratio in rad/m from `LeadScrewNut.travelPerRevolution()`)
  instead of only keeping the motor and screw identity.
- `MotionSystemBlueprint.fromRobotModel`: derive `jointScales` and
  `jointOffsets` from the transmissions when present. Keep explicit values
  as an override for now, with a deprecation note in the doc comment.
- **Design only** for the runtime and device side. Write a section in
  `robotkit/ARCHITECTURE.md` specifying:
  - joint → actuator conversion at the endpoint boundary;
  - limits enforced in both spaces;
  - device layout mapping channels to actuators;
  - the fingerprint change;
  - dual-drive skew monitoring as a future fault.

  Do not change the runtime, the device layout or the fingerprint in this
  item.

Tests:
- v3 JSON still loads;
- a v4 round trip;
- the compiled LinearAxis has the lead-screw ratio;
- the derived axis scales equal the old explicit ones;
- a two-actuator joint is accepted.

---

## Out of scope for this plan (next plans)

In rough order, for context only:

1. RKD6 scheduled device protocol:
   - session and queue revision, commit and replace, host–device clock
     mapping;
   - polynomial segments from D4;
   - device-side underflow and communication-loss controlled stop;
   - shared C++/Rust segment test vectors.
2. Real actuator on `nucleo-g474re` (a stepper compiler beneath RKD6).
3. Runtime and device transmission support from P11's design.
4. Unify arm toolpaths (`robotkit.process`) with MotionKit paths.
   `FinishSurface` should submit timed plans, not per-tick `JointTargets`.
   `processOn` becomes events keyed to path progress.
5. `PathTrajectory` q(s) + s(t), TOPP-RA path timing, and replacing
   `LineLookaheadPlanner`.
6. IK solver interface plus OPW; CncKit; ProcessKit; the rest of the
   architecture's stages 6–12.

## Progress log

### P1 — Split the pure core and RobotKit adapter

Moved the three RobotKit/MachineKit integration classes to the new
`motionkit-robot` project, updated imports and documentation, and added a core
import boundary check. Commit: the commit containing this entry. The pinned
NativeKit and MuJoCo commits had to be fetched from the shared checkout because
their remotes did not provide them. MotionKit and RobotKit Haxe tests and the
RobotKit native tests passed; TCP integration passed in default, session and
lease-timeout modes.

### P2 — Reject unsupported trajectory queues

The runtime now rejects trajectory chunks when the endpoint does not advertise
queue support, before enqueue and at owner application, without latching a
fault. Added a native regression with an endpoint lacking queue support; it
failed before the runtime change and now passes. Commit: the commit containing
this entry. MotionKit and RobotKit Haxe tests, all nine RobotKit native tests,
and TCP integration in default, session and lease-timeout modes passed.

### P3 — Plan correction before implementation

Stopped before writing P3 code. `JointTrajectory.sample` interpolates stored
positions, velocities and accelerations independently, so a degree-1 position
polynomial cannot match all three values as P3's original test required. The
smallest correction above limits the equivalence claim to position and states
the native analytic derivative semantics. No P3 implementation or tests have
been started. Commit: the commit containing this entry.

### P3 resumed — Derivative-source rule for P9

The user confirmed the P3 degree-1 correction and directed work to proceed.
P9 now states which trajectories provide exact derivatives for retargeting,
and preserves stop-first behaviour for `moveLinear` and path moves until
native path timing exists.

### P3 — Add native polynomial trajectories

Added the standalone `motionkit_core` C++17 library, C ABI with versioned
structs and owned handles, a shared polynomial evaluator, segment continuity
reports, degree-1 sample conversion, generated Haxeon bindings and the Haxe
`Trajectory` wrapper. Robotd builds and loads the library. The wrapper calls
`mk_trajectory_from_samples` directly; its 1 kHz position round trip passed
at 1e-12 while native velocity, acceleration and jerk use analytic degree-1
semantics. Commit: the commit containing this entry. The native MotionKit test,
MotionKit Haxe suite, RobotKit Haxe and native suites, FFI audit, and TCP
integration in default, session and lease-timeout modes passed. The Haxeon FFI
exposes arrays of small coefficient structs, so the C ABI represents each
joint's six coefficients as one struct inside the segment.

### P4 — Validate limits and create execution plans

Added analytic position/velocity/acceleration/jerk extrema checks through
degree five, optional continuity claims, revisioned validation reports, and
native plans that deep-copy a trajectory and reject failed checks while still
returning the report. The Haxe wrapper requires callers to provide start-state
derivatives explicitly, avoiding the degree-1 derivative trap noted for P9.
Added calibration revision to the RobotKit blueprint, compiler, and snapshot,
with old C ABI struct prefixes accepted and defaulted to zero. Position limits
use an explicit claim flag because zero is a valid bound; unlike derivative
limits, their numeric values cannot double as an unclaimed sentinel. Commit:
the commit containing this entry. MotionKit Haxe and native tests, RobotKit
Haxe and native tests, both FFI audits, and TCP integration in default,
session, and lease-timeout modes passed.

### P5 paused — Nanosecond phase-boundary conflict

The first Ruckig conversion test exposed a conflict between exact limit
validation and the required 1e-9 acceleration match. In a nonzero-initial-
velocity case, a saturated acceleration reversal occurs at
1.2402138573773835 s. The native trajectory can place its segment boundary
only at an integer nanosecond. Placing it at 1.240213857 s and preserving
Ruckig's cubic on either side gives an analytic acceleration magnitude of
2.0000000018869173 against a limit of 2.0. Moving or clamping the boundary
state enough to pass exact validation creates more than 1e-9 acceleration
error just after the boundary. This is a representation constraint, not a
root-finding error. P5 is paused with partial work uncommitted. Proposed
correction, pending approval: keep strict validation and generate with a small
documented inward guard band on Ruckig's velocity/acceleration limits, then
compare conversion against that guarded Ruckig reference with a stated
nanosecond-quantization tolerance. Do not silently weaken `mk_validate` for
arbitrary trajectories.

The user approved a different correction: preserve Ruckig's output, round
phase boundaries to nanoseconds, and make the validator's comparison
tolerance explicit and reportable. The P4 and P5 sections above now state
that contract. P5 implementation resumed; the guard-band proposal was not
used.

### P5 — Add the Ruckig state-to-state generator

Pinned Ruckig Community `v0.19.4` as a submodule, linked only its offline
calculator sources, and recorded the MIT license and source audit in
`motionkit/native/THIRD_PARTY.md`. Added a native API for time-synchronized
position moves and velocity-control stops. It converts the union of each
joint's brake and seven phase boundaries to degree-3 segments, dropping
sub-nanosecond phases deterministically. A zero-duration unchanged state is
represented by one 1 ns constant segment. Ruckig result codes are returned
alongside MotionKit errors. The user-approved correction keeps the generated
curve unchanged and reports an explicit derivative-based comparison tolerance
and signed margin for every validation check. Native tests cover 1 kHz
comparison to Ruckig, rounded boundaries, multi-axis timing, moving starts,
reversals, retargeting, stopping, over-limit braking, travel overshoot, and
error mapping. Commit: the commit containing this entry. MotionKit and
RobotKit Haxe/native suites, FFI audit, and TCP integration in default,
session and lease-timeout modes passed.

### P4/P5 follow-up — Executor resolution, capped tolerance, task-space slot

Validation now takes an executor time resolution through `mk_limits`, defaulting
to 1 ns for the host runtime and older limits struct prefixes, and records the
effective resolution in `mk_validation_report`. The quantization allowance is
the next derivative's maximum times half that resolution. A 1e-9 relative
comparison floor handles floating-point error, while a 1e-6 relative cap
prevents a trajectory's extreme next derivative from widening its own limit
indefinitely; the cap matches the runtime's existing chord-speed slack.
`MK_CHECK_TASK_SPACE` is reserved and always unchecked until Cartesian/tool-path
validation is implemented. Bumped the MotionKit ABI, regenerated its Haxe
binding, and exposed the resolution and slot in the Haxe wrapper. A one-tick
extreme-jerk test fails under the old uncapped rule and passes with the cap.
Commit: the commit containing this entry. MotionKit native and Haxe, RobotKit
native and Haxe, both MotionKit and RobotKit FFI audits, and TCP integration in
default, session, and lease-timeout modes passed.

### P6 paused — Legacy sample STOP semantics conflict

Stopped before P6 code changes. `trajectory_stop_counts_trajectory_braking` in
`robotkit/runtime/tests/runtime.cpp` requires the current STOP controller to
budget the deceleration inferred across adjacent legacy sample chords. A
degree-1 position segment has zero analytic acceleration inside each segment
and a velocity step at each knot. Replacing that finite-difference estimate
with the segment's analytic acceleration would add STOP deceleration to a path
already braking at its joint limit, violating the unchanged runtime test and
the P6 degree-1 equivalence promise. Existing queue tests also publish sample
counts (`trajectory_queue_depth == 3` for three points), while two valid
positive-duration degree-1 segments represent those three points. The queue
can store segments, but legacy depth and capacity accounting need an explicit
compatibility rule. Proposed smallest correction, pending approval: preserve
legacy point-chunk STOP braking estimates and published queue-depth semantics;
use analytic velocity/acceleration for native segment chunks, and bound the
underlying queue by actual segments with conservative reservations for legacy
submissions. No P6 tests or implementation have been started.
