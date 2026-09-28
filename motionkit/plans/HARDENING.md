# MotionKit hardening plan

Keep this work on the `motionkit-phase0` branch in the single worktree at
`../materia-mk-phase0`. The MotionKit suite, native CTests, and RobotKit
integration suite must pass for a phase to be complete, and every commit must
leave them green.

## Phase 0: correctness fixes

The pinned Haxeon compiler needs `haxe.ds.ObjectMap.remove` for MotionKit's
trajectory bookkeeping. Add the HashLink binding and its regression case in
the Haxeon submodule in this worktree.

### 0.1 Normal abort reports idle too early

- Test first: start a long move, run a few ticks, call `abort()`, then
  immediately call `moveAxes(...)`. Assert that the new move is deferred (it
  returns null) and starts from the position where the stop settled. Also
  assert that `isMoving()` stays true until the runtime reports rest. This
  should fail on the current main.
- Fix `MotionSystem.abort`: when motion is in progress, send the abort,
  discard the queue, set `stoppingForReplacement = true`, and set
  `afterStop = []`. Do not call `clearBufferedMotion()`. `advanceStop()` moves
  to idle once the runtime reports rest.
- Keep the existing test at `MotionKitBootstrapTests.hx:2043`, which aborts
  while nothing is running.

### 0.2 ProgramCompiler start tolerances

- Replace `tolerances()` in `ProgramCompiler` with a required
  `StartTolerances {position, velocity, acceleration}` object, with one value
  per joint in solver order and no default.
- Have the production compiler callers derive values from their machine,
  manipulator, or blueprint settings. Tests pass explicit values.
- Reject arrays with the wrong length and values that are not finite or are
  negative.
- Test that a program compiled with a tight position tolerance is rejected
  by the runtime when its start state differs by more than that tolerance.

### 0.3 Stale path report

- Clear `lastPathValidationReport` and diagnostics at the start of
  `planPathFrom` and `planLinearPathFrom`.
- Test that after a successful plan, a path that throws leaves the report
  null.

### 0.4 Captured command inputs

- Make the `queue*` deferral closures in `MotionSystem` capture copies of
  targets or paths, rather than caller-owned arrays or objects.
- Test that a move queued during a stop uses the original targets after the
  caller changes its array.

### 0.5 Stale docs

- In `motionkit/README.md`, remove the position-target and non-queue fallback
  text. State that robots must support the trajectory queue and execution
  plans.
- Document `home()` as software homing to an authored coordinate, not
  hardware referencing.

## Deferred Phase 1: CI

CI work is deferred for now.

- Add `.github/workflows/motionkit.yml`, triggered on pushes and pull requests
  that touch `motionkit/`, `robotkit/`, or `haxeon/`.
- Job 1 builds native code in Debug and runs the CTests.
- Job 2 performs the same build with ASan and UBSan.
- Job 3 runs the MotionKit Haxe suite, RobotKit tests,
  `check-core-boundary.sh`, the FFI audit, and TCP integration tests.
- Resolve the `haxeon` pin first: it points to a commit that often exists only
  locally and has not been pushed. CI needs the commit pushed and a bootstrap
  step that needs no network access beyond fetching.
- Add TSan later, after checking the native runtime threading model against
  it.
- Decide whether this check is required on `main` before merging.

## Phase 2: MotionSystem refactor

The public API stays unchanged, so RobotKit and MachineKit callers remain
unaffected. Do this as five commits on one branch.

1. Extract `TrajectoryStream`. It takes over chunk submission, the two-second
   window, refill decisions, the tag-to-chunk map, elapsed-time sync from the
   runtime, completion detection, and smooth-replacement submission. Move
   both `MotionSystem` and `PlanExecutor` onto it; `PlanExecutor` keeps event
   handling and joint expansion. This step changes no behavior and edits no
   existing tests.
2. Extract stateless `AxisPlanner` and `PathPlanner` APIs:
   `plan(start, request) -> {trajectory, report, diagnostics}`. `MotionSystem`
   stores their results.
3. Introduce `MotionRequest`, an enum of copied inputs: `Axes`, `Linear`,
   `Path`, `Queued`, and `Jog`. Replace `afterStop` closures.
4. Introduce `MotionSession` and `SessionState` with states `Idle`, `Running`,
   `Holding`, `Held`, `Stopping(then: Discard | Replan)`, and `Faulted`.
   Remove `held`, `stoppingForReplacement`, `afterStop`, `activeJogAxis`, and
   scattered `activeTrajectory != null` checks. Only `update()` leaves
   `Holding` or `Stopping`, and only after the runtime reports rest. A nonzero
   snapshot `faultCode` or rejected submission moves the session to `Faulted`;
   leaving `Faulted` requires explicit `reset()`. Document the transition
   table on `MotionSession`.
5. Add virtual-robot tests driven by the transition table. Each state/event
   pair asserts the next state and the runtime command. Include abort followed
   immediately by a move, hold during `Stopping`, resume during `Holding`, a
   jog continued after replacement, and a fault while `Stopping`.

Outcome: `MotionSystem` shrinks to roughly 250 lines of facade.

## Phase 3: guarantees and tests

- Add `ValidationReport.guarantees()` with per-check status: joint position,
  velocity, and acceleration proven; jerk unchecked; task space sampled every
  1 ms. Include it in plan telemetry and `ManipulatorProgress` output.
- Add a README section explaining what “validated” means.
- Split `MotionKitBootstrapTests.hx` into `PlannerTests`, `StreamTests`,
  `SessionTests`, `ProgramTests`, `KinematicsTests`, and `ProcessTests`, with
  shared test support. Move code only; keep the assertion count unchanged.

## Out of scope

Lane D (redundancy and live servoing), collision avoidance, hardware homing,
and hardware-in-the-loop testing remain roadmap items. `ProgramCompiler`
restructuring beyond 0.2 is out of scope.

## Order and risk

Phase 0 goes first and is low risk. Phase 1 can run in parallel with it.

The riskiest part is Phase 2, step 1, because refill and hold timing are
subtle. Existing buffering and hold tests are the safety net, so that step
must not edit any test.
