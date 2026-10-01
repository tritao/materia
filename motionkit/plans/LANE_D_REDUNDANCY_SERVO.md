# Lane D — Redundancy and live servoing (stub, not yet scheduled)

**Status:** recorded for later. Start it only after Lane C's C4 (OPW behind
`KinematicsSolver`) is on `main`, and once collision checking exists, for
D4. Before it starts, expand each item below into the full handoff format
used by Lanes A–C: problem, code locations, what to change, tests.

**Moved (2026-09-30):** D1–D3 (the QP differential IK core and 7-axis
redundancy) are now item K3 of `kinematicskit/plans/KINEMATICS.md`, and the
native core lives in `kinematicskit/native` rather than `motionkit/native`,
so RobotKit (humanoid H7) and CadKit can use it without depending on
MotionKit. D4–D6 (collision limits, live servoing, external axes) stay in
this lane and build on that core.

**Goal:**
- constrained differential IK as a native QP;
- real 7-axis redundancy handling;
- live Cartesian servoing (jogging, teleoperation, bounded corrections)
  through the runtime execution session.

This covers the original architecture's live-servo pipeline (§7C) and
redundancy model (§9.3), which the current lanes leave out.

## Reference: mink

[mink](https://github.com/kevinzakka/mink) (Python, Apache-2.0, built on
MuJoCo) is the design reference. It is **not** a runtime dependency.

- **What it models:** each step solves one QP that best meets weighted
  **tasks** within hard **limits**.
  - Tasks: tool-frame pose, posture, relative frame, centre of mass, damping.
  - Limits: joint positions, joint velocities, collision avoidance between
    geometry pairs.
- **Why we port it rather than use it:**
  - It's Python, and anything near the control loop has to be native and
    bounded-time (decision D1).
  - Its kinematics model is MuJoCo, which is our *simulator*. Planning must
    work on hardware without the simulation model.
- **How we use it:**
  - as the design reference for the formulation, the same approach as the
    "Descartes-shaped" decision in Lane C;
  - as an optional **test oracle**: it solves the same problems on the same
    MuJoCo model, and our native solver must agree within tolerance. These
    tests skip with a clear message when Python or mink isn't installed;
  - as a **prototyping tool** for task weights, posture costs and collision
    margins before committing them to C++.

## Lane decisions (provisional)

- **LD-D1 — Native, mink-shaped QP differential IK** in `motionkit/native`:
  - task and limit interfaces modelled on mink;
  - Jacobians from our own kinematics (RobotKit's chain now, Pinocchio when
    it arrives), never from the simulator;
  - (Superseded: the first backend is ProxQP's dense solver, shared with
    humanoid H7; see `kinematicskit/plans/KINEMATICS.md` KK-D13.)
  - OSQP as the first QP backend, behind a solver interface so it can be
    swapped.
- **LD-D2 — It implements contract C4's `solveDifferential`** and replaces
  the §P0 damped least-squares baseline as the default. The baseline stays as
  a fallback for when the QP fails, with a diagnostic.
- **LD-D3 — Live servoing is a cyclic-reference mode of the execution
  session**, not a stream of long queued trajectories being replaced.
  - It has explicit command deadlines, stale-command rejection (the unused
    `JointTargets.expiryNs` finally gets enforced) and bounded update rates.
  - It publishes a short horizon to the runtime.
  - When input stops, the session brakes to a stop within limits.

## Items (outline)

- **D1 — QP differential IK core.**
  - Tasks: frame pose, posture, damping.
  - Limits: configuration, velocity.
  - OSQP vendored as a pinned submodule, recorded in `THIRD_PARTY.md`.
  - C ABI and Haxe wrapper.
  - Mink oracle tests on the M9 6R arm and on a 7-axis fixture.
- **D2 — MJCF export for oracle tests.** Export a RobotKit model to the MJCF
  that SimKit's MuJoCo backend already builds internally, so mink and the
  simulator see identical kinematics.
- **D3 — 7-axis redundancy.**
  - Posture and elbow preferences expressed as tasks, never one hard-coded
    elbow parameter.
  - Joint-limit avoidance.
  - Redundancy optimised along a whole path: task weights over a path
    horizon, combined with Lane C's configuration selection where candidates
    exist.
- **D4 — Collision-avoidance limits** (needs collision checking): distance
  rows between declared geometry pairs, with margins recorded in the
  validation report.
- **D5 — Live Cartesian servoing (LD-D3).**
  - Jog and teleop twists become short-horizon references in the session.
  - Deadlines and stale-command handling.
  - Controlled stop when input stops.
  - Tests with the virtual device (Lane A) and in MuJoCo.
- **D6 — Coordinated external axes:** arm + rail + positioner as extra task
  and limit rows against the work frame, planned as one group, with a
  synchronized-execution contract.

## Dependencies

- Lane C: C4 (`KinematicsSolver` + OPW); C5 (configuration selection), for
  D3.
- Collision checking (`kinematicskit/plans/COLLISION.md`), for D4: it is
  that plan's C4.
- Lane A: the virtual device, for realistic servo-latency tests in D5.
- Pinocchio (optional, later): better Jacobians and derivatives. Not
  required to start.

## Progress log

- 2026-09-30 — **D5a done: `ServoSession` (motionkit-robot).** A tool twist
  goes through `ManipulatorServo` (one QP step) and out as runtime joint
  velocity targets on the manipulator's joints (`Manipulator.jointIndices`).
  - Each command carries a sequence and a deadline in robot time.
    Out-of-order commands are rejected as `Stale`, non-finite ones as
    `NotFinite`, and already-expired ones as `Expired`. After the deadline
    the session brakes to rest with twist 0, then holds exact zero targets.
    Those targets are sticky in the runtime, so the zero must be exact or
    the arm creeps.
  - The servo takes the previous velocity and the acceleration limits. It
    keeps each joint's velocity change within a·dt, and caps the approach to
    a stop at a·(√(dt² + 2d/a) − dt). That is the discrete braking bound:
    one tick of travel plus a full brake fits in the distance left. The
    continuous √(2ad) overshoots a·dt per tick. On a conflict, staying
    inside the limits wins over acceleration.
  - dt is capped at the control period. The runtime applies a velocity
    target at once, so missed updates ramp back up instead of jumping.
  - Test (SimulatedRobot, 10 ms): follows a 5 cm/s twist, keeps per-tick
    jumps ≤ a·dt, brakes within peak/a after a missed deadline, holds still,
    and turns the base into its stop with no overshoot.
- 2026-09-30 — **D5b done: runtime-enforced deadlines.**
  - `rk_robot_command.expires_at_ns` is appended to the ABI. It is on the
    robot's source clock, and `RobotCommand.JointTargets.expiryNs` finally
    carries it.
  - Once the latest sample reaches the deadline, each of the batch's
    velocity targets brakes to zero at the joint's `max_acceleration` (at
    once without one), then holds zero. The snapshot reports
    `RK_FAULT_COMMAND_EXPIRED` (non-latched) until the next target batch.
  - The deadline is compared with the source clock, not mapped onto the
    owner clock: the simulation's owner clock is a tick counter, and the
    host reads its deadline from the source clock anyway. A source clock
    that stops advancing is the sample-staleness watchdog's job.
  - `ServoSession` sends the command deadline with live targets, a
    two-period keepalive while braking, and none on the final zeros.
  - Tests: a native runtime test covers timing, a·dt per period, the exact
    zero hold, the diagnostic clearing, lapsing at once, and renewal. In the
    MotionKit test the host stops calling `update` mid-stream; the simulated
    arm keeps moving until the deadline (100 ms), then stops within v/a,
    changing speed by at most a·dt per tick.
  - Still open:
    - The runtime's brake is plain per joint (the tool leaves its line).
      Braking along the path would need the servo in the runtime.
    - Remote robots (robotd) still reject deadlines until host and robot
      clocks are mapped.
- 2026-09-30 — **D5c done: servoing through streamed plan chunks.** With
  `ServoPlanOptions`, `ServoSession` drives robots that execute plans (the
  virtual device, and the runtime's own plan path).
  - Each update appends one-period chunks (`ServoPlan`) to keep about 40 ms
    queued. Each chunk is quadratic and ramps every arm joint to the
    servo's next velocity from the state at the end of the queue.
    Consecutive chunks keep position and velocity continuous but not
    acceleration, so they are submitted jerk-unchecked.
  - Braking ends the stream with a chunk that ends at rest. A stalled host
    lets the queue run dry within the lead. The runtime then ramps to a
    stop and reports `RK_FAULT_TRAJECTORY_UNDERFLOW`; the device brakes
    each actuator at its limit and latches a fault.
  - Why append rather than replace: on the device a replacement is only
    accepted at a segment boundary inside its committed window, and it
    commits the whole sent queue at once. So a plan's braking tail could
    never be replaced. Appending is the device's native streaming model,
    and underflow is its native watchdog.
  - `ManipulatorServo.step(..., ramped)` counts the ramp's travel,
    (before + v)·dt/2, in the position and braking bounds, and keeps the
    turning point of a ramp that reverses within the tick inside the range.
    Without that, a chunk reversing at a stop overshot it mid-chunk.
  - A device reports no motion until a new queue's start delay passes, so a
    stream only counts as drained after it has been seen running. A
    stream's first chunk starts from the measured positions, allowed 1e-3
    off the held setpoint (a step or two).
  - Test: the D5a scenario on both backends. Streaming holds 5 cm/s,
    braking takes about v/a, the arm reaches the base-joint stop without
    passing it, and a host stall stops the arm within lead + v/a. Measured
    accelerations stay within the limit: exactly on the runtime, and within
    the device's step quantisation (1e-4 rad) on the virtual device.
  - Still open: a device underflow latches a fault, so after a host stall
    on a device the operator must reset safety before servoing again.
- 2026-09-30 — **Status of D1–D3.** D1 (the QP differential IK core) and
  D2 (MJCF export for the mink oracle) were done inside kinematicskit (K3,
  see `kinematicskit/plans/KINEMATICS.md`). They use ProxQP rather than
  OSQP (KK-D13).
- 2026-09-30 — **D3 done: 7-axis redundancy along a path.**
  - RobotKit's `Manipulator` has an `ArmSwivel`. By default a 7-DOF arm
    uses the pivots of its 2nd, 4th and 6th joints; the definition can be
    overridden. It also has `swivelAngle(q)` and `solveIkAtSwivel`
    (exact or preferred), both on kinematicskit's `SwivelTask`.
  - `ManipulatorKinematics` for a redundant arm:
    - `solvePose` keeps the seed's swivel as a preference, so the elbow no
      longer drifts with each solve.
    - `sampleCandidates` sweeps the swivel on each IK branch.
    - `continueCandidates` grows a path's candidates sample by sample: each
      one continues at its own swivel and at ±0.05 rad. Where a limit
      blocks, the swivel becomes a preference and the limit bends it. One
      candidate is kept per swivel bin.
    - `refinePath` smooths the chosen swivel (a Gaussian over about 1/8 of
      the samples) and re-solves each sample exactly at that swivel.
  - `ProgramCompiler` enables this whole-path selection for redundant arms,
    as it already did for OPW arms.
  - First cut: independent swivel sweeps per sample did not line up from
    one sample to the next, and the search found no connected route. Growing
    the candidates along the path fixes that by construction.
  - Test (7-axis RobotModel fixture):
    - The swivel is controllable, and point IK holds it within 1e-4 rad
      over 20 cm.
    - A compiled 35 cm line meets the Cartesian tolerance with smooth
      joints and swivel.
    - The whole-path choice moves the joints 4.8 rad against 7.8 rad point
      by point.
  - Open:
    - Joint-limit margins and a preferred swivel as costs in the search. The
      native preferred-posture cost is per sample, so on a long path it
      would swamp the motion cost; this needs a normalised state cost.
    - `PoseTarget` resolution (`MoveJ` to a pose) still takes the candidate
      nearest the start.
- 2026-10-01 — **D6 done: coordinated external axes.** The arm, a rail under
  it and a positioner holding the workpiece are one robot model, so one
  plan over all their joints runs on one runtime clock. That shared clock
  is the synchronized-execution contract.
  - kinematicskit:
    - `FrameTask.relativeTo(body, offset)` targets a frame relative to
      another moving body. Its rows are J_frame − J_reference, the
      reference's taken at the target point (exact for position).
    - `DofDampingTask` adds per-DOF step damping with zero target: it shapes
      steps without biasing the answer.
  - RobotKit `CoordinatedGroup`:
    - The joints root→flange (rail, arm) plus root→work frame (positioner).
      Tool poses are in the work frame.
    - External axes (the work branch and any named rail joints) are damped.
    - An optional preferred arm posture: a first pass draws the arm towards
      it, so the external axes bring the work round. A second pass, without
      the pull, meets the tool target exactly.
    - `relativeJacobian` gives the work-frame Jacobian.
  - MotionKit `CoordinatedKinematics` (a `KinematicsSolver`) makes
    `ProgramCompiler` and `ManipulatorMotion` work unchanged on a cell.
  - Test (workcell: the 6-axis arm on a 2 m rail plus a turntable):
    - A 0.3 m circle round the workpiece, the tool following the tangent.
      The far side is beyond the arm's reach.
    - The turntable turns 6.24 rad, the rail moves 2 cm, and no arm joint
      moves more than 0.025 rad.
    - The compiled plan passes the task-space check. Executed through
      `ManipulatorMotion` (the long plan streamed in chunks), the tool stays
      within 0.1 mm of the path in the work frame.
  - Bugs found on the way, all fixed with tests:
    - Runtime: an accepted plan did not clear the joint targets held before
      it, so when the plan ended the robot jumped back to them. Trajectory
      chunks already cleared them; plans now do too.
    - MotionKit timing: stretching or softening a time law re-derives every
      stage from rounded durations. Over 24k stages the rest boundary
      drifted (end speed −6e-7), and the law was rejected as an invalid
      argument. The exact rest boundary is now re-applied after each
      stretch or soften.
    - `ProgramCompiler`'s task-space check timed inspected distances by
      interpolating between sample times. Braking to rest over the last
      sample interval that was off by about a quarter of the interval
      (0.6 mm of false error). It now uses the time law's exact times.
  - Open:
    - Robots in separate runtimes (an external positioner on its own
      controller) need a cross-runtime synchronization contract: a shared
      start time and clock mapping.
    - Candidate selection over external-axis redundancy (D3-style lattice)
      instead of continuation with a posture preference.
- 2026-10-01 — **D5c workarounds replaced, and three device-path bugs fixed.**
  - The snapshot publishes the runtime's setpoint (`setpoint_position`, the
    anchor of the next plan). Streams start there exactly, so the 1e-3
    start tolerance is gone.
  - The RKD6 endpoint counts segments still waiting to be sent, or sent but
    not yet started, as active. "Not active" now means drained, so the
    "seen running" heuristic is gone.
  - The runtime's copy of a device-executed queue followed the owner clock.
    It ran ahead of the device by the start delay, retired knots the device
    had not executed, and took appends for new plans. It now follows the
    device's reported path time while the device runs the queue.
  - Between samples, the runtime overwrote the device-owned progress fields
    with its own copy's. A missed sample then published the copy's view.
    The runtime no longer writes them for device-executed queues.
  - STATE6 carries positions as 32-bit floats, so a joint resting exactly
    on a limit read 5e-8 past it and latched a fault. Endpoints now declare
    their position precision, which the observed-limit check allows for.
  - The servo test measures acceleration on the device's own sample
    timestamps, skipping repeated samples. A repeated sample had produced a
    false spike.
