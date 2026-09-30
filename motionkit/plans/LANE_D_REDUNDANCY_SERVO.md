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
- Collision checking (a future plan), for D4.
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
    - **D5c:** the virtual device rejects JOINT_TARGETS, so servoing there
      needs short plan horizons.
