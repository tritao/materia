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

(not started)
