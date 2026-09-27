# Motion architecture plan: gap map against the current code

This compares the proposed "Materia Motion / Machine Control Architecture —
Updated Plan" (not checked in; section numbers below are the plan's) with what
exists on `main` as of 2026-09-26. For each concept it records whether the code
already has it, where, and what the plan's version would change.

Status legend:

- **Exists** — the concept is implemented; at most a rename or a doc update.
- **Partial** — something is there, but it is narrower than the plan's contract.
- **Missing** — nothing is there yet.
- **Conflicts** — the current code contradicts the plan; one of them must change.

The plan was written as if MotionKit did not exist. It does: `motionkit/`
has about 20 commits and 2.5k lines of Haxe. Stages 1–3 of the plan (§35) are
therefore partly done in some form, and the question for each item is whether
to rename, formalize, or replace.

## Summary

| Area | Status | One-line verdict |
| --- | --- | --- |
| Dependency direction (§3) | Conflicts, cheap to fix | Only 3 MotionKit files import RobotKit/MachineKit |
| Robot model (§4) | Partial | Links, joints, limits, inertia; no transmissions |
| Calibration (§4) | Missing | No calibration object or revision |
| Deployment (§4) | Partial | Channel layout + fingerprint; no homing, brakes, modes |
| Task / joint / actuator coordinates (§5) | Partial | Logical axes map to joints with scale/offset in MotionKit; RobotKit has no actuator coordinates |
| `MotionProgram` (§6.1) | Missing | No MoveJ/MoveL/program type |
| `GeometricPath` (§6.2) | Partial | Lines and planar arcs, position only, no frames/tolerances/orientation |
| `JointTrajectory` (§6.3) | Exists (sampled only) | Fixed-period samples, linear interpolation |
| `PathTrajectory` q(s), s(t) (§6.3) | Partial | `TimeScaledTrajectory` is a time law over a source trajectory, not over a path |
| `ExecutionPlan` (§6.4) | Missing | Nothing carries revisions, validation, required capabilities |
| Positioning pipeline (§7A) | Partial | Trapezoidal, rest-to-rest; jerk not enforced; no Ruckig |
| Path-preserving pipeline (§7B) | Partial | Gantry XYZ only; arm toolpaths use a separate stop-at-every-point timer |
| Live servo (§7C) | Partial | Jog splicing exists; no differential IK, no deadlines |
| Lookahead / blending (§8) | Partial | Junction-velocity lookahead with exact-stop/blend |
| Kinematics (§9) | Partial | One DLS IK, no solver interface, no multi-solution selection |
| Collision checking (§9) | Missing on the planning side | Only MuJoCo contacts in simulation |
| Dynamics (§10) | Missing (sim only) | MuJoCo computed-torque; inertia stored but unused by planning |
| CncKit / ProcessKit (§11–13) | Missing / Partial | No CNC code; `robotkit.process` + `processOn` flag |
| Execution ownership / session (§14–15) | Partial | robotd control lease + runtime queue; no `ExecutionSession` object |
| Planning authority (§16) | Missing | |
| Backend classes and capabilities (§17–18) | Partial | Five capability booleans |
| Native execution / RKD (§19–22) | Missing | RKD5 is immediate targets only; board firmware is a stub |
| FPGA (§23), external controllers (§24) | Missing | |
| Simulation shares execution (§27) | Exists | Simulation runs the same `RobotRuntime` as hardware |
| Validation report (§28) | Missing | Runtime does limit checks, but produces no report |
| Diagnostics / recording (§29) | Partial | MCAP recorder and replay; no planning or validation records |
| Safety (§30) | Partial | Latch, e-stop, lease-expiry e-stop, device watchdog |
| Third-party libraries (§31) | Missing | Only MuJoCo is vendored |

## 1. Package boundaries and dependencies (§2, §3)

**Split in P1.** `motionkit/haxeon.json` has no MachineKit or RobotKit
dependency. The `motionkit-robot` adapter owns the three integration classes
under `motionkit.robot`.

The three adapter files are:

- `motionkit/robot/haxe/motionkit/robot/MotionSystem.hx` — `robotkit.world.{Robot,
  RobotCommand, RobotSnapshot, StopMode, TrajectoryChunk, TrajectoryPoint,
  JointTarget}`
- `motionkit/robot/haxe/motionkit/robot/MotionSystemBlueprint.hx` —
  `robotkit.model.RobotModel`, `robotkit.runtime.{RobotRuntimeBlueprint,
  RobotRuntimeCompiler}`
- `motionkit/robot/haxe/motionkit/robot/MachineKitRobotCompiler.hx` —
  `machinekit.assembly.LinearAxis`, `robotkit.model.*`

`path/`, `planner/`, `trajectory/` and `axis/MotionAxisBlueprint.hx` are
already free of RobotKit. The adapter uses the existing pure MotionKit types.

Other kits already follow the plan's layering:

- `machinekit` depends only on `cadkit` + `projectkit`.
- `automationkit` depends on `robotkit` and computes no trajectories. It
  produces navigation paths and speed limits and hands them to RobotKit
  skills.

## 2. Model, calibration, deployment (§4, §5)

**Robot model — Partial.**

- `robotkit/haxe/robotkit/model/RobotModel.hx` (schema version 3) has links
  with mass, centre of mass and inertia tensor, plus joints, frames and a
  collision approximation.
- `JointLimits.hx` holds position, velocity, effort and maxAcceleration.
- `Actuator.hx` is only `{name, maxEffort, maxRate}`, attached one-to-one to a
  joint. There are no gear ratios, no actuator-versus-joint coordinates, and
  no coupled, mimic or dual-drive joints.

MotionKit covers part of §5 on its own side. `MotionAxisBlueprint` maps one
logical axis to several joints with a scale and an offset per joint, which is
how dual-motor and geared axes stay in proportion today. That is a task→joint
coupling living in MotionKit. The plan wants transmissions in the RobotKit
model instead, so this needs a decision: move the coupling into RobotKit, or
declare that MotionKit axes are the transmission layer.

**Calibration — Missing.** There is no calibration object, no joint zero
offsets and no calibration revision.

- `tool/Tool.hx`'s `flangeTTcp` is authored tool data, not calibration.
- `perception` `SurfaceRegistration` registers work surfaces, not the robot.
- The RKD5 fingerprint hashes "safety-relevant calibration" bytes of the
  deployment layout. That is the only calibration identity there is.

**Deployment — Partial.**

- `robotkit/deployment/bench-nucleo-g474re/` holds `deployment.json` (model
  file, device, baud, schema lock, fingerprint, target error budget) and
  `layout.json`, which maps channel to joint via `device/DeviceLayout.hx`.
- Control mode is chosen per command (`world/JointTargetMode.hx`), not
  configured per channel.
- Homing inputs, brakes, controller assignment and watchdog configuration are
  missing.
- `MotionSystem.home()` is software homing: it moves to the authored home
  coordinate and uses no switch.

## 3. Motion contracts (§6)

**`MotionProgram` — Missing.**

- The only command surface is `robotkit/haxe/robotkit/world/RobotCommand.hx`:
  `JointTargets(targets, expiryNs)` and `TrajectoryChunk`.
- `MotionSystem` offers `moveAxes`/`queueAxes` (roughly MoveJ for axes),
  `moveLinear`/`queueLinear` and `movePath`/`queuePath` (roughly MoveL for a
  direct XYZ machine), `jog` and `home`. These are methods, not a program
  representation.

**`GeometricPath` — Partial.** `motionkit/path/` has `LineSegment`,
`ArcSegment` (planar) and `GeometricPath`, parameterized by arc length with
tangent and curvature. Compared with the plan:

- Position only: no orientation, reference frame, tolerances or orientation
  policy.
- There is a second, independent path type: `robotkit/process/Toolpath.hx`
  (`work_T_tcp` poses, feed rate, `processOn`, normal and standoff). The plan's
  single `GeometricPath` should subsume both. Section 34 of the plan
  ("compatibility facades") applies to `robotkit.process`.

**`JointTrajectory` — Exists, sampled only.** `motionkit/trajectory/JointTrajectory.hx`:

- Immutable, monotonic samples of position, velocity and acceleration.
- Evaluated by linear interpolation of all three.
- Planners emit it on a fixed period (0.01 s by default).

There is no polynomial representation, so it is `JointTrajectory` in the
plan's sense but with only a sampled encoding.

**`PathTrajectory` — Partial.** `TimeScaledTrajectory` plus `TimeScaling`
re-time a source trajectory along itself, which is what makes path-preserving
hold and resume work. That is a time law over trajectory time, not over path
arc length s: there is no q(s) object. The idea is the same; the
parameterization is not.

**`ExecutionPlan` — Missing.**

- No object carries model or calibration revision, start-state assumptions,
  validation results or required capabilities.
- The runtime has `blueprint.revision` (`RobotRuntimeCompiler.compile(robot,
  revision=1)`), which is copied into every snapshot. That is the seed of the
  plan's model revision.

## 4. The three pipelines (§7, §8)

**Positioning (§7A) — Partial.**

- `motionkit/planner/TrapezoidalPlanner.hx` is a synchronized, rest-to-rest
  trapezoidal planner.
- `MotionLimits.maxJerk` exists in the API but no planner enforces it.
- Nonzero initial velocity or acceleration is handled indirectly: an
  immediate move while moving first stops along the current path, then plans
  from rest. This is where Ruckig adds real value — retargeting from a moving
  state without stopping.

**Path-preserving (§7B) — Partial, in two unrelated places.**

- MotionKit: `LineLookaheadPlanner` plans a connected line/arc polyline for a
  direct XYZ machine, where Cartesian coordinates *are* the joints. It does
  junction-velocity lookahead, and it budgets tangential and centripetal
  acceleration on arcs. There is no IK in this path.
- RobotKit arms: `process/CartesianTrajectory.build` times each waypoint pair
  with its own trapezoid (a stop at every waypoint, slerp orientation).
  `process/ToolpathExecutor` then runs IK per sample, seeded from the previous
  solution, and rejects joint jumps over `maxJointStep`. There is no q(s), no
  path timing through joint limits, and no blending.
- `skill/FinishSurface.hx:249` executes arm toolpaths as one
  `RobotCommand.JointTargets` per update tick. It does not use timed
  trajectory chunks, so sample times never reach the runtime.

TOPP-RA and conservative segment timing are missing.

**Live servo (§7C) — Partial.**

- `MotionSystem.jog` with `JogProfile` splices a new jog into the runtime
  queue a few periods ahead. If the splice is late, it falls back to stopping.
- There is no differential IK or twist command, and no command deadlines:
  robotd rejects a nonzero `expiryNs` with "absolute deadlines require clock
  synchronization" (`robotd/src/RobotServer.hx:396`).

**Lookahead and horizons (§8) — Partial.**

- The runtime already has a committed/replannable distinction in practice.
  `TrajectoryChunk` carries `spliceTag`/`spliceTime`, and the runtime
  truncates its queue at that point (`runtime/src/runtime.cpp`, splice
  handling).
- A splice that arrives after its point has been executed is dropped,
  which is the plan's invariant 14 ("committed motion is immutable").
- What's missing is an explicit `committedUntil` exposed to planners and
  horizon sizing based on stopping distance or latency. MotionKit uses fixed
  lead periods.
- Exact-stop and blend modes exist (`PathPlanningOptions`). ExactPath as a
  separate mode does not.

## 5. Kinematics, collision, dynamics (§9, §10)

- **FK / Jacobian — Exists:** `manipulation/KinematicChain.hx` (6×N geometric
  Jacobian).
- **IK — Partial:** `manipulation/InverseKinematics.solve`, a static damped
  least-squares solver with joint clamping.
  - There is no solver interface, so this is the plan's "hard-coded IK choice".
  - Missing: analytic IK (OPW), candidate sampling, configuration selection
    across a path, a velocity-level differential IK API, and null-space or
    redundancy handling.
- **Collision — Missing** on the planning side.
  - `tool/ToolCollisionShape.hx` is declared but never checked.
  - `manipulation/BaseObstacle.hx` is a 2D circle used for base placement only.
  - The only 3D collision is MuJoCo contacts in simulation.
- **Free-space planning:** missing for arms. For mobile bases there is
  `navigation/AStarPlanner.hx` over a 2D costmap.
- **Dynamics:** missing on the planning side. Inertia is stored in the model.
  The MuJoCo backend does computed torque (`tau = M·qacc + bias`) in simulation
  only.

## 6. CNC and process (§11–13)

- **CncKit — Missing.** There is no G-code or CNC code in the repo. The
  MotionKit README already says CNC semantics stay outside MotionKit, which
  matches the plan.
- **ProcessKit — Partial, inside RobotKit.**
  - Generators: `robotkit.process` (`Toolpath`, `ToolpathPoint`,
    `segmentByProcess` into approach/process/retract),
    `work/RasterToolpathGenerator.hx` and `work/DigCyclePlanner.hx`.
  - Skills: `skill/FinishSurface.hx`, `Paint`, `Sand`.
  - Devices: `SurfaceTool`, `Sprayer`, `Sander`, `Gripper`.
- **Process events synchronized to motion (§13) — Missing.**
  - The only process event is a `processOn` flag on each toolpath point.
  - `FinishSurface` switches the tool from software at submit time and passes
    the step index as its "timestamp".
  - This is exactly the pattern §13 says to replace with events keyed to path
    progress.

## 7. Execution ownership and sessions (§14–16)

**Command authority — Partial.**

- robotd has one exclusive owner: `ControlOwner {None, LocalBehavior,
  RemoteController(session)}` (`robotd/src/RobotServer.hx:41`).
- The lease times out after 3 s, is renewed by heartbeat, and triggers an
  emergency stop on expiry.
- Every other connection is an observer.
- This is exclusive ownership, not arbitration. Manual jog versus program on
  the same joints is not modelled.

**Execution state — Partial, split across three places.**

- *Runtime (C++):* a trajectory queue of up to 4096 points, with tags,
  splice, controlled STOP along the path, e-stop latch and fault latch.
  - The only commands are `NONE`, `JOINT_TARGETS`, `STOP`, `EMERGENCY_STOP`,
    `RESET_SAFETY` and `TRAJECTORY_CHUNK`. There is no hold, resume or abort
    command in the runtime.
  - `MotionSystem` builds hold and resume on top of STOP and re-timing on the
    host.
- *MotionSystem (Haxe):* the active trajectory, the queued trajectories, the
  re-timing state, pending splices and the hold flag.
- *robotd:* the control lease and session.
  - It has no trajectory path at all: `TrajectoryRequest` is defined but not
    handled, and `RemoteRobot` throws on `TrajectoryChunk`
    (`haxe/robotkit/world/RemoteRobot.hx:107`).
  - Buffered motion therefore works only in-process: simulation, or
    `SerialRobot` without robotd. It does not work over the network.

The plan's `ExecutionSession` would be one object that unifies these. It
should be built on this state rather than beside it, because the runtime's
queue, tags and splice already implement most of `submit`,
`replace(replannableRegion)` and `controlledStop`.

**Planning authority (§16) — Missing.** Today every plan is Materia-owned, so
there is nothing to record yet. The field becomes necessary with the first
external controller backend.

## 8. Backends and capabilities (§17, §18)

- `rk_robot_capabilities` / `robotkit.world.RobotCapabilities` has five
  booleans: position, velocity, effort, prediction and trajectory queue.
  - Position, velocity and effort are hardcoded to 1 in
    `runtime/src/runtime_c_api.cpp`.
  - The queue flag comes from the endpoint.
- The network `protocol/RobotCapabilities.hx` has no queue field, and robotd
  sends hardcoded values.
- The runtime does not reject a trajectory chunk when the endpoint lacks queue
  support. Only the Haxe adapter checks the flag. This is a small instance of
  the plan's "backends must reject unsupported guarantees" (§6.4, invariant 13).

Backend classes that exist today:

| Plan class | Current code |
| --- | --- |
| Trajectory-execution backend | `RobotRuntime` + `SimulationRobot` endpoint (queue supported) |
| Cyclic-control backend | `RobotRuntime` + `DeviceSerialEndpoint` (immediate targets only, `supports_trajectory_queue() == false`) |
| Program-execution backend | none |

## 9. Native execution and RKD (§19–23)

**RKD scheduled protocol — Missing.** RKD5 (`robotkit/runtime/DEVICE_PROTOCOL.md`,
`schema/device_wire.wire.idl`) already provides:

- sessions, a model fingerprint and sequence numbers;
- a device watchdog (500 ms on the bench board) and latched safety;
- monotonic device timestamps in STATE.

Commands are instantaneous `JointTarget{joint, mode, value:f32}`. There are no
segments, no queue, no commit/replace and no clock mapping. `ARCHITECTURE.md`
already says a host–device clock mapping must come before deadlines.

**Device firmware — stub.**
`device_protocol/boards/nucleo-g474re/src/main.rs` runs a `FakeDevice`: two
virtual wheel joints with no pins driving motors. Velocity targets are
integrated into a virtual position. The plan's "one physical MCU endpoint"
(Stage 4) is further away than "evolve RKD" suggests: no actuator has been
driven yet.

**Device compiler, stepper, FPGA — Missing.** There is no step generation
anywhere; "stepper" appears only as MachineKit CAD parts.

**Underflow (§22).** The host runtime handles the stop case: a stop that runs
out of queued path finishes on an acceleration-limited straight ramp. When no
stop is active and the queue simply ends, the runtime clears the queue and
holds the last point (`runtime/src/runtime.cpp`, queue-exhausted branch).
That is safe only because MotionKit plans every trajectory to end at rest and
refills ahead of time. The device side has nothing yet.

**Evaluation languages.** Today exactly one implementation executes
trajectories: C++ `RobotRuntime`, with `double` positions and linear
interpolation. The Rust device evaluates nothing. The shared-evaluator
concern only becomes real once RKD carries segments. At that point the C++
runtime and the Rust device must interpret the segment encoding identically,
so the encoding needs a precise spec and shared test vectors. That work
belongs in the RKD6 design, not after it.

## 10. Simulation, validation, diagnostics, safety (§27–30)

- **Simulation — Exists, and it already meets invariant 12.**
  `Simulation::add_robot` (`runtime/src/simulation.cpp`) builds an ordinary
  `RobotRuntime` over a `SimulationRobot` endpoint. Queueing, sampling, stop
  and splice use the same code as hardware, and one `Simulation` owns one
  clock for all robots. The backends are a deterministic test backend and
  MuJoCo.
- **Validation — Partial checks, no report.**
  - The runtime rejects chunks with a point outside joint limits or a chord
    velocity over `max_velocity` (including the jump from the queue's end),
    and latches a fault.
  - MotionKit validates path limits before planning.
  - There are no acceleration or jerk checks on submitted chunks, no Cartesian
    tolerance check, no collision check, and no `ValidationReport` object.
- **Recording — Partial.**
  - MCAP writer and reader: `runtime/src/recording.cpp`.
  - Haxe `RecordingRobot`/`ReplayRobot` record commands, snapshots, sensors,
    faults and world events.
  - Planning requests, plan revisions and validation reports are not recorded.
    §29 says to extend this recorder rather than add a second one.
- **Safety — Partial.** Present today:
  - e-stop and fault latch with explicit `resetSafety`;
  - lease-expiry e-stop;
  - device watchdog;
  - sensor stale-sequence rejection;
  - soft speed and acceleration caps for mobile bases.

  Not present: deadline enforcement on joint targets, and staleness checks in
  `ToolpathExecutor`/`FinishSurface`.

## 11. What this changes in the plan's sequence (§35, §36)

The plan's first PR ("MotionKit core contracts + local Ruckig adapter +
RobotKit ExecutionSession for one simulated prismatic axis") overlaps heavily
with existing code. The simulated prismatic axis, trajectory types, a
state-to-state planner and a session-like queue owner all exist. A sequence
that builds on the repo instead:

1. **Split MotionKit** (§3). Move `MotionSystem`, `MotionSystemBlueprint` and
   `MachineKitRobotCompiler` into an adapter package, so `motionkit` depends on
   nothing. There is no behaviour change and the existing tests keep passing.
2. **Decide the Haxe/native line** (§25). Ruckig, TOPP-RA, OPW, OSQP and
   Pinocchio are C++. The existing planners are Haxe on haxeon (reduced
   `Math` stdlib). Either move the numerical planners native behind a C ABI
   next to `robotkit/runtime`, or keep Haxe planners and wrap only the
   external libraries. Most later items depend on this answer.
3. **Formalize the contracts** around what exists:
   - `ExecutionPlan` wrapping `JointTrajectory` with `blueprint.revision`, a
     validation report and required capabilities.
   - The runtime rejecting plans or chunks its endpoint cannot execute.
   - A calibration revision next to the model revision.
4. **Unify the two path systems.**
   - Arm toolpaths should go through timed chunks instead of per-tick
     `JointTargets` (`FinishSurface.hx:249`).
   - `robotkit.process.Toolpath` becomes a facade over MotionKit paths with
     orientation.
   - `processOn` becomes an event keyed to path progress (§13).
5. **RKD6 scheduled segments.** Clock mapping, queue revision, commit and
   replace, device-side controlled stop on underflow and communication loss,
   and a segment-encoding spec with shared C++/Rust test vectors. Build it
   against the `FakeDevice` first, then drive a real actuator on the
   nucleo board. This remains the largest unproven piece.
6. **Carry trajectories over robotd** (`TrajectoryRequest`), so buffered
   motion works remotely and not just in-process.
7. **Ruckig** for retargeting from a moving state, replacing stop-then-replan
   in `MotionSystem.replaceMotion` and the jog fallback.

After that, the plan's stages 6–12 (CncKit, TOPP-RA, OPW/Descartes,
ProcessKit, redundancy, FPGA, dynamics) apply roughly as written, since they
are genuinely greenfield.

## 12. Decisions

The four open questions are settled in `motionkit/IMPLEMENTATION_PLAN.md`
(D1–D4):

- Numerical motion code is native C++ (`motionkit/native`); Haxe owns the API
  and orchestration.
- Transmissions live in the RobotKit model.
- `ExecutionSession` is a native runtime object.
- The canonical joint trajectory is piecewise polynomial; sampled
  trajectories are its degree-1 case.
