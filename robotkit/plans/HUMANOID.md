# Humanoid simulation (handoff)

**Goal:** a humanoid robot standing, walking and recovering from pushes in
SimKit, driven through the normal RobotKit runtime, recorded to MCAP and
replayable. The first reference robot is the **Unitree G1**. The first
controller is a **learned locomotion policy**; a model-based whole-body
controller follows on the same foundation. Hardware is last and out of scope
until H8.

Read first:
1. `robotkit/ARCHITECTURE.md`: the model → blueprint → runtime boundary.
2. `robotkit/runtime/include/robotkit_simkit.h` and
   `robotkit/runtime/src/simulation.cpp`: how a blueprint becomes SimKit
   bodies and joints.
3. `simkit/sim_mujoco/README.md` and `simkit/sim_core/include/nativekit_sim.h`.
4. `sensorkit/README.md`: IMU measurement models.
5. `motionkit/plans/LANE_D_REDUNDANCY_SERVO.md`: the QP IK design that H7
   extends.

Work in `../materia-worktrees/humanoid` on branch `humanoid`. Failing test
first, one commit per item, merge to `main` after each green item. Append to
the Progress log at the end of this file. Stop and log whenever this plan
turns out to be wrong.

## Why the motion stack alone is not enough

MotionKit assumes a fixed-base, fully actuated machine executing pre-planned,
validated trajectories. A humanoid is floating-base and underactuated: it
stays upright only through contact forces that a feedback controller manages
every few milliseconds. Walking is a control loop, not a trajectory.

What carries over: Ruckig (pose transitions, output limiting), TOPP-RA for the
arms, MotionGuard-style command checks, the execution-session lifecycle, MCAP
recording, RKD6's handshake, fingerprint, clock sync and watchdog. What does
not: OPW IK, CncKit, and RKD6's segment queue as the leg command path.

HOLD must mean "keep balancing in place". Freezing a standing humanoid's
joints makes it fall.

## Lane decisions

- **HU-D1 — A floating base is a property of the model's root link, not a
  joint.** `RobotModel.floatingBase` (schema v6) marks the root link as a free
  6-DOF body. Runtime joints stay one-DOF, so joint indices, targets,
  snapshots and trajectories are unchanged. Importers map an MJCF
  `<freejoint>` on the root body, or a URDF world→base floating joint, to this
  flag. `JointType.Floating` stays rejected by the compiler.
- **HU-D2 — Import MJCF through MuJoCo's own parser.** A native tool loads the
  file with the vendored MuJoCo (`mjSpec`) and emits a `RobotModel` plus mesh
  assets. Writing our own MJCF parser would reimplement defaults classes,
  includes and compiler options badly. URDF import follows as a subset.
- **HU-D3 — The humanoid joint command is `q, qd, kp, kd, τ_ff`,** evaluated as
  PD inside every physics substep, with the physics rate independent of the
  control rate. This is what both RL policies and whole-body controllers emit.
- **HU-D4 — Controllers see sensors, not truth.** Observations come from
  SensorKit and encoders. Privileged truth is available only behind an
  explicit debug flag.
- **HU-D5 — Sim-to-sim conformance gates everything after H2.** If SimKit and
  plain MuJoCo disagree on the same model and inputs, policies trained in
  MuJoCo Playground won't transfer.
- **HU-D6 — Learned first, model-based second.** A policy gets the robot
  walking soonest. The whole-body controller (H7) reuses H0–H3 and Lane D.

---

## H0 — Floating base through the runtime

Problem: `RobotRuntimeCompiler` rejects floating joints, and `Simulation`
always creates a robot's root link as a kinematic body, so no robot can fall,
stand or walk.

Do:
- `RobotModel.floatingBase:Bool`, codec schema v6. Update checked-in v5
  fixtures.
- Blueprint: the native `rk_robot_runtime_blueprint.reserved0` becomes
  `floating_base` (same layout). Validation: 0 or 1; a floating root link needs
  positive mass.
- `Simulation`: a floating robot's root link is a dynamic body with no parent
  joint (MuJoCo free joint). Kinematic-base operations (`drive_robot_base`,
  differential and omni drive couplings) return `RK_ERROR_INVALID_STATE` for a
  floating robot. Teleport, place, and reset set the pose and zero the
  velocity.
- `rk_simulation_get_robot_base_velocity`: world-frame linear and angular
  velocity of the root link. Haxe `Simulation.robotBaseVelocity`.

Tests:
- codec round trip and compiler lowering of `floatingBase`;
- a floating two-link robot dropped onto a static floor settles at the
  expected height with near-zero velocity, on MuJoCo;
- kinematic-base operations are rejected for a floating robot;
- reset restores the initial pose with zero velocity.

## H1 — MJCF import (G1 into a RobotModel)

Do:
- `robotkit/tools/mjcf_import` (C++, links the vendored MuJoCo): bodies, hinge
  and slide joints, free joint → `floatingBase`, limits, inertials, geoms
  (mesh, box, capsule, sphere, cylinder), actuators with gear and force range,
  joint armature, damping and friction loss, IMU sites → `Sensor` frames.
- Mesh assets written next to the model for SceneKit and collision.
- Take the G1 model from MuJoCo Menagerie (check the model's own license) as a
  fixture outside the default test run, plus a small hand-written MJCF fixture
  in the default run.

Tests:
- the small fixture round-trips every supported element;
- G1 total mass, link masses, inertias and joint limits match the MJCF within
  tolerance; the model renders in the editor.

## H2 — Physics fidelity for legged contact

Do:
- Engine-neutral settings in `sim_core`, lowered by `sim_mujoco`:
  - joints: armature, damping, friction loss;
  - geoms: friction (sliding, torsional, rolling), soft-contact stiffness and
    damping (MuJoCo `solref`/`solimp`);
  - world: timestep, integrator, friction cone type.
- Carry them through `RobotModel` → blueprint → Simulation.
- The HU-D3 command mode: per-joint `q, qd, kp, kd, τ_ff` with torque
  saturation, evaluated every substep.
- Configurable physics timestep (1–2 ms) separate from the runtime owner
  period (default 10 ms today).
- The gravity-loaded actuator-limit regression from `TODO.md`.

Tests:
- **Conformance:** load the same MJCF in SimKit and in plain MuJoCo, apply the
  same torque sequence from the same initial state, and require joint and base
  trajectories to agree within tolerance for N seconds.
- The actuator-limit stall/move regression.

## H3 — Sensing

Do:
- Pelvis IMU through SensorKit `sensor_sim` (noise, bias, latency).
- Joint encoders with quantization.
- Per-foot contact force aggregated from SimKit contacts.
- An observation assembler that reads only sensors (HU-D4).

Tests:
- a standing robot's IMU reads gravity; foot normal forces sum to its weight
  (m·g);
- noise and latency settings change observations as specified.

## H4 — Policy runtime (first walk)

Do:
- Native ONNX Runtime binding (MIT).
- A policy spec file: observation layout and scaling, action scaling, default
  pose, kp/kd, control period, history length.
- Train the G1 joystick policy in MuJoCo Playground, or start from a
  pretrained unitree_rl_gym policy; export ONNX. Training scripts live in
  `robotkit/tools/humanoid/` and are not part of the build.
- The velocity command enters as a cyclic reference through the execution
  session (Lane D's LD-D3 shape).

Tests:
- G1 walks forward at 1 m/s for 30 s, turns, and survives a push test;
- the run records to MCAP and replays.

## H5 — Humanoid runtime semantics and safety

Do:
- States: passive/damping → stand-up (Ruckig to the default pose) → policy →
  sit-down.
- HOLD sends a zero-velocity command to the policy; it never freezes joints.
- Fall detection (pelvis tilt and height) switches to damping.
- MotionGuard checks every joint command.

Tests: every transition; abort while walking ends in a stable stance; a fall
ends in damping without torque spikes.

## H6 — Editor and recording (parallel with H3–H5)

Base pose, contacts, foot forces, joystick command, policy and fall state in
the editor, each also on an MCAP channel.

## H7 — Model-based control (after H4)

Do:
- Pinocchio binding for floating-base kinematics and dynamics that do not come
  from the simulator (Lane D rule).
- Extend the Lane D QP with a floating base, contact constraints and a
  centre-of-mass task.
- TSID-style inverse dynamics on ProxQP or OSQP.

First target: the policy balances the legs while the whole-body controller
tracks a hand target.

## H8 — Hardware (last)

- A streaming device profile (per-cycle setpoints at 500 Hz) reusing RKD6's
  handshake, fingerprint, clock sync and watchdog.
- For G1, a bridge to `unitree_sdk2` low-level control rather than our own
  MCU firmware.
- First runs on a gantry or harness.

## Out of scope for this lane

Rough-terrain locomotion, perception-driven footstep planning, dexterous hands,
and multi-hull collision decomposition beyond what the G1 model ships with.

## Progress log
