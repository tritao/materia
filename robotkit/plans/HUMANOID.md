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
- **HU-D2 — MJCF through MuJoCo's compiler, URDF through our own Haxe
  loader.**
  - MJCF means whatever MuJoCo's compiler makes of it: nested default classes,
    `childclass`, includes, angle and Euler conventions, `fromto`, inertia
    computed from geometry. A native tool loads it with the vendored MuJoCo
    (`mjSpec`) and emits a `RobotModel` plus mesh assets, so H2's conformance
    test compares physics, not two readings of the file. It needs a
    MuJoCo-enabled build, which is acceptable for an import step.
  - URDF is small and maps almost one-to-one onto `RobotModel`. A Haxe loader
    runs wherever Materia runs, including the editor and Wasm, without MuJoCo,
    and keeps visual meshes that MuJoCo's URDF import drops. It needs an XML
    parser in haxeon, added there rather than privately in RobotKit. Only
    expanded URDF is accepted; `xacro` stays an external step.
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

## H1 — Robot import (G1 into a RobotModel)

Problem: there is no way to bring an existing robot description in, and
`Simulation` gives each link a single collision shape (a box, or a convex hull
of at most 64 vertices). G1 collides through several primitives per link, and
its meshes are far larger than 64 vertices.

Do, in order:
1. **Several collision shapes per link.** `RobotModel` links carry a list of
   primitive collision shapes (box, sphere, capsule, cylinder, convex hull),
   each with a pose in the link frame. The blueprint or robot description
   carries them, and `Simulation` builds a compound shape per link from them.
2. **MJCF importer** (`robotkit/tools/mjcf_import`, C++, links the vendored
   MuJoCo): bodies, hinge and slide joints, free joint → `floatingBase`,
   limits, inertials, collision geoms → the shapes above, visual meshes,
   actuators with gear and force range, IMU sites → `Sensor` frames. Joint
   armature, damping and friction loss are read but stored only once H2 adds
   them to the model.
3. **Haxe URDF loader** on haxeon's XML parser (HU-D2).

G1 comes from MuJoCo Menagerie through a download script, not vendored; check
the model's own license. A small hand-written MJCF and URDF fixture pair runs
in the default tests.

Tests:
- the fixtures import every supported element;
- G1 total mass, link masses, inertias and joint limits match the MJCF within
  tolerance; the model renders in the editor;
- a G1 dropped on a floor lands on its feet's collision shapes, not bounding
  boxes.

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

### H0 — Floating base through the runtime

- `RobotModel.floatingBase` (schema v6) compiles into the blueprint's
  `floating_base`, which reuses the former `reserved0` slot, so the native
  layout is unchanged. The two checked-in v5 `robot.json` fixtures moved to v6.
- `Simulation` creates a floating robot's root as a dynamic body, which the
  MuJoCo backend turns into a free joint. `drive_robot_base` and the
  differential and omni couplings return `RK_ERROR_INVALID_STATE` for it.
  Teleport, place and reset already set the pose and zero the velocity.
- New `rk_simulation_get_robot_base_velocity` / `Simulation.robotBaseVelocity`.
- Tests: `robotkit/runtime/tests/mujoco.cpp` drops a two-box robot from 0.5 m;
  the floating one settles at 0.1 m with near-zero twist and resets to its
  initial pose at rest, while the kinematic one stays at 0.5 m. Codec,
  compiler and `RK_FLOATING_MOBILE` checks are in `RobotWorldTests`.
- The codec test needed `Reflect.deleteField`, which haxeon lacked; it was
  added in haxeon rather than worked around.
- Not covered yet: the deterministic SimKit backend with a floating base, and
  the editor's own robot records (`SensorConfiguration` in `app/`), which do
  not persist `floatingBase`. The editor gets it with H1's importer or H6.

### H1 — Robot import

- **Collision shapes.** `RobotModel` links carry primitive `collisionShapes`
  (box, sphere, capsule, cylinder, half-lengths along local Z), sent through a
  new `link_shapes` tail of `rk_simulation_robot_desc`. SimKit gained a
  cylinder shape. A description may now omit its initial pose (zero
  `struct_size`), so shape-only descriptions keep the default placement.
- **MJCF.** `robotkit_mjcf_import` (built when MuJoCo is) reads MuJoCo's
  compiled model. Geoms that take part in contact (contype, conaffinity or an
  explicit pair) become shapes; other meshes merge into one STL per link. The
  static MuJoCo is linked whole, or its self-registering STL decoder is
  dropped. The walker fixture's output is checked in and checked twice: byte
  for byte by the native import test, and semantically by `RobotWorldTests`.
- **G1.** `tools/humanoid/fetch-g1.sh` fetches Menagerie at a pinned commit
  and imports `scene_mjx.xml`, not `g1_mjx.xml`: the robot's geoms collide only
  through the scene's 49 contact pairs. Result: 30 links, 29 joints, 29
  actuators, 27 collision primitives, 2 IMUs, 33.34 kg. `DropCheck` drops it
  in MuJoCo through `RobotModel` → compiler → `Simulation`: the first floor
  contacts are both ankle-roll links at 0.032 s.
- **URDF.** `robotkit.model.urdf.UrdfLoader` (Haxe, on haxeon's new `Xml`)
  handles the child-frame joint convention, fixed-axis rpy, inertia in the
  inertial frame, a `world` root (fixed or floating), box/cylinder/sphere
  collision, mesh references, `mimic` → couplings and simple transmissions.
  Links without `inertial` get a 1 g placeholder mass, reported as a warning.
- **haxeon fixes made on the way:** `Reflect.deleteField`, enum constructors
  visible through module imports, `Sys.exit`, and the `Xml` stdlib with its
  compiler fixes. Still open: a comprehension whose block body ends in a
  filtering `if` does not compile.

Found for H2/H5, not fixed here:
- G1 cannot stand yet. Within 15 ms of touchdown both knees reach their stop
  (−0.087 rad), even with every joint held at zero: SimKit's position servo
  has none of G1's gains (kp 75, kv 2), armature or friction loss, which the
  importer reads but cannot store until H2.
- A joint pressed onto its stop goes slightly past it, since MuJoCo's limits
  are soft, and the runtime latches any observed out-of-limit position as a
  fault that fails `Simulation.step`. Legged robots rest on stops routinely,
  so H5 needs a penetration tolerance or a distinct stop state.
- Contact-pair friction and solver settings, joint armature, damping and
  friction loss, and actuator gains are reported by the importer as "not yet
  stored (H2)".
- Visual meshes are written and referenced, but the editor does not load mesh
  files yet (see the `gltf-animation` work).
- Contact pairs are approximated by layer collision: every imported shape
  collides with the environment and, unless self-collision is off, with
  other links.
