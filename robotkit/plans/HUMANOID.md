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
   The QP core itself is now `kinematicskit` (`kinematicskit/plans/KINEMATICS.md`,
   K3 and K5).

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
- **HU-D7 — Stand-up goes through a simulated support harness.** The G1 cannot
  hold its default pose on servos, and the pretrained policy was trained to start
  near it. The stand-up state is: supported, Ruckig to the default pose, the
  harness ramps its support down, the policy takes over. The harness is a SimKit
  environment feature built on the force API from H4, not a property of the
  robot model, and it mirrors the gantry that H8's first hardware runs use, so
  simulation and hardware share one state machine.
- **HU-D8 — The runtime keeps rejecting servo targets outside joint travel;
  MotionGuard clamps.** The controller emits raw targets. MotionGuard checks
  every servo batch, clamps with a configured margin, records each clamp and
  puts the count on an MCAP channel. It replaces the controller's own clamping.
  `observed_limit_tolerance` is a separate question, about compliant stops going
  slightly past a limit: derive it per robot from the model's limit softness,
  set by the importer or a humanoid profile, and never as a global default. A
  compiled model keeps 0 unless configured, so arms and CNCs are unaffected.
- **HU-D9 — A faulted robot keeps publishing its true state.** A fault means
  the robot rejects commands and goes to damping, not that it stops reporting.
  The state carries `safety = FAULT` and the fault code, so the editor and MCAP
  show the fall. The change needs a test that arm and CNC results are unchanged.
- **HU-D10 — The pretrained G1 policy and the imported 12-joint model are
  checked in.** `unitree_rl_gym` is BSD-3-Clause, but its repository does not say
  outright that the trained weights are covered. Keep a `NOTICE` beside the
  files naming the source repository, the pinned commit, the licence and that
  ambiguity.
- **HU-D11 — H7 picks the controller's model by what the controller needs
  (2026-10-01).** The controller never reads the simulation (HU-D4, Lane D
  LD-D1), but its model of the robot can come from either library:
  - **A whole-body QP** (inverse dynamics on ProxQP from the mass matrix,
    bias forces, contact and centre-of-mass Jacobians) takes them from a
    MuJoCo model of its own: a separate instance with the controller's
    parameters, never the simulator's state. MuJoCo is already vendored;
    mink (IK) and MuJoCo MPC use it the same way.
  - **TSID or gradient-based predictive control** (Crocoddyl, Aligator)
    needs exact derivatives of the dynamics, so it uses Pinocchio.
    - Pinocchio is built without collision support
      (`BUILD_WITH_COLLISION_SUPPORT` off): collision questions go to
      collisionkit (`collisionkit/plans/COLLISION.md`).
    - Its build requires Boost filesystem and serialization, the objection
      that kept OMPL out. Before vendoring, check what can be cut, as the
      coal fork did.

  Either way the model compiles from `RobotModel` (as the MJCF export does),
  and tests check its mass matrix and bias forces against the simulator's.
  The tests must account for:
  - the terms MuJoCo adds: armature, joint damping, passive forces;
  - the floating-base conventions. MuJoCo's quaternion is w,x,y,z and
    Pinocchio's x,y,z,w. MuJoCo's free-joint linear velocity is in the world
    frame, Pinocchio's in the body frame.

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
- Pick the controller's model by HU-D11: a MuJoCo model of the controller's
  own for a whole-body QP, Pinocchio for TSID or gradient-based predictive
  control. Floating-base kinematics and dynamics never come from the
  simulator (Lane D rule).
- Extend the Lane D QP with a floating base, contact constraints and a
  centre-of-mass task.
- Inverse dynamics on ProxQP, the QP solver kinematicskit's K3 also uses
  (`kinematicskit/plans/KINEMATICS.md` KK-D13/KK-D14), through TSID
  (stack-of-tasks) if HU-D11 picks Pinocchio.
- Build the controller's model directly from `RobotModel` (as the MJCF
  export does), not through URDF files, and check it against the
  simulator's (HU-D11).
- Pinocchio and TSID would be native dependencies of RobotKit's runtime.
  Before vendoring them, confirm the licences, pinned versions and the
  dependency set (Pinocchio's build requires Boost filesystem and
  serialization), and log the decision.
- Keep task definitions aligned with kinematicskit's (SE3 ~ `FrameTask`,
  posture ~ `PostureTask`).

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

### H2 — Physics fidelity for legged contact

Result: the conformance gate passes. `tools/humanoid/check-conformance.sh`
imports an MJCF file, steps it in plain MuJoCo (`robotkit_mjcf_reference`)
and through `RobotModel` → compiler → `Simulation` (`humanoid.Main
conformance`), and compares every 10–20 ms tick:
- the torque-driven biped fixture, 1.5 s through landing and toppling: within
  1.1e-7 m and 1.2e-7 rad;
- G1 from its `home` keyframe, servos holding and swinging ±0.1 rad, 2 s
  through toppling and hitting the ground: within 7e-8 m and 3.1e-7 rad.
The remaining ~1e-8 m offset is the starting pose's single-precision rounding
in the scene graph.

Added, bottom up:
- **SimKit:** a servo target mode (`NKSIM_JOINT_TARGET_SERVO`); joint
  armature, damping, friction loss and limit softness; shape contact
  surfaces (friction, friction dimensions, soft-contact time constant and
  damping ratio); contact filters and explicit contact pairs; integrator,
  friction cone and solver iteration limits; a cylinder shape; setting a
  joint's position as a starting pose.
- **RobotKit:** `RK_TARGET_SERVO` with a command `servos` tail (first 64
  joints); per-joint dynamics on the blueprint; shape surfaces, contact modes
  and contact pairs on the robot description; solver options on
  `rk_simulation_desc`; `rk_simulation_set_joint_positions`; ground-plane
  objects; `observed_limit_tolerance`.
- **Model:** `Joint` dynamics and limit softness, `Actuator` servo gains,
  `CollisionShape` surface and contact mode, `RobotModel.contactPairs`, all in
  schema v6; `JointTarget.servo`, recorded with its terms.
- **Import:** all of the above from MJCF, keyframes to `poses.json`, and URDF
  `<dynamics>`.
- The `TODO.md` actuator-limit regression.

What conformance found, in the order it found them:
1. Every G1 shape touched the floor, but MJX G1 collides only through its 49
   explicit pairs (23 with the floor, 26 between links); the foot boxes, 2 mm
   below the foot capsules, touch only other links. Hence contact modes and
   pairs.
2. A servo computed as explicit torque differs from MuJoCo's position
   actuator, whose damping `implicitfast` integrates implicitly. The servo is
   now a MuJoCo affine actuator whose gains are set each step and are zero
   outside servo mode, and its force limit is the joint's actuator force
   range, as MJCF's `actuatorfrcrange` is: MuJoCo drops an actuator clamped by
   its own force range from the implicit derivative, but not a joint-clamped one.
3. The tools used a box floor; G1's scene has a plane, and foot capsules
   touch the two differently once the feet tilt. Hence ground planes.
4. A shapeless link under the `none` collision approximation still touched
   the environment: clearing its mask did not stop the environment's own
   mask from catching its layer. Fixed in `Simulation`, with a regression test.
5. G1 sets its own joint-limit softness (`solreflimit`, `solimplimit`).
   Hence limit softness.
6. MJX G1 limits the solver to 5 iterations and 8 line-search iterations.
   Truncated solves depend on constraint order, which differs between the two
   compiled models, so the gate solves both to convergence (100 and 50); with
   5 and 8 on both sides they agree to about 1e-4 rad. Policies trained in MJX
   see the truncated solver; H4 should expect that level of difference.

Also found and fixed on the way:
- The runtime faulted on any observed position past a joint limit, and
  compliant stops (MuJoCo's, or a real joint's) always go slightly past. A
  blueprint may now set `observed_limit_tolerance`; the humanoid tools use
  0.05 rad. H5 still decides the humanoid default.
- A runtime test kept four `RobotRuntime`s and two plans on the stack and
  overflowed it as the blueprint grew; it now allocates them on the heap.

G1 cannot stand on joint servos alone, in either simulator: held at `home`
it tilts 0.4 rad within 1 s and falls by 1.5 s. `humanoid.Main stand` checks
standing and passes only once H4's policy balances it.

Still approximated: a pair with any world geom becomes contact with every
environment object, using that pair's surface. Pairs between two world
geoms, geom `solmix` and `solimp` on surfaces, and tendon or site actuators
are not imported.

### H4 — Policy runtime (first walk)

Result: Unitree's pretrained G1 walking policy, run through the ordinary
RobotKit runtime in MuJoCo on IMU and encoder observations only, stands,
walks at 1 m/s for 30 s, turns, survives pushes, and its run records to MCAP
and replays bit for bit. `tools/humanoid/check-walk.sh` is the gate;
`tools/humanoid/tests` holds the tests (`haxeon run --project
robotkit/tools/humanoid/tests/haxeon.json`, about 40 s).

Baseline verification, run first in the new worktree at ffcec085:
- The app tests did not compile: `EditorToolbarLayoutTests` imports both a
  module and one of its enums, which made every constructor of that enum
  ambiguous, so a bare `Full` did not resolve (haxeon 92396dba exposed it).
  Fixed in haxeon with a compiler test. All app tests pass after that.
- `robotkit/tests/world-tcp.sh` passes in all three modes (plain,
  `ROBOTKIT_TEST_SESSIONS=1`, `ROBOTKIT_TEST_LEASE_TIMEOUT=1`), and again after
  this branch's `robotkit/haxeon.json` gained an interface.
- The mixed-scene safety check (step 2 of the brief) is owned by another agent
  on `mixed-scene-safety`; nothing of it is here.

The policy:
- **Source.** `unitree_rl_gym` commit 276801e, `deploy/pre_train/g1/motion.pt`,
  trained in Isaac Gym with PPO and an LSTM actor (47 observations, 12 leg
  joint actions, 20 ms control period, 2 ms physics in Unitree's own
  sim-to-sim `deploy_mujoco`). No pretrained MuJoCo Playground G1 policy is
  published, and there is no GPU here to train one.
- **Licence.** BSD-3-Clause, Copyright Unitree Robotics. It permits
  redistribution in binary form with the notice, so the ONNX export
  (127 KB) and the imported 12-joint G1 model (19 KB, no meshes) are checked in
  with the licence text (`tools/humanoid/policies/unitree-g1/LICENSE`,
  `tests/fixtures/humanoid/README.md`). The repository does not say in so
  many words that the weights are covered; we treat them as part of it. STL
  meshes and the original TorchScript are not vendored:
  `tools/humanoid/fetch-unitree-g1.sh` fetches the model at the pinned commit.
- **ONNX export.** `policies/export_unitree_g1.py` makes the LSTM's state an
  explicit input and output (`obs, h_in, c_in` to `action, h_out, c_out`),
  because ONNX Runtime sessions are stateless. It checks the export against the
  TorchScript module over 200 random steps (1e-5 max difference).
- Kept out of the build: the export, `make_test_fixtures.py` and
  `reference_unitree_g1.py`, which runs the policy in plain MuJoCo as
  `deploy_mujoco.py` does and is the yardstick below.

Built:
- **ONNX Runtime binding** (`robotkit/policy`, library `robotkit_policy`,
  option `RK_BUILD_POLICY`, on in the MuJoCo/humanoid native build only). ONNX
  Runtime 1.30.0 (MIT) comes from its pinned prebuilt release, SHA-256 checked
  by CMake, or from `ROBOTKIT_ONNXRUNTIME_DIR` when offline; it is not built
  from source. Only linux-x64 has been built and run; the aarch64, macOS arm64
  and Windows x64 pins are the release's published digests, untested.
  Tensors cross the C ABI as flat `double` arrays in the model's own order, so
  the ABI knows nothing about the model; `OnnxPolicy` (Haxe) addresses them by
  name. Native and Haxe tests match Python's ONNX Runtime on the G1 model over
  a 100-step recurrent sequence.
- **Policy spec** (`policy.json`, `PolicySpec`): observation terms and scales
  in order, history length, joints, default pose, kp/kd, action scale, control
  period, recurrent state pairs, IMU mount, command limits, hold gains for
  joints the policy does not drive, and provenance. The controller checks it
  against the model and the network at construction.
- **Sensing (the H3 the policy needs).** `PolicyController` reads encoders and
  the base IMU and nothing else (HU-D4); `debugTruth` is the explicit debug
  switch, used only by tools comparing the estimate. Gravity comes from
  `GravityEstimator`, a complementary filter on gyro and accelerometer.
  Deliberately not done here, still H3: encoder quantization, IMU noise, bias
  and latency, foot contact forces.
- **Command reference and session.** `VelocityReference` is LD-D3's cyclic
  reference: sequence, deadline, limits, acceleration-limited ramp, brake to
  zero when the deadline lapses, stale/non-finite/expired commands rejected.
  `PolicySession` closes the loop over the `Robot` boundary (so a
  `RecordingRobot` records it) and submits `JointTarget.servo` batches.
  Zero velocity is "hold": the policy keeps balancing.
- **Force push.** SimKit could not disturb a running robot, so
  `nksim_session_submit_forces`, `rk_simulation_apply_robot_force` and
  `Simulation.applyRobotForce` apply a force at a robot's base for one tick.

Acceptance (`WalkTests`, on the checked-in fixture; 2 ms physics, Euler, 100
solver iterations, as Unitree's scene):
- stand 10 s: ends 0.14 m from the start, worst tilt 0.087 rad;
- walk 1 m/s for 30 s: mean forward speed 0.925 m/s over 5-30 s, 25.7 m
  forward, worst tilt 0.097 rad;
- turn: 0.5 rad/s walking and in place, both signs: heading changes 1.45-1.94 rad
  in 10 s, upright;
- push: 100 N for 0.2 s (about 0.6 m/s on 33 kg) forward, back, left and right,
  standing and walking: upright at the end every time;
- MCAP: a 6 s walk records 3000 snapshots (one per 2 ms tick) and 301 servo
  batches with their gains and targets, the recorded commands fed into a fresh
  simulation reproduce every joint position exactly (0 rad, same timestamps),
  and `ReplayRobot` preserves the command stream.

Sim-to-sim gap, measured (10 s, SimKit against `reference_unitree_g1.py`, same
model, same command; SimKit ramps a command at 1 m/s^2, the reference steps it):

| command (vx, vy, wz) | SimKit x, y, yaw | plain MuJoCo x, y, yaw |
| --- | --- | --- |
| 0, 0, 0 | 0.14, -0.04, -0.09 | 0.13, -0.18, -0.09 |
| 0.5, 0, 0 | 4.57, -0.30, -0.15 | 4.59, -0.39, -0.17 |
| 1.0, 0, 0 | 8.45, -0.53, -0.21 | 8.69, -1.04, -0.26 |
| 0, 0, 0.5 | -0.08, -0.04, 1.451 | -0.12, -0.06, 1.450 |
| 0.5, 0, 0.5 | -1.14, -1.21, 1.63 | -1.18, -1.18, 1.60 |

Standing pushes of 200 N for 0.2 s topple it in both simulators, forward and
sideways, and neither topples backwards; the end positions of the survivors
agree to a few centimetres. So SimKit adds no measurable gap on this policy
beyond chaotic divergence of a walking gait. The remaining distance to Isaac
Gym, where the policy was trained, cannot be measured here (no Isaac); Unitree
deploys the same policy from the same MuJoCo scene, and what we observe is what
that gives. Also run, not in the tests: the same policy on the 29-joint
Menagerie G1 (`policy-menagerie.json`, waist and arms held at zero, its own
IMU): it stands, walks 6.2 m in 10 s at 0.5 m/s and 30 s at 1 m/s without
falling, so it tolerates other inertias. Those runs use 2 ms Euler with 100
iterations, not MJX's truncated 5/8: the truncated solver was not tried.

Found, in the order it was found:
1. **The runtime rejects servo targets outside joint travel** and latches a
   fault (`RK_ERROR_LIMIT`), which failed `Simulation.step`. A policy's
   `default + 0.25 * action` goes past travel now and then, so the controller
   clamps its targets (95 of 18000 in a 30 s walk). In Isaac and plain MuJoCo
   the target is unclamped and the joint just meets its stop, so this is a
   small deliberate difference. Not a bug in the runtime: it owns position
   bounds.
2. **Sensor frames only refresh when `snapshot()` is called**
   (`RuntimeRobotAdapter.sensors()` returns the last ones), so a session must
   call it every tick. Reading the IMU once per control period aliased the
   gait's impacts into a 0.05 to 0.1 rad tilt bias and made the robot walk 25
   percent too fast. At tick rate (2 ms) the estimate stays within 0.002 rad.
3. **An accelerometer cannot be trusted while walking**: averaged over a gait
   it reads a gravity tilted 0.05 to 0.1 rad, and the more it corrects the
   worse the estimate. The filter therefore corrects only while the robot is
   still (|f| within 10 percent of g and under 0.1 rad/s) and otherwise
   integrates the gyro exactly.
4. **G1 cannot hold its default pose on servos, so there is no warm-up**: the
   policy must run from the first control tick, as in Unitree's own script
   (H2 already found G1 falls on servos alone). `PolicySession.start` has a
   warm-up argument for a robot that can hold still (a gantry); H5's stand-up
   state should replace it.
5. **The policy turns clockwise for a positive yaw command**, in plain MuJoCo
   too, at about a third of the commanded rate. The spec negates the yaw scale
   so RobotKit commands are counter-clockwise positive (REP-103).
6. The imported model keeps only primitive collision shapes: the pelvis and
   leg meshes that collide in Unitree's MJCF (contype 1) are skipped by the
   importer, so only the foot spheres touch the floor. This did not show up
   in walking, standing, pushes or the toppling comparison; a fall onto the
   torso will behave differently.
7. `Math.round` returns a 32-bit Int, so `Int64.fromFloat(Math.round(s * 1e9))`
   overflows after 2.1 s. Two tests caught it; the code uses
   `Int64.fromFloat(s * 1e9 + 0.5)`.
8. Two haxeon bugs, fixed there with tests: an enum imported by module and by
   name lost its constructors, and FFI interfaces were registered in path order
   so `robotkit/policy/...` (depends on RobotKitRuntime) came before
   `robotkit/runtime/...`; the session now registers dependencies first.
9. The disk filled during the work (other checkouts' builds) and truncated
   two files being written; they were rewritten, and nothing committed is
   affected.

Not done, and where it goes: `robotkit/TODO.md`.

### Plan for H3–H5

Decisions HU-D7 to HU-D10 (above) settle the open questions from H4. Order:
0. Verify `main` after the H0–H4 and mixed-scene merges: the Haxe suites, the walk
   gate, the three `world-tcp.sh` modes, and the cnckit, toolpathkit, motionkit and
   app tests.
1. H3: encoder quantization, IMU noise, bias and latency, per-foot contact force,
   and an observation assembler that only reads sensors. Sensor frames refresh
   every tick. Re-tune `GravityEstimator` against a biased, noisy gyro, and run
   the walk gate at a mild noise level to see how much slack the policy has.
2. H5: the support harness (HU-D7), the state machine (passive/damping, stand-up,
   policy, sit-down, fault), HOLD as a zero-velocity command, fall detection,
   MotionGuard on every servo batch (HU-D8), and faulted robots that keep
   publishing (HU-D9).
3. Mixed-scene follow-ups inside H5: collision shapes for the pelvis, torso and
   hands so a fall lands on the torso and arms cannot pass through it, and the
   humanoid at 2 ms beside the CAM pocket scenario.
4. H4 leftovers: the truncated MJX solver, and MCAP channels for the command
   reference, gait phase, estimate and clamped-target count.
