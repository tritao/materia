# Mixed-scene safety

Robot arms, CNC machines and a floating-base humanoid (Unitree G1) share one
SimKit/MuJoCo session in Materia. Humanoid phases H0-H2 (`HUMANOID.md`) ran one
robot at a time, so "they do not disturb each other" was an inference. This
note checks it, with the code read first and a test for each claim. Branch
`mixed-scene-safety`, on top of `main` at ffcec085. Line numbers below are
those of ffcec085 unless marked "now".

Tests:
- `robotkit/runtime/tests/mixed_scene.cpp` (`robotkit_mixed_scene_tests`):
  hand-built arms, an XYZ gantry and a legged floating-base robot in one
  MuJoCo `Simulation`. It runs in seconds and needs no fixtures.
- `robotkit/tools/humanoid/src/humanoid/MixedScene.hx` (`humanoid.Main mixed
  <g1 robot.json>`): a UR-class arm, MachineKit's `compileXYZGantry` and the
  real G1 from `tools/humanoid/fetch-g1.sh` (network was available).

## Results by hypothesis

### 1. Solver settings are world-wide. Confirmed; the arm and CNC stay valid.

Evidence:
- One `rk_simulation_desc` per world: `robotkit_simkit.h:52-61` (timestep,
  substeps, integrator, friction cone, iteration limits), copied into
  `nksim_world_desc` at `simulation.cpp:172-178`, and into MuJoCo's options at
  `mujoco_backend.cpp:292-315`. `step()` sets `opt.timestep = dt / substeps`
  for the whole model (`mujoco_backend.cpp:590`).
- Every runtime's period is that world timestep, not the blueprint's
  `owner_period_ns` (`simulation.cpp:164`, `:656`). `owner_period_ns` (10 ms
  default) is read only by standalone runtimes and RKD6 devices
  (`runtime_c_api.cpp:45`, `device_compiler6.cpp:100`, `rkd6_endpoint.cpp:133`).
  So a humanoid scene at 2 ms also samples the arm's and the CNC's trajectory
  queues, commit lead (2 periods) and rate limits every 2 ms.
- The MuJoCo constraint solver is one model-wide object: its convergence scale
  is `1 / (meaninertia * nv)` over the whole model
  (`simkit/vendor/mujoco/src/engine/engine_solver.c:481, 784, 2833`).

Test (`world_settings_are_shared`): the same arm and gantry job at 10 ms x5
(default Euler), at 2 ms x2 implicitfast, and at 2 ms with MJX G1's 5 and 8
iteration limits. Tracking is as good (arm 0.19 rad, gantry 8 cm at 10 ms;
0.20 rad and 10 cm at 2 ms; the job is deliberately fast), poses differ from
the 10 ms run by at most 8 mm (arm) and 1.7 mm (gantry), and the iteration
limits change nothing while nothing is in contact. With a far humanoid added
at any of the three settings the arm and gantry are bit-identical to the same
setting without it. What is not covered: a truncated solve (5 and 8
iterations) on arm or CNC contacts. It will differ from a converged one, as
G1's own conformance run found (`HUMANOID.md`, H2 item 6).

Take-away: a scene has one timestep and one solver. If a humanoid needs 2 ms,
the arm and the CNC run at 2 ms too. Their results move by millimetres, not
by errors, and the plans they execute are re-sampled at the new period.

### 2. The observed-limit fault fails the whole session step. Confirmed; fixed.

Evidence and the two paths, both fatal to everyone:
- Publish: `runtime.cpp:1628-1631` latches a fault when an observed position
  is past its limit and returns `RK_ERROR_LIMIT`. `Simulation::publish`
  (`simulation.cpp:1484-1495`) returned on the first failing runtime, so
  robots after it in the list were not published (order-dependent), the step
  returned the error, and in a realtime session `session.cpp:649` left the
  loop, stopping every robot. The physics had already advanced.
- Apply: a robot in FAULT or e-stop rejects every non-reset command with
  `RK_ERROR_SAFETY_STOPPED` (`runtime.cpp:1015, 1035`). `Simulation::prepare`
  (`simulation.cpp:1423-1434`) returned it, `session.cpp:604-611` discarded the
  tick for everyone and did not advance the world (manual steps), and the
  commands of the other robots in that tick, swapped out of their mailboxes at
  `runtime.cpp:671`, were rolled back and lost.

So a fallen humanoid whose policy keeps commanding it froze every arm and
machine, and dropped their commands.

Measured before the fix (test written first):
- C++: toppled humanoid, arm and gantry behind it: 35 of 400 steps fail
  (the joints sit past their stops); with the humanoid added first the arm
  and gantry traces change by 5.4 and 3.4; with a policy sending a batch every
  tick, 376 of 400 steps fail and the arm and gantry change by 42 and 20.
- Real G1 (`humanoid.Main mixed`): 292 of 300 steps fail, the arm's trace
  changes by 174, the gantry's by 1.8.
- `DropCheck` on the real G1 already showed it ("simulation.step failed with
  RobotKit status -10" at 1.834 s, `HUMANOID.md` progress log).

Fix (`b911c5d5`): `Simulation::prepare` and `publish` visit every robot; a
robot that fails either phase goes through the new `RobotRuntime::fail_tick`
(roll back its own half-applied commands, latch its fault, unless it is
already faulted or e-stopped) and the tick continues. `Simulation.step` returns
OK; the fault is in that robot's state. This replaces the transactional
"one rejection rolls back everyone" tick, and the test that asserted it
(`failed_command_phase_does_not_advance`, `simkit.cpp`) now asserts the
opposite (`rejected_command_faults_only_its_robot`). Two `mujoco.cpp` asserts
that read a fault off `step` read it off the robot instead. `DropCheck` reads
the safety state.

Consequence to know about: `Simulation.step` no longer throws for a robot's
fault. Code that used the exception to notice one must read
`snapshot.safety == RK_SAFETY_FAULT`. Two things I noticed and did not change:
a faulted robot's snapshot freezes at its last in-limit sample while a joint
stays past its limit (`publish_sample_impl` returns before storing the sample),
so a supervisor sees stale joint positions for a fallen humanoid; and
`observed_limit_tolerance` still defaults to 0 in the compiler, so a G1
compiled without the tools' 0.05 faults the first time it presses a stop
(H5 decides).

After the fix (`mixed_scene_tests`, `mixed` on the real G1): 0 failed steps in
every case, arm and gantry never fault, a realtime session keeps ticking, and
the toppled humanoid faults alone.

### 3. Cross-robot contact. Two real problems; fixed.

How contact is set up (`simulation.cpp:537-540`, `mujoco_backend.cpp:1067-1079`,
`:1352-1355`):
- Every robot link with a shape, and every environment object, is on layer 1
  with mask 1, so by default layered shapes of all robots and objects
  collide with each other except inside one articulation (parent-child and
  rest-overlapping pairs are excluded, `add_self_collision_excludes`).
- A contact shape's filter (`NKSIM_CONTACT_*`): LAYERS collides as above; PAIRS_ONLY
  collides only through explicit pairs (contype and conaffinity 0);
  PAIRS_AND_ENVIRONMENT (G1's foot capsules) also pairs with "every body no
  joint connects".
- Every link gets SimKit's default 10 cm box as its shape
  (`simulation.cpp:419`); under the `none` approximation a link with no shape
  of its own keeps it, on layer 0 and mask 0 (H2's fix), which collides with
  nothing by layers.

What G1 (all shapes pairs-only or pairs-and-environment) touched before the
fix, measured (`contacts_follow_the_contact_filter`, a leg with something
overlapping it):

| leg shape filter | arm link | gantry link | static box | dynamic box | shapeless none robot |
|---|---|---|---|---|---|
| layers (0) | touches | touches | touches | touches | no |
| pairs only (1) | no | no | no | no | no |
| pairs and environment (2), before | **no** | **no** | touches | touches | **touches** |
| pairs and environment (2), now | touches | touches | touches | touches | no |

Problems, both in the pairs-and-environment path:
1. A robot of one link, no joints and no collision shape under `none` counted
   as "environment" (no joint connects it) and its invisible 10 cm box was
   paired with a humanoid's feet: H2's "shapeless links touch nothing" was
   true for layers only. Pre-H2 such a link touched objects too (a dropped
   box rested at 0.15 m on it, measured on 8acfd1f2; it now passes through,
   -18 m after 2 s, which is the intended H2 behaviour).
2. Every link of an arm or a CNC machine has a joint, so it was never
   environment. A G1 dropped on a machine bed (a gantry base link, 1 m x 1 m
   x 10 cm at 0.5 m) fell through it to the floor 0.55 m below: torso at 0.66
   instead of 1.21. The same held for the arm's base and links. Humanoids
   stand on floors that are objects, and on nothing else that a mixed scene
   has.

Fix (`ebf1bfbf`, `mujoco_backend.cpp` `add_contact_pairs`): a
pairs-and-environment part now pairs with every part outside its own
articulation that (a) is layered, and (b) sits on a layer the shape's body's
mask meets, or whose mask meets the shape's layer. A shape on layer 0/mask 0
collides with nothing, and another robot's pairs-only parts are not
environment. Conformance for G1 is unchanged (`check-conformance.sh`: 7.3e-8
m and 3.1e-7 rad, the values `HUMANOID.md` reports for H2).

Intended, and still open:
- With this, a G1 foot touches arm links, CNC links, objects and the floor,
  and only those. G1's torso, hands and legs above the foot are pairs-only,
  so an arm or a CNC carriage can still pass through them, and the G1's own
  non-foot links pass through machines. That preserves MJX's contact model,
  which policies train against, and is a choice for H5 (safety) to revisit:
  cross-robot collision with the whole humanoid needs the importer to keep
  layer shapes next to the MJX pairs.
- Two humanoids' feet do not touch each other.
- The `SimKit` deterministic backend (backend 0) is unaffected by any of this.

### 4. H2's second idle servo actuator and joint-level force limit. Confirmed unchanged.

Evidence: `add_joint_actuators` gives every non-fixed joint a motor and, since
H2, a servo actuator with zero gain and bias outside servo mode
(`mujoco_backend.cpp:1096-1130`); every step sets `jnt_actfrcrange` from the
joint's effort limit (`:1483`), a bound the motor's torque already respects.

Measured: the same arm + XYZ gantry + machine bed + dropped box job (400
ticks, MuJoCo) built at 8acfd1f2 (before H2) and at this branch, every
joint position, velocity, effort, link pose and box pose printed as hex
doubles: byte-identical for position targets at 10 ms x5 and at 2 ms x2, for
velocity targets up to the tick the old code faulted (370), and for effort
targets up to the tick the old code faulted (28). (Those two stopped early
because the pre-H2 run fails its steps at a joint limit, which is problem 2.
The script was `trace.cpp` in the scratchpad; it is not committed.)

### 5. Scheduling. Confirmed coherent and deterministic.

- One clock: `Simulation::step` drains every mailbox, applies every robot,
  submits, advances the host once, and publishes every robot from one
  snapshot (`ARCHITECTURE.md`, "One simulation tick"). Every runtime has the
  session's period; physics substeps subdivide that period only.
- A policy at ~50 Hz is an external client that puts a batch in a robot's
  mailbox; the batch applies at the next tick. It does not tick anything.
  Between policy batches the runtime holds the last target.
- Test (`mixed_scene_steps_coherently_and_repeatably`): arm and gantry have
  the same source timestamp and sequence every tick; the same scene run twice
  is bit-identical for every robot; the result does not depend on the order
  robots are added in (`far_humanoid_...`, humanoid first and last); a
  realtime session (`realtime_session_survives_a_humanoid_fault`) keeps ticking
  after the humanoid faults and the arm reaches its target.
- What differs from "coherent": a faulted robot's own state stops at its last
  in-limit sample (see 2). That is its own state.

## Arm and CNC results with a humanoid in the session

`far_humanoid_does_not_change_arm_and_gantry` (C++, 3R arm and XYZ gantry with
box collision shapes) and `MixedScene.hx` (UR-class 6R arm, MachineKit
gantry, real G1 10 m away on a plane):

- Robots with no constraint of their own (the gantry; the 3R arm; the UR arm
  with self collision off): bit-identical to the run without the humanoid,
  whether it stands, topples onto its joint stops, or is commanded after it
  faults, and in either add order. MuJoCo solves constraints per island, so
  dofs with none are `qacc_smooth` exactly.
- A robot with active constraints of its own: the UR-class arm's 10 cm boxes
  touch each other (4 active contacts at most). It changes at solver round-off
  when any robot is added: 1e-16 (one ulp) to 4e-12 in the traces of the C++
  and G1 scenes. Cause: only partly pinned down. Removing the arm's contacts
  makes it bit-identical, so the arm's own island solve is what sees the rest
  of the model. The size of the change does not depend on the humanoid's mass
  or on its dof count (4.12e-13 for 12 kg and 3000 kg, for 0 to 29 extra
  joints; 3.7e-13 at 60), so it is not the model-wide solver tolerance scale
  (`meaninertia * nv`, `engine_solver.c`) that I first suspected. I did not
  find which ordering or warm-start detail differs; the tests bound it at
  1e-9 (`arm_constraints_see_only_solver_round_off`, `MixedScene.hx`).

## Also run

All on this branch, with `CADKIT_OCCT_DIR` pointing at the prebuilt OCCT:
- MuJoCo native tests (`robotd/native-mujoco`, Release with asserts on):
  `robotkit_mujoco_tests`, `robotkit_simkit_tests`,
  `robotkit_mujoco_backend_tests`, `robotkit_mixed_scene_tests`, and the
  runtime, c_api, validation, virtual device, RKD6, recording, device compiler,
  clock, wire and segment vector tests: pass.
- SimKit standalone (`-DNK_BUILD_TESTS=ON -DNKSIM_BUILD_MUJOCO=ON`): 6 of 6 sim
  tests pass (run after the contact change).
- `check-conformance.sh` with G1: pass, unchanged.
- `haxeon run` of `robotkit/tests` (the default-backend wall-finishing
  scenario among them): pass; `robotkit/tests/mujoco` (wall finishing on
  MuJoCo, tracking max 5.4e-8): pass; `cnckit/tests`: 304 assertions pass.
  (The first `robotkit/tests` run exited 1 at the wall-finishing scenario
  while the disk was full and passed on rerun.)
- `toolpathkit/motion/tests` (ToolpathKit motion tests, the "Machining run"
  CAM pocket with feed hold, restart and spindle fault at 10 ms, and the toolpath
  scenarios): pass. `motionkit/tests`: 6693 assertions pass.
- `app/tests` (scene editing, stock, human simulation, scene documents, CAD
  workflow) and `app/tests/project-source` (`app.ProjectSourceTests`: the
  MachineKit assembly on both backends, link and hull collision with the
  `none` approximation): exit 0. The app tests do not compile with this
  branch's pinned haxeon (`EditorToolbarLayoutTests.hx:8: E1005 Unknown
  variable "Full"`, an enum imported through its module and by name); I ran
  them with haxeon 3d96aff7 from the `humanoid-h4` branch's haxeon and put the
  pinned one back. Their native build needed the animkit vendor submodules
  (cgltf, ozz-animation, stb) and the prebuilt OCCT; `0658001a` is a
  cherry-pick of `493d7120` (`cadkit-prebuilt-occt`) for the latter and can be
  dropped when that branch merges.
- `none` approximation with shapeless links: no CNC (`MachineKitRobotCompiler`)
  or arm scene in RobotKit, MotionKit or ToolpathKit sets it, so they use
  bounding boxes and are not affected. It is used by the app's MachineKit
  assembly bridge (`ApplicationSimulation.hx:139`, parts whose collision is
  off are shapeless; ones with a collision hull get a shape), by
  `UrdfLoader` and the MJCF importer (with shapes), by the editor's saved robot
  configuration (`SensorConfiguration.hx:384`, which can store `none`), by the
  app tests
  (`ProjectSourceTests.hx:57-133`, `HeadlessEditorProfile.hx:386`) and by the
  H2 fixtures. For those shapeless assembly parts the H2 layer change was
  real: they used to touch spawned objects (measured), and now touch nothing.
  That is what "collision off" means.

## Open questions

- Should a robot's fault also publish its state each tick (with the FAULT
  flag) rather than freeze at the last in-limit sample? (H5.)
- Should the compiler default `observed_limit_tolerance` for floating-base
  or legged models, so an imported humanoid does not fault on its first
  compliant stop? (H5, `HUMANOID.md`.)
- Whole-humanoid collision with arms and machines (see 3).
- One world means one timestep and one solver. A humanoid at 2 ms therefore
  runs the CNC's plan queue at 2 ms (commit lead 4 ms). The Haxe motion
  drivers take `dt` per `update`, so that works, but the CAM pocket scenario
  (`MachiningRunTests`) has only been run at 10 ms.
- `Simulation.step` no longer reports a robot's fault. Anything that relied on
  the thrown error must read the state.
