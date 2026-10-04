# Gantries plan

**Goal.** Gantries become first-class machines, not only "the CNC router". This covers three kinds, each
building on the one before:

1. **Cartesian pick-and-place gantry.** X, Y and Z, a dual-drive Y, and a tool that always points
   down. It runs the same missions as the arm, at the speeds its motors allow, and homes against
   switches the way a real machine does.
2. **Process gantry with a rotary head.** It adds C (spin about Z) and A (tilt) axes, so a torch or
   gripper can be oriented. Processes say which parts of the tool orientation they need, and the
   planner leaves the rest free.
3. **Arm on a gantry or track.** A 6R arm rides on gantry or track axes. The track joints are
   external axes that the planner moves cheaply and coordinates with the arm.

The machines are built from parts as the router and CoreXY plotter are. Their limits come from their
drives, and nothing is written twice. See [TRANSMISSION_PLAN.md](TRANSMISSION_PLAN.md) for the drive
model this builds on.

Branch `gantries`, worktree `/home/joao/dev/materia-worktrees/gantries`. It is based on local main
`712019dbc`:
- transmissions X1–X9 complete, with X9's full-suite gate passed;
- the robot welder W0–W3 and the first part of W4: torch tools, `weld` steps, `WeldingPlanRunner`,
  `ArmClearance`;
- transmissions X10 (belt reductions and loops between shafts, its gate passed) and the X9e review
  fixes;
- the RobotKit restructuring plan, `robotkit/RESTRUCTURE_PLAN.md`.

The merge commit says it was not built or tested after the merge.

The survey below was taken at `aaa2b911a` (X7 T1). X7–X9 renamed `Drive`→`Transmission`, split the
motor and driver, put the drive-derived limits into the compiled `RobotModel`, and removed
saved-data compatibility.

## What exists (survey 2026-10-03, at aaa2b911a)

**Machines.**
- `CncRouter` (`machinekit/examples/cnc-router/CncRouter.hx`) is already a gantry.
  - Joints `x`, `y` and `z` are prismatic, in mm.
  - Dual Y: one screw and motor per side, both coupled to the single `y` joint. The right Y block is
    attached rigidly to the gantry.
  - Belts option, overtravel from the rail room, motors as actuators. Speed and acceleration come
    only from `RobotModel.coupledLimits` (`slide()` passes `velocity: null`).
- `CoreXyPlotter` (`machinekit/examples/corexy/`) uses two-leader couplings. It is not on the Start
  page.
- Both duplicate the same private builder helpers: `place`, `attach`, `hang`, `slide` (overtravel
  from rail room), `connect` (world-aligned `to-<child>`/`attach-<child>` connectors), `zeroPose`,
  `orient`, `driveScrew`, `mountNut`, `turnWithBelt`, `beltX`, `beltY`.
  - Router: `CncRouter.hx:637-837`. CoreXY: `CoreXyPlotter.hx:247-330`.
- `machinekit.assembly.LinearAxis`: a screw axis with round rods or a profile rail, built along +Z
  with the motor at the origin. It has no `addMotor` and no example uses it.
  - Only the motionkit test path uses it: `MachineKitRobotCompiler.compileAssemblyAxes`,
    `compileLinearAxis` and `compileXYZGantry`.
  - That compiler builds its own `"xyz-gantry"` `RobotModel` from three `LinearAxis`, a second model
    path beside the app's `AssemblySimulationBridge`.

**Robots and missions.**
- Any `MachineAssembly` becomes the scene's single implicit robot (`app/src/AssemblyRobot.hx`).
- `MissionPlayer` builds `new Manipulator(model, model.links[0].id, tcp)` from the root to the tool
  contact, so nothing there requires an arm.
- Pick/place keeps the home orientation (`HandlingPlanRunner.hx:89-101`), so a fixed-orientation XYZ
  gantry can probably run it today. This has not been tried.

**IK.**
- kinematicskit already has partial tasks:
  - `FrameTask(…, positionAxes = AXIS_X|AXIS_Y|AXIS_Z, orientation:FrameOrientation)`, with
    `FrameOrientation {Full; Axis(x,y,z); Free}`.
  - All solvers size by `rowCount()`, so they handle the reduced rows.
  - Tested in `KinematicsKitTests.testMaskedPositionAndAxisTasks`.
- RobotKit never uses them:
  - `KinematicGroup.toolTask` (`robotkit/haxe/robotkit/manipulation/KinematicGroup.hx:448-455`) and
    `solveWithBase` (419-441) always build all axes plus `Full`.
  - `IkOptions` has no orientation field.
- MotionKit's `OrientationPolicy {Fixed; Interpolated; Cone; FreeAboutTool}` only relaxes the
  after-the-fact task-space check (`ProgramCompiler.orientationError`, ~812-831).
  - IK still gets the full slerped pose, and `solveDifferential` gets the full angular twist.
  - `ManipulatorKinematics.solveDifferential` with n ≤ 6 silently takes a least-squares step
    (`:167`).

**External axes.**
- `KinematicGroup(…, externalAxes, externalWeight)` exists, as do `ExternalAxesParameterization`,
  `RedundancyResolver` and `DofDampingTask`.
  - MotionKit `testCoordinatedExternalAxes` covers a rail under a 6R arm plus a turntable, building
    the `KinematicGroup` by hand.
- `Manipulator` passes no external axes, so a 6R arm on one track joint counts 7 arm joints.
  `KinematicGroup:151` then builds a wrong swivel through the track joint.
- The plan runners (`HandlingPlanRunner`, `SurfacePlanRunner`, `ToolpathPlanRunner`, and on
  mobile-welder `WeldingPlanRunner`) accept only a `Manipulator`.

**Limits.**
- All plan runners take `group.limitsOf(j).velocity`, falling back to 2.0 (10.0 in `ToolpathPlanRunner`),
  and one scalar acceleration (`MissionPlayer.ARM_ACCELERATION = 2.0`).
- `RobotModel.coupledLimits(id, ?steady)` is used only by `RobotRuntimeCompiler` and
  `CncProgramPlayer`.
- The result: missions plan above what the runtime enforces, and a router-like gantry would be
  planned at 2 m/s.

**CNC route.**
- It is XYZ only throughout:
  - cnckit rejects A/B/C words (`CncInterpreter.hx:57-66`).
  - `Point3`/`PathGeometry` carry no tool axis.
  - `MachineBinding` and `AxisKinematics` require three axes. `AxisKinematics` rejects rotation
    (`:52-53`, `:105-106`).
  - `CncProgramPlayer` requires exactly three prismatic axes.
  - SceneArtifact `machining.axes` must have three entries.
- camkit is 2.5D.

**Homing and switches.** There are none.
- `MotionSystem.home()` moves to authored coordinates.
- No switch part, sensor kind, runtime "referenced" state, RKD6 input message, or `Board` input read.
- The simulated end stop is the MuJoCo joint limit, at the soft limit plus overtravel.
- Hooks a switch can use:
  - host-side thresholding as `EncoderReading` does;
  - a `SimulationStepObserver` publishing a digital external sensor frame, as `tool_contact` and
    `EndEffectorVacuumFeedback` do;
  - `Simulation.setJointSlip` for unknown power-up offsets;
  - `EncoderMonitor.reset` and `StepperSlip.reset`, which already say "as homing does".

**Dual-drive skew.**
- RKD6 and the device have a `DUAL_DRIVE_SKEW` fault, but skew groups form only between actuators
  with the same `actuator_joint` (`device_virtual/src/lib.rs:143-155`).
- The host sends each motor's own shaft joint (`DeviceBinding.hx:85-101`), so the router's two Y
  motors do **not** form a skew group.
- `DeviceLayout.forActuators` sets no `skew_bound`.

**Tools.**
- `EndEffector.mount(instance, connector)` is generic.
- `EndEffectorPlate` needs a `RobotFlange`, and the arm's `ArmTool` interface takes a `RobotFlange`
  and returns six ready joint values.
- On `mobile-welder` (not merged, W4 uncommitted):
  - torch tools (`robotTools` kind `torch`) and `weld` mission steps;
  - `WeldingPlanRunner`, which treats the last three joints as the wrist (`:156-161`) and solves
    full 6-D poses;
  - `ArmClearance`, which is generic over any `KinematicGroup`.

**Related in-flight work.**
- **X7** (Codex, worktree `x7-transmissions`):
  - T2 renames `Drive`→`Transmission`, `addDrive`→`addTransmission`, `DriveRecord`→`TransmissionRecord`,
    and adds `Sense` and a wire `ScrewSupport`.
  - T3–T5 derive stiffness, backlash and allowances, delete `LeadScrewTransmission`, and add
    provenance.
- **X8** splits motor, driver, supply and controller: `addMotor(id, joint, motor, driver, margin,
  ?gearbox)`, with microsteps in the actuator.
- **Machine tending** (worktree `machine-tending`):
  - MT1 adds ball screws and HGR rails.
  - MT3 moves `ArmTool` into `machinekit.robotics` and adds `CobotArm`.
  - MT6 adds scene `controllers` (several robots from one assembly).

## What is missing

1. Missions plan with arm defaults instead of the drives' limits.
2. IK and path planning can't leave part of the orientation free. A 4- or 5-axis machine can't
   follow a full 6-D pose.
3. External axes are not derived, the 7-joint swivel misfires on arm + track, and the runners can't
   take a `KinematicGroup`.
4. There is no reusable gantry assembly. The axis builder helpers are copied between machines.
5. There is no homing, no switches, no referenced state, and no dual-drive squaring. The router's
   dual Y isn't skew-guarded on the device.
6. There are no rotary heads, no gantry tool flange, and no tool interface that isn't arm-shaped.
7. There are no examples of any of the three kinds.

## Decisions

**G-D1. The process states its tool freedom; the robot does not.**
- A step or path says which orientation it needs:
  - **pick**: tool Z down, spin free unless the part's yaw matters;
  - **weld**: torch axis along the work and travel angles, spin free;
  - **CNC**: tool axis fixed.
- Freedom maps to one kinematicskit `FrameOrientation` plus position mask per IK call.
- A robot that can't provide some freedom simply fails IK for poses that need it, with a clear
  diagnostic: "tool axis unreachable: machine has no tilt axis".
- **Rejected:** per-robot "orientation capability" flags. They duplicate what the kinematics
  already know.

**G-D2. One mapping from path policy to IK task, in one place.**
- MotionKit's `OrientationPolicy` gains `Free` (position only). The others map as follows:
  - `Fixed`/`Interpolated` → `Full`;
  - `FreeAboutTool` → `Axis(tool Z)`;
  - `Cone(axis, half)` → `Axis` with `orientationTolerance = half`.
- The mapping is used by `solvePose`, `solvePath` and `solveDifferential` (reduced twist rows),
  and by the task-space check, so IK and check can never disagree.

**G-D3. Planning limits come from the drives everywhere.**
- One helper, `PlanningLimits.of(model, jointIds, ?steady)`, built on `RobotModel.coupledLimits`,
  gives per-joint velocity, acceleration and jerk. It is used by `CncProgramPlayer`, every plan
  runner and `MissionPlayer`.
- Joints whose drives give no acceleration (revolute arm joints today) fall back to their own
  `maxAcceleration`, then to the runner's stated default, which is reported as assumed.
- **Rejected:** keeping scalar `maxAcceleration` arguments on the runners.

**G-D4. External axes are derived from the assembly.**
- The arm is the include that owns the `RobotFlange` member on the root-to-tool path. Joints on
  that path outside it are external.
- `Manipulator` derives this, so swivel detection counts only arm joints.
- The scene never lists external axes.

**G-D5. A gantry ends in a standard tool flange.**
- The Z carriage carries the same `RobotFlange` part (ISO 9409-1 pattern) as the arm, so every end
  effector (`EndEffectorPlate`, suction, gripper, torch) fits both.
- The arm-specific parts of `ArmTool` (`ready()` with six values) stay with the arm. Gantry tools
  use the generic interface MT3 introduces in `machinekit.robotics`; if MT3 hasn't landed, put the
  shared interface there yourself and note it for MT3.
- **Rejected:** a gantry-specific tool mount.

**G-D6. A gantry is one `MachineAssembly` built from shared axis-builder pieces; machines differ
only in their frame and spec.**
- `machinekit.gantry.Gantry` is built from a `GantrySpec`:
  - travel per axis, and the drive per axis: screw, belt or rack-and-pinion;
  - single or dual Y;
  - rail profile, motor, and head: none, C, or C+A;
  - beam and frame size.
- The router and the CoreXY plotter keep their own frames but use the same builder.
- Their baseline numbers must not move when they migrate.

**G-D7. Dual-drive axes are guarded and squared per axis, not per motor shaft.**
- A skew group is the set of actuators whose shafts follow the same leader joint.
- The skew bound is derived from a stated racking tolerance on the gantry spec: default 0.5 mm,
  marked assumed until X7 T5 provenance can carry it.
- Squaring homes each side against its own switch.

**G-D8. Switches are parts; homing is a motion program; "referenced" is runtime state.**
- `LimitSwitch`/`HomeSwitch` parts (mechanical micro-switch, inductive proximity) mount on the
  machine. Each declares:
  - the joint it watches;
  - a trip point (from where the trigger and target meet in the geometry, not typed in);
  - a side;
  - hysteresis;
  - repeatability;
  - a `Signal` port.
- The simulation reports trips from joint positions.
- An unreferenced axis accepts only jog and homing moves.
- `MotionSystem.home()` runs the homing cycle when the machine has switches, and keeps the current
  software home when it doesn't.
- The board input that reads a switch is deployment data, like encoder wiring (X8 keeps the
  controller out of the model).

**G-D9. CNC stays three-axis.**
- Rotary heads are for missions and process paths (welding, picking).
- **Rejected for now:** 4/5-axis G-code (A/B/C words), tool-axis toolpaths and 5-axis CAM. That is
  a separate CAM project ("Later").

**G-D10. One robot per assembly until MT6.**
- A gantry with an arm on it is one robot with one controller.
- A scene that has both `machining` and `mission` is rejected at validation. Today they would drive
  the same robot from two players.
- Cells with several independent robots wait for MT6 `controllers`.

## Coordination with the RobotKit restructuring (R0–R6)

`robotkit/RESTRUCTURE_PLAN.md` was planned on 2026-10-03, to start after X9e, which is now on main.
It moves and renames code this plan touches. Rules:

- **Placement (R2).** New robot *kinds* are a `RobotProfile`, never a new `RobotModel` field.
  - G3's derived external axes belong in the `Manipulator(…)` profile once R2 has landed. Until
    then, derive them in `Manipulator`, in one function that R2 can move as is.
  - Switches (G10) are physical sensors, so they belong on the model, as encoders are.
  - "Referenced" (G11) is runtime state, so it belongs in the runtime.
- **Process code (R3).** `SurfacePlanRunner`, `ToolpathPlanRunner`, `WeldPlan`/`WeldRunner`/`WeldSeam`
  and `SimulatedWelder` move to ProcessKit. Keep G1's and G15's changes to them small and
  self-contained, so they merge across that move. If R3 has landed, edit them in ProcessKit.
- **Packages (R5).**
  - Generic gantry mechanisms go in RobotKit core or autonomy: `HomingCycle` in MotionKit,
    `PlanningLimits` in motionkit-robot.
  - Simulation-only pieces (`SwitchReading`'s simulated trips) go where `robotkit-sim` will be.
  - Nothing new may make RobotKit core depend on SimKit or VisionKit.
- **Device protocol (R0).**
  - G8 and G13 change RKD. If R0's renaming is in flight, do G13 after it lands, and use the new
    naming and versioning scheme.
  - G8's grouping fix is small; do it under the current names if R0 hasn't started.
- **Merging.** Restructuring phases arrive through local main (G4). Their moves are mechanical
  commits, so merge them as soon as they land rather than letting the branch drift.


### Phase A — planning (no machinekit assembly changes; can start now)

**G0. Baseline.**
- Run the full suite on this branch's merged base `712019dbc` in this worktree. Record the numbers
  this plan must hold, or must explain when they move:
  - router plate times (screw and belt) and belt deviation;
  - the CoreXY summary;
  - robot-arm pick/place mission time;
  - mobile-base numbers;
  - MotionKit redundancy tests.
- Put them in the Progress section.

**G1. Planning limits from the drives** (G-D3).
- X9a (in this branch's base) already writes `coupledLimits` into the compiled `RobotModel` joint
  limits, keeps missing limits as `Null`, and makes the mission planner and jogging use them. Start
  from that. What is left: per-joint acceleration and jerk instead of one scalar, `PlanCheck` in
  missions, and `SteadyLoads`. Drop anything X9 already did.
- Add `PlanningLimits` (motionkit-robot, next to `PlanCheck`) and replace the scalar-acceleration
  arguments of `HandlingPlanRunner.create`, `SurfacePlanRunner.create` and
  `ToolpathPlanRunner.create` with it.
- `MissionPlayer` passes the model's coupled limits with `SteadyLoads` and attaches `PlanCheck` as
  `CncProgramPlayer` does.
- `CncProgramPlayer.planningModel` uses the same helper.
- Tests:
  - a router-like XYZ model through `HandlingPlanRunner` plans at the coupled speed, not 2 m/s, and
    the runtime accepts every segment;
  - arm mission numbers either hold or are recorded with the reason (the arm got servo + gearbox
    drives in X6).

**G2. Tool freedom through IK and planning** (G-D1, G-D2).
- **RobotKit:**
  - `IkOptions` gains `positionAxes` and `orientation:FrameOrientation`, with a fluent setter.
  - `KinematicGroup.toolTask` and `solveWithBase` pass them through.
  - `tcpJacobian`-based differential paths drop the free rows.
- **MotionKit:**
  - add `OrientationPolicy.Free`;
  - add the mapping helper;
  - `PathRequest` carries each sample's policy;
  - `KinematicsSolver.solvePose` and `solveDifferential` take an optional freedom;
  - `ManipulatorKinematics` builds reduced rows instead of a silent least-squares fit for n < 6;
  - when a requested freedom is impossible, it fails with the residual named;
  - `ProgramCompiler.checkTaskSpace` uses the same mapping.
- Pick/place passes `Axis(tool Z)` plus all position axes. The home orientation stays the
  preference, not a hard row.
- Tests:
  - XYZ+C test model follows a `FreeAboutTool` path whose interpolated spin it could not follow,
    with zero orientation error about the free axis;
  - XYZ model rejects a tilted `Fixed` target with the new diagnostic;
  - 6R arm results for `Fixed`/`Interpolated` are bit-identical to G0 (redundancy suite, robot-arm
    mission);
  - kinematicskit masked-task tests extended to `KinematicGroup`.

**G3. External axes derived** (G-D4).
- `Manipulator` derives external joints from the include that owns the flange. The bridge must
  carry each joint's include path; check what `AssemblySimulationBridge` already keeps.
- The swivel default counts arm joints only.
- The plan runners accept a `KinematicGroup` (a `Manipulator` is one).
- `IkOptions.posture` keeps working with externals.
- Tests:
  - a 6R arm on one prismatic track built by assembly gets no swivel, `ExternalAxesParameterization`,
    and the external damping;
  - `testCoordinatedExternalAxes` passes with the group built by derivation instead of by hand;
  - a plain arm and the mobile base are unchanged.

### Phase B — the gantry assembly (needs X7 T2 committed)

**G4. Keep up with main.**
- X7–X9 are already in the base (main `4b952231f`).
- At step boundaries, merge local `main` when it gains work this plan uses:
  - X10, belt reductions and loops between shafts, from `x7-transmissions`. It matters for belt
    axes in G5–G7.
  - Machine-tending MT1–MT3: ball screws, HGR rails, `CobotArm`, the shared tool interface.
  - The rest of the welder's W4–W6.
- Never merge those branches directly: only what has reached `main`.

**G5. Shared axis builder** (G-D6).
- Extract the duplicated helpers from `CncRouter` and `CoreXyPlotter` into
  `machinekit.assembly.AxisBuilder`, or a base class if that reads better:
  - `place`, `attach`, `hang`, `connect`, `zeroPose`, `orient`, and `slide` with rail-room
    overtravel;
  - `driveScrew`/`mountNut` (screw, nut member, `supportScrew`, `addMotor`);
  - `turnWithBelt`, and a generic two-pulley belt axis replacing `beltX`/`beltY`;
  - rack-and-pinion (`RackAndPinion` exists as a transmission source; it needs a `Rack` member and a
    pinion on the motor).
- The router and CoreXY use the builder. Every router and CoreXY number is unchanged, and the
  MachineKit smoke output is identical apart from ordering.
- Decide what happens to `LinearAxis`: rebuild it on the builder, or keep it as the test fixture
  for `compileAssemblyAxes` until G9. Record the decision.

**G6. `Gantry` assembly** (G-D5, G-D6).
- `machinekit.gantry.Gantry` + `GantrySpec`:
  - frame (posts + beams from `TSlotExtrusion`, or a table-mounted variant);
  - dual-side Y by default;
  - X carriage on the beam, Z slide;
  - `RobotFlange` at the bottom of Z, exposed as connector `toolFlange`;
  - drive per axis (screw / belt / rack-and-pinion);
  - optional head slot (G13).
- `racking` tolerance and the stated fields are marked assumed.
- `GantryChecks` in the MachineKit smoke cover:
  - FK of the flange at the eight travel corners;
  - no OCCT interference at the ends of travel;
  - ratios per drive kind;
  - derived axis speeds and accelerations, printed;
  - BOM;
  - one dual-Y spec with two motors on one joint.

**G7. Gantry picker example** (kind 1).
- `machinekit/examples/gantry-picker/`: a ~1.5 × 1.0 × 0.5 m belt-driven gantry (rack-and-pinion Y
  optional) with a suction tool on the flange. It moves boxes from an infeed table to a pallet
  pattern using the existing pick/place mission and scene objects.
- Scene validation rejects `machining` plus `mission` (G-D10).
- Start-page entry. MachineKit smoke `GantryPickerChecks`.
- `ProjectSourceTests.checkGantryPicker`, mirroring `checkRobotArm`:
  - every box lands within 2 mm / 2° of its slot;
  - planned speeds ≤ coupled limits;
  - `PlanCheck` reports no stall;
  - no collisions;
  - allocation budget;
  - cycle time recorded.

**G8. Dual-drive skew on the device** (G-D7).
- `DeviceLayout` groups actuators whose shafts follow the same leader joint and sets `skew_bound`
  from the spec's racking tolerance in steps.
- The RKD6 grouping uses the leader (decide whether the host sends the leader as `actuator_joint`
  with a composed ratio, or the protocol gains a group field, and bump the protocol if needed).
- Tests:
  - the virtual device with `miss_next_steps` on one router Y motor faults `DUAL_DRIVE_SKEW` within
    the bound;
  - the X axis (single motor) never groups;
  - the Nucleo firmware is compile-checked with `cargo check --offline`.

**G9. One gantry model path.**
- X9a already compiles `compileLinearAxisModel`/`compileXYZGantry` through the axis's
  `MachineAssembly` and `compileAssemblyAxes`. Start from that.
- Retire `MachineKitRobotCompiler.compileXYZGantry` and `compileAssemblyAxes`.
- MotionKit tests that used them build a small `Gantry` (or `LinearAxis` on the builder) and go
  through `AssemblySimulationBridge`, as the app does.
- Numbers that move (the old model had no couplings or drives) are recorded with the reason.
- Keep `robotkit/tools/humanoid` `MixedScene` working.

### Phase C — homing and switches (G-D8)

**G10. Switch parts and simulated readings.**
- `machinekit.motion.LimitSwitch` (roller micro-switch) and `ProximitySwitch` (inductive M8/M12)
  parts with BOM lines and a `Signal` port. Generic catalog entries are marked assumed.
- `MachineAssembly.addSwitch(id, joint, part, trigger, side)`: the trip position is derived from
  where the trigger member meets the switch along the joint.
- Saved as `AssemblySwitch` in the projectkit format (optional, written only when present).
- Bridge to `RobotModel.switches`. A host-side `SwitchReading` with hysteresis and repeatability
  (deterministic seed) is published each tick as a digital external sensor frame (pattern:
  `tool_contact`).
- `Gantry` places home switches at the negative end of each axis, one per Y side, plus limit
  switches inside the overtravel.
- Checks: trip points lie strictly between the soft limit and the end stop.

**G11. Referenced state and the homing cycle.**
- **Runtime:**
  - per-joint "referenced" state;
  - an unreferenced joint accepts only jog/homing segments; plans are rejected with a named error;
  - a limit-switch trip faults like an observed overtravel.
- **Simulation:** a power-up offset per axis via `setJointSlip` (stated in the scene for tests,
  zero by default), so the machine wakes up not knowing where it is.
- **MotionKit `HomingCycle`:** per axis, seek at the drive-derived seek speed, back off, re-approach
  slowly, latch the trip position, set the offset, and mark the axis referenced. Order: Z first,
  then X/Y.
- `MotionSystem.home()` runs it when switches exist. `EncoderMonitor.reset` and `StepperSlip.reset`
  are called at the latch.
- The router and the gantry home before their program or mission.
- Tests:
  - with random power-up offsets, the homed position equals the true zero within the switch
    repeatability;
  - a program before homing is refused;
  - router plate times unchanged apart from the homing time, which is reported separately.

**G12. Dual-drive squaring.**
- Each Y side has its own switch. During homing the two Y motors run on the same command until the
  first side trips. Then that side holds and the other continues (per-motor offsets through the
  motor followers' slip) until it trips, and both latch.
- Test: a 1 mm racking offset at power-up is removed to within repeatability, and the skew fault
  never fires during squaring. The skew bound is relaxed only for the squaring move, and explicitly.

**G13. Switches and homing on the device.**
- RKD6 gains input channels. `State6` carries input bits, and the device latches the step count at
  a switch edge (capture, for repeatability). A `home` command or a homing segment flag; pick one
  and record it.
- The `Board` trait gains input reads. The virtual board simulates switches from its step counts.
- The deployment layout maps switch ids to board inputs (aligned with X8: controller wiring is
  deployment data).
- Bump the protocol and regenerate with
  `python3 -B -m tools.wire generate --config robotkit/wire6.json`.
- Tests:
  - router homes through the virtual device;
  - Nucleo `cargo check --offline`;
  - pin assignment written down for the bench but not verified (no hardware).

### Phase D — rotary heads and the process gantry

**G14. Rotary heads.**
- `machinekit.gantry.RotaryHead` assemblies:
  - `CHead` (spin about Z);
  - `CaHead` (spin plus tilt, fork or knuckle layout).
- Each is driven by a servo + gearbox (X6 drives) or a stepper + belt reduction, ending in a
  `RobotFlange`. `GantrySpec.head` selects one.
- Checks: FK, joint limits from cable wrap and hard stops, derived speeds, and the tool flange
  offset.
- Picker variant with a C head turning boxes to their pallet yaw. The pick step states yaw as
  needed (G-D1).

**G15. Gantry welder example** (kind 2).
- The welding stack is on main: torch tools, `weld` steps and `WeldingPlanRunner`. W4–W6 (whole
  weldment, weave and multipass, real I/O) are being finished by another session. Use what is on
  main, and pick up more via G4 when it lands.
- `machinekit/examples/gantry-welder/`: a `CaHead` gantry with the welding torch on the flange,
  welding the robot-welder weldment (plate T-joint + tube frame) on a table.
- `WeldingPlanRunner`:
  - derive the wrist from the rotary joints nearest the tool instead of "last three joints";
  - pass the weld's freedom (torch axis, spin free).
- `ProjectSourceTests.checkGantryWelder`:
  - every seam welded;
  - leg size within the robot-welder tolerance;
  - torch work and travel angles within the process window;
  - no collisions (ArmClearance on the gantry group);
  - cycle time recorded.

### Phase E — arm on a gantry or track (kind 3)

**G16. Linear track.**
- `machinekit.gantry.LinearTrack`: a floor track with rack-and-pinion drive, a carriage plate with a
  `RobotFlange`-style arm mount, and cable chain room. It is built on the axis builder.
- Arm include on the carriage. External axes are derived (G3).
- Checks: FK, ends of travel clear, derived track speed.

**G17. Track arm example.**
- `machinekit/examples/track-arm/`: `RobotArm` (or `CobotArm` if MT3 has landed) on a ~3 m track
  that picks from one end of a long table and places at the other (positioning mode: the track
  moves, the arm works).
- Coordinated mode: a straight `FollowPath` longer than twice the arm's reach (a weld if Phase D
  has landed, otherwise a dispensing path with `SurfacePlanRunner`).
- `ProjectSourceTests.checkTrackArm`:
  - path error within tolerance;
  - the track carries most of the travel;
  - arm joints stay within a stated posture margin;
  - no swivel or IK-branch flips.

**G18. Inverted arm on an overhead gantry.**
- Variant of G17: the arm hangs from a `Gantry` Z carriage (or directly from the X carriage), with
  three external axes.
- Same checks, plus reach over a full table area. Do this last; skip it if time runs out.

## Order

**Updated 2026-10-03:**
- The branch is based on main with X7–X9 and the welder merged, so every phase can go ahead now.
- G4 is just keeping up with main.
- Saved data follows X9's policy: one schema version per format, no migration or compatibility code.

```
G0 → G1 → G2 → G3 ─────────────────────────────┐
                 G0 → G5 → G6 → G7 → G8 → G9
                                  G6 → G14 → G15
            (main gains X10 / MT1–MT3 / W4–W6) → G4 at the next step boundary
                                    G6 → G10 → G11 → G12 → G13
                              G3 + G6 → G16 → G17 → G18
```

- Phase D's gantry welder uses the welding stack already on main. Whole-weldment features come with
  W4–W6 through G4. If they haven't landed by the end, weld the seams main supports and report
  the rest.

## Later

- 4/5-axis CNC: A/B/C words, tool-axis IR, 5-axis CAM.
- CoreXY on the device (one motor per joint today).
- Gantry racking dynamics: a flexible beam, not a rigid right block.
- Several robots per cell (MT6).
- Encoder index homing on hardware.

## Progress

| Step | State | Commits |
|------|-------|---------|
| G0 | done; full gate passed | `a8349cdea`, `29b9da11b` |
| G1 | done | `e0b50b5ec` |
| G2 | Complete; full gate passed | `c73877872`, `566ad9ec4` |
| G3 | Complete; full gate passed | `d9d6de8f7` |
| G4 | R0–R6/W4/W5 complete; full gates passed | `05271b08d`, `bd2ccec83`, `0ec408c0d`; source boundaries `bce647683`, `af673c4f4`, `757127cf0` |
| G5 | complete; full gate passed | Shared axis builder, router/CoreXY extraction and physical rack regression |
| G6 | planned | — |
| G7 | planned | — |
| G8 | planned | — |
| G9 | planned | — |
| G10 | planned | — |
| G11 | planned | — |
| G12 | planned | — |
| G13 | planned | — |
| G14 | planned | — |
| G15 | planned | — |
| G16 | planned | — |
| G17 | planned | — |
| G18 | planned | — |

### G0 — merged-base baseline (2026-10-04)

**Gate:** `/home/joao/dev/materia-cache/claude-scratch/gantries-suite.sh g0-final`
passed all twelve kit suites, CadKit, MachineKit, the app build and the full
project-source suite. The summary is `gantries-suite-g0-final.txt`; logs are
`g-s-*.log` beside it. The initial branch HEAD was `35a0fe879`, based on
`712019dbc`; `a8349cdea` adds test-fixture reuse only, with no machine or planner
behavior change. OCCT was prebuilt 8.0.1 from the specified cache directory.

The G0 target was corrected from the old survey commit `aaa2b911a` to the
merged base `712019dbc`, as required by the handoff. These are the numbers
subsequent steps must preserve or explain when they move.

| Suite | Baseline result |
|-------|-----------------|
| KinematicsKit | 210 assertions |
| RobotKit | 4935 world assertions, including the excavator scenarios |
| MotionKit | 9865 assertions, including redundancy and coordinated external axes |
| ToolpathKit | 2921 scenario assertions |
| CadBridge | 157 assertions |
| CncKit | 317 assertions |
| StockKit | 100673 assertions, 17494 reference rays |
| CamKit | 12308 assertions |
| ProcessKit | 52 welder assertions, 23 core assertions |
| ProjectKit | 131 assertions |
| HumanKit, AnimKit, CadKit, MachineKit | exit 0 |
| App build and full project-source suite | exit 0 |

**Router motor plate:** same controller, feeds and stock.

| Figure | Screw | Belt |
|--------|-------|------|
| Machining time | 220.2 s | 201.6 s |
| Planned Y/X/Z speed | 22.3 / 25 / 25 mm/s | 500 / 500 / 25 mm/s |
| Planned Y/X/Z acceleration | 5.21 / 5.37 / 5.65 m/s² | 12.35 / 14.17 / 5.65 m/s² |
| Free Y/X/Z speed | 22.3 / 34.6 / 43.7 mm/s | 873.1 / 873.1 / 43.7 mm/s |
| Free Y/X/Z acceleration | 5.4 / 5.58 / 6.12 m/s² | 12.79 / 15.08 / 6.12 m/s² |
| Checked / flagged plans | 126 / 0 | 126 / 57 |
| Stepper stalls / accuracy findings | 0 / 0 | 0 / 64 |
| Worst torque / predicted deviation | 56.7% / 0.05 mm | 56.5% / 1.76 mm |
| Ticks of rapid-label cutting | 0 | 6 |

Both removed 9914.9 mm³ from 9996.5 mm³ planned material, with 63.5 mm³
leftover and 0.6 mm³ gouge. The old 1.89/1.90 mm belt-deviation figures in
transmission milestone notes are not this merged branch's baseline. Screw
router assembly: 33 definitions, 61 occurrences, 32 BOM lines and 36.9 kg.

**CoreXY:** 17 definitions, 38 occurrences, 17 BOM lines, 4.6 kg, pulley radius
6.366 mm; motors 204.1 rad/s, axes 649.6 mm/s, free accelerations 71.5 / 34 m/s²
(the app reports X as 71.549 m/s²). MotionKit steady-load accelerations are
57.409 / 27.415 m/s²; the square takes 79 ticks, with motors at most
96.2 / 131.9 rad/s. Worst predicted belt deviation at planned limits: 0.133 mm.

**Arm:** 32 definitions, 38 occurrences, 20.3 kg above the base flange and
150.1 kg total. Suction contact (35, -524, 444) mm points down; ready-pose tool
(35, -524, 563) mm. MuJoCo mission cumulative completions: pick 5.8 s, place
12.0 s, pick 17.2 s, place 23.5 s.

**Mobile:** both deterministic and MuJoCo base checks travel 400 mm in 1 s,
turn 0.5 rad and report 400 mm odometry. MuJoCo mission cumulative completions:
goTo 9.6 s, pick 15.1 s, goTo 25.2 s, place 30.6 s, pick 36.1 s, goTo 47.8 s,
place 53.2 s, goTo 61.0 s. Obstacle round: cruise 0.4 m/s, slowest 0 m/s,
closest approach 416 mm, one replan, completed at 65 s.

**Other baselines:** Cartesian blend times: exact stop 2.523507353 s,
0.5 mm 2.543744738 s, 2 mm 2.456784326 s. Robot welder: 36 definitions,
42 occurrences, 41 BOM lines, 315.2 kg; normal seam run 20.2 s, restart run
21.6 s on both backends. CNC controls held at line 6, restarted at line 30,
and restarted at line 243 with tool 2.

**Baseline preparation decision:** session hold/replacement/jog/path sweeps
repeatedly compiled identical CAD gantries. Test support now lazily compiles
one unchanging blueprint per test instance, while each trial still creates
fresh runtime, simulation, endpoint and motion-system state. Both original
and reused-fixture full MotionKit runs passed 9865 assertions with identical
CoreXY and Cartesian blend figures. No sweep points or assertions were
removed; commit `a8349cdea` keeps this preparation separate from features.
An interrupted initial MotionKit run and a shell-driver parse error were
superseded by the complete clean gate. Original logs are preserved in
`gantries-g0-original-logs/`; temporary test-name logging was removed. No
haxeon issue or pre-existing test failure was found.

### G1 — drive-derived planning limits (2026-10-04)

**Gate:** `gantries-suite.sh g1-complete` passes all twelve kit suites, CadKit,
MachineKit, the app build and the full project-source suite. Focused PlanCheck
tests pass 3261 assertions; full MotionKit passes 13059, an increase of 3194
regression assertions over G0. RobotKit remains at 4935 assertions.

`PlanningLimits` snapshots coupled speed and acceleration in group order with
shared steady-load assumptions, per-joint jerk, validation limits and a physical
`PlanCheck`. Handling, surface, toolpath and welding runners use it; mission
factories and the CNC planning model share the helper. Missing speed is an error
naming the joint. Missing acceleration uses a reported assumed default without
rewriting the physical model; unstated jerk is likewise reported as assumed.
The compiler retains these assumptions when forked for a worker.

The three-screw XYZ handling regression plans at 25/12.5/20 mm/s with per-joint
jerk caps of 2/4/6, accepts every segment in the native runtime, observes no
speed beyond its drive cap and reports no stepper stalls. Its model materializes
coupled limits before runtime compilation, matching the assembly bridge.

**Numbers:** all G0 machine, CNC, CoreXY, arm, mobile and welder results hold.
In particular, arm mission completions remain 5.8/12.0/17.2/23.5 s; welder
normal/restart runs remain 20.2/21.6 s on both backends; CNC screw/belt machining
remains 220.2/201.6 s and predicted deviation 0.05/1.76 mm. No machine or mission
baseline number changed intentionally.

**Fixture decisions and interrupted gates:** the synthetic wall-finishing arm
had no velocity, so its former implicit 2 rad/s runner assumption is now explicit
in the fixture. The synthetic excavator likewise states its former 10 rad/s
assumption. These are test assumptions, not new physical drive ratings. The new
screw fixture also supplies unbounded shaft travel rather than the default
zero-width bounds. The first gate's app and the second gate's MachineKit run
ended on SIGTERM (143), without a source exception. Their logs are preserved in
`gantries-g1-first-gate-logs/` and `gantries-g1-second-gate-logs/`; the journal did
not identify the sender. The complete gate ran with signal tracing, passed in
full and received no SIGTERM. No haxeon issue was found.

The pre-R0 MotionKit native libraries and G0 reference binary are preserved in
scratch for G2 comparisons. Local main now contains restructuring R0; G4 takes
that committed work at the next boundary.

### G4 — R0/R1 source integration complete (2026-10-04)

Take the committed R0/R1 changes from local main's merge commits `7db70f2cf`
and `bce647683`, using their first-parent source diffs. R0 extracts planner-free
trajectory core, separates protocol naming/versioning and injects runtime
endpoints; R1 publishes bounded endpoint capabilities and execution-plan
contracts. The surface-patch reaching-IK fix is included. G0 fixture reuse and
G1 shared limits are preserved.

**Integration decision:** a whole-main merge also brought unrelated UI changes
that require newer Skribidi APIs, and four dependency pin updates. Retaining the
pins made the app build fail on missing Skribidi functions. To honor the
handoff's explicit prohibition on pin changes and preserve future merge behavior,
G4 integrates the R0/R1 source commits from main without taking unrelated UI
changes or adding main ancestry. The existing UI and submodule revisions remain
unchanged; no other sessions' uncommitted files or feature branches were used.
This is a deliberate adjustment to G4's whole-main merge workflow.

The app runtime target list adds `trajectory_core` while retaining the existing
CadKit target required by the pinned build layout. NativeTransport retains its
explicit byte-count arguments required by the pinned NativeKit FFI.

A clean RobotKit rebuild exposed a pinned-haxeon enum membership issue:
`Array<JointTargetMode>.contains(Servo)` and `Type.enumEq` returned false for
`Position,Velocity,Effort,Servo`, before mutation of a defensive copy. Native
capability values were correct. `RobotCapabilities.accepts` uses an explicit
loop with statically typed enum equality; the original capability regression
passes, with richer failure values. No haxeon source was changed. Focused
RobotKit passes 4946 assertions, 11 more than G0/G1 from R0/R1's new endpoint
and contract tests.

All fourteen suites passed the whole-main integration attempt, but its app
build failed; its stale app run was stopped and is not validation evidence.
The source-only app build passed, followed by the clean `g4-source-complete`
full gate: all fourteen suites, app build and project-source suite exited 0.
The traced gate received no SIGTERM. The configured CadKit uses prebuilt
OCCT 8.0.1. G0 mechanical, planning, cutting and mission numbers remain
unchanged; RobotKit alone adds the eleven R0/R1 assertions noted above.
MotionKit remains at 13059 assertions, including G1 coverage. Submodule pins
and the two unstaged vendor symlinks are unchanged.


### G2 — tool freedom through IK and planning complete (2026-10-04)

`IkOptions` carries the position mask and `FrameOrientation` through ordinary
and moving-base tool tasks. Its immutable orientation preference is a separate
soft task, optimized with the prioritized solver after the hard rows. A frame
task can be marked as a preference without changing its residuals or Jacobian.

`ToolFreedom` is the shared mapping in MotionKit's RobotKit adapter layer;
MotionKit core still has no native kinematics dependency. Free spin holds local
tool Z, `Free` drops orientation, and cones align tool Z with their path-frame
axis and use the aperture as the orientation tolerance. Pose IK, candidate
sampling, differential row projection, redundant lattice/refinement and the
path check use this mapping. `PathRequest` carries aligned sample policies;
`MoveL`/`MoveC` have an optional policy, defaulting to the existing interpolated
full orientation. The existing planar full-orientation blender remains exact;
reduced-task MoveL blends currently follow the existing exact-stop fallback.

Handling moves request tool Z with spin free, preferring home orientation. If
that orientation is feasible, its zero-error solution preserves the original
full solve. Otherwise the prioritized solver improves the soft orientation
within joint bounds without compromising hard rows. Worker solvers retain the
immutable preference. Reduced differential IK checks its hard residuals;
underactuated full tasks no longer silently accept an impossible twist. Failed
numeric IK reports separate position and orientation residuals to the compiler.

Focused tool-freedom coverage passes 281 assertions: a limited-C XYZ machine
follows an otherwise unreachable spinning path, a soft orientation reaches C's
nearest limit, fixed XYZ tilt and angular velocity fail with named residuals,
cones use a path-frame axis, and external-axis lattice/refinement propagates
per-sample freedom. Full 6R and swivel-preserving solves are compared bit for
bit with the pre-G2 algorithms. Redundant searches also retain the soft
orientation preference. A `Fixed` MoveL/MoveC rejects inconsistent endpoint
orientations instead of silently discarding them. The saved G0 executable and current code both
pass the same 193-assertion redundancy suite. RobotKit passes 4955 assertions, including moving-base and copied-preference
checks. The first gate was deliberately stopped after RobotKit to extend soft
orientation preference through redundant lattice/refinement; the new regression
then passed. The `g2-final` core gate passed all fourteen suites, app build and the full
project-source suite (all exit 0): MotionKit 13340, RobotKit 4955. All recorded
G0 physical and mission numbers hold, including the 6R mission at
5.8/12/17.2/23.5 seconds. Review found that path-junction angular comparison
still includes free spin; a regression and projection fix remain before the
final G2 commit. This checkpoint is preparation, not step completion.

Pinned-haxeon workaround: an omitted optional argument on an interface call
produced E1008 (`KinematicsSolver.sampleCandidates` expected four arguments,
got three). Calls through that contract pass an explicit null for the default
full policy. `ModuleCanonicalizer.canonicalInterface` reconstructs method arguments with
name, type and span but drops their optional/default metadata.
`CallResolver.typeDeclaredCallArguments` consequently treats all interface
arguments as required; concrete calls retain their flags. A fresh one-source
project, with the compiler server disabled, reproduces E1008 for an omitted
optional integer argument. No compiler
source or dependency pin was changed.


**G2 junction follow-up:** the core gate is committed as `c73877872`.
A two-line path with continuous translation but differing free-spin rates
exposed an unnecessary exact stop: corner splitting compared the raw angular
rates and produced two plans. Both splitting and in-plan junction validation
now use `ToolFreedom.requiredAngular`, projecting onto the hard task. Full
policies return the original angular array, preserving their comparisons.
The free-spin regression now produces one continuous plan; the full-orientation
control still produces two plans for its genuine angular-rate discontinuity.
Focused coverage passes 283 assertions. The `g2-junction-complete` full gate passed all fourteen suites, app build
and full project-source tests (all exit 0). MotionKit passes 13342 assertions
(+283 G2 coverage), RobotKit 4955 (+9 masked-task/preference coverage).
All recorded G0 mechanical, drive, cutting and mission numbers hold. This
follow-up completes G2. The next G4 boundary will take committed main through
R6 and the whole-weldment W4 work before G3 adopts the package layout.

### G4 — R2–R6 and W4 source integration complete (2026-10-04)

The source boundary is committed local main `af673c4f4`. A three-way squash
integration preserves the earlier decision to import sources without main ancestry.
Unrelated UI changes, CAD build tooling and dependency pins remain at the gantries
versions. Conflicting earlier R0/R1 copies follow main's package layout; G1's
drive-derived planning limits and G2's tool freedom remain in the moved sources.
W4's wrist acceleration uses the minimum planning acceleration of its wrist joints,
instead of reintroducing the old scalar acceleration. Main's tighter welding IK
position tolerance (50 micrometres) is retained for the W4 path-clearance gate.
The existing typed capability comparison and three-argument NativeKit send
workarounds are retained. New interface calls pass explicit null for optional
freedom, matching the pinned compiler workaround documented under G2.
Integration checks found old surface-runner callers and one lost synthetic fixture
velocity cap in conflicting test files; these retain G1's explicit limits. New W4
solver fixtures implement G2's freedom arguments. The app retains the previously
validated aggregate CadKit configuration: the main configuration assumed a CadKit
package migration that this source integration excludes. Configuration confirms
prebuilt OCCT 8.0.1. A missing `haxe.Int64` import in the new weld compilation path
is explicit now. Focused RobotKit passes 4,981 assertions, MotionKit freedom passes
283, ProcessKit passes its 52 welder, 44 planning, 11 schedule and 23 general
assertions, and the app compiles. The package-boundary checker passes. Two early
full runs were deliberately stopped after diagnosed integration failures; they
were not unexplained terminations. The final `g4-r6-final` gate passed all 14 library suites, app build and full
project-source validation; the runner exited zero. RobotKit has 4,981 assertions
(+26 from the R6 integration), MotionKit 13,342 (unchanged), and ProcessKit adds
44 weld-planning and 11 rate-schedule assertions to its existing 52/23 checks.
This step commit imports the source boundary `af673c4f4`; no submodule pin changed.

Arm, router and CoreXY assembly counts/masses/poses are exact matches to the G2
gate. Arm mission, both mobile missions, obstacle navigation, CNC plan checks and
controls match exactly. Screw/belt machining remains 220.2/201.6 seconds, with
unchanged volumes, gouge, rapid-cut ticks, speeds, accelerations and accuracy.
CPU timings and allocation/GC counts vary with execution.

The intentional W4 numerical changes are:
- Welder 36/42/41 definitions/occurrences/BOM lines becomes 38/44/43: neck and
  nozzle are separate collision bodies. Mass stays 315.2 kg.
- Ready wire tip moves from (132, -620, 254) to (132, -584, 330) mm: the cell's
  ready pose lifts the torch clear of the table; the wire direction is unchanged.
- Single-seam normal time is 20.6 s on both backends (was 20.2); restart is
  22.0 s MuJoCo and 22.1 s test backend (was 21.6): W4's revised geometry and
  clearance-safe entry/retreat change the travel. MuJoCo's rounded tip error
  improves from 0.1 to 0 mm; seam length, arc ticks, current, leg measurements,
  overlap, restart count and stray metal remain unchanged.
- The newly tested whole weldment has 10 seams, 680 mm in four runs, with
  386 mm air travel and 176 degrees turning versus 846/290 in discovery order.
  Both backends complete in 106.5 s, runs at 31.3/65.7/85.9/106.5 s, with
  no clearance violation and tip error at most 0.1 mm. This expands the former
  single-seam mission rather than changing its deposition requirements.

Logs are retained in `gantries-g4-r6-final-gate-logs` in the handoff scratch area.
The next implementation step is G3, against the new package/profile layout.

### G3 — assembly ownership and external derivation complete

RobotFlange declares its face as a physical capability. Mechanical definitions
preserve that connector, and flattened occurrences and joints carry their owning
include paths. The bridge emits a physical flange frame with that ownership,
even when fixed parts share a simulated link. KinematicGroup derives externals
when no explicit list is supplied, so Manipulator gets the same derivation.
The nearest marked flange on the tool chain owns the arm; joints in other
includes are external, while child includes of the arm remain arm joints.
Ambiguous ownership or a missing joint scope fails explicitly. Non-assembly
models with no marked flange retain their existing behavior. The RobotModel
codec preserves this metadata, including an empty root include.
Handling, surface, toolpath and welding runners now accept KinematicGroup.
The coordinated workcell fixture derives its rail instead of listing it, with
checks for no swivel, external-axis parameterization and posture preference.
The existing 283 tool-freedom assertions and 196 redundancy assertions pass.
CadBridge passes 172 assertions (+15): a real six-joint arm with base and tool
RobotFlanges sits on a prismatic track below two include levels, while other
flanges sit on the track and root. The nearest tool flange selects the arm, the
track is external, no swivel is fabricated, the parameterization is external,
and posture retains seven coordinates. Frozen descriptions and RobotModel
round trips preserve ownership, including the empty root scope. A flange
referencing an absent connector is rejected.
A one-iteration tracking comparison against the same model with an explicit
empty external list confirms that the derived track receives external damping:
it moves toward the target, but less than the undamped control.

The nested fixture exposed a pre-existing shared-definition compaction bug:
a root occurrence could share a definition first named inside an include,
leaving a child-path definition ID in a local subdefinition. Compaction now
reassigns such retained definitions to deterministic local IDs and updates
their occurrences. It preserves sharing and definition counts. The deeper
assembly regression fails without this correction and passes with it.
Pinned Haxeon requires an explicit class value after the nullable flange
selection; the test uses a typed cast after checking the selection.
The first full gate passed the twelve kit suites, then CadKit's exact nested
versus hand-built flat comparison failed because the expected flat fixture
lacked the new ownership fields. The expected root/member/joint include paths
are now explicit; the full equality assertion remains intact. The focused
CadKit suite passes. That gate was deliberately terminated before completing
MachineKit and application checks; its logs are archived separately.
The `g3-final` full gate passes all fourteen library suites, the application
build (1,745 sources), and the project-source runtime suite, with terminal exit
0. MotionKit passes 13,345 assertions (+3); CadBridge passes 172 (+15).
The 64 MachineKit physical/reachability lines match G4 exactly. All 34 application
physical result lines also match G4 after excluding CPU timing: arm mission
5.8/12/17.2/23.5 s; both whole weldments 106.5 s with no clearance violation;
single welds 20.6 s and restart 22.0/22.1 s; CNC screw/belt 220.2/201.6 s,
unchanged cuts, limits and plan checks; mobile mission 61 s and obstacle 65 s
with 416 mm clearance. No physical baseline changed. Logs are archived in
`gantries-g3-final-gate-logs`; the deliberately stopped first gate has its own
archive. No dependency pins or publication state changed.

### G4 — W5 source integration complete

At the G3 boundary local main is `757127cf0`. Its committed changes since
`af673c4f4` add seam-progress weaving, CAD-derived multipass recipes, interpass
cooling, deposited-bead grounding and clearance, scene schema 16, saved woven
and multipass examples, and smaller restart humps. These support the G15
process gantry. The existing source-only integration decision remains: apply
the committed main delta with a three-way merge, without changing dependency
pins or importing another branch or worktree's uncommitted content.
The sole conflict is weld-runner construction in MissionPlayer. W5 rebuilds
clearance whenever it creates a runner; the resolution retains that behavior
and G1's drive-derived PlanningLimits. G3's KinematicGroup runner signatures
and G2's tool-freedom changes remain present.
The focused ProcessKit suite passes: 55 welder, 44 planning, 16 rate-schedule,
106 weave, 13 pass-sequencing, 12 bead-work, 19 pass-path, and 23 general
assertions. Standalone MotionKit weave passes 738 assertions and MachineKit
welding recipes pass 33. The full-gate helper now includes both standalone
suites in addition to the original fourteen, followed by the application build
and project-source runtime checks. The `g4-w5-final` gate passed all sixteen
suites, the application build and project-source checks, with terminal exit 0.
Logs are archived in `gantries-g4-w5-final-gate-logs` in the handoff scratch directory.
W5 intentionally reduces restart backoff from 10 to 2 mm, uses minimum stable
wire feed for the recovery overlap, and omits its pooling dwell. The gate verifies
21.1 s recovery on both backends, 3 mm overlap, and a 5.4 mm peak instead of
9.4 mm; the ordinary seam remains 20.6 s. Recovery has 1623 arc ticks instead
of 1706 and mean leg 5.0 mm instead of 5.2 mm. Both whole-weldment runs remain
106.5 s. The 64 MachineKit physical/reachability lines match G3 exactly; router,
CoreXY, arm and mobile application physical results retain their baselines.

New quality checks pass on both backends: woven 7 mm target gives
6.9982130667 mm leg in 27.17 s with one strike; three-pass 10 mm target gives
9.9993558074 mm (MuJoCo) and 9.9991930154 mm (deterministic) in 65.55 s with
three strikes. ProjectKit passes 138 assertions, seven more for schema 16.
CPU planning and simulation timings vary and are not physical invariants.

### G5 — shared axis builder complete

G4 W5 is committed as `0ec408c0d`. The shared `AxisBuilder` base now owns
placement, connector attachment, slide/overtravel, screw/nut support, belt
couplings, two-pulley axes and physical rack-and-pinion construction. Router
and CoreXY inherit it; platform-specific mounting plates remain in their
examples. Construction order, identifiers and geometry formulas are preserved.
A new physical rack fixture checks transmission ratio, motor drive, initial
position, endpoint FK and rail overtravel. All ten regression checks pass.

Decision: retain `LinearAxis` as the `compileAssemblyAxes` fixture until G9,
when that compilation route is retired. G5 does not change its behavior.

G5 validation notes: the initial rack regression accessed optional coupling
and actuator arrays directly, causing pinned haxeon E1005. Explicit local
null checks fix the typing. Its first runtime attempt used the mechanical
description, which deliberately excludes compiled actuators; the fixture now
uses `addTo(AssemblyModel)` and the resulting definition, as the existing
motor-drive tests do. The second corrected build compiles all 1011 sources
and passes the smoke suite. No compiler or dependency changes were needed.

The corrected focused MachineKit run passes with terminal exit 0. Its 64
selected physical/reachability lines match G4 W5 exactly, and the CoreXY drive
summary remains 204.1 rad/s motors, 649.6 mm/s axes, 71.5/34 m/s² acceleration.
The required full G5 gate completed with terminal exit 0: all sixteen suites,
application build and project-source runtime pass. Physical results remain at
the G4 W5 baseline: screw/belt machining 220.2/201.6 s, unchanged drive limits
and deviations, both weldment runs 106.5 s, normal/recovery welds 20.6/21.1 s,
7 mm woven fillets 27.17 s and 10 mm three-pass fillets 65.55 s on both backends.
CPU timings vary. Logs are archived in `gantries-g5-final-gate-logs` under the
handoff scratch directory.

Local main advanced to `b25366059` during validation. The next G4 boundary will
integrate its committed X9e source repairs before G6; dependency pins remain
unchanged. Its expected folded-Z backlash change is 0.0505 to 0.051 mm because
both pulley contacts contribute clearance.
