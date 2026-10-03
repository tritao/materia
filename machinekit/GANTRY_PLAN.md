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

Branch `gantries`, worktree `/home/joao/dev/materia-worktrees/gantries`. It was created from
`x7-transmissions` at `aaa2b911a` (X7 T1). That commit includes transmissions X1–X6 and X5 multi-leader
couplings (merged with local main 1048bf768).

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

## Steps

### Phase A — planning (no machinekit assembly changes; can start now)

**G0. Baseline.**
- Run the full suite on `aaa2b911a` in this worktree and record the numbers this plan must hold,
  or must explain when they move:
  - router plate times (screw and belt) and belt deviation;
  - the CoreXY summary;
  - robot-arm pick/place mission time;
  - mobile-base numbers;
  - MotionKit redundancy tests.
- Put them in the Progress section.

**G1. Planning limits from the drives** (G-D3).
- **Do this after G4.** X9a (on `x7-transmissions`) already writes `coupledLimits` into the compiled
  `RobotModel` joint limits, keeps missing limits as `Null`, and makes the mission planner and jogging
  use them. Rebase this step on that: what is left is per-joint acceleration and jerk instead of one
  scalar, `PlanCheck` in missions, and `SteadyLoads`. Drop anything X9 already did.
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

**G4. Merge X7, X8 and X9.**
- X7 and X8 are committed (2026-10-03). X9 (review fixes, no legacy compatibility, belt and load
  model) is in progress. Its commits are unverified until the X9d full-suite gate. Merge
  `x7-transmissions` into `gantries` once its plan's X9 status line says the gate passed, not
  before.
- After the merge, rerun the full suite and re-record the G0 baselines. X7–X9 moved numbers on
  purpose; take the new values from their plan. From here on, write against `Transmission`/`addTransmission`.
- Merge again at later step boundaries whenever X7 or X8 has new committed steps, so the gantry
  never diverges from the drive API. Resolve conflicts in favour of X7's API.
- If X8 has landed, use `addMotor(…, driver, …)`.
- Never merge uncommitted work from another worktree.

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

### Phase D — rotary heads and the process gantry (needs `mobile-welder` merged to local main)

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
- Prerequisite: merge local main once `mobile-welder` is merged there, or `mobile-welder` itself if
  its owner says it is finished. Never pick up its uncommitted W4.
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
- Start with G0 → G2 → G3 on the branch as it is. X9 does not touch IK or external axes.
- Then wait for the X9 gate, do G4, then G1, then continue with Phase B.
- While waiting, G10's part and format design may go ahead on paper, but not in code. It touches
  `MachineAssembly`, which X9 is still changing.
- Saved data follows X9's policy: one schema version per format, no migration or compatibility code.

```
G0 → G2 → G3 ──────────────────────────────────┐
         (X9 gate passed) → G4 → G1 → G5 → G6 → G7 → G8 → G9
                                    G6 → G10 → G11 → G12 → G13
            (mobile-welder on main) → G14 → G15
                              G3 + G6 → G16 → G17 → G18
```

- Phase A can start at once. If X7 T2 still isn't committed when G3 is done, go on to anything
  that doesn't need the assembly API (G13's protocol design, G2 follow-ups), and check again.
- Phase D waits for `mobile-welder`. If it hasn't landed when B, C and E are done, stop and report
  instead of merging unfinished work.

## Later

- 4/5-axis CNC: A/B/C words, tool-axis IR, 5-axis CAM.
- CoreXY on the device (one motor per joint today).
- Gantry racking dynamics: a flexible beam, not a rigid right block.
- Several robots per cell (MT6).
- Encoder index homing on hardware.

## Progress

| Step | State | Commits |
|------|-------|---------|
| G0 | planned | — |
| G1 | planned | — |
| G2 | planned | — |
| G3 | planned | — |
| G4 | planned | — |
| G5 | planned | — |
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

Baseline numbers (G0): to be filled in.
