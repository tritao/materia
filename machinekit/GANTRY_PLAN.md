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
| G4 | R0–R6/W4/W5 gates passed; X9e implemented, final gate stopped at user request | `05271b08d`, `bd2ccec83`, `0ec408c0d`; `c75e5d1a3`; source boundaries `bce647683`, `af673c4f4`, `757127cf0`, `b25366059` |
| G5 | complete; full gate passed | `13e993340`; shared axis builder and physical rack regression |
| G6 | implemented; build/runtime validation deferred at user request | `d11e4dacb` |
| G7 | implemented; build/runtime validation deferred at user request | `11e9bc82b` |
| G8 | implementation added; runtime and firmware verification deferred | `823de3191`, `6ac44c34e` |
| G9 | implementation added; migration verification deferred | `cd7853a82` |
| G10 | implementation added; mechanical/simulation verification deferred | see progress notes |
| G11 | in progress; reference-state foundation added | see progress notes |
| G12 | in progress; side holds and counter calibration wired; validation pending | see progress notes |
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

### G4 — X9e source integration implemented; full validation deferred

Integrate committed local main `b25366059` (X9e `e5eb445f0`) from the previous
source boundary `757127cf0`. This retains the established source-only decision,
excludes the transmission owner's plan and changes no dependency pins. The
repairs retain requested stopping acceleration through plans, runtime and
recordings, validate elastic networks structurally, project shared-axis
compliance correctly and diagnose editable belt components. Both pulley
contacts now contribute tooth clearance; folded-Z backlash becomes 0.051 mm.

The SessionTests conflict retains G0's immutable blueprint with fresh runtime
per trial and takes X9e's assertions against requested acceleration, removing
the obsolete physical-ceiling helper. Full validation remains required.

The first focused MotionKit run compiled successfully but failed the weak-motor
program with an array bounds error. X9e's new `controlAcceleration` defaults
and count validation used segment count instead of joint count. Correcting
both to the start-state joint count preserves unstated physical caps and
explicit requested caps. Regression checks use two joints and one segment,
verify default size and copy preservation, and reject a one-element cap array.
No compiler or dependency changes are needed.

The corrected focused MotionKit suite passes with terminal exit 0 and 13,375
assertions. Requested stopping caps also have recording round-trip coverage.
The full integration gate is running against this source before G4 completion.

The first full X9e gate passed fifteen suites and the application build, but
MachineKit received SIGTERM (exit 143) after printing the welder chain reach
poses. It reported no assertion failure. The signal source is unproven; the
inspected journal gives no explanation. Logs are preserved under
`gantries-g4-x9e-first-machinekit-terminated`. Application runtime checks are
still running. This attempt cannot establish a passing full gate; validation
must be retried, with signal metadata captured if needed.

The first gate ended with overall exit 1: application runtime also received
SIGTERM (143). Neither terminated process reported an assertion failure. The
complete attempt is archived under `gantries-g4-x9e-first-gate-logs`. The full
retry captures SIGTERM metadata for both processes using strace signal tracing
with syscall tracing disabled. Test source and commands remain unchanged.

The signal-traced retry ended with overall exit 1. Fifteen suites, the
application build and the complete project-source suite passed (exit 0).
Physical router, CoreXY, mobile and welder results match the recorded baseline;
folded-Z backlash has the intended 0.051 mm value. MachineKit alone received
SIGTERM (143). Its trace proves an external current-user sender
(`SI_USER`, uid 1000, pid 2642068), whose executable was not captured before it
exited. This is not evidence of a test assertion failure or kernel OOM. The
complete retry is archived under `gantries-g4-x9e-retry-gate-logs`. A passive
process metadata monitor is active for the next full gate; it records process
identity and parentage without command arguments. G4 completion still requires
a passing full gate.

The third gate passed all sixteen kit suites (including MachineKit) and the
application build. Both application whole-weldment backends passed at 106.5 s
with no clearance violation; their physical summaries match the G5 archive
exactly. Normal welding and recovery also match on both backends. The user then
instructed: "lets stop testing, lets just continue on the gantry plan". The
remaining application checks were stopped deliberately, with terminal exit
143. Logs are archived under `gantries-g4-x9e-third-user-stopped-gate-logs`.
This deliberate stop is separate from the earlier unexplained SIGTERMs.
Implementation proceeds without further test runs under this instruction;
remaining validation is deferred and must not be described as passed.

### G6 — physical gantry assembly implementation

Add `Gantry` and `GantrySpec` on the shared axis builder. The mechanical tree
contains extruded posts and beams, a table-mounted option, two Y guides with one
translational leader, X and downward Z slides, and a physical ISO-style flange
exposed as `toolFlange`. Screw, belt and rack choices construct their real
parts, mounts, drivers and power wiring; dual Y uses two motors on one joint.
Authored spec fields and the 0.5 mm racking tolerance are explicitly assumed.
The rotary head selection is reserved for G14; G6 creates the mounting slot.

Bored pinions retain their bore in recipe reconstruction and use distinct
geometry/BOM identities. Existing zero-bore gear recipes keep their defaults.
Preview export preserves flange ownership and included assembly paths.
Prepared smoke coverage checks all eight travel corners, flange orientation,
part-derived ratios, positive coupled speeds/accelerations, BOM, dual-Y
ownership, shaft seating and OCCT interference between moving bodies. Actual
speeds, accelerations and clearance results remain unverified: no test runs
are made after the user instruction to stop testing.

Mounting plate identity includes its motor and cutout frame, and belt clamp
identity includes band thickness, so preview sharing cannot substitute a
different bore or belt slot. Temporary OCCT allocations are owned through
`Solids.building`, including failed mount-cutout creation. This source has not
yet been compiled or exercised; the printed limit numbers remain pending.

### G7 — Cartesian gantry picker implementation

Add a 1500 × 1000 × 500 mm belt-driven picker with six 70 × 70 × 50 mm
cartons, an infeed and a two-row pallet pattern. Both tables have a stated
200 mm top height, chosen to leave the default gantry Z column above the
cartons during horizontal travel. Table height, carton sizes, layout and
materials are authored assumptions; their clearance has not been measured.
The mission has six pick/place pairs, uses the existing handling runner and
vacuum sensor, and ends after placing the sixth carton.

Extract the existing arm suction assembly into library `SuctionTool`. The arm
wrapper constructs the same parts and mates through this class; the picker
shares the catalog parts without depending on another example's source.
Register the project in the Start page, example build script, MachineKit source
roots/smoke and the application project-source checks (`PROJECT_SOURCE_ONLY=gantry`).
Scene artifact validation now rejects simultaneous machining and mission data.

The prepared application check uses the real mission and MuJoCo, verifies
three controllable coordinates, six final placements within 2 mm / 2°,
coupled speed/acceleration ceilings, sampled plan speeds and PlanCheck stalls,
robot contacts, and an assumed 200 KB budget per ordinary execution tick.
Intended compliant cup contacts allow 4 mm penetration on object contacts;
other contacts exceeding 0.5 mm fail. It prints the actual cycle time and
allocation rate when run. These results, the low-rank IK path, and source
compilation remain unverified because the user stopped testing. No measured
cycle time or passing clearance result is claimed.

### G8 — leader-based device grouping in progress

RKD6 already groups actuators by `actuator_joint`. Resolve one-leader chains
on the host and send the independent leader with the fully composed signed
ratio and joint-coordinate zero. Wiring direction changes the ratio only. A multiple-input coupling
(such as a CoreXY shaft) keeps its original shaft coordinate, so the native
compiler still evaluates the coupling sum and cannot form a false skew group.
No protocol field or version change is required for this mapping decision.

The existing step generator converts actual pulse counts through steps per
actuator unit and the signed actuator ratio before comparing skew in leader
units. Preserve that existing convention: a stated 0.5 mm racking tolerance
is transmitted as 0.0005 m, rather than raw steps. The assembly stores this
optional metadata at wire field 8 without changing existing field IDs; the
bridge converts it to SI and the robot codec/coupled limits retain it. Automatic
layouts apply the bound only to multiple single-leader drives. Multi-input
transmissions cannot request RKD6 skew monitoring.

Correct composition follows `actuator = ratio * (joint - offset)` and
`follower = couplingRatio * leader + couplingOffset`: each chain step changes
the zero to `(offset - couplingOffset) / couplingRatio`. RKD6's pulse comparison
has no per-channel zero field, so guarded pairs with different composed zeros
are rejected explicitly. G12/G13 must extend this when independent homing
introduces distinct side references.

Prepared Haxe checks cover signed dual-Y grouping, lone-axis bounds, codec
retention and transmission zeros. A native router-shaped five-joint fixture
includes both Y shafts and a separate X shaft, checks normal motion and X
missed-pulse isolation, then injects 1000 missed Y pulses against the assumed
0.5 mm bound (800 microsteps on a 2 mm lead at 16 microsteps). These checks have
not been run. Native follower coefficients are authored consistently with
the physical couplings, as required by the device compiler.

Firmware review: Nucleo remains a stub board with no pulse counter/GPIO step
implementation; its fault enum supports skew but the board does not yet run
the step generator. Hardware integration belongs to G13. Firmware compile,
runtime timing and hardware fault proof remain deferred; no build or test
was run after the user's stop-testing instruction.


### G9 — one physical gantry compilation path

Retire the public `compileXYZGantry` and `compileAssemblyAxes` APIs. All
MotionKit/ToolpathKit XYZ callers and humanoid MixedScene now construct the
actual `Gantry` and call `compileGantry`, which passes its saved mechanical
assembly, actuator metadata and component mass/inertia through
`AssemblySimulationBridge`. There is no post-bridge drive attachment for a
gantry. The old private drive attachment remains limited to the existing
single `LinearAxis` adapter; its fixture and API are retained.

The logical motion view orders X, Y and Z first while retaining all motor
shafts and passive belt idlers. It maps each leader's one-input follower chain
and caps job speeds/accelerations by the model's coupled drive limits. Job
ceilings do not overwrite motor ratings. Fixture dimensions are preserved;
their old three-Tr10-axis mechanism is replaced with the authored GantrySpec
defaults (belt X/Y, screw Z, dual Y). Existing single-axis IDs are retained.
Gantry encoder/slip checks use the real `x` leader.

Update mechanical assertions for downward Z, physical flange zero, authored
leader IDs and the extra idler/dual-drive topology. The Toolpath physical
fixture still checks upstream hull conversion independently of motion mapping.
MixedScene sends targets for all joints, derives shaft/idler targets through
couplings, derives trace dimensions from the compiled topology, and bounds
its sinusoidal amplitude by the physical rate/acceleration. Its G1 bed/base
collision setup remains in place.

Changed speed, acceleration, stall, cycle and trace numbers cannot be measured
while testing is stopped. Compilation, MixedScene execution and all migrated
fixture results remain deferred; source review does not prove those outcomes.
No build or test was run for G9.


### G10 — switch model and readings in progress

Add immutable `JointSwitch` metadata to RobotModel with monitored joint,
physical frame, home/limit role, increasing/decreasing trip side, SI trip
coordinate, hysteresis, bounded repeatability and deterministic seed. The
robot codec writes optional `switches` only when present and validates joint,
frame, duplicate ID and integer references. Models without switches retain
their existing encoded shape; no robot schema version bump is required for
this optional metadata.

`SwitchReading` uses a seeded closing-position variation per approach and a
separate opening threshold. It changes the variation only after leaving the
full uncertainty band, avoiding random stationary chatter. It records edge
coordinates and closing-edge count for later homing. `joint_switch` is an
external digital sensor kind, validated like tool contact. The simulation
adapter implements the existing step observer contract and publishes each
reading against its authored sensor mount using an explicit actual-position
reader and clock. It has not yet been installed into application simulation.

Remaining G10 work: physical roller/inductive switch parts and BOM/provenance,
optional AssemblySwitch wire metadata with flatten/freeze/codec support,
geometry-derived MachineAssembly.addSwitch, bridge lowering, physical Gantry
home/limit placement and observer installation. No compilation or test runs
have been made; this is preparatory implementation, not a completed G10.


### G10 — physical switch parts and assembly records

Add recipe/catalog-backed `LimitSwitch` and `ProximitySwitch` components and
register both in MachineKitComponents. The roller part has a housing, lever,
roller, mount holes, a pretravel-adjusted trip connector and Signal port. The
inductive M8/M12 parts expose mount, sensing face, sensing-gap trip connector,
Signal and power ports. Housing, gap, hysteresis and repeatability values are
authored generic assumptions; catalog metadata marks them Unverified and
GenericApproximation with no verified fields or vendor rating claims. Their
designations generate ordinary component BOM lines. Physical threading and
vendor-specific mounting certification are not claimed for these envelopes.

Add optional AssemblySwitch wire field 14 at the root and 11 in nested
subdefinitions. Frozen switch fields retain the same wire IDs. Flattening
resolves both switch and trigger connector endpoints, prefixes switch/joint
IDs and keeps trip coordinates unchanged under nested placement. The assembly
codec validates endpoints, roles, sides, finite thresholds and duplicate IDs.
CadKit AssemblyModel can retain these records. The simulation bridge creates
a frame on the actual switch part's collapsed rigid link, subtracts the saved
joint placement from the trip coordinate, converts distances to SI and creates
the JointSwitch plus external digital Sensor together.

These are preparatory changes. MachineAssembly geometry-derived addSwitch,
its include/rebuild/export paths, Gantry placement and application observer
installation remain pending. No builds or tests were run; recipe geometry,
wire compatibility and bridge lowering remain unverified at runtime.


### G10 — geometry-derived MachineAssembly switch bindings

Implement `MachineAssembly.addSwitch(id, joint, part, trigger, side)` with
optional role and deterministic seed. Registration requires a SwitchPart,
a fixed switch relative to the monitored prismatic joint, and a trigger
carried by that joint. Project connector separation onto the normalized
world joint direction to derive trip travel. Reject a trigger path that
misses the trip connector; reject a trip/repeatability band touching either
the soft limit or physical end stop. The part supplies hysteresis and
repeatability rather than independent copied constants.

Refresh these values when describing/exporting the current assembly so
part or connector edits change the derived trip. Copy, include, saved
rebuild, included-module reconstruction and export retain switch records
and scope their IDs and member references. Canonical sorting includes
switches. The flattener also resolves concrete member paths in builder side
records below reusable nested definitions, while retaining exposed-connector
resolution for ordinary nested endpoints. `check()` reports switch geometry
errors with the `assembly.switch-geometry` diagnostic.

Gantry switch mounting/trigger parts, placement and simulation observer
installation remain pending. No builds or tests ran; the geometry projection,
round-trip paths and end-stop checks are unverified at runtime.


### G10 — gantry home and limit switch placement

Place generic M8 inductive home switches on X/Z and independently on each
Y side when dual Y is selected. Each track also has negative/positive limit
switches. Steel trigger plates attach to the moving carriage/beam foot and
extend 3 mm beyond its longitudinal edges; their transverse extension carries
the sensing track outboard of the carriage. Fixed mounting blocks connect
each sensor's rear mount to the actual supporting beam/frame/column. Supports
at frame ends stop at the frame face rather than extending into carriage
travel. All these mounts and triggers are stated geometric design assumptions.

Home sensing planes sit one quarter of the derived guide overtravel beyond
the negative soft limit; limit sensing planes sit three quarters beyond each
soft limit. These are placement fractions, not copied trip coordinates:
MachineAssembly derives each trip again from the sensor gap connector and
trigger edge. Registration checks the repeatability band fits before the end
stop. Opposite Y home switches have distinct IDs and deterministic seeds.
Single Y omits the right-side track.

No GPIO assignments or verified electrical operation are claimed. Mounted
sensor retention, actual corner clearances, sensor trip outcomes and geometry
compilation remain unverified. Simulation observer installation and G10 checks
remain pending; no build or test was run.


### G10 — simulation switch observations installed

Carry detached immutable JointSwitch definitions in the runtime blueprint.
Compiler diagnostics require a unique switch ID, known monitored joint/frame
and the matching mounted external `joint_switch` sensor. The simulation adapter
now binds against compiled identity and sensor mounts rather than a mutable
RobotModel. Simulation validates these bindings before creating its native
robot and installs the adapter as a tick observer for ordinary simulation.
Observers are removed with Simulation disposal.

The source reader takes `runtime.snapshot().q` after each physics tick.
Native `SimulationRobot.sample` reads `source.position` from the SimKit latest
joint snapshot; `SimulationRobot.apply` adds configured joint slip to positional
targets. Thus the adapter observes resulting physics positions instead of
reconstructing requested coordinates. It publishes fresh zero-or-one digital
frames with the simulation source clock and each authored sensor mount.

Virtual-device simulations do not synthesize host switch inputs from device
feedback: G13 must supply input reads and edge capture from its device/board
path. Switch referencing, fault enforcement, power-up offsets and homing remain
G11/G12 work. G10 source implementation now includes parts, assembly records,
geometry-derived trips, bridge lowering, Gantry placement and ordinary simulation
publication. All compilation, geometry, codec round trips, end-stop checks and
simulation outcomes remain unverified because tests/builds are stopped.


G10 source review also found an existing native-slot limit applied to all
RobotModel sensors. Count only native sensor kinds for RK_MAX_SENSORS: external
switch/tool frames do not occupy that eight-slot native array. Blueprint native
lowering already filters external kinds, so this aligns compiler validation
with the actual boundary. The dual-Y gantry's twelve switch frames can coexist
with native defaults and tool feedback without changing the native ABI. This
correction has not been compiled or tested.


### G11 — joint reference state in progress

Add JointReferenceState for a compiled runtime: home-switch joints start
unreferenced, while joints without home switches retain their existing
reference convention. Every home switch on a joint must latch; one Y side
cannot establish the whole dual-drive reference. The first authored home
is the leader zero reference, and other side offsets remain separate for
G12 squaring rather than being averaged. A named RK_JOINT_UNREFERENCED error
identifies an ordinary-motion admission failure.

A physical latch records nominal trip minus observed counter coordinate.
Logical position/target conversion uses this established zero. Reference
status and coordinate offsets propagate through complete follower chains
and multiple-input coupling sums. Power-cycle invalidation removes all
home-derived references; individual-joint invalidation clears its home
latches and dependent readiness.

This is the state foundation only. Runtime/native admission, controlled
jog/homing classification, limit-switch faults, calibration application,
seek/backoff/slowlatch HomingCycle, simulation power-up offsets and automatic
router/picker homing remain G11 work. No tests or builds were run and no
reference enforcement or homing result is claimed yet.


### G11 — sensor-driven HomingCycle foundation

Add HomingAxis, HomingDriver and HomingCycle. HomingAxis takes real drive
velocity/acceleration, coordinate travel/overtravel, authored home switches
and fixed sampling interval. Seek speed is capped by one quarter of drive
speed and half the braking-distance speed before the end stop. Latch speed
is capped by seek speed and a quarter of repeatability per tick; release
travel clears hysteresis and both sides of the repeatability band. With
zero authored repeatability, latch requires an edge capture; the numerical
return-position tolerance uses an explicit 1 µm floor when no positive
repeatability is available.

The cycle orders Z before X/Y (including qualified axis IDs), starts only
at rest, seeks to the first switch, stops under acceleration control, backs
off until every switch is released with margin, stops, approaches slowly,
captures every home edge, stops, establishes references, then returns to
the authored home. Initially active switches start with backoff. Stops
must finish before reversing or applying coordinate zeros. Return uses
an observation after calibration, so the raw-to-logical coordinate change
cannot be mistaken for excessive travel. Missing signals, excessive
travel/time, late sampling without edge capture and invalid observations
fault and request a controlled stop. Cancellation stops the current axis.

HomingDriver is the explicit execution boundary: observation, homing-classified
velocity/stop/return commands, and latch that establishes runtime zeros and
resets encoder/slip monitors. Its runtime implementation is still pending.
The state machine is not yet connected to MotionSystem.home(), runtime
admission or application startup. No homing result is claimed and no tests
or builds have been run.


### G11 — native reference admission foundation

Add native per-joint reference requirements/latches and C APIs to configure,
latch and query them. Readiness propagates through every coupling leader,
including chains and multi-input followers. Configuration/latch changes require
an empty mailbox, no trajectory/stop ramp, observed rest and no nonzero active
velocity target. Joints without a configured requirement retain existing
behavior. Native ordinary position/servo/effort targets, unclassified segment
batches and ordinary plans reject unreferenced coordinates with
RK_ERROR_UNREFERENCED; velocity targets remain available for controlled jog.

Add exclusive RK_PLAN_JOG/RK_PLAN_HOMING purpose flags. They may coexist with
jerk-unchecked metadata but cannot coexist with each other or carry timed
process events. They retain existing polynomial, coupling and drive-limit
validation. Plan submission performs the reference admission check under the
queue lock before any sequence or queue mutation. Runtime API version becomes
25; there is no native struct-layout change in this stage.

This remains foundational: generated FFI bindings must be regenerated for API
25 and the new functions/constants; Haxe runtime must configure requirements
from compiled homes, apply latch/calibration and identify jog/homing plans.
Homing travel into overtravel, native coordinate-zero application and
limit-switch fault enforcement also remain pending. Thus existing application
machines do not yet configure this native gate automatically. No build/test
ran; native admission behavior is unverified at runtime.


### G11 — execution purposes and generated bindings

Native admission foundation: `0fb59f54d`. Regenerate runtime FFI bindings from
its API 25 header using the existing portable ABI source generator. Add
immutable Program/Jog/Homing purpose metadata to ExecutionPlanSubmission,
preserve it in copies and array factories, and reject process events on jog
or homing submissions. RobotRuntime maps purpose to the native plan flags.
MotionSystem tags initial, deferred and smoothly replaced jog trajectories;
TrajectoryStream carries the tag through every submitted chunk. Ordinary
programs retain Program as the default. Purpose belongs to each trajectory,
so a later ordinary move cannot inherit the jog admission category.

This is source integration only. No tests or project builds ran. Runtime
reference configuration, calibration, homing-driver execution, overtravel and
limit enforcement remain pending; G11 remains in progress.


### G11 — configure native reference requirements on runtime creation

Execution-purpose integration: `b75c926e3`. RuntimeEndpoint now exposes native
reference requirement configuration. RobotRuntime creates its per-machine
JointReferenceState and configures each directly home-monitored joint before
returning the runtime to its caller. Coupled follower readiness remains native
fixed-point propagation, without redundant follower requirements. A failed
initialization closes the endpoint instead of leaving its handle owned by an
unconstructed runtime. Models with no home switches retain their existing
reference convention.

Machines with authored homes now reject ordinary plans and positional targets
until real homing establishes references. No synthetic startup latch is added.
The runtime latch/calibration and HomingDriver connection still need work, so
router/picker startup remains incomplete. No tests or builds ran; this source
change and its effects on application startup are unverified.


### G11 — validate homing polynomials through authored overtravel

Reference configuration: `e7a3e94fa`. The native polynomial admission check
widens position bounds by each joint's authored overtravel only for plans
explicitly flagged RK_PLAN_HOMING. Jog, ordinary plans and legacy segment
batches keep soft-limit bounds. Velocity/acceleration claims and continuity
checks retain their existing limits; observed stop/fault bounds are unchanged.
Missing versioned overtravel means zero, and nonfinite widened endpoints are
rejected before admission.

This is one part of homing execution. Straight velocity-target homing and its
stop ramp still use the existing soft-limit handling, and must be integrated
with purpose-aware control before the homing driver can use them. Coordinate
calibration and application home startup remain pending. No tests or builds
ran; no homing travel or stopping behavior is claimed as verified.


### G11 — preserve homing bounds when a queued path stops

Homing polynomial admission: `a2724349e`. Native trajectory knots now retain
plan flags, including end markers. When a path stop falls back to a straight
ramp, it captures the interrupted chunk's homing purpose before clearing the
queue. Braking room and emitted ramp targets then use the same authored
overtravel bounds as that homing plan. Ordinary interrupted chunks retain
soft limits, even when a later queued chunk has another purpose. Existing
acceleration-duration and limit-hit fault handling remain in place.

Direct velocity targets have no homing-purpose transport yet and retain soft
bounds. Native coordinate calibration, latch, driver and startup integration
remain pending. No tests or builds ran; stopping behavior is unverified.


### G11 — carry closing-edge captures in switch sensor frames

Purpose-aware queued stop ramps: `950cfb8a6`. SwitchReading now retains its
latest closing edge separately from the latest edge, which may be a release.
Simulation publishes digital state, capture-valid bit, closing-edge position
and closing-edge count in immutable joint_switch SensorFrames. Their existing
sequence/timestamp/source-clock metadata remains available to the homing
observation driver. JointSwitchFrame validates and decodes that payload;
legacy one-value digital frames remain supported without an edge capture.
Capture positions use the source joint coordinate and require the eventual
counter/calibration transformation in the driver. This is simulated capture
metadata, not proof of firmware input capture (G13 remains pending).

Homing observation freshness and edge-count advancement must still be enforced
by the runtime driver. No tests or builds ran; runtime behavior is unverified.


### G11 — enforce homing observation freshness and edge identity

Switch capture transport: `90e7c8e72`. Homing observations now require source
timestamps/clocks, and each home signal carries its sequence plus optional
closing-edge count. Reject missing/duplicate signals, nonpositive sequences,
invalid capture identity and clock changes. HomingCycle requires position and
switch observations to advance on each control update; switch timestamps must
share the position clock, never lie in its future, and stay within one authored
sampling period. Freshness rejection requests the existing controlled stop.

At slow-approach entry, record closing-edge counts. A captured latch must have
a larger count; a count reset or an old seek capture faults. Digital-only
sources retain the existing sampled-position repeatability budget. Post-latch
coordinate rereads may share the same physics sample and are not a new control
update. Actual driver construction and counter transformations remain pending.
No tests or builds ran; no freshness or homing behavior is runtime-verified.


### G11 — preserve simulation source-clock identity

Freshness enforcement: `da46ecf94`. Native ordinary simulation samples use
simulation time (simulation_robot.cpp), while RuntimeRobotAdapter previously
labeled every position snapshot "unspecified". RobotRuntime now retains an
explicit source clock supplied at creation; the adapter carries it into its
transport-independent snapshot. Ordinary Simulation runtimes declare
"robotkit.simulation", matching their switch sensor frames. Virtual-device and
other existing factories retain "unspecified" until their actual source domain
is explicitly configured; no device clock conversion is inferred.

This fixes the source-clock mismatch needed by the homing freshness checks.
The concrete observation/motion driver and calibration remain pending. No
tests or builds ran; the source integration remains unverified at runtime.


### G11 — concrete runtime homing observations

Source-clock propagation: `4b92c3784`. RuntimeHomingObserver reads joint state
and switches from one Robot snapshot. It resolves each authored home sensor by
ID and mounted frame, decodes digital/captured payloads, and forwards sequence,
timestamp, clock and edge-count metadata to HomingCycle. Reject unknown joints,
missing/wrong mounts, duplicate/null frames, malformed payloads and runtime
safety faults. Legacy digital frames explicitly carry no captured-edge count.

An explicit coordinate-translation callback converts both observed positions
and captured edges; velocity is unchanged because this boundary supports only
coordinate translation. The eventual calibration driver must supply counter
coordinates before latch and logical coordinates after it. This observer is a
concrete read path; command execution, native calibration and MotionSystem
startup wiring are still pending. No tests or builds ran.


### G11 — native limit-switch fault enforcement

Runtime observation path: `cd8995640`. Add native limit-input reporting per
joint, RK_FAULT_LIMIT_SWITCH and API 26 generated bindings. An active input
clears pending commands and invokes the same native fault/stop path used by
observed overtravel. Safety reset is rejected while any reported input remains
active. Runtime external switch publication aggregates all authored limit
signals on the same joint under its publication mutex before reporting native
state, so releasing one switch cannot clear another active input. Home signals
do not use this path. RuntimeRobotAdapter exposes the named limit_switch fault.

The existing simulation switch observer now reaches this native path through
its frame publications. Releasing inputs permits an explicit reset but does
not automatically clear the latched fault. Device input reporting remains G13
work. No tests or project builds ran; only FFI source generation was performed.
Calibration, homing commands and application startup remain pending.


### G11 — native coordinate translation at the endpoint boundary

Limit fault enforcement: `fc03334aa`. Add native atomic coordinate calibration
for owner-driven endpoints: logical position = endpoint position + per-joint
zero. Require an empty mailbox/trajectory, no stop ramp, observed rest and
positional held targets; reject safety faults, nonfinite rebases and zeros
inconsistent with every coupling's linear terms. Rebase observed/commanded
positions, held targets and controller references together under owner, queue
and state locks. Plan coefficients and limits stay in logical coordinates.
Before endpoint apply, translate positional outputs back into endpoint
coordinates; translate sampled observations into logical coordinates before
limit/following-error checks. Reset loses coordinate zeros and home latches.

This C++ operation is not yet exposed through C/Haxe or invoked by the homing
driver. Device-owned polynomial queues explicitly return unsupported pending
G13 transport calibration. Simulation switch sensing must keep reading actual
endpoint coordinates after calibration rather than calibrated logical values;
that wiring remains pending alongside latch and startup integration. No tests
or builds ran; coordinate behavior is unverified.


### G11 — expose calibration and preserve physical switch coordinates

Native translation: `1f3834340`. API 27 adds borrowed-array coordinate
calibration and a snapshot in endpoint coordinates. The latter converts both
positions and held setpoints under the native owner/state locks, keeping a
single coherent coordinate zero. RuntimeEndpoint exposes calibration and
endpoint observation. RobotRuntime.physicalPositions() reads that snapshot;
the simulation switch observer now uses it, so logical calibration cannot
shift physical trip locations or their captured edges. Ordinary robot
snapshots remain logical.

Regenerate FFI bindings from the C header; no manual generated edits. The
actual homing latch transaction, monitor reset and motion driver remain
pending. No tests or project builds ran, and behavior is unverified.


### G11 — atomic calibration and reference admission

Calibration exposure: `2ed56edfa`. API 28 adds calibrate_home: install a full
coupling-consistent coordinate-zero array and latch the explicitly completed
home joints in the same native transaction. Reject unknown, duplicate or
non-required reference indices before any coordinate mutation. Existing
at-rest, mailbox, queue, safety and finite-value checks apply to the whole
operation; failures leave both zeros and admission state unchanged. Empty
reference lists permit intermediate side captures without prematurely admitting
a multi-switch joint. RuntimeEndpoint exposes this operation for the driver.

Regenerate bindings from the header. Haxe reference-state proposals and monitor
reset still need to invoke it. Device queues remain unsupported until G13
calibration transport. No tests or project builds ran.


### G11 — transact runtime home latches from Haxe

Atomic native admission: `f7df755a7`. JointReferenceState now makes detached
candidate copies and exposes a copied full zero array. RobotRuntime.latchHome
applies an actual captured endpoint/counter coordinate to a candidate, invokes
the atomic native calibrate_home operation, then commits Haxe state only on
success. All home signals on the joint must be latched before its reference
index is included. Serialize candidates and reference queries with a mutex.
Expose reference readiness and zero for driver observation conversion.

Simulation.resetRobot synchronizes Haxe invalidation after the native reset
and clears old external frames, preventing reuse of pre-reset switch data or
host-side references. Mechanical switch-reader reset/reseed and monitor reset
at latch still need integration, as do homing commands and startup. No tests
or project builds ran; latch behavior is unverified.


### G11 — reset mechanical switch history and distinguish capture coordinates

Haxe latch transaction: `c23a126da`. Simulation retains each installed switch
observer by robot index and reseeds its SwitchReadings after robot reset.
Closing/release history and captures restart from the authored deterministic
seed; publication sequence remains monotonic for existing sensor consumers.
Native reset also discards old limit-input state; the next physics sample
republishes the restored physical state. Disposal clears observer ownership.

Correct RuntimeHomingObserver's conversion contract: runtime positions are
logical after calibration, while physical switch captures remain in endpoint
coordinates. Use separate mandatory position and edge translation callbacks.
Applying the same translation to both would double-apply the zero to one source.
No driver constructor calls existed to migrate. Homing motion, encoder/slip
monitor resets and application startup remain pending. No tests or builds ran.


### G11 — concrete runtime homing motion driver

Reset/coordinate corrections: `9a0532072`. RuntimeHomingDriver implements
HomingDriver with bounded, explicitly Homing-classified plans. Clone logical
axis mappings with authored physical overtravel bounds for homed coordinates;
retain follower scales/offsets. Seek/backoff/approach choose the endpoint in
the requested direction and use drive-derived speed/acceleration. Generate
coupling-consistent jerk-limited plans through AxisPlanner, anchor them on
runtime held setpoints and submit immutable full polynomial payloads. Native
controlled stop interrupts those paths at a switch observation. Return uses
the authored home coordinate through the same homing admission category.

Latch calls the atomic runtime operation, then a required caller-owned monitor
reset callback. RuntimeHomingObserver supplies logical joint observations and
physical captured edges without double translation. Reject missing physical
mappings or primary coordinates not expressed directly in SI. HomingAxis now
retains physical travel bounds. MotionSystem/home startup and actual monitor
ownership must still connect this driver; simulation power-up offsets and G12
independent side control remain pending. No tests or builds ran.


### G11 — MotionSystem sensor-home lifecycle

Runtime driver: `84de3e407`. MotionSystem derives physical HomingAxes from
compiled home-switch metadata and independent logical-axis mappings, requiring
physical velocity/acceleration and an authored overtravel window. Reject home
switches without a mapped independent axis. configureRuntimeHoming binds the
runtime and a required monitor-reset callback. With home switches, home()
starts HomingCycle and update() advances it; isMoving reports active homing.
Abort cancels via controlled stop (emergency requests remain available).
Reject ordinary immediate/queued motion while the cycle is active. Expose
homingStatus for startup progress. Unswitched robots retain coordinate home.

Application owners still need to configure their monitor callback and home
before program/mission startup; this wiring is not yet present. Hold/resume
semantics during homing, simulation power-up offsets, monitor/slip physical
reset effects and G12 side control remain pending. No tests or builds ran.


### G11 — preserve authored axis placement during homing

MotionSystem integration: `1ec505a7e`. Router MotionAxis mappings use a placement
intercept (native joint position = authored position minus initial placement).
Convert authored home to native joint coordinates when building HomingAxis.
RuntimeHomingDriver retains that intercept: express expanded physical bounds
and every requested physical seek/return coordinate in the mapped axis's
logical coordinates before planning. Continue requiring unit SI scale for the
independent homing joint, while retaining all follower mappings. This avoids
rejecting the router's existing placement convention or returning it to the
wrong native home coordinate.

Router/picker startup and monitor ownership still need wiring. No tests or
builds ran; coordinate mapping and homing remain runtime-unverified.


### G11 — expand switched motion axes through physical followers

Placement mapping correction: `a91b86795`. MotionAxisCouplings expands task
axes through the compiled coupling graph, composing scales and affine offsets
across chains and same-axis multiple terms. Retain authored order/primary
joints; append physical followers and reject contradictory existing mappings
or joint sharing. MotionSystem applies this expansion when home switches exist,
so homing plans include motor followers as well as carriage coordinates.

Reject followers with terms from several independent task axes, or a partially
mapped moving follower: these need coordinated polynomial planning rather than
one independent-axis profile. Existing unswitched planning stays unchanged.
Router startup now has a reusable full-model mapping path, but application
configuration and monitor resets still need wiring. No tests or builds ran.


### G11 — home router machines before machining startup

Follower expansion: `faa6304b1`. CncProgramPlayer now creates a full-machine
homing MotionSystem when the runtime has home switches, using the existing
axis placement and drive caps. Its latch callback invokes StepperSlip.reset
and EncoderMonitor.reset with the calibrated observed joint positions.
Feed waits for the first home sensor frames, starts the physical cycle, and
advances it before any machining program is submitted. After completion,
recompile the initial program from the homed machine pose. Homing errors stop
startup and report a machine-homing failure. Reset recreates the home view and
requires homing again.

Expose homingSeconds separately from existing machining run/motion CPU timing.
Picker mission startup remains pending. Homing after previously accumulated
physical slip, nonzero power-up offsets, collision/clearance and cycle timing
are unverified; no tests or builds ran. G11 is not complete.


### G11 — home switched machines before mission startup

Router startup: `a539f37f2`. MissionPlayer builds a full-machine homing view
from home-monitored independent joints and compiled physical limits. Native
origin zero is the saved assembly pose; clamp the return coordinate to its
soft travel. Configure coupled follower expansion and caller-owned EncoderMonitor
and StepperSlip resets at latch. Feed waits for actual home frames and finishes
the cycle before starting any mission skill, including picker handling.
Report homing failure and duration separately. Reset recreates the homing state;
beforeReset cancels active homing through its controlled-stop path.

Machines without home switches keep their existing mission startup. Power-up
nonzero offsets, previously accumulated slip, independent Y squaring and all
physical/mission timing checks remain pending or unverified. No tests or
builds ran, so G11 is not proven complete.


### G11 — separate simulated counter origin from lost-step slip

Mission startup: `86da82a74`. Simulation endpoints now keep a distinct
power-up counter-origin vector, initialized/reset to zero. Positional commands
convert counter targets to physical coordinates through that origin; sampled
counter positions subtract it. Existing lost-step slip remains a separate
command effect, so observing counters still detects newly lost steps.
Physical sensor snapshots convert counters back through the endpoint's
physical_position contract. Simulation switch readers sense physical trips
but publish closing-edge captures converted into counter coordinates.

No nonzero origins are configured yet: the scene/API setter and startup pose
handling remain pending. This separation enables proper calibration rather
than latching a physical edge as if it were an unknown counter. Native/Haxe
concurrent reset/calibration and nonzero-offset behavior remain unverified.
No tests or builds ran.


### G11 — stopped-world power-up offset API

Counter-origin separation: `75b6b6540`. Add Simulation.setPowerUpOffsets and
its native borrowed-array API. Validate a full finite coupling-consistent SI
vector, unchanged cold/reset origins, no existing lost-step slip, and physical
poses within soft travel before changing the world. Require runtime sequence
zero and a stopped world; virtual-device endpoints return unsupported until
G13. Place one-DOF joints with SimKit's initial-state API and retain those offsets
as counter origins. Replace startup holds with holds at the physical offset
pose, so counters retain their initial readings while the actual machine is
shifted. Attempt to restore prior joint states if a backend placement fails.

Regenerate SimKit FFI sources. Typed scene authoring, coupling propagation from
named axes and reset reapplication remain pending. Existing setJointSlip stays
available for later lost steps; combining unknown startup counter origin with
that later physical fault would hide monitor errors. This refines the plan's
original setJointSlip-only sketch. No tests or builds ran.


### G11 — typed scene power-up offsets

Stopped-world simulation API: `726449a72`. Add optional powerUpOffsets to
machining and mission JSON sections, with typed entries {joint, offset} in SI
units relative to the saved physical pose. Decode field by field; reject
unknown entry fields, nonnumbers, null entries, duplicate names, nonfinite
values and layouts beyond the runtime joint bound. Validate joint references
against the flattened assembly and require home-monitored prismatic axes.
The existing JSON sections carry the optional data without a binary layout
change; absent fields remain absent and retain zero startup offsets.

Application projection into full coupling-consistent vectors and stopped-world
configuration/reset reapplication remain pending. No tests or builds ran;
serialization and semantic validation are source-reviewed only.


### G11 — apply authored power-up offsets and retain them across reset

Typed scene data: `43f1a1dc5`. Preserve machining powerUpOffsets through CncJob
projection. AssemblyRobot selects its machining/mission offsets while creating
the runtime, resolves names through compiled identity and applies the complete
vector before the first sample. PowerUpOffsets resolves independent home axes
and propagates displacements through every coupling term, including chains and
same-/multi-input sums. Do not add affine intercepts to a displacement. Reject
unknown/dependent named axes, missing homes, nonfinite values and unresolved
graphs. Native stopped-world placement performs physical travel checks.

Simulation copies the successful vector and reapplies it after robot reset,
then invalidates host references and resets sensor history. Default absent
scene offsets stay zero. Nonzero startup poses, homing repeatability, reset
geometry and follower placement remain unverified; no tests or builds ran.
G11 source now covers startup offset authoring through application creation,
but full G11/G12 behavior is not proven complete.


### G12 — explicit switch-side drive observation

G11 startup-offset wiring: `e45258fe8`. JointSwitch gains optional driveJoint,
retained by model codec and runtime copies with known-joint validation. Absent
bindings preserve leader observation. SwitchDriveBinding resolves a specified
motor shaft through its single-input coupling chain to the monitored axis,
composing ratios and affine zeros; reject unrelated, cyclic or multi-input
side mappings. Simulation switch preflight resolves the binding before robot
creation. Observe that physical drive and convert shaft position into the
axis SI coordinate before applying the switch trip model. Captured edges are
converted into the same axis's counter coordinate.

Gantry assembly authoring must still identify each side's motor, and the
squaring controller must hold sides independently after their edges. This is
side-observation support, not completed squaring. Skew relaxation, motor zeros
and the 1 mm result remain pending/unverified. No tests or builds ran.

### G12 — author drive bindings through assembly metadata

Core drive observation: `8ab0616c3`. Add optional switch wire field 13
driveJoint to mutable and frozen assembly records. Preserve and namespace it
through copy, flattening, inclusion and export; validate that it names a movable
joint. The simulation bridge forwards the binding to JointSwitch. Gantry home
and limit switches now name their actual screw, driven pulley or pinion shaft,
including distinct YLeft and YRight drives. Source inspection confirms these
IDs match AxisBuilder's authored motor joints. Model encoding also checks the
optional reference before writing it.

This completes assembly wiring for side observation. Independent side holds,
motor reference offsets and explicit squaring-only skew relaxation remain
pending. No tests or builds ran; compilation and physical behavior are
unverified.

### G12 — simulation shaft-hold foundation

Assembly side-drive wiring: `504c84dd2`. Add an internal simulation operation
to hold an actuated, nonpassive shaft at an explicit physical position. While
the leader's positional targets continue, update that follower's slip to keep
the shaft target fixed. Held servo velocity is zero. Releasing retains the
accumulated follower displacement; reset clears all hold flags and slip.
Reject nonpositional commands on held shafts before staging any targets.

This operation is not yet exposed through the C/Haxe boundary or connected to
the homing cycle. The controller must supply a fresh physical shaft position,
release holds on all exit paths, establish independent motor reference offsets
and manage squaring-only skew relaxation. Mechanical coupling behavior remains
unverified. No tests or builds ran.

### G12 — expose simulation holds and stage follower displacement

Native hold foundation: `6b771a626`. Expose set_squaring_hold through the C
API and Simulation.setSquaringHold through Haxe. The C boundary accepts only
active values 0/1; Haxe rejects nonfinite positions. Regenerate canonical FFI
declarations from the header for the four portable ABI targets (source
generation only). Stage hold-induced slip with positional targets and commit
it with accepted pending commands, so discarded staging leaves retained slip
unchanged.

No project build or test ran. Controller wiring, release on exit paths,
independent calibration, squaring-only skew relaxation and physical coupling
behavior remain pending/unverified.

### G12 — simulation side-control adapter

Hold API: `3b9397949`. Add method-only HomingSideControl and a simulation
adapter resolving explicit home-switch drive bindings. Simulation.homingSides
uses the blueprint and runtime retained at the same robot index, preventing
caller-supplied identity mismatches. Hold a shaft at its current physical
snapshot coordinate; repeated holds on the same shaft are idempotent. Release
attempts every held shaft, retains failed releases for retry and reports the
first failure. Successful releases preserve native follower displacement.

HomingCycle and application owners still need to use this capability; reset
reconciliation, independent motor calibration and skew relaxation remain.
No tests or builds ran; the adapter is uncompiled and behavior unverified.

### G12 — connect side holds to slow homing approach

Side adapter: `8ecef17c0`. MotionSystem accepts a HomingSideControl and passes
it into HomingCycle; router and mission players supply their simulation-owned
adapter. Multi-switch axes require distinct explicit drive bindings and a side
controller. Each new slow-approach capture holds its motor while uncaptured
sides continue. Release all holds after the controlled stop before latching;
start/update faults and cancellation attempt release even if stopping fails.

Independent motor calibration and monitor-reset preservation of the resulting
slip remain unresolved; this wiring alone does not prove squaring. Fast seek
still stops the whole axis at its first switch. Skew relaxation and the 1 mm
result remain pending. No tests or builds ran; source remains uncompiled.

### G12 — preserve side alignment and physical slip during monitor rebase

Controller wiring: `4a4026210`. Refine simulation offset storage: alignment
created by a side hold is its own additive squaring_offset, alongside lost-step
slip and the power-up counter origin. Stage alignment transactionally and retain
it on release; reset clears it. Hold compensation subtracts current lost-step
slip, so existing losses do not change the held physical target. Cold power-up
placement refuses an existing alignment offset.

StepperSlip.rebaseAfterHoming clears loss history without writing endpoint
offsets. Retain previously applied offsets as a baseline, then add subsequent
plan losses on top after coupling propagation. Router and mission latch
callbacks use this rebase instead of reset; session reset still clears physical
slip. This also avoids moving an already slipped machine merely to clear its
monitor history.

Independent motor coordinate calibration, skew relaxation and the physical
1 mm squaring result remain pending. No tests or builds ran; all source and
behavior remain unverified.

### G12 — retain independent shaft zeros and bound simulation ownership

Alignment preservation: `a0f8ee79c`. JointReferenceState resolves each home
switch drive and retains that mapping across transactional copies. Expose the
individual shaft translation as ratio times captured axis-coordinate zero;
affine coupling intercepts do not enter translations. Validate finite shaft
zeros before accepting a latch, and expose a mutex-protected runtime accessor.
These zeros are not yet installed in native motor coordinates: native
calibration currently requires coupling-consistent propagated zeros, so that
contract needs an explicit independent-side extension.

Reject simulation shaft holds on virtual-device bindings. Router beforeReset
now aborts active homing so side-release cleanup runs before native reset, as
in mission playback. No tests or builds ran; behavior remains unverified.

### G12 — native independent counter-rebase foundation

Retained side zeros: `f5a752c6c`. Keep the planner/native coordinate-offset
coupling contract intact by rebasing the individual simulation motor counter
origin. Internal calibrate_home_drive computes delta = propagated zero minus
that motor's captured side zero. Require referenced, stationary joints and no
pending commands, trajectory or stop ramp. The endpoint shifts counter_origin
by delta and alignment by minus delta, preserving their sum and physical
targets. Adjust the cached measured logical position by minus delta; commanded
logical targets remain coupled. Preflight finite results before endpoint writes.

The default endpoint rejects this operation; SimulationRobot accepts only
actuated nonpassive shafts with no hold or staged targets. C/Haxe exposure,
controller invocation exactly once per side per latch cycle, atomic multi-side
handling and device support remain pending. No tests or builds ran. This source
foundation does not establish verified independent calibration or squaring.

### G12 — atomic multi-side counter rebasing

Counter-rebase foundation: `61a3a5a08`. Replace the internal single-shaft
operation with a complete batch. Runtime and simulation endpoint reject empty,
null, duplicate, unknown or nonfinite batches before changing state. Validate
every side's readiness, measured result and endpoint origin/alignment results,
then commit all counter origins and cached measurements together. The endpoint
contract explicitly requires all-or-none behavior. Preserve existing stationary,
no-hold/no-staging and physical-target invariants.

Successful side calibration advances the calibration revision, refusing
overflow before mutation. Plan revision admission moves under the owner/queue
locks so a plan prepared before rebasing cannot race the calibration commit.
C/Haxe exposure and once-per-cycle controller invocation remain pending. Other
coordinate-calibration revision paths still need auditing. No tests or builds
ran; atomic behavior and compilation remain unverified.

### G12 — expose atomic motor-counter calibration

Atomic native batch: `3bf133ebb`. Runtime API 29 adds
rk_robot_runtime_calibrate_home_drives with borrowed equal-length joint/zero
arrays. Add matching RuntimeEndpoint and NativeRuntimeEndpoint methods; reject
null, empty or mismatched Haxe arrays before borrowing. The native method
validates all motors and commits the batch while preserving physical targets.

Regenerate runtime and simulation FFI declarations for four portable ABI
targets (source generation only); simulation imports the updated runtime
declarations without a local generated diff. RobotRuntime switch-ID projection,
once-per-latch tracking and HomingCycle invocation remain pending. No tests or
project builds ran; compilation and behavior remain unverified.

### G12 — apply each captured motor calibration once

Public calibration API: `b0ecac1ea`. Track motor-calibration consumption per
home latch in JointReferenceState, preserving it in transactional copies and
clearing it on fresh latch or invalidation. RobotRuntime projects switch IDs
to distinct drive indices and captured shaft zeros under its reference mutex,
requires all leader-side latches and commits consumption only after successful
atomic native calibration. HomingSideControl exposes calibration; the simulation
adapter refuses it while any shaft hold remains. HomingCycle invokes it after
releasing holds and latching all sides, before return motion.

No tests or builds ran in this step. Explicit squaring-only skew handling,
1 mm startup racking authoring/coverage and physical calibration behavior remain
pending. Do not claim G12 complete.

### Updated validation cadence — user instruction

Finish G12 before proceeding to G13. Then stop implementation for a build pass:
compile every kit and the app (use --compiler-only), build the native runtime
and run ctest, then run G7 picker and homing tests once. Resolve failures before
continuing. After that checkpoint, full test runs occur only at phase boundaries.
This replaces the earlier stop-testing instruction; avoid starting the checkpoint
until G12 implementation is ready.

### G12 — resolve explicit startup side displacement

Homing calibration wiring: `edc64c50a`. PowerUpSideOffsets resolves a named
home switch to its explicit motor follower. Input displacement is in the
monitored axis SI units: a 1 mm side offset is 0.001, converted through that
side's composed ratio, without an affine intercept. Add it to a copied full
base offset vector and return the explicitly affected drive indices. Reject
unknown/nonhome switches, absent bindings, leader bindings, duplicate shafts
and nonfinite/overflowing offsets.

Native cold placement currently requires coupling-consistent offsets. It must
accept the returned explicit side-drive list through a separate checked
placement API, preserve reset behavior, and reject nonactuated/passive shafts.
Scene authoring and the 1 mm regression must then use that API. This resolver
alone does not place or square the machine. No tests or builds ran; the post-G12
checkpoint is still pending G12 implementation.

### G12 — place independent startup motor sides

Startup resolver: `968c74ecd`. Add rk_simulation_set_power_up_sides and the
Haxe setPowerUpSides wrapper. Cold placement accepts a full offset vector plus
an explicit side-drive list; validate unique known actuated nonpassive shafts
with exactly one incoming coupling. Only those followers may deviate from
shared-axis displacement propagation. Existing finite/travel checks, cold
state requirements, virtual-device rejection and backend rollback remain.
The original power-up API retains its coupling-consistent contract.

Haxe retains copied offsets and side identities, reapplies them through the
same side-placement API on robot reset, and clears caches on disposal.
Regenerate simulation FFI declarations for four portable ABI targets (source
generation only). Scene authoring, 1 mm coverage and skew scope remain pending.
No project builds or tests ran; physical placement remains unverified.

### G12 — author and apply startup racking in scenes

Native side placement: `b746b422f`. Add typed optional powerUpSideOffsets
entries {homeSwitch, offset} to mission and machining data. Decode fields
strictly and validate finite SI displacement, bounded arrays, physical home
switches on prismatic axes and distinct explicit motor drives. Preserve data
through CncJob projection. AssemblyRobot combines ordinary propagated offsets
with resolved side offsets, then performs one cold placement before samples.
Absent fields preserve default startup.

The picker scene now authors a +0.001 m right-Y startup side offset using
switchYRighthome, exercising the requested 1 mm racking case when played.
This is input authoring, not proof of successful physical squaring. Explicit
skew scope and homing coverage still remain before the post-G12 compile/runtime
checkpoint. No builds or tests ran; all new behavior remains unverified.

### G12 — explicit squaring scope and bounded device skew override

Startup scene wiring: `623de1e3f`. HomingSideControl now opens/closes an
explicit squaring group. The simulation adapter validates distinct known drives,
refuses holds outside that group and requires calibration of its complete
identity set. HomingCycle opens the group for a multi-side homing move, releases
holds for latching, calibrates, then closes before return. Fault/cancel cleanup
closes the scope; failed releases retain it for retry.

StepGenerator gains an explicit finite bounded override for one configured
skew pair. Reject unknown/ambiguous pairs, repeated activation and bounds below
the normal bound. Other pairs retain their limits, and end_squaring restores
all normal bounds. This does not disable skew checks. G13 must carry homing
scope and independent counter zeros through the device protocol and call these
operations on every lifecycle exit; simulation holds remain unsupported on
virtual-device endpoints until that transport work. No builds or tests ran.
Physical homing regression coverage and the post-G12 checkpoint remain pending.

### G12 — compensate leader zero for first-side hold displacement

Squaring scope: `d6eb92737`. Source tracing of the intended physical regression
revealed that using the first held motor's original edge directly as the leader
zero leaves the leader's subsequent travel unaccounted for. Retain two zeros:
the individual motor zero from its original captured counter, and the leader
zero from a compensated capture. At rest, convert current motor counter to
axis coordinates and subtract its difference from the current leader counter
from the original capture. This includes the displacement accumulated while
the motor held. Runtime positions are reduced by their existing coordinate
zeros before computing the counter difference.

Pass the compensated leader capture through HomingDriver and RobotRuntime,
retain/copy/invalidate it separately in JointReferenceState, and continue using
the original side zero for the atomic motor calibration. For 1 mm subsequent
leader travel, this corrects the missing 1 mm leader translation instead of
leaving both aligned motors displaced from true zero on return. This is source
reasoning, not runtime evidence. Physical regression and the requested build
checkpoint remain pending; no builds or tests ran.

### G12 — native physical one-millimetre regression

Leader capture correction: `8e98924bb`. Add a native simulation regression
with one prismatic leader and two 1000 rad/m motor followers. Place a 1 mm
right-side startup offset, advance to the first edge, hold the left motor,
continue the leader by 1 mm until both motors align, release and install the
compensated leader and original motor zeros. Check that duplicate motor batches
are rejected, calibration preserves all physical positions, and return targets
put both shafts at true zero within 20 micrometres, including relative skew.
The fixture uses actual simulation stepping and physical snapshots; it does
not emulate or cover Haxe sensor freshness and cycle sequencing. Register it
in the existing native simkit test executable.

The regression has not run. Add cycle sequencing coverage, then execute the
requested post-G12 kit/app compiler-only and native build/ctest checkpoint, plus
picker and homing runs once. G13 remains unstarted.

### G12 — cycle sequencing coverage and checkpoint start

Native physical regression: `d4a61a7c5`. Add standalone HomingTests and
MOTIONKIT_HOMING_ONLY selection in the existing MotionKit test entrypoint.
Cover release/backoff/slow-approach ordering, first-side holds, stop/release/all
latches/calibration/scope close before return, compensated leader capture,
stale closing-edge rejection and cancellation cleanup when stopping throws.
The fixture covers controller sequencing; native simkit covers physical
placement/calibration separately. Both remain unrun at this commit.

Begin the requested post-G12 compiler-only checkpoint. G13 is unstarted.
Do not mark G12 verified until native ctest, picker and homing runs establish
the required physical and controller behavior.

### G12 checkpoint — first compiler error

MotionKit test compilation terminated with E1009 at CoreXyPlotter slide:
its local PlotterAxisSpec omitted the optional rackingTolerance now present in
the shared AxisSpec. Replace the local shape with the shared AxisSpec alias.
Restart the compiler-only pass after that source fix; no runtime tests have
run yet. This is a compilation checkpoint, not evidence of completed G12.

### G12 checkpoint — native build and first ctest results

Enable ROBOTD_BUILD_TESTS in the existing RobotKit native build configuration.
Native runtime and all enabled test targets built successfully. ctest ran all
18 tests: 17 passed; robotkit_simkit failed at the new 1 mm startup assertion
for the right shaft before holding. Physical coupling/origin behavior needs
investigation; do not weaken the assertion. The other native results do not
prove G12 squaring.

MotionKit compile retries exposed SwitchPart narrowing hiding the component
connector method, then nullable edge-count numeric comparison. Use a fresh
component lookup for connector validation and explicit integer sentinels for
optional counts, preserving capture freshness checks. A new compiler-only retry
is started. Other kits, app, picker and Haxe homing execution remain pending.

### G12 checkpoint — physical coupling conflict confirmed

Measured startup after the first move: leader -0.005 m, left -5 rad, right
-5 rad instead of -4 rad. The backend unconditionally overwrote follower
positions with physical coupling equations. Install hard constraints only for
coupling groups connected to a servo; commanded kinematic followers must retain
the independent slip/origin targets. Servo-connected groups are identified by
fixed-point propagation through the coupling graph.

Make the physical homing regression submit full joint targets, matching actual
homing plan payloads. Rebuilt native simkit and reran its ctest: passed, including
1 mm placement, first-side hold, calibration and true-zero return within 20 um.
Removing backend kinematic constraints also exposes the old reliance of sparse
direct leader commands on physical propagation; preserve that behavior through
explicit target projection before considering the native change complete.

MotionKit compile found conditional nullable count typing; replace ternaries
with guarded casts to the known integer type. The compiler pass still needs
retry, other kits/app and focused picker/Haxe homing runs remain pending.

### G12 checkpoint — sparse targets preserved, native ctest green

Kinematic follower fix: `5a9636faf`. Retain skipped single-input kinematic
coupling terms on the simulation endpoint. When a direct command omits a
follower, project its position/velocity through those terms, including chains,
before applying follower slip/origin/alignment. Explicit follower targets remain
authoritative. Full payloads avoid the expansion allocation. Reject unsupported
effort projection and nonfinite results. Multi-input coupling groups retain
physical constraints along with servo-connected groups.

Extend the physical 1 mm regression to run full and sparse target variants.
Native rebuild passed. Full enabled ctest rerun passed 18/18 in 9.13 s, including
both variants, device protocol Rust, virtual endpoints and serial PTY tests.
This establishes the native checkpoint for this source state. MotionKit
compiler-only retry remains running; other kits/app and focused picker/Haxe
homing execution remain pending before G13.

### G12 checkpoint — nullable latch interface and compile batch

Sparse native projection: `5acc78e6a`, native ctest remains 18/18 green.
MotionKit compile failed at HomingDriver.latch argument 3: an optional Float
parameter did not accept Null<Float> under the pinned compiler. Declare the
driver interface/implementations with an explicit required Null<Float> argument
(all call sites pass it), and retain explicit nullable defaults on standalone
RobotRuntime and JointReferenceState methods. Retry MotionKit compiler-only.

Start a serial compiler-only batch for the available top-level kit test
entrypoints, additional established nested kit entrypoints, app and app
project-source. Record individual logs/statuses in external scratch. This batch
does not run tests; focused picker and homing execution remains pending. CadKit
uses its separate compiler entrypoint and still needs a compiler-only invocation.
G13 remains unstarted until the checkpoint succeeds.

### G12 checkpoint — endpoint snapshot FFI direction and CadKit compile

Nullable latch correction: `b283c8bd8`. MotionKit compile reached
NativeRuntimeEndpoint.observeEndpoint and failed because snapshot_endpoint
lacked the INOUT annotation used by snapshot_full. Annotate its borrowed
snapshot correctly and regenerate runtime and simulation canonical FFI sources
for four portable ABI targets. Native ABI layout/signature is unchanged; the
Haxe projection now includes the intended status/result shape. Retry MotionKit
compiler-only with the corrected declaration.

CadKit HaxeonSmoke compilation through the compiler-only portion of its script
passed (exit 0); no CadKit test execution. AnimKit and AutomationKit batch
compiler checks passed. Remaining kit/app batch is live. Focused picker is
PROJECT_SOURCE_ONLY=gantry in app.ProjectSourceTests; Haxe homing selector is
MOTIONKIT_HOMING_ONLY=1. Neither focused test has run yet.

### G12 checkpoint — MotionKit compile and focused homing passed

Snapshot annotation: `149af187e`. MotionKit compiler-only retry finished
exit 0, compiling 1119 sources. Execute the resulting module directly with
MOTIONKIT_HOMING_ONLY=1 and native library paths prioritizing the newly built
RobotKit runtime: exit 0. Reported checks cover side hold order, compensated
capture, stale closing-edge rejection and cancellation cleanup; no full
MotionKit suite ran. Native physical homing remains covered by 18/18 ctest.

The serial batch now records AnimKit, AutomationKit, CamKit and CncKit compile
exit 0; CadKit previously passed separately. Start app project-source compiler
preflight to its dedicated output for the focused picker run while remaining
kit compiles continue. The picker has not run and the full compile checkpoint
is still incomplete. G13 remains unstarted.

### G12 checkpoint — app startup-offset typing and clearance metadata

The compiler-only batch passed HumanKit, KinematicsKit, ManufacturingKit and
ProcessKit in addition to the earlier successes. MachineKit failed because
GantryClearanceChecks read transmissions and belt paths directly from the
assembly description rather than its machine metadata. Corrected that access.
The app project-source compiler also required explicit nullable array types
for the two optional startup-offset lists; added those declarations. Both
failed compiles are being retried. These changes do not alter native code or
homing sequencing, so the green native ctest and focused homing run stand.
The picker and complete compiler checkpoint remain pending; G13 is unstarted.

### G12 checkpoint — picker compilation follow-up

The app project-source compiler passed after explicit offset-array typing.
MachineKit's retry found a missing import for the secondary GantryPickerChecks
type; imported it from GantryPickerPreview. The first focused picker attempt
stopped during generated project compilation because its scene conflict check
assigned an empty object to the machining schema. Replaced that fixture with
a structurally complete machining object; the simultaneous-mission rejection
remains the assertion. Retried the failed picker, with the newly built native
RobotKit runtime first on the library path. ProjectKit and RobotKit compiler
checks passed. Remaining compiler checks and picker execution are pending.

### G12 checkpoint — generated picker assembly corrections

The focused picker reached actual assembly construction and exposed three
errors hidden by compiler-only checks. The Z belt idler was 40 mm below the
column end, leaving no common supporting plane; belt endpoints now span the
column length, with their clamp on the upper part of the Z carriage. Switch
registration now checks trigger connectors on the assembly member definition
(including added member connectors), rather than only its component. Finally,
the belt clamp connector names the lower belt face used by the planar path,
while its jaw remains centred across the belt width. The picker is being
retried against these corrections. Native runtime and homing tests remain
unchanged. The compiler batch has passed StockKit, ToolpathKit and VisionKit.

### G12 checkpoint — picker reaches simulation, collision unresolved

The corrected picker generates a 1,992,238-byte scene with 127 component
records and enters MuJoCo simulation. Its strict contact check fails at
0.01 seconds, before completing a mission step. This is unresolved and must
not be treated as a successful picker or G12 checkpoint. Added explicit
contact indices, penetration and world position to the assertion (the pinned
compiler's anonymous-object string only listed field types). The app compiler
is being rerun for that diagnostic. CadBridge, MotionKit weave and the welding
recipe compiler-only checks passed; app root compilation is underway.

### G12 checkpoint — compiler pass complete, intended carton supports

All 21 batch manifests completed: 15 top-level kit manifests, four nested
manifest checks, and both app entries. The sole initial MachineKit failure
passed its final retry after the attachment fixes. Separate MotionKit and
CadKit compilers also passed, completing the requested per-kit/app compiler
checkpoint. Native build/18-of-18 ctest and focused homing remain green.

The picker diagnostic identified a fixed-root contact with carton body 1 at
(0.185, 0.115, 0.20951) metres, penetration 0.981 mm: the carton settling on
its authored infeed pad, rather than a moving gantry collision. Its assertion
now allows shallow (at most 4 mm), vertical support only for each carton's
own infeed/target pad footprint and height. Other fixed-root contacts, moving
link contacts and deeper penetration still fail. Recompiling the app test
entry for the justified picker retry. G12 is not verified and G13 is unstarted
until the picker completes.

### G12 checkpoint — switch and joint source timestamps

The carton-support assertion compiled and the picker advanced into homing,
where switchZhome failed freshness validation. Simulation publishes joint
snapshots using a floating-point session-time conversion, but the Haxe step
observer had stamped switches from its separately accumulated integer clock.
A rounding difference can place a switch in the future relative to its joint
observation, which the strict validator correctly rejects. Switch publication
now uses the timestamp of the same runtime joint snapshot used for its counter
conversion. Sequence checks and clock compatibility remain strict. Recompiling
the app entry for a picker retry; this is not yet a verified homing integration.

### G12 checkpoint — wait for endpoint calibration readiness

The timestamp correction compiled and the picker passed switch freshness,
then reached native latch calibration, which rejected the endpoint as busy
(status -2). The cycle's selected-axis speed budget did not guarantee native
calibration readiness: the whole endpoint must finish stopping, drain its
trajectory and have every joint below the native 1e-6 velocity threshold.
RuntimeHomingObserver now reports that readiness from the same snapshot;
StopAfterLatch waits for it before releasing holds and calibrating. The focused
homing fixture now checks that both holds remain while readiness is false.
App and MotionKit compiler retries are running. A focused homing rerun is
justified by this controller change; the unchanged native tests need no repeat.

### G12 checkpoint — bounded terminal clock rounding

MotionKit and app compilers passed the endpoint-readiness change; the focused
homing regression passed again with the new wait-before-release assertion.
The picker then reached Z's return plan, rejected with status -1. Native
debugging confirmed all start states and polynomial coupling coefficients
were consistent. Its terminal acceleration was 2.420116e-6 m/s² after
nanosecond phase rounding (and 0.000380151 rad/s² on belt followers), exceeding
the unscaled/scaled 1e-6 rest threshold despite essentially zero terminal speed.

Native terminal admission now accepts at most half a nanosecond of terminal
jerk as acceleration rounding, capped by 1e-6 of the physical acceleration
scale, alongside the existing rest tolerance. Terminal speed remains strict.
The new runtime regression accepts a 4e-6 acceleration gap for 10,000-unit
jerk and rejects 1e-4. Its first fixture accidentally failed start-anchor
validation (-2); the fixture now explicitly budgets its initial acceleration
to isolate the terminal check. Final native build passed. The full ctest run
passed the other 17 tests; the corrected runtime test passed its targeted retry
(1/1), restoring green results for all 18 without repeating the unchanged
17 tests. The picker is being retried against this runtime. G13 is unstarted.
