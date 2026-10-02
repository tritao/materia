# Transmissions plan

Goal: machines drive their axes the way real ones do. Motors turn screws,
pulleys and pinions; carriages follow at the ratio the parts give; and axis
speed and acceleration limits come from the motor and the drive instead of
being written by hand. Lead screws, belts, rack and pinion and gears are one
concept, a linear coupling between joints. CoreXY-style drives are the same
concept with more than one leader.

## What exists (X0 survey, 2026-10-02)

One coupling, `follower = leader × ratio + offset`, already runs end to end:

- **Assembly** (`projectkit` `AssemblyJointCoupling {id, source, target,
  ratio, offset}`): validated (non-zero ratio, one source per target, no
  cycles, a target is never `driven`), flattened through nesting, and
  propagated by `AssemblyState`. CadKit authors it with
  `AssemblyModel.couple` and persists it as a `cadkit.coupling` relationship
  between occurrences. Mate solving leaves coupled joints alone.
- **KinematicsKit**: coupled joints share their source's degree of freedom
  (scaled Jacobian columns, intersected ranges).
- **RobotKit**: `JointCoupling {leader, follower, ratio, offset}` in
  `RobotModel`, the runtime blueprint (`rk_robot_joint_coupling`, up to 512),
  validation of plans, segments and commands (a consistency check, nothing
  solved), the RKD6 device compiler, the CadKit bridge (mm→m rescaling) and
  URDF `<mimic>`. Actuators reach joints through `SimpleTransmission`
  (`joint = offset + actuator / ratio`); devices carry `steps_per_unit`.
- **SimKit**: the core backend enforces couplings kinematically; MuJoCo uses
  a joint equality (`mjEQ_JOINT`, polycoef offset and ratio).
- **MotionKit**: `MotionSystemBlueprint` maps axes to scaled joints;
  `ProgramCompiler` projects follower coefficients after timing.
- **MachineKit**: `LinearAxis` couples its carriage to a turning screw through
  `LeadScrewTransmission`; `LeadScrewNut`/`LeadScrewThread` give a signed
  lead. `GearPair`, `Rack`, `SpurGear`, `TimingPulley` and `Sprocket` carry
  pitch data but are geometry only.

What is missing:

- The CNC router models no drive train. Its four NEMA 23 motors and Tr10×2
  screws are fixed parts, and its axis limits are hand-written `specs`.
- `CncProgramPlayer`/`PlanExecutor` plan over a three-joint chain and hold every
  other joint still. With coupled screws, the plans would fail segment validation.
- `ToolpathMotionBinding` scales a follower's limits from the axis but never
  lets a follower's own limit (screw or motor rpm) cap the axis.
- MuJoCo gives every non-fixed joint its own motor and servo, coupled
  followers included.
- Coupling ratios are bare numbers. Nothing derives them from a screw's lead,
  a pulley's teeth or a gear pair. `MachineKitRobotCompiler` expresses a lead
  screw two ways (a coupling, or a transmission on the prismatic joint).
- CadKit documents drop `overtravel` and `acceleration` limits on save.
- No motor data beyond dimensions: no torque, speed or rotor inertia.
- Couplings have one leader, so CoreXY and differentials cannot be expressed.
  No belt component exists.
- Naming differs between layers (`source/target` vs `leader/follower`;
  KinematicsKit's `couple(target, source)` order).

## Design decisions

- **One primitive.** A transmission is a linear coupling between joints. No
  helical joint: a screw turns in its bearings (revolute), the nut's carriage
  slides (prismatic), and a screw coupling ties them. Mates that leave a
  screw motion stay rejected.
- **The leader is the planned coordinate, not the cause.** A coupling is a
  holonomic constraint, so either end can lead. Machine axes (the carriages)
  lead, and screws, pulleys and motor rotors follow. That is what toolpaths
  are planned in, and it lets one axis drive two screws (the router's dual
  Y). Actuators sit on the motor joints and reach the axis through the
  coupling, which the device compiler already handles.
- **Ratios come from parts.** A coupling records its kind and the parts that
  define it: screw (lead and hand of the screw), gear pair (teeth), rack and
  pinion (module and teeth), belt (pulley pitch radius). The ratio is derived
  from those parts, so it is stored in one place. A plain ratio stays for
  imported `<mimic>` joints.
- **Every coupled joint's limits bind the axis.** Axis velocity is the minimum
  of its own limit and each follower's limit divided by |scale|. Motor joints
  get their limits from the motor catalogue.
- **Only declared actuators are actuated** in simulation, not every joint.

## Milestones

### X1 — The router turns its screws (done)

Kinematics through the whole stack, with ratios still given as numbers.

- **Router.** Each motor turns its Tr10 × 2 screw through a standard shaft
  coupling (`ShaftCoupling`, 6.35 to 10 mm). The coupling and screw ride a
  continuous joint `<screw>-turn` on the motor's shaft line, and coupling
  `<screw>-lead` ties it to x, y or z at 2π·(axis · screw direction) / signed
  lead: π rad per mm here. Dual Y means two screws follow `y`. The motor mounts
  (Y plates, right gantry upright, Z bracket) gained NEMA 23 pilot and bolt
  holes (`RouterPlate` takes a motor and where its face sits), so the coupling
  turns inside the pilot bore. The Z screw runs in a 13 mm gap between the X
  and Z plates, too narrow for its coupling, so the Z motor stands on four
  37 mm `Standoff`s above its bracket, as many real Z axes do. The router went
  from 4 rigid bodies to 8 (4 screw bodies) and from 3 joints to 7.
- **Plans stay in axis coordinates; the runtime turns the screws.** A plan
  over a subset of joints used to hold the others still. Now a joint no
  planned joint drives follows its coupling's leader, in topological order.
  This happens in the native `copy_segments` (from the blueprint's couplings)
  and in the Haxe `SegmentArrays` (from `RobotDescription.couplings`, filled by
  `RuntimeRobotAdapter` from `RobotRuntime.couplings`) for robots that need
  materialised segments. So `PlanExecutor` plans only the three axes, as
  before, and the plans pass `validate_segments_for_blueprint`. A chunk's
  declared start state and tolerances follow the same rule
  (`SegmentArrays.robotValues` / `robotTolerances`). A screw's tolerance is
  its axis's times |ratio|, because rounding slack that suits millimetres is
  3142 times too tight for the screw's radians.
- **Coupled joints without limits.** The runtime gives a follower with no
  velocity or acceleration limit of its own its leader's, scaled by |ratio|,
  when it takes the blueprint. Without that, a hold was refused (every joint
  needs an acceleration limit) and a path-following stop could not brake.
- **Limits.** `RobotModel.coupledLimits(id)` tightens a joint's velocity and
  acceleration by every joint coupled to it, scaled back through the ratios.
  `CncProgramPlayer` builds its axes and planning chain from it. The screws
  have no limit of their own yet (X3 gives them the motor's). Cost: 0.43 ms
  and 61 KB a simulated tick, against 0.39 ms and 56.5 KB, from four more
  bodies and joints. Machining is unchanged: 196.3 s, same removed volume.
- **SimKit/MuJoCo.** Followers keep their servo, which gets the same coupled
  target, so it agrees with the joint equality. Actuating only the motor
  joints belongs with X3, when actuators sit on them. The `qpos0` concern
  was unfounded: the bridge measures every joint from its initial placement
  and rebases the coupling offset to it.
- CadKit documents keep `overtravel` and `acceleration`.
- **Checks.** The MachineKit smoke checks that every screw turns π rad per
  mm on all six travel corners, and that couplings, motors and standoffs clear
  their mounts. The app router test checks the 8 bodies and 7 joints, the four
  couplings (3142 rad/m each), and that the simulated X and Z couplings have
  turned π rad per mm of axis travel every 1000 ticks. Native C API test: a
  plan leaving a coupled joint out is accepted. It was rejected before.
- **Found on main while testing.** `ProgramTests.testProgramPlanner` used the
  pre-K6a `Manipulator(model, chain)` constructor. The RobotKit runtime's
  copy of a device-executed queue now follows the device's path time
  (`cb516ab4`), so it ran out a tick or two after the device reported the
  queue finished. A plan submitted in between joined a finished device
  queue and never ran (virtual-device CNC test). The runtime now publishes
  a device's queue as active until its copy has run out too, while the copy
  still follows the device's own report.

### X2 — Ratios from parts (done)

- **Drives.** `machinekit.assembly.Drive` names the parts that set a
  coupling's ratio: `LeadScrew(screw, alignment)` (2π·alignment / signed
  lead), `GearMesh(driver, driven, alignment)` (−alignment·teeth ratio),
  `RackAndPinion(pinion, alignment)` and `Belt(pulley, alignment)` (alignment
  / pitch radius, from a `SpurGear`, `TimingPulley` or `Sprocket`).
  `MachineAssembly.addDrive(id, leader, follower, drive, leaderZero)` adds
  the coupling with the ratio worked out from the members and returns it. The
  zero point is in leader units, so the offset follows the ratio.
- **Saved and rebuilt.** A drive is MachineKit data: a `DriveRecord` in the
  machine-side record (descriptions and CadKit documents), carried through
  `include`, `rebuildIncluded` and `copyInto`. `fromDescription`, and so
  `MachineAssemblyDocuments.rebuildAssembly`, works each driven ratio out
  again from the parts' current values. Editing a screw's pitch in a document
  therefore changes its coupling. Before, the saved number was copied through.
  A drive whose coupling was removed goes with it.
- **Users.** `CncRouter` and `LinearAxis` add their screws as lead-screw
  drives.
- **Decisions.**
  - CadKit and the assembly format keep couplings as plain numbers: they
    don't know what a pulley is, and MachineKit is the layer that does.
  - `MachineKitRobotCompiler`'s two lead-screw paths stay. One is the
    abstract model (the carriage with an actuator transmission, as URDF
    does). The other is the physical one (a turning screw joint). Both take
    the lead from the part.
  - The layers' names (`source/target` in the assembly format,
    `leader/follower` in RobotKit) stay. Renaming wire fields would break
    saved files for no behaviour.
- **Checks.** A MachineKit test covers a screw, a GT2 pulley and a gear mesh:
  their ratios, the offset from the zero point, and an edited pitch through a
  saved description and through a document. The router smoke checks its four
  screws turn through lead-screw drives.

### X3 — Motors as actuators, limits derived

- Motor catalogue data for NEMA 17/23: holding torque, a simple torque-speed
  (pull-out) curve, rotor inertia and steps per revolution.
- An `Actuator` comes from a motor part: max rate, effort and steps per
  revolution. Device deployments get steps per unit from the motor, the
  microstepping and the ratio.
- Axis limits come from the drive: velocity from motor speed × travel per
  radian (capped by screw critical speed later). Acceleration from motor
  torque through the ratio, against the moving mass (CAD mass properties)
  plus reflected rotor and screw inertia.
- The router's hand-written `specs` go away.
- Simulation actuates the joints that carry actuators (the motors) and lets
  couplings move the rest, instead of a servo on every joint.
- **Gate:** the router's derived limits are close to today's hand-written
  ones, documented, and the plate still machines.

### X4 — Belts

- `TimingBelt`: a closed loop around pulleys and idlers. Its length and tooth
  count come from the pitch, and one strand clamps to a carriage.
- Belt couplings (pulley joint → carriage) derived from the pulley.
- Rendering: the belt moves along its loop, and the pulleys and screws turn.
- A machine that uses them. Either the router gets a belt-driven option on
  X/Y, or a small belt gantry is added.
- **Gate:** a belt axis machines a program; the belt's teeth move at carriage
  speed.

### X5 — Couplings with more than one leader (CoreXY)

- `follower = Σ ratioᵢ × leaderᵢ + offset` across the assembly, KinematicsKit,
  the RobotKit model and blueprint (ABI change), validation, the device
  compiler, and SimKit. MuJoCo uses a fixed tendon with a tendon equality;
  the core backend projects the sum.
- A CoreXY gantry example, such as a plotter or a laser.
- **Gate:** the CoreXY machine runs a program in simulation and on the
  virtual device, and moving one motor moves the head diagonally.

### Later

Belt stretch and screw backlash in simulation (following error), screw
critical speed and efficiency (back-driving a vertical Z), gearboxes as
components, an editor UI to author couplings, and differentials.
