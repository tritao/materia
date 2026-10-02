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

### X3 — Motors as actuators, limits derived (done)

- **Motor data.** `NemaStepper.ratingCatalog()` gives the three named
  variants their holding torque, rated current, phase inductance, rotor
  inertia and step angle. Torque and current match the product names. The
  inductance and rotor inertia are typical datasheet values I supplied
  from memory, marked unverified. `pullOutTorque(speed, volts)` is a
  first-order model: the holding torque until the winding's reactance at
  rated current takes the supply (V / (50 × L × I) for a 1.8° motor), then
  falling as 1 / speed. `usableTorque(margin)` and `usableSpeed(volts,
  margin)` give the torque the motor can be relied on for (half its holding
  torque by default) and the speed it holds it to.
- **Screw efficiency.** `LeadScrewThread.efficiency(friction = 0.1)` is
  tan λ / tan(λ + φ') on the 15° flanks: 0.40 for Tr10 × 2. Drives put it on
  their coupling (typical values for gears, racks and belts), and couplings
  carry an optional `efficiency` through the assembly format, CadKit's
  `AssemblyModel.couple`, the bridge and RobotKit's `JointCoupling`.
- **Actuators.** `MachineAssembly.addMotor(id, joint, motor, volts, margin)`
  records a `MotorRecord` (saved and rebuilt like drives) and puts an
  `AssemblyActuator` {joint, maxEffort, maxRate, rotorInertia} in the
  assembly format. The bridge makes it a RobotKit `Actuator` on that joint
  (unit-converted and rebased to the initial placement) and adds the rotor
  inertia to the joint's `armature`, which MuJoCo simulates.
- **Derived limits.** `RobotModel.coupledLimits` caps a joint at each
  actuator's rate through the ratios. For a sliding joint it caps
  acceleration at Σ η·T·|ratio·scale| over the carried mass plus Σ η·(link
  inertia about the axis + armature)·scale². `RobotRuntimeCompiler` gives
  the runtime blueprint these limits, and the CNC player plans with them.
  Gravity, friction and screw critical speed are left out.
- **Router.** Its axis specs keep only travel. Each motor drives its screw
  joint on a 24 V supply (`SUPPLY_VOLTS`). Derived: 43.7 mm/s on every axis
  (a NEMA 23 holds half its torque to 1310 rpm, and a 2 mm lead turns that
  into 43.7 mm/s), and 5.4 (Y, two motors), 5.6 (X) and 6.1 (Z) m/s². The
  rotors dominate, reflected through π rad/mm. The plate machines in
  184.9 s, against 196.3 s with the hand-written 80 mm/s and 0.3–0.5 m/s².
  Most moves are short, so acceleration matters more than top speed.
- **Checks.** A MachineKit test covers the torque model, the screw's 40%,
  and an actuator following a motor swapped in a description. The RobotKit
  test covers velocity and acceleration from one and from two motors. The
  app test checks each router axis's derived speed against the motor's
  numbers and prints the limits.
- **Left for later.** Simulation still puts a servo on every joint;
  actuating only the motor joints needs the runtime to command the
  actuators. Device deployments don't yet take steps per unit from the
  motor, its step angle, the microstepping and the ratio. Gravity on Z,
  friction and screw critical speed are not modelled.

### X4 — Belts (done)

- **`TimingBelt`** (`machinekit.transmission`): a closed loop round `BeltWrap`s
  (pitch-circle centre, pitch radius and which way the belt wraps: +1
  counter-clockwise, -1 clockwise, as an idler on the belt's back does) given in
  the belt's plane. The path is the oriented tangents between consecutive wraps
  (also the crossed ones) and arcs on them. It reports the pitch length, the
  nearest whole tooth count, the slack of that count (`slack()`) and the centre
  adjustment that makes the loop whole teeth (`centreAdjustment()`: half the
  slack, for two parallel runs with one pulley moved along them). `strands()`
  and `pointAt(distance)` give the path, so a carriage can clamp a strand, and
  `rotation(wrap, strand, travel)` gives which way a wrap turns while a carriage
  on that strand moves. It chose whole teeth rather than a standard length
  table: the router places its idler so the loop is whole teeth (20-tooth GT2
  pulleys make a loop 20 teeth plus the centre distance in mm).
  Geometry is the band only: `width` × `thickness` (1.38 mm for GT2, per profile)
  centred on the pitch line, as a polygon with 7.5° arc steps. Teeth are omitted,
  because they would change no clearance and cost an OCCT sweep.
  A two-pulley belt has a recipe (`machinekit.transmission.timing-belt`:
  profile, driver and idler teeth, centre distance, width); other layouts are
  code-only. The idler is a `TimingPulley` (toothed idlers are common).
- **Belt couplings** are the X2 `Drive.Belt(pulley, alignment)`: ratio =
  alignment × 2 / pitch diameter, efficiency 0.97. The router takes the alignment
  from the belt's `rotation`, and the smoke checks it from first principles (the
  pulley's point on the clamped strand moves along the axis).
- **Router.** `new CncRouter(belts = true)`, defaults stay screws. X: motor on a
  plate bolted to the back of the gantry beams, shaft pointing forward into the
  beam gap with a 20-tooth GT2 pulley, an idler on an axle plate at the other end
  (centres ±222 mm, belt in the vertical plane, 464 teeth, 928 mm); the carriage
  bracket clamps the lower strand and the upper passes above it. Y: one belt per
  side in the vertical plane outside the frame (centres ±320 mm, 660 teeth,
  1320 mm), motors on plates behind, idlers on axle plates in front; the gantry
  bracket has a slot for the upper strand and clamps the lower. Z keeps its screw.
  The pulleys and idlers turn on continuous joints coupled to their axes; the
  motors stay actuators on the pulley joints, so limits are derived.
  Bodies: pulleys and idlers add 6 joints and couplings.
- **Numbers** (24 V NEMA 23, half holding torque): pitch radius 6.366 mm, so
  every belt axis gets 873.1 mm/s (a screw axis gets 43.7), X 15.1 m/s², Y (two
  motors) 12.8 m/s², Z unchanged. Those are limits, not feeds: a machining run
  of the belt router was not done (the motion limits are 20 times the screw
  router's, so the plate would machine far faster than the screws, not
  comparable).
- **Checks.** MachineKit smoke: belt maths (two-pulley length and teeth, slack
  and adjustment, a triangle of pulleys, path wrap-around, geometry bounds,
  recipe round trip) and the belt router: the nose still lands at machine
  coordinates, every pulley turns travel / pitch radius (and the right way), belt
  lengths are whole teeth, belts clear the frame, each bracket overlaps its belt
  by exactly one strand's cross-section (6 × 1.38 × 40 mm³), and motors, plates,
  idlers clear the gantry at the ends of travel. App test
  (`checkBeltRouter`, project `machinekit/examples/cnc-router/belts`): couplings
  1/6.366 mm for X and Y, derived speeds from the pulleys.
- **Left.** The belt is frame-fixed geometry; showing teeth travelling (a mesh
  updated per frame without OCCT) is not done. Belt stretch and a machining run
  of the belt variant are left.

### X5 — Couplings with more than one leader (CoreXY)

- `follower = Σ ratioᵢ × leaderᵢ + offset` across the assembly, KinematicsKit,
  the RobotKit model and blueprint (ABI change), validation, the device
  compiler, and SimKit. MuJoCo uses a fixed tendon with a tendon equality;
  the core backend projects the sum.
- A CoreXY gantry example, such as a plotter or a laser.
- **Gate:** the CoreXY machine runs a program in simulation and on the
  virtual device, and moving one motor moves the head diagonally.

### Device steps and identity (done)

A real stepper board used to get no layout from the serial path: an identity
map at a hard-coded 1000 steps per unit, and a layout fingerprint compiled into
firmware, so any configuration change needed a reflash. Each fact now lives in
one place.

- **The model owns the motor.** `Actuator.fullStepsPerRevolution` (zero for a
  non-stepper) comes from the stepper's rating (360 / step angle) through
  `MachineAssembly.addMotor`, the assembly format and the RobotKit bridge. A
  stepper's actuator coordinate is the rotor angle in radians.
- **The deployment owns the wiring.** `DeviceLayout` channels name an
  `actuator`, a `direction` (1 or -1), the driver's `microsteps`, and
  optionally `direction_setup_ticks` and `skew_bound`. Microstepping lives
  here, not in the model or the firmware: it is a driver setting.
- **The board owns only board facts**: its channel count, pins, tick rates and
  unique id (on an STM32G4, the 96-bit unique-ID register). Firmware is per
  board type, not per machine.
- **`DeviceBinding.bind(model, layout, stepTickHz)` joins them.** Per channel:
  joint, ratio (signed by direction) and offset from the transmission, steps
  per radian = full steps × microsteps / 2π, and a rate ceiling of
  min(actuator rate, step tick / steps per radian). It fails loudly on a
  stepper with no channel, a channel with no actuator or a non-stepper, or two
  channels on one actuator. It also returns the model with those ceilings, which
  `coupledLimits`, the runtime compiler and the CNC player plan on, so the
  planner's limits are the device's real ceiling.
- **Identity and agreement are checked while the session opens (RKD6 v12).**
  The session names the controller it is for; the board refuses another id but
  reports its own, so `robotd identify` can read it. The board acknowledges a
  64-bit FNV-1a digest of the configuration it received, which the host
  compares with its own, along with the channel count and the step tick.

### X6 — Drive kinds, plan checks and drive-aware simulation

Steppers and servos fail differently, so an actuator names its drive kind
instead of carrying bare numbers. Drive-level behaviour only: no current
loops, PWM or thermal mass.

- **Drive kinds.**
  - `Stepper`: full steps per revolution, rotor inertia, holding torque and
    a pull-out torque–speed curve. Microstepping stays in the deployment's
    wiring.
  - `Servo`: rated and peak torque, rated and maximum speed, rotor inertia,
    encoder counts and optional default gains, usually behind a gearbox
    drive (ratio and efficiency).
  - Both share a torque–speed curve (a few points), so `coupledLimits`
    derives limits the same way for each.
- **Plan check (stall predictor).** Along each plan, the torque each motor
  needs is worked out from moving mass × acceleration through ratio and
  efficiency, plus rotor and screw inertia, gravity on vertical axes, and
  friction (nut drag, rail preload, a cutting-force allowance on feeds). It
  is then checked against the drive.
  - A stepper must stay under its pull-out curve, with margin, at every
    point.
  - A servo must stay under its peak torque, and its average (RMS) torque
    over the move under its continuous rating.
  - The check runs once per plan, the same for simulation and device.
- **Accuracy check.** Along the same plan, the tool's worst deviation from
  its path is checked against the machining tolerance:
  - belt stretch under acceleration (force ÷ belt stiffness, from belt
    width and free strand length);
  - a backlash allowance per screw nut.
  This flags corners where a belt machine will round or lag. Compliance is
  checked here, not simulated as springs, for the same reasons as steppers.
- **Screw critical speed.** A lead-screw drive caps its screw joint at the
  first bending speed, from root diameter, unsupported length and end supports
  (where its bearings sit), with margin. The router's Y screws, about 600 mm
  with no far-end bearing, may whip near 700 rpm, about half their motor's
  usable speed.
- **Simulation by drive kind.**
  - A stepper's joint follows its plan kinematically, as now. When the plan
    check fails, it slips and keeps the error, as a real stepper loses steps.
  - A servo gets a torque-limited servo on the motor joint only, with its
    gains, and the coupling moves the rest. Use this for arms, wheels,
    humanoids and the exosuit.
  - Not force-driven simulation for steppers: it has the wrong failure mode
    (lag instead of lost steps), is numerically stiff (hundreds of kg of
    reflected inertia through a constraint), and needs gains tuned per
    machine.
- **Gate.** The belt router machines the motor plate under these limits:
  the step-rate cap, the plan check and the accuracy check. Its cycle time
  and accuracy are compared with the screw router's on the same part.
- **Examples.** The robot arm (servos behind gearbox drives) and the mobile
  base (wheel motors) take their limits from their drives instead of typed-in
  numbers.
- **Device.** Step edges are scheduled with hardware timers (STM32G4
  output compare and DMA) instead of a 40 kHz software tick, which caps
  16-microstep NEMA 23s near 25 mm/s and quantises step intervals into
  velocity ripple. A Trinamic driver profile in the wiring covers model
  (TMC5160/2160 for NEMA 23, TMC2209 for NEMA 17), current, chopper mode
  and StallGuard. Motion still uses step/dir; SPI/UART carries
  configuration, diagnostics and stall reports. The drivers' own ramp
  generators stay unused, because paths are planned and coordinated by
  MotionKit. Servo drives on a fieldbus wait for real hardware.

### Later

Belt teeth drawn and moving with the belt: a mesh built in Haxe and
updated per frame, shifted by the coupled joint's travel. It doubles as a
visual check on a drive's sign and ratio. Also gearboxes as components, an
editor UI to author couplings and drives, and differentials.
