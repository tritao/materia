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
  friction and screw critical speed are not modelled here (X6b models them).

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
  of the belt variant are left (done in X6).

### X5 — Couplings with more than one leader (CoreXY) (done)

`follower = Σ ratioᵢ × leaderᵢ + Σ offsetᵢ` runs through every layer, and a CoreXY pen plotter
(`machinekit/examples/corexy`) uses it. Decisions that differ from the sketch above:

- **A coupling is one term, not a list of terms.** A follower with several couplings is the sum of
  them. `AssemblyJointCoupling`, `JointCoupling`, the blueprint's `rk_robot_joint_coupling` and the
  SimKit coupling keep their layout, so every single-leader file, document and blueprint reads and
  writes byte for byte as before (the common case), and each term keeps its own efficiency, stiffness,
  backlash and drag. The ABI change is in meaning: `RK_API_VERSION` is 24 (a follower may have several
  couplings), the `.hxi` regenerated with `runtime/tools/check-hxi.sh`. Validation, everywhere: a
  leader appears once per follower, no cycle through any chain of terms, finite non-zero ratios, and a
  follower is still never `driven`. The old "one coupling per follower" rule is gone.
- **Assembly, CadKit.** `AssemblyDefinitionCodec` checks pairs and cycles and sums the terms for a
  saved state's consistency; `AssemblyState` recomputes each follower a changed joint reaches, sources
  first; documents keep one `cadkit.coupling` relationship per term; the bridge subtracts a follower's
  placement once, in its first term.
- **KinematicsKit** gives every joint its affine map of the DOFs (`jointTermStart/Dof/Scale`,
  `jointConstant`). A single-leader chain keeps `jointSource`, ratio and offset; a joint that sums
  several is marked `COMBINED` (-2) and has one Jacobian column per term. Its limits bound a sum, not
  a box, so they are not folded into the DOF ranges. The native kinematics (manipulator servoing)
  refuses such a joint: nothing there uses one.
- **Runtime.** `copy_segments` derives an undriven follower's sum in dependency order;
  `coupled_values`, segment and plan validation, the device compiler (which checks the authored
  follower against the sum and rewrites it) and the follower's default limits (the sum of
  |ratio| × the leaders' limits, once every leader has one) all take the sum. A plan's rest tolerance
  at its end is scaled for a coupled joint like its start tolerances are (`Σ|ratio| ×` the leaders'),
  since a motor turning 314 rad per metre amplifies the planner's 1e-6 residual.
- **SimKit.** The core backend enforces the sum in dependency order and refuses a repeated pair or a
  cycle. MuJoCo keeps `mjEQ_JOINT` for one leader; for several it adds a fixed tendon
  (`Σ ratioᵢ·qᵢ − q_follower`, wrapped with `mjs_wrapJoint`) and an `mjEQ_TENDON` holding it at
  `−Σ offsetᵢ`. Like the joint equality it is measured from the joints' reference pose.
- **Limits are boxes.** A follower with several leaders limits each leader by its limit over the sum
  of the ratios' sizes (through chains, the L1 weight of the follower), so every combination of
  axis speeds within the per-axis limits stays inside the motor's. A motor's force splits between the
  axes it serves in proportion to the ratios (a half each for CoreXY), so each axis's acceleration
  limit is a motor's force over the mass, not two motors': both axes at their limits at once need
  exactly one motor's full force (`Fa = (Qx + Qy) / 2`). Moving along one axis alone could use twice
  that; the box does not.
- **Plan check.** Motors are counted once per actuator, not per axis: each sample adds what every axis
  asks of a motor (its share of the axis force through the ratio, rotor inertia times its
  acceleration) with one drag in the direction of its total speed, and the motor's speed is the sum
  of its ratios times the axes' speeds. Along x = y one motor of a CoreXY stands almost still and
  the other carries both axes.
- **MotionKit.** `MotionSystemBlueprint.fromRobotModel` maps no axis to a joint that sums several
  leaders; `ProgramCompiler` projects each follower as the sum, in order; `AxisKinematics` checks the
  sums; `StepperSlip` carries a slipped axis's offset to a follower as the sum.
- **The plotter** (`CoreXyPlotter`, 35 occurrences, 4 kg): two NEMA 17 on 24 V on the rear corners,
  GT2 20-tooth pulleys (pitch radius 6.366 mm), two belts of five pulleys each on two levels (belt A
  above B), closed through the carriage, whose clamp holds both gantry strands. Motor A turns
  `(x − y) / R` and motor B `(x + y) / R`; a front or rear frame idler follows its belt's motor
  (two couplings), a gantry end idler only `x` (one). 16 couplings, 10 pulley joints, two
  prismatic axes (`x` ±35 mm, `y` ±35 mm). The signs and the 1/R come from the belts' own
  geometry: the clamped strand's heading (`strands()[0].dx`) gives x, the strand from the gantry idler
  to the front corner (`strands()[1].dy`) gives y, and the wrap's side gives the sense.
  The belts are drawn at the home pose. The checks build each belt with the gantry moved and find its
  length unchanged and the feed at the motor pulley equal to the coupling.
- **Numbers** (24 V NEMA 17 17HS19-1684S1, half holding torque): the motors turn 204 rad/s at most
  and give 0.225 N m, so each axis runs to 649.6 mm/s (204 / (2 × 157.1 rad per metre)); with no
  steady loads x accelerates at 71.6 m/s² and y, which carries the gantry, at 34.0; with the plan check's
  default friction 58.0 and 27.5. On a controller (16 microsteps, 40 kHz step tick) each motor is
  capped at 78.5 rad/s and the axes at 250 mm/s. A 30 mm square planned at 95% of those limits needs no more than 96 and
  132 rad/s of the motors and passes the plan check.
- **Checks.** KinematicsKit (a combined joint's value and Jacobian, a cycle refused), CadKit
  (`AssemblyCouplingSmoke`: sums, validation, document round trip), RobotKit (a model with a summed
  follower: validation, FK, IK), native (`validation`, `c_api`, `device_compiler6`, `runtime`:
  validation of commands, plans and segments, derived followers, summed limits, a cycle refused),
  SimKit (core and MuJoCo: the sum and the tendon), MachineKit (the plotter's geometry, couplings and
  clearances, and its drives in the robot model), MotionKit (`CoreXyTests`: a square run on the
  deterministic backend, every pulley the sum of its terms at every tick, x alone turns the motors
  alike and y alone against each other, and the plan check's diagonal case), and the app (`checkCoreXyPlotter`:
  the project loads, the runtime blueprint carries 16 couplings and the MuJoCo world builds and holds).
- **Left.** The plotter has no program player or CNC job in the app (a pen plot is not a machining job);
  the gate "runs a program in simulation" is the MotionKit test, and "on the virtual device" is not done:
  the RKD6 board's channels map one motor to one joint, and a CoreXY device layout names the two motors
  (`DeviceBinding.bind` works on the actuators already). Belt stiffness is not set on the plotter's
  drives (a CoreXY axis sees two belts in series, which `carriageStiffness` does not model), so its
  accuracy check is rigid. The belts' idlers on the gantry and carriage do not model a moving strand's
  exact wrap, only its feed. KinematicsKit's native manipulator kinematics does not take a combined joint.

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

Status: X6a (drive kinds), X6b (plan check, accuracy, screw critical speed), the
belt-router gate, X6c (simulation by drive kind), X6d's simulation side (encoders) and
the examples are done (2026-10-02); the hardware-timer device work and the encoders'
device counting are not.

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
- **X6a, what was built.** `robotkit.model.ActuatorDrive` (module with
  `StepperDrive` and `ServoDrive`) and `TorqueSpeedCurve` (speed/torque points joined
  by lines, torque held below the first speed and zero above the last).
  `Actuator.drive` is null for a bare effort and rate. `Actuator.fullStepsPerRevolution`
  is now a property of the stepper drive (setting it makes a steps-only stepper), so the
  device binding and older callers are unchanged. `maxEffort`/`maxRate` stay what a
  planner may rely on: a stepper's usable share of holding torque and the speed it holds
  it to, a servo's peak torque and maximum speed (`planningEffort()`/`planningRate()`
  fall back to the drive's peak and maximum when a servo leaves them zero).
  `RobotModel.coupledLimits` reads those, so steppers derive the same numbers as
  before. A servo's gains stay on the actuator (`servoStiffness`, `servoDamping`).
  The assembly format's `AssemblyActuator` gained optional `drive`, `torqueSpeed`
  (alternating speed and torque), `holdingTorque`, `ratedTorque`, `peakTorque`,
  `ratedSpeed`, `maxSpeed`, `encoderCounts` and the gains; CadKit's
  `AssemblyModel.actuateDrive` takes a whole record; the bridge builds the drive.
  `RobotModelCodec` still writes `fullStepsPerRevolution` for every stepper and adds a
  `drive` object only for a stepper with ratings or a servo, so a model with a bare
  stepper keeps its bytes. X9 replaces earlier saved-model readers with one current schema.
  MachineKit: the `MotorDrive` interface (a part says its own actuator) is what
  `MachineAssembly.addMotor` calls. `NemaStepper` implements it, with a pull-out curve
  of the first-order model (`pullOutCurve`: points at 1, 1.1, 1.25 ... 8 times the
  corner speed and at the usable speed; joining them by lines overstates the
  hyperbola by at most about 1%). A servo motor part implements `MotorDrive` the same
  way; no catalogue servo exists yet and the test uses a code-only one.
- **X6b, plan check (stall predictor), what was built.**
  - **Where it runs.** `ProgramCompiler.finish` runs `PlanCheck` on every plan the
    compiler makes, once, and puts the result on `ExecutionPlan.checked`
    (`PlanCheckResult`: structured `PlanDiagnostic`s, the worst torque ratio and
    deviation). That is the one place outside tests that creates an
    `ExecutionPlan`, so every program, toolpath, CNC, handling and arm plan goes
    through it, for simulation and device alike; the check reads the plan's
    polynomial segments (sampled at most every 4 ms), not any runtime. It does not
    cover plans built straight from a `Trajectory` (`MotionSystem`'s direct paths,
    `ServoPlan`), which never become an `ExecutionPlan`; a check there would sit in
    `TrajectoryStream.motionSubmission` and is not done. The compiler's `planCheck` is
    null until a caller attaches one (`PlanCheck(model, jointIds, options)`, jointIds
    being the plan's joints in order); the CNC player attaches it.
    `ManipulatorMotion.checks` sums what the plans it started found.
  - **The model.** `robotkit.model.DriveLoads` finds each axis (a joint no coupling
    leads to) with actuators in its coupling chain: the mass it moves (carried mass
    plus armature, plus the turning inertia of coupled joints no motor drives,
    reflected through their couplings), its gravity force along the axis (from the
    joint frames, +Z up; a turning joint above it makes it worst case), and per
    motor: ratio, efficiency, rotor inertia, drag and its share of the force. Torque
    at a motor is `J·alpha + share·F/(ratio·eta) + drag` (eta flips when the load
    drives the motor back), with `F = M·a + gravity + friction + cutting`. At the
    planner's limit and no steady loads this is exactly the force balance
    `coupledLimits` solves.
  - **Steady loads are in the planner's limits too.**
    `RobotModel.coupledLimits(id, steady)` now takes a `SteadyLoads` and leaves less
    force for acceleration: the motors' drag, the axis's weight and rail friction come
    off. Without that, a plan made at the limits would exceed the pull-out curve near
    top speed by exactly those loads (the 0.5 holding-torque margin and the curve meet
    at the usable speed), and the check would flag every move. Passing none keeps the
    old numbers. The CNC player passes them.
  - **Stepper.** At every sample, `|T|` against `margin × curve(|omega|)`, margin 1 by
    default because the planner's 50% of holding torque is already the margin; beyond
    the curve's last speed nothing is available. One `StepperStall` finding per motor
    per plan, at the worst sample, with the samples over.
  - **Servo.** The same against the envelope's peak (`ServoPeakTorque`), and the
    time-weighted RMS torque over the plan against the rated torque
    (`ServoRatedTorque`).
  - **Assumed constants** (none is a datasheet value; each says so where it is
    defined). Rail running friction 5 N per sliding axis (`SteadyLoads.railDrag`; rail
    makers quote 0.002 to 0.005 of preload plus seal drag); lead-screw drag 0.02 N m
    (about 1.6% of a NEMA 23's holding torque, nut preload and bearings), belt pulley
    drag 0.005 N m (`DriveDefaults`, carried on the coupling as `drag`); cutting force
    0 N unless `PlanCheckOptions.cuttingForce` and `cuttingFeedLimit` say so (applies to
    moves programmed at or under that feed, in the sense that opposes the motion);
    lead-screw backlash 0.05 mm (an anti-backlash nut on a Tr10 x 2; plain nuts are
    0.1 to 0.3 mm); belt cord stiffness 2500 N per mm of width.
  - **Warn or reject?** Warn by default; `PlanCheckOptions.rejects` makes the compiler
    throw instead. A stall or an accuracy miss is a prediction from modelled friction
    and stiffness, not a hard limit like joint range, so it should not stop a machine
    that may run fine (a rapid with no load, a feed with no cutting); it should reach
    the operator, who decides. A device deployment that prefers a stopped job to a
    lost-steps job sets `rejects`.
- **X6b, accuracy check.** Couplings carry optional `stiffness` (force at the leader
  per unit of its travel), `backlash` (leader units) and `drag` (at the follower):
  assembly format, CadKit `couple`, the bridge (to SI), `RobotModel`/codec (written only
  when non-zero), `MachineAssembly` drive records (`setDriveStiffness`; defaults per
  drive kind in `DriveDefaults`). A belt's is `TimingBelt.carriageStiffness`: EA (2500
  N/mm of width, an assumption chosen on the soft side of what a ~400 N, 2.5%-elongation
  6 mm fibreglass GT2 belt allows) times the two free lengths in parallel, the clamped
  strand's length (carriage at its far end from the driver, the worst place) and the
  rest of the loop the other way round; pretension, tooth compliance and the clamp are
  left out. Parallel motors add stiffness, one rigid drive makes the axis rigid. Per
  plan the check reports the worst `|M·a ± friction|/k` plus the nut's backlash when
  the axis reverses (inside the plan, or against how the previous plan of the same
  compiler left it; a worker's check forks fresh), against `PlanCheckOptions.tolerance`
  (0.1 mm by default, an assumption for a hobby-class machine). Static sag from gravity
  is calibrated away and left out.
- **X6b, screw critical speed.** `LeadScrew.criticalSpeed(near, far, unsupported,
  margin 0.8)`: `lambda² · d_r / (4 L²) · sqrt(E/rho)` for a solid round section of root
  diameter `LeadScrewThread.rootDiameter()` (Tr10 x 2: 7.5 mm), `lambda` 1.875 for
  fixed-free, 3.927 fixed-simple, 4.730 fixed-fixed, pi simple-simple (`ScrewSupport`
  Free/Simple/Fixed; two free ends is an error). `MachineAssembly.supportScrew(drive,
  near, far, unsupported?)` records it with the drive and sets the screw joint's
  velocity limit, which `coupledLimits` carries to the axis through the ratio and a
  rebuilt assembly works out again from the part. The router holds each screw fixed at
  the motor (rigid coupling) and free at the far end over its whole length: nothing
  holds the far end and the nut floats, so it is not counted as a support. Results: Y
  (about 617 mm) 668 rpm, X 1037 rpm, Z (short) 9686 rpm, so Y is held to 22.3 mm/s,
  X to 34.6 and Z is not limited by it.
- **Simulation by drive kind (X6c, done).**
  - **Stepper slip.** Where the plan check finds every motor of a stepper axis over its pull-out
    curve (`ratio > 1` at a sample; all motors, because parallel motors on a rigid axis carry
    each other), the axis does not advance: it falls behind its command by what the plan commands
    over those samples, and keeps the error. `PlanCheck` records this as `PlanSlip` on
    `PlanCheckResult.slips` (times and the cumulative distance lost, signed along the motion, plus
    the total in full steps of the first motor). A plan the check passes has no slip, so passing
    plans cannot change. Resync after a stall is not modelled: the rotor stays where it fell
    behind, and the error is whole until reset (a real stepper would resync at some multiple of
    four full steps; the error is the same size).
  - **Where it runs, and why there.** Not in the plan (a slipped trajectory has velocity
    jumps the runtime's continuity checks refuse, and the runtime and any monitor would then
    see a plan that matches reality, hiding exactly what an encoder is for) and not in the runtime
    (it has no model of a machine). It is an offset on the *commands* in the simulation endpoint:
    `SimulationRobot` adds a per-joint `slip_` to every position or servo target
    (`rk_simulation_set_joint_slip`, `Simulation.setJointSlip`), so the runtime's setpoint stays the
    commanded one and the simulated joint is the real one. `motionkit.robot.StepperSlip` carries the
    plan check's findings out as the plan runs (`ManipulatorMotion.slip`, fed each plan when it starts
    and its elapsed time each update), puts the offset on the axis joint and, through the robot's
    couplings, on each coupled joint (`ratio * offset`), and keeps the error across plans until `reset`.
    The CNC player wires it for its machine. Native change, small: `simulation_robot.cpp/.hpp` (the
    offset and servo routing below), `simulation.cpp` (the roles), one C function.
  - **Servo motors carry coupled joints.** The blueprint gained `joint_servo` (stiffness and damping per
    joint). `RobotRuntimeCompiler` gives a joint gains when an actuator with a `ServoDrive` is on it
    *and the joint is in a coupling*: the actuator's `servoStiffness`/`servoDamping`, else a default of
    the drive's peak torque per hundredth of a radian and a 10 ms damping time (assumptions,
    `ServoDrive.defaultStiffness`), times the transmission ratio squared. In `Simulation::add_robot` a
    joint with gains is a servo motor; every other joint in its coupling group is passive. The endpoint
    sends a servo motor's position targets as servo targets (stiffness, damping, and the velocity the
    successive positions imply) torque-limited by the joint's effort (the drive's peak), drops passive
    joints' position targets, and holds only servo motors at rest. The coupling (a MuJoCo equality
    constraint) moves the rest. A servo on an uncoupled joint, an arm's, keeps the computed-torque
    tracking limited to its effort, so arms behave as before; steppers keep kinematic following.
  - **Not force-driven simulation for steppers**, for the reasons already given: wrong failure mode
    (lag instead of lost steps), numerically stiff (hundreds of kg of reflected inertia through a
    constraint), and gains to tune per machine.
  - **Tests.** `PlanCheckTests.testStepperSlip` (slip profile, whole steps, passing plans and servos lose
    nothing, `StepperSlip` offsets through a coupling and carried across plans), and
    `SimulationPoseResetTests.stepperSlip` (the offset on both backends) and `servoCoupling` (MuJoCo: the
    servo carries its coupled joint; the coupled joint takes no commands). The belt and screw routers' plan
    check numbers are unchanged (no stepper stalls, so no slip).
- **Gate (done): the belt router machines the motor plate under honest limits.**
  - **The controller is declared with the machine.** A simulated machine says what its
    steppers are wired to the way a deployment does: the router's machining job
    (`SceneArtifactMachining.controller`, optional, `{microsteps, stepTickHz}`) names the
    nominal wiring (`CncRouter.MICROSTEPS` 16 on `STEP_TICK_HZ` 40 kHz, the RKD6 board's
    software tick), and `CncProgramPlayer` binds the model to it with
    `DeviceBinding.bind(model, DeviceLayout.forActuators(model, microsteps), tick)`,
    the same call a device deployment makes, so the planner sees the real step-rate
    ceiling (78.5 rad/s for a NEMA 23: 25 mm/s on a 2 mm lead, 500 mm/s on a 6.366 mm
    belt pulley). Chosen over a separate layout file because the job is already what
    "the generator made for its own machine, with everything needed to run it", the
    player already builds its limits from it, and a project with no controller keeps the
    old uncapped behaviour. Both routers declare it: one controller, one comparison.
  - **Numbers** (24 V NEMA 23, 3 axes, the same motor-plate program, 0.5 mm stock rays):

    | | screw router | belt router |
    |---|---|---|
    | axis limits (Y, X, Z) from motors, screws and pulleys alone | 22.3, 34.6, 43.7 mm/s; 5.4, 5.58, 6.12 m/s² | 873.1 (Y, X), 43.7 (Z) mm/s; 12.79, 15.1, 6.12 m/s² |
    | axis limits as planned, with the controller and steady loads | 22.3, 25, 25 mm/s; 5.21, 5.37, 5.65 m/s² | 500, 500, 25 mm/s; 12.36, 14.19, 5.65 m/s² |
    | cycle time (before: 184.9 s screw, uncapped) | 220.2 s | 201.6 s |
    | material removed / leftover / gouge | 9914.9 / 63.5 / 0.6 mm³ | the same |
    | plans checked / flagged | 126 / 0 | 126 / 57 |
    | stepper stalls | 0 | 0 |
    | worst torque against the pull-out curve | 56.7% | 56.5% |
    | worst drive deviation (tolerance 0.1 mm) | 0.05 mm (the nut's backlash) | 1.9 mm (Y belts under 12 m/s²), 64 findings |

    The belt router is only 8% faster: the 25 mm/s Z axis (the controller's cap on a
    screw) and the cutting feeds dominate this program, not X and Y rapids. Its 500 mm/s
    rapids are also fast enough for the simulated carriage to lag its command by
    millimetres, so a rapid's label reaches 6 ticks into a cut (screw router: none); the
    test counts and prints them for belts and forbids them for screws. The belts' stretch
    is a prediction by the accuracy check, not simulated.
- **Examples (done).** The robot arm and the mobile base take their joint limits from their drives.
  - **Gearbox.** A `Gearbox` (ratio, efficiency) sits between a motor and a turning joint:
    `MachineAssembly.addMotor(id, joint, motor, volts, margin, gearbox)`, saved on the motor record. In
    the assembly format an actuator gained `gearRatio` and `gearEfficiency`: its torque, speed, curve and
    rotor are the motor's, before the gearbox. The bridge makes the transmission ratio `gearRatio`, puts
    the efficiency on `Actuator.efficiency` (the codec writes it when not 1; `DriveLoads`, `coupledLimits`
    and the runtime compiler use it) and the rotor's inertia on the joint's armature times the ratio squared.
    A joint's speed limit is then the motor's maximum speed over the ratio, and its torque limit the motor's
    peak through the ratio and the efficiency. Chosen over the existing `Drive.GearMesh` because a
    gear mesh needs two gears on two joints (an extra motor joint and body per arm joint, which would
    have reshaped the arm's joint list), and the gearbox is a fact about the actuator.
  - **Servo motor.** `ServoMotor` is a minimal `MotorDrive` part with generic ratings (50, 100 and 200 W
    on a 3000 rpm base: rated torque power over 3000 rpm, peak three times that, 5000 rpm at most,
    131072 encoder counts). They are round assumptions in the range of common AC servo families, not a
    vendor's datasheet. `ArmJoint` takes an optional servo (a recipe parameter) and is a `MotorDrive`
    itself: the joint module has its motor inside.
  - **Arm.** Each joint: servo and ratio chosen so the top speed lands near the old typed speed, and
    the servo the smallest generic one that carries the torque; gearbox efficiency 0.85 (strain-wave
    gearhead, assumption). Limits that used to be typed in, now derived:

    | joint | servo | ratio | speed before / now (rad/s) | torque before / now (N m) |
    |---|---|---|---|---|
    | j1 | 200 W | 250 | 2.0 / 2.09 | 300 / 405.9 |
    | j2 | 200 W | 250 | 2.0 / 2.09 | 400 / 405.9 |
    | j3 | 200 W | 220 | 2.4 / 2.38 | 250 / 357.2 |
    | j4 | 50 W | 175 | 3.0 / 2.99 | 80 / 71.0 |
    | j5 | 50 W | 175 | 3.0 / 2.99 | 60 / 71.0 |
    | j6 | 50 W | 130 | 4.0 / 4.03 | 30 / 52.7 |

    The rotor inertia reflected through the ratio is 1.6 kg m² on j1/j2, 1.2 on j3 and 0.05 to 0.09 on the
    wrist (armature). The pick-and-place mission in MuJoCo finishes at the same times as before (pick 5.8 s,
    place 12.0, pick 17.2, place 23.5 s): its speeds are set by the planner's limits, which barely moved.
  - **Mobile base.** Each wheel is its NEMA 23 through a 10:1 planetary gearhead at 0.9 (assumed; it is
    not drawn, the wheel still sits on the motor's shaft) on 24 V with half the holding torque relied
    on: usable speed 137.1 rad/s over 10 is 13.7 rad/s (1.03 m/s on the 75 mm wheel; it was a typed
    12 rad/s) and 0.63 N m through the gearhead is 5.67 N m (it was a typed 1.2 N m, the holding
    torque of the motor alone). `MAX_LINEAR_SPEED` (0.8 m/s) and the other drive limits stay as
    operating limits (assumed) under what the wheels give; the preview check holds them to it. In
    the app check the MuJoCo turn-in-place step (0 rad in 0.5 s) failed on the run just before this
    change and passes after it (0.5 rad); the wheel torque is the one thing that changed there, but
    that was not isolated. The mobile mission in MuJoCo now runs goTo 9.6 s, pick 15.1, goTo 23.6,
    place 29.0, pick 34.5, goTo 41.0, place 46.6, goTo 57.6 s; there is no earlier run to compare.
  - **Tests.** MachineKit smoke: the arm's drives (six servos, gearbox on each, joint limits equal what the
    drive gives, j1 at 2.094 rad/s and 405.9 N m) and the base's (13.7 rad/s, 5.67 N m, 1.03 m/s); app
    `checkLimitsFromDrives`: the compiled runtime limits of every arm and wheel joint equal the drive's;
    CadBridge: gear ratio, efficiency, armature and compiled limits.
- **Device.** Step edges are scheduled with hardware timers (STM32G4
  output compare and DMA) instead of a 40 kHz software tick, which caps
  16-microstep NEMA 23s near 25 mm/s and quantises step intervals into
  velocity ripple. A Trinamic driver profile in the wiring covers model
  (TMC5160/2160 for NEMA 23, TMC2209 for NEMA 17), current, chopper mode
  and StallGuard. Motion still uses step/dir; SPI/UART carries
  configuration, diagnostics and stall reports. The drivers' own ramp
  generators stay unused, because paths are planned and coordinated by
  MotionKit. Servo drives on a fieldbus wait for real hardware.

### X6d — Encoders

Status: the sensor, the parts, simulated counts, the lost-steps and following-error monitor and the
load-side path error are done (2026-10-02). Device counting and index homing are not (bench).

The plan check predicts stalls and path error; encoders observe them. An
open-loop stepper's lost steps are invisible without one. Model encoders as
sensors with a location, at drive level only: quantisation, no noise or
latency.

- **Sensor (done).** `robotkit.model.Encoder` is a sensor attached to a joint: kind
  (`EncoderKind` incremental or absolute), counts per joint unit (`Encoder.perRevolution`,
  `Encoder.perMillimetre` make them from the usual figures) and an optional index (once a revolution,
  or the reference mark at a sliding joint's zero). `RobotModel.encoders`, the codec (written only when
  there are some, so other models keep their bytes) and `RobotModel.encoderFor(actuator)` carry it.
  It is not compiled into the native runtime: a simulation reads it from joint positions, a device
  reports its own counts. Where it sits decides what it sees:
  - **Motor-side** (on a joint a motor drives directly) sees lost steps, stalls and a servo's
    following error, but not backlash or belt stretch.
  - **Load-side** (a linear scale, or an encoder on the driven pulley) sees where the carriage actually
    is, including stretch, backlash and pitch error.

  `Actuator.encoder` names the encoder that reads a motor, and a servo's own `encoderCounts` is what
  models saved before this recorded: `encoderFor` makes an incremental encoder from it, so old models
  read as before and a model that names an encoder holds the count in one place (the assembly format's
  `AssemblyEncoder` and `AssemblyActuator.encoder`, through the flattener, `AssemblyModel.addEncoder`
  and the bridge, which converts counts per millimetre to per metre).
- **Parts (done).** `ShaftEncoder` (counts per revolution, absolute or incremental, index; mounting
  face and axis) for a motor's back shaft and `LinearScale` (counts per millimetre along a strip) for a
  rail, both recipes, placed and mated like other parts; they are `EncoderPart`s, and
  `MachineAssembly.addEncoder(id, joint, part, ?motor)` records one, naming the motor it is the
  feedback of when it is. Rebuilding asks the part again, as for motors; an included assembly's
  encoders keep their prefix. Housing sizes are assumptions (38 mm, 22 mm; 10 x 2 mm strip).
- **Wiring.** Left: deployment layout channels naming which board input reads which encoder (layouts
  already carry `encoder_counts_per_rev`); needs the device work.
- **Simulation (done).** `EncoderReading` quantises counts from the joint position (incremental from
  the power-up position, absolute from the joint's zero, which for a robot built from an assembly is the
  initial placement) and counts index pulses. With stepper slip (X6c) a motor-side encoder sees the slip
  the commanded position hides.
- **Runtime (done, monitoring only).** `robotkit.runtime.EncoderMonitor` compares each encoder with the
  commanded position (the runtime's setpoint) every tick. A motor-side encoder past its bound latches a
  finding: `LostSteps` naming the motor and how many full steps for a stepper, `FollowingError` otherwise.
  The bound is the blueprint's `following_error_bound` for the joint when it has one, else (assumptions)
  two full steps of a stepper's rotor or eight counts of anything else. A load-side encoder reports the
  measured path error (`pathError`: latest, worst, RMS), the observed counterpart of the accuracy check,
  and faults nothing. The CNC player runs it (`CncProgramPlayer.encoders`) when the machine has encoders.
  Closing a position loop on a load-side encoder belongs to real servo drives. The monitor keeps the
  hardware seam: a device feeds `EncoderReading` its counts and calls `evaluate`.
- **Tests.** `PlanCheckTests.testEncoderSeesStepperSlip`: a gantry of weak steppers runs a program
  through the simulation, loses its whole move to slip (0.05 m), and the motor-side encoder latches
  `LostSteps` with the motor, the steps and the sign; strong motors lose nothing and the encoder is quiet.
  `testLoadSideEncoderReportsPathError`: the same command with the load 0.3 mm short is no fault for the
  motor-side encoder and a 0.3 mm path error for the scale; 40 lost steps on the screw are named as about
  40. Plus RobotKit codec and quantisation, CadBridge, and the MachineKit parts and records.
- **Device.** Count edges in hardware (STM32 timer quadrature mode) and report
  positions in the state frames. Index pulses support repeatable homing,
  together with limit switches at hardware bring-up. This needs the bench to
  verify.
- **Order.** After X6c, since slip is what makes encoders observable in
  simulation:
  1. the sensor and its parts (done);
  2. simulated counts (done);
  3. the lost-step / following-error fault (done);
  4. load-side path error (done);
  5. device counting.

### X7 — Transmissions compile from their parts

Status: T0–T5 done (2026-10-03).
Worktree `x7-transmissions`.

T1: one resolved relation and part-level equations replace the assembly's ratio, efficiency
and allowance switches. Fixed efficiencies and `DriveDefaults` are unchanged. Standalone
MachineKit and the full `x7-suite-t1-session.txt` gate pass: screw plate 220.2 s, belt plate
201.6 s, belt deviation 1.9 mm; CoreXY 204.1 rad/s, 649.6 mm/s, 71.6 / 34 m/s².
Earlier gate runs received external
SIGINT; running tests in a separate process session (`setsid`) lets MachineKit finish.

Through X6 a drive was a prototype for finding the semantics. `Drive` is an enum, but
`MachineAssembly` holds three string switches (`driveRatio`, `driveEfficiency`,
`applyDriveAllowances`) that cast members to `LeadScrew`, `SpurGear`, `TimingPulley` or `Sprocket`
and supply fixed efficiencies (0.98, 0.95, 0.97). `DriveRecord.kind` and the screw supports are
strings on the wire. Some allowances are frozen numbers: `setDriveStiffness` keeps a belt's stiffness
as a scalar that a rebuild does not work out again, a gear mesh never takes `GearPair.backlash`, and
a screw's backlash and drag are `DriveDefaults` whatever its nut is. `Belt` also covers roller chains.

X7 turns this into a small compiler: a **transmission** (the saved source: which parts, how they are
arranged) resolves to a **relation** (ratio, efficiency, stiffness, backlash, drag, speed cap) that
the assembly writes onto its `AssemblyJointCoupling`. The coupling is derived data. While a
transmission exists, its coupling is never edited independently, and a rebuild always overwrites it.

Decisions:

- **Name.** The source concept is a *transmission*: `Drive` becomes `Transmission`, `addDrive`
  becomes `addTransmission`, `DriveRecord` becomes `TransmissionRecord`, and `DriveDefaults` goes
  away (T3). "Drive" is left for motor electronics (`StepperDrive`/`ServoDrive`, X8's drivers).
- **A closed wire enum, not an open interface.** Transmissions are saved and read back, so the
  source is a `@:wire enum`, and the dispatch is one exhaustive `switch` the compiler checks. The
  equations live with the parts (`LeadScrew` with `LeadScrewNut`, `GearPair`, `Rack`/`SpurGear`,
  `TimingBelt` with `TimingPulley`, `Sprocket`/`RollerChainSpec`), not in `MachineAssembly`.
- **One result.** Ratio, efficiency, stiffness, backlash, drag and speed cap are resolved together
  as one `TransmissionRelation`, so no allowance can come from a different place than the ratio.
- **Coupling origin stays MachineKit-side.** ProjectKit does not learn about transmissions. A
  coupling is derived when the machine-side record names it. `MachineAssembly` refuses direct
  ratio/offset/allowance edits on such a coupling.
- **Stated overrides stay explicit.** A machine that knows better (a measured belt stiffness, a
  datasheet backlash) states it on the transmission, and the override is marked as stated (T5).

#### T1 — One resolved relation, numbers unchanged

- `machinekit.transmission.TransmissionRelation` (plain class): `ratio`, `efficiency`,
  `stiffness:Null<Float>`, `backlash:Null<Float>`, `drag:Null<Float>`, `followerSpeedCap:Null<Float>`
  (the screw's critical speed), in the coupling's units.
- Each kind's equations move out of `MachineAssembly` into a static function next to its parts.
  `MachineAssembly` keeps one call (resolve, then write the coupling and the joint's speed cap) in
  `addDrive`/`applyDrive`.
- Keep today's values exactly, `DriveDefaults` and the fixed efficiencies included. This step is a
  pure move.
- **Gate:** MachineKit, MotionKit, robotkit/cadbridge and the app's project-source suite pass, with
  the same printed numbers: screw router plate 220.2 s, belt router 201.6 s with 1.9 mm worst
  deviation, CoreXY motors 204.1 rad/s with axes to 649.6 mm/s, 71.6 (x) and 34 (y) m/s².

#### T2 — Typed source, schema v3, rename

Implementation: `Transmission`, `Sense` and wire `ScrewSupport` replace the string source and
support records. `TransmissionRecord.source` and `sense` use new wire IDs 12 and 13; retired
v2 IDs 2–4 are not reused. The machine description and document carry schema v3, checked
before decoding the old source form. `transmissionFor(id)` returns a detached source record;
its name avoids the old `LinearAxis.transmission` field that T4 removes. Physical router nuts move to T3: adding even the small Z nut changes the belt router's
acceleration (14.19 → 14.18 m/s² for X with steady loads), contradicting T2's unchanged-baseline
gate. T2 permits a null nut reference for the router during this transition; T3 makes it
mandatory and installs the nuts while changing engineering values. CoreXY already had both belt members, so its sources
now name those existing loops explicitly. Chain relations keep T1's old sprocket-as-belt
allowances in T2; their family defaults belong to T3.

- A wire enum for the source, roughly:
  - `LeadScrew(screw, nut)`. `LinearAxis` already has a `LeadScrewNut` member. The router's
    nut reference is temporarily nullable in T2; T3 adds the physical nuts and makes the
    binding mandatory, so T2 leaves its dynamics unchanged.
  - `GearMesh(driver, driven)`
  - `RackAndPinion(pinion, rack:Null<String>)`
  - `TimingBelt(belt, pulley, strand:Int)`, with the belt a member (the router already adds `beltX`
    and `beltY…`; CoreXY adds its two belts)
  - `RollerChain(chain:Null<String>, sprocket)`. Split from belts: a chain differs in compliance,
    backlash, efficiency and chordal speed variation. No example uses it yet; the tests do.
- `alignment:Float` (checked to be ±1 at runtime) becomes a wire enum `Sense { Same; Opposite; }`.
- `TransmissionRecord`: `{coupling, source:<the enum>, sense, leaderZero, stiffness?, backlash?,
  drag?, near?, far?, unsupported?}`; stiffness, backlash and drag are stated overrides.
- Schema version 2 → 3. No saved JSON in the repo has drive kinds; examples and fixtures rebuild.
  A v2 description is rejected with a clear message (no reader for the string form).
- Rename throughout (machinekit, examples, tests, docs).
- Screw supports: `supportScrew` stays (it needs the screw's joint to exist), but the record keeps
  `near`/`far` as the existing `machinekit.motion.ScrewSupport` enum (`Free`/`Simple`/`Fixed`) made a
  wire enum, not the `"free"/"simple"/"fixed"` strings. Deriving the support from the bearing parts
  at each end is a later step, not part of X7.
- **Gate:** the same relation values as T1 (ratios, efficiencies, allowances, caps). The source
  changed, but the relation must not have. Physical nut mass is introduced in T3 so all performance baselines stay unchanged in T2.

T2 validation: standalone MachineKit and every suite in `x7-suite-t2-baseline.txt` pass,
including the app project-source suite. All T1 engineering baselines remain unchanged.

#### T3 — Derive what was frozen

Numbers move in this step. Record the new baselines in this plan with a one-line reason each.

Implementation decisions: screw and nut threads must match; part edits must update both mating
threads. Generic nut assumptions are plain bronze 0.15 mm / 0.01 N m, anti-backlash 0.05 mm /
0.02 N m (the old defaults), ball nut 0.01 mm / 0.005 N m and 90% efficiency. Sliding nuts
use friction 0.1. Gear and rack compatibility is checked when resolving their relations.
`setTransmissionOverrides` states stiffness/backlash/drag; calling it with none clears the
overrides and resolves the current parts. A rejected support proposal leaves the old record intact. Inclusion and included-assembly
reconstruction also resolve their copied sources; they previously dropped coupling allowances.
For CoreXY's summed motor coordinates, axis compliance is the sum of each belt-path compliance
weighted by the square of its force share: equal shares give `K_axis = 4 / (1/K_A + 1/K_B)`.
Independent parallel drives retain their additive stiffness. Sprockets retain the previous
0.97 efficiency and 0.005 N m drag as separate chain-family assumptions; chain stretch and
chordal variation remain unmodelled, since no chain-member recipe exists yet.
Router geometry: 29 → 31 definitions, 52 → 56 occurrences, 28 → 30 BOM lines,
33.0 → 33.1 kg: four actual nuts, with separate flanged and barrel recipes.
CoreXY planned-limit worst deviation 0 → 0.088 mm: the two belts now contribute
their actual compliance instead of being treated as rigid; speed and acceleration are unchanged.

T3 router baselines (motor plate, same controller and feeds):
- Belt X free acceleration 15.10 → 15.08 m/s²: X carries the new Z nut.
- Belt planned Y/X acceleration 12.36 / 14.19 → 12.35 / 14.18 m/s²: both axes carry its added moving mass.
- Belt worst torque 56.5 → 56.6%: the moving nut changes the planned load and sampled trajectory.
- Belt worst deviation 1.90 → 1.89 mm: the added mass changes acceleration limits and sampled forces;
  belt stiffness itself is unchanged for these unchanged loops (the old frozen value used the same equation).
- Screw 220.2 s / 0.05 mm and belt 201.6 s remain unchanged, as do the 0 / 57 flagged plans,
  0 stalls, and 0 / 64 accuracy findings. The screw router's rounded limits and torque remain unchanged.



- **Belt stiffness** from the belt member on every resolve (`TimingBelt.carriageStiffness(strand)`).
  `setDriveStiffness` is gone, replaced by a stated override. CoreXY: each motor's transmission
  resolves its own belt; record the two-belt series stiffness the X5 notes left open.
- **Gear backlash** from `GearPair.backlash` (tangential at the pitch circle), converted to leader
  units (radians: divide by the driver's pitch radius). **Rack and pinion** takes the pinion's (plus
  the rack's, if it states one), in mm.
- **Router nuts.** Add a Tr10×2 nut member to each sliding body here rather than in T2,
  because their mass changes acceleration. Z uses a barrel nut (no flange, body diameter
  1.2 times the screw diameter) to fit its existing 13 mm plate gap; the flanged X/Y nut
  cannot fit there. Check clearance at both ends of Z travel.
- **Lead-screw backlash, drag and nut friction** from the nut. `LeadScrewNut` gains a nut kind
  (`PlainBronze`, `AntiBacklash`, `BallNut`, …) with catalog allowances. Today's
  `DriveDefaults.LEAD_SCREW_BACKLASH` (0.05 mm) and drag (0.02 N m) become the `AntiBacklash`
  values, so the router's numbers stay put if its nuts are that kind. Efficiency stays
  `thread.efficiency()`, with the nut kind's friction where it differs (a ball nut is ~0.9).
- **Fixed efficiencies** (gear 0.98, rack 0.95, belt 0.97) become each part family's documented
  defaults, marked assumed (T5).
- Delete `DriveDefaults`.

T3 validation: focused CoreXY and standalone MachineKit pass; every suite in
`x7-suite-t3-complete.txt`, including the app project-source suite, passes.

#### T4 — One model of a screw

`LinearAxis` builds its carriage-to-screw relation through `addTransmission(LeadScrew(...))` only.
`LinearAxis.setTravel` sets the carriage joint and lets `AssemblyState` propagate the coupling.
`LeadScrewTransmission` is deleted. The smoke checks already exercise the coupling, including handedness, travel and prefixed axes.
`MachineKitRobotCompiler` also used the helper: it now validates against the assembly's resolved
coupling and takes motor speed from the matching RobotKit coupling, without a second screw equation.

T4 validation: focused MachineKit/MotionKit compiler checks and every suite in
`x7-suite-t4-complete.txt` pass, including the app; all T3 engineering baselines remain unchanged.

#### T5 — Provenance of engineering values

The plan check turns these numbers into stall and accuracy claims, so it should say which inputs
were assumed.

Implementation: relation bases project to sorted assumption labels; stated overrides remove the
corresponding label. The critical-speed safety margin is also labelled assumed. Generic servo
models mark their ratings assumed; explicitly supplied servo ratings default to stated.
The frozen coupling schema previously omitted efficiency and allowances; it now carries matching
wire IDs 6–10 so those values and their provenance survive plain-coupling rebuilds as well as
transmission rebuilds. Copies and namespace changes retain labels. RobotKit writes them only
when nonempty, and plan findings name the assumptions along the relevant motor paths.


- A small wire enum `ValueBasis { Derived; Catalog; Stated; Assumed; }` and, on the resolved
  relation and on a motor's actuator, the set of fields whose basis is `Assumed`. Not a wrapper
  around every float: generics and the wire format make `EngineeringValue<T>` costly for little gain.
- Carried as an optional `assumed:ReadOnlyArray<String>` on projectkit `AssemblyJointCoupling` and
  `AssemblyActuator`, then RobotKit `JointCoupling`/actuator via the CadKit bridge, then the `PlanCheck`
  diagnostics: "predicted deviation 1.9 mm (assumed: belt stiffness, drag)". Written only when
  non-empty, so other models keep their bytes.
- Sources of `Assumed` today: the T3 family defaults, `TimingBelt.cordStiffnessPerMm`,
  `NemaStepper.ratingCatalog` inductance/rotor inertia, the generic `ServoMotor` ratings.

T5 validation: standalone MachineKit, focused CoreXY provenance checks and every suite in
`x7-suite-t5-provenance.txt` pass, including the app. All T3 engineering baselines remain
unchanged; the belt-router diagnostic now names its assumptions. No haxeon changes were needed.

### X8 — Motor, driver and controller

Status: X8a–X8e complete (2026-10-03).

Implemented (2026-10-03), after X7. Before this milestone, one call mixed three pieces of hardware:
`MachineAssembly.addMotor(id, joint, motor, volts, margin, gearbox)` takes the supply voltage, the
torque margin and the gearbox as numbers. Microsteps live in the machining job's `controller`
(`SceneArtifact`, `CncProgramPlayer`) and reach `DeviceLayout` channels. The step tick rate lives
there too, and in `SerialDeployment`. A stepper's torque–speed curve depends on its driver (current,
voltage, decay mode), and microsteps are a driver setting, so neither belongs to the motor or the
job.

Target split:

- **Motor** (part): intrinsic only (holding torque, inductance, resistance, rotor inertia, step
  angle; for a servo, rated/peak torque and speed). Unchanged from today's catalogs.
- **Driver** (new part, `machinekit.motion.MotorDriver`; stepper and servo families): supply-voltage
  range, current setting (A rms), microsteps, maximum step input rate, control mode. It has a BOM
  line and service ports: power in, motor out, step/dir or bus in. Generic catalog entries, marked
  assumed (e.g. a TMC2209-class and a DM542-class stepper driver, a generic servo amplifier).
- **Supply**: the driver's voltage comes from the service graph (`upstream()` from its power port to
  a supply part), not a number on `addMotor`. A machine without a modelled supply states the voltage
  on the driver.
- **Actuator binding**: `addMotor(id, joint, motor, driver, margin, ?gearbox)` names the motor and
  driver members. The torque–speed curve resolves from motor + driver (voltage, current; a lower
  current setting scales the torque). Microsteps go into `AssemblyActuator`, so `DeviceLayout` derives
  them from the model, and the job's `controller.microsteps` goes away.
- **Gearbox**: a member reference (`Gearbox` starts as a plain drive value; X8d makes it a recipe-backed part), not `gearRatio`/`gearEfficiency`
  numbers on the motor record.
- **Controller** (the board: step tick rate, bus cycle, channel count, identity) stays deployment
  data (`SerialDeployment`, the job's `controller.stepTickHz`). The model never carries it. The
  binding checks the driver's maximum step rate against it.
- **Encoders** stay sensors (X6d). A servo's built-in encoder is a spec on the motor that adds an
  encoder sensor on its joint when no separate encoder is wired.

Steps: X8a driver part + catalog; X8b `addMotor` with a driver, the curve from motor + driver, and
microsteps in the actuator (MotorRecord schema bump, router/CoreXY/arm/base updated); X8c supply via
ports; X8d gearbox member; X8e job/deployment cleanup (`controller.microsteps` removed, step-rate check).

X8a implementation:

- `MotorDriver` is a recipe-backed part with an assumed envelope, BOM identity and mount connector.
  Its catalog separates stepper and servo families and step/dir and bus control. Generic entries are
  `GENERIC-TMC2209`, `GENERIC-DM542` and `GENERIC-SERVO-AMP`; no entry claims verified vendor ratings.
- Current is an explicit A rms setting; microsteps are validated powers of two within the entry's
  limit (one for a servo). Supply voltage, when stated on the driver, must lie within its range.
  Power in, motor out and command in are service ports. Power in is required without a stated
  voltage; controller commands and motor wiring can remain implicit in the actuator binding.
- Generic step-input ceilings are assumed 250 kHz and 200 kHz, with assumed maximum rms currents
  of 2 A and 3 A respectively. The generic servo amplifier uses bus control and has no step-input
  ceiling. These are editable catalog assumptions, not product-selection guarantees.
- Driver recipes round-trip settings, BOM and ports. Cross-recipe checks cover every catalog entry;
  focused tests reject excessive current, unsupported microsteps and out-of-range voltage. Existing
  examples and actuator calculations are unchanged in this step.
- Validation: full `x7-suite-x8a-drivers.txt` gate passed all suites, application build and
  project-source tests. All X7 engineering baselines are unchanged: router plates 220.2/201.6 s,
  belt deviation 1.89 mm and CoreXY 204.1 rad/s, 649.6 mm/s, 71.6/34 m/s².

X8b implementation:

- `addMotor` now names a driver member. Schema v4 retires MotorRecord's voltage id 4 and adds
  driver id 8; v2/v3 descriptions and older documents are rejected before decoding their side data.
  Copies, inclusions and rebuilds resolve both member references again. Gearbox numbers remain
  until X8d. Driver voltage is explicitly stated in this step; wired supplies follow in X8c.
- A stepper's driver current must be positive and at most its motor rating. Lower current scales
  holding torque linearly and raises the existing reactance corner inversely with current. The
  same rated-current curve equation supplies both the planning limits and sampled curve. Current
  scaling is an assumption; back EMF, decay mode and resonance remain outside this model.
- Servo ratings have no winding-current or torque-constant datum, so amplifier current validates
  against the amplifier rating but does not invent a servo derating equation. Servo torque/speed
  stays intrinsic. Motor and driver families must match. Generic driver ratings join the actuator's
  assumption labels.
- Optional actuator fields 21/22 carry microsteps and the driver's maximum step-input rate through
  flattening, the CAD bridge and RobotModel's codec. DeviceLayout uses actuator driver settings. X9 requires these fields for every step/dir actuator.
- X9 requires an explicitly wired encoder sensor instead of inferring one from servo ratings.
  Its counts are expressed at the joint, including gearbox reduction, rather than retaining an
  inline motor count as the new model's feedback. Sensor references are prefixed on inclusion.
  Encoder records are the sole source of feedback; servo curve counts are ratings, and
  rewiring replaces the explicit sensor record.
- Router and CoreXY drivers are fixed to their frames, arm amplifiers to the pedestal and wheel
  drivers to the base plate outside the battery footprint. Assumed solid envelopes add fixed mass:
  router 33.1 → 36.3 kg (four DM542-class drivers), mobile base 27 → 28.6 kg (two), CoreXY
  3.99 → 4.01 kg (two TMC2209-class drivers). Their additional definitions/occurrences/BOM lines
  are respectively 31/56/30 → 32/60/31, 9/16/9 → 10/18/10 and 15/35/15 → 16/37/16.
  Arm total mass 146.9 → 149.8 kg and definitions/occurrences 24/25 → 25/31: six assumed
  servo-amplifier envelopes on the pedestal; its 20.6 kg moving mass is unchanged.
- Focused MachineKit smoke passed after correcting the wheel driver's connector frame and
  confirming no intersections at either tested wheel angle. CoreXY drive limits remain unchanged.
- Validation: full `x7-suite-x8b-hierarchy.txt` gate passed every suite, application build and
  project-source tests. The arm hierarchy test checks its six amplifiers by id under the pedestal.
  Router plate times, limits, torque usage and deviations retain X7's baselines; CoreXY drive limits,
  arm mission times and the mobile obstacle summary are unchanged. Only the hardware envelopes,
  fixed mass, BOM/part counts, explicit servo sensors and provenance labels change in the examples.

X8c implementation:

- `ElectricalSource` states the voltage at a supply port. Recipe-backed `PowerSupply` provides
  one shared DC voltage/current rating and separate output ports; the service graph pairs each
  physical port once, so driver feeds use distinct terminals rather than sharing one endpoint.
  Its rectangular envelope is assumed. The example battery pack exposes left/right feeds at
  its stated voltage; its BOM identity now includes voltage.
- A modelled supply, traced with `upstream()`, wins over a driver's stated fallback. Without a
  modelled supply, including an exposed external boundary, the driver must state its voltage.
  Unknown modelled output voltages and voltages outside the driver range are errors. Aggregate
  supply loading and voltage sag are not modelled; the DC current rating is not a phase-current
  limit on the motor.
- Motor and encoder bindings are source records. `addTo` compiles fresh actuator curves and
  sensors from current parts and the completed power graph instead of retaining derived arrays
  in the assembly. Late power wiring therefore updates a previously bound motor on compilation.
  Incomplete modules may bind before their power is known; export requires a supplied graph or
  stated voltage. Known ratings still validate when bound.
- Recipe ports are authoritative. Saved port records are reader snapshots, regenerated on
  reconstruction so changes to a driver's wired/fallback setting cannot leave stale required flags.
  Power connections and exposures are restored before validating known motor ratings.
- Rewiring feedback clears the old sensor record's motor association while retaining that sensor.
  This gives each motor one feedback source and preserves it through canonical sensor sorting.
- Router/CoreXY/arm supplies are fixed to their frames/pedestal. Each adds a 0.5832 kg assumed
  aluminium envelope: router 36.3 → 36.9 kg, arm 149.8 → 150.4 kg and CoreXY 4.01 → 4.60 kg.
  Router definitions/occurrences/BOM lines 32/60/31 → 33/61/32, arm definitions/occurrences
  25/31 → 26/32 and CoreXY 16/37/16 → 17/38/17. Mobile base remains 28.6 kg: its existing battery
  supplies both drivers. Carriage/arm moving masses and CoreXY drive limits remain unchanged.
- Focused MachineKit smoke passed, including a supply edit from 24 to 48 V (the stepper rate
  doubles), late wiring overriding fallback voltage, missing/range/unknown voltage failures,
  included and nested power reconstruction, and canonically sorted feedback rewiring.
- Validation: full `x7-suite-x8c-supplies.txt` gate passed all suites, application build and
  project-source tests. Router times/deviations/limits, CoreXY drive limits, arm mission times
  and the mobile obstacle summary remain unchanged from X8b.


X8d implementation:

- Schema v5 retires MotorRecord reduction/efficiency ids 6/7 and adds optional gearbox member
  id 9. Compilation resolves current parts, including prefixed and reconstructed members, and
  regenerates intrinsic motor-side feedback counts. Wrong component types are errors.
- `Gearbox` was a plain value, not a physical part as the target originally claimed. It now has
  a recipe, BOM identity, input/output connectors and an assumed bored cylindrical aluminium
  envelope. Ratio/efficiency are stated unless its saved basis says assumed; example gearheads
  retain their assumed ratios and efficiencies. Teeth, bearings, backlash and compliance remain
  outside this drive-level model. The bore clears the input shaft; detailed output shafts are omitted.
- `GearedArmJoint` is a separate recipe for a steel module with a machined pocket. Existing
  `ArmJoint` recipe inputs stay unchanged. Independent pocket dimensions reject oversized gearheads;
  installing separate gearheads does not double-count solid housing material. Original exterior
  dimensions and joint frames are unchanged. Arm and wheel joint effort/rate limits are no longer
  frozen copies of drive limits: compilation derives those limits from current actuator ratings.
- The mobile base installs each 40 mm gearhead between its motor and wheel. Track width changes
  300 → 380 mm from the solved geometry; clearance slots follow the relocated wheels. Drive ratios,
  motor curves, wheel radius and operating speed policy are unchanged.
- Arm mass above the base flange 20.6 → 18.3 kg, total 150.4 → 148.1 kg: six steel pockets
  are replaced by smaller assumed aluminium gearhead envelopes. Definitions/occurrences
  26/32 → 32/38: six separately dimensioned gearbox members. Mobile base 28.6 → 29.3 kg:
  two gearhead envelopes plus relocated clearance slots; definitions/occurrences/BOM lines
  10/18/10 → 11/20/11. Combined mobile cell definitions 41 → 48: its wheel and arm gearheads.
- Validation: full `x7-suite-x8d-gearheads.txt` gate passed all suites, application build and
  project-source tests. Rebuild tests cover edited ratio/efficiency/basis, regenerated feedback
  counts, wrong member types and nested member prefixes. Router plate times/deviations, CoreXY
  limits, arm/mobile mission times and the mobile obstacle summary remain unchanged from X8c.


X8e implementation:

- Machining jobs retain only `controller.stepTickHz`; driver settings come from the machine's actuator
  model. X9 layouts carry actuator wiring only and require explicit driver settings on every stepper.
- Pulse frequency is capped by `min(controller.stepTickHz, driver.maxStepRate)` before conversion
  to actuator rate. A faster board clock is valid: it idles between pulses. The tightened model
  supplies that same ceiling to planning and virtual/device execution without changing the source.
- `SerialDeployment` already has controller identity/timing separate from channel wiring and no
  global microsteps, so its schema does not change. Runtime protocol and robotd documentation now
  describe driver-setting agreement and the driver input-rate ceiling.
- Tests cover model codec settings, faster/slower controller clocks and immutable source rates.
  X9 replaces the earlier saved-data recovery paths and fixtures with current-schema rejection.


### X9 — Review fixes, no legacy compatibility, and the belt and load model

Status: X9a–X9d complete (2026-10-03), from a review of X7/X8 (`45d44846..351e772c`). The full suite passed
at `351e772c`, but that review found two confirmed bugs (arm and wheel joints lost their limits;
CoreXY Y takes the X strand's belt stiffness) plus model gaps and loose ends.

Policy for this milestone and after: **no backward compatibility for saved data.** Old files,
saved models, jobs and layouts are not migrated. Each saved format has one current schema
version, and anything else is rejected with one generic message ("schema vN is unsupported;
expected vM"). Remove compatibility code instead of maintaining it. Example projects, fixtures and
test data are regenerated by the current code. Wire ids may stay as they are, but comments about
retired ids go.

Each step has its own commit. At the user's request, the full suite (`x7-suite.sh`) ran after
the complete milestone rather than at each intermediate change. The final combined gate
`x7-suite-x9-final7.txt` passes all 13 kit suites, the app build and project-source tests.
MotionKit passes 9881 assertions and RobotKit world passes 4912. Seven selected native suites
pass (`x9-native-final2-tests.log`): runtime, validation, RKD6 compiler, virtual endpoint, MotionKit
validation, generator and path. Focused continuous-jog/session-end/blend checks pass 72/484/1036
assertions too. Each changed engineering number and its reason is recorded below; wall times,
allocations and collection counts remain performance observations.

#### X9a — Correctness

Progress: complete. Catalog selectors, document motor/encoder sources, required sense, per-field overrides,
read-only derived couplings and chain type/spec checks are implemented. Rebuilds retain incompatible
transmissions with actionable diagnostics and warn when resolved values replace a saved coupling.
Nullable effective limits now reach the compiled model and native runtime; native presence bits
separate missing caps from stated zeros. Runtime lowering no longer derives coupled caps. The axis
convenience compilers use the physical assembly and its resolved transmissions, retaining rated
motor assumptions; they state a 24 V rated-current drive with 16 microsteps and the generic DM542
step-input rating. Each stage's grounded preview parts attach to its motor body. Driver/controller
ceilings are named in plan-check `speedLimits` (informational, not plan violations). RKD6 feedback reconstructs independent carriage coordinates from the measured motor Jacobian,
including summed CoreXY terms and offsets; unobservable wired combinations are rejected. Native
float segment seams use twice the declared conversion error, while queued C2 checks retain a
bounded nanosecond quantization allowance. Hardware rate and position checks remain enforced. Mechanical limits are retained separately from effective limits,
so rebuilds cannot feed derived follower caps back into the sources and motor edits can raise caps.
The combined milestone gate above verifies the final implementation, including physical limits,
held/replaced/jogged motion, virtual motor feedback and real-discontinuity rejection.

X9a number changes: convenience NEMA 23 / 10 mm screw axes requested at 100 mm/s now cap near
43.7 mm/s: they use the actual 24 V rated-current motor curve through the resolved screw, rather
than a synthetic rated motor. XYZ fixtures now carry 7 links / 6 joints instead of 4 / 3: the
three motor shafts remain explicit alongside the three carriage travels (three independent DOFs).
The convenience XYZ carriage body origins are 82 rather than 112 mm on each axis: the old datum
was the bore connector; the physical body origin is 30 mm behind it. Kinematics checks now derive
that offset from each carriage connector. Virtual channel ratios become 1 for explicit shaft
coordinates; the source screw coupling still owns the signed travel-to-rotation ratio. Start vectors, goals,
jerks, IK jump guards and tolerances use the saved axis mapping, including shaft units. Accepted
jog fixtures change 50 → 40 and 80 → 43 mm/s (reversal −50 → −40 mm/s) to stay below the physical
43.7 mm/s ceiling. The clamp fixture requests the compiled ceiling rather than 100 mm/s, and
the rejected-jog test derives its bound as 1.001 times that ceiling.
The late-replacement fixture uses 40 mm/s for 2.5 s instead of 50 mm/s for 2 s, retaining its
100 mm endpoint. It delays the replacement at tick 80 rather than 40, so the two-second stream
window includes the longer jog's final deceleration before the deliberately late arrival.
A Cartesian acceleration-cap regression compares 0.4 against 2 m/s² rather
than requiring the peak to reach 2: the retiming grid reaches 1.515 m/s² at the lower speed cap. The router summaries
read screw critical speed from mechanical limits, rather than accidentally labelling a motor's
compiled effective cap as the screw critical speed.

The Cartesian blend fixture at these physical caps takes 2.524 s with exact stops, 2.544 s
with a 0.5 mm fillet and 2.457 s with a 2 mm fillet. A small fillet is not guaranteed to be
faster under the conservative junction bounds. The test keeps the 0.5 mm geometry and
non-stop checks, and uses the 2 mm fillet to demonstrate the speed/tolerance tradeoff.

Haxeon issue: `ExpressionTyper.comparison` contextually types its right operand as the left Int
before numeric promotion; a Float-valued conditional therefore fails E1003. The local workaround
uses `driverRate == null || stepTickHz < driverRate`. `Parser.parseEnumAbstract` accepts only
constants, so finding-kind quantity selection lives in the `PlanDiagnostic` helper class instead
of an enum-abstract method. Haxeon is unchanged. Cartesian validation now normalizes mapped shaft
coordinates to carriage units, keeping the 1 nm continuity claim independent of screw ratio.
Cartesian retiming and Hermite lowering use those same units, so their 1 µm distance tolerance
does not become a shaft-angle tolerance that changes with gearing.
Followers' polynomial coefficients are rebuilt from the axis mapping after lowering instead of
keeping independent Hermite fits. Axis moves and jog replacements run Ruckig once per logical
axis and project its complete polynomial onto the physical joints, so followers cannot acquire
independent rounding or synchronization profiles. Smooth replacements recover the whole trajectory's
clock origin from the streamer's recorded chunk offset; a later chunk tag cannot restart that clock
at zero. Continuous-jog regressions exercise replacements on both sides of chunk boundaries.
Native Ruckig lowering preserves the requested position, velocity and acceleration exactly at
time zero when an initial phase collapses below a nanosecond; its rounding belongs to the next
phase seam, rather than changing the promised replacement anchor.
Path limits allow 1 pm of floating-point roundoff for ordinary
travel ranges; a 1 nm authored excursion is still rejected. Checked machine streams declare the
same half-nanosecond jerk-based seam allowance as the native runtime, whose physical cap remains.
Machine submissions project their 1e-6 logical-unit position/velocity/acceleration tolerances
through the axis mapping too. For a 2 mm-lead screw the shaft allowance is 0.003142 rather than
0.000001 rad (or its derivative units), retaining the same 1 µm carriage allowance through a
blended-path continuation. Native checked-C2 comparisons still cap the declared allowance by
the clock quantization bound.
Native C0 comparisons use the position range's numerical tolerance, bounded by one clock quantum
of observed travel, rather than the requested jump as their scale. Regression tests cover
metres/radians, shifted origins, broad continuous-joint ranges and a real discontinuity. A
0.137 µrad screw seam (0.044 nm of carriage travel) no longer fails; wide joint ranges cannot
widen this allowance unchecked.

- **Compiled joint limits.** `RobotModel` is the compiled model, so it holds the effective
  limits.
  - After adding actuators, `AssemblySimulationBridge` writes `RobotModel.coupledLimits(joint)`
    into each joint's velocity/effort/acceleration limits: the tighter of the mechanical stop and
    the drive.
  - MachineKit joints state only mechanical limits.
  - A missing limit stays missing (`Null`). Remove the bridge's `null → 0` ("unlimited")
    conversion, and every place that reads 0 as unlimited.
  - `RobotRuntimeCompiler` then reads the compiled limits instead of deriving its own.
  - Test: the arm's compiled joint limits equal its drives' (2.09/2.09/2.38/2.99/2.99/4.03 rad/s
    before gearbox changes), and the mission planner and jogging use them. Without this, the
    mission planner falls back to 2.0 rad/s and jogging is unbounded.
- **One screw path in the compiler.** `MachineKitRobotCompiler.compileLinearAxisModel` and
  `compileXYZGantry` compile through the axis's `MachineAssembly` and `compileAssemblyAxes`.
  Delete `addPrismaticJoint`'s own `2π / travelPerRevolution` ratio. Compiled motors keep their
  assumption labels.
- **Design errors are diagnostics, not throws.**
  - Rebuilding (`fromDescription`, `include`, document load) always yields the assembly plus
    `machinekit.assembly.Diagnostics`.
  - A transmission whose parts don't fit stays unresolved, with an error naming it and the fix:
    a screw/nut thread mismatch ("update the nut to match the screw"), gear/rack module mismatch,
    a chain member that isn't a chain or doesn't match the sprocket.
  - Only malformed data (wrong schema, missing members) still throws.
- **Derived couplings are read-only.**
  - `MachineAssembly` throws on any edit by id of a coupling a transmission owns.
  - On load, a stored coupling that differs from its resolved values gives a warning naming the
    transmission.
  - (Implements X7's planned guard, which was skipped.)
- **API cleanups.**
  - `Sense` is required on `addTransmission` (no default); restore the per-kind meaning of +1 in
    the `Transmission` docs.
  - Overrides are set per field (state / clear / leave each of stiffness, backlash, drag), not
    all three at once.
  - `RollerChain` checks the chain member's type and that it matches the sprocket's spec.
  - Recipes list their options from the catalog. `GearedArmJoint` lacks `GENERIC-SERVO-100W`
    today. Add a test that every recipe offers every catalog entry.
- **Documents save sources, not compiled arrays.** `MachineAssemblyDocuments` saves the
  machine-side source records, including motor bindings and wired `EncoderRecord`s (missing even
  before X8), and compiles actuators and sensors on load. No document may lose a binding.
- **Driver step-input ceiling always applies.** The driver's maximum step rate caps the model's
  planning limits on its own. A bound controller's tick rate caps them further. When either one is
  what limits an axis, the plan check names it ("y limited by driver step input").
- Fix the stale `MobileBase` docs (the gearhead is now drawn; the wheel no longer sits on the
  motor shaft).

#### X9b — Remove legacy compatibility

Progress: complete, verified by the combined milestone gate. Scene artifacts now have
one schema (v15), with no snapshot section or old-version readers. App previews and the inspector
read the current assembly definition/state directly. RobotModel v7 saves drive records and nullable
caps; layouts use schema v1 and actuator wiring only. Stepper models require driver settings.

Compatibility survey: removed MachineKit's derived-property migration, edited-default recovery,
missing-tool-input fallback, old scene readers and controller microstep recovery, joint-only layouts,
channel microsteps/full-step fallback, legacy assembly snapshots/codecs, inferred servo encoder
sensors, and deployment-specific migration instructions. The shared CadKit DocumentCodec also
accepts only v11, because MachineKit recipes load through it; its v1–v10 branches and migration
registry are removed. BimKit uses the same current v11 document format; its old wrapper/import
readers and fixtures are removed. Obsolete document/naming fixtures are removed. Native derivative claims use
presence bits only; MotionKit's internal TOPPRA validator explicitly claims its velocity and
acceleration bounds too. Current fixtures are regenerated through the v7 model and v1 layout
codecs, with explicit drivers and current deployment versions. Optional current-format fields remain optional. Runtime detector
records, hook APIs and protocol rejection tests are not saved-data migration paths. App user-data
migrations remain out of scope.

Delete, with their tests and fixtures:

- `MachineAssembly.checkDescriptionVersion`'s version-specific messages. Use one generic check
  inside `fromDescription`, so every caller (including direct `JsonWire.decode` users such as
  `EndEffectorExampleChecks`) gets it.
- `SceneArtifact`: dropping old `controller.microsteps`, and the length-prefixed legacy-artifact
  fixture in `ProjectKitTests`. Machining jobs carry `stepTickHz` only.
- `DeviceLayout`/`DeviceBinding`:
  - layouts that name joints (layouts name actuators);
  - the full-steps fallback for models without driver settings;
  - explicit per-channel microsteps (they come from the actuator).
  - A stepper actuator without `microsteps` and a step-rate ceiling is a model error. Regenerate
    the virtual-device fixtures from models.
- `AssemblyActuator`: "absent for legacy actuators" wording; driver fields are required for
  step/dir actuators.
- Comments and plan text that justify behaviour by old data ("older models still load", "legacy
  models may state…", "retired id").
- Other compatibility paths in the kits this work touches (machinekit, projectkit, robotkit,
  motionkit). Survey them, remove those that only serve old saved data, and record each one here.
  Known candidates:
  - the `machinekit.legacy-derived-properties` document migration and the edited-defaults version
    check in `MachineKitRecipes`;
  - `AssemblyRecord`'s legacy snapshot and `MateriaProjectRunner.legacySnapshot`.

  App-level user data (`AppPreferences` legacy-file import, `EditorWorkspaceLayout` viewport
  migration, `ScriptOwnershipRecord`) is out of scope: leave it unless the user decides otherwise.

#### X9c — Belt and load model

Progress: complete, verified by the combined milestone gate. Belt paths save an explicit
clamp connector and wrap connectors in loop order (approved by the user), so assembly poses identify
moving wraps. Assembly schema v6 saves these sources and rebuilds them, including nested tools.
Drive stiffness uses both elastic paths to a held drive pulley and the weakest sampled travel pose;
part length and attachment routing must agree. Idlers carry no elastic spring. The shared drive
compliance is the inverse of Jᵀ diag(K_motor) J with rigid motor constraints; motor lost motion is
projected through the full Jacobian. A shared belt uses the weakest motor-side spring found
across its sampled leader travels, making the rebuilt matrix a conservative constant envelope.
Plan checks apply the force vector including gravity to this compliance. Missing catalog belt modulus, rail drag and gearbox input inertia stay labelled
assumptions. Per-quantity provenance follows rebuilds and each diagnostic selects the fields it uses.


X9c baseline changes (physical deterministic results; per-tick wall times and collection counts
are performance observations, not engineering baselines):

- Belt router planned Y/X acceleration 12.35/14.18 → 12.25/14.06 m/s²: passive idler bearing drag
  now reaches the axis load. Bare motor acceleration stays 12.79/15.08 m/s².
- Belt router worst deviation 1.894 → 1.759 mm; tolerance findings 64 → 76: clamp-to-drive span
  lengths and the shared Jacobian replace the selected-strand and series/share approximations.
  The worst finding moves from line 150/op 4 at 1973.551 mm (0.027 s) to line 252/op 9 at
  270.48 mm (0.04 s), because the governing elastic load changes. Flagged plans stay 57 of 126.
- Belt router worst motor utilization 56.6 → 56.4%: the matrix/load projection and idler drag
  change the motor load sampled along each retimed plan. No stepper stalls remain.
- CoreXY bare X acceleration 71.6 → 71.5 m/s² (71.549 in the app): idlers no longer apply drive
  efficiency to their reflected inertia. Bare Y remains 34 m/s²; motor speed and axis speed
  remain 204.1 rad/s and 649.6 mm/s.
- CoreXY deviation 0.088 → 0.148 mm at planned limits: Y uses its actual frame spans, shared
  belt compliance and passive bearing drag.
  Planned accelerations 57.957/27.542 → 38.264/21.306 m/s²: five passive idler bearings per
  belt now contribute drag under steady loads. The square takes 79 → 85 ticks, with peak motor
  speeds 96.2/132 → 86.8/124.5 rad/s because those short moves accelerate more slowly.
  Accuracy findings are checked separately from stall findings; the belts exceed 0.1 mm.
- The deliberate overload fixture uses 10× rather than 3× caps: the conservative Y acceleration
  with idler drag no longer stalls on the old short move. Its diagonal uses equal X/Y caps to
  keep one motor still, rather than generating a curved move with different axis accelerations.

Unchanged router machining baselines: screw/belt 220.2/201.6 s; removed 9914.9 of 9996.5 mm³,
leftover 63.5 mm³ and gouge 0.6 mm³; rapid-labelled cutting ticks 0/6. Screw utilization/deviation
remain 56.7% / 0.05 mm, with 126 plans and no findings. Router bare and controller-limited speeds,
screw accelerations, belt tooth counts and part lengths are unchanged.

- **Belt stiffness from geometry.** `TimingBelt(belt, pulley)` drops the hand-picked strand
  index. The resolver works out, from the belt's path, how much each span stretches per unit of the
  leader, and takes the stiffness from those span lengths and stretches.
  - A two-pulley loop reproduces `EA(1/a + 1/(L − a))`.
  - CoreXY Y gets its frame spans. Today both axes use strand 0, so the recorded 0.088 mm CoreXY
    deviation is wrong.
- **Idlers are not drives.** A new source kind `BeltIdler(belt, pulley)` resolves ratio and bearing
  drag only. Belt stiffness belongs to the pulley that delivers torque. The router's and CoreXY's
  idlers use it, so no consumer can count a belt twice.
- **Axis stiffness from the coupling Jacobian.** Replace `DriveLoads`' `terms > 1` series rule and
  share-weighted compliance with the general rule:
  - axis stiffness matrix `Jᵀ · diag(K_motor-side) · J`, with `J` from the coupling ratios (motor
    coordinates per axis unit);
  - backlash as the worst projection of each motor's lost motion.

  This covers single drives, parallel dual-Y and CoreXY (equal belt tension whatever the motors)
  without special cases.
- **Provenance per field.**
  - Assumption labels are kept per quantity (stiffness, backlash, drag, efficiency, inertia, motor
    curve, speed limit), and each finding kind declares which quantities it uses. Deviation uses
    stiffness, backlash, drag and steady loads; stall uses the motor curve, inertia, efficiency and
    drag.
  - The screw critical-speed label moves from the coupling to the joint's speed limit.
  - Label what is still unlabelled: rigid screw/gear/chain couplings ("rigid"), rail drag,
    `ArmJoint`'s servo ratings basis (saved in its recipe).
- **Gearbox reflected inertia.** Gearbox entries carry input inertia, added to the motor's
  armature through the ratio. It is assumed until there's catalog data.

#### X9d — Example design follow-ups

Progress: complete, verified by the combined milestone gate. Gearboxes declare a saved mass and
housing inertia, using a steel-class annular engineering estimate until catalog data is available.
The base plate derives its width from the wheel slots and checks a 10 mm minimum web; track remains
380 mm. Wheel limits read rebuilt actuator ratings from the connected battery, with a 48 V regression.
The shared KinematicsKit helper derives the unicycle envelope from compiled wheel caps; navigation, path
trajectory planning and commands all use it, and the preview checks combined translation and turning.
Deployment note: RKD6 at 40 kHz and 16 microsteps caps this base near 0.59 m/s; 8 microsteps or a
faster board can raise that ceiling.


X9d baseline changes:

- Arm mass above the base flange 18.3 → 20.3 kg; total 148.1 → 150.1 kg: steel-class annular
  declared gearbox masses replace aluminium display-envelope estimates.
- Mobile base 29.3 → 30.8 kg; plate width 440 → 452 mm: steel-class gearheads and a plate sized
  from wheel slots plus a checked 10 mm edge web replace the old 4 mm web. Track stays 380 mm.
- Mobile cell mission completion times 9.6/15.1/23.6/29/34.5/41/46.6/57.6 →
  9.6/15.1/25.2/30.6/36.1/47.8/53.2/61 s: combined turning/translation now respects the wheel
  envelope, with declared gearhead masses in the MuJoCo plant.
- Mobile obstacle round 59 → 65 s, closest obstacle 419 → 416 mm: the same wheel envelope and
  plant mass changes alter path timing and sensor-triggered replanning. Cruise/minimum speed
  remain 0.4/0 m/s, with one replan.

Held example baselines: arm pick/place 5.8/12/17.2/23.5 s; both mobile backends travel 400 mm
in 1 s and turn 0.5 rad, with 400 mm odometry. Definition/occurrence/BOM counts stay arm 32/38,
router 33/61/32, mobile 11/20/11 and CoreXY 17/38/17; the mobile cell still has 48 definitions
and 3 goals.


- **Gearbox mass from catalog data.** `Gearbox` entries state mass and inertia
  (`declaredMass`), assumed steel-class values until there's catalog data. The aluminium envelope
  stays for display and collision only. The rebuilt moving mass is 20.3 kg with these assumptions,
  replacing the preliminary estimate near 20.6 kg.
- **Derived base plate.** Keep the derived 380 mm track. The plate half-width becomes wheel slot
  edge plus a minimum web (design rule, 10 mm, checked), replacing today's 4 mm web.
- **Base velocity envelope.** The base's speed limits are a unicycle envelope from the wheels:
  `|v| + |ω|·b/2 ≤ ω_wheel·r`, used by the planner and the preview check. Today a fast turn at top
  speed asks for 15.7 rad/s against 13.7.
- **Supply-derived wheel limits.** `WHEEL_SPEED`/`WHEEL_TORQUE` come from the battery through the
  power graph, not the stated 24 V fallback, so changing the pack (the mobile welder's 48 V) moves
  every limit.
- Design note, no code: on the RKD6 board (40 kHz tick, 16 microsteps) the base tops out near
  0.59 m/s. With the X9a ceiling this shows in the plan check. Choosing 8 microsteps for the
  wheels, or a faster board, is a deployment decision.

Merge order: X7–X9 onto local main first, then rebase the `mobile-welder` branch. It must take
the 24 V → supply-derived wheel limits and the `RobotArm.hx` changes.

#### X9e — Review fixes for X9

Status: complete (2026-10-03). `x7-suite-x9e-final2.txt` reports 11/11 kit suites, CadKit,
MachineKit, app build and project-source suite at exit 0. Focused app worker/scene tests,
humanoid tests and mixed-scene command, and native RobotKit/SimKit MuJoCo tests also passed.
This review follows X9 on local main `4b952231f` (X9 merged with the robot welder).

**Breaks (fix first):**

- **Worker examples don't open.** `app/examples/worker-rack-to-table.materia` and
  `worker-gallery.materia` embed a `cadkit.document` at v9. Since X9b only v11 loads, so the Start
  page examples and `--worker-demo` fail. Regenerate rack-to-table with the current code, then the
  gallery from it (`app/tools/make-worker-gallery.py`). Add both to the gate.
- **Plan check uses the wrong axes for partial plans.** `PlanCheck.hx` (around line 205) builds
  forces only for the plan's axes, but `loads[0].elastic` is a `DriveCompliance` over every driven
  axis in the model, and `DriveCompliance.deflections` never checks the size.
  - Build the compliance for the plan's axes: other axes are held by their own drives.
  - Make `deflections` reject a size mismatch.
  - Test with a model that has a driven axis outside the plan, ordered before the plan's axes.
- **Under-actuated coupled pairs throw everywhere.** `DriveLoads.of` always inverts the shared
  matrix (`DriveCompliance.invert` throws "singular coupling Jacobian"), and `RobotModel.steadyForce`
  and the effective limits go through it.
  - A CoreXY with one motor bound (mid-edit) must give a design diagnostic and per-axis limits
    from what is driven, not a throw.
  - Build loads and the matrix once per model, not once per `forAxis` call.
- **Humanoid mixed scene arm is frozen.** `robotkit/tools/humanoid/src/humanoid/MixedScene.hx:55`
  sets `velocity = 0.0`, which X9a made a real cap of 0. Use `null` or a real cap. Make its check
  prove the arm moves, not just that it matches the arm alone.
- **Sensor documents don't reload.** `app/src/SensorConfiguration.hx` saves missing limits as
  `null` but loads them with `finite()`. Load them as nullable, and save `maxAcceleration` and the
  mechanical limits too. Test the round-trip with a URDF joint that has no velocity limit.

**Model errors:**

- **Gearbox input inertia.** `Gearbox`'s default 5e-5 kg·m² for every size is about 17× the 50 W
  servo's rotor; real 60 mm planetary and size 14–17 strain-wave heads are about 1e-6–8e-6. Scale it
  with gearbox size (or take it from catalog entries), labelled assumed. Re-record the arm's numbers.
- **Idler drag.** `BeltIdler` resolves with the belt family's drag, so CoreXY (5 idlers a belt) counts
  belt drag about 6× and applies it to both axes even when an idler doesn't turn. Use an idler
  bearing drag, and apply it only along the axes that turn that idler. Re-record CoreXY's
  acceleration (57.96 → 38.26 m/s² came from this).
- **Belt stiffness minimum on 2-D paths.** `BeltStretch` samples only the leader's own travel, with
  other joints at their defaults. For a clamp that moves with two axes (CoreXY), search the joint
  workspace (corners plus the balanced-length points) for the softest pose. With no travel limits,
  report that the minimum is unknown instead of using the default pose alone.
- **Mobile base envelope vs acceleration.** `robotkit/mobile/MobileBase.hx` scales the command to
  the velocity envelope after `motionLimits.constrain`, so a turn request can cut forward speed in
  one tick. Apply the envelope before the acceleration limits, so deceleration stays within them.
  Make the preview check test something `constrain()` doesn't guarantee by construction.
- **Rebuilt end effectors keep diagnostics.** `MachineAssembly.copyInto` (used by
  `EndEffector.fromDescription` and `EndEffectorSet.fromDescription`) drops `diagnostics`, so a
  tool's design error resurfaces as a throw in `addTo`. Copy them.
- **Smaller:**
  - Gearbox mass: derive it from size (or catalog) on every rebuild, not a saved `massKg`, and
    label it assumed.
  - A stated stiffness skips the belt-geometry derivation instead of failing with it.
  - `MachineKitRobotCompiler` must not save a requested planning speed/acceleration as a mechanical
    limit.
  - A stated effort of 0 means the same in runtime, simulation and MuJoCo (today: fault vs unlimited).
  - `writeTransmission` clears a screw speed cap when a later resolve has none.
  - The recipe-catalog test covers every catalog-backed parameter, not only ones named `servo`.

**Stale docs from X9b:**

- `robotkit/robotd/README.md` (layout channels no longer state `microsteps`; layouts carry
  `schemaVersion: 1`) and `robotkit/runtime/DEVICE_PROTOCOL.md` ("legacy models may state them").
- CadKit: `MODELING.md` ("older documents remain readable"), `TopologyFingerprint.hx` comments,
  `cadkit/plans/TOPOLOGICAL_NAMING.md` and `CONSTRAINT_SOLVING.md` (the deleted pre-v10
  `BOX_FILLET_V10` case is still listed as passing).
- `projectkit/README.md` (scene artifact v10) and `machinekit/TODO.md` (compatibility reader).

Also record in this plan that X9b removed CadKit `DocumentCodec` v1–v10 and the BimKit v2 import,
beyond the kits X9b listed. The user accepted it with the no-compatibility policy.

X9b also removed CadKit `DocumentCodec` v1–v10 and the BimKit v2 import. The
user accepted both under the no-compatibility policy; old fixtures and docs
describing those readers are historical only.

**Watch:** the welder arm now plans at its drive caps (2.09–4.03 rad/s) instead of the 2.0 rad/s
fallback. Check the robot welder's timing expectations (`ProjectSourceTests.checkRobotWelder`) and
re-record any that move.

**X10 review findings to fix in the same pass (2026-10-03):** The passing X10 gate exercised
the folded-Z network, but did not establish correct behavior for every mixed machine.

- A network anywhere switches every axis to `ElasticSolve`. Scope assumptions, compliance,
  backlash, and diagnostics to the network's connected axes. Test a CoreXY and an independent
  screw axis alongside a shaft-belt network. Preserve the largest lost-motion allowance for
  several couplings driving one joint, and keep an unbound motor from breaking unrelated axes.
- Derive belt-reduction direction from the pulley contact sides and axes of the posed path;
  validate authored `Sense` against that direction. Check an ordinary two-pulley loop and a
  back-side serpentine wrap. A sign that only follows the authored coupling is insufficient.
- Give every loaded screw pulley in a Z-sync loop the tooth-contact clearance, including
  outputs joined by lead-screw constraints rather than their own reduction record.
- Make the folded-Z motor mount tensionable with an explicit range and verify belt working
  tension against a stated catalog or engineering limit. Assert its expected speed,
  acceleration, stiffness, backlash and mass so those numbers cannot drift silently.
- Finish the shared belt-span calculation for both carriage clamps and rotary attachments;
  validate path wrap count before indexing; specify pretension and reject load cases that
  would slacken a span. Record the cost of posing a copy on each `describe()` rebuild.
- Verify the suspected HXI ordering defect before changing haxeon: `PackageResolver.resolveFiles`
  alphabetizes RobotKit's interfaces, but the pinned haxeon also orders them by declared
  dependencies before registration. Keep the router's CadBridge assertions in MachineKit tests;
  record whether the compiler issue actually remains.

The completed fix pass has a passing full gate, and each changed engineering number is recorded
here with a reason.

X9e resolves belt-network compliance only for axes connected by shared motors or elastic spans;
independent screw axes retain their own solve and assumptions. Under-actuated groups keep a design
diagnostic and per-axis limits without blocking independent groups. A pulley reduction gets its
direction from the belt's contact sides and posed joint axes; an incompatible authored Sense is a
design error. Every loaded screw pulley in a Z-sync loop receives tooth clearance. Belts use the
same free-span spring calculation for carriage and shaft attachments, validate their wrap count,
and state assumed pretension and a working-tension bound. The folded-Z motor pilot and bolts have
5 mm tensioning slots; its 6 mm GT2 belt uses an assumed 120 N pretension and 250 N working limit.
The folded stage still asserts 43.653927/21.826964 mm/s, 6117.456/3263.410 mm/s²,
486.867 MN/m, 0.05/0.0505 mm backlash, and 36.9/37.1 kg with the baseline router.
`describe()` still poses a copy for each rebuild: pose context is shared across its belt paths,
but the copy remains proportional to assembly size and is not cached across rebuilds.

X9e numerical changes from the X10 gate (per-tick wall times and collection counts are performance
observations, not engineering baselines):

- Assumed input inertia for a 60 x 40 mm gearbox 5e-5 → 5e-6 kg·m²: the size-scaled steel-class
  estimate replaces a single oversized value. Other sizes scale with the authored diameter and
  length; mass is recomputed from those dimensions on each rebuild. Arm mass stays 20.3 kg above
  the flange, 150.1 kg in all, and pick/place stays 5.8/12/17.2/23.5 s.
- Belt router controller-limited Y/X acceleration 12.25/14.06 → 12.35/14.17 m/s²: passive idlers
  contribute bearing drag only where their pulley actually turns. Bare motor limits stay
  12.79/15.08 m/s². Worst motor utilization 56.4 → 56.5%, and over-tolerance findings 76 → 64:
  corrected idler drag and the 2-D softest-pose belt stiffness change the retimed load projection.
  Worst deviation 1.759 → 1.758 mm, at line 252/op 9 → line 171/op 4: the new weakest path and
  sampled load move the governing finding. The rounded 1.76 mm worst deviation remains unchanged.
- MotionKit CoreXY planned X/Y acceleration 38.264/21.306 → 57.409/27.415 m/s²: unpowered
  zero-ratio idlers no longer charge both axes bearing drag. The square takes 85 → 79 ticks and
  peak motor rates 86.8/124.5 → 96.2/131.9 rad/s because the corrected drives accelerate more
  strongly. Worst deviation 0.148 → 0.133 mm: the changed load and 2-D workspace stiffness alter
  its elastic deflection. Bare speed and acceleration remain 204.1 rad/s, 649.6 mm/s and
  71.5/34 m/s².

The screw and belt router plate times remain 220.2/201.6 s; screw deviation/utilization remain
0.05 mm/56.7%, and the belt keeps 57 flagged plans with no stepper stalls. The robot welder's
20.2/21.6 s MuJoCo runs and four-side 19.9 s run match the X10 app log. The mobile cell keeps
9.6/15.1/25.2/30.6/36.1/47.8/53.2/61 s mission steps and a 65 s obstacle round with 416 mm
closest clearance and one replan. The mobile limiter now applies the wheel envelope before
independent linear/angular acceleration caps, then stays inside their convex intersection;
the first X9e gate found that scaling both acceleration deltas together delayed steering and
hit a table, while the corrected focused mobile run follows the original route.

The suspected haxeon HXI dependency-order defect was already fixed in the pinned submodule by
`bd014aec` (2026-09-29). `PackageResolver.resolveFiles` still sorts interface paths, but
`CompilerSession` calls `HxiInterfaceOrder.dependenciesFirst` before registration and immediate
composition validation. `HxiInterfaceOrderMain` tests a dependent path that sorts first; X9e's app
build log also shows `RobotKitRuntime` loading before `RobotKitInference`. No further compiler
change or submodule pin change is needed for this issue. The CadBridge assertions remain in
MachineKit tests; keeping them out of the router example is no longer required by HXI ordering.

### X10 — Belt reductions and loops between shafts

Status: X10a–X10d complete (2026-10-03). Shared belt-span elasticity is implemented across
MachineKit and RobotKit. The combined full suite passed after the complete milestone:
`x7-suite-x10-final3.txt` reports 13/13 kit/CadKit stages, app build and project-source suite
at exit 0. The intermediate X10a–X10d commits were checked with focused tests; the combined
gate was run on the completed tree, as requested.

X9c derives belt stiffness from the belt's path, but only for a belt clamped to a sliding
carriage: `BeltStretch` throws unless the leader is prismatic. Common drives that turn a shaft
through a belt can't be modelled with real parts:

- folded-back motors on ball or lead screws (1:1 or 2:1);
- rotary 4th axes (3:1–6:1);
- belt-driven spindles;
- Z-sync loops tying two or four lead screws to one motor (Voron Trident, many printers);
- belt reduction stages (Voron 2.4 Z 80:16);
- belt-driven arm joints (Moveo/Thor-style arms, SCARA, wrist motors moved toward the base).

Faking these as `GearMesh` gets the direction wrong (a belt keeps the turning direction) and has
no belt stretch.

One physical belt must compile to one elastic network. MachineKit owns its attachments,
geometry and belt-family assumptions; RobotKit solves the derived span network under the
combined loads. Pairwise stiffness is a result of that network under stated boundary conditions,
not a collection of independent springs replacing the belt.

#### X10a — `BeltReduction`

- New source kind `BeltReduction(belt, driver, driven)`. Both pulleys are `TimingPulley` members
  on the same belt; the driver turns on the leader joint, the driven pulley on the follower.
  - **Ratio:** `driver.teeth / driven.teeth` (follower radians per leader radian). A belt keeps
    the turning direction, so `Same` means both turn the same way about the belt plane's normal.
  - **Efficiency and drag:** the belt family's values, as for `TimingBelt`.
  - **Stiffness at the leader (N·m/rad), as the two-terminal case of the span network:**
    - hold the driven pulley's teeth in place;
    - turn the leader by a small angle;
    - measure the stretch of the two belt paths between the pulleys;
    - stiffness = `EA · (da²/a + db²/b)`, with the stretch per radian.

    For a pretensioned two-pulley loop this is `EA (1/L₁ + 1/L₂) r_driver²`, where the free
    lengths end at tooth engagement, rather than the middles of the engaged pulley arcs.
    Convert N·mm/rad to N·m/rad at the assembly boundary. Record the assumed pretensioned
    operating mode. A slack span cannot be modelled as a bilateral linear spring: either solve
    tension-only spans with stated pretension and load direction, or reject the unsupported
    mode explicitly; do not silently apply the two-span formula without pretension.
  - **Backlash:** a timing-belt tooth-clearance allowance at the driven pulley's pitch circle,
    converted to leader radians. It is an assumed belt-family value until there's catalog data.
- `BeltStretch` generalises to elastic paths between attachments. A clamp on a carriage
  (prismatic leader) and teeth engaged on a pulley (revolute leader) use the same span-energy
  calculation. Remove the prismatic-only check. Validate that each pulley follows its stated
  joint and that its axis is aligned with the belt-plane normal; account for local axis signs.
- **Router or CoreXY-style axis drive through a reduction:** a motor pulley drives a big pulley
  whose shaft carries the carriage belt's drive pulley. That is `BeltReduction` from motor to
  shaft plus `TimingBelt` from shaft to carriage, with coupling orientation following the
  assembly's planning-coordinate convention. Preserve the intermediate shaft as an elastic
  coordinate until the solve eliminates it. Check that the two networks' compliances add in
  series at the axis, with squared ratio scaling, and that each spring is counted once.

X10a implementation: source wire id 7, MachineKit schema 7, optional clamp for shaft-only
paths, matching pulley pitch radii and rotary attachments, and assumed pretensioned stiffness.
Tooth clearance is 1% of family pitch, pending measured data. Existing example numbers have
not intentionally changed. The focused test project now includes the example roots and dependencies
its source graph requires; the completed milestone passed the combined gate.

#### X10b — Belt loops with several driven pulleys

- One belt can drive several pulleys: a Z-sync loop driving 2–4 lead-screw pulleys from one motor,
  or a reduction with a tensioner idler.
  - Use `BeltReduction(belt, driver, drivenN)` for each new motion relation; idlers on the
    loop use `BeltIdler`. When a screw already follows a common carriage through a `LeadScrew`
    relation, do not add another motion term to that follower: the terms would sum and double
    its travel. Its attached rotary pulley is still a loaded terminal of the same belt network.
    Loaded rotary terminals are derived from the wrap attachments and fixed shaft mounts;
    explicit `BeltIdler` sources identify free terminals.
  - These source records describe motion relations and reference the same belt network; they
    must not compile into independent copies of its springs.
  - Resolve a driven pulley's scalar stiffness with the other driven pulleys free only for
    reporting and two-terminal verification. Combined loads use the full network.
  - Check: a loop's tooth count, wrap radii, pulley profiles and attachment order must agree,
    as for any belt. Free idlers pass tension and contribute their bearing drag and inertia;
    they do not anchor the belt elastically.

The original shared-axis claim was incomplete: X9c's motor-coordinate Jacobian handles CoreXY
loads, but cannot represent independent stretch between several loaded pulleys on one loop.
For three equal free spans with stiffness `k = EA/L`, holding the motor gives the two outputs
the stiffness matrix `k [[2, -1], [-1, 2]]`. Either output with the other free measures `1.5k`.
With equal force `F` on both, the actual deflection is `F/k` at each; independent `1.5k` springs
predict `2F/(3k)`, understating deflection by one third.

- **MachineKit:** derive each free span's length and attachment-displacement coefficients
  from the posed belt path. Split a span at a clamp; exclude engaged arcs at loaded pulleys;
  retain the free paths through freely turning idlers. Each span contributes energy
  `EA/(2L) · (delta_length)²`. Preserve pulley coordinates and shared belt identity so the
  solver can distinguish separate loads. Material stiffness and tooth-clearance assumptions
  live in the belt family; overrides have an explicit scope and cannot duplicate a network.
- **Assembly bridge:** carry the derived network into RobotKit with explicit units and joint
  references. Save physical sources and attachments as the authority, and derive the network
  on every rebuild. If a derived snapshot is stored, replace it from those sources on load.
  Keep existing wire ids stable, allocate new ids and bump affected current schema versions;
  reject older schemas as in X9b, without adding compatibility paths.
- **RobotKit:** assemble span energies together with other transmission springs, held-motor
  constraints and the actual mechanical constraints joining screws to a common carriage.
  Apply loads at the coordinates they act on, then eliminate internal coordinates to obtain
  axis deflections. Nominal motion couplings must not rigidly suppress the stretch being
  solved. A shared motor's summed motion relation alone is not a multi-screw constraint.
  Keep existing non-belt drive behavior and the CoreXY coordinate mapping intact.
- **Checks:** verify the analytic two-pulley spring, the three-span matrix above (equal,
  unequal and single-output loads), a free idler, 2–4 screw outputs on a common axis, and a
  carriage belt in series with a reduction. Check signs, units, energy symmetry, force balance,
  invariance to source-record order, and that referencing one belt several times does not
  multiply its stiffness. Reject unresolved attachments and unsupported elastic mechanisms
  with a useful diagnostic instead of treating them as rigid.

X10b implementation: AssemblyDefinition schema 3, MachineKit schema 8 and RobotModel schema 8
carry derived network snapshots with stable new wire ids. Each shaft belt owns its motion
couplings once, and its spans preserve shaft coordinates through the constrained energy solve.
Adjoining spans share one tooth-contact clearance. A stated two-terminal spring scales the
whole network; pairwise stiffness overrides on a multi-output belt are rejected.

The supported shaft-loop model has fixed free-path geometry. Rebuild checks joint perturbations
and authored travel endpoints, rejecting moving tensioners/eccentric paths whose stretch Jacobian
is not represented. Carriage belts remain their existing scalar elastic paths; a separate
shaft-reduction belt and carriage belt combine in the same constrained solve. A loop mixing shaft
reduction and carriage sources is explicitly rejected pending a full geometric span Jacobian,
rather than silently counting their springs twice. This preserves current carriage/CoreXY
baselines and leaves the generic span-coordinate format available for that extension.

Focused analytic checks cover the three-span matrix, load combinations, contact clearance,
source order, serialization, common-axis screw constraints and series compliance. No existing
example baseline intentionally changed in X10b.
The first combined run exposed seven authored RobotModel JSON fixtures at schema v7; X10b's
RobotModel v8 bump requires their current-schema headers to be v8. Their physical data and
numbers are unchanged. RobotKit's full test project passed after the fixture update.

#### X10c — Belt-path cleanup

- `BeltPathRecord.strand` is still a hand-picked index (which span the clamp sits on). Derive it
  from the clamp connector's position: the straight span the clamp point lies on, within a
  tolerance. Reject a clamp that is on no span, or on a wrap.
- `BeltStretch` copies the whole assembly through JSON encode/decode on every resolve. Pose a
  shared copy once per rebuild for all belts, and measure the router's rebuild time before and
  after.
- Record the pose-sampling policy. The existing scalar stiffness is the weakest over sampled
  travel, a conservative constant that overstates mid-travel deviation. For a shared network,
  preserve each complete sampled matrix: independent minima of matrix entries or pairwise
  springs do not establish a conservative network. Evaluate the network at the requested pose,
  or use a documented conservative envelope over complete samples; do not claim an envelope
  covers unsampled travel without justification.

X10c implementation: the stored clamp-strand field (former wire id 3) is removed, with
MachineKit schema 9; its connector is projected onto the unique straight span. A connector on
a tangent/wrap or on no span is rejected. Existing CoreXY/router clamps keep their geometry.
A `BeltPoseContext` clones and unbounds the assembly once per rebuild, then resets its pose
before each belt resolves. It also serves the shaft-loop network and saved-data rebuilds.

Router `describe()` benchmark, three timed rebuilds after one warmup in the same test entry:
screw router 0.027 → 0.036 s/rebuild (wall-time variation; it has no belt path and does not
allocate a pose context); belt router 0.123 → 0.059 s/rebuild (one posed copy serves its three
belts). These are local wall timings, not a real-time guarantee. X9c's carriage stiffness is
the minimum over authored default, ends of travel and any interior balance point. This
constant is conservative for the sampled travel and overstates mid-travel deviation; it does
not prove a bound outside those samples. A shaft network requires fixed free-path geometry over
sampled motion; it rejects moving paths rather than claiming a constant remains valid.

#### X10d — Example

Add a belt reduction to an existing example rather than a new machine. For example, fold the
screw router's Z motor back beside its screw with a 2:1 `BeltReduction` (it then fits under the
gantry), or give the robot arm's wrist a belt stage. Record how Z speed, acceleration and
stiffness change against the direct-coupled Z, with a reason for each.

X10d implementation: `CncRouter(false, true)` and `CncRouterPreview.foldedZRouter()` add an
optional folded Z stage to the existing screw router. A 20-tooth motor pulley drives a
40-tooth screw pulley through a 129-tooth GT2 belt; its two free spans compile into one
elastic network. The screw and motor retain separate rotary joints, and the motor sits on
a machined bracket below the gantry. The original router remains the default. Focused
checks compare the direct and folded robot models, test belt tooth fit and Z kinematics
at three travel poses, check collisions around the folded stage, and exercise the folded
Z stage together with the X/Y carriage belts.
The folded checks live in MachineKit's test source and run from `MachineKitSmoke`;
the router example keeps its existing runtime dependencies for the app's project-source
compiler. A separate app build fix adds the lifecycle's empty `beforeReset` to weld-bead
view state, which has no runtime work to cancel. The router-only source check then passes
with unchanged default plate times of 220.2/201.6 s and belt deviation of 1.76 mm.

At the same poses and limits, direct → folded Z baselines are:

- Z speed 43.654 → 21.827 mm/s: the motor turns twice for each screw revolution, so its
  unchanged rate cap allows half the axis speed.
- Z acceleration 6117.5 → 3263.4 mm/s²: the 2:1 stage reflects more motor inertia into
  the axis; belt loss and the extra hardware also contribute.
- Z stiffness rigid (no finite compliance in the direct-coupler model) → 486.9 MN/m:
  the 129-tooth belt's two actual free spans and the GT2 family's assumed tensile EA
  supply the new elastic compliance.
- Z backlash 0.0500 → 0.0505 mm: the GT2 family's assumed 1%-pitch tooth clearance
  adds 0.0005 mm at the axis through the screw pulley and 2 mm lead.
- Router mass 36.9 → 37.1 kg: pulleys, belt, bearing and the folded motor plate replace
  the direct coupler and standoffs. These are rounded assembly masses.

The belt and its radial screw support are represented as manufacturable parts. The
screw's bearing journal and axial-retention machining are not detailed in this example;
the bearing is fixed in the assembly model. Those details would be required before
fabrication, but they do not affect the modeled drive ratio or belt-span solve.

Final combined gate: screw and belt router plates remain 220.2/201.6 s; belt worst deviation
remains 1.76 mm. CoreXY remains 204.1 rad/s at its motors, 649.6 mm/s at its axes and
71.5/34 m/s² (71.549 m/s² unrounded for X in the app). These are X9 baselines, unchanged
by X10. The optional folded Z values above are the only intentionally changed example
performance figures in this milestone. Router rebuild timings are recorded under X10c.

Validation: one commit per step, focused checks when needed during implementation, and the
combined full suite (`/home/joao/dev/materia-cache/claude-scratch/x7-suite.sh <tag>`) after X10d.
Record the final gate result, every changed baseline with its physical reason, and the router
rebuild timings here. Reconcile any fixes into their owning step commits. Do not claim untested
intermediate commits passed the full suite. No pushing, merging into main or submodule changes.

### Later

Belt teeth drawn and moving with the belt: a mesh built in Haxe and
updated per frame, shifted by the coupled joint's travel. It doubles as a
visual check on a drive's sign and ratio. Also an editor UI to author transmissions, and
differentials and planetaries (relations over more than two joints, beyond X5's summed terms).


### Loose-end follow-through (2026-10-04)

Worktree: `motion-loose-ends`, based on local main `712019dbc`. Work through the ten reviewed loose ends in their original order.

1. **Direct motion and live servo plan checks — implemented.** `PlanCheck` reads a shared polynomial-segment interface; native program arrays remain in place. `MotionSystem` checks a complete direct trajectory once before its first chunk, including smooth replacements. `ServoSession` checks each live plan chunk before submission. Both expose configurable checks and retained findings. Program checks keep their existing policy. Focused MotionKit plan-check gate: 74 assertions passed, including rejection before submission and report-only refill coverage.
2. **Belt-router rapid contact — in validation.** CNC exact stops wait for measured axis positions within 10 µm before advancing. The stock test now requires zero rapid contacts for both routers.
3. **CoreXY motor-space constraints — pending.**
4. **CoreXY belt stiffness — verify current X7/X9 implementation.**
5. **Draw and animate belt teeth — pending.**
6. **Plotter virtual device and app player — pending.**
7. **Native summed-joint kinematics — pending.**
8. **Boards with unused actuator channels — pending.**
9. **Serial-port flake and sim_core_host abort — pending.**
10. **MuJoCo wall-finishing approach failure — pending.**
