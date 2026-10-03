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
  stepper keeps its bytes and older models decode.
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

Status: T0–T4 done (2026-10-03); T5 not started.
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

- A small wire enum `ValueBasis { Derived; Catalog; Stated; Assumed; }` and, on the resolved
  relation and on a motor's actuator, the set of fields whose basis is `Assumed`. Not a wrapper
  around every float: generics and the wire format make `EngineeringValue<T>` costly for little gain.
- Carried as an optional `assumed:ReadOnlyArray<String>` on projectkit `AssemblyJointCoupling` and
  `AssemblyActuator`, then RobotKit `JointCoupling`/actuator via the CadKit bridge, then the `PlanCheck`
  diagnostics: "predicted deviation 1.9 mm (assumed: belt stiffness, drag)". Written only when
  non-empty, so other models keep their bytes.
- Sources of `Assumed` today: the T3 family defaults, `TimingBelt.cordStiffnessPerMm`,
  `NemaStepper.ratingCatalog` inductance/rotor inertia, the generic `ServoMotor` ratings.

### X8 — Motor, driver and controller

Planned (2026-10-03); after X7. Today one call mixes three pieces of hardware:
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
- **Gearbox**: a member reference (the `Gearbox` part exists), not `gearRatio`/`gearEfficiency`
  numbers on the motor record.
- **Controller** (the board: step tick rate, bus cycle, channel count, identity) stays deployment
  data (`SerialDeployment`, the job's `controller.stepTickHz`). The model never carries it. The
  binding checks the driver's maximum step rate against it.
- **Encoders** stay sensors (X6d). A servo's built-in encoder is a spec on the motor that adds an
  encoder sensor on its joint when no separate encoder is wired.

Steps: X8a driver part + catalog; X8b `addMotor` with a driver, the curve from motor + driver, and
microsteps in the actuator (MotorRecord schema bump, router/CoreXY/arm/base updated); X8c supply via
ports; X8d gearbox member; X8e job/deployment cleanup (`controller.microsteps` removed, step-rate check).

### Later

Belt teeth drawn and moving with the belt: a mesh built in Haxe and
updated per frame, shifted by the coupled joint's travel. It doubles as a
visual check on a drive's sign and ratio. Also an editor UI to author transmissions, and
differentials and planetaries (relations over more than two joints, beyond X5's summed terms).
