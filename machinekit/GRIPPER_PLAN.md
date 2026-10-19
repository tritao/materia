# Electric parallel gripper plan

The tending cell uses an assumed pneumatic parallel gripper (`ParallelGripper`, `ParallelGripperSpec`).
This plan adds our own parametric electric parallel gripper family. It is designed from mechanical truth:
the motor, a self-locking stage, the drive to the jaws and the guide are real components, and grip force,
holding without power, opening and jaw loads are derived from them. The pneumatic family stays as the
baseline. A commercial unit is modelled only as a catalogue envelope for comparison.

Packages involved: `machinekit` (transmission, motion, robotics, examples), `robotkit` (tool commands and
feedback), `simkit` (drive behaviour at Physical fidelity, see `simkit/plans/FIDELITY.md`).

## What exists

- Motors and drives: `NemaStepper` (current-dependent pull-out torque), `ServoMotor`, `MotorDriver`
  (StepDir and Bus control, position-loop rows), `ShaftBrakePart`, `MotorBrakeSpec`, encoders.
- Transmissions: `LeadScrewThread.efficiency(friction)` (driving efficiency only), `LeadScrewNut`,
  `BallScrew`, `Gearbox` (ratio, one efficiency), `GearPair`, `Rack`, `TransmissionRelation` (ratio,
  efficiency, stiffness, backlash, drag, with value bases), `LinearRail`/`LinearRailBlock`.
- Grippers: the pneumatic `ParallelGripper` with jaws, replaceable fingers, support steps, jaw switches
  and contact grip (MT-D: no kinematic hold); `DualArmGripperTool`; `GripperAirRouting`;
  `GripperIntegrityTests`.

## What is missing

- Direction of power flow. A relation has one efficiency, so nothing can say whether a drive can be
  driven backwards, and no part can hold without power.
- A force-limited drive mode: a driver that limits current so that it sets the jaw force, and a force
  estimate with an uncertainty band derived from friction.
- An electric gripper whose ratings are derived through its mechanism.

## Decisions

- EG-D1. **Own design, generic names.** Electric grippers are a MachineKit family named generically,
  like `CobotArm`. Reference designs are studied, not copied. SSG-48 (Apache 2.0) and
  OpenParallelGripper (MIT) may be reused with attribution. MSG (CERN-OHL-S-2.0) and PincOpen (CC BY-SA
  4.0) are share-alike, so we learn from their architecture and derive no geometry from their CAD. DH
  Robotics datasheet rows are used only for the catalogue-envelope comparison (EG7), with their source
  recorded.
- EG-D2. **Fixed architecture, swappable actuator.** The first stage is self-locking (trapezoidal lead
  screw or worm), so holding without power belongs to the mechanism, not the motor. The jaws run on a
  miniature rail; force is set by the actuator's current limit. The output stage (opposite-hand lead screw,
  worm to pinion to twin racks, or one screw plus a rack-and-pinion mirror) is chosen in EG3. This is DH's
  structure (self-locking stage, rack and pinion, T-slot guide, current-set force). The actuator is a
  parameter of the gripper spec with two kinds:
  - **FOC stepper** (closed-loop NEMA 14 class with field-oriented control): built first, because its
    data is standard and sourced, its driver is open and `NemaStepper` already models it.
  - **Brushless smart servo** (24 V RS485 class, e.g. Feetech SM or Dynamixel P sizes, named
    generically): likely lighter and simpler to wire, but its gearbox backdrives and its data is mostly
    assumed.
  The tending cell picks between them on measured results (EG6), not on paper.
- EG-D3. **Efficiency has a direction.** `TransmissionRelation` gains a backdrive efficiency. Self-locking
  is derived (backdrive efficiency ≤ 0 at the static friction coefficient with a stated margin); it is
  never a flag. A lead screw backdrives at tan(λ − φ) / tan λ; a worm uses the same law on its lead
  angle; ball screws, spur gears and racks are backdrivable with assumed efficiencies. A chain
  self-locks if any stage does. Vibration-induced creep stays an assumption on the basis label.
- EG-D4. **Force comes from current, as a band.** Jaw force = motor torque at the current limit × ratio
  × forward efficiency, evaluated over the friction range, so the rated force is a band, not one number.
  Holding force without power is derived: up to the structural rating if the drive self-locks, from the
  brake or detent torque through the backdrive path if it does not.
- EG-D5. **Grip stays contact physics.** As in tending (MT-D), the jaws hold by force and friction. The
  jaw joint gets a force-limited actuator from EG-D4. At Physical fidelity the drive backdrives according
  to EG-D3, so a power loss keeps or drops the part as the mechanism says.
- EG-D6. **Shared jaws and fingers.** Fingers, pads, support steps, grip windows and the expected-part
  geometry are shared by the pneumatic and electric families; only the actuation differs. One spec
  carries the shared part; no copy per family.
- EG-D7. **Feedback is derived; the tool flange is the interface.** The electric family reports
  gripped, position and dropped from encoder position and motor current, not reed switches. The driver
  sits inside the gripper. Power is 24 V and control is RS485 Modbus RTU, which cobot tool connectors
  provide; CAN is an option. With tool-flange I/O there is no air and no cabinet run; otherwise services
  are routed like `GripperAirRouting` with cable instead of hose.
- EG-D8. **Smart servos are compositions.** An integrated servo is a motor, a backdrivable gearbox, a
  built-in driver and a bus port, built from EG1/EG2 parts, with each sourced or assumed value labelled.
  Brushless is the gripper candidate; brushed hobby servos (STS/XL class) are only for EG-L1.

## Steps

**EG0. Requirements from the cell.** Derive, don't type in, the load cases from the tending cell: blank
mass (60 × 60 × 20 mm 6061), peak TCP accelerations from the actual missions, vise seating force,
fingertip opening needed for the approach clearances (check the ~76 mm estimate), jaw moments in the vise
push, and the wrist payload budget for the dual tool on the chosen cobot class. Record them as a checked
requirements record. Score the pneumatic baseline against it.

**EG1. Directional efficiency.** Backdrive efficiency and its basis on `TransmissionRelation`; backdrive
on `LeadScrewThread`/`LeadScrewNut`, `Gearbox`, `GearPair`, `Rack`, `BallScrew`; a new worm gear pair in
`machinekit.transmission`; series composition in the resolver (product of efficiencies, chain self-locks
if any stage does). Tests: Tr8×2 self-locks, Tr8×8 does not, ball screw does not, worm locks below its
friction angle, chain composition.

**EG2. Force-limited actuators.** A current-limit mode on `MotorDriver` with an assumed FOC closed-loop
stepper row (Bus control); stepper torque from current limit with assumed detent/ripple. An integrated
smart-servo actuator (EG-D8) with a brushless 24 V RS485 row. Both expose one actuator interface to the
gripper. The jaw-force band and unpowered holding force functions from EG-D4. Tests on known
screw/motor and servo/screw pairs, including that a servo without the self-locking stage does not hold.

**EG3. Mechanism trade study.** Compare the three output layouts in EG-D2 on mass, body length for the
EG0 opening, force band, unpowered hold, backlash, jaw-load capacity and part count, with EG1/EG2
numbers, for both actuator kinds. Pick one layout that fits both and record it as EG-D9.

**EG4. Electric parallel gripper.** Built with the stepper first, then the servo as a spec
change on the same assembly. Refactor the shared jaw/finger spec out of `ParallelGripperSpec`
(EG-D6). Build the electric assembly: housing, rail and blocks, motor, self-locking stage, output stage,
encoder, connector. Mates and couplings come from the resolved transmission relations, not a typed-in
mirror. Derived ratings: stroke, fingertip opening, force band, speed, unpowered hold, mass. Integrity
tests next to `GripperIntegrityTests`: opening ≥ EG0 requirement, no interference over the stroke, jaw
loads within the rail rating, hold without power.

**EG5. Runtime and simulation.** RobotKit tool commands: position, speed and force limit; feedback:
gripped, position, dropped (EG-D7). Physical fidelity drive model per EG-D3/EG-D5. Tests: the grip holds
the blank through the mission accelerations at the low end of the force band; on power loss the
self-locking gripper holds and a backdrivable variant drops.

**EG6. Tending cell integration.** A `DualArmGripperTool` configuration with electric grippers on
tool-flange power and RS485. Run the tending missions with pneumatic, electric-stepper and
electric-servo grippers and report cycle time, wrist mass and payload margin, door and vise clearance,
grip margin and the share of assumed values. Choose the tending actuator from this and record it as
EG-D10.

**EG7. Catalogue comparison.** A generic catalogue-envelope electric gripper with PGC-140-class values
(datasheet rows with source, no vendor geometry or name) in the same dual assembly. Comparison table:
pneumatic, own electric, catalogue envelope.

**EG8. Build path (optional).** Manufacturable parts through the AM and sheet-metal kits, BOM and cost
roll-up, and a build or buy decision gate. No hardware commitment before this step.

**Later.**
- EG-L1. A low-cost printed gripper on a brushed hobby servo (PincOpen/OpenParallelGripper class).
- EG-L2. BLDC force-control variant with a separate gearhead and open driver (backdrivable, current as
  a good force sensor) for fragile or varied parts.
- EG-L3. Adaptive or compliant fingers.

**Order.** Starts after the in-progress pneumatic gripper work (`ParallelGripper*`, `GripperAirRouting`,
`GripperIntegrityTests`) lands on main. EG1 and EG2 are independent of the cell and can start first;
EG0 runs in parallel with them. Then EG3 -> EG4 -> EG5 -> EG6 -> EG7, and EG8 optional.
