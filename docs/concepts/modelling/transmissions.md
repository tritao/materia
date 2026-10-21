---
title: Transmissions
description: Screws, belts, gears, racks and chains as coupled joints, with ratios, efficiencies and limits derived from the parts.
status: draft
---

# Transmissions

## The problem

A motor turns; an axis slides. Between them is a lead screw, a belt, a rack,
a gear train or a chain, and that transmission decides almost everything about
how the axis behaves: how fast it can go, how hard it can push, how much it
stretches, how much play it has.

The usual shortcut is to model the axis alone and type in a maximum speed and
acceleration. Those numbers are then guesses, and they stay wrong when the
motor, the pulley or the load changes.

## The model

Materia models the transmission as it is built, then derives the numbers.

### A transmission is a coupling between two joints

Every rotating part of the drive is its own joint: the screw, the pulley, the
motor shaft. A **transmission** couples a **follower** joint to a **leader**
joint: `follower = ratio × leader + offset`. The **leader is the coordinate
motion is planned in**, usually the axis; the screw, pulley and motor follow
it. That is a statement about planning, not about cause: the motor still
drives the axis.

There is no special "helical joint" for a screw. A screw is a rotating joint,
its nut rides a sliding joint, and a coupling ties them together. The same
primitive covers every kind of drive.

| Transmission | Couples | Ratio from |
|---|---|---|
| `LeadScrew(screw, nut)` | the screw's rotation to the nut's slide | the thread's lead and hand |
| `TimingBelt(belt, pulley)` | the pulley's rotation to the clamped belt | the pulley's pitch diameter |
| `RackAndPinion(pinion, rack)` | the pinion's rotation to the rack | the pinion's pitch diameter |
| `GearMesh(driver, driven)` | two gears' rotations | the tooth counts |
| `RollerChain(chain, sprocket)` | the sprocket's rotation to the chain | the sprocket's pitch diameter |
| `BeltIdler(belt, pulley)` | an idler to its belt (bearing drag only) | the pulley's pitch diameter |
| `BeltReduction(belt, pulley, pulley)` | two shafts through a belt | the tooth counts |

**Ratios come from the parts**, never from a typed number. A GT2 belt on a
20-tooth pulley gives 2π per 40 mm; swap the pulley and the ratio, the limits
and every plan change with it. The resolver also checks the parts fit each
other: matching threads, belt and pulley profiles, gear modules and pressure
angles, chain pitches. A mismatch is a diagnostic on the assembly.

A follower can have several leaders: a CoreXY motor follows both axes, and its
position is the sum of one coupling per leader (`follower = Σ ratioᵢ ×
leaderᵢ + offsetᵢ`). The ratio's units and sign are defined once in
[Conventions](../foundations/conventions.md#couplings-and-transmissions).

### What else a transmission carries

Each transmission also records, from its parts:

- **efficiency**: 0.98 for a gear mesh, 0.97 for a belt, a lead screw's from
  its thread and nut type;
- **stiffness**: a belt's comes from its actual path and how much belt is free
  between pulley and clamp, at the worst point of the axis's travel; a ball
  screw's from the shaft, nut and support bearings in series;
- **backlash** and **drag**: from the nut type, the gears or the rack;
- **speed caps**: a long, thin screw whips above its critical speed, so a
  supported screw caps its own speed.

Every value records its **basis**: derived from the parts, from a catalog,
stated by the designer, assumed, or measured on the machine. A stated or
measured value overrides the derived one explicitly, never silently.

### Motors are actuators on a joint

A motor is attached to the joint it actually turns (for a belt axis, the
pulley's joint, not the axis) together with its **driver**. The motor's model
comes from its catalog entry and from the driver and supply it is wired to:

- **Steppers** get a pull-out torque curve: full holding torque up to a corner
  speed set by the winding and the supply voltage, falling beyond it. A safety
  margin (0.5 by default) keeps planned motion well inside the curve. The
  driver adds its microstepping and its maximum step rate.
- **Servos** get their peak and rated torque and their speed.
- A **gearbox** between motor and joint adds its ratio, efficiency and inertia.

### Limits are derived

From all of this, Materia derives each axis's limits (`RobotModel.coupledLimits`):

- **velocity**: the slowest of every coupled joint's own limit, each motor's
  usable speed through its ratio, the driver's step-rate ceiling, and the
  screw's critical speed;
- **acceleration**: the motors' force at the axis (through each coupling's
  efficiency) less steady loads such as rail friction and weight, divided by
  the carried mass plus every turning part's inertia seen through its ratio.

Try it on a belt-driven axis. The limits below are computed in your browser by
RobotKit's `coupledLimits`, from a robot model of a carriage, a GT2 belt, a
pulley and a stepper. A bigger pulley moves the carriage further per turn, so
the axis gets faster but the motor's torque pushes less hard: the ratio trades
acceleration for speed. A heavier carriage only costs acceleration.

<figure class="materia-figure materia-widget" data-widget="belt">

| Pulley | Carriage | Motor | Speed | Acceleration |
|---|---|---|---|---|
| 20 teeth | 2 kg | NEMA 17, 0.45 N·m | 1.53 m/s | 31.4 m/s² |

<figcaption>Derived limits of a belt-driven axis, computed live by RobotKit. The motor's torque holds to its corner speed and falls as 1 / speed beyond it. Without WebAssembly GC, the table shows one case.</figcaption>
</figure>

### Checking a plan against the drives

Limits keep plans reasonable; a **plan check** confirms them. For every sample
of a planned motion it asks: is each stepper under its pull-out torque at this
speed, is each servo under its peak and RMS torque, does the drive stretch or
lag beyond tolerance? Findings are attached to the plan (see
[Execution plans](../motion/execution-plans.md)). In simulation, a stepper
asked for too much **slips**: it loses the steps the check predicted, and the
axis ends up where a real one would.

## In Materia

| Piece | Where |
|---|---|
| Transmission kinds and sense | `machinekit.assembly.Transmission`, `Sense` |
| Adding drives | `MachineAssembly.addTransmission`, `addMotor`, `addEncoder`, `addBeltPath`, `supportScrew` |
| Ratio, efficiency, stiffness, backlash, drag | `TransmissionResolver`, `TransmissionRelation`, `ValueBasis` |
| Motor and driver models | `NemaStepper`, `LeadshineServo`, `ServoMotor`, `MotorDriver`, `MotorDrive` |
| Drive kinds in the robot model | `ActuatorDrive`: `StepperDrive`, `ServoDrive`; `TorqueSpeedCurve` |
| Derived limits | `RobotModel.coupledLimits`, `SteadyLoads` |
| Plan check and slip | `PlanCheck`, `PlanSlip`, `StepperSlip` |
| Decisions | `machinekit/TRANSMISSION_PLAN.md` (X0–X10) |

Encoders sit on a joint too, and the joint decides what they see: on the
motor's own joint an encoder sees lost steps; on a joint beyond the
transmission it sees stretch and backlash.

## What it isn't

- **Not a ratio typed into the robot.** The assembly stores plain coupling
  numbers, but MachineKit derives them from parts and refuses manual edits to a
  coupling a transmission owns.
- **Not cause and effect.** The leader is the planned coordinate. The motor is
  where force comes from.
- **Not a force simulation of the stepper.** In simulation a stepper follows its
  commands and slips only where the plan check says it would; servos are
  simulated as torque-limited motors.

## See also

- [The life of a machine](../start/life-of-a-machine.md), steps 3 and 5
- [Mechanical truth](mechanical-truth.md)
- [Machine components](machine-components.md)
- [Execution plans](../motion/execution-plans.md)

## Open questions

- **`TRANSMISSION_PLAN.md` describes earlier names.** X2–X6 mention `Drive`,
  `addDrive` and `DriveDefaults`, renamed or removed since (X7, T3).
- **Inferred servo encoders.** The plan says encoders must be wired
  explicitly, but a servo with encoder counts and no wired encoder still gets
  one at compile time.
- **`BeltReduction`'s pulleys** are `leaderPulley`/`followerPulley` in the type
  and `driver`/`driven` in the plan and resolver.
- **The `coupledLimits` doc says gravity and friction are left out**, which is
  true only when no steady loads are passed.
