---
title: The life of a machine
description: One gantry pick-and-place machine followed from geometry to the controller, naming the concept behind each step.
status: draft
---

# The life of a machine

Materia is a set of kits that each own one part of a machine's life. The fastest
way to see how they fit is to follow one machine through all of them. This page
follows the **gantry picker**: a three-axis Cartesian gantry with a suction cup
that moves six cartons from one table to another.

The picker is built in code (`machinekit/examples/gantry-picker/`), opened in the
editor from the Start page, and played in simulation. Every step below names the
concept it relies on and links to the page that explains it. Where the picker
does not exercise a stage, the step says so and names the example that does.

```
 geometry ─► components ─► assembly ─► scene artifact ─► editor
                              │
                              ├─► kinematics ─► limits
                              │                   │
       mission (pick/place) ──┴─► motion plan ◄───┘
                                      │
                       simulation ◄───┴───► runtime ─► device / controller
```

<!-- figure: one persistent 3D view of the picker that changes with each step
     (scrolling story). Baked from GantryPickerPreview by the docs build. -->

## 1. Geometry: every part is a solid

Each part of the picker is a solid shape, computed when the machine is built.
A table is a union of boxes; a carton is a single box. Shapes come from CadKit,
a headless CAD core: Haxe calls a small C ABI (`cad_box`, booleans, and so on),
and Open CASCADE does the geometry underneath. The same shapes give display
meshes (tessellation) and mass properties (volume times material density).

The picker builds its parts directly in code. It does not use CadKit's
parametric documents, which add features, sketches, recompute and persistent
face names; for those, see the editable examples in `machinekit/examples/` and
[CAD documents](../modelling/cad-documents.md).

**Concepts:** [CAD documents](../modelling/cad-documents.md),
[Identity](../foundations/identity.md)

## 2. Components: parts that know how they attach

A loose solid does not know where it connects to anything. A
**machine component** does: it pairs a geometry generator with named
**connector frames** and a **BOM line** (`MachineComponent`). A carton has a
`base` and a `top` connector; the pick-and-place steps later target `top`.

Most of the picker's components come from catalogs with published dimensions:

- aluminium extrusion for the frame (`HFS5-4040`);
- MGN12C linear rails and their carriage blocks;
- GT2 timing belts on 20-tooth pulleys, with idlers and belt clamps;
- NEMA 23 stepper motors, each with a motor plate, plus a stepper driver chosen
  from the motor's current and the bus voltage;
- inductive home and limit switches on brackets, for every axis side;
- a power supply, a robot flange, and a suction tool (ejector, cup, hose and
  vacuum sensor) whose `contact` frame is the point that touches the carton.

Because every component carries its BOM line, the bill of materials is a
by-product of the model, not a separate list to keep in sync.

**Concepts:** [Machine components](../modelling/machine-components.md),
[Frames and transforms](../foundations/frames.md)

## 3. Assembly: joints and transmissions

Components become a machine by **joining** their connectors. `GantryPicker`
extends `Gantry`, which builds on `AxisBuilder` and `MachineAssembly`:

- **Fixed joints** bolt things together: posts to frame, rail blocks to plates,
  the tool to the flange.
- **Sliding (prismatic) joints** make the three axes. Each is a prismatic joint along a world
  axis with travel limits (1500 × 1000 × 500 mm here), overtravel, and for the
  dual Y axis a racking tolerance. The Z axis slides along −Z, so a positive Z
  coordinate moves the tool down.
- **The dual Y axis has one leader.** The left carriage slides; the right one is
  carried rigidly through the beam. Both sides still get their own belt, motor
  and switches.
- **Transmissions couple joints.** Each belt is modelled as what it is: the
  pulley is a continuous rotating joint coupled to the linear axis by
  `follower = leader × ratio + offset`. The ratio is not typed in; it is
  computed from the parts. A GT2 belt (2 mm pitch) on a 20-tooth pulley moves
  40 mm per turn, so the pulley turns 2π / 40 ≈ 0.157 rad per mm of travel.
- **Motors are actuators on the pulley joint**, not on the axis. The axis is the
  coordinate you plan in; the pulley and motor follow it.
- **Homing order is part of the machine:** Z homes first, then X and Y.

This is [mechanical truth](../modelling/mechanical-truth.md): the model follows
the real mechanism closely enough that limits can be derived from it (step 5).

The finished `MachineAssembly` flattens into a CadKit assembly model: joints,
couplings, switches and actuators. (MachineKit's builder calls joint creation
`addMate`; in the data model, *mates* are placement constraints and *joints*
are degrees of freedom. See [Assemblies and mates](../modelling/assemblies-and-mates.md).)

**Concepts:** [Assemblies and mates](../modelling/assemblies-and-mates.md),
[Transmissions](../modelling/transmissions.md)

## 4. Scene artifact: what the editor loads

The machine lives in code, so nothing assembly-specific is saved next to the
example. Instead, the example's entrypoint (`GantryPickerPreview`) builds the
machine and returns a **scene artifact** (`SceneArtifact`): parts with display
meshes, the assembly definition and its initial state, the robot tool, and the
**mission** (the pick and place steps).

`materia.project.json` tells the editor how to run that entrypoint and which
parts are free bodies (the six cartons). The editor caches the artifact under a
fingerprint of its sources, so reopening an unchanged example skips the rebuild. The
example is listed on the Start page by the example catalog.

Two unit systems meet here. Assembly geometry is authored in millimetres;
everything that moves or is planned is SI (metres, radians, seconds). The
conversion happens once, when the assembly is compiled into a robot model.

**Concepts:** [Data vs. runtime](../foundations/data-and-runtime.md),
[Conventions](../foundations/conventions.md),
[The editor and projects](../platform/editor-and-projects.md)

## 5. Kinematics and limits: from joints to a robot

To plan motion, the assembly is compiled into a **robot model**: one link per
rigid body, one robot joint per movable assembly joint, couplings rescaled into SI.
Joint zero is rebased to the authored starting pose. Two compilers feed one
kinematics core (KinematicsKit): CadKit's assembly kinematics for the editor's
posing, and the simulation bridge for the robot model.

Speed and acceleration limits are **derived, not authored**
(`RobotModel.coupledLimits`). For each axis, Materia walks the couplings from
the axis to its motors and caps:

- velocity by the motor's usable speed divided by the ratio;
- acceleration by the motor's force at the axis, over the carried mass plus the
  inertia reflected through the belt and pulley.

Change the pulley, the motor or the payload and the limits change with them.

**Concepts:** [The kinematic forest](../motion/kinematic-forest.md),
[Machine-axis views](../motion/machine-axes.md),
[Mechanical truth](../modelling/mechanical-truth.md)

<!-- figure (live): pick belt, pulley teeth, motor; show derived limits and the
     torque-speed curve. -->

## 6. Mission and motion: from "pick" to a timed plan

The mission is declarative: twelve steps, each "pick at `box<i>/top`" or
"place at `slot<i>/top`". Each step expands into a short motion program:

1. move linearly to a point 120 mm above the target;
2. move down, pressing 3 mm into the carton;
3. switch the vacuum on (or off), and dwell 0.4 s;
4. move back up.

A **path** says where to go; a **trajectory** adds when. Straight moves are
timed by TOPP-RA, which finds the fastest timing along a fixed path within the
limits from step 5. Free moves between poses are timed by Ruckig, which limits
jerk. The result is an **execution plan** (`ExecutionPlan`): an immutable,
validated set of bounded joint polynomials.

A **plan check** runs on every plan. For the steppers it predicts stalls: is the
torque each motor needs below what it can deliver at that speed? Findings are
attached to the plan (`ExecutionPlan.checked`). By default they are warnings;
callers can ask for rejection.

Before the first step the machine **homes**: each axis runs to its switch in the
order from step 3, and the status bar shows progress.

**Concepts:** [Paths vs. trajectories](../motion/paths-and-trajectories.md),
[Execution plans](../motion/execution-plans.md)

<!-- figure: same path timed by TOPP-RA and by Ruckig, with position, velocity,
     acceleration and jerk synced to a 3D scrubber. -->

## 7. Collision: what is checked, and when

The picker's moves are not collision-planned: clearance comes from the design.
The tables sit low enough, the approach height is high enough, and the switches
are placed out of the tool's sweep. Collision geometry (convex hulls of the
parts) is still used, to tell the physics engine which parts of the gantry may
never collide with each other, and tests check afterwards that nothing touched
that should not have.

Collision-aware planning is used by the gantry welder and the track-mounted arm.

**Concepts:** [Collision model](../modelling/collision.md)

## 8. Simulation: playing the plan

Press **Play** (F5) and the editor runs the mission in simulation. The world
advances in fixed ticks: commands are applied, the physics backend (MuJoCo by
default) steps once, and the new state is published to the scene. The cartons
are free bodies that rest on the tables and can be bumped.

The suction cup is simplified: when the cup touches a free carton and vacuum is
on, the carton is carried with the tool. Suction force is not modelled.

Materia currently runs every simulation at full physical detail. A plan to make
fidelity selectable (kinematic, ideal or physical) is described in
[Fidelity levels](../execution/fidelity.md).

**Concepts:** [Simulation worlds](../execution/simulation-worlds.md),
[Fidelity levels](../execution/fidelity.md),
[Determinism](../foundations/determinism.md)

## 9. Execution: from plan to hardware

In the editor, the picker's plans run on an in-process **robot runtime**: the
same runtime that drives real hardware, attached to the simulated world instead
of a device. The runtime owns the robot, routes commands and executes plans.

Real hardware goes through the same runtime. A **device binding** maps the
model's actuators to controller channels (steps per unit, step-rate caps, skew
bounds between paired motors), and the runtime talks to the controller over
RobotKit's device protocol, either directly over serial or through `robotd`.
The picker does not use this path today; the CoreXY plotter and the 3D printer
examples do, through a virtual device.

**Concepts:** [The robot runtime](../execution/robot-runtime.md),
[The controller boundary](../execution/controller-boundary.md)

## 10. Making it: BOM and fabrication

The picker's bill of materials exists in the model (every component's BOM
line), and tests check it. The picker does not yet produce fabrication output.

The sheet router shows the full path: from the same kind of model it produces a
shop packet with cut lists, nesting layouts, weld joints, sequences and
stitches, and a gated shop release (`machinekit/examples/sheet-router/fabrication/`).

**Concepts:** [Machine components](../modelling/machine-components.md),
[Manufacturing outputs](../process/manufacturing-outputs.md)

---

## Try it

Run the editor from `app/` and pick **Gantry picker** on the Start page:

```sh
../haxeon/scripts/haxeon run --project haxeon.json
```

Press **F5** to home the machine and run the mission. The yaw variant
(`materia.yaw.project.json`) adds a rotating head that turns each carton by 90°,
alternating direction.

The end-to-end checks for this example live in
`app/tests/src/app/ProjectSourceTests.hx` (`checkGantryPicker`). They cover
planner limits staying within the derived limits, placement within 2 mm and 2°,
no unintended contacts, homing time, and no predicted stalls:

```sh
PROJECT_SOURCE_ONLY=gantry ./haxeon/scripts/haxeon run --project app/tests/project-source/haxeon.json
```

## Open questions

Writing this page raised four questions. All four are now answered in
[Conventions](../foundations/conventions.md):

- **Coupling direction**: couplings and transmissions use the same ratio
  (motor-side units per axis-side unit), written from opposite sides.
- **Joint axis**: a unit axis in the parent connector frame, +Y by default.
  `AxisBuilder` uses world-aligned connectors, so its axes are world directions.
- **Joint zero**: robot-model joint zero is the authored initial pose.
- **Units at the seam**: `AssemblySimulationBridge.toRobotModel` converts
  millimetre assemblies to SI.
