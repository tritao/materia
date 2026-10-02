# Six-axis robot arm

A serial arm on a pedestal, generated from MachineKit parts and simulated in
Materia. From the repository root:

```sh
./haxeon/scripts/haxeon build --project=app/haxeon.json
./app/run-built.sh --project=./machinekit/examples/robot-arm/materia.project.json
```

Or open **Six-axis robot arm** from the Start page. Press **Play** in the
toolbar to run the arm's pick-and-place cycle: the suction cup picks a block off
one pad, carries it to the other, sets it down, and then carries it back.

## What it contains

`RobotArm` is a `MachineAssembly` of 20 parts: a `Pedestal` with its
`RobotFlange`, six joint modules, five links, the tool `RobotFlange`, and a
suction tool. The arm has the classic shoulder, elbow and spherical-wrist
layout:

| Joint | Axis at the zero pose | Range (rad) |
| ----- | --------------------- | ----------- |
| `j1`  | vertical (base yaw)   | ±2.9        |
| `j2`  | horizontal, shoulder  | ±1.9        |
| `j3`  | horizontal, elbow     | ±2.4        |
| `j4`  | along the forearm     | ±3.1        |
| `j5`  | horizontal, wrist     | ±2.1        |
| `j6`  | along the tool        | ±6.2        |

At zero on every joint the arm points straight up; the saved pose is a ready
pose with the tool pointing down.

Every housing belongs to the link before it, so each revolute mate joins a
housing's `rotor` to the next link's `start`. Joint limits, speeds and torques
come from `RobotArm.specs`. Housings are steel and links are aluminium, so the
simulation gets a realistic mass distribution (about 20 kg above the base).

`ArmJoint` and `ArmLink` are MachineKit library parts (`machinekit.robotics`) with editable
recipes, so a different arm is a matter of parameters:

- `ArmJoint` is a cylindrical joint module. Its `stator` face is fixed to the
  link before it and its `rotor` face carries the next link. The last module
  takes a `RobotFlange` (by ISO 9409-1 pitch circle) instead and carries it
  against the housing end, pilot boss outward, ready for a tool plate.
- `ArmLink` is a hollow tube closed at both ends, with a collar where it meets
  the previous joint. Its `start` and `end` connectors point along the joint
  axes given by `ArmAxis`; a lateral end joint is centred on the tube's end.

## Suction tool

`ArmSuctionTool` builds an `EndEffector` in the style of `examples/eoat`: an
`EndEffectorPlate` bolted to the tool flange, a `FrameBar`, and a catalog
Schmalz ejector, SAF 40 cup, push-in fitting and hose, with an inline
`VacuumPressureSensor` between the ejector and the hose. The arm includes it as
`tool/...` and mates its plate to the flange's pilot boss. The assembly exposes
the cup's `toolContact` connector and the ejector's `compressedAir` inlet.

The tool's capabilities say how it is worked: `AssemblyPreview.robotTools` derives the scene's
`robotTools` from them (the cup's contact, the ejector's vacuum channel, the sensor's pressure
signal), so the simulation, the skills and a real controller name the same channels.

## The work cell and the mission

The arm stands in front of a table with two pads and a workpiece block, all in the assembly.
`RobotArm` holds their coordinates (`TABLE_TOP`, `PICK_X`, `PLACE_X`, `WORK_Y`). The table and
pads are fixed roots, like the pedestal.

The workpiece is different: `materia.project.json` lists it in `dynamicParts`, which frees it from
the assembly. The simulation gives it no link on the arm's robot and instead simulates it as a
dynamic box that rests on a pad under gravity and collides with everything.

The scene's `mission` is the arm's work, on repeat: pick the workpiece by its `top`, place it on
`padPlace`'s `top`, then pick it again and put it back on `padPick`. Each step is RobotKit's
`HandlePart` skill: MotionKit plans the approach, the press and the retreat from where the
workpiece and the pad actually are, the ejector's channel switches the vacuum, and the sensor's
reading tells whether the cup sealed. Nothing about the cycle is authored as joint motion, so
moving a pad or the workpiece in the assembly moves the work with it.

The simulated vacuum holds a free object the cup is touching to the cup's link while the channel
is on, and reads a sealed pressure on the sensor while it holds one. It is a kinematic attachment,
not a suction model: there is no evacuation time, leakage, or load limit.

## Checks

```sh
./haxeon/scripts/haxeon build --project=machinekit/examples/robot-arm/haxeon.json
./haxeon/.tools/hashlink/hl machinekit/examples/robot-arm/build/host/main.hl
```

The check generates the preview, checks it carries the tool with its sensor and the four-step
mission, and verifies forward kinematics: with every
joint at zero the tool flange face is at (35, 0, 1306.5) mm, pointing up, and in
the ready pose it points down with the cup contact face below it.
