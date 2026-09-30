# Six-axis robot arm

A serial arm on a pedestal, generated from MachineKit parts and simulated in
Materia. From the repository root:

```sh
./haxeon/scripts/haxeon build --project=app/haxeon.json
./app/run-built.sh --project=./machinekit/examples/robot-arm/materia.project.json
```

Or open **Six-axis robot arm** from the Start page. Press **Play** in the
toolbar to run the arm's pick motion: it swings to one side, lowers the tool,
lifts, swings to the other side while turning the tool, lowers it again, and
returns.

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
Schmalz ejector, SAF 40 cup, push-in fitting and hose. The arm includes it as
`tool/...` and mates its plate to the flange's pilot boss. The assembly exposes
the cup's `toolContact` connector and the ejector's `compressedAir` inlet. The
tool is geometry and ports only: nothing here simulates vacuum or grips a part.

## Motion

`materia.project.json` names `robot-arm.motion.json` in `robotMotions`. The
file lists one looping track per joint, keyed by the joint's assembly id:

```json
{"version": 1, "tracks": [{"joint": "j1", "loop": true, "keys": [{"time": 0, "position": 0}, ...]}]}
```

Positions are relative to the generated initial pose, like every Materia motion
track, so `0` means the ready pose. A looping track must end where it starts.
The tracks are installed with the project and drive the arm's own assembly
model in the simulation; they are saved with the document like any other track.

## Checks

```sh
./haxeon/scripts/haxeon build --project=machinekit/examples/robot-arm/haxeon.json
./haxeon/.tools/hashlink/hl machinekit/examples/robot-arm/build/host/main.hl
```

The check generates the preview and verifies forward kinematics: with every
joint at zero the tool flange face is at (35, 0, 1306.5) mm, pointing up, and in
the ready pose it points down with the cup contact face below it.
