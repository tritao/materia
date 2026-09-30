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

`RobotArm` is a `MachineAssembly` of 14 parts: a `Pedestal` with its
`RobotFlange`, six joint modules, five links, and the tool `RobotFlange`. The
arm has the classic shoulder, elbow and spherical-wrist layout:

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

Two generators live beside the assembly, because they only make sense for this
arm so far:

- `ArmJoint` is a cylindrical joint module. Its `stator` face is fixed to the
  link before it and its `rotor` face carries the next link. The last module
  takes a `RobotFlange` instead and cuts the flange's pilot and bolt pattern
  into its output end.
- `ArmLink` is a hollow tube closed at both ends, with a collar where it meets
  the previous joint. Its `start` and `end` connectors point along the joint
  axes given by `ArmAxis`; a lateral end joint is centred on the tube's end.

Every housing belongs to the link before it, so each revolute mate joins a
housing's `rotor` to the next link's `start`. Joint limits, speeds and torques
come from `RobotArm.specs`. Housings are steel and links are aluminium, so the
simulation gets a realistic mass distribution (about 20 kg above the base).

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
joint at zero the tool flange is at (35, 0, 1299) mm, pointing up, and in the
ready pose it points down.
