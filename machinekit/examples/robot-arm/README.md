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
Schmalz ejector, SAF 40 cup, push-in fitting and hose. The arm includes it as
`tool/...` and mates its plate to the flange's pilot boss. The assembly exposes
the cup's `toolContact` connector and the ejector's `compressedAir` inlet. The
tool is geometry and ports only: nothing here simulates vacuum or grips a part.

## The work cell and the grip

The arm stands in front of a table with two pads and a workpiece block, all in the assembly.
`RobotArm` holds their coordinates (`TABLE_TOP`, `PICK_X`, `PLACE_X`, `WORK_Y`) so the arm, the
cell and the motion authoring agree. The table and pads are fixed roots, like the pedestal.

The workpiece is different: `materia.project.json` lists it in `dynamicParts`, which frees it from
the assembly. The simulation gives it no link on the arm's robot and instead simulates it as a
dynamic box that rests on a pad under gravity and collides with everything.

The motion file's `grips` list is the vacuum: `grip` and `release` commands for the link that
carries the cup. When a `grip` fires, the simulation looks for a free object touching that link;
if there is one it holds the object to the link at the offset it had (`SimSession.holdObject`),
with a 1 mm gap, so the cup carries it without pushing back on the arm. A `release` lets it go
as an ordinary dynamic body. With nothing under the cup the vacuum finds no seal and holds
nothing. The hold is a kinematic attachment, not a suction model: there is no evacuation time,
leakage, or load limit, and the workpiece's weight does not load the arm.

## Motion

`materia.project.json` names `robot-arm.motion.json` in `robotMotions`. The file lists one
looping track per joint, keyed by the joint's assembly id, and the vacuum commands that go with
them, with times in the same clock:

```json
{"version": 1,
 "tracks": [{"joint": "j1", "loop": true, "keys": [{"time": 0, "position": 0}, ...]}],
 "grips": [{"time": 3.0, "link": "tool/cup", "action": "grip"}, ...]}
```

Positions are relative to the generated initial pose, like every Materia motion track, so `0`
means the ready pose. A looping track must end where it starts. The tracks are installed with the
project and drive the arm's own assembly model in the simulation; they are saved with the
document like any other track.

The motion is authored in Cartesian space, not by hand. `authoring/ArmMotionAuthoring.hx` builds
the arm's kinematic model from its assembly (the same bridge the simulation uses), takes the
suction cup's contact face as the tool point, and walks it through waypoints: home, above the
pick, touch (vacuum on), lift, above the place, touch (vacuum off), retreat, home, and then the
same back the other way. Moves are straight lines at a fixed
downward orientation, sampled every 2 cm; motionkit's numeric inverse kinematics solves each
sample from the previous one, and each step's duration respects both the tool speed and half of
every joint's velocity limit. The vacuum commands are emitted at the moments the tool has settled
on a pad. To change the cycle, edit the waypoints and regenerate:

```sh
./haxeon/scripts/haxeon build --project=machinekit/examples/robot-arm/authoring/haxeon.json
./haxeon/.tools/hashlink/hl machinekit/examples/robot-arm/authoring/build/host/main.hl \
  write machinekit/examples/robot-arm/robot-arm.motion.json
```

## Checks

```sh
./haxeon/scripts/haxeon build --project=machinekit/examples/robot-arm/haxeon.json
./haxeon/.tools/hashlink/hl machinekit/examples/robot-arm/build/host/main.hl
```

`authoring/…/main.hl check <file>` (also run by `machinekit/scripts/test-haxeon`) re-plans the
motion and fails if the committed file differs or the tool misses a waypoint, so the file cannot
drift from the geometry.

The check generates the preview and verifies forward kinematics: with every
joint at zero the tool flange face is at (35, 0, 1306.5) mm, pointing up, and in
the ready pose it points down with the cup contact face below it.
