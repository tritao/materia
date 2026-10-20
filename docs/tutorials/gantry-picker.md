---
title: "Your first machine: a gantry picker"
description: Build Materia, open a three-axis gantry, run its pick-and-place mission in simulation, and find where each part of it is defined.
status: draft
---

# Your first machine: a gantry picker

In this tutorial you build Materia from source, open the **gantry picker**
example, run its pick-and-place mission in simulation, and read the code that
defines it. It takes about half an hour, most of it the first build.

The gantry picker is a 1500 × 1000 × 500 mm belt-driven Cartesian gantry with a
suction cup. It moves six cartons from an infeed table to a two-row pallet
pattern. [The life of a machine](../concepts/start/life-of-a-machine.md)
explains the same machine conceptually; this page is hands-on.

<a class="open-in-materia" data-example="gantry-picker" hidden>Open the gantry picker in the browser</a>

## Before you start

You need Linux with Git, a C/C++ toolchain and CMake, and at least 10 GB of
free disk space for the sources, tools and first build.

## 1. Get the code and build the editor

Clone the repository with its submodules:

```sh
git clone --recurse-submodules https://github.com/tritao/materia.git
cd materia
```

Install the pinned Haxeon tools and check your machine:

```sh
./haxeon/scripts/bootstrap-tools.sh
./haxeon/scripts/haxeon doctor
```

Build the reference editor:

```sh
./haxeon/scripts/haxeon build --project app/haxeon.json
```

The first build compiles the native libraries (Open CASCADE, MuJoCo and the
kits) and takes a while. Later builds reuse them.

## 2. Open the gantry picker

Start the editor with the example's project file:

```sh
./app/run-built.sh --project=./machinekit/examples/gantry-picker/materia.project.json
```

You can also start `./app/run-built.sh` on its own and pick **Gantry picker**
on the Start page.

The editor runs the example's code, which builds the machine and hands the
editor a [scene artifact](../concepts/start/life-of-a-machine.md#4-scene-artifact-what-the-editor-loads):
the parts with their meshes, the assembly, the suction tool and the mission.
The first open builds the example; later opens reuse the cached result.

Take a look around before running anything:

- The **frame** is aluminium extrusion. The **X beam** rides on two **Y rails**,
  and a **Z column** hangs from the beam's carriage.
- Each axis is driven by a **GT2 timing belt** from a **NEMA 23 stepper**.
- Small **inductive switches** sit at the ends of every axis: home and limit
  switches.
- The **suction cup** hangs under the Z column. The left table holds six
  cartons; the right table has six empty seats.

Select a part to see it in the **Inspector**, including the joints that move it.

## 3. Run the mission

Press **Play** (F5).

1. **The machine homes first.** Z retracts, then X and Y run to their home
   switches one axis at a time. The status bar names the axis and the phase.
   This takes about nine seconds of simulated time.
2. **Then it picks and places.** For each carton: move above it, press down,
   switch the vacuum on, lift, carry it across, set it down on its seat, switch
   the vacuum off, lift again.

What you are watching is not an animation. Every move is a planned, validated
[execution plan](../concepts/motion/execution-plans.md), run by the same robot
runtime that drives real hardware, inside a MuJoCo physics simulation. The
cartons are free bodies: a bad move would knock them over.

## 4. Read the machine's code

The whole machine is defined in two files in
`machinekit/examples/gantry-picker/`.

`GantryPicker.hx` describes the hardware. The gantry itself comes from
MachineKit's `Gantry`, configured by a `GantrySpec`:

<!-- snippet: machinekit/examples/gantry-picker/GantryPicker.hx#L25-L40 -->
```haxe
	public function new(?headKind:GantryHead = GantryHead.None) {
		var belt:GantryDrive = Belt(TimingBeltProfile.GT2, 20, 9);
		var driveBus = headKind == GantryHead.None ? new machinekit.power.DcVoltage(24) :
			machinekit.motion.ServoMotor.LOW_VOLTAGE_BUS;
		// The suction tool projects in front of the slide. Keep the front crossbar
		// beyond its vertical sweep, including the guide's homing overtravel.
		// A rotating carton needs its diagonal reach, preserving the original
		// clearance to the Z switch mount during turns at the home height.
		super(new GantrySpec(1500, 1000, 500, belt, belt, belt, true, "MGN12C", 23,
			"HFS5-4040", "HFS5-4040", false, headKind, 0.5, driveBus, 16, 200, 100, 50,
			headKind == GantryHead.None ? 60 : 60 + BOX_WIDTH * (Math.sqrt(2) - 1) / 2));
		var flange:RobotFlange = head == null ? cast component("flange") : head.flange;
		tool = new SuctionTool(flange);
		include("tool", tool);
		var output = connector("toolFlange");
		addMate("tool-mount", "fixed", output.instanceId, output.connectorName, "tool/plate", "robot");
```

- `Belt(TimingBeltProfile.GT2, 20, 9)`: every axis uses a GT2 belt on a
  20-tooth pulley. From that, Materia derives the
  [transmission ratio](../concepts/foundations/conventions.md#couplings-and-transmissions)
  (2π per 40 mm of travel), and from the ratio and the motor, each axis's
  speed and acceleration limits.
- `GantrySpec(1500, 1000, 500, …, "MGN12C", 23, "HFS5-4040", …)`: travel per
  axis in millimetres, the linear rail, the NEMA motor size and the extrusion
  profile.
- `SuctionTool(flange)` is a catalog tool assembly, fixed to the gantry's
  flange by a joint (`addMate("tool-mount", "fixed", …)`; despite its name,
  `addMate` creates a [joint](../concepts/modelling/assemblies-and-mates.md)).

`GantryPickerPreview.hx` turns the machine into a scene and gives it its job:

<!-- snippet: machinekit/examples/gantry-picker/GantryPickerPreview.hx#L10-L24 -->
```haxe
	static function build(yaw:Bool):Bytes {
		var picker = new GantryPicker(yaw ? machinekit.gantry.GantrySpec.GantryHead.C : machinekit.gantry.GantrySpec.GantryHead.None);
		var scene = AssemblyPreview.scene(picker, "gantry-picker");
		scene.robotTools = machinekit.robot.RobotScene.robotTools(picker.tool, "tool");
		var steps:Array<materia.project.SceneArtifact.SceneArtifactMissionStep> = [];
		for (index in 0...GantryPicker.BOX_COUNT) {
			var pick:materia.project.SceneArtifact.SceneArtifactMissionStep = {kind: "pick", at: {occurrence: "box" + index, connector: "top"}};
			var place:materia.project.SceneArtifact.SceneArtifactMissionStep = {kind: "place", at: {occurrence: "slot" + index, connector: "top"}};
			if (yaw) { pick.yaw = 0.0; place.yaw = (index % 2 == 0 ? 1 : -1) * Math.PI / 2; }
			steps.push(pick); steps.push(place);
		}
		scene.missions = [{loop: false, steps: steps,
			powerUpSideOffsets: [{homeSwitch: "switchYRighthome", offset: 0.001}]}];
		return SceneArtifact.encode(scene);
	}
```

The mission is declarative: "pick at `box0/top`, place at `slot0/top`", and so
on. Each step names a [connector](../concepts/modelling/machine-components.md)
on a part. Planning turns the steps into motion when you press Play.

## 5. Try a variant

The example has a second project file with a rotating C axis on the head. It
turns each carton by 90°, alternating direction:

```sh
./app/run-built.sh --project=./machinekit/examples/gantry-picker/materia.yaw.project.json
```

Press Play again and compare: the same mission, with one more joint to plan.

## 6. Run its test

Every example is also a test. The gantry picker's checks build the machine,
run the mission in MuJoCo, and check that every carton lands within 2 mm and
2°, that nothing touched that should not have, that homing finishes in time,
and that no stepper was asked for more torque than it has:

```sh
PROJECT_SOURCE_ONLY=gantry ./haxeon/scripts/haxeon run --project app/tests/project-source/haxeon.json
```

## Where next

- **Change it.** Edit the belt in `GantryPicker.hx` (for example, more pulley
  teeth), reopen the example, and press Play. The ratio, the derived limits and
  every plan are recomputed from the new part. Then rerun the test: the plan
  check tells you if the motors can no longer keep up.
- **Understand it.** [The life of a machine](../concepts/start/life-of-a-machine.md)
  follows this same gantry through every layer of Materia.
