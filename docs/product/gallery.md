---
title: Gallery
description: Every example machine in the repository, what it shows, and how to open it.
status: draft
---

# Gallery

Each entry below is a machine you can open in the editor. Most appear on the
editor's Start page; all of them can also be opened from the command line.
Build the editor once from the repository root:

```sh
./haxeon/scripts/haxeon build --project=app/haxeon.json
```

then open a project with `./app/run-built.sh --project=<manifest>`. The first
open of a project compiles it, so it can take a while; later opens use the
cache. Where a project has several variants, its Inspector offers a
configuration or job selector, or each variant has its own manifest.

Dimensions, ratings and loads in these examples are design inputs. Where they
are assumptions rather than supplier data, the example's README says so.

## Gantries, CNC machines and printers

### Gantry picker

A 1500 × 1000 × 500 mm belt-driven Cartesian gantry with a sensed suction
tool. Press Play: the machine homes its axes (Z first, then the two Y motors
squared) and moves six cartons from an infeed onto a two-row pallet pattern.
A second manifest adds a yaw axis to the tool. This is the machine followed in
the [tutorial](../tutorials/gantry-picker.md).

Concepts: [Transmissions](../concepts/modelling/transmissions.md),
[Machine-axis views](../concepts/motion/machine-axes.md),
[Execution plans](../concepts/motion/execution-plans.md),
[Simulation worlds](../concepts/execution/simulation-worlds.md).

```sh
./app/run-built.sh --project=./machinekit/examples/gantry-picker/materia.project.json
./app/run-built.sh --project=./machinekit/examples/gantry-picker/materia.yaw.project.json
```

### Desktop CNC router

A three-axis gantry router of 44 MachineKit parts (T-slot frame, MGN12 rails,
lead screws) with a block of aluminium stock on its bed. Press Play: it pockets
a NEMA 23 motor plate with an end mill, changes to a drill, drills the screw
holes, and the stock takes the plate's shape as the tools move. A second
manifest has belts on X and Y instead of screws.

Concepts: [Toolpaths](../concepts/process/toolpaths.md),
[Stock and material removal](../concepts/process/stock.md),
[Transmissions](../concepts/modelling/transmissions.md).

```sh
./app/run-built.sh --project=./machinekit/examples/cnc-router/materia.project.json
./app/run-built.sh --project=./machinekit/examples/cnc-router/belts/materia.project.json
```

### Full-sheet CNC router

A router for 1600 × 2600 mm sheets on a welded steel frame: servo gantry on
racks, ball-screw Z, a zoned vacuum bed, an ISO30 tool-changer spindle and an
eight-pocket tool rack. In the editor the machine loads, holds its pose in
simulation and homes; a second entrypoint is the long-bed variant. It is being
designed to be built, so it also carries a frame fabrication packet and DXF
import for sheet jobs.

Concepts: [Machine components](../concepts/modelling/machine-components.md),
[Machine-axis views](../concepts/motion/machine-axes.md),
[Manufacturing outputs](../concepts/process/manufacturing-outputs.md).

```sh
./app/run-built.sh --project=./machinekit/examples/sheet-router/materia.project.json
```

### Mill

A mineral-cast vertical mill in two sizes, Small and Compact VMC, chosen with
the **Configuration** selector, plus an editable spindle-motor study. Both
sizes carry a pneumatic vise and a named CNC mission: Play machines a bearing
block, with feed hold, resume and program inspection in the CNC panel.

Concepts: [Toolpaths](../concepts/process/toolpaths.md),
[Stock and material removal](../concepts/process/stock.md),
[Assemblies and mates](../concepts/modelling/assemblies-and-mates.md).

```sh
./app/run-built.sh --project=./machinekit/examples/bench-mill/materia.project.json
```

Shortcut manifests open single configurations directly:
`materia.y-servo.project.json` (Small), `materia.capacity.project.json`
(Compact VMC) and `materia.motor-study.project.json` (spindle-drive study), all
in the same folder.

### Bench mill

The earlier servo bench mill, open and in an enclosed cell with a sliding door
and a preset pneumatic vise, machining the same bearing block.

Concepts: [Toolpaths](../concepts/process/toolpaths.md),
[Machine components](../concepts/modelling/machine-components.md).

```sh
./app/run-built.sh --project=./machinekit/examples/bench-mill/materia.bench.project.json
./app/run-built.sh --project=./machinekit/examples/bench-mill/materia.enclosed.project.json
```

### CoreXY pen plotter

A small CoreXY mechanism: two motors whose belts together move the pen in X
and Y. Play loops a short square-and-diagonal program on a virtual device. It
is the simplest example of a coupled belt drive.

Concepts: [Transmissions](../concepts/modelling/transmissions.md),
[The controller boundary](../concepts/execution/controller-boundary.md).

```sh
./app/run-built.sh --project=./machinekit/examples/corexy/materia.project.json
```

### CoreXY 3D printer

A CoreXY FDM printer on a 2020 T-slot frame: MGN12 rails, GT2 belts, a bed on
three lead screws with a motor each, a geared extruder, heaters and fans, and
switch and probe homing. The default project prints a small tab from a sliced
program; two more print a calibration cube (full, and a short solid version).
The same assembly also exports a Klipper configuration.

Concepts: [Toolpaths](../concepts/process/toolpaths.md),
[Stock and material removal](../concepts/process/stock.md),
[Transmissions](../concepts/modelling/transmissions.md).

```sh
./app/run-built.sh --project=./machinekit/examples/printer/materia.project.json
./app/run-built.sh --project=./machinekit/examples/printer/cube-short/materia.project.json
./app/run-built.sh --project=./machinekit/examples/printer/cube/materia.project.json
```

## Robot arms

### Six-axis robot arm

A serial arm of 20 MachineKit parts on a pedestal, with a shoulder, elbow and
spherical-wrist layout and a suction tool. Play runs a pick-and-place cycle
between two pads and back.

Concepts: [The kinematic forest](../concepts/motion/kinematic-forest.md),
[Paths vs. trajectories](../concepts/motion/paths-and-trajectories.md),
[The robot runtime](../concepts/execution/robot-runtime.md).

```sh
./app/run-built.sh --project=./machinekit/examples/robot-arm/materia.project.json
```

### Cobot size classes

A generic collaborative arm family in four reaches (500, 850, 900 and 1300 mm,
for 3 to 16 kg payloads), each running a six-joint motion loop. The actuator
sizes are initial assumptions, not validated hardware.

Concepts: [The kinematic forest](../concepts/motion/kinematic-forest.md),
[Mechanical truth](../concepts/modelling/mechanical-truth.md).

```sh
./app/run-built.sh --project=./machinekit/examples/cobot-arm/materia.reach500.project.json
./app/run-built.sh --project=./machinekit/examples/cobot-arm/materia.project.json
./app/run-built.sh --project=./machinekit/examples/cobot-arm/materia.reach900.project.json
./app/run-built.sh --project=./machinekit/examples/cobot-arm/materia.reach1300.project.json
```

### UR and FAIRINO reference arms

22 official UR and FAIRINO robot descriptions, converted from their published
URDF and selectable as model jobs, plus a comparison of a ZeroErr-based arm
with the UR5e and FR5 on floor, wall and ceiling mounts. The descriptions are
downloaded by a preparation script first (see the
[example README](../../machinekit/examples/reference-arms/README.md)).

Concepts: [The kinematic forest](../concepts/motion/kinematic-forest.md),
[Conventions](../concepts/foundations/conventions.md).

```sh
./app/run-built.sh --project=./machinekit/examples/reference-arms/materia.project.json
./app/run-built.sh --project=./machinekit/examples/reference-arms/comparison.project.json
```

### Robot arm on a linear track

The six-axis arm on a 3 m linear track, working at each end of a long table;
the track is an external axis.

Concepts: [Machine-axis views](../concepts/motion/machine-axes.md).

```sh
./app/run-built.sh --project=./machinekit/examples/track-arm/materia.project.json
```

### Overhead gantry arm

An inverted industrial arm hanging from the Z carriage of a 3 m × 1.5 m XYZ
gantry (racks on X and Y, a lead screw on Z). A six-step mission picks a
workpiece near one corner and places it at the opposite one, using all three
external axes.

Concepts: [Machine-axis views](../concepts/motion/machine-axes.md),
[Collision model](../concepts/modelling/collision.md).

```sh
./app/run-built.sh --project=./machinekit/examples/overhead-arm/materia.project.json
```

## Mobile robots

### Mobile base

A differential-drive base: two NEMA 23 steppers driving the wheels directly,
casters, a battery, an upper deck and a planar lidar. The `cell` entrypoint
puts it in a room, where it drives a round of the shelves and the dock on its
own, seeing obstacles with the lidar; `base` shows the base alone.

Concepts: [The robot runtime](../concepts/execution/robot-runtime.md),
[Truth vs. observation](../concepts/foundations/truth-and-observation.md),
[Facilities and fleets](../concepts/execution/facilities-and-fleets.md).

```sh
./app/run-built.sh --project=./machinekit/examples/mobile-base/materia.project.json
```

### Mobile robot welder

The welding arm and torch on a mobile base, and a welding mission planned
from mobile work stations.

Concepts: [Process vs. robot](../concepts/process/process-and-robots.md),
[Machine-axis views](../concepts/motion/machine-axes.md).

```sh
./app/run-built.sh --project=./machinekit/examples/robot-welder/materia.mobile.project.json
./app/run-built.sh --project=./machinekit/examples/robot-welder/materia.mobilemission.project.json
```

## Process cells

### Virtual picking station

An extrusion rack with two shelves of addressed bins, pick indicators and an
operator bench. A project-supplied panel runs an order: start it, click the
highlighted bin, confirm the pick, and the stock and progress update. The
**Cut planning** panel shows the sheet requirements and the plywood cut plan.

Concepts: [The editor and projects](../concepts/platform/editor-and-projects.md),
[Manufacturing outputs](../concepts/process/manufacturing-outputs.md).

```sh
./app/run-built.sh --project=./machinekit/examples/picking-station/materia.project.json
```

### Machine tending cell

A cobot loads and unloads the mill through four unattended cuts. A second
project wires the mill and the cobot to separate controllers with cell I/O.
Two more open the tending end effector and a dual electric gripper reference
on their own.

Concepts: [Process vs. robot](../concepts/process/process-and-robots.md),
[The robot runtime](../concepts/execution/robot-runtime.md),
[Collision model](../concepts/modelling/collision.md).

```sh
./app/run-built.sh --project=./machinekit/examples/machine-tending/materia.project.json
./app/run-built.sh --project=./machinekit/examples/machine-tending/materia.controllers.project.json
./app/run-built.sh --project=./machinekit/examples/machine-tending/materia.end-effector.project.json
./app/run-built.sh --project=./machinekit/examples/machine-tending/materia.electric-tool.project.json
```

### Robot welder

A fixed MIG cell: the six-axis arm with a robot torch, a wire feeder, a power
source, a gas cylinder and a welding table with a plate T-joint and a small
tube frame. The services are connected through ports the way a real cell is
cabled. Seams are found from the CAD faces, not authored. The job selector
offers the whole weldment, one seam, a tube post welded as one continuous
path, a 7 mm woven seam and a 10 mm three-pass seam.

Concepts: [Process vs. robot](../concepts/process/process-and-robots.md),
[Machine components](../concepts/modelling/machine-components.md),
[Paths vs. trajectories](../concepts/motion/paths-and-trajectories.md).

```sh
./app/run-built.sh --project=./machinekit/examples/robot-welder/materia.project.json
```

Each job also has its own manifest in the same folder:
`materia.seam.project.json`, `materia.post.project.json`,
`materia.weave.project.json` and `materia.multipass.project.json`.

### Gantry welder

A screw-driven XYZ gantry with a two-axis torch head, welding the same plate
and tube-frame weldment as the robot welder.

Concepts: [Machine-axis views](../concepts/motion/machine-axes.md),
[Process vs. robot](../concepts/process/process-and-robots.md).

```sh
./app/run-built.sh --project=./machinekit/examples/gantry-welder/materia.project.json
```

### Coordinated track welding

The arm on its linear track welds a 2.6 m CAD-derived fillet, more than twice
the arm's reach, moving the track and the arm together.

Concepts: [Machine-axis views](../concepts/motion/machine-axes.md),
[Paths vs. trajectories](../concepts/motion/paths-and-trajectories.md).

```sh
./app/run-built.sh --project=./machinekit/examples/track-arm/materia.welder.project.json
```

### Robot screwdriver

A screwdriving cell: a spindle on a sensed compliant feed, a hex bit with a
vacuum sleeve, a tightening controller, screw presenters and four fastenings
found from the CAD. The projects in this folder build up from the equipment
alone to complete cycles:

| Manifest | What it shows |
|---|---|
| `materia.project.json` | The equipment and idle tool feedback |
| `materia.drive.project.json` | One M6 fastening |
| `materia.assembly.project.json` | All four flange fastenings, with a reject bin and replacement supplies |
| `materia.registered.project.json` | Six touch measurements locate the flange before fastening |
| `materia.force-search.project.json` | A force and spiral search for screw entry |
| `materia.registered-force-search.project.json` | Touch registration followed by force search |
| `materia.staged.project.json` | Snug, then final tightening |
| `materia.feeder.project.json` | A bowl feeder replenishing one pickup nest |
| `materia.blow-feed.project.json` | Screws blown through a tube to the tool |
| `materia.pick-place.project.json` | Two arms pick, place and fasten a flange housing |
| `materia.housing-handling.project.json` | Commissioning of the housing handling |
| `materia.threads.project.json`, `materia.fastenings.project.json` | Cosmetic threads and fastening discovery in CAD |

Concepts: [Process vs. robot](../concepts/process/process-and-robots.md),
[Truth vs. observation](../concepts/foundations/truth-and-observation.md),
[CAD documents](../concepts/modelling/cad-documents.md).

```sh
./app/run-built.sh --project=./machinekit/examples/robot-screwdriver/materia.assembly.project.json
```

## Manufacturing equipment

### Flexspline hobber

A vertical moving-column hobbing machine for strain-wave flexsplines: a
direct-drive work table, radial column infeed, vertical head feed, swivel and
hob shift, in a guarded enclosure. Play scenarios show the doors, a coarse
cut, or loading, cutting and unloading a cup. The machine panel shows the
blank, clamped and finished part. It uses supplier rail CAD that a
preparation script downloads first (see the
[example README](../../machinekit/examples/flexspline-hobber/README.md)).

Concepts: [Stock and material removal](../concepts/process/stock.md),
[Specialized models](../concepts/modelling/specialized-models.md),
[Toolpaths](../concepts/process/toolpaths.md).

```sh
./app/run-built.sh --project=./machinekit/examples/flexspline-hobber/materia.project.json
```

### Sheet metal press brake

A 40 t, 1600 mm hydraulic press brake with ram, tooling, depth stop and
backgauge as ordinary assembly joints, and hydraulic services in the BOM. The
default project runs an empty stroke; the editor does not run a bending cycle.
A second project is a feedback-controlled press. Bend-sequence planning is
exercised by the example's checks, not yet in the editor.

Concepts: [Specialized models](../concepts/modelling/specialized-models.md),
[Mechanical truth](../concepts/modelling/mechanical-truth.md).

```sh
./app/run-built.sh --project=./machinekit/examples/press-brake/materia.project.json
./app/run-built.sh --project=./machinekit/examples/press-brake/materia.controller.project.json
```

## Components and references

### Motor, shaft and bearings

A NEMA 17 motor on a plate with four M3 screws, and an output shaft on two 608
bearings: the smallest MachineKit assembly.

Concepts: [Machine components](../concepts/modelling/machine-components.md),
[Assemblies and mates](../concepts/modelling/assemblies-and-mates.md).

```sh
./app/run-built.sh --project=./machinekit/examples/materia.project.json
```

### CAD modelling

Separately generated B-rep components of a procedural excavator, built with
CadKit's sketch, extrude and feature API. The folder also holds smaller
modelling examples (plates, flanges, enclosures) used by CadKit's tests.

Concepts: [CAD documents](../concepts/modelling/cad-documents.md).

```sh
./app/run-built.sh --project=./cadkit/examples/modeling/materia.project.json
```

### Leadshine servo motors

The Leadshine ELM1 and ELM2 AC servo catalog, 50 W to 7.5 kW, with and without
brakes, modelled from the supplier datasheet.

Concepts: [Machine components](../concepts/modelling/machine-components.md),
[Transmissions](../concepts/modelling/transmissions.md).

```sh
./app/run-built.sh --project=./machinekit/examples/leadshine-servos/materia.project.json
```

### DH Robotics PGC-140-50 gripper

Supplier STEP geometry of an electric parallel gripper, split into housing,
moving jaws and a routed cable, plus custom finger and dual-tool variants.
The supplier CAD is downloaded by a preparation script first and is not
redistributed (see the [example README](../../machinekit/examples/dh-gripper/README.md)).

Concepts: [Machine components](../concepts/modelling/machine-components.md).

```sh
./app/run-built.sh --project=./machinekit/examples/dh-gripper/materia.project.json
```

### Enclosure kit

A 1000 × 600 × 800 mm machine enclosure of 40-series extrusions, composite
infill panels and a sliding polycarbonate door, with panel layers in the BOM
and rule checks that report unqualified protective use.

Concepts: [Machine components](../concepts/modelling/machine-components.md),
[Diagnostics](../concepts/foundations/diagnostics.md).

```sh
./app/run-built.sh --project=./machinekit/examples/enclosure-kit/materia.project.json
```

### HZMotion mounts and a Y-drive study

A catalog of supplier motor mounts drawn from their drawings, and a worked Y
axis stack with a Leadshine servo.

Concepts: [Machine components](../concepts/modelling/machine-components.md).

```sh
./app/run-built.sh --project=./machinekit/examples/supplier-fasteners/materia.hzmotion.project.json
```

## Other

These open from the editor's Start page only; they have no project manifest.

- **Two robots with sensors.** A setup written as a script: press Play to
  simulate lidar and IMUs on two robots.
  See [Truth vs. observation](../concepts/foundations/truth-and-observation.md).
- **Worker moves a rack.** A walking human worker in a running physics
  simulation.
  See [Specialized models](../concepts/modelling/specialized-models.md).
- **Worker gallery.** Six workers side by side: bending, crouching, kneeling,
  turning and two-handed holds.

## Studies without an editor project

Some folders in `machinekit/examples/` are design studies and checks rather
than machines to open. Most run headless with
`./haxeon/scripts/haxeon run --project=machinekit/examples/NAME/haxeon.json`
and write their outputs (STEP, drawings, reports, `.mtrg` scenes) to the
folder's `build/` directory. Their READMEs have the details:

- `box-mold` and `panel-tooling`: casting mold and panel joinery tooling
  ([Manufacturing outputs](../concepts/process/manufacturing-outputs.md)).
- `eoat`: suction end effectors with a tool changer.
- `tool-service`: an electrical tool interface for the cobot wrist.
- `linear-rail-reference`: generated rails compared with supplier CAD.
- `zeroerr-arm`: an arm built from a supplier reference design.
- `spindle-drive-study`: spindle motor requirements for the mill and the
  hobber, analysed with Python scripts.

A turning lathe, a hot-wire foam cutter and a circular-spline skiving machine
are planned; their folders hold only design plans so far.

<!--
Sources: app/src/editor/ExampleCatalog.hx (Start page entries), the
materia.project.json manifests and READMEs under machinekit/examples/ and
cadkit/examples/modeling/, docs/tutorials/gantry-picker.md.
Every manifest path above was checked to exist on 2026-10-21.
-->
