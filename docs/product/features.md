---
title: Features
description: What Materia can do today, grouped by stage, with what is planned listed separately.
status: draft
---

# Features

This page lists what is built and tested in the repository today. Each line
links to the concept page that explains it. Many machine values in the
examples (frame sizes, servo ratings, payloads) are stated assumptions, and the
pages and READMEs mark them as such. What is not built yet is in
[Planned](#planned) at the end.

## Modelling: describing a machine

- **Parametric CAD on Open CASCADE.** CadKit has documents of features
  (extrude, pocket, hole, fillet, boolean, imported shape) that recompute
  with undo, constrained 2D sketches with a solver, and constraint diagnosis.
  See [CAD documents](../concepts/modelling/cad-documents.md).
- **STEP in and out, drawings out.** CadKit imports and exports STEP geometry,
  reads planar DXF, and writes SVG and DXF drawing views.
  See [CAD documents](../concepts/modelling/cad-documents.md).
- **Persistent face and edge names.** Features keep referring to the same face
  or edge when the model above them changes.
  See [Identity](../concepts/foundations/identity.md).
- **Assemblies with joints and mates.** Parts are placed by mates and moved by
  joints; closed loops are solved, and the editor drags parts under their
  mates. See [Assemblies and mates](../concepts/modelling/assemblies-and-mates.md).
- **Machine components with a bill of materials.** MachineKit parts know their
  geometry, connectors, material and BOM line; catalogs record where each
  value came from, including supplier STEP parts.
  See [Machine components](../concepts/modelling/machine-components.md).
- **Transmissions from real parts.** Lead and ball screws, belts (including
  CoreXY), gears, racks and chains are coupled joints whose ratios come from
  the parts; motor and drive models give the axis limits.
  See [Transmissions](../concepts/modelling/transmissions.md).
- **Integrity checks.** A machine is checked for parts that float, overlap at
  rest or during sampled motion, and drives whose geometry disagrees with
  their ratio. See [Mechanical truth](../concepts/modelling/mechanical-truth.md).
- **Collision.** A collision world over coal with pair rules, margins, batched
  distance queries and convex decomposition (V-HACD).
  See [Collision model](../concepts/modelling/collision.md).
- **Sheet metal.** Walls, bends, flanges, hems, rips and corners, with flat
  patterns and blanks for sheet planning.
  See [Specialized models](../concepts/modelling/specialized-models.md).
- **Gears.** Tooth profiles, hob design and ISO 1328 inspection in GearKit.
  See [Specialized models](../concepts/modelling/specialized-models.md).
- **Buildings.** BimKit models buildings as typed CadKit documents and exports
  IFC. See [Specialized models](../concepts/modelling/specialized-models.md).
- **People.** Skinned human workers that walk, hold props and carry out jobs
  in a simulated cell or facility.
  See [Specialized models](../concepts/modelling/specialized-models.md).

## Motion: making it move

- **One kinematics core.** CAD assemblies and robot models compile into the
  same kinematic forest, with forward kinematics, Jacobians and several IK
  solvers. See [The kinematic forest](../concepts/motion/kinematic-forest.md).
- **Path timing and free moves.** TOPP-RA times geometric paths (lines, arcs,
  seams, toolpaths); Ruckig plans free state-to-state moves; RRT-Connect plans
  around obstacles.
  See [Paths vs. trajectories](../concepts/motion/paths-and-trajectories.md).
- **Validated execution plans.** Programs compile into immutable plans that
  are checked against the drives (stepper stall, servo peak and rated torque)
  before they run. See [Execution plans](../concepts/motion/execution-plans.md).
- **Machine-axis views.** Gantries, tracks and external axes, including
  dual-motor axes with squaring during homing.
  See [Machine-axis views](../concepts/motion/machine-axes.md).

## Process: making it do work

- **One toolpath format.** CAM output, G-code, printing and stock simulation
  share one program format. See [Toolpaths](../concepts/process/toolpaths.md).
- **CAM for the examples.** 2.5D profiles, pockets and drilling, sheet-part
  profiles, slicing for FDM printing, and gear hobbing.
  See [Toolpaths](../concepts/process/toolpaths.md).
- **G-code in and out.** CncKit reads and writes LinuxCNC G-code and printer
  G-code (Marlin and Klipper flavours); the printer example also exports a
  Klipper configuration. See [Toolpaths](../concepts/process/toolpaths.md).
- **Material removal.** StockKit cuts a tri-dexel stock as the simulated tool
  moves, and compares the result with the target part.
  See [Stock and material removal](../concepts/process/stock.md).
- **Process skills.** Welding (seams found from the CAD faces, weaving,
  multipass), screwdriving (with an Open Protocol client), surface finishing
  and scanning, on top of RobotKit's generic skills.
  See [Process vs. robot](../concepts/process/process-and-robots.md).
- **Manufacturing outputs.** Sheet cut plans with CSV and SVG export,
  inventory and cut records, casting-mold and panel tooling, and fabrication
  packets. See [Manufacturing outputs](../concepts/process/manufacturing-outputs.md).

## Simulation and execution

- **Physics simulation.** SimKit runs MuJoCo for the editor's Play, on the
  desktop and in the browser, with a deterministic backend for tests.
  See [Simulation worlds](../concepts/execution/simulation-worlds.md).
- **One robot runtime.** The same runtime owns a robot in simulation and on a
  device, with snapshots, sensor streams and MCAP recording; `robotd` serves
  it over the network. See [The robot runtime](../concepts/execution/robot-runtime.md).
- **Open controller protocol.** The RKD6 device protocol, a reference device
  core in Rust (`no_std`), a virtual device, and a bench board profile
  (Nucleo G474RE, no motors connected yet).
  See [The controller boundary](../concepts/execution/controller-boundary.md).
- **Simulated sensors and cameras.** SensorKit models IMU measurement noise
  and timing; VisionKit does camera calibration, pose estimation and marker
  detection over OpenCV.
  See [Truth vs. observation](../concepts/foundations/truth-and-observation.md).
- **Facilities and fleets.** Zones, stations, lanes, routing, dispatch and
  traffic reservations, built in code and tested headless.
  See [Facilities and fleets](../concepts/execution/facilities-and-fleets.md).

## Editor and platform

- **One editor, desktop and browser.** Hierarchy, inspector, viewport, Play,
  and panels for missions, CNC programs and cut planning. The browser build
  opens prebuilt examples and runs Play with MuJoCo.
  See [The editor and projects](../concepts/platform/editor-and-projects.md)
  and [Web vs. native](../concepts/platform/web-and-native.md).
- **Projects as code.** A project manifest names Haxe entrypoints that build
  the machine; the editor compiles and runs them, and saves your edits as an
  overlay. Projects can add their own editor panels.
  See [The editor and projects](../concepts/platform/editor-and-projects.md).
- **Haxeon.** Materia's own Haxe-compatible compiler and runtime: HashLink on
  the desktop, WebAssembly in the browser, with live code patching.
  See [Haxeon](../concepts/platform/haxeon.md).
- **Rendering and UI.** A retained 3D scene with GPU rendering and picking, and
  a retained widget toolkit. See [SceneKit rendering](../concepts/platform/rendering.md)
  and [NativeKit and UIKit](../concepts/platform/nativekit-and-ui.md).

## Planned

These are designed but not built. They are listed so you know what not to
expect yet; none has a date.

- **Simulation fidelity levels.** Selectable Kinematic, Ideal and Physical
  simulation, and checks that state the level they need.
  See [Fidelity levels](../concepts/execution/fidelity.md).
- **Automatic tool change.** One arm exchanging end effectors at a dock in the
  middle of a mission.
- **Jerk-limited path timing.** Today no planner limits jerk along a timed
  path; how to add it without changing the path's geometry is being
  researched, with the routers as the first users.
- **A pellet printer.** A large-format Cartesian pellet printer that prints
  and then trims.
- **More machines.** A slant-bed turning lathe, a hot-wire foam cutter, a
  circular-spline power-skiving machine and an electric parallel gripper.
- **Building the sheet router for real.** The full-sheet router is being
  designed to be built and commissioned on Materia's own control stack.
- **A commercial controller.** Controller hardware and production firmware
  with certified safety functions, kept outside this repository.
  See [Licensing](licensing.md).
- **A saved facility format** loaded by the editor; today facilities are built
  in code.

<!--
Planned items are distilled from: simkit/plans/FIDELITY.md (SF1–SF5),
robotkit/plans/TOOL_CHANGE.md (TC0–TC6), motionkit/plans/ONLINE_MOTION.md,
machinekit/examples/printer/PLAN.md (PL1–PL12), machinekit/examples/lathe/PLAN.md,
machinekit/examples/foam-cutter/PLAN.md, machinekit/examples/circular-spline-skiver/PLAN.md,
machinekit/GRIPPER_PLAN.md, machinekit/examples/sheet-router/PLAN.md, docs/OPEN_CORE.md.
The facility-format line comes from the "What exists today" note on the
facilities concept page; no plan schedules it.
-->
