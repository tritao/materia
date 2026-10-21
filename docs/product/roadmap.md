---
title: Roadmap
description: What is being built now, what is planned next, and what is planned later, with the concept page behind each item.
status: draft
---

# Roadmap

Materia is a 0.x project. This page lists the work that is under way and the
work that is planned, grouped by how close it is. Nothing here has a date. For
what is built today, read [Features](features.md).

## Now

Work that is partly done and still moving.

- **Building the full-sheet router for real.** The frame, drives, tool
  changer, vacuum table, DXF import, sheet operations and nesting are designed
  and checked; tooling and feeds, simulated acceptance, board firmware and the
  physical build follow.
  See [Machine-axis views](../concepts/motion/machine-axes.md) and
  [The controller boundary](../concepts/execution/controller-boundary.md).
  <!-- machinekit/examples/sheet-router/PLAN.md (SR1–SR12 done; SR13–SR24 planned) -->
- **Simulation fidelity levels.** Selectable Kinematic, Ideal and Physical
  simulation, and acceptance checks that state the level they need.
  See [Fidelity levels](../concepts/execution/fidelity.md).
  <!-- simkit/plans/FIDELITY.md (SF0–SF6; work on branch simulation-fidelity) -->
- **Jerk-limited path timing.** Timed paths that also respect jerk limits,
  with smooth corners and input shaping, without changing the path's geometry.
  The routers are the first users.
  See [Paths vs. trajectories](../concepts/motion/paths-and-trajectories.md).
  <!-- motionkit/plans/ONLINE_MOTION.md (OM-R1, J0–J7; work on branch jerk-timing) -->
- **Automatic tool change.** One arm exchanges end effectors at a dock in the
  middle of a mission, knowing which tool is mounted only from lock and
  identity feedback.
  See [Process vs. robot](../concepts/process/process-and-robots.md).
  <!-- robotkit/plans/TOOL_CHANGE.md (TC0–TC6; work on branch tool-change) -->
- **Process path planning.** One planner that picks arm postures and external
  axis positions along a process path, then times it against the drives.
  See [Machine-axis views](../concepts/motion/machine-axes.md).
  <!-- motionkit/plans/PROCESS_PATH_PLANNING.md (PP0–PP11; several in progress) -->
- **Casting molds and panel tooling.** Molds and foam cores for mineral-cast
  machine bases, with a fabrication package that lists what is still
  unresolved. Physical qualification is still open.
  See [Manufacturing outputs](../concepts/process/manufacturing-outputs.md).
  <!-- manufacturingkit/TOOLING_PLAN.md (phases 1–5 done, 6 mostly, 7 started) -->
- **Flexspline hobbing machine.** A precision gear hobber for strain-wave
  flexsplines that cuts and grades the teeth in simulation; precision
  acceptance is still open.
  See [Specialized models](../concepts/modelling/specialized-models.md).
  <!-- machinekit/examples/flexspline-hobber/PLAN.md (HB1–HB11 mostly done; HB12 planned) -->
- **Machine enclosures.** One enclosure generator for every machine: it clears
  the axes, contains the process and interlocks the doors. The hobber is first.
  See [Machine components](../concepts/modelling/machine-components.md).
  <!-- machinekit/ENCLOSURE_PLAN.md (EN4, EN5, EN7 in progress) -->
- **Hot-wire foam cutter.** A four-axis foam cutter with its own CAM, G-code
  and heat model. The assembly checks it needed are built.
  See [Toolpaths](../concepts/process/toolpaths.md).
  <!-- machinekit/examples/foam-cutter/PLAN.md (FC1a–FC1e on main; later steps on branch foam-cutter) -->
- **Machine tending: camera and hardening.** The tending cell locates parts
  with a simulated camera, and its acceptance checks get stricter.
  See [Truth vs. observation](../concepts/foundations/truth-and-observation.md).
  <!-- machinekit/examples/machine-tending/PLAN.md (MT0–MT12b done; MT13, MT14) -->
- **A robot arm of our own.** An arm designed against a frozen supplier
  reference, with commercial arms used to check the software.
  See [The kinematic forest](../concepts/motion/kinematic-forest.md).
  <!-- machinekit/examples/reference-arms/PLAN.md, ARM-DESIGN.md, zeroerr-arm/ARM-BENCH-PLAN.md -->

## Next

Planned phases that are queued behind the work above.

- **A robotic press brake cell.** A robot loads and bends sheet parts in a
  simulated press brake, one bend first, then several, then calibration and
  fault recovery.
  See [Specialized models](../concepts/modelling/specialized-models.md).
  <!-- cadkit/plans/SHEET_METAL.md (SM13–SM18) -->
- **A pellet printer.** A large-format Cartesian pellet printer that prints
  and then trims. Planned after the fidelity levels.
  See [Toolpaths](../concepts/process/toolpaths.md).
  <!-- machinekit/examples/printer/PLAN.md (PL0 done; PL1–PL12) -->
- **A slant-bed turning lathe.** Turning tools, section stock, a turret lathe
  with spindle-geared feeds, and a tending cell.
  See [Stock and material removal](../concepts/process/stock.md).
  <!-- machinekit/examples/lathe/PLAN.md (TL0–TL15) -->
- **A bill of materials you can buy from.** Every line a real part with a
  stable part number, made or bought, exported as versioned JSON for a
  sourcing tool.
  See [Machine components](../concepts/modelling/machine-components.md).
  <!-- machinekit/BOM_PLAN.md (B0 survey done; B1–B8) -->
- **Live seam correction for welding.** A simulated laser profiler measures
  the seam and corrects the weld path while it runs.
  See [Truth vs. observation](../concepts/foundations/truth-and-observation.md).
  <!-- machinekit/examples/robot-welder/PLAN.md (P5) -->
- **Humanoid: sensing and recovery.** Sensor noise in the walking policy,
  standing up, fall detection and policy state in recordings.
  See [Simulation worlds](../concepts/execution/simulation-worlds.md).
  <!-- robotkit/plans/HUMANOID.md (H0–H2, H4 done; H3, H5–H8), robotkit/TODO.md -->
- **Safer robot server sessions.** A client that fails authentication can no
  longer hold the controller slot.
  See [The robot runtime](../concepts/execution/robot-runtime.md).
  <!-- robotkit/RESTRUCTURE_PLAN.md (R7a–R7f) -->

## Later

Plans that are accepted but not started, and the commercial products.

- **A circular-spline power-skiving machine.** Cuts the internal teeth of
  strain-wave circular splines.
  See [Specialized models](../concepts/modelling/specialized-models.md).
  <!-- machinekit/examples/circular-spline-skiver/PLAN.md (PS0–PS11) -->
- **Strain-wave tooth design.** A matched flexspline, circular-spline and cam
  design in GearKit, handed to the machines that cut them.
  See [Specialized models](../concepts/modelling/specialized-models.md).
  <!-- gearkit/TOOTH_DESIGN_PLAN.md (TD0–TD10) -->
- **An electric parallel gripper.** A gripper whose grip force, holding without
  power and jaw loads come from its motor and drive.
  See [Transmissions](../concepts/modelling/transmissions.md).
  <!-- machinekit/GRIPPER_PLAN.md (EG0–EG8) -->
- **A precision rotary axis.** A direct-drive rotary table, designed with an
  error budget, then built and measured on a test bench.
  See [Machine components](../concepts/modelling/machine-components.md).
  <!-- machinekit/PRECISION_ROTARY_PLAN.md (RA0–RA10) -->
- **Redundant arms and live servoing.** More of the QP-based motion for
  seven-axis arms and live tool-velocity control.
  See [The kinematic forest](../concepts/motion/kinematic-forest.md).
  <!-- motionkit/plans/LANE_D_REDUNDANCY_SERVO.md (D5a done; rest not scheduled) -->
- **Mobile base on physics and facility routes.** Wheels driven by the physics
  engine, and the base following facility routes.
  See [Facilities and fleets](../concepts/execution/facilities-and-fleets.md).
  <!-- machinekit/examples/mobile-base/PLAN.md (M6, M7, optional) -->
- **More printer features.** Bed mesh and probing, pause and filament runout,
  STL and 3MF slicing, supports and multimaterial.
  See [Toolpaths](../concepts/process/toolpaths.md).
  <!-- machinekit/examples/printer/PLAN.md ("Later features") -->
- **A persistent compiler and language service for Haxeon.** Incremental
  compilation and editor language support.
  See [Haxeon](../concepts/platform/haxeon.md).
  <!-- haxeon/ROADMAP.md (sections B, D, 3) -->
- **The Materia controller (commercial).** Controller hardware and production
  firmware with certified safety functions, built on the open device core and
  kept outside this repository. See [Licensing](licensing.md) and
  [The controller boundary](../concepts/execution/controller-boundary.md).
  <!-- docs/OPEN_CORE.md section 3 -->
- **Finished machine designs (commercial).** Production-ready machines sold
  with their technical file, risk assessment and instructions for use.
  See [Licensing](licensing.md).
  <!-- docs/OPEN_CORE.md sections 3 and 7 -->
- **Hosted services (commercial, possibly).** Sharing and collaboration, and
  support. See [Licensing](licensing.md).
  <!-- docs/OPEN_CORE.md section 3 -->

## About this page

This roadmap is a draft written from the project's plans as of 2026-10-21. The
maintainer decides the priorities, and items can move between groups or be
dropped.
