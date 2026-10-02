# Mobile welder plan

Goal: a battery-powered mobile robot that drives up to a steel weldment, finds
it, and MIG-welds its seams, with the seams, the torch and the welding
equipment all coming from the CAD. It is planned so the simulated welder and a
real one share one interface.

## Hardware this models

These choices were made on 2026-10-02 and set what the model has to represent:

- **Power:** a 48 V LFP pack (16s, about 25 kWh for a shift) feeds a 48 V to
  230 V pure-sine inverter (8–10 kW, low-frequency type). That inverter feeds a
  single-phase MIG/flux-core power source. The arm, base and computer run from
  the pack through isolated DC-DC converters. The pack floats (it is not
  connected to the chassis), and the work clamp goes to the workpiece only.
- **Welding power source:** a robot-interface unit (Modbus TCP, EtherCAT or
  I/O), or for the prototype a manual welder retrofitted with a remote socket.
  The robot sees the welder through these signals:
  - outputs: arc on, wire-speed setpoint, voltage setpoint, job number;
  - inputs: arc established, actual current and voltage, touch, fault.
  A retrofit welder reports some of these as unsupported.
- **Process:** short-circuit MIG or self-shielded flux-core, 150–250 A, with
  the arc on about 40% of the time.

## What exists

The mobile manipulator is done (mobile-base M0–M5 on main). The base drives
missions from the scene artifact's `mission` section. The arm stands on the
deck. Tools come from CAD capabilities: MachineKit `EndEffectorControls`
writes the scene's `robotTools` section, and a RobotKit simulated device
(`SimulatedSuctionTool`) reads the robot's channels. Skills run MotionKit
programs (`HandlePart`, `HandlingPlanRunner`).

Other pieces to build on:
- MotionKit Lane B has `FollowPath` with timed channel events, holds on the
  path, and approach and retract moves.
- ProcessKit has `ProcessRun` (prepare, ready, active, interrupted, with
  recovery that re-approaches the path distance where processing stopped), a
  `ProcessDevice` boundary, and `ProcessRecipe` (speed range, standoff, rate
  per distance).
- RobotKit has `power.BatteryState` and a `Charge` skill.
- The CNC router's runtime geometry (`EditorScene.setRuntimeGeometryParts`,
  `StockPreviewWorker`) shows material changing during a run.
- Topological naming gives faces and edges names that survive model edits.

What is missing is everything specific to welding: no part models a torch,
wire feeder or welding power source; nothing finds seams in an assembly;
there is no arc device, no bead, and no weaving. Lane B lists seam tracking,
weaving and force control as out of scope. Nothing uses `BatteryState` in
simulation.

## Steps

**W0. Welding equipment parts.** MachineKit gains:
- a `WeldingTorch`: a swan neck (22°/45°), nozzle and contact tip. Its `tcp`
  connector sits at the wire tip at the nominal stickout, and the neck is the
  collision geometry that matters;
- a `WireFeeder`, a `WeldingPowerSource`, an `Inverter` and a `BatteryPack`,
  each with its mass, ports (weld cable, gas, control, 48 V and 230 V) and
  capabilities;
- new capabilities: `ArcTorch(tcpConnector, controlPort)`,
  `WeldingSupply(processes, maxCurrent, interface)` and
  `BatteryStorage(nominalVolts, capacityWh)`. `EndEffectorControls` derives
  the welding channels from them, as it derives vacuum channels.

The robot is `MobileBase(RobotArm(torch))`, with the feeder on the upper arm
or the deck and the power source, inverter and pack on the base plate. If the
base is too small for them, it gets a variant with a larger plate. Checks: the
TCP pose at the ready pose, the torch neck clears the arm, mass budget,
cables routed from port to port.

**W1. Seams from the CAD.** The workpiece is a weldment: a `FrameAssembly`
tube frame, plus a plate T-joint so there is a fillet joint in plate. A seam
is not authored as a polyline. It is found where two members meet: the
intersection edges of their touching faces, named after the two faces so the
name survives edits. A `WeldSeam` has that edge reference, a joint type
(fillet, butt, lap), a leg size, a number of passes, and start/stop
direction. The seam frame is derived from the geometry: tangent along the
edge, torch axis on the bisector of the two face normals, then the travel
angle (push/pull) and the work angle. Checks: a T-joint's seam frame bisects
it at 45°; editing a tube size keeps the seam, which follows the new edge; a
gap between members reports no seam.

**W2. The welder as a simulated robot tool.**
- The scene artifact's `robotTools` gains a `torch` kind with its channels:
  `weld.arc` (digital), `weld.wire_speed` and `weld.voltage` (analog), and the
  sensors `weld.arc_established`, `weld.current`, `weld.voltage_actual`,
  `weld.touch` and `weld.fault`.
- RobotKit gains a `SimulatedWelder`, a `SimulationStepObserver` like the
  suction tool:
  - the arc lights when it is commanded on and the wire tip is within reach
    of grounded work;
  - current and voltage come from a simple synergic model of wire speed and
    stickout;
  - touch closes when the wire meets work while the arc is off;
  - faults: no arc within a timeout, wire stuck to the work, out of wire, out
    of gas.
- ProcessKit gains a `WelderProcessDevice`, the `ProcessDevice` over those
  channels: ready means the supply is ready, and safe means the arc is off.

**W3. Weld one seam.**
- The `mission` section gains a `weld` step that names a seam.
- A RobotKit `WeldSeam` skill runs a MotionKit program:
  1. a `MoveJ` to the approach pose and a `MoveL` to the start;
  2. arc on, holding on the path until arc established, with a short start
     dwell;
  3. `FollowPath` along the seam frame at travel speed;
  4. a crater-fill dwell, arc off, burnback, then retract.
- `ProcessRun` drives it, so an arc loss mid-seam re-approaches and restarts
  with overlap.
- The bead grows as runtime geometry, appended per tick along the path the
  TCP actually travelled. Its cross-section comes from the deposition rate
  (wire speed × wire area ÷ travel speed), so the leg size it reaches is a
  result, and it is checked against the seam's leg size.
- The parking pose is still chosen by hand.
- Tests: one fillet weld on MuJoCo; the bead's leg size is within 0.5 mm of
  the target; an arc loss is injected and the run recovers.

**W4. Weld a whole weldment.** Choose the parking poses from reachability
(RobotKit construction M8): the fewest stations that cover every seam with
the torch angles each seam needs. Order the seams within each station, and
the stations themselves. Approaches must clear the work: collision plan CL4
validation when it lands, until then a swept-volume clearance check. The
mission is generated from the weldment, so adding a member adds its seams.

**W5. Finding the part after parking.** The base parks with ±20 mm and ±2°
of noise (realistic AMR localization). Touch-sense search moves on two or
three faces near each station's seams give the workpiece's offset, and every
seam frame at that station is corrected by it. Test: with injected noise, the
seams still hit the leg-size check that passed without noise.

**W6. Energy.**
- Each consumer reports its power draw: the arc (current × voltage ÷ supply
  efficiency ÷ inverter efficiency), the arm (from joint torques and speeds),
  the drive, and a fixed load for the computer.
- A simulated 48 V pack integrates those draws into `BatteryState`, with a
  voltage that sags with charge and load and a cutoff below which the arc
  can't be held.
- Before each seam, the mission checks there's enough energy left for the
  whole seam plus the drive to the dock. If not, it docks and charges first,
  because a seam must not stop part-way.
- The efficiency chain is a parameter: about 0.78 for inverter plus mains
  welder, about 0.92 for a direct 48 V converter.

**W7. Weaving and multi-pass.** A MotionKit path modifier lays a weave
(sine, triangle or zigzag: amplitude, frequency, dwell at the edges) across
the seam frame, keeping exact timing and events. The motion check covers the
woven path, not just the centreline. Multi-pass seams get root, fill and cap
passes offset in the seam frame, the bead from earlier passes counting as
work.

**W8. Real hardware interface.** Map the channels to:
- the retrofit I/O board: an optoMOS relay for the trigger, an isolated 0–10
  V output for wire speed and voltage, a Hall-effect current sensor, an
  isolated voltage input, and a separate touch-sense circuit;
- a robot power source protocol: Modbus TCP registers, or I/O.

Then carry them over RKD6 like other process channels, with the arc channel
going safe (off) on every stop.

**W9. Seam tracking (stretch).** SensorKit gains a laser line profiler built
on depth rendering. The seam is found before welding and corrected during it
(Lane D servoing).

## Order

W0 → W1 → W2 → W3 is the core: one real seam from CAD, welded in simulation.
W1 carries the modelling risk, because seams have to come from geometry, not
from authored lines. W4–W6 make it mobile in earnest. W7 is process quality.
W8 is the step to hardware. W9 is a stretch goal.

Dependencies:
- W4's collision checks lean on the collision plan's CL3/CL4, with a
  fallback until those land.
- W1 leans on topological naming's edge persistence, which TN9 covered for
  edge picks.

## Progress

| Step | State | Commits |
| --- | --- | --- |
| W0 | | |
| W1 | | |
| W2 | | |
| W3 | | |
| W4 | | |
| W5 | | |
| W6 | | |
| W7 | | |
| W8 | | |
