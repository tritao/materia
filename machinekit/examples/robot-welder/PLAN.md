# Robot welder plan

Goal: a robot that MIG-welds the seams of a steel weldment, with the seams, the
torch and the welding equipment all coming from the CAD, and a simulated welder
that shares one interface with a real one.

It is built in two phases. **Phase 1 is a fixed welding cell:** the six-axis
arm on its pedestal, a torch, a mains-powered welding power source on the
floor, and a welding table holding the weldment. **Phase 2 puts the same
welding robot on the mobile base,** powered from a 48 V battery through an
inverter. Phase 1 is built so phase 2 only adds things and changes nothing.

## Keeping phase 2 open

- **The welding robot is a reusable assembly,** not part of the cell. The arm
  takes its tool as a constructor argument (today it hard-codes the suction
  tool). The cell includes `RobotArm(torch)` on the floor, and phase 2 puts the
  same `RobotArm(torch)` on `MobileBase`'s deck, as mobile-base M5 does with the
  suction arm.
- **The equipment's power comes in through a port.** The welding power source
  has a `mains` input port (230 V single-phase). In the cell that port goes to
  the wall, an exposed port of the cell. In phase 2 it goes to the inverter's
  output, which is fed from the pack. Nothing else about the power source
  changes.
- **The work is a mission, not a script.** Welds run as `weld` steps of the
  scene's `mission` section, through a RobotKit skill that only uses the robot.
  A fixed robot runs `weld` steps alone. Phase 2 interleaves them with the
  `goTo` steps the mobile base already runs.
- **The welder device reports its power draw from the start,** so the energy
  model in phase 2 only adds the battery and the other consumers.
- **Seam frames are found against the workpiece as it actually lies.** In the
  cell the workpiece sits on its fixture, so the correction is zero. Phase 2's
  touch sensing fills that correction in.

## What exists

- The six-axis arm example (`robot-arm`): pedestal, joints j1..j6, and an ISO
  tool flange with a suction tool.
- The mobile manipulator (mobile-base M0–M5): missions in the scene artifact,
  and tools that come from CAD capabilities. MachineKit `EndEffectorControls`
  writes the scene's `robotTools` section, and a RobotKit simulated device
  (`SimulatedSuctionTool`) reads the robot's channels. Skills run MotionKit
  programs (`HandlePart`, `HandlingPlanRunner`).
- MotionKit Lane B: `FollowPath` with timed channel events, holds on the path,
  and approach and retract moves.
- ProcessKit: `ProcessRun` (prepare, ready, active, interrupted, with recovery
  that re-approaches the path distance where processing stopped), the
  `ProcessDevice` boundary, and `ProcessRecipe`.
- The CNC router's runtime geometry (`EditorScene.setRuntimeGeometryParts`)
  shows material changing during a run.
- Topological naming: faces and edges keep their names across model edits.

Missing: no part models a torch, wire feeder or welding power source; nothing
finds seams in an assembly; there is no arc device, no bead, and no weaving
(Lane B left weaving and seam tracking out of scope).

## Phase 1: fixed welding cell

**W0. Welding equipment and the cell.**
- `RobotArm` takes its end effector from the caller. The arm and mobile-base
  examples pass the suction tool and behave as before.
- MachineKit gains:
  - a `WeldingTorch`: a swan neck (22°/45°), nozzle and contact tip, mounted
    on the tool flange through a breakaway mount. Its `tcp` connector sits at
    the wire tip at the nominal stickout, and the neck is its collision shape;
  - a `WireFeeder`, a `WeldingPowerSource` (`mains` input port, weld cable,
    gas and control ports), and a `GasCylinder`;
  - capabilities `ArcTorch(tcpConnector, controlPort)` and
    `WeldingSupply(processes, maxCurrent, interface)`.
- `EndEffectorControls` derives the welding channels from them, as it derives
  vacuum channels.
- `WeldingCell`: the arm on its pedestal, the feeder on the upper arm, the
  power source and gas cylinder on the floor, a welding table, and a weldment
  clamped on the table. The power source's `mains` is the cell's exposed port.
- Checks: the TCP pose at the ready pose; the torch neck clears the arm; cables
  are routed from port to port; the weldment's seams are within reach. Add a
  Start-page entry.

Done (see Progress). What was built and decided:
- **Arm tool.** `RobotArm(withCell, ?armTool)` takes an `ArmTool` (in
  `robot-arm/RobotArm.hx`): `build(flange)` makes the end effector, and
  `expose(arm)` publishes the tool's connectors and the inlets that must be
  supplied from outside. `ArmSuctionTool` is the default and exposes the same
  `toolContact` and `compressedAir` as before. `ArmWeldingTool` (in this
  directory) is the torch on an `EndEffectorPlate`, and exposes `toolTcp` and
  the torch inlets `torchPower`, `torchGas`, `torchWire`, `torchControl`.
- **Parts** in `machinekit.welding`: `WeldingTorch` (the breakaway mount is the
  torch's own base, not a separate part; 22° or 45° neck bending toward +X, 360 mm
  to the wire tip; `tcp` has +Z along the wire), `WireFeeder`,
  `WeldingPowerSource`, `GasCylinder`. Mass is declared, since the boxes are
  solid. All are code-only: no recipes yet.
- **New port kinds `Gas` and `Wire`.** Shielding gas must not connect to the
  compressed-air ejector, and the wire liner is a service of its own, so
  `PortKind` gained both (appended, so saved ids are unchanged).
- **Cabling follows the real set-up.** Cylinder to power source, power source to
  feeder (weld current, gas, control), feeder to torch (the four services). The
  feeder and the power source pass services through with port bridges. The
  work lead (`weldNegative`) is a port with nothing on it yet: it needs a work
  clamp on the weldment, which W2's "grounded work" will want.
- **Capabilities.** `ArcTorch(tcpConnector, controlPort)` and
  `WeldingSupply(processes, maxCurrentA, controlInterface)`, with
  `WeldingProcess` and `WeldingControlInterface` enums.
  `EndEffectorControls.derive` gives each torch an arc channel
  `<prefix>/<member>.arc` in a separate `arcs` list, not in `controls`: the
  RobotKit bridge switches exhaustively over `controls`, and W2 adds the
  binding there along with the `torch` robot-tool kind. Until then the
  preview emits no `robotTools`.
- **Feeder on the upper arm,** on a seat on the tube (`feederSeat`), so it moves
  with the arm. The torch's hose pack is a BOM line, not a routed hose.
- **Layout.** The table is 600 mm ahead of the arm and the weldment (300 x 150 x
  10 base plate, 300 x 8 x 80 upright, held by two fixture bars) sits 480 mm
  ahead. At the bisector torch angle the near seam would need the wrist 740 mm
  out, past the arm's reach at 600 mm, hence the closer weldment.
- **Checks** (`RobotWelderChecks`, in the MachineKit smoke suite): the services
  all reach the torch; the arc channel derives; at the ready pose the wire points
  down, 45° off the flange axis; a small damped-least-squares IK in the check
  reaches the start, middle and end of both seams with the wire on the
  plate/upright bisector; the torch clears every arm part, the feeder, the
  table, equipment and weldment in those poses, and the arm and feeder clear the
  fixed items; the BOM lists the equipment. That IK is only the check's: W4's
  reachability belongs to RobotKit.

**W1. Seams from the CAD.** The weldment is a small tube frame
(`FrameAssembly`) plus a plate T-joint, so there are fillet joints in tube and
in plate. A seam is not authored as a polyline. It is found where two members
meet: the intersection edges of their touching faces, named after the two
faces so the name survives edits. A `WeldSeam` has that edge reference, a
joint type (fillet, butt, lap), a leg size, a number of passes, and start/stop
direction. The seam frame is derived from the geometry: tangent along the edge,
torch axis on the bisector of the two face normals, then the travel angle
(push/pull) and the work angle.

Checks:
- a T-joint's seam frame bisects it at 45°;
- editing a tube size keeps the seam, which follows the new edge;
- a gap between members reports no seam.

**W2. The welder as a simulated robot tool.**
- The scene artifact's `robotTools` gains a `torch` kind with its channels:
  `weld.arc` (digital), `weld.wire_speed` and `weld.voltage` (analog), and the
  sensors `weld.arc_established`, `weld.current`, `weld.voltage_actual`,
  `weld.touch`, `weld.fault` and `weld.power`.
- RobotKit gains a `SimulatedWelder`, a `SimulationStepObserver` like the
  suction tool:
  - the arc lights when it is commanded on and the wire tip is within reach of
    grounded work;
  - current and voltage come from a simple synergic model of wire speed and
    stickout;
  - touch closes when the wire meets work while the arc is off;
  - faults: no arc within a timeout, wire stuck to the work;
  - power draw is arc power divided by the supply's efficiency.
- ProcessKit gains a `WelderProcessDevice`, the `ProcessDevice` over those
  channels: ready means the supply is ready, and safe means the arc is off.

**W3. Weld one seam.**
- The `mission` section gains a `weld` step that names a seam.
- A RobotKit `WeldSeam` skill runs a MotionKit program:
  1. a `MoveJ` to the approach pose and a `MoveL` to the start;
  2. arc on, holding on the path until arc established, with a start dwell;
  3. `FollowPath` along the seam frame at travel speed;
  4. a crater-fill dwell, arc off, burnback, then retract.
- `ProcessRun` drives it, so an arc loss mid-seam re-approaches and restarts
  with overlap.
- The bead grows as runtime geometry, appended per tick along the path the TCP
  actually travelled. Its cross-section comes from the deposition rate (wire
  speed × wire area ÷ travel speed), so the leg size it reaches is a result,
  and it is checked against the seam's leg size.
- Tests: one fillet weld on MuJoCo; the bead's leg is within 0.5 mm of the
  target; an arc loss is injected and the run recovers.

**W4. Weld the whole weldment.** Generate the mission from the weldment's
seams, so adding a member adds its seams. Order the seams to minimise air moves
and torch reorientation. Approaches and retracts must clear the work: collision
plan CL4 validation when it lands, until then a swept-volume clearance check. A
seam the torch can't reach at its angles is reported, not skipped.

**W5. Weaving and multi-pass.** A MotionKit path modifier lays a weave (sine,
triangle or zigzag: amplitude, frequency, dwell at the edges) across the seam
frame, keeping exact timing and events. The motion check covers the woven path,
not just the centreline. Multi-pass seams get root, fill and cap passes offset
in the seam frame, the bead from earlier passes counting as work.

**W6. Real welder interface.** Map the channels to:
- the retrofit I/O board: an optoMOS relay for the trigger, an isolated 0–10 V
  output for wire speed and voltage, a Hall-effect current sensor, an isolated
  voltage input, and a separate touch-sense circuit;
- a robot power source protocol: Modbus TCP registers, or I/O.

Then carry them over RKD6 like other process channels, with the arc channel
going safe (off) on every stop.

## Phase 2: mobile welder

These are the hardware choices made on 2026-10-02: a 48 V LFP pack (16s,
about 25 kWh for a shift) feeding a 48 V to 230 V pure-sine inverter (8–10 kW,
low-frequency type) that feeds the same single-phase power source as phase 1.
The arm, base and computer run from the pack through isolated DC-DC
converters. The pack floats (not connected to the chassis), and the work clamp
goes to the workpiece only.

**P1. On the mobile base.** `MobileBase(RobotArm(torch))`, with the power
source, inverter, pack and gas cylinder on the base. If they don't fit, the
base gets a variant with a larger plate. The power source's `mains` port mates
to the inverter instead of the wall. MachineKit gains `Inverter` and
`BatteryPack` (capability `BatteryStorage(nominalVolts, capacityWh)`).

**P2. Stations.** Choose the parking poses from reachability (RobotKit
construction M8): the fewest stations that cover every seam at its torch
angles, then order the stations and the seams within each. The mission becomes
`goTo` and `weld` steps.

**P3. Finding the part after parking.** The base parks with ±20 mm and ±2° of
noise (realistic AMR localization). Touch-sense search moves on two or three
faces near each station's seams give the workpiece's offset, and every seam
frame at that station is corrected by it. Test: with injected noise, the seams
still pass the leg-size check.

**P4. Energy.**
- A simulated 48 V pack integrates the welder's `weld.power`, the arm's draw
  (from joint torques and speeds), the drive's, and a fixed load for the
  computer into `BatteryState`. Its voltage sags with charge and load, and
  below a cutoff the arc can't be held.
- Before each seam, the mission checks there's enough energy left for the whole
  seam plus the drive to the dock. If not, it docks and charges first, because
  a seam must not stop part-way.
- The inverter's efficiency is a parameter (about 0.92; the whole chain is
  about 0.78).

**P5. Seam tracking (stretch).** SensorKit gains a laser line profiler built on
depth rendering. The seam is found before welding and corrected during it
(Lane D servoing).

## Order

W0 → W1 → W2 → W3 is the core: one seam from CAD, welded in simulation. W1
carries the modelling risk, because seams have to come from geometry, not from
authored lines. W4 completes the cell, W5 is process quality, and W6 is the
step to hardware. Phase 2 starts after W4.

Dependencies:
- W4's collision checks lean on the collision plan's CL3/CL4, with a fallback
  until those land.
- W1 leans on topological naming's edge persistence, which TN9 covered for edge
  picks.

## Progress

| Step | State | Commits |
| --- | --- | --- |
| W0 | done | tool-agnostic arm; welding parts and capabilities; cell, checks, Start entry |
| W1 | | |
| W2 | | |
| W3 | | |
| W4 | | |
| W5 | | |
| W6 | | |
