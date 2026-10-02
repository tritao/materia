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

Done (see Progress). What was built and decided:
- **Seams are found, never authored.** `machinekit.welding.WeldSeams.find(a, b)`
  takes two members' parts (in one frame) and reads their B-reps through
  CadKit's shape queries: faces with names, normals and boundary loops; edges
  with end points and adjacent faces. It needs no native change. Two kinds of
  contact between planar faces are looked for:
  - *edge on face* (fillet, lap): an edge of one member lies on a face of the
    other, and the member's face on that edge lies flat against it (opposite
    normals): the contact. The member's other face on the edge is the side face.
    The seam is the stretch of the edge where the other member's face continues
    past it, toward the side face's normal (a probe point a little beside the
    edge, tested in the face by even-odd over all its loops). Where the face
    stops (a flush end) there is no corner, so no seam. Only edges on the
    contact face's *outer* loop count, which leaves the inside of a hollow
    section alone. Joint type: fillet when the contact is narrower than the side
    face is tall (a T-joint), lap when it is wider.
  - *edge to edge* (butt, corner): an edge of each member coincides and the
    faces beside it meet flat; the seam is the groove the two other faces make.
    Flat groove faces give a butt, others a corner (mitred or square, inside or
    out).
  Both directions are tried, and a seam found from both sides is kept once.
- **Names.** A seam is `member:face|member:face`, the two faces being the ones the
  weld metal fills against (the plate's top and the upright's side), ordered by
  name, so the same pair always gives the same seam. `member` is the occurrence
  id and `face` is the CadKit topological name in that member's geometry. A
  second seam between the same two faces (a face cut into pieces) is numbered
  `~1`, `~2`, … by position.
- **Seam frame.** The tangent is `normalA × normalB` (so the direction is a result of the faces, and
  the two sides of an upright run opposite ways; flat butt faces go the
  positive way along the edge); `reversed()` flips it. The torch axis is the
  bisector of the two outward normals, pointing into the open side. The work angle
  tilts it about the tangent toward the first face; the travel angle then swings the wire toward the
  direction of travel (push, positive; the default is 10°) or away (drag).
  `SeamFrame.toAssemblyFrame()` gives the wire tip's pose: +Z along the wire,
  +X along the travel. `frameAt(distance)` and `frameAtParameter(fraction)` sample.
  Planar faces and straight edges only: planar-planar contact is always a line,
  and CadKit has no face normal at a point yet, so curved seams (round tube
  against a plate) are not found.
- **Weldment.** `Weldment(reference, members)` declares the joints with
  `join(a, b, legSize, passes, travelAngle, workAngle)`. `find(parts)` (or
  `findIn(assembly, poses)`) returns the seams and a `Diagnostics`: a declared
  pair with no seam is an error `weld.no-seam` naming the pair and the gap
  between their bounds, never a silent skip, and `require()` throws on any. The
  parts' pose is only used to place the parts: seams come out *in the
  workpiece's frame* (the reference member's pose), so a workpiece pose
  correction (phase 2's touch sensing) is just a different pose to
  `SeamFrame.transformed`, and seams don't change when the whole workpiece moves.
- **Tube perimeters** give one seam per side, since each side lies between a
  different pair of faces and the faces' angle (and so the torch axis) changes at
  every corner. `WeldSeams.chains(seams)` joins sides that meet end to end into
  ordered, direction-corrected chains with a `closed` flag, for W4 to weld a
  post as one run. The choice (a seam per side, a chain on top) keeps names and
  frames simple and still lets a program corner-weld.
- **The workpiece** (`WeldingWorkpiece`, a `MachineAssembly` included in the
  cell as `work`): the base plate and upright, and a `FrameAssembly` with a
  beam and two posts standing on it, each member its own occurrence
  (`FrameMemberComponent`). The beam is 20 mm wider than the posts so a fillet
  runs round each foot; with equal tubes the sides would be flush edges, which
  are not fillets and are rightly not found. The upright is as long as the plate,
  so its ends are flush and only its long sides are seams; a shorter upright gains
  end seams. Ten seams in all: two of 180 mm and eight of 40 mm.
- **Layout.** Posts 80 mm tall stand 140 mm apart, so the torch at 45° clears the
  opposite post, and 110 mm from the upright, which the same clearance needs. The
  plate is 180 mm long (was 300), the frame is 290 mm to its side, and the whole
  workpiece sits at x = −155 so that every seam is in the arm's reach with the
  10° push.
- **Checks** (`WeldSeamChecks`, run from `RobotWelderChecks`): the T-joint's
  seam names, ends and length equal the plate's; the torch axis is 45° to both
  faces and square to the seam, the push and work angles do what they say; each
  post has four fillet seams that chain into a closed loop; editing the upright
  length and the tube size keeps every name and moves the edge (and the shorter
  upright gains its end seams); the seams are the same on their own and in the
  cell; a 0.5 mm gap reports no seam with the gap; lap, butt and mitred-corner
  joints are told apart. The IK reach check and the clearance check now use the
  derived seam frames.

Left open: seams on curved faces; accessibility (a seam on the inside of a
tight corner is found even if no torch fits there: W4's reach and clearance
checks decide); touching tolerance (`WeldSeams.TOUCH`, 0.01 mm) is a constant;
leg size is declared, not derived from the geometry.

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

Done (see Progress). What was built and decided:
- **Work clamp and grounding.** `WorkClamp` is a magnetic clamp with a `lead` inlet and a `contact`
  connector (capability `WorkReturn(leadPort, contactConnector)`). The cell's `work-lead` cable runs from
  the power source's `weldNegative` to it, and the clamp is mated to the weldment's base plate, not to the table:
  a loose part lying on a table is not a dependable conductor (scale, paint, a thin edge), a magnet needs no
  re-clamping per part, and phase 2 has no table, so the clamp goes to the workpiece in both phases.
  `WeldingEquipment.of(assembly, weldments)` derives the grounded work: it traces each clamp's lead
  upstream to the power source's work lead, takes the member the clamp is mated to, and adds every member
  welded to it (a `Weldment`'s members are one conductor). A mate does not conduct (the table and fixtures stay
  out unless the clamp is on them). It throws when the work lead reaches no clamp or the clamp is mated to
  nothing. It also reads the supply's rating and efficiency (`WeldingSupply` gained `efficiency`, 0.88 for the
  inverter source) and the wire (new capability `WireFeed(wireDiameterMm, maxSpeedMPerMin)` on the feeder).
- **Channels** follow the suction convention, `<prefix>/<member>.<signal>`: `…torch.arc` (digital),
  `…torch.wire_speed` (m/min) and `…torch.voltage` (V), and one sensor `…torch.weld`. The six sensors of the
  plan are one `tool_weld` frame of six values (arc established, current A, voltage V, touch, fault code, mains
  power W; `WeldSensor`): one frame is a consistent snapshot of the circuit and one sensor to mount, where six
  would be sampled apart. A real welder publishes the same frame. `EndEffectorControls` now has an `Arc`
  control in `controls` (so cadbridge's exhaustive switch binds it, as an inlet check: the kinematic
  `ToolRuntime` has no arc) and the torch's analogue channels and sensor in `arcs`.
- **Scene artifact.** `robotTools` gains kind `torch`: `contact` is the wire tip, `channel` the arc, `sensor` the
  weld sensor, and `torch` (`SceneArtifactTorch`) has the wire speed and voltage channels, `groundedWork`
  (occurrence ids), the supply's `maxCurrentA` and `efficiency`, `wireDiameterMm` and `stickoutMm`. Validation
  checks the connector, distinct channels, one torch, existing distinct grounded occurrences and sane ranges.
  The section is JSON, so there is no format version bump (M5 added `robotTools` inside version 13 the same way).
  `AssemblyPreview.robotTools(tool, prefix, ?equipment)` emits it; the cell's preview does, and the W0 check that
  asserted none now asserts the torch.
- **Arc model** (`robotkit.tool.WeldArcModel`, pure and deterministic; documented in its header). With the
  arc commanded, the supply ready and the wire fed (at least 1 m/min), the arc strikes when grounded work is
  within 5 mm ahead of the wire tip along the wire (the wire advances to scratch it), is established 80 ms
  later, and burns while the arc length (tip distance + 3 mm nominal) stays at most 12 mm. Current is the
  positive root of the burn-off law `MR = α·I·A0/A + β·l·I²·(A0/A)²` (MR in mm/s, α = 0.27, β = 7e-5, stickout
  l = 15 mm, scaled by wire cross-section against the 1.2 mm reference), capped at the supply's rating:
  8 m/min of 1.2 mm wire is 250 A. Voltage is the setpoint plus 1.2 V per mm of arc length beyond nominal
  (at least 10 V). With the arc off the sensing voltage is 24 V, collapsing to 0 on touch; touch is the tip
  within 0.5 mm of grounded work (or inside it) with the arc off. Power is `I·U/η`. Faults latch until the arc
  is commanded off (a stuck wire also until the torch is free): `NoArc` (commanded 1 s without striking),
  `ArcLost` (went out while commanded: pulled away, supply dropped out), `WireStuck` (arc commanded off with the
  wire still feeding while touching: no burnback). Stopping the wire first is a burnback and ends the arc
  without a fault.
- **Geometry.** The physics engine reports contacts of collision shapes, and the wire is none: it is
  millimetres of metal beyond the nozzle and the arc strikes across a gap, which contacts do not report. So
  `SimulatedWelder` (a `SimulationStepObserver` like the suction tool) asks the grounded work directly, in
  the world frame: the tip's distance to it and a ray along the wire. The work is `GroundedWork`, convex solids
  (`ConvexSolid`, planes from the hull vertices; boxed above 80 vertices) on the links that carry them, taken
  from the same collision hulls the simulation uses; an occurrence without a hull is refused. Outside a solid
  the distance is the largest face-plane distance, exact in front of a face and a little short at an edge.
- **Safe on stops.** `WeldChannels.declarations`: the arc and wire speed are not `keepOnStop`, so any stop
  or fault takes them to off and zero; the voltage setpoint keeps its value. The app wires it in
  `AssemblyRobot` (sensor kind, channel declarations, the hulls) and `SimulatedTools.welderFor`, built from the
  tool's connector, so a project with a torch tool runs a `SimulatedWelder`.
- **ProcessKit** `WelderProcessDevice` speaks `WelderOutputs` (arc, wire speed, voltage) and `WelderFeedback`
  (the `tool_weld` reading): `prepare` sets the voltage with the arc off and the wire still, `ready` is prepared
  and no fault, `fault` the supply's fault in words, `safe` arc off then wire stopped (and needs a new prepare),
  `apply` carries out fired records (the arc takes a digital or an analog rate above zero, as a process run emits
  it). Nothing in it is RobotKit's simulation.

Tests: `WeldTests` (RobotKit: solids, arc strike near work and not in air, ignition delay, burn-off current and
voltage, arc lost, touch, no-arc timeout, power, stuck wire and burnback, safe channels, frame),
`WelderProcessTests`, `ProjectKitTests.torchTool`, `CadBridgeTests.testTorchBindings`, and the MachineKit
smoke's `checkTorchTool` (grounded work is the weldment). The first two and ProjectKit's run on the host with
no native build; the cadbridge test and the app only type-check in this checkout (their native build needs
submodules it does not have).

Left open: `SimulatedWelder` itself (the part that reads channels and the simulation) is not run in any test
here, since it needs a physics backend; everything it decides is in the tested model and work. Grounded work
must be a part of the assembly robot (a free workpiece is refused). `supplyReady` is an input of the simulated
welder, not yet derived from the mains, gas or battery, which phase 2's energy model can give it. Shielding gas
is not modelled. Touch is a binary contact, not a model of the sensing circuit. The W0 note that the arc was
kept apart in `arcs` is replaced by the `Arc` control above.

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
| W1 | done | seams found from member faces (`WeldSeams`, `WeldSeam`, `Weldment`); workpiece with a tube frame; checks |
| W2 | done | work clamp and derived grounded work; `torch` robot tool in the scene artifact; `SimulatedWelder` + `WeldArcModel`; `WelderProcessDevice`; tests |
| W3 | | |
| W4 | | |
| W5 | | |
| W6 | | |
