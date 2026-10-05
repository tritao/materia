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
- **Arc model** (`processkit.tool.WeldArcModel`, pure and deterministic; documented in its header). With the
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
smoke's `checkTorchTool` (grounded work is the weldment).

**Verification (done with W3).** W2 was first only compile-checked where it touches a physics backend. It has
since been run for real, in the repo's own test projects: `robotkit/tests/tool-capabilities` (the tool, process
and weld tests), `processkit/tests`, `projectkit/tests`, `robotkit/cadbridge/tests` (147 assertions, including
`testTorchBindings`) and the MachineKit smoke all pass. `SimulatedWelder` runs against the real simulation, on
MuJoCo and on the deterministic test backend, through the app's project-source suite: the robot-welder project
opens, the torch tool is created with its grounded work from the collision hulls, a program moves the wire tip to
the seam with the arc on, the arc strikes once the tip is at the work (after the model's 80 ms ignition delay) and burns
at 250 A (8 m/min of 1.2 mm wire, as the burn-off law says), a one-tick dropout of the supply loses the arc with the `ArcLost` fault and
the fault clears when the stop takes the arc channel to off, and a torch held 30 mm clear of the work faults with
`NoArc` each time. Nothing in W2 needed fixing; what the runs added is W3.

Left open: Grounded work
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

Done (see Progress). What was built and decided:
- **The `weld` step.** A mission step `{kind: "weld", weld: SceneArtifactWeld}` is a *path* of line segments (see
  Hardening below), each naming the seam it lies along by its W1 name (`member:face|member:face`) and carrying what the
  generator derived from it, so a player needs no CAD: the wire-tip poses at its two ends (they carry the work and push
  angles), the two faces' outward normals (they say where the metal sits) and the joint type (only `fillet` is deposited).
  The weld has the declared leg, the part that carries the weld metal (`metal`), the occurrence the path is relative to
  (`frame`) and a `process`. Lengths are metres, as in the machining section. Validation checks the names against the
  assembly's occurrences, the poses, the normals, ranges for every process value, and that the robot has exactly one torch.
- **The process lives in the step, derived from the leg.** `machinekit.welding.WeldingRecipe.fillet(leg)` picks the
  wire speed for the leg (`1.5·leg + 0.5` m/min), the voltage on a synergic line (`15 + 1.1·wire speed`), and the travel
  speed at which that wire speed deposits an equal-leg fillet's section: `v = wire speed · wire area · efficiency / (leg²/2)`.
  For the 5 mm leg: 8 m/min, 24 V, 11.46 mm/s, with 0.15 s start and crater dwells, 0.1 s burnback and a 40 mm approach.
  The values are written into the step, not looked up by a recipe name: the artifact is explicit and a player needs
  no table. The leg the weld reaches is then measured on the metal the torch laid, which is compared with the leg asked for (see Hardening: the recipe assumes the wire's deposition efficiency, the bead is what was laid).
- **`WeldSeam` skill** (RobotKit) is `HandlePart`'s sibling: a `WeldPlan` (the two tip poses and the parameters, read
  when the skill starts and taken from the map frame to the base's), a `WeldRunner` boundary that RobotKit names and
  ProcessKit implements, and the `tool_weld` sensor to read the outcome from. It succeeds when the runner is done and the
  welder reports its arc out and no fault. It only uses the robot - the torch's channels, the arm's motion and the welder's
  sensor - so a mobile base can run it from any station (phase 2).
- **ProcessKit `ProcessRun` drives it, with two additions.** The model fit for the path and the recovery
  (interrupt on a device fault, safe the device, re-approach `recoveryBackoff` before where it stopped, so the new bead
  overlaps the old), but a weld has to be engaged before the path and disengaged after it. A `ProcessRecipe` now
  carries an optional `ProcessEngagement`: operations that run after the approach move (`entry`) and after the path
  (`exit`), run again by every restart. With an exit the path's events do not zero the process output at the path's end,
  and the run ends when the caller says the program is done (`ProcessRun.finish`), not at the path's end. `interruptNow`
  lets the caller interrupt for a reason the device did not report (an arc that never established), and `followOp` says
  which operation follows the path. The recipe also has an `approachSpeed`, so the approach is not at the travel speed.
  Nothing changes for a recipe without an engagement (the paint and dispense tests are unchanged).
- **`WeldingPlanRunner` (ProcessKit)** is the `WeldRunner` over a `ManipulatorMotion` and a `WelderProcessDevice`. Its
  program: `MoveJ` to the approach pose (the start, `approach` out along the wire), `MoveL` to the start, then the process
  run's: wire speed and arc on (a restart's first move aims at the backed-up point on the seam), `WaitInput` on the
  established arc with a 2 s timeout, the start dwell, `FollowPath` along the seam at the travel speed with the wire speed
  that keeps the deposit per length constant, the crater dwell with the arc up, the wire stopped, a 5 mm lift over the
  burnback time (the arc, with nothing feeding it, burns back; the model goes out without a fault) and the arc command
  ended, then the retract to `approach` out along the wire from the seam's end. The runner watches the welder's fault while the program runs; a fault
  makes the process run interrupt, the arm stops (`abort`, which takes the arc and wire channels to off), and once the
  fault has cleared the run restarts; after `maxRestarts` (3) the weld fails with the reason. Where the plan says
  "arc off; burnback (wire stops after arc off)" the order is the other way round on purpose: the model, like a real supply,
  burns back when the wire stops first, and an arc commanded off while the wire still feeds sticks the wire (`WireStuck`).
- **Outputs of a process device in simulation.** A channel can only be written by a motion program, so
  `ChannelWelderOutputs` (the `WelderOutputs` of a welder worked through the robot's channels) holds the device's writes
  (`prepare`, `safe`) and the runner puts them at the start of the next program as `SetOutput` operations. A stop already
  takes the channels to their safe values, so a `safe` is the same value again. The input wait reads the established
  arc from the same reading the device reads.
- **MotionKit needed nothing.** Its output events attach to a motion (a `SetOutput` after a barrier leads the next move,
  and a program may not end on one), which shaped the exit: the wire stop leads the lift, the arc command ends it.
- **The bead** (`processkit.tool.WeldBead`, pure) takes each tick the arc state, the wire speed and the wire tip, and
  keeps the metal at each 1 mm station of the seam: melted wire (`wire speed · A · dt`) times the deposit efficiency (0.95)
  goes to the station the tip is over, so a station's section is deposition over travel speed, and standing still piles
  metal into a crater. An equal-leg triangular fillet's leg is `sqrt(2·area)`. It is the result that is compared with the
  declared leg. Metal that lands off the seam is counted as `stray`, restarts are told apart (`runs(i)`), and a bare stretch
  between metal is a `gap`.
- **Showing it** (`app/WeldBeads`, a session member): the part named by the step's `metal` (a 1 mm speck of steel,
  `machinekit.welding.WeldMetal`, mated to the workpiece's reference member, in the BOM as the weld metal) takes the bead as
  runtime geometry in chunks of 20 stations, re-meshed at most every 0.1 s of simulation time and only where the bead grew,
  with a triangular section per station, legs along the two faces, an end cap where the metal stops. A reset takes it off.
- **`MissionPlayer`** runs `weld` as it runs `pick` and `place`, from the one torch tool (the arm's tool frame is the wire
  tip). A mission that does not loop now stops after its last step; before it started the last step again, which for a weld
  would have laid a second bead.
- **The generator** (`RobotWelderPreview.cell`) finds the T-joint's `+y` seam from the geometry, places it where the
  workpiece stands, and writes the mission; `RobotWelderChecks.checkMission` checks the seam, the leg the recipe's numbers
  deposit, and the torch pointing down.

Results (project-source suite, `PROJECT_SOURCE_ONLY=welder`): on MuJoCo and on the test backend the mission welds the
180 mm seam in 20.2 s of simulation time: the arc burns for 1603 ticks (16 s) at up to 250 A, the wire tip stays within
0.1 mm of the seam line, the bead is 180 mm long with no gap, its mean leg over the middle 70% is 5.0 mm (4.8 to 5.1) against
the 5 mm asked for (the test allows 0.5 mm), and no metal misses the seam. A run that loses the arc half way (the supply
drops out for one tick) stops, backs up 10 mm along the seam, strikes again and finishes in 21.6 s: one restart, an 11 mm
overlap whose metal is doubled (the leg there is 9.4 mm, a restart hump), mean leg 5.2 mm, no gap. A seam held 30 mm clear of
the work faults `NoArc` four times in 7.5 s and the weld fails with "the arc could not be held in 3 restarts".

Left open: only straight seams of fillet joints are deposited (butt, lap and corner joints would need other sections, curved
seams a path of frames). The section is a straight-faced triangle; real beads are convex and the leg for a given area
differs by a few percent, which the equal-leg law does not capture, and a start hump and crater dip are shaped only by the
dwells. The wire speed, voltage and travel speed come from a rule of thumb, not a synergic table; the voltage setpoint does
not change the simulated arc beyond the arc length. The approach and retract are not checked for collisions (W4). The
`WaitInput` timeout and the restart count are constants of the runner. A restart re-strikes where the arm stopped, a few
millimetres past where the arc went out, so a long-lost arc leaves a bare stretch the backoff must cover (10 mm). Seam
frames are the workpiece as designed: no pose correction yet (phase 2). The suite's exit crash (a double free or segmentation
fault now and then after the results were printed) is explained and fixed under Hardening below.

## Hardening after W3

Done before W4, on the review of W0-W3. What was found and decided:

- **Native host libraries are the worktree's own.** The project-source runs borrowed another worktree's
  `app/build/host/native`, which can change under a run. A plain `haxeon build --project app/haxeon.project-source.json
  --output=<wt>/app/build/host/project-source.hl` (no `--compiler-only`) builds them into `<wt>/app/build/host/native`
  (`app`, `kinematicskit-native`, `stockkit`; 207 MB, about 8 minutes on 20 cores, against the shared prebuilt OCCT via
  `CADKIT_OCCT_DIR`). It needs the worktree's submodules for `animkit/native/vendor/*` and `uikit/vendor/*`
  (`git clone --shared --no-checkout <src> <wt>/<path>` and `git checkout <pinned sha>`; no network). After that
  `--compiler-only` rebuilds the test program in a minute, and the run takes
  `LD_LIBRARY_PATH=<wt>/haxeon/out:<wt>/haxeon/.tools/hashlink:<wt>/app/build/host/native/{app,kinematicskit-native,stockkit}:$OCCT/lib`.
  The tests in `robotkit/tests`, `motionkit/tests` and the like build their own `robotd_native` the same way.
- **The exit crash: root cause.** A core from one crash (a segmentation fault 1 run in 24 with the borrowed libraries, none
  in 39 runs of the new ones) shows the main thread inside `exit()`, running static destructors (`~HandleMap<Session>` of
  the physics library, which tears a still-open MuJoCo session down), while three `ProgramPlanner` worker threads are
  still inside MotionKit's native library (`mk_time_path`, `mk_trajectory_append_segment`), one of them crashing in a
  `shared_ptr` release of the library's own tables. So the cause is the process exiting while program-planning workers
  were still planning: `ProgramPlanner.dispose` returns at once and the worker finishes the plan it is on (seconds, in a
  debug build), and a weld that gives up, or an arm that is mid-mission when its test ends, leaves such workers. The
  suite showed it at the end of the arm check (2 planners still planning), the weld in the air, and so on; the
  `simulation.clear()` the W3 agent added was a partial cure (it tore the session down in order, but waited for no worker).
  Fix, in the layer that owns the workers: `ProgramPlanner` keeps the list of planners with a live worker, and
  `ProgramPlanner.shutdown()` cancels them all and waits for the workers to stop. `ApplicationSimulation.clear()` calls it
  (a world must not go while its programs are planned), so does the editor's close handler, and the project-source suite
  calls it at its end and fails if a worker is left. MotionKit has a test for it.
- **Arc safe on every stop.** The runtime does it (`RobotRuntime::safe_channels`); `WeldChannelTests` (RobotKit world
  tests) prove it on the real runtime, with the declarations the app uses (`WeldChannels.declarations`,
  `SuctionChannels.declaration`): a commanded stop and an abort take arc and wire speed to off and zero and keep the voltage
  setpoint and a suction tool's vacuum; an emergency stop and a robot fault (a target beyond the joint's limit) take all of
  them safe. No gap was found. The declaration of the suction channel moved out of the app into RobotKit with the welder's.
- **The weld step is a path, relative to the workpiece.** `SceneArtifactWeld` is `{frame, metal, path, legSize, process}`;
  `path` is a list of segments `{kind: "line", seam, joint, start, stop, normals}`, each starting where the one before ends
  (validated), so a straight seam is one segment, an `arc` kind can come later, and the four sides of a tube post
  (`WeldSeams.chains`) are one step of four segments, each with its own faces. The process is one per step (one leg).
  An older step with `seam`, `joint`, `start`, `stop` and `normals` on the weld decodes to a path of one segment with no
  `frame` (the assembly as designed); the artifact version did not change (the section is JSON, as for M5 and W3).
  `frame` is the occurrence the path is relative to (the weldment's reference member): the generator gives the seams in
  that member's frame, and `MissionPlayer` places the path by the member's live pose when the step starts, as a pick finds
  its part; `WeldBeads` keeps the stations in the member's frame and takes the wire tip into it each tick, so the weld and
  its bead follow a workpiece that is not where it was designed (a test moves the table 6 mm, 4 mm and 3 degrees: the
  tip stays within 0.1 mm of the seam where the workpiece really is and the leg is 5.0 mm).
- **A chain in one step, with the torch turning at the corners.** `WeldingPlanRunner` follows all segments as one
  `FollowPath` with the arc up throughout. Where the next segment's orientation differs, the torch turns half the way on
  the last 12 mm (at most 45%) of one segment and half on the first 12 mm of the next, so the turn is part of the travel,
  at the travel speed, and no metal is piled where the torch would otherwise stand to turn (stopping and restarting at
  each corner was the alternative; it leaves a crater and a start at every corner). The roll of the torch about its wire is
  free for the weld, but the seam frame fixes it (the neck leads), and for the sides of a tube that fixes four rolls of
  which the arm holds one: so the runner picks, per segment, the roll nearest the previous one at which the arm can follow
  the segment, the corner and the approach or lift without leaving its IK branch (`withRolls`). The weld of one post's
  perimeter (four 40 mm sides, closed) on MuJoCo takes 19.9 s without losing the arc, the tip stays within 0.1 mm of the
  four seams, and the legs are 5.2, 5.1, 5.2 and 5.0 mm. A chain that is not weldable (no roll the arm follows) is
  reported by the planner as an unreachable pose, not skipped. `WeldPathBead` keeps a bead per segment, each with its own
  faces, and puts the metal into the segment whose line the tip is nearest. The closed chain ends at its start; there is
  no extra overlap there, and a start/stop hump at that corner is only what the dwells make.
- **Restart point.** A restart backed up 10 mm from where an interruption was *reported*, and an interruption before
  the torch reached the seam on a restart (a failed re-strike) reports the restart point itself, so each failed re-strike
  moved the point 10 mm further back. `ProcessRun` now backs up only when the process got beyond the point its program began at;
  a failed re-strike retries the same point (unit test, five failures in a row). A fault in the crater, burnback or lift,
  after the seam is travelled, no longer travels it again: the rest of the program ends the arc, which clears the fault
  (tested with a supply dropout in the crater: no restart, no overlap, the bead whole).
- **Preparing times out.** A welder holding a fault (a wire stuck to the work) never became ready and the weld hung.
  The recipe has a `prepareTimeout` (the welding runner uses 2 s) after which `ProcessRun` is `Failed` with the welder's
  fault as the reason, and the weld fails with it.
- **The leg is measured.** The deposition efficiency has one source, the wire feeder's `WireFeed` capability
  (`WireFeeder.SOLID_WIRE_EFFICIENCY`), carried in the scene's torch block to the recipe and to `WeldBead`, which has no
  default of its own. The recipe is made for the CAD's wire (diameter, efficiency, top speed): a leg that needs more wire speed
  than the feeder feeds is refused. The bead is what the torch laid, so the leg *is* a measurement, and an independent
  test (`testLegFollowsTravelSpeed`) welds at other speeds than the recipe's and checks the sqrt(1/v) law, the metal balance
  (bead volume = wire fed times efficiency) and another wire's efficiency. The earlier wording that the recipe and the
  bead agree "by construction" is replaced by this: they share one measured fact (the efficiency), not one formula.
- **Validation holes closed.** A mission that picks and welds is rejected (one arm, one tool); parallel face normals are
  rejected (they make no corner and would throw when the bead is built); the wire must point into the corner (against
  the sum of the outward normals); the burnback has a floor (0.05 s: with none, wire and arc stop together and the wire
  sticks); the wire speed may not pass the torch block's top speed.
- **Smaller.** The wait for the arc to establish is recognised by the program's operation index, not by matching an
  error string. A program from `ProcessRun.takeProgram` may not end on an output change (the caller gives the closing
  motion; the welding runner gives its retreat), or it throws. The welder device's `safe` writes the wire stop then the
  arc off, as the program's exit and `ChannelWelderOutputs.drain` do (burnback order); a robot stop or fault cuts both
  channels at once. `ConvexSolid.distance` is now the true distance to the solid (edges and corners were 0.71 and 0.58
  of it), found by projection with Dykstra's corrections when the perpendicular foot on the nearest plane is not in the solid.

Open after hardening: the roll search and the path's IK are about the arm's reach, not clearance (W4 checks the torch
and neck against the work along the path); the corner turn is a fixed length; the arc, wire and voltage channels are
declared by the app for a project's tool, while the runtime enforces what they say (a project that skips the declaration
has no stop policy).

**W4. Weld the whole weldment.** Generate the mission from the weldment's
seams, so adding a member adds its seams. Order the seams to minimise air moves
and torch reorientation. Approaches and retracts must clear the work: collision
plan CL4 validation when it lands, until then a swept-volume clearance check. A
seam the torch can't reach at its angles is reported, not skipped.

W4 implementation notes (done, 2026-10-04):
- Corner length comes from a rest-to-rest angular turn with half the wrist's speed and acceleration reserved
  for reorientation. With `w = 0.5 * angularSpeed` and `a = 0.5 * angularAcceleration`, its time is
  `2 * sqrt(angle / a)` below the speed limit, otherwise `angle / w + w / a`. Each side contributes
  `travelSpeed * time / 2`, bounded to 4–25 mm and at most 45% of that segment; straight joins contribute zero.
  The compiler still enforces the actual coupled joint limits, and process quantity follows any resulting slowdown.
- ProcessKit owns welding preparation, ignition, interruption/re-strike, crater fill, burnback and retreat.
  RobotKit's generic stop/fault policy cuts the tool outputs; neither the arm interface nor its spec table gains
  welding policy. CAD capabilities provide the torch stickout, supply limits, wire and grounded-work connections.
- The torch is three collision bodies: mount/handle, neck and nozzle/contact tip, rather than a single convex hull
  filling the crook. The cell uses an 8 mm neck radius and 10 mm nozzle radius; posts are 60 mm tall with centres
  80 mm from the beam centre. These CAD dimensions replace the initial draft's 80 mm height / 70 mm spacing and
  the original 9 mm neck / 13 mm nozzle radii. Seam side lengths remain 40 mm and the requested legs remain 5 mm.
  The full-cell planner checks 318,233 poses for the first post and 121,608 for the second; measured planning
  costs are approximately 112–177 s per post and 1–14 s per plate seam on this machine, separate from cycle time.
- ProcessKit checks the whole run before accepting an entry IK configuration. If the CAD tour's preferred corner
  cannot start a closed perimeter, the planner tries its other corners, preserving every seam and its direction.
  This is a reach/clearance decision; the CAD mission remains unchanged. The focused planning suite covers both
  a later entry branch needed to finish a seam and a different entry corner needed to weld a closed run.
- At corners, the roll search first tries the roll that continues the previous orientation by the wire's shortest
  swing without additional twist. Numeric roll proximity alone ignores the next seam frame's change. The discrete
  roll candidates remain alternatives for reach and clearance.
- The planner samples at 2 mm and enforces the compiler's per-joint continuity bounds before accepting a run.
  Individually reachable poses can otherwise hide a wrist branch jump until the arc is already up. A focused
  regression rejects such a path.
- Before ignition, each candidate's complete approach, weld, burnback lift and retreat is compiled using MotionKit's
  existing API, then the resulting native joint trajectories are checked against the cell. Trajectory samples are
  timed from the fastest joint's limit to bound travel between samples to 0.02 rad. Rejected candidates backtrack;
  no plan is submitted and no arc output is issued during validation. IK position accuracy is 0.05 mm, leaving
  headroom within the 0.5 mm compiled-path tolerance. Entry IK also uses the current arm configuration, since
  broad grid sampling alone can miss a reachable branch. If the CAD tour's chosen direction has no feasible entry,
  the planner tries reversed travel. It derives the reverse wire from the stored wire and seam tangent, reversing
  the push component and preserving the face cross-section component. No seam angles are guessed or hard-coded.
  Focused planning passes 44 assertions; full-cell execution passes on MuJoCo and the test backend.
- The cell owns its ready configuration (`WeldingCell.readyPose`); the preview writes that configuration into the
  scene's assembly state and uses it for CAD checks. The arm and its tool interface need no welding-specific method.
- Pre-path weld decoding is removed: a weld without an explicit segment path is rejected. ProjectKit's rejection
  check replaces its former migration check (131 assertions pass, commit `05f1b67f4`).
- Initial draft validation: ProcessKit's welder tests pass 52 assertions, weld planning passes 28, and the remaining
  process tests pass 23. The planning fixture chooses roll zero in free space (51 checked poses), then a quarter-turn
  beside a wall (126 poses); a blocked seam reports the colliding bodies and pose. Full cell validation is pending.
- Cell clearance uses 3 mm for arm and air motions and 0.5 mm for tool/work pairs in the working zone. The generic
  RobotKit default stays 5 mm. CAD inspection of X8's drive installation gives a 4.5 mm intentional gap at j2:
  the gearbox ends at 0.5 + 45 mm in the 90 mm housing, and the next tube extends back by its 40 mm radius.
  Thus the generic 5 mm margin rejects every welding configuration of that assembly; 3 mm retains positive
  clearance without excluding the gearhead or editing the arm. The runtime check uses the same configured margins
  on actual joint snapshots. No colliding bodies are exempted to obtain a plan.
- R3 and R4 landed before W4's final gate. They were merged from local main, and the remaining planner imports
  were repaired separately (`4f7e9ccaf`). ProcessKit still passes 52 welder, 44 planning and 23 process assertions.
- Full-cell execution reached all four runs, but the first post side measured a 6.98 mm leg: constant wire speed
  was overfeeding a trajectory slowed by joint limits. The correction stays in ProcessKit: validate the program's
  time law, then schedule wire rates as quantity per seam metre times each interval's actual progress speed.
  Held rates conserve the requested quantity over each interval; the schedule is bounded per native section,
  and the crater restores the recipe's wire speed. The motion and device execute the events together. Focused
  timing tests cover a tenfold slowdown, section offsets, event-budget coalescing and invalid time maps (11 assertions).
  The MuJoCo whole-cell check now passes: 106.5 s for four runs and ten seams, legs 4.9–5.1 mm,
  bead lengths 40 mm on each of the eight post sides and 180 mm on each plate fillet, tip within 0.1 mm,
  no clearance violation and no restart. The test backend also passes in 106.5 s with legs 4.9–5.1 mm,
  full bead lengths and no clearance violation or restart. The complete welder app gate exits successfully.
- Baseline timing changes are intentional consequences of W4's validated motion. The cell starts at shoulder
  0.25 rad instead of the arm's generic 0.4 rad, keeping the torch above the table. Entry selection now checks
  whole-path joint continuity and swept clearance instead of resolving an unchecked nearest pose target.
  The single-seam cycle is 20.6 s (formerly 20.2), and one arc-loss recovery is 22.0 s on MuJoCo / 22.1 s on
  the test backend (formerly 21.6). Both retain full 180 mm beads with no gaps; the undisturbed leg remains
  5.0 mm (4.8–5.1). Recovery retains one restart and 11 mm overlap; reducing its hump belongs to W5.
  The focused post cycle is 29.8 s (formerly 19.9), legs 4.9/5.0/5.0/5.1 mm, no restart. Its roll/entry
  must now clear the cell, and its turn is derived from wrist limits rather than the old fixed 12 mm ramp;
  MotionKit can slow the turn under those limits, with wire feed following the actual progress. The previous
  reach-only timing is not retained by bypassing clearance or angular limits. The in-air seam still fails with
  “the arc could not be held in 3 restarts”; crater dropout finishes with no restart and a whole bead;
  displacement by 12.3 mm leaves the tip within 0.1 mm of the real seam and the leg at 5.0 mm.
- Final W4 gates pass: ProcessKit 52 welder / 44 planning / 11 rate-schedule / 23 process assertions;
  RobotKit 27 tool / 33 process / 152 weld / 33 clearance assertions; RobotKit world 4,950 assertions;
  CAD bridge 157; ProjectKit 131. MachineKit smoke passes, including the complete mission and rejection checks.
  The generated tour covers 680 mm, with 386 mm air travel and 176 degrees of turning versus 846 mm / 290 degrees
  in discovery order. The app compiles against R5 and passes welder, arm and mobile. Arm remains 23.5 s;
  the MuJoCo mobile mission completes in 61 s and its obstacle round in 65 s with one replan.
  The whole-weldment test enforces 2 mm bead-length tolerance and 0.5 mm leg tolerance per seam.
  R3/R4 landed early and their import repair is separate; R5's package move is merged with ConvexDistance in core.
  W5 can start after this W4 sync, with the R0/R3/R4 prerequisites already present.

**W5. Weaving and multi-pass.** A MotionKit path modifier lays a weave (sine,
triangle or zigzag: amplitude, frequency, dwell at the edges) across the seam
frame, keeping exact timing and events. The motion check covers the woven path,
not just the centreline. Multi-pass seams get root, fill and cap passes offset
in the seam frame, the bead from earlier passes counting as work.

W5 design note while waiting for R3/R4 (2026-10-04; implementation has not started):
- Wrap `PosePrimitive` and retain its length as seam progress, so `PosePath` events keep their original distances.
  The lateral offset uses the CAD seam's material frame, independent of the torch roll chosen for clearance.
  Its first and second derivatives include the material frame's derivatives; zero amplitude returns the base path.
- Keep weave phase continuous across primitives. Convert a frequency in cycles per second to cycles per metre using
  the recipe's nominal seam speed. An edge dwell holds the lateral offset while forward seam progress continues.
  Piecewise patterns need explicit joins and one-sided derivatives; do not hide derivative discontinuities in a
  numerical approximation. Taper the offset to zero at sharp seam joins to preserve a connected path.
- Deposition follows actual seam progress per second, including kinematic slowdowns. The nominal wire/area ratio
  is the recipe's policy; lateral tip speed during weaving must not increase the deposited area per seam metre.
- Use one current saved weld representation: a non-empty pass list over one CAD path. Each pass carries its process,
  offsets along the path's face normals, weave and interpass dwell (zero when no cooling is requested). The common
  path remains the deposition station reference, avoiding duplicated or independently drifting seam geometry.
  Bump the then-current scene version and regenerate fixtures; reject older versions.
- For the required 10 mm example, split its 50 mm² area into root/fill/cap fractions 1/4, 3/8 and 3/8
  (12.5, 18.75 and 18.75 mm²). Derive offsets from the seam's face normals and previously deposited height.
  Earlier station geometry must participate in both grounded-work sensing and clearance before later passes ignite.

W5 implementation notes (completed, 2026-10-04):
- The intermediate 20.5/20.9 s seam timings and 106.0 s whole-cell cycle came from duplicate skill updates in
  `switch pass.update(...)`: the running pass's host clock advanced twice per snapshot. Native motion and deposited
  interior area were correct, but each 0.15 s start/crater dwell lasted only seven observed ticks. The strengthened
  saved-data check confirms both dwell values survive the codec. A focused sequencer regression fails with the
  direct-switch implementation; storing the returned status in a local before switching fixes it. On both backends,
  the focused app now observes fourteen ticks per dwell and restores ordinary/recovery times to 20.6/21.1 s,
  preserving one restart, 3 mm overlap, a 5.4 mm peak and no gap. The corrected full W5 app gate passes on both
  backends: all 10 seams in 106.5 s, legs 4.9–5.1 mm, exact bead lengths and no clearance violations. The in-air
  failure, crater dropout, displaced workpiece and post-chain checks also pass with the W4 baseline behavior.
- The full app quality checks pass on both MuJoCo and the test backend. Woven 7 mm: 6.998 mm measured leg,
  one strike, 27.17 s. Three-pass 10 mm: 9.999 mm leg, three strikes, 65.55 s. The later-pass strike checks verify
  preceding bead deposition, a tip within 0.6 mm of that bead's surface, and available deposited clearance hulls.
  The complete entry/weld/exit is checked clear, sampled every five simulation ticks; both beads cover the seam
  without gaps and end with the arc off. MachineKit smoke passes, including the CAD-derived 10-seam / 680 mm
  mission in four runs. Shared app checks pass: arm 23.5 s; mobile 400 mm in 1 s on both backends, mission 61 s,
  obstacle round 65 s with one replan and 416 mm closest clearance.
  Focused suites pass: MotionKit weave 738 assertions; ProcessKit 55 welder, 44 planning, 16 rate, 106 weave,
  13 pass-sequencer, 12 deposited-work, 19 offset and 23 process assertions; ProjectKit 138; CAD recipes 33.
- `WeldPassPath` intersects adjacent offset lines at CAD chain corners rather than leaving gaps between shifted
  segments. A closed chain includes its final join. Zero offsets preserve the exact original path object;
  skew/parallel joins that cannot meet and offsets consuming more than 45% of an adjacent side are rejected.
  Focused ProcessKit offset checks pass 19 assertions, including a lifted/expanded closed square and bounds.
  Recipe serialization now shares the single-pass weaving policy too: `fillet(7).step` includes its weave and
  face lift, and all multi-pass settings use the same unit conversion. Recipe checks pass 33 assertions.
- Pass skills are now created lazily after cooling, allowing each runner to compile clearance against the metal
  deposited by preceding passes. `WeldBeads` supplies measured station prisms in world coordinates; `MissionPlayer`
  transforms them to the robot base link and includes them in `ArmClearance`. No RobotKit API changes are needed.
  ProcessKit passes 12 sequencer assertions (including the fill factory observing the completed root), 12 bead-work,
  55 welder, 44 planning, 16 rate, 106 weave and 23 process assertions. App compile for the clearance integration
  passes 1,759 sources. New woven and multi-pass example entrypoints retain the CAD-derived seam and configure
  its target leg to 7/10 mm. The new focused `welder-quality` checks are being compiled and have not yet run.

- Runtime grounded work now accepts changing geometry providers. `WeldBeadWork` derives a convex triangular
  prism per deposited station from its measured leg and CAD face directions; empty stations contribute no metal.
  Queries use the live workpiece frame, cache unchanged prisms, and reject stations outside the query's seam
  interval before expensive hull work. The same local vertices are exposed for subsequent clearance integration.
  `WeldBeads` registers its deposited geometry with the simulated welder, so distance, ray and touch see earlier
  metal without changing RobotKit. Focused ProcessKit bead-work checks pass 12 assertions, including a later
  strike with only the root bead as grounded work, touch before ignition, moved/rotated frames, and reset clearing
  cached metal. App compile passes 1,759 sources / 16,112 functions. Later-pass clearance integration and the
  7 mm / 10 mm app gates remain open; this does not yet prove multi-pass execution quality.
- Scene artifact version 16 replaces the weld's root process with a required non-empty `passes` list over its
  shared CAD `path`. Pass offsets are two nonnegative distances in metres along the CAD face normals; spatial
  weave frequency is cycles/metre. Validation checks pass bounds, process/feeder limits, patterns and edge dwells
  that leave time to traverse the weave. Version 15 is rejected; the decoder also rejects a root `process`.
  ProjectKit passes 138 assertions, including a two-pass weave round trip and malformed pass settings. Existing
  fixtures are generated in tests; example previews now generate version 16 directly (no stored scene binaries).
  `WeldingMission` attaches the recipe's complete pass schedule. `MissionPlayer` evaluates each pass against the
  live workpiece frame when that pass starts, applies its face offsets and weave, and sequences it with `WeldPasses`.
  Cooling begins after a pass has completed its burnback/retract and confirmed the arc off. Sequencer tests pass
  11 assertions, including cancellation during travel/cooling and failure preventing later strikes. ProcessKit
  remains 55 welder / 44 planning / 16 rate / 106 weave / 23 process assertions. App compile passes 1,758 sources,
  16,100 functions against R6. Runtime earlier-bead grounding and offset joins at chain corners remain open,
  together with the required woven 7 mm and three-pass 10 mm app quality gates. The focused `welder-seam`
  check exits 0 on both backends: ordinary 20.5 s, leg 5.0 mm (4.8–5.1), 180 mm, no restart; recovery 20.9 s,
  leg 5.0 mm (4.8–5.4), 180 mm, one restart and 3 mm overlap, no gap or stray. These are 0.1/0.2 s shorter
  than the preceding focused build (20.6/21.1); this difference was subsequently diagnosed and fixed as duplicate pass updates (above).
- CAD-side `WeldingRecipe.passes` now chooses one pass through 8 mm and root/fill/cap above it (through 12 mm).
  Individual pass area determines wire speed, voltage and seam speed using the CAD wire's diameter, efficiency
  and feeder limit. Single-pass recipes now reject legs above 8 mm. Weaving starts above a 6 mm pass leg:
  amplitude is 20% of that leg, period 10 mm, and edge dwell 0.05 s. A woven first pass's centre opens toward
  both faces by one quarter of its leg. Fill and cap target alternate sides of the preceding fillet's exposed
  face (65/35 then 35/65 of its cumulative leg along the CAD face normals); optional cooling occurs only
  between passes. These offsets are recipe targets; runtime grounding and clearance still need validation.
  Focused `machinekit/tests/welding-recipe` passes 28 assertions: conserved area/volume, 7 mm weave settings,
  10 mm three-pass split, prior-surface offsets, cooling, and feeder/single-pass limits. This recipe policy
  is not yet connected to the saved pass schema or mission generation. R6 main `af673c4f4` merged cleanly.
- MotionKit's `WeavePath` wraps pose primitives without changing seam-progress length, feed or event distances.
  The lateral material frame supplies its axis and exact first/second derivatives separately from torch roll;
  product-rule derivatives include the frame's motion. A quintic endpoint envelope starts and ends on the seam
  with zero added velocity and acceleration. Phase continues across base primitives.
- Sine uses cosine half-waves between edges. Triangle uses equal linear traverses; zigzag uses a 3:1 outward/return
  traverse ratio. Edge holds consume forward distance at nominal seam speed within the requested cycle, so the
  frequency remains unchanged; holds that consume the whole period are rejected. Frequency helpers accept
  cycles/mm or cycles/s. Pattern joins become explicit primitive boundaries with one-sided derivatives.
- Focused command: `haxeon/scripts/haxeon run --project motionkit/tests/weave/haxeon.json` with the standard
  OCCT/library environment. It passes 738 assertions, including numeric first/second derivatives in a rotating
  material frame, zero amplitude, endpoint taper, edge holds, frequency units and unchanged event coordinates.
  Saved recipe integration, multi-pass schema and grounding, restart hump, and app quality gates remain open.
- ProcessKit's weld parameters can carry the spatial profile. `WeldingPlanRunner.pathOf` applies it per CAD seam,
  deriving the lateral axis from the travel tangent and open-face bisector; torch roll never supplies that axis.
  Endpoint envelopes meet at the seam corners while phase continues across seams. The same generated path is used
  for native preflight and execution, so preflight IK and swept clearance see the woven trajectory. The existing
  bead projection deposits the woven tip into seam stations without a separate travel-length normalization.
  Focused ProcessKit weave tests pass 106 assertions: a quarter-turn of torch roll leaves the weave unchanged,
  authored seam length remains unchanged, and a simulated woven pass measures a 7 mm leg with no gaps or stray
  metal and the prescribed total volume. This is a bead-model test; MuJoCo quality remains to be proven. Existing
  ProcessKit welder/planning/rate/process checks still pass 52/44/11/23 assertions.

- Recovery engagement now has its own sequence in ProcessKit, with the original engagement retained by default
  for other processes. Welding re-strikes at `WeldArcModel.MIN_WIRE_SPEED`, waits for the arc, and omits the
  initial pooling dwell over existing metal. The first strike and crater/burnback exit stay unchanged. The
  ignition-watch operation index follows the shorter recovery sequence. ProcessKit passes 55 welder / 44 planning /
  11 rate / 106 weave / 23 process assertions. Overlap-dose compensation and the measured restart peak were subsequently validated below.

- Restart backoff is 2 mm, with maintenance wire feed over the already deposited prefix and normal validated
  rates restored exactly at the interruption distance. The low-rate re-strike and omitted pooling dwell avoid
  depositing another full section into the overlap. The original 10 mm / full-dose overlap is removed.
  `PROJECT_SOURCE_ONLY=welder-seam` proves the result on MuJoCo and the test backend: 21.1 s with one restart,
  3 mm measured overlap, a whole 180 mm bead with no gap or stray metal, mean leg 5.0 mm and peak 5.4 mm
  (previously 9.4 mm). The undisturbed seam remains 20.6 s, leg 5.0 mm (4.8–5.1). The app asserts the
  restart peak is at most target + 1 mm. Focused ProcessKit rate tests pass 16 assertions; other process checks
  pass 55 welder / 44 planning / 106 weave / 23 process assertions. The app compiles 1,748 source files.

**W6. Real welder interface.** Map the channels to:
- the retrofit I/O board: an optoMOS relay for the trigger, an isolated 0–10 V
  output for wire speed and voltage, a Hall-effect current sensor, an isolated
  voltage input, and a separate touch-sense circuit;
- a robot power source protocol: Modbus TCP registers, or I/O.

Then carry them over RKD6 like other process channels, with the arc channel
going safe (off) on every stop.

W6 implementation notes (started after W5 main sync, 2026-10-04):
- R0 is complete on main: generation 6 / wire revision 12 naming is frozen and `RuntimeEndpoint` exists.
  `main` was merged before starting W6. `WeldContract` owns channel declarations and the six-value feedback layout;
  `WeldChannels` and `WeldSensor` refer to it. Arc is digital, wire is m/min, voltage is V, and an optional job is
  a nonnegative integer carried as an analog value. Arc and wire go safe on commanded stops; fault and emergency
  stop safe every output. Voltage and job may retain their setpoints on an ordinary stop.
- Device-side welding must use the scheduler's existing safe channel policy, including its link watchdog.
  A Modbus connection failure cannot rely on a final network write to stop the physical source: the register map
  must express a device watchdog/lease, and the fake server must enforce its expiry independently of the host.
  The same process mission must run through each backend rather than replacing it with backend-specific scripts.
- The virtual device now observes scheduler-controlled arc/wire/voltage channels through its configured welding
  profile. Deployment refuses unsafe arc/wire stop policies. Its ignition/current/power model latches no-arc and
  arc-loss faults; a generic `ProcessFault` stop safes every channel on the device before another host frame is needed.
  Three model tests and three framed-device tests pass; the latter cover ignition, abort, commanded stop, emergency
  stop, link loss, no-arc and arc loss. Seven focused existing protocol stop tests also pass. Feedback still needs
  its wire transport and host sensor mapping; the unchanged shared mission has not yet been validated on this backend.
- RKD6 keeps its frozen generation name and advances to wire revision 13 for generic `SENSOR6` feedback packets.
  The protocol's pre-hardware policy requires a revision bump for wire changes; revision 12 is rejected, with no
  compatibility path. Sensor packets carry a compiled slot, device ticks, sequence and finite numeric values;
  welding semantics remain in ProcessKit. The schema lock/codecs/current shared vectors are regenerated. Nine Rust
  wire tests and the C++ codec/vector test pass. Device publishing and runtime snapshot mapping remain to be connected.
- The virtual welder publishes all six feedback values in `SENSOR6`, with a configured sensor slot and device
  acquisition clock. Its C interface exposes profile configuration before deployment and grounding changes for
  host tests. The RKD6 endpoint packs samples into the existing runtime sensor pool, retaining source/reception
  times and sequences. Wrong-session, stale-sequence, backwards-time and aggregate pool-overflow packets are ignored.
  Four framed virtual-device tests and the native endpoint suite pass, including sensor metadata, values and stale
  packet handling. The host's native endpoint test uses a mock transport; the unchanged welding mission still needs
  to exercise the complete virtual-device link and the Modbus backend.
- The retrofit policy lives beside board configuration and maps an optoMOS trigger, two external 0–10 V outputs,
  calibrated Hall current/isolated voltage inputs and independent digital touch. Current derives arc established;
  no-arc and sustained high-current/low-voltage faults latch and safe the outputs. A persistent short uses the
  existing wire-stuck code 3; brief MIG shorts reset the persistence timer instead of faulting.
  Nucleo's optional PA4/PA5 DAC, PB0/PB1 ADC, PB2 trigger and PB10 touch profile passes `cargo check --offline`,
  with HAL traits checking analog pin capabilities. Three virtual I/O tests pass for scaling, feedback, no-arc,
  short persistence, reset and invalid command safety. The bench firmware remains a virtual-output UART image;
  the pin profile is defined and checked, but no physical welding firmware is enabled or flashed.
- Modbus foundations are in ProcessKit: MBAP framing with big-endian words, holding-register reads/writes,
  engineering-unit scale/offset data, swappable output/feedback addresses and bit masks, and an optional job register.
  The current register encoding is unsigned 16-bit; unsupported wider or vendor-specific encodings are not silently
  reinterpreted. Every map requires an independently enforced device lease of 20–1000 ms; a source without that
  watchdog needs an external safety interlock before this adapter can satisfy link-loss safety.
  The focused Modbus suite passes 37 assertions, including malformed frames, setpoint overflow and invalid maps.
  This proves the codec/map foundations only; TCP transactions, the in-process fake and mission integration remain open.
- The nonblocking Modbus TCP client now executes bounded, acknowledged transactions through NativeKit. Its stream
  decoder handles fragmented and coalesced MBAP records; responses must match transaction, unit and function.
  An in-process fake uses a real loopback socket and an independent device watchdog clock. The focused suite passes
  74 framing/map/stream assertions plus real TCP setpoint writes, feedback-bit reads, connection-fault detection and
  arc/wire shutdown after the connection drops without a final write.
  NativeKit's transport worker can wait 100 ms before noticing outgoing data; a 50 ms request timeout was therefore
  invalid. The client uses a bounded 500 ms default; the TCP test uses a 1 s device lease. The adapter must budget
  lease renewal against transport delay instead of assuming a write is applied synchronously. Focused TCP tests need
  the existing app native directory on `LD_LIBRARY_PATH` for `libnativekit.so`.
  The `WelderOutputs`/`WelderFeedback` adapter and unchanged process mission are still pending.
- `ModbusWelder` now implements both existing welder interfaces. Its owner polls acknowledged atomic setpoint/lease
  writes and a coherent feedback block; initialization begins with the source off. Stop discards unsent ignition
  commands and sends arc/wire off; a lost connection faults feedback while the independent device lease expires.
  Output registers, including the lease and optional job, must be contiguous, preventing writes into unmapped
  vendor registers. Feedback must fit one 125-register read. The lease must exceed two bounded transactions plus
  100 ms owner allowance. The adapter test uses a 300 ms transaction bound and 1 s lease, rather than the generic
  client's 500 ms default (which is intentionally refused with that lease).
  Supply faults and connection loss use the existing supply-drop/arc-loss code (2), preserving RobotKit's frozen
  external sensor contract (codes 0–3). Adapter diagnostics expose the detailed cause separately; introducing codes
  4 and 5 in the draft was incompatible with that contract and was removed. Feedback is validated before publication.
  The fake safes and clears stored ignition on a supply
  fault, so clearing it cannot reignite. Tests pass: 78 focused assertions plus TCP safe initialization, ignition,
  scaled feedback, stop/restart, supply fault, explicit restart, link-loss fault and watchdog shutdown; affected
  tool regression suites pass 27 tool / 33 process / 152 weld / 33 clearance assertions.
  These are adapter-level checks; running the unchanged welding process/mission on all three backends remains open.
- `WelderChannelBinding` forwards executed channel transitions through the device-neutral outputs/feedback boundary
  and polls the supply continuously. It does not consume the runtime's event queue or rewrite the authored program;
  the runner retains its existing channel commands and tool-sensor waits. Repeated off samples do not discard queued
  feedback reads. Stop inhibits ignition until the robot's arc channel has actually reported off, preventing stale
  on samples from reigniting a stopped source. Invalid channels/feedback and reported faults safe the outputs.
  Focused binding checks pass for unchanged values, channel transitions, continuous polling, stop inhibition and
  invalid setpoints; the 78 Modbus assertions and TCP adapter tests remain green. Runtime sensor publication and
  complete CAD-mission execution through this binding are still to be connected and verified.

- `WelderRuntimeBinding` now publishes supply feedback through RobotKit's existing external-sensor API, preserving
  authored sensor identity, mount, source timestamp and explicit clock domain. Transport polling time is supplied
  separately from the observation timestamp. Publication errors safe arc/wire, and sequence advances only after
  successful publication. Focused checks use a real in-memory runtime: six-value frame/mount/clock preservation,
  fault code 2 publication, safe outputs and backward source-clock rejection. The 78 Modbus assertions and real TCP
  adapter/watchdog tests pass; the fault-contract change also passes 27 tool / 33 process / 152 weld / 33 clearance.
- **W6 stopped at the ownership boundary; not complete or landed.** The full unchanged CAD mission has not run on
  the RKD6 virtual welder or Modbus fake. The virtual board's C ABI can configure the welder only before its session,
  but `robotkit/runtime/src/virtual_device_endpoint.{hpp,cpp}` creates and privately owns that board without an
  initialization or tool access hook. `VirtualDeviceOptions`, `Simulation`, `robotkit_simkit.h` and the simulation
  native adapter likewise expose no configuration, grounding-input or tool-feedback access. RKD6 numeric packets
  reach native snapshots, but the Haxe snapshot layout accepts only encoder/IMU/lidar native sensors; `tool_weld`
  remains an authored external sensor. A clean completion needs generic endpoint/device initialization and numeric
  sensor access, with welding configuration, CAD grounding and external-sensor publication owned by ProcessKit.
  These generic host endpoint/simulation files belong to the restructuring session under the handoff. The goal
  explicitly requires stopping when finishing needs those files beyond post-merge compile fixes; no such edits
  were made. Resume after that session supplies the hooks or the ownership restriction is explicitly revised.
  The Modbus CAD-mission harness, cross-backend stop/abort/fault/link-loss mission checks, final W6 app gate and main
  sync remain open. Local main remains `757127cf01605ef843a7e24c22b74a0ee13e6c61` (W5).

### W6 resumed: host hooks implemented; latest-main regression blocks the mission gate

The user explicitly authorized the endpoint/simulation extensions after the ownership stop above.
Local main `38601ac61c60461548899007ef9db33d476a801a` was merged as `2c5bab9ac`.
The conflict retained both welding safety validation and main's active-actuator subset support.

- Generic virtual peripheral profile parameters, numeric inputs and cached sensor observations now cross the host
  interface. ProcessKit owns the welding profile, derives grounding from the CAD including deposited metal, and
  publishes on the authored external sensor. Explicit external-slot routing prevents welding values from being
  mislabeled as native encoder data. Device epochs retain increasing host publication sequences and have distinct
  clock identifiers. STOP6 is processed before a paused simulation clock freezes, with unsent UART work discarded.
- The arm is a servo model, so physical stepper `DeviceBinding` correctly refuses it. A virtual-only servo adapter
  derives its observation grid from the authored encoder, gearing and speed, capped at one count per board tick.
  It exposes the resulting joint error budget (0.000100692 rad in this cell), without editing the CAD motor or
  claiming a physical servo deployment is implemented. The native braking bound uses the existing mission policy,
  2 rad/s², tightened by any lower authored bound; it is not an invented motor acceleration.
- The Rust welding profiles now use sensor indices generated from the canonical ProcessKit channel contract.
  The app can select virtual-device or supplied Modbus feedback; that same feedback drives bead deposition.
  A focused harness loads the unchanged single-seam CAD mission. Modbus uses a real loopback socket, an independent
  server watchdog clock, CAD grounding, and wall-clock pacing instead of advancing motion ahead of TCP responses.
- **Passed:** native runtime build; simulation ABI audit on Linux, Windows, Intel macOS and ARM macOS; host RKD6
  ignition, numeric sensor metadata, stop/abort/estop, link loss, arc loss, reset, shutdown with a frozen clock, and
  exclusion of external slots from native encoders; 13 Rust virtual/profile tests; generated contract check;
  78 Modbus assertions plus binding/TCP/watchdog checks; 27 tool / 33 process / 152 weld / 33 clearance assertions;
  focused app compilation. These component results do not prove the unchanged mission gate.
- **Failed, outside W6:** the CAD mission reaches plan submission and fails at 0.03 s with
  `plan chunk [0,4] of 26 ... Array index out of bounds`. On latest main,
  `robotkit/core/haxe/robotkit/execution/ExecutionPlanSubmission.hx:107–108` defaults and validates
  `controlAcceleration` against the segment count. `RobotRuntime.hx:240` reads one entry per joint. The focused
  reproduction proves a six-joint, four-segment plan has only four control accelerations and fails with
  `Control-acceleration defaults are sized by segments instead of joints`.
  Reproduce with the brief's kit environment and
  `haxeon/scripts/haxeon run --project processkit/tests/device/haxeon.json`.
  Both default sizing and length validation need to use the joint count. These unrelated main files were not edited.
- **Stop required:** the user requires reporting an unrelated main regression instead of repairing it in this task.
  W6 remains incomplete. The Modbus CAD mission, cross-backend mission shutdown checks, final welder gate and
  arm/mobile checks (shared app code changed) remain open. No MachineKit code changed in this resumed work.
  Main was not advanced to W6 and remains `38601ac61c60461548899007ef9db33d476a801a`.
  The merged main records haxeon `8521d494e43233d6c0433e831e1219aa24caa88d`, which is absent from this submodule's
  local objects. The clean submodule working copy remains at `4f1ac30fec7ec8a28e9ae84e1979027387753292`; checks used
  that existing compiler. No pins were manually changed or fetched.

Resumed W6 commits (branch only):

- `2c5bab9ac499cd9ee531fdc20ef3846ff8267ea8` Merge main into mobile-welder for W6 host integration
- `3dac0dc69d4d8508e312d0221749bc3972b4cc5f` RobotKit: expose virtual peripheral profiles and numeric device observations
- `a084f1a1406f6a0a558e225535631d7db59a7c99` ProcessKit: generate the device welding sensor layout from its channel contract
- `71eb2bff05174acec86c47bb466a52cd8b7e9594` ProcessKit: reproduce the main plan control array length regression
- `1f898b2ea4c323379c62aecd73a20c67828c121f` RobotKit: route external device sensors outside the native sensor layout
- `1809a3b098519856a80143e4efa26d3c8c15bab5` RobotKit: derive virtual servo command grids from model drive data
- `68034eb23837750bbc6ff4d2367f267209455477` ProcessKit: connect CAD welding observations to external supply backends
- `bde03e2e139eef80bc37050397050a93b3e4f620` App: exercise an unchanged CAD weld mission through RKD6 and Modbus

### Phase 1 commit ledger at the W6 ownership stop

First-parent task history, including main sync merges; W6 is unfinished.

W4:

- `3106781d543edc7173253da37a03d1b57118bd82` Robot welder handoff: synced with main 712019dbc, RobotKit restructuring R0/R3 constraints
- `c67506284f7d7851b59cdd967f7b1ee94b9928cd` Robot welder handoff: coordination with the RobotKit restructuring (sync points A and B)
- `05f1b67f44e0871f247032e7063382ad6f269dfb` ProjectKit: reject weld steps without a path
- `e8eb7795a90490a3eef968ec4a1ce2862c497555` Merge branch 'main' into mobile-welder
- `6d8c9a1199d018bb13d53ab008b559de4a528109` RobotKit: check swept arm clearance between convex hulls
- `0f8c82e9530aa4ebfc382d16dc82e526e8010bbc` ProcessKit: plan weld paths with reach and clearance
- `cceb415023a424b45efd0b327c28805962a49d56` ProcessKit: reverse weld runs when their entry is unreachable
- `0badaeae682900bf090b32d2b2c24c87ade6e567` Merge branch 'main' into mobile-welder
- `4f7e9ccafa81c2b83410c9e3a6ea6a376dcfb653` ProcessKit: follow the extracted weld plan namespace
- `0c248c7e153b12460bf31c2d74631145db424054` ProcessKit: schedule wire feed from validated seam progress
- `362e0a64fca10cf9a5d00c72e77e48da26cf205e` MachineKit: derive torch stickout and split its collision bodies
- `8d9a034e5e37920025663ea14163ef73c6f01ca1` MachineKit: generate the welder mission from every declared seam
- `e7879f9057b4fea563cb278b2acae522814e5607` App: check complete welder missions against swept clearance
- `44604b5a4fba60ee841d4617d51175d7d452d377` Merge branch 'main' into mobile-welder
- `b6f588d3e920d3cde79146b7b052ec4169e1cbe5` MachineKit: record W4 clearance and process timing decisions
- `1245b8c9923212228e33fd1bded5cabfe520d148` MachineKit: complete the whole-weldment W4 validation gate

W5:

- `ced4a77e3322bfb851e29ce2ee9ecf0442cf568a` MotionKit: weave pose paths in authored seam progress
- `3dbedfd1b6a50fa82c718dd70ee176cf2ec7ceca` MotionKit: ignore generated weave test builds
- `30ef670b637ce7e9330775db09e55ce7456d36d4` ProcessKit: validate and execute woven weld paths in the CAD frame
- `c6e936c4300c97555171fd59c627741ac5e646c3` ProcessKit: separate re-strike engagement from the initial dwell
- `761543995199f60db9b9a73856ab64d58fb593be` ProcessKit: limit restart deposition over the existing bead
- `e3ee4ea8fdec697e4cec20fa50ba970b96496a83` Merge branch 'main' into mobile-welder
- `aa31a409c01c0ef626f41846c76ce7e0e2fd73b6` MachineKit: derive fillet pass recipes from deposited area
- `401da9d020795c99fef491b052ad5c79dc2bf4a2` ProcessKit: sequence weld passes with interpass cooling
- `2b1e24b489f1f12a89e3c553e7b9f79aaa407c91` ProjectKit: represent welds as passes over a shared CAD path
- `0ec3f5b545bb2345e663eb17b63296edff2fd1be` ProcessKit: ground welds on deposited bead geometry
- `c8f451a340ee54153669cf59fdf6ec7a8e8b49a4` ProcessKit: plan later passes against deposited weld metal
- `9832c564ba0b41a78d765591c49d807193b8adad` ProcessKit: join offset pass paths at CAD chain corners
- `a5a0b4e94689602089660d42f904095298318c19` MachineKit: generate and verify woven and multi-pass weld examples
- `23c5b95933e09a22970881eea121aae277a7e059` ProjectKit: verify weld pass dwell round trips
- `e3267f30916242474f6061f4c75af009ff70d201` ProcessKit: update each running weld pass once per snapshot
- `4d1734becd08035a1db5c33b3ad0e887cf70e6a7` ProcessKit: record corrected W5 app validation
- `757127cf01605ef843a7e24c22b74a0ee13e6c61` ProcessKit: complete W5 validation record

W6 (partial, branch only):

- `b8d552a6f9e37f25132c5af2792dadad6ae07933` ProcessKit: centralize the welder channel contract
- `9bda6e6b6fd2023a6abce247e72749594949b9c3` RobotKit: model a welder in the virtual device
- `b67f12ba60f2cd9a9bb037017ea97249a2df890a` RobotKit: safe the virtual welder through device stop policies
- `71f5fb745d0f879e9e5004077ce7b77779722d79` RobotKit: carry numeric sensor samples in RKD6 revision 13
- `113affcec3bff12bbf1eb403658f01f4039160b5` RobotKit: publish virtual welder feedback into runtime sensor snapshots
- `e17f2cd9415069d9a311f81b8ca1396f7f230465` RobotKit: define and check the retrofit welding I/O profile
- `994cb63c5e3d64571f2cdcb0bc344688fb210716` ProcessKit: ignore generated Modbus test builds
- `2e34f4f8e3e6264b65cd7c47a5779f23eacd3b91` ProcessKit: define Modbus welding register maps and TCP frames
- `e5bc4e5654bd393a97a1b5e7ba60289103da0ac7` ProcessKit: execute Modbus TCP transactions against a watchdog fake
- `a1a81fdb0ac14b798ffab719fc7f76c2289d4481` ProcessKit: adapt Modbus supplies to the welder process interfaces
- `51da75921e7ee75fa8301973ff574c2e7c10dbf3` ProcessKit: bind executed welding channels to device-neutral supplies
- `abcc33df689f435072e3a262e04e20dc4f2726d2` ProcessKit: preserve the external welding sensor fault contract
- `d3178440cbfbadcb24d9fa7a827497a379ba6b51` ProcessKit: publish supply feedback through authored welding sensors

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
| W3 | done (+ hardening: paths, live workpiece frame, exit crash, safety tests) | `weld` mission step; `WeldSeam` skill and `WeldBead`; process engagement and `WeldingPlanRunner`; weld metal part and recipe; bead as runtime geometry; welds on MuJoCo and the test backend, restart with overlap |
| W4 | Done (2026-10-04) | Whole CAD weldment: 10 seams / 680 mm in 4 runs; swept clearance and derived corner turns; 106.5 s on both backends, legs 4.9–5.1 mm; affected gates pass. Timing changes recorded above. |
| W5 | Done (2026-10-04) | Seam-progress weave; CAD-derived pass recipes; deposited bead grounding and clearance; scene schema 16 rejects older versions; smaller restart hump. Woven 7 mm measures 6.998 mm, three-pass 10 mm measures 9.999 mm on both backends; all affected gates pass. |
| W6 | Done (2026-10-04) | RKD6 numeric feedback and virtual welder; checked retrofit profile; Modbus map/adapter with independent ProcessKit owner; unchanged seam and all six shutdown cases pass. Full welder, arm, mobile and MachineKit smoke gates pass; phase one lands on local main. |
| P1 | Done (2026-10-05) | Mobile welding carrier, insulated storage and converter service graph; CAD fit/mass checks and both-backend carrier movement pass. |
| P2 | Done (2026-10-05) | Minimum verified candidate cover: 2 stations / 10 seams; complete goTo/weld/stow mission passes both backends in 270.3 s; affected milestone gates pass. |
| P3 | Open | Executed touch searches and correction under injected parking error. |
| P4 | Open | Measured load integration, voltage sag/cutoff and pre-seam docking/charging. |
| P5 | Open; required | Rendered-depth laser profiler and executed live seam correction. |


### W6 latest-main retry: nonzero device start anchor blocks the mission

User authorized the plan-control regression repair and continuing W6. Local main had advanced to
`1484b924d9789334503781fce4ec1515021fdc6c` (machine tending MT5); it already contains the joint-count
control-acceleration correction. Merge `95f6109ca` preserves both pneumatic/velocity simulation tails
and appends virtual peripheral fields after them; regenerate the four-platform simulation HXI.
Commit `aab794fa8` extends the focused control-array reproduction: six joints/four segments produce
six defaults, accept six explicit accelerations and reject four. The test passes with main's recorded
compiler `8521d494e43233d6c0433e831e1219aa24caa88d`, now checked out locally without changing the gitlink.
Fetching that object succeeded; automatic recursion reported unavailable `vendor/hashlink`, but the
existing local tools work and the focused app compiles (1810 files, compiler 21.953 s).
The merged RobotKit native runtime builds successfully, using the prebuilt OCCT install.

The unchanged RKD6 CAD seam mission still fails at 0.03 s before motion/ignition:
`runtime.submitPlan failed with RobotKit status -2` (`RK_ERROR_INVALID_STATE`). Temporary native
diagnostics identify the start-state guard in `RobotRuntime::submit_plan`: joint 1 has requested and
polynomial start position -0.25 rad, but held commanded anchor 0 rad; all start derivatives are zero,
with position/velocity/acceleration tolerances 0.005/0.02/0.2. The virtual endpoint derives this initial
joint position from the CAD transmission offset. Generic runtime construction/reset initializes its
commanded anchor to zero and does not establish it from the initial device observation. This needs
an explicit generic initialization policy, not a welding tolerance increase or a fabricated CAD offset.
Diagnostics are removed from source and the native runtime rebuilt cleanly.

Per the user's stop rule for unrelated main errors, W6 pauses here. Modbus mission, mission shutdown
cases and final welder/arm/mobile gates remain pending; no W6 cycle time or bead result is claimed.
Main is not fast-forwarded to incomplete W6. The next authorized repair must establish the initial
held device state once, retain commanded anchors after execution, and verify nonzero deployment
origins and reset before retrying the same mission.


### W6 initial-anchor repair: passed; short Cartesian segments block qualification

The user authorized the generic held-origin repair. The first **validated** observation now seeds
untouched joints' commanded anchors once, after construction or reset. A command or plan already
owns its anchor, and later observation drift cannot move it. The focused native runtime suite passes,
including nonzero -0.25 rad initialization, subsequent drift, reset to -0.4 rad and existing following-error
and held-command checks. No changes to `RobotRuntime.hx` or `RobotArm.hx`.

RKD6 then exposed missing startup integration: its plan clock needs two synchronization observations,
so the app now waits on an explicit virtual-device readiness query before feeding the first mission.
Readiness clears on rebuild/reset; once started the mission continues receiving failures. This is a
native endpoint fact, not a fixed delay. Four-platform simulation ABI audit, native rebuild, focused
host welding shutdown/readiness test and app compile pass. The host test still verifies stop, abort,
emergency stop, link loss, grounding fault and synchronous stop, with arc/wire off, and reset.

The virtual servo link defaults to configurable 3 Mbaud: the first six-axis polynomial chunk requires
1,423,437 baud, above the stepper-oriented 921,600 default. A separate 2% acceleration planning reserve
covers the clock fitter's up-to-0.5% rate adjustment and float polynomial conversion. Physical/native
limits remain 2 rad/s²; ordinary simulation defaults remain unchanged. Without the reserve, conversion
correctly rejects joint 0 acceleration 2.0005428791046143 against limit 2 and tolerance 0.000002.
Device conversion errors now retain these values for diagnosis. The focused device-converter suite passes.

With these changes the unchanged CAD seam mission executes the initial motion. At simulation 2.47 s,
it fails submitting Cartesian plan chunk [0,199] of 672 (events=0, jerkUnchecked=true), with status -10:
`Rkd6Endpoint: plan exceeds serial qualification: minimum_baud=18446744073709551615 minimum_queue_depth=4`.
`minimum_baud` returns UINT64_MAX when the shortest converted segment is at most the configured
2 ms per-segment processing allowance. Increasing baud cannot qualify this plan. Resolving that requires
a generic trajectory-generation or buffered-queue qualification design; it is outside the authorized
held-anchor repair. Stop here under the user's scope rule rather than bypass the native qualification.
No full bead/cycle result is claimed. Modbus mission, mission shutdown coverage and final welder/arm/mobile
gates remain pending. W6 is incomplete and not fast-forwarded to main, still
`1484b924d9789334503781fce4ec1515021fdc6c`. The worktree uses main's recorded haxeon revision.

Commits for this retry:

- `5b99eb1bc7232a514d7c9162ff2e13a3048de4c8` RobotKit: establish held origins from initial device observations
- `eae3c12bf0de5afcf62a1a8c591df2f04b358355` RobotKit: expose virtual device plan readiness
- `70e6ab5dd7e704852a2c2110727ffcf0533c9b71` App: wait for virtual device clock qualification before welding
- `2daa96c825b93c60cd4340582e236236b8c39fe9` RobotKit: report rejected device conversion limits
- `fd82c53d704cba654963343e74e5dd00e3ad8cfc` RobotKit: configure virtual servo link and planning headroom
- `28233766033c792226f513b8db14a646ae0db416` App: apply virtual servo acceleration headroom to weld planning

### W6 buffered-link repair: unchanged RKD6 CAD mission passes

The user authorized buffer-aware serial qualification. Main's MachineKit restructure is merged as
`083aa07211212a4d0eab21089998f703019fdf4e`, from main
`a5d1e7e4008a8970fb7f627e124881aa4d4e7659`; the focused app compiles with its new hierarchy/facets.

Choose qualification in the native device-link layer, preserving CAD polynomials and process events.
Commit `5c20666415cf1d306899c9ace13b42bc8a5e2c52` qualifies actual converted row durations against
finite FIFO refill deadlines. A prefetched initial queue permits bounded short bursts; retained
refill debt prevents later chunks from borrowing a new full queue. Revisions replace only their
future rows, and retired rows fold into bounded scheduler state. Admission accounts for exact
serial frame sizes, owner-tick whole-frame batching, queue-release guards and clock uncertainty.
Startup reserves the prefill and actual queue/event/commit serialization before playback.

The configured 2 ms processing allowance is documented as one-way target-streaming latency,
so it stays a pipelined latency guard rather than being charged as serialized CPU service for
every row. Baud determines serial service time. Sustained overload and unbufferable bursts still
fail admission before any queue or segment write; CAD trajectories are not coalesced or stretched.

Focused endpoint tests pass, including equal-average streams with different burst order, finite
prefill, continuation debt, sustained overload, pipelined latency, owner-cadence throughput and
existing replacement/clock/backlog checks. Native runtime rebuild and host welding shutdown/
readiness checks pass. The unchanged RKD6 CAD seam mission passes at **20.36 s**, mean leg
**4.99734 mm**, bead **180 mm**, no gap, and arc/wire off at completion. This is the first complete
W6 RKD6 mission result. Modbus mission, mission shutdown cases and final app gates remain pending;
W6 has not been fast-forwarded to main.

The first full Modbus CAD mission exposes a separate owner-scheduling issue. The focused app compiles,
but its fake server eventually reports `client=Modbus response timed out, requests=3` before a complete
bead. Preparing CAD before opening TCP removes the long compiler gap, and initializing at supply
attachment removes the early rebuild gap; neither makes the synchronous setup/planning thread a
bounded transport owner. The harness now preserves the client fault in its error instead of masking it
with the subsequent server receive failure. No Modbus mission result is claimed.

Next choose an explicit ProcessKit transport owner that keeps Modbus I/O and watchdog refresh polling
independent of synchronous CAD/mission planning, with synchronized setpoint and feedback exchange.
Do not increase deadlines or suppress timeout faults to make the mission pass. Prove shutdown and
link-loss behavior through that owner before running the remaining W6 gates and landing W6.

### W6 independent device owner: Modbus mission passes

The user authorized the independent-owner architecture. `PolledWelderDevice` separates a nonblocking
supply from its host scheduler; `WelderDeviceOwner` owns all adapter calls on one thread. Planning
publishes only desired arc/wire/voltage to a mutex-protected latest-state mailbox, and reads copied
feedback snapshots. There is no growing command queue. Stops clear ignition and wire under the same
lock used to apply setpoints; fault inhibition is latched, healthy feedback alone cannot re-ignite,
and explicit reset clears the latch without replaying old commands. Close services an off request
until acknowledgement or a bounded 2 s attempt, then closes the connection; hardware retains its
independent watchdog. The Modbus adapter and vendor register map remain transport-specific, while
the owner is ProcessKit device policy. The fake hardware has its own clock/thread too.

The focused Modbus gate passes: 78 map/frame assertions, channel-binding checks, real TCP adapter/
watchdog cases, and new owner cases covering a 1.5 s caller stall (longer than response timeout and
lease), isolated snapshots, stop priority, explicit reset, supply faults and dropped connections.
The unchanged Modbus CAD seam now passes at **21.10 s**, mean leg **4.99734 mm**, bead **180 mm**, no
gap, with the fake's arc/wire off. TCP round trips extend ignition-feedback waiting relative to RKD6;
CAD geometry, trajectory and weld deposition values are unchanged.

Mission shutdown coverage exposed a device watchdog gap at rest: an ignition dwell could finish
before the lost-link timeout, disabling the motion-only watchdog while the arc remained on. Device
event policy now supplies a generic `requires_link` fact when a safe-on-stop channel differs from
its safe value. The scheduled core checks the lease during such process activity as well as motion;
completed safe programs retain their existing idle behavior. Focused scheduled-core tests (16),
event tests (3), Rust virtual-device tests (14), native rebuild, host shutdown/numeric-feedback tests
and Nucleo `cargo check --offline` pass. The host link-loss test now ends its dwell before cutting
the link and requires numeric arc-off feedback, rather than checking only physical channel outputs.

Commits for this repair:

- `f4d5abcdf8b925cf4f060ebb559dda1c7d4e9ca9` ProcessKit: move the shared Modbus fake into its test package directory
- `f5c2d1bd207df6ff27611f67dfe32e03f73b5d25` ProcessKit: resolve the shared Modbus fixture by its explicit package
- `2a9977f51556ed4596c95d46ebca9f602cbe1244` ProcessKit: service welding devices independently of planning
- `c7eeb86ef31da49666c69224555fcb2efb2160e3` App: run the Modbus weld mission through its independent device owner
- `d7a5e443e85828050df83443df3a35842b1e9854` RobotKit: enforce link leases for active process outputs at rest

All six active CAD mission shutdown cases pass on RKD6 and Modbus: stop, abort, emergency stop,
simulation pause, supply fault and link loss leave arc and wire off. RKD6 link loss is rechecked after
the lease repair, with a generic runtime fault even if the welding sensor itself reports a clean off.
The Modbus repeat is **20.92 s**, with the same leg and bead; actual TCP scheduling introduces small
ignition-feedback timing variation. `52316f89b6d75952b25000e2c9f03344071af371` records those mission tests.

`ed6409f803b9b10dd94d9a12fb5a4e72344691ce` refines device event policy to track applied output values,
including safe-on-hold values that are not restored on resume. Four event-policy tests and five focused
Rust welding cases pass. The final app compile passes (1835 files, compiler 26.974 s).
Final app gates are still being checked; W6 is not yet landed.


### W6 final gate record: owner repair complete; mobile baseline blocks landing

The independent-owner repair and mission shutdown coverage are committed and validated. The final
`PROJECT_SOURCE_ONLY=welder` gate passes once, including both complete weldments and all standard
seam/recovery/quality cases. Its main results match W4/W5:

- Whole weldment, both backends: **106.5 s**, 10 seams / 680 mm, legs **4.9–5.1 mm**, no clearance violation.
- Single seam, both: **20.6 s**, mean leg **5 mm** (4.8–5.1), bead **180 mm**, no gap.
- Arc loss, both: **21.1 s**, one restart, 3 mm overlap, peak **5.4 mm**, no gap.
- Air seam: expected failure after **7.4 s**, `the arc could not be held in 3 restarts`.
- Crater dropout: **20.6 s**, no restart; displaced workpiece: **12.3 mm**, tip within **0.1 mm**, leg **5 mm**.
- Post chain: **29.8 s**, legs **4.9/5/5/5.1 mm**, tip within **0.1 mm**.
- Weave, both: **6.998213 mm**, **27.17 s**, one strike.
- Three passes: **9.999356 / 9.999193 mm**, **65.55 s**, three strikes, both backends.
- `PROJECT_SOURCE_ONLY=arm`: pass, pick/place cycle **23.5 s**.

The final native rebuild and focused host welding shutdown test pass. The Nucleo profile passes
`cargo check --offline` after the applied-output policy refinement. The generated welding contract
matches its checked-in Rust output. Commit `15fa95fdba7c8313d3484da83d7708d6c3a511d7` adds the missing CMake dependency on
`device_events.rs`, so changing event policy rebuilds the embedded virtual-device library.

**Failed gate:** `PROJECT_SOURCE_ONLY=mobile` exits 1 at
`app/tests/src/app/ProjectSourceTests.hx:461`:

```
deterministic: the wheels spin at v / r, the right one negative (0, 0)
```

The preceding chassis displacement check passed. This assertion reads literal joint slots 0 and 1;
its actual CAD-resolved wheel indices and the root cause have not been isolated. The W6 application
paths are disabled for this test (`virtualWelder=false`, no welding factory); the new process lease
runs only on device endpoints, and the native held-origin repair leaves zero-origin untouched joints
at zero. No mobile code or assertion was repaired under the welder task. Per the user's instruction
to stop on unrelated breakage, stop here and report the exact mobile error rather than landing W6
with a failed required gate. MachineKit smoke and final sync remain pending.

Local main remains **a5d1e7e4008a8970fb7f627e124881aa4d4e7659**. W6 is branch-only. Hardware bench
calibration, vendor-specific maps and flashing the checked retrofit profile remain hardware follow-up;
phase 1's device tests use simulated hardware. The next step is to inspect the mobile drive's authored
joint mapping and observations, repair the cause in its owning layer, and rerun only that failed gate
before the pending smoke and guarded main sync. Passed welder/arm suites need no repeat unless that
repair affects them.

### W6 complete branch commit ledger before the mobile stop

First-parent commits since the W5 landing, in order (including local-main merges):

- `b8d552a6f9e37f25132c5af2792dadad6ae07933` ProcessKit: centralize the welder channel contract
- `9bda6e6b6fd2023a6abce247e72749594949b9c3` RobotKit: model a welder in the virtual device
- `b67f12ba60f2cd9a9bb037017ea97249a2df890a` RobotKit: safe the virtual welder through device stop policies
- `71f5fb745d0f879e9e5004077ce7b77779722d79` RobotKit: carry numeric sensor samples in RKD6 revision 13
- `113affcec3bff12bbf1eb403658f01f4039160b5` RobotKit: publish virtual welder feedback into runtime sensor snapshots
- `e17f2cd9415069d9a311f81b8ca1396f7f230465` RobotKit: define and check the retrofit welding I/O profile
- `994cb63c5e3d64571f2cdcb0bc344688fb210716` ProcessKit: ignore generated Modbus test builds
- `2e34f4f8e3e6264b65cd7c47a5779f23eacd3b91` ProcessKit: define Modbus welding register maps and TCP frames
- `e5bc4e5654bd393a97a1b5e7ba60289103da0ac7` ProcessKit: execute Modbus TCP transactions against a watchdog fake
- `a1a81fdb0ac14b798ffab719fc7f76c2289d4481` ProcessKit: adapt Modbus supplies to the welder process interfaces
- `51da75921e7ee75fa8301973ff574c2e7c10dbf3` ProcessKit: bind executed welding channels to device-neutral supplies
- `abcc33df689f435072e3a262e04e20dc4f2726d2` ProcessKit: preserve the external welding sensor fault contract
- `d3178440cbfbadcb24d9fa7a827497a379ba6b51` ProcessKit: publish supply feedback through authored welding sensors
- `46788d2e4def64d541736dfa4ca5cab59c8a83fb` MachineKit: record phase-one commits at the W6 ownership stop
- `2c5bab9ac499cd9ee531fdc20ef3846ff8267ea8` Merge main into mobile-welder for W6 host integration
- `3dac0dc69d4d8508e312d0221749bc3972b4cc5f` RobotKit: expose virtual peripheral profiles and numeric device observations
- `a084f1a1406f6a0a558e225535631d7db59a7c99` ProcessKit: generate the device welding sensor layout from its channel contract
- `71eb2bff05174acec86c47bb466a52cd8b7e9594` ProcessKit: reproduce the main plan control array length regression
- `1f898b2ea4c323379c62aecd73a20c67828c121f` RobotKit: route external device sensors outside the native sensor layout
- `1809a3b098519856a80143e4efa26d3c8c15bab5` RobotKit: derive virtual servo command grids from model drive data
- `68034eb23837750bbc6ff4d2367f267209455477` ProcessKit: connect CAD welding observations to external supply backends
- `bde03e2e139eef80bc37050397050a93b3e4f620` App: exercise an unchanged CAD weld mission through RKD6 and Modbus
- `b5453e8e60c8a0ed6fbe1c023ae650de00225eed` MachineKit: record resumed W6 integration and the main plan regression
- `95f6109caf9d1c6d1d593c875e29be0ebc175f4e` Merge main into mobile-welder for the W6 mission gate
- `aab794fa8742ae394634dc86e20a6d2a337ff67f` ProcessKit: verify plan control arrays use joint count
- `c188fd90893c9baaeb97795f273280e0abccf2d1` MachineKit: record the W6 nonzero device anchor blocker
- `5b99eb1bc7232a514d7c9162ff2e13a3048de4c8` RobotKit: establish held origins from initial device observations
- `eae3c12bf0de5afcf62a1a8c591df2f04b358355` RobotKit: expose virtual device plan readiness
- `70e6ab5dd7e704852a2c2110727ffcf0533c9b71` App: wait for virtual device clock qualification before welding
- `2daa96c825b93c60cd4340582e236236b8c39fe9` RobotKit: report rejected device conversion limits
- `fd82c53d704cba654963343e74e5dd00e3ad8cfc` RobotKit: configure virtual servo link and planning headroom
- `28233766033c792226f513b8db14a646ae0db416` App: apply virtual servo acceleration headroom to weld planning
- `00eda3131102fa27b7b0cd1636a3111ee770f852` MachineKit: record the W6 anchor repair and serial qualification blocker
- `083aa07211212a4d0eab21089998f703019fdf4e` Merge branch 'main' into mobile-welder
- `5c20666415cf1d306899c9ace13b42bc8a5e2c52` RobotKit: qualify buffered device streams by refill deadlines
- `cb8a30de92c1eb1c748fdf4f9c80daca954c32cd` MachineKit: record buffered-link qualification and RKD6 mission result
- `b4bf08d1948face07ff5a82b4e0a2ea9d1e32ad4` App: prepare CAD before opening the welding test connection
- `bc35cc79aa69a947aa92eaf6afcbcb0b54d38db5` MachineKit: record the Modbus mission owner scheduling failure
- `f4d5abcdf8b925cf4f060ebb559dda1c7d4e9ca9` ProcessKit: move the shared Modbus fake into its test package directory
- `f5c2d1bd207df6ff27611f67dfe32e03f73b5d25` ProcessKit: resolve the shared Modbus fixture by its explicit package
- `2a9977f51556ed4596c95d46ebca9f602cbe1244` ProcessKit: service welding devices independently of planning
- `c7eeb86ef31da49666c69224555fcb2efb2160e3` App: run the Modbus weld mission through its independent device owner
- `d7a5e443e85828050df83443df3a35842b1e9854` RobotKit: enforce link leases for active process outputs at rest
- `52316f89b6d75952b25000e2c9f03344071af371` App: verify device shutdown during active CAD weld missions
- `ed6409f803b9b10dd94d9a12fb5a4e72344691ce` RobotKit: derive process link requirements from applied channel values
- `15fa95fdba7c8313d3484da83d7708d6c3a511d7` RobotKit: rebuild the virtual device when event policy changes


### W6 final mobile gate repair and phase-one landing

The user authorized repairing the remaining mobile gate. The error came from the test reading
robot joint slots 0 and 1 as wheels. The restructured CAD assembly does not promise that ordering;
the drive already resolves the authored wheel joint names into the robot blueprint and supplies
those indices to its odometry. The long-term choice is to measure through that existing mapping,
without changing production joint ordering or introducing compatibility rules. The test still
checks the independently authored wheel radius, opposite shaft directions, chassis travel, turn
and wheel odometry. An explicit local odometry type also avoids haxeon's captured-local field
inference failure.

- `7143da2cab31aa89cec154f84699d269c6b169b5` MachineKit: record the completed device owner repair and final gate results
- `b1155a8b3fe2c5c523c802d6baf37b3200203cbf` App: check mobile wheel rates through the CAD joint mapping

Incremental app compilation passes (1,835 sources). The complete
`PROJECT_SOURCE_ONLY=mobile` gate now passes: deterministic and MuJoCo both travel 400 mm in 1 s,
turn 0.5 rad, and estimate 400 mm through wheel odometry. The MuJoCo mission retains 61 s;
obstacle recovery retains 65 s, one replan and 416 mm closest clearance. The production mobile
code is unchanged, so passed welder and arm gates were not repeated.

The full W6 validation includes the earlier passed welder gate (whole weldment 106.5 s,
680 mm, legs 4.9–5.1 mm on both backends), arm gate (23.5 s), RKD6 seam (20.36 s),
Modbus seam (20.92 s), all six shutdown cases on both device routes, 78 Modbus assertions,
16 scheduled-core checks, four applied-output policy checks, virtual/native device checks,
and the offline Nucleo profile check. Earlier baseline timing changes remain explained above.
MachineKit smoke passes (exit 0, `MachineKit smoke passed`), run once with the prebuilt OCCT
and read-only shared CadKit build. No OCCT rebuild or shared-build write was needed.

Local main has remained `a5d1e7e4008a8970fb7f627e124881aa4d4e7659` and is already an ancestor
through the W6 merge. All required gates pass. The completion-record commit is the landing commit; its full SHA
is reported after the guarded `git update-ref` fast-forward from that exact old main SHA.
Hardware bench/flashing and a verified vendor register map remain follow-ups: phase one proves
the unchanged CAD mission against simulated devices, including the independent supply owner and
arc-off shutdown behavior.


### Phase 2 execution scope (2026-10-05)

The user requested completing P1 through P5. P5 is now required rather than an optional stretch.
Work remains in `mobile-welder`; merge only local main before each milestone and land a milestone
only after its affected focused checks and its app gate pass. Hardware commissioning remains
separate from proving this complete mobile workflow against simulated hardware.

- P1: one mobile welding assembly with CAD-derived equipment placement, footprint and service
  connections; reusable battery/inverter parts, declared mass and discoverable capacity; no tool-specific
  requirements on ArmTool and no RobotArm changes. Validate fit, mass, power tracing and scene generation.
- P2: a reach/clearance-based station cover and ordering over every CAD seam, generating actual
  `goTo` and `weld` steps; validate coverage and a complete mobile weld mission.
- P3: executed contact-search moves on multiple work faces produce the observed work frame;
  injected parking errors up to 20 mm and 2 degrees must still yield correct bead length and leg.
- P4: battery observations integrate measured welder power, torque/speed motor power and auxiliaries,
  conversion losses and voltage sag; reserve the whole seam plus a route to the dock and execute
  charging before resuming. Validate discharge, cutoff, reserve decisions and an actual charging mission.
- P5: SensorKit's line profiler uses rendered depth observations; pre-weld localization and live
  seam correction run through the servo lane. Validate sensor geometry and correction with injected
  seam displacement during an executed weld, including stale/missing observation behavior.

P1 starts with `machinekit.power.BatteryPack`, `BatteryStorage` and `Inverter`. Storage is a
component facet; immutable runtime `robotkit.power.BatteryState` remains the observation type.
The inverter owns its conversion rating and efficiency; the service graph records a conversion,
not a transparent DC bridge. Envelopes, capacity and mass are explicit CAD inputs, avoiding
manufacturer claims or inferred battery mass from a solid enclosure. The focused power suite passes
12 checks covering capacity discovery, supply tracing through the inverter, losses, overload and
invalid ratings. No native build was needed. P1 assembly integration and its gates remain open;
the original example-local battery will be replaced rather than kept as a compatibility layer.


### P1 mobile carrier implementation (2026-10-05)

The carrier reuses the existing `RobotArm(false, new ArmWeldingTool())`. No RobotArm or ArmTool
changes were needed. `MobileBaseLayout` provides instance plate dimensions, payload and battery
positions; posts, caster seats and lidar placement follow the plate. The default base keeps its
600 mm plate, 380 mm track and geometry; its previous example-local battery is removed in favour
of `machinekit.power.BatteryPack`. Default pack capacity (480 Wh) and mass (5.148 kg) are explicit
fixture assumptions; the mobile welder's 48 V-class, 25 kWh LFP envelope (1,000 x 600 x 180 mm,
230 kg) and inverter envelope (600 x 400 x 300 mm, 45 kg) are engineering profile inputs,
not vendor-verified hardware claims.

The 10 kW inverter feeds the unchanged welding source's mains port. Separate isolated DC branches
feed the arm, wheel drive and computer. Four pack outlets feed those three branches and the inverter. The pack has no conductive connection to the chassis; the work lead
ends at the clamp on the separate weldment. `replaceComponent` preserves a local member's mating
and port interfaces, allowing the arm's standalone supply to be replaced by an isolated converter
without editing the reusable arm. Supply voltage resolution uses the nearest electrical conversion
output, while energy-source tracing traverses that conversion to the battery. Focused tests prove
that a 48 V pack through a 24 V converter preserves the 24 V motor curve, and a rejected replacement
does not mutate the assembly.

The initial layout placed the battery too close to the wheels; actual posed-solid overlap caught
it. Its forward edge now includes the wheel radius plus a 40 mm service gap. The equipment layout
then derives a 2,450 x 820 mm platform. Tests verify the battery, power source, inverter, cylinder
and both deck-mounted DC converters are within the platform and intersect no other carried component.
Total carried mass is **579.6 kg**, with no unaccounted component mass. Drive effort, wheel radius
and that mass set **0.209 m/s²** acceleration, including a 20% effort reserve. The yaw acceleration
uses the platform rectangle's mass-inertia approximation; this is an explicit conservative operating
profile, not a claim of measured yaw inertia. The actual mounting envelopes and source dimensions
remain CAD-derived.

The mobile scene is `materia.mobile.project.json`, with a grounded weldment on an independent
worktable. It has a torch, planar scanner and drive, but no generated welding mission until P2.
The CAD-to-runtime bridge now distinguishes robot membership from dynamic/free ownership: it selects
the declared mobile robot's kinematic subgraph and leaves joined stationary surroundings outside it.
A genuinely dynamic joined part or a joint/coupling/network crossing the robot boundary still fails.
Robot selection works on the flattened copy and does not mutate the source assembly. Grounded work
hulls outside the robot follow their actual simulated object poses, using the same preview-centred
body frame as those objects; hulls on robot links keep their existing path.

Validation: 17 power checks and the existing PowerSupplyTests pass; default MobileBaseChecks
and the mobile equipment fit, mass, service graph, scene and bridge-selection checks pass.
Final `PROJECT_SOURCE_ONLY=mobile-welder` passes on deterministic and MuJoCo: **400 mm in 2 s**,
with the battery and power source following the chassis (66 component records, including drive
isolation). App compilation passes (1,840 sources). The full fixed-welder gate completes with
exit 0: whole-weldment cycle **106.5 s**, 10 seams / 680 mm, legs **4.9–5.1 mm**, tip within
0.1 mm and no clearance violation on both backends. Single seam is 20.6 s, recovery 21.1 s with
one restart, woven 7 mm measures 6.998 mm, and three-pass 10 mm measures 9.999 mm. These match
the W5/W6 baselines and their previously recorded timing corrections.
Shared-app regressions pass: arm mission **23.5 s**; mobile handling round **61 s**;
mobile obstacle round **65 s**, one replan, closest 416 mm. MachineKit smoke passes with exit 0, including the mobile carrier checks. All P1 gates pass;
the milestone is ready for the guarded main sync. The captured-local IR bug was avoided by using direct
expressions in the inherited track-width method; no haxeon checkout or pin was changed.

P1 commits so far (since the W6 landing):

- `8d26c6e57080feee474ca1ba149e5e851bcfb785` MachineKit: model mobile battery storage and inverter conversion
- `b995b593bbb91fb49bc07c724bb2f6dd4c328eae` MachineKit: support an isolated DC supply in reusable assemblies
- `b5e4a8f6fdd86fd7ddf8cd52be4efe9a8b7ec3c2` MachineKit: resolve driver voltage at the nearest conversion output
- `4f8c7ecaa2e3d06286dc7c7970e2db445a6c3f8a` MachineKit: derive mobile platform geometry from its payload layout
- `1e5e320b060116a299e0cc3708c1dc99e5e98b63` CadBridge: select mobile robot membership separately from free parts
- `8b5b827ccb1315e2b610a5ba25f165ff13fd720f` App: ground external weldments through their live object poses
- `6d48e10bbc8afd77751ebb91e6824925e88eed0f` MachineKit: assemble the battery-powered mobile welding carrier
- `60d00e14721c5fb70f132058fa61b1b94771803f` App: verify the mobile welding carrier on both simulation backends

P1 drive-isolation completion: the wheel drivers now receive 48 V through a dedicated isolated
DC converter, matching the phase-2 hardware contract. The optional converter belongs to the reusable
base layout; the default base remains directly battery-fed. Posed-solid fit and service tracing
pass with the additional 2 kg component. The full fixed-welder gate completed with exit 0;
both three-pass checks measure 9.999 mm at 65.55 s. Carrier, arm, mobile and smoke gates now all pass.

### P2 station-planning design

Station policy belongs in ProcessKit; MachineKit supplies CAD seams, mounted-arm geometry and
posed collision bodies. A finite candidate grid is derived around the weldment with spacing and
standoff documented as planning parameters. Each accepted coverage relation must prove the actual
torch angles, whole path, approach and retreat reachable and clear; sampling only endpoints is
insufficient. Navigation feasibility and transition costs use the same footprint/map as execution.
Select the minimum number of stations over this explicit candidate set, then minimize feasible
route cost among equal-cardinality covers. This is not a claim of a global continuous optimum.
Uncovered seams or disconnected covers fail with a reason; the mission never silently drops work.
Within each station, retain the existing weld-run ordering and direction policy. The output uses
existing goTo/weld steps, so planning alone does not require a saved-format schema change.

P1 final commits: `fd2845b0f7282a907a25e37eac2ada846415191b` records the carrier decisions;
`3941a1a7e52b011b30da24db8e4d04668d78b2fa` isolates the mobile welding wheel drive supply.
P2–P5 remain required and open; carrier movement is not proof of a mobile welding mission.

P2 first implementation: `processkit.WeldStationPlanner` selects an exact minimum cover over
supplied candidates, then the cheapest feasible directed open route from the initial pose.
Access callbacks own full-motion reach/clearance evidence; navigation callbacks own feasible
route cost. Route edges are computed lazily and cached, including unreachable edges. Each seam
is assigned once even when station coverage overlaps. Missing coverage and unreachable complete
covers fail explicitly. The focused `processkit/tests/stations` compiler-only suite passes
26 checks, including a greedy-cover counterexample, count-before-cost priority, directed blocked
edges, overlapping assignment, input validation and route-call caching. No native build needed.
CAD candidate generation, actual swept access checks and complete mobile mission execution
remain open. Inspection also found that MissionPlayer's weld reference-frame lookup and clearance
currently assume work on robot links: P2 must include independent object frames/hulls, as P1's
simulated grounding already does. This cannot be claimed complete from the selection unit tests.

P1 landed on local main at `7e3474144d11ab731d01b661ce4a5b0d5228879c` using the guarded
fast-forward from `8a209d9ff03522eac26200b1595947da14c41225`. Main was merged before P2
and was already up to date. No branches were created and no remote was changed.

P2 external-work integration: `SimulationAssemblyParts` resolves CAD occurrence frames and convex
hulls uniformly from robot links or independent scene objects, restoring the CAD origin from each
preview centre. Weld grounding, mission reference frames and bead geometry now use that resolver.
External collision hulls are transformed from their current object poses into the arm's base frame
when each pass is planned, alongside the robot and deposited-metal hulls. The internal WeldBeads
constructor no longer takes a robot-only membership list. No RobotArm, ArmTool or saved schema changed.
Focused `PROJECT_SOURCE_ONLY=mobile-welder-frames` passes with exit 0 on deterministic and MuJoCo:
an external weld mission builds, both motion and deposition follow a live 12/-8 mm work displacement,
the carrier's ready pose clears, and putting the external base plate through the torch produces a
clearance violation naming that workpiece. App compiler-only build passes (1,842 sources).
The complete mobile weld execution and once-per-P2 full welder/arm/mobile gates remain pending.

P2 planning cost: station selection now supports conservative screening followed by deferred
full-motion verification of every assigned station/seam edge. Rejected full checks remove edges
and rerun exact selection; proven edges are cached. A successful minimum optimistic cover is also
minimum over proven feasible covers, since removing other provisional edges cannot improve its
count or route cost. Screens may reject only impossible edges, not merely unproven ones.
The focused station suite passes 37 checks, including a provisional one-station cover whose
swept entry fails, retry to a proven two-station cover, cached full checks and no unverified
assignment. This is an optimization of proof order, not a substitute for swept motion checks.

P2 candidate geometry: `WeldStationCandidates.around` proposes a documented heading/standoff grid
around CAD seam endpoint bounds in the work frame. The chassis pose subtracts the actual mounted
arm offset, so standoff describes arm access rather than a long platform's centre. Work-frame
translation and yaw transform every station; no world parking coordinates are authored. Grid
resolution and standoff band are explicit planning parameters. Candidates remain unproven until
navigation and full-motion checks accept them. The focused station suite passes 148 checks,
including mounted-arm distances, heading, rigid-work-frame equivariance and invalid bounds.

P2 shared planning boundaries: the existing posed-obstacle floor rasterizer is extracted into
`robotkit.navigation.FloorMap`, preserving navigation ownership and the public MissionPlayer
helper surface. Focused tests pass 82 assertions for rotated-obstacle detours and blocked parking.
The follow-up map fix anchors cells to world multiples of the resolution; 482 assertions now
also prove that adding candidate goals does not move obstacles between cells. Mission maps must
include the robot's initial live pose, not only destinations, because a one-station mobile weld
can start farther from its destination than the previous fixed map margin.

`WeldingPlanRunner.planning` exposes the same compiled swept-motion checks used by execution,
without creating a robot/channel owner. `PlannedWeld.endJoints` records the verified retreat's
joint configuration, including the compiled trajectory endpoint, so ordered runs can continue
from the actual branch instead of resetting their proof to a home seed. The focused planning
suite passes 44 existing planning assertions and 19 pass-path assertions, plus a complete compiled
factory check and a retreat-endpoint check within 0.5 mm. No native build needed.

P2 stow validation exposed a real nozzle/upright collision on the direct retreat-to-ready joint
move. `robotkit.manipulation.JointRoute` supplies a generic bounded bidirectional joint-space
search with deterministic low-discrepancy sampling, finite joint bounds and caller-owned directed
edge checks. It keeps exact endpoints and shortcuts only checked edges. Search exhaustion is
reported as a budget failure, not proof of geometric impossibility. The focused suite passes
21 assertions for detours, exact endpoints, deterministic output, blocked endpoints, bounded
failure and direction-sensitive checks. The mobile integration checks each proposed air edge
against both the joint-space sweep and the actual compiled exact-stop trajectory; the CAD
station integration result remains pending.

P2 scene pass interpretation is moved unchanged from MissionPlayer to `processkit.WeldScenePlan`.
The CAD station checker and live mission runner now share process, weave, corner-normal and
pass-offset conversion. ProcessKit depends on ProjectKit's scene data contract; ProjectKit has
no dependency on ProcessKit, so this introduces no dependency cycle. Scene schema stays 16.
The focused CAD station target compiles this shared conversion and uses it in full-motion checks;
the complete mobile execution gate is still pending.

P2 CAD station integration now passes: two stations cover all ten CAD seams, with 14 mission
steps and 9.323968 m of A* route. Candidates come from work-local seam bounds and the mounted
arm offset, then conservative parking screening and full compiled weld checks select the cover.
Actual ordered runs continue from verified retreat joints. Stow uses checked joint-space detours
and emits absolute mechanical joint targets by restoring the CAD initial-position offset.
The previous nozzle/upright collision is resolved by the validated detour. Physical views are
created from the normalized current-schema scene so resolved material properties are available.
`materia.mobilemission.project.json` exposes the planned mission while the carrier-only manifest
continues serving the focused carrier check. This CAD result proves planning and coverage; P2
remains incomplete until complete mobile execution and the required regression gates pass.

P2 first complete execution reached the first station's welds and checked stow, but failed on
step 9 (the second `goTo`) at 111.72 s: AStarPlanner could not find a traversable route. The
focused floor-map regression reproduces the cause: a fixed one-metre margin clips the enlarged
carrier's detour when the map bounds include only selected stations, while the proposal map has
more distant candidates. `FloorMap.navigationMargin` now derives bounds from the actual footprint
radius, soft inflation distance and two grid cells, keeping the old one-metre minimum for light
carriers. CAD planning and execution use the same rule. All 485 floor-map assertions pass,
including rejection of the clipped route and successful detour with derived bounds. The app
execution retry and milestone gates remain pending.

P2 execution after the map repair reached both stations and all welds, then rejected the last
stow submission at 268.71 s (step 13): RobotKit status -2. MissionPlayer cached a joint runner's
old commanded endpoint across intervening welding ownership. Each new joint step now resets
the completed runner's planning anchor and acquires the live robot start while retaining its
compiler and plan sequence. A focused executed handoff passes on both native backends: cached
owner to 0.2 rad, independent owner to 0.4 rad, cached owner reacquires and returns to 0 rad.
This is application ownership policy; no RobotRuntime or MotionKit semantics change. App compile
passes with 1846 sources. The full mobile mission retry and milestone regression gates remain open.

P2 complete mobile execution now passes on both native backends using the same generated scene:
two stations, ten unique CAD seams, 270.3 s cycle, lengths 40/40/40/40/40/40/180/180/40/40 mm.
The test backend measures legs 5.0–5.1 mm; MuJoCo measures 4.9–5.1 mm. Every bead meets the
2 mm extent and 0.5 mm leg tolerances with no gap; tip tracking stays within 1.5 mm while the
arc is established. Weld and stow clearance checks pass, and driving/stow transitions require
the arc out. The cover is minimum among the documented CAD-derived candidate grid; this is
not a claim of global optimality over continuous parking space. The final welder, arm, mobile
and MachineKit milestone regression gates and main sync remain pending.

### P3 observation and registration design, prepared while P2 gates run

Use three CAD-derived, nonparallel work faces and a 3–2–1 contact pattern to constrain the
complete rigid work frame: three separated contacts on the first face, two on the second and
one on the third. Fit point-to-plane constraints from measured joint configurations and fresh
`tool_weld.touch` observations; reject deficient rank or inconsistent residuals. Do not reuse
`SurfaceRegistration` as a complete weld-frame solution: its documented single-plane result
leaves translation in the plane and rotation about its normal unobserved.

CAD supplies the nominal faces, reachable probe regions and search bounds. Executed contacts
supply the correction. The registration input must not include live simulation object poses.
Those poses remain valid for physical grounding, bead attachment, clearance and test assertions.
Keep contact search and welding registration in ProcessKit, CAD probe selection in
`machinekit.welding`, and generic motion/servo mechanisms in their existing owners. Reacquire
the frame after every parking step, and use the accepted observation for all station seams.
Injected parking displacement must change the real base pose without giving the estimator the
same correction: Simulation.placeRobotBase applies a jump without stopping the clock or resetting
sensor history, and wheel odometry can retain the estimated pose. The test must demonstrate that executed contacts,
rather than an exact simulation localization or work-body lookup, recover the weld frame.
P3 implementation, saved-format changes and executed noise checks have not started.

P3 probe uncertainty must include parking yaw about the chassis origin, not only translation.
At 1.7 m from that origin, 2 degrees contributes about 59 mm of position error; add the
translation bound and the tool's clearance when deriving search envelopes from CAD. A fixed
30 mm search would not cover the requested parking error. Use provisional observable pose
components from early face contacts to place later probes on narrower faces. Provisional
estimates can guide searches but cannot authorize welding; the final registration must pass
the full observability and consistency checks.

### P2 final gate record (2026-10-05)

All affected gates passed, with heavy suites run one at a time. App compiled 1846 sources;
MachineKit smoke compiled 1307 sources. No native rebuild or OCCT rebuild was needed.

- Focused station policy: 148 checks for minimum covers, directed routes, deferred verification
  and CAD-derived proposals. Shared welding planning: 44 planning and 19 pass-path assertions
  plus compiled-motion/end-joint checks. Joint-route search: 21 assertions. Floor map: 485 assertions.
- CAD station integration: two stations, all ten seams, 14 steps, 9.323968 m planned route.
- Complete mobile welding: both backends, 270.3 s, all 680 mm covered without gaps; legs
  5.0–5.1 mm on the test backend and 4.9–5.1 mm on MuJoCo; weld and stow checks pass.
- Independent frame and collision checks passed earlier on both backends. The focused joint-owner
  handoff passes on both backends with moves to 0.2 rad, 0.4 rad and back to zero.
- PROJECT_SOURCE_ONLY=welder: terminal exit 0. Whole fixed weldment remains 106.5 s on both
  backends, all ten beads within length/leg tolerance and tip within 0.1 mm. Single seam 20.6 s,
  recovery 21.1 s with one restart and 3 mm overlap, no gaps. In-air failure retains the exact
  three-restart reason. Crater dropout has no restart; displaced work remains within 0.1 mm and
  leg 5 mm. Post chain remains the recorded W4 baseline of 29.8 s, not the former unchecked
  19.9 s; the clearance/wrist-limit reason is recorded above. Woven 7 mm measures 6.998 mm at
  27.17 s; three-pass 10 mm measures 9.999 mm at 65.55 s on both backends.
- PROJECT_SOURCE_ONLY=arm: terminal exit 0, pick/place round remains 23.5 s.
- PROJECT_SOURCE_ONLY=mobile: terminal exit 0. Both bases move 400 mm and turn 0.5 rad; handling
  round 56.4 s, obstacle round 60 s, one replan, closest 413 mm. The prior 61/65 s and 416 mm
  record changes with world-aligned map cells and bounds that include the starting pose. This
  changes discretized routes and turn/guard timing while retaining handling success, obstacle
  avoidance and one replan; the enlarged-carrier route-clipping regression is fixed.
- MachineKitSmoke: terminal exit 0, including mobile carrier and welding generation checks.

P2 preserves scene schema 16 and the public ArmTool/RobotArm interfaces. The planned mobile
mission has its own manifest; the carrier-only manifest remains a distinct motion/fit fixture.
P3 executed contact registration, P4 measured energy/docking and P5 rendered-depth tracking
remain required and open. P3 design notes above are preparation, not implementation evidence.
Local main was still 7e3474144d11ab731d01b661ce4a5b0d5228879c throughout these gates; perform
the guarded fast-forward after committing this milestone record.

P2 commit ledger before this final record:

- `dc16212bb637c176d9029c9b3edd10c0852d7702` ProcessKit: select minimum welding station covers and feasible routes
- `a7637b14e8adc1ba800f5627d2ed303e7fccf0b4` App: resolve independent welding work through live assembly frames
- `833dc0b88b416b8b60b9172f0b48a02455d3eb8a` ProcessKit: verify selected welding station coverage before accepting it
- `18f000453447fc73f25ef1f956859b258e99a722` ProcessKit: derive welding parking candidates from work bounds and arm mounting
- `1c63277550263eeb2c38eb7ddfc37b5c6dc37fbb` RobotKit: share posed floor obstacle mapping with upstream planners
- `7c0624d0c3ba00a47a414f9fabc6cf520a2b4c50` ProcessKit: expose complete welding motion checks for station planning
- `54c918cfb396cc4be63ed29aeeb9f09f6000467c` RobotKit: plan checked joint-space detours around obstacles
- `75c98a5639ec87e99e420df8523fb9dde2be0fb7` RobotKit: keep floor cells stable across mission goal sets
- `e0262617dcc2d1d890b25f6bd49e56a853c51e90` ProcessKit: share scene welding pass conversion with CAD planning
- `591a02580f78ad091afa5040690b0fa0da15e395` MachineKit: generate mobile welding stations and checked stow moves
- `aa75823205c08a7a4c5e14ad8697104a55dcba19` RobotKit: size navigation map margins for enlarged footprints
- `574d49ba97fa49250c1650de4a806dec13b0e14c` App: reacquire live arm starts between mission motion owners
- `0e20c96730971d68e02d969280381216ac447a65` App: verify complete mobile welding missions on both backends
- `619f2cf2734a048e37823d030fda4a197b6ed873` ProcessKit: record the observed-frame design for mobile weld registration

### P3 rigid contact fit implementation (2026-10-05)

`processkit.perception.ContactRegistration` fits nominal CAD planes to measured observer-frame
contact points. The scaled six-dimensional normal system uses a symmetric pseudoinverse so
partial fits update only observable directions. One face gives rank three, two faces rank five;
only a converged rank-six fit inside residual, translation and rotation bounds authorizes welding.
Partial estimates remain available to guide subsequent probes. No live work-body pose is an input.

The focused contact-registration suite passes 109 assertions: full rigid-frame recovery,
chassis-origin parking extremes (20 mm per axis and 2 degrees yaw), observer-frame equivariance,
scaled plane equations, deficient/duplicate contacts, inconsistent observations, exceeded limits,
iteration exhaustion and invalid inputs. This is numerical solver evidence only. Executed contact
searches, CAD probe-region selection, saved mission integration and displaced-parking weld checks
remain open; P3 has not passed its milestone gate. P4 and P5 remain required and open.

### P3 bounded touch-search policy and servo adapter (2026-10-05)

`ContactSearch` owns the arc-off contact-search policy: a measured TCP origin, inward direction,
finite distance/time/corridor bounds, and an explicit calibrated sensing-threshold offset. Fresh
noncontact feedback must arm a search before a new touch can finish it. Missing, stale, future,
regressing or clock-mismatched observations, invalid frames, active arc/current and welder faults
stop probing. Sensor skew is bounded on the joint clock rather than inferred from host receipt time.

`ContactSearchRunner` connects this policy to an exclusively owned fresh MotionKit servo session.
It reads measured joint positions for TCP FK and captures that point on touch before requesting
braking. Completion waits until the servo reports rest. The caller must first execute a checked
approach with arc and wire channels off; this adapter does not replace approach planning or CAD
probe-region clearance checks. Existing MotionKit servo deadlines bound a stalled host's commands.

The focused suite passes 20 search assertions plus the existing 109 registration assertions.
The servo adapter compiles, but its executed native motion has not yet been exercised: the unit
suite proves the search policy and constructor validation, not a mobile contact-search mission.
CAD probe selection, safe approach/retreat, saved mission steps and observed-frame use remain open.

P3's focused native-motion suite now passes 15 assertions on the deterministic backend. A
prismatic probe executes the real ServoSession and native runtime against WeldArcModel contact
feedback: calibrated contact is within 0.06 mm of a physical plane, the recorded point precedes
braking, and stale feedback and a no-touch search both brake to rest without a registration point.
This proves the adapter's executed motion boundary, not the multi-face mobile mission or MuJoCo.

### P3 CAD probe patches and parking envelopes (2026-10-05)

`machinekit.welding.WeldProbeGeometry` extracts polygonal planar B-rep faces in the nominal
weldment reference frame. Face boundaries and holes determine sample regions; table and clamp
faces can obstruct nominal approach rays without becoming registration targets. Containment of
an entire rectangular uncertainty region checks boundary crossings as well as corners, so a hole
inside a region cannot be missed. Unsupported curved geometry fails explicitly instead of being
silently dropped from visibility. This initial implementation supports the example's polygonal
solids; curved registration geometry remains unsupported.

`WeldProbeParkingBounds` derives conservative face-plane intersection bounds from the nominal
CAD chassis frame, per-axis translation limits and yaw about that chassis. Projected yaw extrema
are analytic over the continuous angle interval, rather than a discrete corner-only sample. Ray
normal projection and tangential rotation expand the region bounds. Planar parking error introduces
no artificial vertical uncertainty on horizontal CAD faces. At a 1.7 m chassis lever arm, the
20 mm/2 degree envelope can exceed 70 mm laterally, rejecting narrow first-contact patches.

The focused CAD suite passes 3,101 assertions, including actual mobile-workpiece patches (468
nominally exposed samples), hole/boundary rejection, fixture masking, world-placement invariance,
unsupported geometry and dense displaced-ray checks of the continuous parking bounds. No native
build or OCCT rebuild was needed. These are geometric candidates, not reach-approved probe moves.
Multi-face selection, checked approach/refinement/retreat and mission observed-frame integration
remain open. Probe order must follow observable uncertainty and feasible patch width rather than
always assuming the top face is the first usable datum.

P3's `WeldProbePatterns` now builds CAD-derived 3–2–1 stage candidates. Three contacts must be
noncollinear; the second pair separates along the two planes' intersection to observe the remaining
rotation; the final plane's normal must complete an independent basis. Pattern ordering favors
geometric spread, while later arm planning must still prove each complete move. An uncertain
approach prism conservatively masks every other face's projected bounds, including off-centre
fixtures; yaw-induced search-ray tilt expands that prism. This can reject usable patches but must
not accept a nominal centre ray whose uncertain contact could hit a neighbour.

The CAD suite passes 3,167 assertions, including generated pattern observability and full rigid
frame recovery from the selected six CAD constraints. Its later-stage tests deliberately use zero
uncertainty to verify pattern geometry; they do not claim that runtime observations have reduced
uncertainty. Actual posterior bounds and staged probe execution remain to be integrated.
For planar parking error a broad vertical datum can be followed by an independent vertical datum,
then a horizontal one: the second vertical face measures the unresolved horizontal displacement
before a narrow top patch is approached. Choose feasible stages by current measured uncertainty,
not by a fixed top-first order. Fine contact refinement is needed so detection-period error divided
by the first-stage baseline does not amplify into unacceptable seam error far from those contacts.

### P3 observed-rest correction (2026-10-05)

The complete probe integration exposed an ownership boundary in `ContactSearchRunner`: a servo
braking tick's `atRest` describes its commanded zeros, which are applied on the next native owner
tick. Search completion now also requires observed joint velocities below 1e-5 and no active
trajectory. Waiting for measured rest has a finite bound derived from observed speed and configured
joint acceleration plus two seconds; a brake that does not settle fails instead of waiting forever.
Detection-time contact capture remains unchanged. The focused native suite passes 31 assertions,
including observed rest at every basic search handoff and the prepared refinement integration.

### P3 complete single-probe execution (2026-10-05)

`ProbeMotionPlanner` checks complete compiled approach and straight refinement/retreat trajectories,
including short swept clearance edges. Approach IK candidates use bounded JointRoute detours; a
blocked endpoint-to-endpoint sweep cannot become an accepted move merely because its goal is clear.
Prismatic checks sample at 1 mm joint increments, revolute checks at 0.02 rad, and actual compiler
end joints are retained. A measured per-joint deadline-brake prediction is also screened during
searches. This is a predicted stopping reference from observed velocities and configured acceleration,
not a proof of every possible Cartesian QP braking path or an independent safety controller.

`ContactProbeRunner` executes an arc/wire-off checked approach, a coarse search, checked withdrawal,
a fresh slow contact measurement and checked retreat. Servo and motion programs own the arm in
separate phases. Observed rest alone does not release a sticky native velocity target: the runner
submits a measured position hold, waits for its native owner tick, and reacquires the program start
before retreat/refinement. This resolves the native -2 rejection at the first withdrawal without
changing RobotRuntime or weakening the plan anchor contract. Fresh servo instances are disposed
between searches. Failed probing clears its contact result and stops the robot with its existing
channel-safe policy; a contact is only complete after retreat.

The focused deterministic native suite passes 31 assertions. The full one-axis probe executes two
separate touch episodes, refines a 20 mm-deep plane observation to within 6 micrometres, returns to
its prepared air pose and leaves the joint at rest with arc and wire off throughout. Separate cases
verify stale/no-touch stops, unreachable goals, intervening fixtures and predicted braking obstruction.
This proves a complete single probe; CAD stage selection, measured posterior bounds, mobile scene
steps and the noisy multi-face welding mission remain open. P3 has not passed its milestone gates.

### P3 measured pose uncertainty (2026-10-05)

`ContactPoseEnvelope` retains a bounded set of chassis pose errors consistent with measured plane
contacts. Conservative interval rotation, translation contraction and same-plane contact differences
reduce the set without treating unobserved degrees of freedom as zero. A finite subdivision/cell
budget leaves wider feasible regions; reducing the cell cap merges them conservatively. Inconsistent
contacts fail explicitly. Its provisional centre places subsequent probes but does not authorize
welding: the final six-observable-component rigid fit remains required.

Search-ray intersection bounds include the chassis yaw lever arm and remaining translation/rotation,
with explicit metre units and calibrated contact error. Focused tests pass 51 envelope assertions,
20 search assertions and 109 rigid-fit assertions. They cover parking extremes, partially observed
interior poses, general six-component uncertainty, exact tolerance boundaries and budget exhaustion.
CAD selection using these measured bounds and the executed mobile registration mission remain open.

P3 CAD pattern selection now accepts a `WeldProbeUncertainty` region provider. The analytic parking
prior implements this interface; observed bounds are supplied through an explicit provider without
adding a ProcessKit dependency to MachineKit. Region extents/travel use CAD length units while tilt
is dimensionless. The focused CAD test converts measured metre bounds explicitly and selects all
three 3-2-1 stages using the surviving pose set, updating it after each stage rather than substituting
zero uncertainty. For an interior displaced chassis pose, each selected search-ray intersection is
inside its predicted region, and the six contacts produce an accepted rank-six fit. The CAD suite
passes 3,177 assertions with 468 exposed samples. These observations are synthesized for the focused
geometry test; execution on the mobile robot, saved registration steps and noisy mission gates are
still required before P3 can land.

### P3 measured registration execution sequence (2026-10-05)

`ContactRegistrationSequence` owns the 3-2-1 measurement policy. A caller supplies checked CAD-derived
probes on one nominal work plane per stage; the selector receives previous independent normals and
the measured pose envelope. Each complete stage refines that envelope before selecting the next.
Parallel planes, inconsistent observations and deficient final contact rank fail explicitly. Only an
accepted six-component rigid fit publishes a result, with residual tolerance tied to calibrated
contact error. A sequence is single-use so measurements cannot be carried across parking events.

`ContactRegistrationRunner` drives the sequence through one `ContactProbeRunner`, accepting each
measurement only after its checked retreat completes. Probe failure, selector failure and cancellation
leave no published work frame. ProcessKit owns this execution policy; the selector boundary keeps
CAD shape extraction and candidate/motion preparation outside the measured registration state.

Focused tests pass 18 sequence assertions plus 51 envelope, 20 search and 109 rigid-fit assertions.
The native contact-motion suite passes 34 assertions: its new wrapper case executes three full probes
(six coarse/fine touch episodes), rejects an unavailable second independent plane, and leaves arc/wire
off with observed joint rest. This one-axis case proves the execution/failure boundary, not successful
six-component mobile registration. Saved mission integration, CAD-aware request preparation on the
six-axis arm and both-backend injected-parking mission gates remain open. P4 and P5 remain required.

### P3 saved contact job boundary (2026-10-05)

SceneArtifact schema 17 adds `findWork` with the actual torch contact connector and a nominal CAD
contact job. It stores the work reference occurrence, nominal assembly_T_work in metres, bounded
chassis XYZ/RPY errors, calibrated observation error/contact offset, and polygonal target/fixture
faces in the work frame. It stores no measured result or live body pose. Each face carries its
occurrence/face identity, outward normal, plane centre and all contour edges, including holes.

The strict decoder requires typed numeric/vector fields. Validation rejects missing occurrences,
invalid frame quaternions, unsupported bounds, repeated face identities, non-unit normals,
nonplanar/open boundaries and a target set without three independent normals. Probing must name
the torch's connector, not another valid connector. Schema 16 is rejected; there is no migration.
Tracked scene-envelope JSON examples have their own unchanged version 1; no tracked binary
SceneArtifact fixtures were found to regenerate. Generated artifacts use the current schema.

ProjectKit passes 18 new saved-contact assertions and 143 existing assertions. The new mission step
is not yet emitted by the mobile generator or executed by MissionPlayer: those are the next P3
integration changes, followed by actual noisy multi-face registration on both backends.

MachineKit now exports probe patches to the saved contact face format with explicit CAD-to-metre
conversion and reconstructs them without reopening CAD solids. The actual mobile cell's faces,
fixture masks, holes and material samples survive this conversion. Its exported geometry passes
saved-job validation, and the focused CAD suite passes 4,912 assertions with 468 exposed samples.
This establishes the CAD data boundary; it does not prove arm reach, actual multi-face execution or
the injected-parking welding mission. No native build or milestone-wide gate was run for this change.

### P3 normal probe pose preparation (2026-10-05)

`ProbePosePlanner` prepares a normal sensing corridor from a nominal root-frame CAD point, outward
normal and conservative remaining normal travel. Wire +Z points inward. Sixteen rolls are tried,
nearest the observed torch orientation first; each accepted prepared air pose has a complete checked
approach through ProbeMotionPlanner. The air offset is normalTravel plus 3 mm, and the bounded search
distance is twice normalTravel plus 3 mm, covering either sign of the unknown plane displacement.
Continued IK is checked at 1 mm intervals over that entire corridor, rather than accepting a reachable
nominal point while a possible displaced contact lies beyond the arm's reachable branch.

Corridor kinematics do not turn a search into a nominal buffered contact move: before localization,
that nominal move might penetrate the real plane. CAD patch/approach-region screening checks geometric
exposure, and the executed search retains its measured collision/braking checks. This separation is
necessary under parking error; checked approach and post-contact refinement/retreat remain unchanged.

The focused native motion suite passes 43 assertions. New cases verify wire alignment, both signs of
normal uncertainty and rejection specifically at an unreachable corridor despite reachable nominal
contact/approach poses. On the existing six-axis arm fixture, three independent plane normals produce
prepared wire orientations and checked approaches ending within 0.1 mm of their requests. This is
six-axis preparation evidence, not an executed registration of the actual mobile CAD weldment. Saved
mission emission/execution, actual mobile probe selection and both-backend parking-error gates remain
open. No native build or milestone-wide app gate was run.

### P3 mission integration draft (2026-10-05)

The current uncommitted integration emits `findWork` after each mobile parking step and executes it
through `FindWeldWork`/ContactRegistrationRunner. A measured root_T_work is scoped to that station;
starting another goTo or resetting the mission clears it. Welds whose frame has a contact job require
an accepted measurement and compose it with the same wheel estimate used by WeldSeam, cancelling
that estimate when expressing the motion in the robot root. Registration's nominal input uses the
wheel estimate and saved CAD assembly_T_work; live body poses are used only by clearance checks and
test assertions, not by the registration fit or nominal prior.

The saved-CAD-to-probe adapter is in an optional `machinekit-process` integration package under
`machinekit/process`, in the `machinekit.welding` namespace. Adding ProcessKit to machinekit-robot
would create a dependency cycle through CadBridge, so the core packages remain unchanged. ProcessKit
owns measured sequence execution and probe motion; MachineKit supplies CAD patch selection and the
adapter. The application wires those owners to its skill lifecycle and station frame scope.

The application compiles (1,862 sources). A focused production-scene check is running: it injects a
12/-9 mm chassis-frame translation and 0.025 rad yaw after parking, without resetting wheel odometry,
then requires contact registration and the first weld on both backends. The first run failed before
probing: geometrically selected broad patterns included blocked/unreachable approach endpoints.
This draft is not a passing P3 milestone and is not landed on main.

P3 candidate-screening repair: ProbePosePlanner now screens prepared approach configurations over
its torch rolls and IK candidates, including endpoint clearance. WeldProbePatterns accepts an
optional point predicate before selecting its broad contact pattern. A geometrically good but
unreachable extreme no longer displaces all reachable interior points. This remains candidate
screening: selected probes still require the complete checked approach and bounded corridor, and
execution rechecks its actual measured start. Focused native tests pass 45 assertions; CAD tests
pass 4,913 assertions. The mission adapter tries 9/17/33-point-per-axis lattices in order until a
checked pattern succeeds, retaining finite search and avoiding the densest scan when unnecessary.
The repaired focused mission check remains pending.

P4 preparation audit while the P3 production preview runs: declared component mass is currently
lost at the preview boundary. MachineComponent.massProperties returns declared vendor kg/COM and
optional centroidal inertia in kg mm², but AssemblyPreview.scene exports Part.massProperties from
the preview solid. MateriaProjectRunner then computes kg as volume*density*scale³, and CadBridge
uses that same geometric mass unless its occurrence callback overrides it. The saved scene currently
has geometric inertia moments, scaled by density*scale⁵, rather than explicit physical mass/inertia.
P4 must repair this producer/data/consumer boundary before claiming measured motor-energy realism;
changing pack ratings or multiplying loads afterward would hide the physical-model mismatch.
No energy implementation or mass-format change has been made during this P3 execution check.

The filtered focused run is live after production CAD generation; no pass/fail result is available
at this point. Its source geometry artifact is cached beneath app/build/project-cache using the
existing content/dependency/native-stamp fingerprint, keeping cache writes in this worktree. The
next compiled focused check additionally requires twelve coarse/fine touch episodes, verifies that
wheel localization did not receive the injected base jump, and measures frame error at the first
weld's actual CAD seam endpoints rather than an arbitrary nearby point. Those added assertions have
not yet been compiled/run; the current live process uses the preceding test binary.

### P3 clock ownership stop (2026-10-05)

The filtered focused run has terminated with exit 1. It now reaches the coarse contact search, then
fails exactly: `step 1 (findWork): Coarse: contact search needs fresh welding feedback on the joint clock`.
The scene generator completed and emitted 66 component records (2,877,017-byte artifact). No successful
registration or weld is claimed; the enhanced contact-count/odometry/seam-endpoint assertions remain
uncompiled and unrun.

The concrete framework mismatch is RuntimeRobotAdapter.snapshot (robotkit/core/haxe/robotkit/runtime/
RuntimeRobotAdapter.hx, source-clock argument): it labels all joint snapshots `unspecified`.
VirtualWelderSupply publishes raw device-clock observations as `robotkit.device.<epoch>`.
VirtualDeviceEndpoint::sensor_sample converts board timestamp ticks to nanoseconds at the device
rate, retaining its device epoch; it does not map them into the joint observation clock. Equal numeric
units do not establish a shared clock, and republishing at receipt time would hide sampling uncertainty.
The existing strict ContactSearch condition is therefore correctly failing.

A clean prerequisite is authoritative source-clock metadata from the runtime adapter/endpoint plus
an explicit bounded device-to-joint mapping, using RobotKit's existing ClockMapping/ClockMappings
contracts or an endpoint-owned equivalent. Its uncertainty must count against the permitted contact
skew/position error; clock epochs, missing mappings and stale observations must fail. RobotRuntime.hx
has not been edited, and the clock equality/freshness guard has not been weakened. This crosses the
brief's RobotKit framework ownership boundary, so execution work stops here for an owner repair or
explicitly authorized scope change. The mission adapter/application/generator draft remains uncommitted;
main remains f4dc7456a65fb32dd2459d9a36d36dd8c80457a1. P3, P4 and P5 remain incomplete.

### P3 authorized clock prerequisite (2026-10-05)

The user authorized the framework repair across the earlier ownership boundary. RobotRuntime.hx
and RobotArm.hx remain untouched. Inspection corrects the earlier diagnosis: RKD6 joint records,
like its peripheral samples, retain board ticks converted to nanoseconds. They already share the
same board oscillator; their source clock must not be called the host/simulation clock and does
not need a fictitious device-to-joint calibration. Direct simulation joint records use physics time.

RuntimeEndpoint now exposes its source-clock identity. Factories supply endpoint-owned identities
with instance tokens and reset epochs; RuntimeRobotAdapter preserves that identity on joint and
native sensor observations, while external frames retain their actual publisher's clock. SimKit
notifies observers after a successful session reset so RobotKit can invalidate every endpoint epoch
even when a later timestamp numerically repeats; individual robot resets invalidate that endpoint.
This is a Haxe lifecycle change with no native ABI change or rebuild. Unexpected backward endpoint
time also starts a distinct epoch; no mapping across it is inferred.

SimulatedWelder publishes the completed physics observation's actual joint source timestamp and
endpoint clock, rather than the separately monotonic step-observer integration time. The virtual
supply retains raw board sample time and uses the same board clock identity as its joint records.
For genuinely independent devices, ContactSearch and its execution wrappers accept explicit
ClockMappings. The complete mapped uncertainty interval must precede measured FK and fit inside
the existing age budget, retaining the speed*age contact-position bound. Unknown identities,
missing/expired mappings, excessive uncertainty and a changed joint or sensor epoch fail probing.

Final focused checks pass: 24 endpoint-clock assertions including a 70 ms device offset, 125 ppm
drift, preserved external clocks and repeated-timestamp resets; a separate-process clock-identity
check; 20 existing search assertions plus 10 mapping/epoch/invalid-time assertions; 50 native
probe-motion assertions. The latter also prove that a repeated-timestamp reset aborts search,
buffered probing and the entire registration sequence before points from different epochs can
be combined. The final app source compiles 1,863 sources, using compiler-only builds and existing
native libraries. No OCCT/native rebuild was performed.

The production registration retry completed CAD generation (66 records, 2,877,017 bytes), passed
the former clock prerequisite and reached the coarse servo search. It then terminated with exit 1:
`step 1 (findWork): Coarse: Joint target must be finite`. The twelve-contact/seam checks did not run
to completion; neither backend has a successful P3 registration result. The final epoch-guard
additions were checked in the focused native suite and the app was recompiled after them.

A separate diagnostic beneath ignored app/build/servo-diagnostic reproduces the framework error
without CAD: one bounded prismatic DOF, velocity limit 0.1, absent maxAcceleration, requested TCP
speed -0.005 m/s and dt 0.01. ManipulatorServo produces `velocity=NaN, fallback=true, status=-1`.
The same call with acceleration override 1 produces `velocity=-0.004999989999781267`, finite,
`fallback=false, status=0`. ManipulatorServo maps an absent acceleration to positive infinity;
StepLimits.brakingSpeed then evaluates infinity times zero for a finite travel distance, and its
NaN bounds propagate through the differential-IK fallback into the joint target. No MotionKit or
KinematicsKit source was changed in this clock prerequisite. The user's outside-scope stop rule
applies to this newly exposed framework defect; the clean next repair belongs to the acceleration
limit contract and bounded differential IK, followed by resuming the P3 mission check.

Clock repair commits are `bcfcd3e07f79869cb89a633a71e1f7898b92be7b` (RobotKit endpoint clocks
and reset epochs) and `cfe73e334` (ProcessKit mapped-clock probing and simulation observations).
The mission adapter/application/generator draft remains uncommitted. P3 is not a passing milestone,
so no main fast-forward was attempted; main remains f4dc7456a65fb32dd2459d9a36d36dd8c80457a1.
P3 full two-station and 20 mm/2-degree boundary validation, P4 energy/docking and P5 rendered-depth
tracking remain required and open.

### P3 authorized unlimited-acceleration repair (2026-10-05)

The user authorized repairing the newly exposed kinematics prerequisite and resuming P3.
Local main was merged before this work (already up to date at f4dc7456a65fb32dd2459d9a36d36dd8c80457a1).
KinematicsKit owns differential-step travel and braking limits, so the repair is in StepLimits.
The braking roots are now evaluated in their rationalized form: it is algebraically equivalent
for finite positive acceleration, avoids cancellation at high acceleration, and has a finite
travel-bound limit when acceleration is positive infinity. Absent acceleration, zero and positive
infinity consistently mean unlimited acceleration; position, velocity and ramp turning-point
bounds still apply. Invalid previous velocities fail before QP bounds can be produced. No motor
rating or arbitrary acceleration cap was added to hide the numerical defect.

Focused checks pass: 39 pure bounds assertions (including multiple numerical scales and interior
ramp overshoot), 32 native differential-IK assertions (including the bounded fallback after QP
failure), and the 50 native contact-probe motion assertions. Builds remain compiler-only with the
existing native libraries. The production app rebuild passed (1,863 sources). The repair is
committed as 00e80e2b43e8a0daaa0952069cdfce79fb724082, `KinematicsKit: keep unlimited acceleration
steps finite`.

The production registration retry then stopped before CAD generation with exit 1:
`Could not compile Materia project entrypoint (exit 1)` and
`/tmp/materia-project-3098987-1791196217.54144-804289383/preview.hl: No such file or directory`.
The producer reports MateriaProjectModuleBuild.hx:75, its File.saveBytes call for the compiled
module; the runner rethrows at MateriaProjectRunner.hx:222. The exact log is in ignored
app/build/p3-limits-registration.log. The cause of the missing temporary output directory is
not established. No compiler change, stale artifact bypass or production registration pass is
claimed. This new tool/environment failure triggers the user's outside-scope stop rule.

P3 remains incomplete, with production registration, full two-station coverage and boundary
validation still open. Main remains f4dc7456a65fb32dd2459d9a36d36dd8c80457a1; no fast-forward was
attempted. P4 energy/docking and required P5 rendered-depth tracking remain open.

### P3 disk-space retry and lazy single-contact screening (2026-10-05)

The user asked to continue and suggested low disk space as the temporary-output failure's cause.
With 4.9 GB free, a retry using TMPDIR under app/build/project-tmp compiled and generated the
expected production artifact (66 components, 2,877,017 bytes). This demonstrates that the tool
failure no longer reproduces; it does not establish its original cause. No compiler/runtime or
producer change was needed. All temporary files and the existing dependency-fingerprinted cache
remain inside this worktree.

The first uninstrumented registration run was stopped after approximately 25 minutes without a
result. Test-only parking/contact progress messages were added and the app rebuilt successfully
(1,863 sources). Its retry loaded the cached scene and executed ten touch episodes on the
deterministic backend, from 27.05 through 52.48 simulated seconds, passing the former nonfinite
servo prerequisite. It was stopped while still computing after approximately 20 minutes, to
repair an identified avoidable candidate-screening cost. Neither interrupted run is a pass or
a terminal registration failure; measured frame accuracy and bead acceptance remain unverified.

For the one-contact stage, WeldProbePatterns formerly screened arm configurations for every
geometrically admissible point, then selected the nearest reachable point to the face centre.
It now screens nearest points in order and stops at the first reachable point on each face.
Grid-order ties remain deterministic, rejected points fall through to the next nearest point,
and an entirely unreachable face is rejected. The approach bound is derived from the selected
point. Three- and two-contact broad-baseline selection, exposure/uncertainty checks and full
checked motion preparation are unchanged. This avoids redundant IK/clearance work without
reducing the candidate lattice, search bounds or physical proof requirements.

The affected CAD probe suite passes 4,940 assertions and retains 468 exposed samples, including
nearest-point preservation, one screening call per immediately reachable face, rejection
fallback ordering and entirely unreachable faces. The production retry after this optimization
is still required. P3 is incomplete and main has not been advanced.

### P3 executed-phase diagnosis and probe wire model (2026-10-05)

The subsequent retry reproduced the ten-touch sequence and was interrupted without a terminal
registration result. A phase-instrumented app retry then established that all executed searches
so far were the three probes on the first face, not two completed faces. Four touch episodes
occurred during air approaches (two before the first coarse search and two before the third).
The first face's final retreat began at 52.51 simulated seconds; computation then delayed the
second-face selection/preparation. The single-contact screening optimization is valid focused
work but did not address this observed delay. The diagnostic run was stopped to repair the
confirmed air-contact omission and an identified envelope refinement inefficiency.

ContactPoseEnvelope now bounds yaw sensitivity by point radius times the horizontal component
of the observed plane normal. This follows d(Rz p)/d(yaw) = Z cross Rz p. A horizontal plane's
residual is yaw-invariant, so splitting its unobserved yaw interval cannot improve a constraint.
The previous all-axis radius bound could create 256 redundant cells. The full parking prior
remains present in one cell for that case; no pose component is assumed known. Focused tests
pass 67 envelope assertions (including all planar parking corners), 18 sequence assertions,
109 fit assertions and the existing 20 search assertions. The affected CAD suite still passes
4,940 assertions and 468 exposed samples. Commit: 4cae123ecb9b76e79f0cefcadc9b8d1f18435ec5.

The rigid physics hull deliberately excludes consumed wire, but the unlit protruding wire is
real geometry during contact probing. ProcessKit ProbeWireClearance derives a conservative
circumscribed wire envelope from the supplied CAD diameter, stickout and link-local tip. It
checks wire pairs against non-tool solids, including intervening air paths and predicted
braking. The wire's own rigid tool skins are excluded from these additional pairs; ArmClearance
continues to check the complete arm/nozzle with its original margins. Air wire clearance is
3 mm. Near-contact sensing permits positive separation below that air margin but rejects
touch/overlap of the wire envelope. No physical simulation collision body or saved schema was
changed, and no nominal contact motion is used to locate the real plane.

ProbeMotionPlanner and ProbePosePlanner include this optional, matching-arm wire check. The app
supplies it only for registration, from the current saved torch facet; fixed welding planning
keeps its original clearance. Scene collision-body extraction is shared so both checks use the
same posed hulls. Native probe tests pass 57 assertions, including wire-only intervening
collisions and braking, near-contact sensing and penetration rejection. Commits:
db47f8a261504e6f287f493ab525e0d9186729db (wire checks) and
f02af8e5180c83403736ac94d97b8339010c5fd8 (sweep input validation).

The updated app compiles 1,864 sources. Its production retry is running with per-probe phase
diagnostics; no production pass is claimed yet. All builds remain compiler-only using existing
native libraries. Main remains f4dc7456a65fb32dd2459d9a36d36dd8c80457a1; P3, P4 and P5 remain open.

### P3 parking-fault publication boundary (2026-10-05)

The wire-aware production run removed the two first-probe approach contacts, but one contact
remained during the third approach. Its diagnostic retry terminated with exit 1 at that contact:
FK matched the physical tool point within 8.968100940422724e-16 m, while the frozen clearance
world returned null. The measured joints were 0.6676030989851316, -0.21797454761040297,
-0.9093220264586326, -1.3662865416110088, -0.7644976321991027, 2.464939730522297.
The stricter test now rejects unexpected touch during any checked air approach instead of
waiting for the final touch-count assertion. Log: ignored app/build/p3-air-diagnostic.log.

A focused posture replay parks the production carrier, applies that joint configuration and
compares the sensor/FK and hull queries without executing the expensive probe selector. It
passes: sensor/FK error 8.955206467987463e-16 m, physical touch true, grounded distance
-0.0008329146493810356 m, wire clearance correctly rejected against work/postRight. The matching
point-to-hull distance is -0.0008329046672925777 m and the convex wire distance is zero. This
rules out the hypothesized wire-query or sparse-sampling explanation for the remaining contact.
Diagnostic geometry is saved only beneath ignored app/build/p3-wire-posture.json. Its focused
PROJECT_SOURCE_ONLY filter is mobile-welder-wire-clearance.

The fault-injection test violated placeRobotBase's documented next-tick semantics: it queued
the jump, then started findWork before the plant published it. Planning captured clearance
against the old root pose and execution used the jumped pose. The test now advances the plant
and sensor observers once directly after injection, before feeding the mission. It does not
reset wheel odometry, copy the physical root into the registration prior, or reset source-clock
epochs. A consistent post-jump observation precedes planning, while the same stale wheel pose
and injected parking uncertainty remain inputs to contact registration. No simulation/runtime
source change was needed. The new full registration retry is pending.

### P3 checked sensing branch and direct approach preference (2026-10-05)

Preparation now retains its checked joint goal in the runtime contact request. Execution
checks an air route to that same configuration, verifies the resulting TCP against both IK
tolerances and rechecks continuation through the complete bounded sensing corridor before
submitting movement. Manually supplied TCP requests receive the same executed-corridor check.
This closes the previous mismatch where execution could select a different IK branch from
the one proved during preparation. These requests are ephemeral; no saved format changed.

Approach policy tries checked direct IK alternatives across the existing torch rolls before
spending the bounded detour budget. Rolls retain their nearest-orientation order within each
pass. If no direct approach has a reachable sensing corridor, the complete original detour
search remains the fallback. Clearance sampling, wire guards and the 1,024-proposal detour
budget are unchanged. This is a ProcessKit planning choice, not a runtime or CAD exception.

The focused native probe suite passes 64 assertions, including retained joint goals, manual
corridor rejection, mismatched-goal rejection before motion and obstructed direct approaches.
The rejection tests publish the queued stop before retrying; otherwise the native owner is
correctly unavailable for a new plan. Compiler-only builds use existing native libraries.
The production P3 retry remains outstanding; main has not advanced.

The branch-preserving production retry generated 66 records (2,877,017 bytes) and completed
three coarse/fine pairs on its first face without air-approach touches. The last retreat
started at 59.49 simulation seconds. It was deliberately interrupted with exit 143 while
planning the second face; this is not a complete registration pass. Log:
ignored app/build/p3-branch-registration.log. Commit for the tested branch policy:
97ff470b267acd3edfc959aeb6c5901c0ce4e749.

Two-contact CAD selection previously screened every eligible lattice point for IK before
using only the extrema along the planes' intersection. It now screens those extrema lazily,
rejecting unreachable points from the outside inward and preserving lattice-order ties.
This retains the same broadest reachable pair and face score/order for a fixed predicate;
interior points cannot change either extremum. The stage approach bound is derived from its
two selected points. All geometric region and exposure checks remain in force, and three-
contact breadth selection is unchanged. The focused CAD probe suite passes 4,975 assertions
with 468 exposed samples, including unchanged extrema/order, unreachable-extreme fallback,
all-unreachable rejection and two checks per face when both extrema are reachable. An initial
call-count assertion incorrectly ignored geometrically narrow patches discarded after
screening; that assertion was corrected. No production speedup or P3 pass is claimed yet.

Three-contact selection now applies the same lazy ranking policy to the original four
farthest-point sweeps and largest-area third point. It first finds the earliest reachable
lattice point, then rejects unreachable points in each geometric rank order. Already
rejected points cannot affect any later sweep. With a fixed feasibility predicate this
produces the same choices as eager filtering; lattice-order ties remain deterministic.
The approach bound is derived from the three selected points. Reachable patterns need six
ranked checks per face rather than a complete lattice of IK checks. Focused CAD tests pass
5,141 assertions and 468 exposed samples, including unchanged geometric scores, ordering
and point choices with six reach checks per face. The two-contact-only production retry was
interrupted (exit 143) during initial-face planning; no additional production pass is claimed.
The next retry includes lazy screening for all three contact-pattern sizes.
