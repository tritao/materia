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
- Use one current saved weld representation: a non-empty pass list, each pass carrying its path, process, weave
  and optional interpass dwell. Bump the then-current scene version and regenerate fixtures; reject older versions.
- For the required 10 mm example, split its 50 mm² area into root/fill/cap fractions 1/4, 3/8 and 3/8
  (12.5, 18.75 and 18.75 mm²). Derive offsets from the seam's face normals and previously deposited height.
  Earlier station geometry must participate in both grounded-work sensing and clearance before later passes ignite.

W5 implementation notes (in progress, 2026-10-04):
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
| W3 | done (+ hardening: paths, live workpiece frame, exit crash, safety tests) | `weld` mission step; `WeldSeam` skill and `WeldBead`; process engagement and `WeldingPlanRunner`; weld metal part and recipe; bead as runtime geometry; welds on MuJoCo and the test backend, restart with overlap |
| W4 | Done (2026-10-04) | Whole CAD weldment: 10 seams / 680 mm in 4 runs; swept clearance and derived corner turns; 106.5 s on both backends, legs 4.9–5.1 mm; affected gates pass. Timing changes recorded above. |
| W5 | | |
| W6 | | |
