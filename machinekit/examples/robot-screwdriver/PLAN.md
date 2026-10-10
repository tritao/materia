# Robot screwdriver plan

Goal: a robot that drives the screws of an assembly, with the screws, the
threaded holes and the screwdriving equipment all coming from the CAD, and a
simulated screwdriver that shares one interface with a real tightening
controller. Threads become real model data on faces, and the editor draws them
as cosmetic threads.

It follows the robot welder's shape (`../robot-welder/PLAN.md`): the work is
found in the CAD, the tool is declared through capabilities, a deterministic
process model runs in simulation, a `ProcessDevice` boundary is shared by
simulation and hardware, and a mission step drives it.

## Decisions (2026-10-09)

- **Axial compliance is in the tool, not the arm.** The spindle sits on a
  spring-loaded feed slide with a sensed stroke, as on production screwdriving
  units. The arm only makes position moves, and the slide supplies and measures
  the feed force. No force control on the arm in this plan (see Later).
- **The screw is simulated by a process model, not by thread contact.** A
  deterministic torque-angle model decides how the screw advances and seats, as
  `WeldArcModel` decides the arc. The screw is held on the bit and moved
  kinematically along the hole axis by `angle × lead`. MuJoCo never sees a
  thread.
- **Screws are presented, not blown in.** A presenter nest with an escapement
  and a part-present sensor; the bit has a vacuum sleeve that holds the screw,
  reusing the suction pieces. Blow-feed through a tube is later.
- **Threads are face attributes, drawn cosmetically.** A thread is semantic data
  on a named face (holes and shanks alike). Helical geometry is never modelled.
  The editor draws threads as a texture overlay on the face's own triangles (see
  SD2, which takes FreeCAD's thread texture as its reference and avoids its
  bugs).

### Implementation decisions

- **SD0 — Property class owns the fastener's mechanical material.** Screws and
  bolts offer 8.8, 10.9, 12.9 and A2-70 as typed classes; their recipes store
  `propertyClass` and derive material, rather than accepting a second,
  contradictory material input. Other component recipes retain their material
  input. The machine member record drops its duplicate material field: recipes
  already supply material, including overrides on other components. Defaults
  remain 12.9 for cap screws and 8.8 for hex bolts.
- **SD0 — Published table values retain their limits.** The tightening catalog
  records Optimas's [steel table](https://optimas.com/technical-resources/pre-load-and-tightening-torques-of-metric-thread-steel-fasteners/)
  at friction 0.125 and [A2-70 table](https://international.optimas.com/technical-resources/stainless-steel-fasteners-pre-load-and-tightening-torques/)
  at friction 0.1, with torque in Nm and preload in N. These are published
  maximum assembly guidance values, not a validated joint process program.
  Only the catalog's M3–M20 coarse sizes are included; unsupported fine pitches,
  hands, starts and custom dimensions report a missing row instead of
  interpolation. Fine thread designations still derive from custom screw specs.
- **SD0 — Thread data belongs in CadKit.** MachineKit fasteners and lead screws
  share `cadkit.threads.ThreadSpec`; metric, trapezoidal, Acme and ball-race
  profiles retain their distinct meaning. Ball races have no cosmetic flank
  profile. A function-only `ScrewFastener` interface shares driven-fastener
  facts without haxeon's class-to-typedef structural assignment.

## What exists

- Fasteners in `machinekit.standard`: `SocketHeadCapScrew` (ISO 4762),
  `HexBolt` (ISO 4017), `HexNut`, `FlatWasher`. Specs carry diameter, pitch,
  head and socket sizes, thread length, tap drill, ISO 273 clearances and
  DIN 974-1 counterbores; screws expose `head` and `tip` connectors and
  clearance/tap/counterbore cutters. Threads are semantic only. There is no
  drive type, no property-class mechanics and no tightening torque.
- `LeadScrewThread` (`machinekit/motion`) has diameter, pitch, starts and hand.
- Assemblies seat screws with a `fixed` mate from a frame on the clamped part
  to the screw's `head` (`FlangeBearingAssembly` `screw$i-seat`, also
  `MotorShaftBearings`). `FlangeBearingAssembly` already sizes screws for 1.5 d
  engagement and exposes `screw<i>Tip`. Which part the screw threads into is
  implicit: there is no fastening relation.
- CadKit: `HoleFeature` (plain, counterbore, countersink; blind or through)
  names its bore body `"bore"` for topological naming, but has no thread data.
  `Shape.faceAxis()` gives a cylinder/cone face's axis and radius, and
  `GeometricConnectors` turns faces into frames.
- Tool plumbing from welding and machine tending: `ArmTool`,
  `EndEffectorControls`, `RobotScene.robotTools`, cadbridge
  `EndEffectorControlBinding`/`EndEffectorRuntimeBridge`, `ToolChannels`,
  `SimulatedSuctionTool` (holds an object on a link), process-driven tool
  joints (`rk_simulation_velocity_drive`, the bench mill's spindle, with an
  effort cap), `PortKind` `Vacuum`/`Pneumatic`/`ElectricalPower`/`Signal`.
- ProcessKit: `ProcessDevice`, `ProcessEngagement`, `ProcessRun`, the welder's
  outputs/feedback split, `WeldContract` and its Rust generator, the Modbus
  client, and the bounded guarded move of `ContactSearch`.
- Rendering: SceneKit has one PBR material per node with base-colour and normal
  textures, `REPEAT` samplers, and `TEXCOORD0` streams. CAD tessellation keeps
  each face's triangle range (`cad_mesh_face_range`), and the face-hover
  overlay (`ScenePresentation.ensureFaceHover`) already draws one face's own
  triangles in a child node with its own material. Tessellation does not export
  UVs, and there is no per-face material.

Missing: thread data on faces, the fastening relation, a screwdriver tool kind,
a tightening model, a screw mission step, screw feeding, an Open Protocol
client, and any thread rendering.

## Steps

**SD0. Fastener data.**
- A `ScrewDrive` enum (hex socket, hexalobular, cross, hex head, slot) with its
  size, so the bit is chosen from the screw. SHCS: hex socket of `socketSize`;
  hex bolt: hex head of `acrossFlats`.
- `PropertyClass` (8.8, 10.9, 12.9, A2-70, …) on screws and bolts, replacing the
  free-text `material` default. Class determines material; recipes have no second
  material input. Defaults stay 12.9 for cap screws and 8.8 for hex bolts.
- A tightening table: assembly torque and preload per size and property class
  at a stated friction coefficient, with its source recorded like the existing
  catalog sources. No numbers are invented: they come from a cited table.
- A `ThreadSpec` value type: designation (`M6`, `M6x0.75`), major diameter,
  pitch, starts, hand, internal/external, tolerance class. Metric coarse and
  fine come from the existing specs; `LeadScrewThread` yields a `ThreadSpec`
  (trapezoidal), so lead screws join the same scheme.
- Checks: every catalog screw has a drive, a thread spec and a torque for its
  default class; `LeadScrewThread`'s lead equals `ThreadSpec` pitch × starts.
  Class survives recipe and saved-assembly round trips; contradictory material
  overrides and unsupported tightening rows are reported.
  Recipe changes advance the machine format from v13 to v14 and the CadKit
  document from v11 to v12; the two empty BIM examples are regenerated with the
  current encoder. Existing geometry, default materials and mass stay unchanged.

**SD1. Threads on faces.**
A thread is attached to a named cylindrical (or conical) face, never found again
by matching radii.
- `ThreadedFace`: face name, `ThreadSpec`, start (the face end the thread begins
  at) and length (full face or a stated depth).
- `HoleFeature` gains a tapped style: the bore is cut at the tap drill and the
  bore face carries an internal `ThreadSpec` and thread depth. `tapHole()` on
  the fastener components uses it.
- Screw and bolt shanks carry an external `ThreadSpec` over `threadLength` from
  the tip, so a partly threaded ISO 4762 screw shows its plain shank.
- `Threads.of(part)` returns each threaded face with its axis frame from
  `Shape.faceAxis()` of that face: origin at the thread start, +Z into the
  material for internal threads and toward the tip for external ones.
- Through-holes, counterbores and countersinks keep working; a counterbored
  tapped hole is not a thing (rejected).
- Checks: a tapped M6 hole's thread axis and depth; editing the hole position
  keeps the thread on the same named face; two equal tapped holes keep their
  own threads (the case FreeCAD #30005 got wrong); a part with an unthreaded
  hole of the same diameter as a threaded one shows no thread there.

**SD2. Cosmetic threads in the editor.**
Reference: FreeCAD PR #22573 "PartDesign: thread texture" (merged 2026-03),
fixes #28461 and #30005, open #24072 (thread-end ring). FreeCAD re-triangulates
the bore faces and lays a hand-drawn translucent PNG over them with UVs
u = angle / 2π, v = axial distance / pitch, a clip plane at thread depth, a
mirrored U for left hand, and a per-hole hash phase. Its overlay lives outside
the model and found its faces by radius and axis, which caused both fixes.

Here:
- **Bound to the model.** The overlay is built from `Threads.of(part)` (SD1):
  the face comes by name and the axis from the face itself, not from a sketch
  or a radius search.
- **Same triangles as the face.** The overlay mesh is the face's own triangle
  range (`subelementGeometry(faceIndex)`), like face hover, so depths match the
  base mesh exactly and there is no z-fighting. It lives in a child node of the
  part's runtime node and follows pose overrides as hover does.
- **UVs in Haxe.** u = angle around the thread axis / 2π × starts, mirrored for
  left hand; v = axial distance from the thread start / pitch. Triangles that
  cross the seam get duplicated vertices (u + 1). Triangles past the thread
  length are cut at the end plane, instead of a clip plane.
- **A generated texture, not a drawing.** A small tileable normal map from the
  60° (or trapezoidal 30°) profile, plus a base-colour texture slightly darker
  at the roots, both mip-mapped with a `REPEAT` sampler. This uses SceneKit's
  existing normal mapping, so no shader change, and works on desktop and web.
  The overlay's material takes the part's appearance (colour, metallic,
  roughness) so a thread looks like the part's own surface.
- **Phase from the name.** Each thread's u offset comes from a stable hash of
  its face name, so overlapping threads (a screw in its hole) look distinct.
- **Thread-end ring.** A cosmetic circle at the end of a partial thread, drawn
  through the existing stroke pass.
- **External threads too.** Screw shanks draw the same overlay facing outward.
- A view setting turns cosmetic threads off.
- Only if the textured look fails close up: a thread mode in
  `scene_fragment.glsl` that draws the profile analytically with `fwidth`
  antialiasing. Not planned unless needed.
- Checks: overlay vertex count equals the face's; UVs across the seam stay
  continuous; a 20 mm deep thread in a 30 mm hole stops at 20 mm; left-hand
  threads mirror; a screenshot test of a tapped plate and an M6 screw on the
  desktop and web builds.

**SD3. Fastenings from the CAD.**
- `Fastening`: screw (or bolt), clamped member and its clearance hole, threaded
  member and its tapped hole (or a nut), engagement length, `ThreadSpec`,
  tightening spec (torque, angle window), and a name made from the face names,
  so it survives edits like a weld seam.
- `Fastenings.find(assembly)` derives fastenings from the existing seat mates
  and the threads: the screw's axis runs through a clearance hole in the seated
  part and into a coaxial threaded face of the next part. It reports instead of
  skipping: no threaded face on the axis, thread mismatch (diameter, pitch,
  hand), engagement under the minimum (default 1 d in steel, 1.5 d in
  aluminium), clearance hole smaller than the major diameter, screw too long
  for a blind hole.
- `FastenedAssembly` resolves them, like `Weldment.join`.
- Workpiece: `FlangeBearingAssembly` on a plate with tapped holes, then a NEMA
  motor on `NemaMountPlate` (four M3).
- Checks: the flange assembly gives four fastenings with the right engagement;
  shortening a screw reports under-engagement; moving the plate's holes off the
  flange's bolt circle reports no threaded face.

**SD4. Screwdriving equipment and the cell.**
- MachineKit `machinekit.screwdriving`:
  - `ScrewdriverSpindle`: motor and gearbox body, the spindle as a real
    revolute tool joint, on a feed slide that is a real prismatic tool joint
    with a spring rate and a sensed stroke. `tcp` at the bit tip.
  - `ScrewdriverBit` for a `ScrewDrive`, with a vacuum sleeve (`Vacuum` port).
  - `TighteningController` (mains input, tool cable, control port; program
    slots; control interface: discrete I/O or Open Protocol).
  - `ScrewPresenter`: nest, escapement and part-present sensor, fed by hand for
    now, holding one screw size.
  - Capabilities: `ScrewdriverFacet(tcpConnector, controlPort, maxTorque,
    maxSpeed, strokeMm, springRate)` and `TighteningSupplyFacet(programs,
    controlInterface)`.
- `ArmScrewdrivingTool implements ArmTool`.
- `EndEffectorControls`: a `Driver` control with channels
  `<prefix>/<member>.drive` (start), `.direction`, `.program`, `.vacuum`. The
  cadbridge binding gains the variant.
- Scene artifact: `robotTools` kind `screwdriver` with its sensor and
  `screwdriver` block (bit, stroke, spring rate); validation alongside
  `suction|torch`. One schema version; older documents are rejected.
- `ScrewdrivingCell`: arm on its pedestal, controller on the floor, presenter
  and fixture plate on a table, the SD3 workpiece clamped. Start-page entry.
- Checks: the bit matches every fastening's drive or the cell reports which;
  every fastening and the presenter are reachable with the tool on the hole axis;
  the tool clears the workpiece at each approach.

**SD5. Tightening model and simulated screwdriver.**
- `TighteningProgram`: run-down speed, final speed, target torque, angle
  window from snug, torque limits per phase, maximum run-down angle, stroke
  window. Programs come from the fastening's tightening spec.
- `TighteningModel` (pure, deterministic): from the screw's state, the feed
  force (spring rate × stroke) and the spindle angle it gives
  - engagement: the screw starts only if it is on the axis within the hole's
    lead-in and the feed force is in range; otherwise it slips (no advance) or
    cross-threads (torque rise before the expected run-down angle);
  - run-down: advance = angle × lead, prevailing torque;
  - snug: when the head reaches its seat face;
  - final: torque rises with joint stiffness per degree until the target.
  Outcomes: OK, no screw, no start, cross-thread, stripped (torque drops), early
  high torque, angle out of window, run-down too long, screw lost.
- `ScrewdriverSensor` contract: torque, angle, speed, stroke, phase, result
  code, vacuum, with a Rust contract generated like `weld_contract.rs`.
- `SimulatedScrewdriver`: a step observer on the tool link. It reads the
  channels, computes the stroke from the TCP depth against the screw head, runs
  the model, drives the spindle and slide joints to the model's state, moves the
  held screw, and publishes the sensor frame. Holding reuses
  `SimulatedSuctionTool`'s attach.
- `ScrewdriverProcessDevice` with `ScrewdriverOutputs` (select program, start,
  reverse, vacuum) and `ScrewdriverFeedback` (result, live values).
- Checks: each outcome from a constructed case; an M6 into steel reaches
  target torque within its angle window; a misaligned start reports no start;
  the same inputs give the same trace.

**SD6. Drive one screw.**
- Scene `screw` mission step: fastening name, program, and the derived hole
  frame relative to the threaded member's live pose.
- ProcessKit `DriveScrew` skill and `ScrewdrivingRunner`: pick a screw from the
  presenter (vacuum and present check), approach on the hole axis, engage at
  run-down speed, follow the screw at the feed stroke's midpoint while driving,
  wait for the result, release vacuum, retract.
- App: `MissionPlayer.skillFor` gains `screw`; `SimulatedTools.screwdriverFor`;
  the sensor kind in `AssemblyRobot`. The screw occurrence moves and turns with
  the model as runtime pose, and its cosmetic thread turns with it (SD2), so the
  screw is seen winding into the hole.
- A tightening result record per fastening (final torque, angle, outcome).
- Checks: one M6 driven on MuJoCo and the test backend, OK result, screw seated
  on its seat face; with the screw removed from the nest, the step reports no
  screw.

**SD7. Whole assembly.**
- `ScrewdrivingMission.generate`: every fastening into `screw` steps, with
  presenter trips; ordering by pattern (cross order on a bolt circle) and an
  optional two-stage run (all to snug, then all to final); reach and clearance
  diagnostics shaped like `WeldAccess`.
- Not-OK recovery: reverse out by the driven angle, drop the screw in a reject
  bin, retry with a new screw once, then report the fastening as failed and go
  on.
- A run report: per-fastening results and failures.
- Checks: the flange assembly's four screws in cross order on both backends; an
  injected cross-thread on one screw is recovered and the run finishes.

**SD8. Real tightening controller.**
- Open Protocol client (TCP): communication start, parameter-set select, tool
  enable/disable, tightening-result subscription and keep-alive; the message ids
  and revisions come from the protocol specification, recorded in the client.
- A discrete I/O profile (program bits, start, reverse, OK/NOK, ready) for
  controllers without a network interface.
- An RKD6 virtual nutrunner behind the same contract.
- `ScrewdriverProcessDevice` runs against either, with the same mission
  unchanged from simulation.
- Checks: the client against a recorded or scripted controller; the virtual
  device runs the SD6 mission unchanged.

## Later

- **Force control on the arm:** a force/torque sensor kind, contact-force
  readout from MuJoCo (`mj_contactForce`), an admittance loop over
  `ServoSession`, and a spiral hole search. Needed for holes whose position is
  uncertain beyond the lead-in, and for screwing without a compliant slide.
- **Blow-feed:** a feeder, a feed-tube port kind and a nosepiece on the tool.
- **Hole location by vision or touch,** reusing the welder's registration.
- **Physical screw joints** (spindle plus coupled revolute/prismatic screw in
  the world), if a process ever needs MuJoCo to feel the thread.
- **Drawings:** cosmetic thread lines from the same `ThreadedFace` data.
- **Pick, place and fasten cells** wait for machine tending's physical jaws
  (MT8), cell signals (MT6/MT7) and part trays (MT9), and an automatic tool
  change.

## Order

SD0 → SD1 → SD3 is the CAD core: threads and fastenings from geometry. SD2
(cosmetic threads) needs only SD1 and can run beside SD3. SD4 → SD5 → SD6
drives one screw in simulation; SD7 completes the cell and SD8 is the step to
hardware.

Dependencies:
- SD1 leans on topological naming for face names across edits.
- SD4–SD6 follow the welder's pieces closely; changes to `robotTools` and
  `EndEffectorControls` must keep `suction` and `torch` working.

## Progress

Validation uses the supplied suite, copied to `build/sd-suite-local.sh` to create
the root `build/` directory and include the app's CollisionKit, KinematicsKit,
MotionKit and TrajectoryKit library directories at runtime. The initial full
baseline exposed those environment omissions; its successful build stages were
retained while the runtime stages were retried with the complete path. NativeKit's
empty mandatory libwebsockets and sokol checkouts were populated at their recorded
pins. All native CadKit configurations use the prebuilt OCCT cache.

| Step | State | Commits |
| --- | --- | --- |
| SD0 | open | |
| SD1 | open | |
| SD2 | open | |
| SD3 | open | |
| SD4 | open — waits for machine tending on local main | |
| SD5 | open — waits for machine tending on local main | |
| SD6 | open — waits for machine tending on local main | |
| SD7 | open — waits for machine tending on local main | |
| SD8 | open — waits for machine tending on local main | |
