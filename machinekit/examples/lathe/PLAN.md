# Slant-bed turning lathe plan

**Goal.** A compact, precise CNC turning lathe for steel, designed in materia from real
catalogue-class parts so it could later be built. Its first job is the turned parts of our own
strain-wave gears. In the editor it turns a part the way the real machine would, and measures the
result like an inspection room: diameters, roundness, wall thickness, runout, finish, threads.

1. A blank sits in a power chuck, or on an expanding mandrel, on a belt-driven spindle (S, with C
   positioning later). An 8-station servo turret on a 45° slant bed carries the tools on X
   (radial) and Z (axial) slides.
2. Feed per revolution and threading are a controller relation on the **measured** spindle
   position, the same electronic gearbox as the flexspline hobber (HB-D2). Spindle sync error is
   a real output, so a mistimed threading pass shows up as a damaged thread.
3. CAM turns a revolved target part plus a tool table into a turning program: facing, roughing,
   finishing with nose-radius compensation, boring, grooving, threading, part-off, over one or
   more setups.
4. The machine runs the program through the MotionKit/RobotKit stack. The simulation removes
   material on meridian sections of the workpiece, with each insert's real cutting edge at the
   instant the edge passes each section, so feed marks, thread profiles and non-round shapes
   appear from the kinematics.
5. A virtual inspection reports the part against its drawing tolerances. With named error sources
   switched on (spindle runout, thermal growth, turret indexing, tool and wall deflection, chuck
   distortion), it produces an error budget that says which part limits the result.
6. The machine is tending-ready from the start: power chuck, automatic door, a defined loading
   envelope and robot interface signals. The tending cell itself comes later.
7. It is a platform: live tooling, C, a tailstock, a bar feeder and other options attach later
   to interfaces designed in now; a Y axis, a sub-spindle and turn-grinding would be sibling
   machines.

**Target parts:** the turned blanks of the strain-wave chain, up to about Ø90 × 60 mm:
- the flexspline cup blank, soft-turned, then hard-turned to its thin wall after heat treatment.
  Its bore is the hobbing datum, so this lathe feeds the flexspline hobber
  (`examples/flexspline-hobber/PLAN.md`);
- the circular-spline ring blank;
- later, the wave-generator cam by non-round turning (X following C).

Background on the parts and their process chains is in `machinekit/STRAIN_WAVE_RESEARCH.md`
("Manufacturing").

The work spans kits:
- `stockkit`: the section stock (`SectionStock`) and the turning sweep;
- `toolpathkit/tool/`: ISO inserts, holders and the tool table;
- `machinekit/haxe/src/machinekit/turning/`: spindle cartridge, power chuck and actuating
  cylinder, turret and its stations, slant bed, door;
- `machinekit/examples/lathe/`: the `SlantBedLathe` assembly, checks, preview, project;
- `robotkit` runtime and `motionkit`: path progress geared to the measured spindle (feed per
  revolution, threading), constant surface speed;
- `toolpathkit`: turning ops (per-revolution feeds, threading passes, turret index, chuck, door);
- `camkit`: `TurningJob`;
- `cnckit`: a lathe G-code dialect (later);
- `app`: real-time section-stock cutting, inspection panel.

## What exists

Surveyed 2026-10-19 on local main 023c0a520. Nothing on main turns. The hobber's branch
(`flexspline-hobber`, a73ca2a54, unmerged) holds several pieces this plan needs; they are listed
separately.

**Machines and parts (main).**
- `AxisBuilder` / `MachineAssembly` (`machinekit/haxe/src/machinekit/assembly/`):
  `place/attach/slide/driveScrew/turnWithBelt/addMotor(…, ?processVelocity)/addEncoder/
  addCoupling/addCylinder/addSwitch/addAtSpeed/addPresence/addRobotZone`.
- `examples/bench-mill/` is the closest template:
  - `BenchMill.hx`: belt-driven spindle as a continuous joint with `ProcessVelocityDrive`
    (`spindle.speed` / `spindle.direction`, scale 2π/60) and `addAtSpeed`;
  - `EnclosedBenchMill.hx`: pneumatic sliding door and vise on `PneumaticCylinder` +
    `SolenoidValve`, wired to `CncController` valves, with `door-open/closed`,
    `vise-open/clamped` switches, `stock-present` presence and a robot zone.
- Spindles: `milling/SpindleCartridge.hx` (ER20), `Bt30Cartridge.hx`, `SpindleMotor.hx` (1.1 kW,
  Assumed), `Bt30SpindleMotor.hx` (2.2 kW). All milling cartridges; no lathe nose or through-bore.
- Workholding: `milling/PneumaticVise.hx` only. No chuck, collet, mandrel, tailstock or turret.
- Motion: `BallScrew` (SFU1605), `BallNut`, `ScrewSupportUnit`, `LinearRail` (HGR15 only),
  `ServoMotor`, `MotorDriver`, `ShaftEncoder` / `LinearScale` (`EncoderPart`, no index pulse),
  `Gearbox`, `ShaftCoupling`. Transmission: `TimingBelt`, `TimingPulley`, `BeltElasticity`.
- Bearings: `standard/DeepGrooveBearing.hx` only.
- Castings: `structural/MineralCasting`, `MineralFoundation` (+ `CastInsertPlacement`),
  `FoundationIntegrity`; `hobbing/MineralCastBed`, `MineralCastColumn`, `FoundationLayout`.
- Fluid power: `pneumatic/PneumaticCylinder`, `SolenoidValve`, `PneumaticManifold`, `AirSupply`;
  `fluid/CylinderSpec`, `ValveSpec`. No hydraulics.
- `gantry/RotaryHead.hx` is the only rotary-axis precedent (servo plus gearhead).

**Stock (main).**
- StockKit is tri-dexel only (`stockkit/core/include/stockkit.h`, API v3). Tools are surfaces
  of revolution about +Z (`toolpathkit/tool/CutterProfile`, `stockkit/core/src/profile.hpp`
  segments with arcs, `tool.hpp` cutting/shank/holder zones). `CutMotion.Path` only.
- `StockComparison`, deviation colouring, `StockPreviewWorker`.
- `app/MachiningStock` cuts the chord the tool tip travelled each tick and throws unless the
  spindle axis is the stock's +Z.
- `stockkit/docs/DESIGN.md:93,147` and `TODO.md:49` expect turning as new `SweptVolume` kinds in
  the rotating workpiece frame on the same stock. TL-D5 departs from that, with reasons.

**Programs (main).**
- `ToolpathOp`: `Move`, `MachineMove`, `Spindle(direction, rpm)`, `Coolant`, `ToolChange`,
  `Dwell`. Feeds are m/s only; no per-revolution feed, CSS, threading or canned cycles.
  `PathGeometry.Circular` with `ArcPlane.XZ` covers G18 arcs.
- `MachineBinding` requires exactly three distinct axes.
- `camkit` is 2.5D milling only (`CamJob`).
- `cnckit`: LinuxCNC and printer dialects; G95 is explicitly rejected; no G96/G97, G32/G33/G76,
  G70–G72.
- `PlanCheck`: drive torque, compliance, critical speed and an axial cutting-force allowance; no
  spindle power check.

**Tending (main).**
- `examples/machine-tending/`: `TendingCell`, `TendingMission`, `TendingOperation`
  (`OpenDoor, Unclamp, Clamp, Load, StartCycle, Unload, …`), `PartTray`, reach certificates.
  The `machine-tending` branch has more, unmerged.
- `cnckit/controller/RobotInterface*`: the PLC state machine and its channels
  (`robot.request_door_open/close`, `robot.request_clamp/unclamp`, `robot.cycle_start`;
  `cnc.ready`, `cnc.door_open/closed`, `cnc.clamped/unclamped`, `cnc.part_present`,
  `cnc.in_cycle`, `cnc.cycle_complete`, `cnc.alarm`).

**On the hobber branch only (unmerged).**
- Electronic gearbox: `robotkit/runtime/include/electronic_gear.hpp`, `ELECTRONIC_GEAR.md`,
  `GearTerm`, `GearEvents`, MotionKit `GearCheckState`.
- ToolpathKit `FeedPerRevolution(master, axis, distance)`, `ElectronicGear`, `ReleaseGear`,
  `GearBinding`.
- Parts: `AngularContactBearing`, `CrossedRollerBearing`, `RotaryEncoderRing`, `TorqueMotor`,
  `ExpandingMandrel`, `RotaryClamp`.
- Stock: `RingStock` core (`ring.cpp`, `rake.cpp`), `HobbingSweep`, `KienzleMaterial`,
  `RingCompliance`.

**Conventions.** Plan format and working rules as in `examples/flexspline-hobber/PLAN.md`.
Registration:
- `machinekit/tests/src/MachineKitSmoke.hx`, plus the example in `machinekit/tests/haxeon.json`
  `sourceRoots`;
- `app/src/editor/ExampleCatalog.hx`; `ExampleBrowser.categoryOf` ("Machining");
  `app/tests/src/tests/StartPageTests.hx`;
- `app/tests/src/app/ProjectSourceTests.hx` (`PROJECT_SOURCE_ONLY=<id>`, the `mill` group) and
  `app/haxeon.project-source.json` source roots;
- the scene job schema `SceneArtifactMachining` (`projectkit/src/materia/project/SceneArtifact.hx`),
  which assumes a milling job.

## What is missing

**Parts:** a lathe spindle (A2 nose, through-bore, angular-contact sets sized for radial and
thrust load, spindle encoder with index pulse); power chucks with jaws, a drawtube and an
actuating cylinder; an expanding mandrel (the hobber's, once on main); a servo turret with a
curvic coupling and clamp; tool holders and ISO inserts; a touch probe and a tool setter;
size-25 linear rails; a slant bed; an automatic door with interlock; provision interfaces for
later options (TL-D13).

**Control:** feed per revolution and threading as path progress geared to the measured spindle
angle; synchronised thread starts on the spindle index pulse; constant surface speed (spindle
speed from X); turret indexing as a planned move with a clamp state; spindle orientation (C
positioning).

**Simulation:** a stock for a rotating workpiece with micrometre resolution on diameters. The
tri-dexel works for tools moving over a fixed stock at 0.5 mm in the app; it has no rotating-work
sweep. The hobber's ring stock resolves surfaces that cross its circles (tooth flanks) but not
surfaces that run along them, which is every turned diameter.

**Metrology:** dimensional inspection of turned parts (two-point and least-squares diameters,
roundness, coaxiality, wall thickness, face runout, roughness from the profile, thread pitch
diameter) graded against ISO 286 / ISO 1101 tolerances.

**Accuracy:** lathe error sources (spindle runout and thermal growth, X thermal drift, turret
index repeatability, centre-height error, tool and thin-wall deflection, chuck clamping
distortion).

## Decisions

**TL-D1. A 45° slant bed, horizontal spindle, rear turret: axes X, Z and spindle S (C later).**
- Headstock on the left; the X/Z slides ride on the slant face behind and above the spindle
  centreline. The turret sits behind the centreline, so the main cutting force pushes the turret
  into its ways rather than lifting it.
- Chips and coolant fall clear of the ways into a front chip trough.
- Mineral-cast bed on the shared `MineralFoundation` recipes, like the bench mill and the hobber.
- No tailstock in v1: the target parts are short chucked parts (length below about 1.5 × the
  chucked diameter). The bed carries its rail pads and length from v1 (TL-D13).
- **Rejected:**
  - A flat bed: chips land on the ways, and the cutting force acts across the ways instead of
    into them.
  - A vertical turning lathe: built for large, heavy, short parts; Ø90 does not need it.
  - An inverted pick-up spindle: elegant for automation, but it makes the spindle the moving axis
    and departs from every part we already have. A later sister machine if wanted.
  - Sliding-headstock (Swiss) designs: for long slender parts, not cups and rings.

**TL-D2. An 8-station servo turret with a curvic coupling, not a gang plate.**
- A gang plate changes tools by moving X, so the spacing between tools must exceed the part
  diameter. With Ø90 parts and six to eight tools (rough and finish OD, face, deep boring,
  groove, thread, part-off), X travel would grow to about 500 mm.
- The turret: an index servo (with gearhead) turns the disc; a three-piece curvic coupling locks
  it, clamped by a spring-and-piston unit. Stations take 20 mm square shanks (radial and axial)
  and boring bars up to Ø25.
- It is **live-ready** from v1 (TL-D13): drive coupling, drive motor envelope and coolant
  passages to every station, fitted with static holders until live tooling is added.
- Model:
  - the disc is a revolute joint; an index is a planned Ruckig move, with **unclamp → rotate →
    clamp** as timed states, a clamp-confirm sensor and a stated index time;
  - index repeatability (curvic coupling) is an error source on the coupling part;
  - indexing sweeps every mounted tool through a circle, so the program indexes only at a safe
    index position, and CAM and PlanCheck check the swept envelope against chuck, jaws, part and
    door.
- **Rejected:** a gang plate (part size above); a quick-change tool post (manual changes, no
  tending).

**TL-D3. A belt-driven spindle cartridge with its own encoder.**
- A2-5 nose, through-bore ≈ 45 mm, angular-contact sets front and rear (precision class P4),
  servo spindle motor, poly-V belt.
- The encoder sits on the spindle, not the motor, so belt slip and stretch do not reach threading
  or C positioning. It has an index pulse for synchronised thread starts.
- The spindle is a `ProcessVelocityDrive` (continuous revolute, as on the bench mill). C
  positioning (Phase E) switches the same joint to position control through the spindle motor,
  so the spindle drive is chosen position-capable in v1, and the cartridge reserves space for a
  spindle brake or clamp that holds C against live-tool cutting torque (TL-D13).
- **Rejected:** a direct-drive motorised spindle (cost and heat at the spindle for a prototype
  lathe; reconsider if hard-turning accuracy is limited by belt-induced vibration).

**TL-D4. Workholding is a modelled mechanism with clamping force and its side effects.**
- `PowerChuck`: 3 jaws on wedge-hook slides, coupled rigidly to one drawtube joint
  (`AssemblyJointCoupling`, fixed ratio = wedge ratio). Master jaws plus soft or hard top jaws as
  separate parts.
- `ActuatingCylinder`: a pneumatic rotary cylinder at the spindle rear, on the existing
  pneumatic parts (`CylinderSpec`, `SolenoidValve`, manifold, controller valves), with a
  regulated clamp pressure. Clamp force follows from pressure × piston area × wedge ratio,
  reduced by jaw centrifugal force at speed. PlanCheck limits spindle speed so that clamp force
  stays above the cutting need.
- Pneumatic is enough here: the soft roughing cut needs ≈ 13 kN of clamp force with a safety
  factor of 2 (44 N·m at r 35 mm, μ 0.2), and thin walls want low force anyway. Hydraulics would
  be a new fluid domain. Revisit only if the clamp check fails for a real part.
- Jaw stroke and clamp/unclamp sensors give the tending interface its signals.
- Clamping distorts thin walls (a three-lobe shape after unclamping). This is a named error
  source on the chuck (TL-D9). The flexspline's thin wall is finished on an **expanding
  mandrel** in its bore, the same part as the hobber's (HB5), which also supports the wall
  against the cutting force.
- **Rejected:** a fixed "part is held" constraint (clamp force, speed limits and distortion
  would be invisible).

**TL-D5. Turned parts are cut on a section stock: meridian sections with 2D dexels.**
- `SectionStock` (StockKit core, C++): N meridian half-planes at angles θₖ fixed in the work
  frame. Each section is a 2D bi-dexel: rays along r at fixed z, and rays along z at fixed r,
  each a sorted list of material intervals with exact endpoints and provenance (which tool, which
  pass).
- Removal is event-based. A turning tool's cutting edge lies in the plane through the work axis
  (on centre, rake face in that plane). Each section loses the insert's profile region at the
  exact instant the edge crosses θₖ in the work frame. Joint states are interpolated between
  control ticks, as in the hobber (HB-D4).
- With N = 1 this already shows feed marks (cusps of height ≈ f²/(8·rε)) and the true thread
  profile. With N = 32–128 it shows roundness, lobing, runout and non-round turning (X geared to
  C), because each section sees the tool at a different instant.
- Exact endpoints come from intersecting rays with the insert profile (lines and arcs), the same
  segment form as StockKit's tool half-profiles (`profile.hpp`).
- A display mesh is lofted around the sections (revolved for N = 1).
- **Rejected:**
  - Turning as new `SweptVolume` kinds on the tri-dexel (what `stockkit/docs/DESIGN.md`
    anticipated). In the rotating frame each tool sweep is a helix, and micrometre diameters
    would need micrometre ray spacing in 3D: the hobber measured 244 MB for 200 × 200 mm at
    0.1 mm. TL2 updates DESIGN.md and TODO.md to say so.
  - The hobber's ring stock: a turned diameter runs along its circles, so a diameter resolves
    only to the radial level spacing.
  - Exact 2D polygon booleans with arcs: no robust library in the tree, and the dexel form is
    deterministic and matches the rest of StockKit.
  - Removing the swept envelope of the tool path: smooth by construction, so it hides feed marks,
    threads and every rotation-dependent error.

**TL-D6. Feed per revolution and threading are path progress geared to the measured spindle.**
- On a real lathe, `G95` feeds and `G32/G76` threads take the axis motion from the spindle
  encoder. Here one relation covers both: the path parameter `s` of a feed move follows
  `ds = f · dθ̂_S`, where `θ̂_S` is the spindle's measured angle extrapolated over the stated
  loop latency (the hobber's HB7 gearbox, with the path parameter as the follower instead of a
  joint).
- The hobber's `FeedPerRevolution(master, axis, distance)` gears one axis. A turning feed moves
  X and Z together along a line or XZ arc, so TL7 generalises it to a path follower rather than
  adding a second mechanism.
- Threading adds a synchronised start: each pass waits for the index pulse plus a stated angular
  offset, so every pass lands in the same groove. Multi-start threads shift the offset by
  360°/starts.
- Constant surface speed: the spindle speed command is `min(vc / (π·d(X)), S_max)`. Because the
  feed is geared to the measured spindle, the axes follow whatever the spindle actually does.
- Rapid moves and positioning are ordinary planned moves (Ruckig / TOPP-RA).
- PlanCheck checks that `f · S_max` and the thread run-out deceleration fit the X/Z limits, and
  that spindle acceleration under CSS stays within what the axes can follow.
- Sync error (path position against `f·θ_S(true)`) is recorded per tick.
- **Rejected:**
  - Converting mm/rev to mm/min at a nominal spindle speed: wrong under CSS and whenever the
    spindle sags under load, and it cannot thread.
  - A rigid joint coupling between spindle and Z: sync error invisible, and the ratio fixed at
    compile time.

**TL-D7. Turning programs are machine-axis programs; CAM works on the revolved profile.**
- ToolpathKit ops:
  - `MachineMove` for X/Z (and turret, C);
  - `Spindle` with a CSS mode (`vc`, `S_max`);
  - per-revolution feeds on X/Z moves;
  - `ThreadPass(pitch, starts, startAngle, infeed)`;
  - `TurretIndex(station)`, `Chuck(clamp|release, pressure)`, `Door(open|close)`, `Coolant`;
  - `ProbeDiameter`, `ProbeFace`, `SetTool(station)`: measuring cycles whose results update work
    and tool offsets (TL-D13).
- CAM: `camkit` `TurningJob(target, stock, toolTable, machine, setups)`.
  - The target is a revolved part: an axis plus a meridian profile, taken from a CadKit solid of
    revolution or stated directly.
  - Features: face; centre drilling and on-centre drilling (peck cycles, chip breaking); OD and
    ID roughing (constant depth, finishing allowance, like `G71`); finishing with nose-radius
    compensation; grooving; threading (infeed per pass, flank infeed option, spring passes);
    chamfers; part-off.
  - The stock (solid bar, hollow bar, later forgings) is an input, checked by TL-D12.
  - Setups: Op10, Op20 … each with its workholding (chuck jaws, soft-jaw bore, mandrel). Heat
    treatment between setups is off-machine: a stock state change (hardness, stated distortion).
- Binding: `MachineBinding` requires three axes. A `LatheBinding` (X, Z, spindle, turret,
  optional C) binds turning programs; X is programmed in diameter at the IR boundary only if
  the G-code dialect needs it, and radius everywhere inside.
- G-code (a Fanuc-style lathe dialect with `G71/G70/G76/G96/G95`) is later. The IR is what the
  machine and the sim consume.

**TL-D8. Tools are ISO inserts in holders, with offsets and nose direction.**
- They live in `toolpathkit/tool/` next to `CutterProfile` (`TurningInsert`, `TurningHolder`,
  `TurningToolTable`), since CAM, the stock and the machine all read them. `CutterProfile`
  stays the rotating-cutter type; inserts are not surfaces of revolution.
- An insert from its ISO 1832 code: shape (C 80°, D 55°, V 35°, W trigon, grooving, threading
  60°), inscribed circle, thickness, nose radius rε.
- A holder from its approach angle and hand, giving the insert's pose and the holder's
  non-cutting body. The cutting region is the insert profile in the meridian plane. The holder
  and the insert's non-cutting faces are collision bodies: any contact with stock is a crash,
  reported with the tool and the program line.
- Drills (twist and indexable) sit stationary on axial stations, on centre. Their lips lie in
  a plane through the axis, so the section stock removes the drill's point profile at each lip
  crossing, as for an insert. The drill body is a collision body like a holder.
- A tool table maps turret stations to holder, insert or drill, X/Z geometry offsets, nose
  radius and tip direction (as a Fanuc tool table does).
- Centre-height error of a station is an error source on the turret station (TL-D9).

**TL-D9. Errors are named sources with a physical home and a ValueBasis (as HB-D7).**
- Each source sits on the part that causes it:
  - spindle radial and axial runout, non-repeatable runout on the spindle bearings;
  - spindle thermal growth (Z and centre-height drift) on the cartridge;
  - X ball-screw thermal growth on the screw (open-loop X without scales);
  - turret index repeatability on the curvic coupling; centre-height error per station;
  - tool deflection under cutting force on the holder;
  - thin-wall deflection on the workpiece, from its section geometry and support (chuck,
    mandrel, unsupported);
  - chuck clamping distortion (lobing after release) on the chuck;
  - servo loop rate and latency on the driver and controller (feeds, threads, non-round X).
- The **error budget** runs the same job with each source alone and with all together, and names
  the dominant part per inspection characteristic.
- When SimKit fidelity levels reach main, these sources map onto the Physical level. Until then
  the turning sim owns them.

**TL-D10. Tending-ready in v1; the cell later.**
- The machine exposes the existing `RobotInterface` channels unchanged (`request_door_open/close`,
  `request_clamp/unclamp`, `cycle_start`; `door_open/closed`, `clamped/unclamped`,
  `part_present`, `in_cycle`, `cycle_complete`, `alarm`), wired as on `EnclosedBenchMill`, so
  `TendingMission` drives a lathe and a mill the same way.
- A loading envelope is part of the machine description: the door opening, the chuck face
  approach along the spindle axis and the turret park position.
- The cell (Phase D) reuses the machine-tending arm, gripper, tray and mission code against this
  interface instead of building a new cell.

**TL-D11. Generic names in code.** No trade names in identifiers or catalogue rows; vendors
appear only as sources in docs. Standards (ISO 1832, ISO 286, ISO 1101, A2 spindle noses) are
fine.

**TL-D12. Stock is chosen by one rule: its bore must fit inside the part's narrowest
through-bore.**
- Hollow stock can only be used where the finished part has no material inside the stock's
  bore at any axial position. So the stock ID must be at most the part's smallest through-bore
  along the whole axis, minus a stated machining allowance. A closed-back cup's limit is its hub
  bore; a ring's or a hat's is its main bore.
- `TurningJob` checks this before planning and rejects infeasible stock, naming the feature that
  would be missing (for example "hub: stock bore Ø40 exceeds hub bore Ø12 − 1 mm allowance").
- Stock kinds: solid bar, hollow bar (OD, ID, and the eccentricity and straightness of drawn
  tube as stated values), later forgings (a near-net revolved profile).
- When a feasible hollow stock exists, CAM prefers it and reports removed volume and roughing
  time for each candidate.
- **Rejected:** assuming hollow stock to save drilling. It is a part-geometry question; a cup
  from a tube would lose its diaphragm and hub.

**TL-D13. A platform with provisions: options attach later, variants are separate machines.**
- The mineral-cast bed cannot gain pads, inserts, rails or channels after casting, so whatever
  the bed or the spindle must carry later is designed in now. This follows machine builders'
  practice: one platform sold with options, and separate models for the large changes.
- **Fitted in v1:** a turret-mounted touch probe and a tool-setter arm. They decide whether
  micrometre tolerances hold over a batch: probing cycles update work and tool offsets against
  thermal drift and insert wear (TL8, TL11).
- **Provisioned in v1, fitted later:**

  | Option | Provision in the base design |
  |---|---|
  | Live tooling | live-ready turret (TL-D2): drive coupling, motor envelope, power and axis in the cabinet |
  | C axis | position-capable spindle drive; spindle encoder (already); brake or clamp space on the cartridge |
  | Tailstock, steady rest | bed length past the turning length; rail pads and cast-in inserts on the slant face; envelope and lines for a quill cylinder |
  | Bar feeder | through-bore (already), hollow chuck and cylinder (already), spindle liner, rear clearance, floor alignment points |
  | Linear scales X/Z | machined mounting faces and covers along both slides |
  | High-pressure coolant | through-turret passages (TL-D2), sump and filter space |
  | Fast tool servo | a turret station mount and an amplifier slot in the cabinet |
  | Chip conveyor, part catcher | a standard chip-trough opening; a catcher envelope below the part-off position |

- The control cabinet is sized for 7 axes (X, Z, turret, S/C, live drive, tailstock or steady,
  one spare). `CncController` has 4 today.
- Each provision is a real interface in the model (a cast-in insert, a mounting face, a reserved
  envelope, a spare axis or slot) with an attach check, so an option is added without changing a
  base part.
- **Variants, not options:** a Y axis (a wedge slide stack replacing the cross slide), a
  sub-spindle (second headstock on its own Z, longer bed, bigger control) and turn-grinding
  (grinding spindle, dressing, grit sealing). Each would be a sibling machine sharing the base
  parts and bed recipe, planned separately when wanted. The cross-slide-to-bed interface is a
  defined part so that a Y variant can replace it.
- **Rejected:**
  - Designing every option in from the start: cost and complexity for parts we do not make yet.
  - Designing none: live tooling, C and the tailstock are the most valuable extensions, and
    without provisions each would mean a new turret, spindle drive or bed.

## Test parts

- **Coarse demo shaft:** 1045 steel, from Ø45 × 100 bar stock: faces, Ø40 / Ø30 / Ø24 steps,
  two chamfers, a 3 mm groove, an M24 × 1.5 thread, part-off. Fast; used for interactive runs and
  smoke checks.
- **Reference flexspline blank:** to match the hobber's reference flexspline (m 0.3, z 200,
  d 60 mm, face 8 mm, wall 0.6 mm, 40CrNiMo 38–42 HRC). Cup, diaphragm, and a hub of Ø30 OD with
  a Ø12 centre bore (assumed). The closed back means its narrowest through-bore is Ø12, so it
  starts from **solid bar** (TL-D12):
  - Op10/Op20 soft: from a Ø70 × 55 slug sawn from bar; centre-drill and drill, rough the cup
    interior, then OD, bore and hub with 0.5 mm per side left. About 90 % of the slug becomes
    chips (≈ 1.66 kg slug, ≈ 0.16 kg part), most of it from the interior;
  - (off-machine: harden and temper, stated distortion);
  - Op30 hard: bore to pre-hone size and faces, OD held in soft jaws while the wall is still
    thick;
  - Op40 hard: on an expanding mandrel in the bore, OD to the hob blank diameter, giving the
    0.6 mm wall.
  - The acceptance part; the finished blank is handed to the hobber as its `GearSpec` blank.
- **Circular-spline ring blank:** Ø85 OD, ≈ Ø60 bore, 15 mm wide, two setups, soft only, from
  **hollow bar** (≈ Ø90 × Ø55): no drilling, the bore is only opened to size.
- **Variants, chosen by the strain-wave design, not by the lathe:**
  - a hollow-shaft cup (cables through the hub): a larger hub (≈ Ø45 OD for a Ø25 bore) and a
    changed diaphragm. Hollow bar only if a section with an ID below the hub bore exists for
    ≈ Ø70 OD (not checked);
  - a hat-type flexspline (diaphragm flaring out to a flange): open through the tube bore, so
    hollow bar fits naturally, but the stock must cover the flange OD.
- **Later:** the wave-generator cam (Phase E): a non-round two-lobe profile, about 3 µm profile
  tolerance.

## Reference numbers (assumed until checked against a datasheet or a measurement)

| Quantity | Value | Source |
|---|---|---|
| Swing over bed / max turning Ø | Ø250 / Ø150 mm | assumed, sized for Ø90 parts plus jaws |
| Travel X / Z | 130 / 250 mm | assumed (X: max radius plus past centre for facing and boring) |
| Bed | 45° slant, mineral cast | assumed |
| Spindle | A2-5, bore 45 mm, 4500 rpm max, 5.5 kW S1 servo, poly-V belt | assumed |
| Spindle bearings | angular-contact sets, P4; radial runout ≤ 2 µm | assumed |
| Spindle encoder | 4096 lines with index pulse, on the spindle | assumed |
| Chuck | Ø165 hollow 3-jaw power chuck, pneumatic rotary cylinder, ≈ 20 kN at 6 bar | assumed |
| Clamp need, soft roughing | ≈ 13 kN (Fc 1.25 kN at r 35 mm, μ 0.2, safety 2) | derived |
| Turret | 8 stations, 20 mm shanks, boring bars ≤ Ø25, index 0.3 s adjacent | assumed |
| Turret index repeatability | ± 1.5″ (≈ 0.7 µm at 100 mm tool radius) | assumed, curvic coupling |
| Axes | size-25 rails, Ø25 × 10 ball screws, 750 W servos, 20 m/min rapid | assumed |
| Position loop / gear rate | 4 kHz, latency one loop period | as hobber HB7 |
| Bed length for the tailstock provision | turning length + ≈ 300 mm (tailstock body and quill stroke) | assumed |
| Control cabinet | 7 axes, I/O for probe, setter, chuck, door, tailstock | assumed (TL-D13) |
| Probe / tool setter | probe repeatability ≤ 1 µm (2σ); setter ≤ 2 µm | assumed |
| Roughing (soft, 40CrNiMo) | vc 200 m/min, f 0.25 mm/rev, ap 2 mm | assumed |
| Roughing power | kc ≈ 2500 N/mm² → Fc ≈ 1.25 kN → ≈ 4.2 kW at the cut | derived |
| Hard finishing (38–42 HRC) | vc 150 m/min, f 0.05 mm/rev, ap 0.1 mm, rε 0.4 mm | assumed |
| Hard finishing rpm at Ø60 | ≈ 800 rpm | derived |
| Theoretical roughness | Rth = f²/(8·rε) ≈ 0.8 µm (Ra ≈ 0.2 µm) | standard estimate, check |
| Hard finishing force | Fc ≈ 20 N, passive force of the same order | derived, kc assumed |
| Thin-wall finish tolerances | wall 0.6 mm ± 10 µm, variation ≤ 5 µm, OD runout to bore ≤ 5 µm, roundness ≤ 3 µm | assumed from the hobber's datum needs; check |
| Bore (pre-hone) | IT6, Ra ≤ 0.8 µm, ≈ 10 µm honing allowance | assumed |

The thin wall drives the design. A rough ring-bending estimate puts the deflection of an
unsupported 0.6 mm wall under a 20 N point load on the order of a millimetre, so the wall cannot
be finished unsupported. Op40 must run on the mandrel, and TL10 must show both cases.

## Reference machines

Checked 2026-10-19 from brochures and dealer listings, not datasheets. Prices are rough.

| Machine | Relevance | Notes | Price |
|---|---|---|---|
| Hardinge T42 Super-Precision | closest: slant-bed turret lathe for precise small parts, soft and hard turning | A2-5, 6000 rpm, 42 mm bar, 11 kW (T42SP), 12 stations, polymer-concrete-reinforced cast-iron base; maker claims 3 µm diameter deviation after warm-up | new by quote, ≈ $250–400k estimated |
| Haas ST-10 | common job-shop equivalent | A2-5, 6.5" chuck, 44 mm bar, 12 stations, 15 hp, ≈ Ø356 mm max cut | new ≈ $45–65k |
| EMAG VL 2 | the volume route for these parts (inverted pick-up, rejected in TL-D1) | 160 mm chuck, Ø100 × 150 mm parts, 6000 rpm, polymer-concrete base | used €27–62k asking; new by quote, ≈ €250–400k estimated |

Our machine is roughly a simplified T42 for prototypes: half the spindle power, no sub-spindle or
Y, fewer stations, a smaller envelope, similar precision targets.

## Steps

### Phase A: tools, section stock, ideal turning, inspection (no machine, can start now)

**TL0. Baseline.**
- Run the gate on the starting commit and record failures that come from main, not from us.
- Tests: the gate's numbers recorded in Progress.

**TL1. Inserts, holders and the tool table (TL-D8).**
- Insert geometry from ISO 1832 fields; holder geometry from approach angle and hand; cutting
  region and non-cutting bodies in the meridian plane; `TurningToolTable` with offsets and tip
  direction.
- Tests:
  - every insert shape's profile closes, and its nose arc is tangent to both edges;
  - an 80° insert at 95° approach has the expected end and side cutting-edge angles;
  - tip-direction codes match the virtual tool tip for each holder hand;
  - a threading insert's flanks are 60° to 1e-9 rad;
  - a 118° twist drill's point profile and an indexable drill's insert pair give the right
    cutting diameter and point length.

**TL2. Section stock (TL-D5).**
- StockKit core: `sk_section_stock` with N sections, bi-dexel rays per section, exact endpoints
  and provenance, snapshots, deterministic multithreading as in the tri-dexel core.
- Haxe: `SectionStock` (`cylinder`, `tube`, `fromProfile`), `removeRegion(section, region,
  provenance)`, `contour`, display mesh, `compare` against a target revolved profile.
- Update `stockkit/docs/DESIGN.md` and `TODO.md` (turning now has its own stock, TL-D5).
- Tests:
  - an analytic step cut gives exact endpoints on every ray it crosses;
  - memory and time budgets: the reference blank at 2 µm with N = 1 and at 10 µm with N = 64,
    each below 100 MB; a full snapshot below 0.5 s;
  - snapshot and restore are identical; bit-identical results across thread counts.

**TL3. Ideal turning (TL-D5).**
- `TurningSweep` takes the tools, the machine's tool-to-work frame chain (X, Z, turret angle,
  spindle angle as functions of time) and removes each insert's region from each section at its
  crossing instants. Geometric error sources (TL-D9) enter as frame perturbations, zero for now.
- A pure-kinematics driver with ideal gearing turns the test parts without a machine.
- Tests:
  - a facing pass and a longitudinal pass give exact diameters and lengths;
  - the cusp height matches `f²/(8·rε)` (oracle, cite it), and halving f quarters it;
  - an ideal M24 × 1.5 thread has the threading insert's profile at the right pitch on every
    section, each section shifted by `pitch·θₖ/2π`;
  - a pass started at the wrong spindle angle cuts a second groove (the cross-threading case);
  - a holder that touches stock is reported as a crash with tool and time.

**TL4. Turned-part inspection.**
- `TurnedPartInspection.of(sectionStock, drawing)`: two-point and least-squares diameters,
  roundness, cylindricity, coaxiality, wall thickness (min, max, variation), face runout,
  lengths, Ra/Rz from the profile, thread pitch diameter, lead and flank angles.
- A drawing format: nominal revolved profile plus ISO 286 / ISO 1101 tolerances per feature.
- A report object the app and the tests share.
- Tests:
  - a perfect synthetic part passes with zero deviation;
  - injected eccentricity e gives coaxiality 2e and face runout as expected;
  - a three-lobe part shows its roundness, while its two-point diameters stay constant (the
    classic trap the report must not fall into);
  - the ideal TL3 shaft passes its drawing.

### Phase B: the machine and its control

**TL5. Parts (TL-D2, TL-D3, TL-D4).**
- `machinekit.turning`:
  - `LatheSpindle`: cartridge, A2-5 nose, bearing sets, belt, spindle encoder with index pulse;
  - `PowerChuck` + `ActuatingCylinder` + `ChuckJaw` (master, soft, hard);
  - `ServoTurret`: disc, curvic coupling, clamp unit, index drive, stations, live-drive
    provision and coolant passages; `ToolHolder` parts from TL1;
  - `TouchProbe` (turret station) and `ToolSetter` (swing arm by the chuck);
  - `SlantBed`, `Headstock`, `CrossSlide`, `ChipTrough`, `LatheDoor`;
  - provision interfaces (TL-D13): tailstock rail pads and inserts, spindle brake space,
    scale faces, bar-feeder clearance, cabinet slots.
- `machinekit.motion`: size-25 rail rows; bearing rows as needed (shared with the hobber's
  `AngularContactBearing` once on main).
- Catalogue rows are marked Catalog or Assumed. Error-source fields default to the reference
  numbers.
- Tests: part geometry valid; BOM rows; clamp force from pressure; chuck speed limit from jaw
  mass; turret index time from the index drive's limits.

**TL6. The machine.**
- `SlantBedLathe extends MachineAssembly`:
  - joints X, Z (prismatic, ball screws), S (continuous, process velocity), turret (revolute,
    clamped), drawtube with three coupled jaws, door (prismatic, cylinder);
  - tool-to-work frame chain exported for TL3; loading envelope and interface signals (TL-D10).
- `SlantBedLatheChecks` in MachineKit smoke:
  - FK of each station's tool tip at the travel corners;
  - every tool reaches the largest and smallest target diameters and the chuck face;
  - the indexing envelope clears chuck, jaws and door from the safe index position;
  - the probe reaches the target diameters and faces; the setter arm clears the chuck when
    stowed;
  - every TL-D13 provision exists as an interface, and a stand-in option (tailstock envelope,
    live drive envelope, scale, spindle brake) attaches without interference or a change to a
    base part;
  - derived axis limits; BOM.
- `materia.project.json`, a Start-page entry in the machining category, `StartPageTests`, and
  `ProjectSourceTests.checkLathe` (loads, MuJoCo holds pose, chuck clamps).
- Tests: smoke checks; app check.

**TL7. Spindle-geared feeds, threading and CSS (TL-D6).**
- RobotKit runtime: a path-progress follower on the measured spindle angle, extending the
  hobber's measured-master gearbox (HB7) from a joint follower to the active path's parameter;
  synchronised starts on the index pulse; CSS spindle commands from X. `EncoderPart` gains an
  index pulse.
- ToolpathKit: per-revolution feeds on `Move` (X/Z path), `ThreadPass`, CSS on `Spindle`;
  `LatheBinding` (TL-D7).
- MotionKit/PlanCheck: per-revolution feed limits, thread run-out deceleration, CSS acceleration.
- Tests:
  - a load step that drops spindle speed 20 % keeps the feed per revolution within 1 %;
  - a 10-pass thread has all passes in one groove; with the start sync removed, it does not;
  - CSS facing keeps vc within 1 % until `S_max`, then holds `S_max`;
  - a pitch too large for Z at the programmed speed is rejected by PlanCheck, naming Z.

**TL8. Turning CAM (TL-D7).**
- `camkit` `TurningJob`: the TL-D12 stock check; features from the revolved target and the
  stock; drilling, roughing, finishing with nose-radius compensation, boring, grooving,
  threading, part-off; setups with workholding; turret indexes at the safe position; feeds and
  speeds from the tool table and material.
- Probing: tool setting at the start of a batch; an in-process diameter probe before the finish
  pass of each critical feature, correcting the finishing offset (measure, then cut).
- Tests:
  - the demo shaft program removes all material down to the finishing allowance, then to size;
  - the flexspline blank from solid bar drills before the first boring pass, and the drill
    diameter admits the smallest boring bar in the table;
  - hollow bar for the cup is rejected, naming the hub; hollow bar for the ring is accepted and
    plans no drilling;
  - removed volume per candidate stock matches the stock minus target volume to 0.1 %;
  - nose-radius compensation puts a 45° chamfer on size with rε 0.8;
  - no rapid crosses stock; every index happens at the safe position;
  - a feature no tool in the table can reach (a bore deeper than the longest bar) is rejected,
    naming the feature.

**TL9. Turning in the app.**
- `app`: a `TurningStock` (like `MachiningStock`) runs the TL3 sweep from the simulated joint
  states each tick, crossings at exact times. Section-stock preview on the worker. An inspection
  panel after the cut, showing TL4 results against the drawing.
- `ProjectSourceTests.checkLathe` turns the demo shaft end to end: inspection pass, cycle time,
  sync error, no crashes, an allocation budget.
- Tests: app check; the ideal machine (no error sources) gives the TL3 result to 0.1 µm.

### Phase C: forces, error sources, acceptance

**TL10. Cutting forces and deflection.**
- Per crossing, chip section from the removed region × Kienzle specific force (kc1.1, mc, by
  material and hardness, assumed and marked), giving cutting, feed and passive forces.
- Outputs: spindle torque and power, axis loads, tool deflection, thin-wall deflection from the
  workpiece's section and its support. Reuse the hobber's `KienzleMaterial` and
  `RingCompliance` once on main.
- PlanCheck gains a spindle power and torque check from these forces (it has none today), and the
  chuck clamp-force check (TL-D4).
- Tests:
  - force scales with f^(1−mc);
  - soft roughing of the flexspline blank stays below spindle power with the stated margin;
  - Op40 unsupported deflects the wall far outside tolerance; on the mandrel it stays within it.

**TL11. Error sources and budget (TL-D9).**
- Turn on each source from its part; `ErrorBudget` per source and all together, contributions per
  inspection characteristic and the dominant part.
- Tests (each source alone shows its signature):
  - spindle radial runout → roundness and coaxiality at once-per-revolution;
  - centre-height error h → diameter error ≈ h²/r (oracle);
  - spindle thermal growth over a batch → length and diameter drift without warm-up;
  - chuck clamping distortion → three-lobe roundness after release, invisible to two-point
    diameters;
  - turret index repeatability → diameter scatter across tool changes;
  - sync latency without extrapolation → thread lead error.
- Probe compensation: a batch of 20 parts from a cold start drifts out of the bore tolerance
  without probing; with the pre-finish probe cycle the drift stays within probe uncertainty plus
  the remaining sources.

**TL12. Acceptance.**
- The reference flexspline blank, over Op10–Op40 with the reference parts, meets its drawing:
  wall, wall variation, OD runout to bore, roundness, bore size and finish. The report shows the
  budget.
- A cheaper specification (P5 spindle bearings, motor-mounted spindle encoder, no warm-up,
  finishing in chuck jaws) fails, and the report names the causes in the right order.
- The finished blank is exported as the hobber's blank (bore datum and runout as measured), so
  the two machines run as a chain.
- The circular-spline ring blank passes its (looser) drawing.
- A short report of the numbers in this plan, with every assumed value listed.

### Phase D: tending cell

**TL13. Lathe tending.**
- A tending cell variant with the lathe instead of the mill: reuse the machine-tending arm,
  gripper, tray and mission code against the TL-D10 interface. The arm loads along the spindle
  axis through the door; the chuck clamps on a confirmed grip; the door interlock gates motion.
- Tests: the reach certificate at the chuck station; a full load–cycle–unload in the app with
  no collisions and the interface signals in the right order; a dropped-part or no-clamp case
  stops the cell.

### Phase E: non-round turning (wave-generator cam)

**TL14. C positioning and X geared to C.**
- Spindle orientation and C-axis position control through the spindle motor; X following a
  profile `X(θ_C)` through the same measured-master gearbox; section stock at N ≥ 64.
- Tests: a two-lobe profile at the stated speed tracks within the X loop's bandwidth; the
  tracking error appears in the section stock as profile error with the right phase.

**TL15. Wave-generator cam acceptance.**
- The cam profile (from the research notes' wave-generator geometry) turned to its 3 µm profile
  tolerance, or the report names what limits it (X bandwidth, latency, spindle runout) and the
  upgrade that would fix it (for example a fast tool servo or a linear-motor X).

### Phase F: options on the provisions (TL-D13)

Each option gets its own numbered steps when started. Recommended order, with what each adds:

1. **Live tooling + C axis.** Finishes a part in one setup: drilled and tapped bolt circles,
   cross holes, flats, keyways, hex. No second setup on the mill, so no concentricity is lost
   between machines.
   - Unlocks: flexspline hub holes, circular-spline mounting holes, joint flanges, shafts with
     flats or keyways, pulleys with set-screw holes.
   - Needs milling ops on a rotating-axis binding, and N sections or a local tri-dexel for the
     non-axisymmetric features.
2. **Tailstock + steady rest.** Long, slender parts.
   - Unlocks: ball-screw end journals and locknut threads on bought screws, motor and spindle
     shafts, rollers.
   - Needs a deflection check for slender work (TL10).
3. **Light power skiving** on a driven, tilted station. Internal teeth geared to C, reusing the
   hobber's gearbox and ring stock.
   - Unlocks: circular-spline teeth in the same setup as its bore, internal splines.
   - Limited by the driven station's power and stiffness; heavy skiving is a mill-turn variant.
4. **Fast tool servo.** When the ball-screw X cannot follow non-round profiles fast enough
   (TL15 decides).
   - Unlocks: wave-generator cams at production speed, polygon profiles.
5. **Linear scales, high-pressure coolant.** When the error budget or cycle time asks for them.
6. **Thread whirling** on a driven station, with the tailstock and steady.
   - Unlocks: our own lead screws and ball screws (pre-grind or rolled class).
7. **Bar feeder, part catcher, chip conveyor.** Unattended production of small parts from bar.
   Robot tending (TL13) covers chucked parts until then.

**Variants (separate machines on the shared parts):** Y axis; sub-spindle; turn-grinding (for
the flexspline bore datum and the cam's final profile, if hard turning plus honing cannot reach
them).

With options 1–3 and Phase E, the lathe makes every turned and toothed part of a strain-wave
gear except the flexspline teeth (the hobber) and the flexible bearing.

### Phase G: later
- G-code: a Fanuc-style lathe dialect in CncKit (writer, then reader).
- Insert wear and tool-life; chip formation.
- Forged near-net cup blanks (backward-extruded), as a stock kind for volume production.
- Buildable output: drawings, sourcing and a controller choice.

## Order

```
TL0 ── TL1 ──┬── TL3 ── TL4 ───────────────┐
TL2 ─────────┘                              │
TL5 ── TL6 ── TL7 ── TL8 ──────────────── TL9 ── TL10 ── TL11 ── TL12 ── TL13
                                                                  └──── TL14 ── TL15
```

- Phase A (TL1–TL4) needs no machine and can run in parallel with TL5–TL6.
- The hobber branch is the main dependency. TL7 needs its gearbox and `FeedPerRevolution`
  (HB7/HB8); TL5 its `AngularContactBearing` and `ExpandingMandrel`; TL10 its `KienzleMaterial`
  and `RingCompliance`. Until the branch lands on local `main`, TL7 is `blocked: hobber on main`,
  TL8 runs against ideal gearing, and Op40 uses a stated mandrel envelope.
- TL6 adds a turning job to `SceneArtifactMachining` (or a sibling schema) for the app.
- TL13 needs the machine-tending work on local `main`.

## Working rules

- Work in a worktree `lathe` from local `main`. Dependencies count only once they are on local
  `main`. Merge local `main` in to pick one up; never merge another session's live branch.
- A blocked step is marked `blocked: <what>` in Progress, and work goes on to the next unblocked
  step.
- Commit per step, e.g. `Lathe: section stock (TL2)`, with Progress updated in the same commit.
- Diagnose before working around. Numbers not from a datasheet or a measurement are marked
  assumed.
- No legacy compatibility: one schema version, reject others.
- Heavy suites go through the heavy-run wrapper.

## Progress

| Step | State | Commits |
|---|---|---|
| TL0 | planned | |
| TL1 | planned | |
| TL2 | planned | |
| TL3 | planned | |
| TL4 | planned | |
| TL5 | planned | |
| TL6 | planned | |
| TL7 | planned | |
| TL8 | planned | |
| TL9 | planned | |
| TL10 | planned | |
| TL11 | planned | |
| TL12 | planned | |
| TL13 | planned | |
| TL14 | planned | |
| TL15 | planned | |
| Phase F | options, after TL12 | |
| Phase G | later | |
