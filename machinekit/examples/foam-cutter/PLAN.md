# Hot-wire foam cutter plan

**Goal.** A 4-axis hot-wire foam cutter that cuts a foam block in the editor as the machine moves,
from a CAD part, using the signals a real machine would use.

1. A part is cut from a foam block with a heated wire. The wire runs between two towers. Each tower
   moves one wire end in its own plane: the left tower on axes X/Y, the right one on U/V. Moving
   the ends differently makes ruled, tapered cuts such as wing panels.
2. CAM turns a CadKit solid (or two paired profiles) into a wire program. It pairs the two
   profiles, checks that the part really is ruled between the foam faces, and compensates the
   kerf on each side.
3. The machine runs the program through the existing MotionKit/RobotKit stack. The simulation
   removes the foam swept by the wire in real time. Kerf comes from wire power and local wire
   speed. Wire lag comes from drag on a tensioned string. Severed offcuts fall away.
4. G-code goes out to the controllers foam cutters actually use (LinuxCNC `xyuv`, grblHAL/FluidNC)
   and comes back in with each controller's real feed semantics.

Inspiration: the FreeCAD **FoamCut** workbench (github.com/Shkolik/Foamcut, LGPL-2.1+, active, v0.2.0
2026-08). We take ideas only, not code. It has the right CAM model but no material simulation. What
we take from it:
- paired points on the part faces, extrapolated to the tower planes;
- kerf applied at the part faces, scaled on the slow side;
- G93 inverse-time output;
- an indexed turntable only between an exit and an entry.

The other prior art (Jedicut, GMFC, WiHoWi, Hot_wire_QT, LinuxCNC `axis_foam.ini`, Marlin
`FOAMCUTTER_XYUV`) adds:
- the two-point heat/speed material calibration;
- user sync points for profiles that do not pair one to one;
- per-controller feed semantics.

The work spans kits:
- `machinekit/haxe/src/machinekit/foam/`: wire, wire guide, tension spring, wire supply, foam table
  parts;
- `machinekit/examples/foam-cutter/`: the `FoamCutter` assembly, checks, preview, project;
- `motionkit`: `WireKinematics` and the ruled wire path primitive;
- `toolpathkit`: ruled moves and wire setups;
- `stockkit`: the swept wire slab and severed pieces;
- `camkit/haxe/camkit/foam/`: `FoamJob`, pairing, ruledness check, kerf, the hot-wire process model;
- `cnckit`: the hot-wire dialect and posts;
- `robotkit` device and virtual heaters: the wire power mode and wire plant;
- `app`: player, real-time stock, wire view.

## What exists

Surveyed 2026-10-13 on local main b3405449e and on branch `3d-printing` 35f49ac33. Updated
2026-10-14 on main b190f9126 (= origin/main), which now includes the whole 3D printing work
(AM0–AM12, AM5b and the review fixes). Everything below is on main. The printer plan's review
follow-ups RF1–RF11 (`machinekit/examples/printer/PLAN.md`) are planned, not done; RF5 matters
here (see FC3).

**Machines.**
- `AxisBuilder` (`machinekit/haxe/src/machinekit/assembly/AxisBuilder.hx`) provides the building
  blocks: `place/attach/hang/slide/driveScrew/mountNut/turnWithBelt/twoPulleyAxis/addMotor`.
  `CoreXyPlotter` and `CncRouter` are the closest examples.
- `Gantry`/`RotaryHead` (`machinekit/haxe/src/machinekit/gantry/`) is the only rotary-axis
  precedent. There is no rotary table part.
- Homing switches and `HomingCycle` come from gantries G10–G13.
- Checks classes are registered in `machinekit/tests/src/MachineKitSmoke.hx`.
- Start-page entries go in `app/src/editor/ExampleCatalog.hx`, plus `ExampleBrowser.categoryOf`.
- App tests are in `app/tests/src/app/ProjectSourceTests.hx` (`PROJECT_SOURCE_ONLY=<id>`).
- `AssemblyIntegrity` (MK10) is planned only.

**Toolpath IR and motion.**
- `ToolpathOp.Move(kind, PathGeometry, feed, tol, provenance)`. `PathGeometry` is
  `Line/Arc/Circular` of `Point3`.
- There is no 4th or 5th axis, no U/V and no tool-axis vector. G-D9 ("CNC stays three-axis") and
  `toolpathkit/docs/PLAN.md` defer multi-axis on purpose.
- `MachineBinding` requires exactly three axes.
- The chain is `ToolpathLowering` → `MotionOp.FollowPath(PosePath)` → `ProgramCompiler`
  (`KinematicsSolver.solvePath`, TOPP-RA) → `ProgramPlanner`/`TrajectoryStream`.
  `AxisKinematics` is the exact Cartesian solver.
- A solver may declare an exact affine map: `KinematicsSolver.affineMap():Null<AffineJointMap>`
  (`motionkit/haxe/motionkit/kinematics/AffineJointMap.hx`). It has **three columns, over the
  tool's x, y, z**. Affine machines then get the fast planner `AxisJointPathPlanner`
  (sampled once, staged by speed changes, closed-form bounds) instead of the general TOPP-RA path.
- `OrientationPolicy {Fixed, Interpolated, Cone, FreeAboutTool, Free}`.
- Multi-frame IK exists in `kinematicskit` (`KinematicProblem` with several `FrameTask`s) but is
  not wired into toolpaths.
- `DistanceDrive` injects a process-driven joint column after IK. This is the extruder
  precedent for "a quantity along the path, not an axis".
- `Heat(heater, celsius, wait)` and `Fan` ops; `ToolpathChannels` heater/fan prefixes.
- `ToolpathPlayer` app base with hooks `prepare/beforeStep/afterStep/resetJob`.
- `CncInterpreter` (`cnckit/haxe/cnckit/interp/CncInterpreter.hx`) rejects U/V/A/B/C as
  "unsupported word". `CncWriter` is the LinuxCNC post. There is no GRBL anything.

**Stock.**
- StockKit is a tri-dexel model with exact interval endpoints (`stockkit/core/include/stockkit.h`,
  `core/src/sweep.cpp`).
- Sweeps support only an axisymmetric tool with its axis along +Z. Tilted sweeps are open as
  `stockkit/TODO.md` Phase 7. DESIGN.md says new motions are new `SweptVolume` kinds, not a new
  stock model.
- `app/src/MachiningStock.follow` cuts the segment the tool travelled each tick.
  `StockPreviewWorker` remeshes changed tiles off-thread.
- `sk_stock_add`. There is no connected-component or severed-piece query.

**CAD/CAM.**
- `cad_loft(..., ruled)` builds wing solids and reference ruled surfaces.
- `cad_wire_offset` (`Curve.offset`) is on main.
- `cad_shape_section(solid, origin, normal)` takes any plane. Also:
  `cad_planar_offset/cut/common/fuse`; `PlanarRegion.section` is z-only.
- CamKit has 2.5D profile/pocket/drill (`CamJob`) and `toolpathkit` `ArcFitting`.

**Heat.**
- Device `robotkit/device_protocol/boards/heater.rs`: PID, Beta thermistor, latched faults
  (SENSOR_OPEN/SHORT, OVER_TEMPERATURE, NOT_HEATING, LEFT_BAND, ...).
- Virtual plant `robotkit/device_virtual/src/heaters.rs`:
  `C dT/dt = P·duty − h(T − T_amb)`.
- Host side: `HeaterSensor`, `HeaterChannels`, `VirtualHeaters`. Parts: `HeaterCartridge`,
  `Thermistor`.
- A hot wire has no thermistor; see FC-D4.

**Scene.**
- `SceneArtifact.VERSION` is 32. It has exclusive `machining`
  (three mandatory axes, cutter tools), `printing` and mission sections. The rule is: bump the
  version and reject the old one.

**Simulation hooks.**
- `SimulationStepObserver.afterSimulationStep` (used by `SimulatedWelder`).
- App `ToolpathPlayer.afterStep`.
- Virtual peripherals via `rkd_virtual_set_peripheral_input`.
- MuJoCo uses joint-space tendons for couplings. No spatial tendons or flex are used.

## What is missing

1. A machine with two independent wire-end carriages, a wire that is not a rigid part (its length
   changes with taper and heat), and tower planes derived from the assembly.
2. A way to say "this line" in a program, and kinematics from a line to XYUV.
3. Feed defined on the faster foam face, and planning that bounds the speed of both wire ends.
4. A stock sweep for a moving line thickened by a kerf that varies along it, plus detection of
   pieces the cut has severed.
5. CAM: profile pairing, a ruledness check, kerf per side, lead-ins, corner treatment.
6. Wire heat by power, not temperature: supply, wire-break detection, and a wire thermal plant
   that cools when cutting.
7. Process models for kerf (power/speed) and lag (drag on a tensioned string).
8. XYUV G-code in and out, with each controller's feed semantics.

## Decisions

**FC-D1. A program describes the wire as a line in the part frame, at two stations, not in tower
coordinates.**
- The setup gains a wire frame: the wire direction `w` in the part frame and two **stations**
  `wA < wB`. These are normally the block's two faces.
- A new geometry kind `PathGeometry.Ruled(a, b)` describes one span. `a` and `b` are each a `Line`
  or `Arc` lying in station plane A and B. Points pair by fraction of each side's arc length.
  Consecutive `Move(kind, Ruled(...), feed, ...)` form the cut. `Rapid` and `Cut` kinds, feed,
  tolerance and provenance are reused unchanged.
- Validation: each side lies on its station plane; spans join on both sides; there are no
  zero-length pairs on one side only (a pure pivot is a valid span with `a` of zero length but is
  flagged).
- A 2-axis cut is a `Ruled` span whose two sides are congruent. There is no separate kind.
- Rejected:
  - XYUV in the IR. It is machine-dependent: a different tower spacing, a rotated block or a
    robot-held bow would each need a new program.
  - One curve plus a tool-axis vector (5-axis style). It loses the pairing on the second face,
    and it would pull in the deferred 5-axis IR. If 5-axis flank milling ever arrives, it should
    share the ruled primitive, because a ruled span is flank milling with an unbounded tool.

**FC-D2. The tower planes and the wire length come from the assembly. `WireKinematics` is
closed form.**
- Each Y/V carriage carries a wire guide whose pivot connector defines that tower's plane. The
  wire passes through the guide, so the guide is where the wire's direction changes.
- Inverse: intersect the line through the two station points with both tower planes:
  `P = A + (x_t − wA)/(wB − wA)·(B − A)`, the same as FoamCut.
- XYUV are **affine in the pair (A, B)**. On `Line` spans the joint paths are exactly linear.
  The cutter should get the affine fast planner, but `AffineJointMap` has three columns over the
  tool position, and the wire needs six (A and B). Generalise the map's columns from "tool x, y,
  z" to "the path's coordinates" (3 for a point path, 6 for a ruled path), and let the planner
  bound both stations' speeds from them. Rejected: a pose-based map with the slide along the wire
  pinned, which gives up exactness on tapered spans. Settled in FC2.
- The wire is not a rigid member. The tension spring is a member with a passive prismatic joint.
  Its extension is **derived** from the guide distance and the wire's thermal length:
  `e = |P_L − P_R| − L_wire(T)`.
  - Its stroke limit is the wire-stretch check (FoamCut's `WireStretchLength`), now a joint limit
    checked by planning like any other.
  - Tension `T = k·e + T0` feeds the lag model (FC-D7).
- The `PosePrimitive` pose is the wire line at station A: point = A, tool Z along `B − A`,
  `FreeAboutTool`. Position along the wire is fixed by the station. The extra "slide along the
  tool" freedom a robot bow needs is added in FC14, not now.

**FC-D3. Feed is the speed of the faster foam face, and planning bounds both faces.**
- A point on the wire moves at `|lerp(vA, vB, s)|`. That is convex in `s`, so bounding both
  station speeds bounds the whole wire inside the block.
- ProgramCompiler gets a task-space speed constraint on two points instead of one TCP.
- Only the component of speed normal to the wire cuts. Sliding along the wire does not. The kerf
  model (FC-D5) uses the normal component.
- G-code out is G93 inverse time, the only form every controller reads the same way.
- G-code in models each dialect's F:
  - LinuxCNC G94: F applies to the XY path only, and UV follow.
  - GRBL forks: Euclidean over all axes (verify per fork).
  - G93: explicit time.

  An imported G94 program then times exactly as the real controller would run it, and the
  simulation shows the resulting kerf error.

**FC-D4. Wire heat is a power setpoint on a heater without a thermistor.**
- A `WireSupply` part: a constant-current DC supply with PWM or a setpoint, sensing V and I.
- The device heater module gets a **Power** mode: target watts, a power loop on sensed V·I, and
  faults:
  - `OPEN_WIRE`: current collapses, meaning a broken wire. Feed hold and heat off.
  - `OVER_CURRENT`.
  - `NOT_HEATING`.
- Temperature is not measured. NiCr80's resistance rises only a few per cent at cutting
  temperature (assumed, check), so estimating temperature from resistance is a later option, not
  the control loop.
- IR: the `Heat` op's `celsius:Float` becomes a typed setpoint
  `HeatSetpoint {Temperature(c); Power(w)}`, validated against the heater's kind. `wait` means the
  wire plant has settled. That takes a few seconds; see the reference numbers.
- Rejected: a separate `WirePower` op. It would duplicate channels, barriers and the player wiring
  that `Heat` already has.

**FC-D5. One kerf model serves CAM and simulation.**
- `kerf(s, t) = k0 + k1·P′(t)/v⊥(s, t)`, clamped:
  - `P′` is the wire's effective cutting power per length, from the wire plant;
  - `v⊥` is the local wire speed normal to the wire.

  This is the energy-per-area form (de Smit & Broek 2004, heated blade in XPS), with a tilt
  factor `1/cos τ` for a slanted wire.
- Calibration is per material: two (speed, power) → kerf points, as in Jedicut/GMFC. It is stored
  as a material profile, and its numbers are marked assumed until measured.
- CAM uses the planned speeds, so the slow side gets a wider kerf: FoamCut's "Dynamic"
  compensation, derived instead of tuned with an exponent.
- Simulation uses the simulated speeds. Deceleration at corners and dwells then widens the kerf
  (melt-back) with no special case.

**FC-D6. StockKit removes a swept slab, and severed pieces drop.**
- A new `SweptVolume` kind: a line segment whose two ends move linearly (a bilinear patch),
  thickened by a radius.
  - Exact where the patch is flat within tolerance: a planar quad ⊕ ball. A ray against it is
    two planes, four capsule edges and four spheres.
  - Subdivided along the wire where it is not flat.
  - A kerf that varies along the wire is cut as pieces, each with its own radius.
- The same kind cuts a tilted end mill later (TODO Phase 7), so it is not foam-only.
- A wire cut separates material rather than removing it. StockKit gains connected components, and
  pieces not containing the fixture anchor are removed from the stock. They are shown as loose
  meshes and can optionally be spawned as free bodies in SimKit.
- Connectivity must not leak across a kerf thinner than the ray spacing. A 0.5 mm kerf on a 1 mm
  grid must still separate.
- Rejected:
  - Rotating the stock so the wire is +Z and cutting with a flat end mill. It only works without
    taper.
  - Mesh booleans per tick.

**FC-D7. Wire lag is a process model on a taut string, not a MuJoCo cable.**
- Drag per length inside the foam is `q(s) = c·max(0, v⊥(s) − v_free(s))`, where `v_free` is the
  speed at which the wire melts ahead without touching, from the kerf model.
- Deflection solves `T·y″ = −q` with zero deflection at the guides. It is piecewise closed form
  and points against the motion.
- The swept wire becomes a polyline of pieces, each cut as an FC-D6 slab.
- The same model gives a monitor ("lag above tolerance at corner N") and the corner rule in CAM:
  dwell for the lag to relax, or slow down before the corner. The dwell is bounded by melt-back.
- Rejected: MuJoCo flex or composite cable. Drag from melting is not contact, a 0.3 mm wire at a
  10 ms step is stiff and unstable, and the machine's joints do not feel the wire.

**FC-D8. The machine: two independent towers, four motors, belts horizontally, screws
vertically.**
- A T-slot base, and a fixed foam table between the towers.
- X and U: GT2 belts on linear rails along the base.
- Y and V: lead screws. They hold the wire height unpowered, and their stall torque is not
  fighting gravity on a belt.
- Each Y/V carriage carries an insulated arm and a wire guide. The tension spring is on one side.
- Homing switches on all four axes.
- An electronics cabinet: four drivers, the controller, the wire supply, an E-stop that also cuts
  wire power.
- The towers are not coupled, because taper needs them independent.
- An indexed turntable (B) is optional, in phase C.
- Sizes are in the reference numbers.

**FC-D9. CAM lives in `camkit.foam` and keeps paired `Line`/`Arc` spans exact.**
- Inputs:
  - a profile, or two profiles plus explicit pairing;
  - or a solid sectioned at the stations with `cad_shape_section`.
- **Ruledness check**: section the solid at k intermediate planes and compare with the ruled
  surface. Report the worst deviation and reject above tolerance. A sphere must be rejected, not
  silently cut.
- Pairing:
  - vertex to vertex where counts match;
  - user sync points split spans where they do not;
  - within a span, by arc-length fraction.

  Spline profiles such as airfoils are sampled to tolerance as paired lines. Arc fitting runs
  only where both sides fit arcs with the same breakpoints.
- Kerf per side from FC-D5 at the station faces, then extrapolation (FoamCut's order).
- Lead-in/out from the block faces, rapids outside the block above a safe height, heat schedule,
  and corner treatment from FC-D7.
- Rejected: discretising everything to equal polylines at 0.5 mm, as FoamCut does. It loses
  exact lines and arcs and inflates programs.

**FC-D10. Scene: a `wireCutting` section, and the shared program envelope extracted at the third
process.**
- `wireCutting` is exclusive with `machining`, `printing` and mission. It holds the wire,
  material profile, supply binding, stock, target and program.
- Machining, printing and wire cutting share program, stock, target, controller and loop. At this
  third copy, extract that envelope into one record used by all three, in its own commit inside
  FC10.
- Needs user OK, since it touches machining and printing.

**FC-D11. Process rules are checked before the program runs.**
- Every `Cut` span inside the block needs heat on and settled.
- `Rapid` spans must not intersect the block.
- The spring stroke stays in limits.
- Both tower points stay in travel.
- On feed hold or stop, heat goes off at once, because a stopped hot wire melts a pocket.
- A wire break holds the feed.

## Test parts

- **Star prism**: an extruded star with sharp inner and outer corners. A 2-axis cut; exercises
  corner lag and melt-back.
- **Frustum**: a square, tapered truncated pyramid. Equal span counts, sharp corners, strong
  taper; exercises per-side kerf.
- **Wing panel**: NACA 2412, root chord 250 mm, tip chord 160 mm, span 600 mm, sweep 40 mm,
  washout 2°. Built with a NACA 4-digit generator plus `cad_loft(ruled)`.
- **Mismatched profiles**: a circle on one face and a square on the other, paired with sync
  points.
- **Not ruled**: a sphere or a doubly curved fillet. Must be rejected by the ruledness check.

## Reference numbers (assumed until checked against a datasheet or a measurement)

| Quantity | Value | Source |
|---|---|---|
| Wire | NiCr80, Ø 0.3 mm, ρ ≈ 1.09 µΩ·m → 15.4 Ω/m | derived |
| Wire span (guide to guide) | 1100 mm → ≈ 17 Ω | assumed |
| Cutting power | ≈ 25 W/m → ≈ 27 W, ≈ 1.3 A at ≈ 21 V | ThomasHeb measured 20–25 W/m; WWU |
| Feed | 150–300 mm/min (XPS); FoamCut default 160 | forums, FoamCut |
| Kerf | ≈ 0.7 mm at 160 mm/min | FoamCut default |
| Wire thermal time constant | ≈ 4 s (m·c ≈ 0.27 J/K/m; loss ≈ 0.06 W/m/K) | derived, check |
| Thermal expansion | ≈ 14 µm/m/K → ≈ 6 mm over 1.1 m at +400 K | assumed, check |
| Tension | 15–25 N | assumed |
| Travel | X/U 900 mm, Y/V 450 mm | assumed |
| Mid-span lag | `δ ≈ q·L²/(8T)` | taut string |

The 4 s time constant matters: modulating power with speed through a short corner cannot work.
The simulation should show this, and FC11 should keep dwell/slow-down as the corner tools.

## Steps

### Phase A: machine, kinematics, IR, stock, CAM, G-code (can start on local main now)

**FC0. Baseline and test parts.**
- Run the gate on the starting commit and record failures that come from main, not from us.
- Add the NACA 4-digit generator and the test parts above. Put them in `camkit` test parts, or
  with the example if CadKit-only.
- Tests: wing solid valid; root and tip sections match the generator to 1e-6 mm.

**FC1. The machine (FC-D8).**
- Parts in `machinekit.foam`: `HotWire` (material, diameter, length), `WireGuide` (pivot
  connector), `TensionSpring` (rate, preload, stroke, passive prismatic joint), `WireSupply`,
  `FoamTable`.
- `FoamCutter extends AxisBuilder`: four prismatic joints `x y u v` in machine millimetres, belts
  and screws through `turnWithBelt`/`driveScrew`, motors through `addMotor`, homing switches,
  exposed `wireLeft`/`wireRight` guide connectors.
- `FoamCutterChecks` in the MachineKit smoke: joint travel, BOM, guides follow the joints
  exactly, the frame clears both towers at all travel corners. Use `AssemblyIntegrity` if MK10
  has landed, otherwise the per-machine clearance pattern.
- `materia.project.json`, a Start-page entry in a new "Foam cutting" category, and
  `ProjectSourceTests.checkFoamCutter` (loads, MuJoCo holds pose).
- Tests: smoke checks; app check.

**FC2. Wire kinematics (FC-D2).**
- `WireKinematics implements KinematicsSolver`: line → XYUV in closed form, forward XYUV → line,
  `solvePath` exact on lines.
- Derived spring extension and its limit. Tension from the spring part.
- Generalise `AffineJointMap` to the path's coordinates (FC-D2) and declare it from
  `WireKinematics`. Existing affine machines (router, CoreXY, printer) keep their numbers.
- Tests:
  - FK∘IK to 1e-9 mm over random lines;
  - an over-tapered line is rejected by the spring stroke, with the joint named in the
    diagnostic;
  - a line missing a tower plane (parallel to it) is rejected.

**FC3. Ruled moves in the IR and in planning (FC-D1, FC-D3).**
- `PathGeometry.Ruled`, setup wire frame and stations, validation.
- `RuledWirePrimitive` (exact pairing by fraction), lowering, a two-point speed constraint in
  `ProgramCompiler`.
- Do this after the printer plan's RF5 (split `ProgramCompiler` into `PathSampler`,
  `TimedMotionTransforms`, `PlanCertification`, one `CouplingProjection`). The two-point speed
  constraint then lands in `PathSampler`/`AxisJointPathPlanner`, not in the 1,500-line class. If
  RF5 has not landed when FC3 starts, mark FC3 `blocked: RF5` and do FC4/FC5 first.
- `MachineBinding` gains a wire binding (the four joints, the stations) next to the 3-axis
  binding, the way the printer added its binding.
- Tests:
  - a hand-written frustum program runs on the machine headless;
  - planned time matches the faster face's feed to 0.5 %;
  - the slower face's speed equals the length ratio;
  - joints follow lines exactly;
  - MuJoCo follows to the router's tolerance.

**FC4. StockKit swept slab and severed pieces (FC-D6).**
- New `SweptVolume` kind and a `CutMotion` variant (wire line pairs plus a per-piece radius).
  Bump the C API to 4 (it is 3 after `sk_stock_add`).
- Connected components, and a "remove pieces not containing point P" call.
- Tests:
  - exact ray hits against an analytic slab;
  - a frustum cut by hand-built sweeps matches the exact frustum to the grid's interval accuracy;
  - a 0.5 mm kerf on a 1 mm grid separates the offcut;
  - preview meshes show the kerf.
- Benchmark: a 1000 × 600 × 300 mm block at 1 mm spacing (≈ 1.1 M rays), µs per tick and memory.

**FC5. CAM (FC-D9).**
- `camkit.foam.FoamJob`: setup (block, wire direction, stations), `profile`, `ruled(a, b,
  syncPoints)`, `fromSolid(solid)`.
- Ruledness check, pairing, kerf per side (uniform until FC9, then the FC-D5 model), lead-in/out,
  joins, safe height, heat ops (once FC7 lands), `finish()` → `ToolpathProgram`.
- Tests:
  - the star, frustum and wing produce valid programs;
  - the sphere is rejected with its deviation;
  - the mismatched profiles pair through sync points;
  - the program's ruled surface reproduces the target faces to the sampling tolerance before
    kerf.

**FC6. XYUV G-code in and out (FC-D3).**
- A `HotWire` dialect in `CncInterpreter` (U/V words, M3/M5 S as wire power, G93/G94).
- Feed semantics per target (LinuxCNC, grblHAL/FluidNC, Marlin `FOAMCUTTER_XYUV`). Verify the
  axis letters and F rule of each from source before writing them down.
- Posts: G93 by default; LinuxCNC `xyuv` with `(AXIS,XY_Z_POS,…)`/`(AXIS,UV_Z_POS,…)` preview
  comments; grblHAL.
- The importer converts tower coordinates back to `Ruled` spans, with the stations at the tower
  planes.
- Tests:
  - round trip CAM → G-code → import → same joint paths;
  - a LinuxCNC G94 program times with UV following and differs from G93 by the expected ratio.

### Phase B: heat, process models, the app (dependencies on main since 2026-10-14)

**FC7. Wire heat (FC-D4).**
- `HeatSetpoint` in the `Heat` op.
- Device heater Power mode with OPEN_WIRE/OVER_CURRENT, in the Rust heater module plus the Nucleo
  profile.
- Virtual wire plant: `m·c·dT/dt = P − h·(T − T_amb) − P_cut`, where `P_cut` is the energy into
  the foam from FC-D5's removal rate. Its resistance follows `R(T)`.
- Host `HeaterChannels` power kind; wire-break → feed hold; heat off on stop.
- Tests:
  - settle time ≈ the time constant;
  - cutting load drops the temperature;
  - a broken wire (virtual input) holds the feed within one owner period and latches the fault;
  - device unit tests for the power loop.

**FC8. Process rules (FC-D11).**
- Pre-run checks on the compiled program.
- Diagnostics name the span and the provenance (CAM op or G-code line).
- Tests: a heat-off cut, a rapid through the block and an out-of-stroke taper each fail with the
  right diagnostic.

**FC9. Kerf model and calibration (FC-D5).**
- `HotWireMaterial` (two-point calibration, `k0`, `k1`, tilt factor) and material profiles
  (XPS, EPS), marked assumed.
- CAM switches to model kerf per side.
- Placement: pure Haxe in `camkit.foam`, or `processkit` if RobotKit R3's rule says process
  models belong there. Decide and record.
- Tests:
  - frustum: the slow side's planned kerf matches `1/v`;
  - calibration round trip.

**FC10. The app (FC-D10).**
- `wireCutting` scene section (version bump, reject the old version).
- The envelope extraction as its own commit, if the user agrees.
- `WireCutProgramPlayer extends ToolpathPlayer`, alongside `CncProgramPlayer` and
  `PrintProgramPlayer`.
- `WireCuttingStock.follow` (the counterpart of `MachiningStock`/`PrintingStock`): wire line
  from the simulated joints → per-piece kerf from local speed and wire temperature → slab cuts.
  Cut every ≈ 0.5 mm of travel, not every tick: the wire moves ≈ 0.05 mm per 10 ms tick.
- Wire view: a line between the guides, colour by temperature, bow later.
- Offcuts drop.
- `ProjectSourceTests.checkFoamCutter` runs the star and the wing.
- Tests:
  - the final stock against the target, as a deviation map;
  - executed time within 1 % of the planned time;
  - headless faster than 10× real time (a wing is minutes long).

**FC11. Wire lag and corners (FC-D7).**
- Taut-string lag from spring tension and drag; bowed-wire slab sweeps; lag monitor.
- CAM corner rules (dwell/slow-down) chosen from the model.
- Tests:
  - mid-span lag matches `qL²/8T`;
  - star corners: rounding without a corner rule versus within tolerance with it;
  - an over-long dwell shows melt-back.

**FC12. Acceptance.**
- Wing panel: final stock within 0.3 mm of the target at root and tip, with CAM and simulation on
  the same model.
- With uniform kerf, the slow side is measurably undersized. This shows the model is not
  circular.
- Frustum and star with corner rules.
- G93 G-code from the same job, re-imported, gives the same stock.
- A short report of the numbers in this plan.

### Phase C: later

**FC13. Indexed turntable.** A `RotaryTable` part (B axis) on the foam table. Rotation only
between an exit and an entry, as in FoamCut. CAM for multi-face parts (a four-sided tower) with
the block frame per index. Posts emit B.

**FC14. Robot hot-wire bow.** A bow end effector on `RobotArm`, using the same `Ruled` IR. The
wire line leaves two free DOF (roll about the wire, slide along it). These go to the process path
planner's tool freedom (PP ladder). The spring and stretch model carry over; the bow's own frame
replaces the towers.

**FC15. Controller configuration export.** FluidNC `config.yaml` or LinuxCNC `.ini`/`.hal` from
the assembly (axes, steps/mm, travel, homing, wire supply output), like the printer's Klipper
`printer.cfg`. Live link only if a real machine is targeted.

**FC16. Hot-wire lathe.** A horizontal rotary axis parallel to the wire, turning simultaneously
with cutting, for revolved parts. Needs a rotary stock sweep and time-varying ruled spans.

## Order

```
FC0 ── FC1 ── FC2 ──(RF5)── FC3 ──┬── FC5 ── FC6
FC4 ─────────────────────────────┤
FC7 ── FC8 ── FC9 ───────────────┴── FC10 ── FC11 ── FC12 ── FC13 … FC16
```

- Every dependency outside this plan is on main except RF5 (printer plan), which gates FC3.
- FC4 (StockKit) and FC7 (heat) are independent of FC1–FC3 and can run in parallel with them.
- FC5's heat ops and FC9's model arrive after FC7. Until then FC5 uses uniform kerf.

## Working rules

- Dependencies count only once they are on local `main`. Merge local `main` in to pick one up.
  Never merge another session's live branch.
- A blocked step is marked `blocked: <what>` in Progress, and work goes on to the next unblocked
  step.
- Commit per step, e.g. `Foam cutter: wire kinematics (FC2)`, with Progress updated in the same
  commit.
- Diagnose before working around. Numbers not from a datasheet or a measurement are marked
  assumed.
- No legacy compatibility: one scene version, reject others.
- Heavy suites go through the heavy-run wrapper.

## Progress

| Step | State | Commits |
|---|---|---|
| FC0 | planned | |
| FC1a | done: `cad_shape_distance`, CadKit core smoke passes | 20c53b08d |
| FC1b | planned | |
| FC1c | planned | |
| FC1d | planned | |
| FC1e | planned | |
| FC1f | planned (a draft of the parts and machine exists uncommitted in the worktree) | |
| FC2 | planned | |
| FC3 | planned (after printer RF5) | |
| FC4 | planned | |
| FC5 | planned | |
| FC6 | planned | |
| FC7 | planned | |
| FC8 | planned | |
| FC9 | planned | |
| FC10 | planned | |
| FC11 | planned | |
| FC12 | planned | |
| FC13–FC16 | later | |
