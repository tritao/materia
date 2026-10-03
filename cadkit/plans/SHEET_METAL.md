# Sheet metal and robotic press brake plan

**Goal.** Design sheet metal parts in CadKit and bend them unattended in a simulated robotic press
brake cell that would drive real hardware with the same signals.

1. A part is designed as sheet metal: base wall, edge flanges, hems, cuts, reliefs. Folded solid
   and flat pattern always agree, because both come from one model.
2. The flat pattern is exported as DXF with bend lines, checked for manufacturability, and becomes
   a blank requirement for `manufacturingkit`.
3. A bend process model turns material, thickness, tooling and target angle into ram depth, real
   inner radius, springback, tonnage and bend allowance. It is calibrated from measured angles.
4. A planner chooses the bend order, tool stations, part orientation, backgauge settings and
   robot grips, with every intermediate part shape collision-checked against the tooling.
5. In simulation, a cobot takes a blank from a stack, registers it against the backgauge, holds it
   while the press bends it (following the rising flange from the measured ram position),
   regrips or flips it when needed, and stacks finished parts.

The reference real machine is a low-cost hydraulic press brake (WC67Y class, torsion-bar
synchronised, mechanical depth stop) whose stock NC controller (E21 class) is replaced by this
stack for the backgauge and depth stop. The ram's hydraulic and safety logic stays in the
machine (SM-D14).

The work spans kits:
- `cadkit/haxe/src/cadkit/sheetmetal/`: the sheet model, features, flat pattern, convert,
  bend process model (phase A);
- `cadkit/haxe/src/cadkit/drawing/`: the 2D DXF/SVG writers;
- `machinekit/haxe/src/machinekit/pressbrake/`: tooling, press brake assembly (phase B);
- `machinekit/examples/press-brake/`: the press alone and the bending cell (phases B and C);
- planner code under `machinekit/haxe/src/machinekit/pressbrake/planning/` (pure, no app);
- RobotKit skill, ProcessKit device and app players for the cell (phase C).

## What exists

Surveyed 2026-10-03 on local main 1048bf768, plus the `machine-tending`, `mobile-welder`,
`x7-transmissions` and `collision` branches.

**CadKit native core** (`cadkit/core/include/cadkit.h`, Haxe binding generated into
`cadkit/haxe/abi/CadKit.hxi` from the header's `CADKIT_HXI_*` annotations):
- primitives, `cad_shape_extrude`, `cad_shape_revolve`, `cad_fuse`/`cad_cut` (all with
  `*_operation` history variants), fillet/chamfer, loft, sweep, `cad_shell`, planar faces from
  wires, STEP import/export, `cad_shape_valid`;
- queries: `cad_shape_subshape_count/at`, `cad_shape_is_same`, `cad_face_surface_kind`,
  `cad_face_center`, `cad_face_area`, `cad_face_normal` (one normal at the middle of the
  parameter range), **`cad_face_axis`** (plane location and normal, cylinder axis and radius),
  **`cad_edge_axis`** (circle centre, normal, radius), edge tangent and position at a parameter;
- topological names: `cad_shape_copy_element_names_bytes`, **`cad_shape_seed_names`** (name a
  shape's faces/edges from identifiers), `cad_shape_stamp_names`, name matching.
- Missing: edge → face ancestry (which two faces share an edge), face parameter range and edge
  curves in face parameters, a normal at a chosen point.

**CadKit parametric layer** (`cadkit/haxe/src/cadkit/parametric/`):
- `Feature` (`evaluate`, `commitEvaluation`/`discardEvaluation`, `registerParameter`,
  `dependencies`, `serializationType`), `Document` (staged atomic recompute, transactions,
  undo/redo), `DocumentCodec` (versioned JSON, dispatches on `featureType` strings around lines
  270–335 and 579–742), `recording/DocumentBuilder`.
- Features: box, cylinder, extrude, revolve, sweep, loft, shell, offset, fillet, chamfer,
  boolean, mirror, patterns, hole, pocket, transform, sketch, constrained sketch, datum sketch,
  imported STEP, tool collection, definition output.
- Persistent references: `TopologyReference`, `TopologyFingerprint`, `TopologyResolver`,
  `SelectionRecipe` (history first, then fingerprint, ambiguity reported, never guessed).
- Sketcher (`cadkit/haxe/src/cadkit/sketch/`): constrained lines, arcs, circles, numeric solver
  with diagnostics, `SketchProfile`, `FaceWorkplane`.
- Immediate modelling API (`cadkit/haxe/src/cadkit/modeling/`): `BuildPart`, `BuildSketch`,
  `Plane`, `Location`, `Axis`, assemblies.

**Elsewhere**
- `projectkit` `MaterialDef`: visual properties plus `physical {density, spec}`. No strength
  or elastic data.
- `manufacturingkit` `materia.sheet`: rectangular sheet stock, `SheetPartRequirement`
  (thickness ± tolerance, blank size, rotations, grain `directionConstraint`), guillotine cut
  plans, `SheetPlanExporter` (CSV + SVG). No nesting, no free-form outlines.
- No DXF anywhere. No sheet metal, press brake, bend tooling or bending code anywhere.
- MachineKit has the component/connector/port/capability model, `ParallelGripper` (envelope
  only), suction tools, `LinearRailSystem`, lead screws, `ServoMotor`, `Gearbox`, encoders,
  `PortKind.Signal`, and the X-series transmissions on their branches.
- Planned on `machine-tending` (MT plan, not built yet) and needed here: cobot arms in size
  classes (MT3), pneumatic actuators + digital switches + presence sensors (MT5), two
  controllers per cell + `CellSignals` (MT6), a machine-side robot-interface state machine
  pattern (MT7), a real parallel gripper and `gripper` robot-tool kind (MT8), mission steps
  `signal`/`waitFor`/`moveVia` (MT10).
- Collision: `kinematicskit` `CollisionWorld` on coal (branch `collision`, CL1). `ArmClearance`
  on the welder branch. No collision-aware planner (CL6).
- SimKit: `hold_object`/`release_object`, kinematic actors, MuJoCo weld equalities (used for
  closures in `mujoco_backend.cpp`). No deformable bodies.
- RobotKit: no regrasp, no force control, D5 streamed Cartesian servoing exists (MotionKit Lane D).
- Real hardware path: `robotd` + RKD6 serial protocol to a Nucleo STM32G474 (Rust no_std);
  output channels with safe values and timed events; no pins driven yet (board layer, encoder
  inputs and servo modes are follow-on work in MotionKit Lane A).

**External reference.** github.com/shaise/FreeCAD_SheetMetal is **LGPL-2.1: ideas and formulas
only, never port or translate code.** Useful ideas: the tangent-face-graph unfolder
(`SheetMetalNewUnfolder.py`), the edge flange parameter vocabulary, K-factor conventions
(ANSI vs DIN), "engineering mode" (no default K-factor), the backgauge distance rules and K-factor
table in `tools/README.md`.

## Decisions

**SM-D1. The sheet is the model; the solid and the flat pattern are derived.**
- A `Sheet` is a graph: `Wall`s (planar regions: an outer loop and holes of lines and arcs, in
  the wall's own 2D frame) joined by `Bend`s (a straight edge segment of one wall to a straight
  edge segment of another, with signed angle, inner radius and a bend-allowance rule). `Rip`s
  are cuts with zero gap that break connectivity. `Form`s (louvres, embosses) are later and
  not developable; they carry a flat-pattern marker.
- Thickness `T` is one value per sheet. Each wall has side A (its frame's +Z face at z = 0) and
  side B (z = −T, or +T, fixed by one sheet-wide convention: **side A is the reference side, the
  material grows toward −Z**). A bend's angle is positive when it folds toward side A
  ("up", side A concave) and negative toward side B ("down").
- A wall's 2D outline is exact on both sides (walls are prisms), so it is valid in the flat and
  the folded state. Only bend zones differ: a flat rectangle `BA × width` flat, a cylindrical
  shell of inner radius `R` and outer `R + T` folded.
- Features edit the graph; nothing edits the solid. This is the Onshape-style design, not the
  FreeCAD one (features build B-rep, a separate tool recognises it). Reasons: exact flat pattern,
  permanent bend identity, and every partly folded state is cheap (SM-D2), which bend planning,
  following and simulation all need.
- **Rejected:** unfolding the folded B-rep on every regeneration. That path is kept only for
  converting imported or plain solids into a `Sheet` once (SM-D7).

**SM-D2. Folding is forward kinematics.**
- `SheetState` assigns each bend an angle between 0 and its target. `flat()` is all zeros,
  `folded()` all targets.
- Evaluation walks a spanning tree from a root wall. Across a bend, the child wall's frame is the
  parent's frame composed with: move to the bend edge, rotate by the angle about an axis parallel
  to the edge on the concave side at distance `R` from the concave surface (z = +R for up bends,
  z = −T − R for down bends), and carry the child's start edge along the arc so that the bend
  zone's neutral length equals `BA` in every state (flat: the child starts `BA` beyond the edge).
- Cycles (closed boxes, walls joined on two sides) are allowed only as closure constraints: the
  non-tree bend must close within tolerance in the evaluated state, or the sheet reports
  `sheet.closure-gap` with the measured gap. Rips are how designs break loops. Never guess.
- Wall and bend frames are `cadkit.modeling.Location`s. No dependency on KinematicsKit.

**SM-D3. Bend allowance comes from a bend rule; there is no silent default.**
- `BendRule` is one of `KFactor(table)` or `Process(tooling)` (SM-D8). `KFactorTable` maps
  `R/T` to K (ANSI convention internally, piecewise linear, clamped at the ends); DIN values
  are accepted and halved on input.
- `BA = θ·(R + K·T)` with θ in radians and `R` always the inner (concave-side) radius, so the
  formula is the same for up and down bends. (FreeCAD measures from a fixed side and writes
  `K − 1` for down bends; measuring from the concave side avoids that sign rule.)
- `OSSB = (R + T)·tan(θ/2)`, `BD = 2·OSSB − BA`.
- A sheet without a bend rule fails with `sheet.no-bend-rule` (FreeCAD's "engineering mode",
  always on). Material library entries carry a default rule, so picking a material is enough.

**SM-D4. Solids are generated from the graph and named by it.**
- Each wall is its outline extruded by `T` along −Z of its frame. Each bend is the rectangle
  `T × width` revolved by θ about the bend axis. Everything is fused.
- Faces are named with `cad_shape_seed_names` from graph identity:
  `wall:<id>:a`, `wall:<id>:b`, `wall:<id>:edge:<loop>:<segment>`, `bend:<id>:inner`,
  `bend:<id>:outer`, `bend:<id>:end:0|1`. Wall and bend ids are permanent (created once, never
  renumbered), so names survive parameter edits by construction.
- Fusing many touching solids is acceptable to start. If it is slow or fragile, a native
  builder that assembles faces with known topology (planar and cylindrical faces sharing exact
  edges, then sewing) replaces it later, with the same names.

**SM-D5. Sheet features form their own chain inside the document.**
- `SheetFeature extends Feature` and exposes `sheet():Sheet` alongside the folded `shape`.
- Sheet features consume only the previous sheet feature's `Sheet`. A general solid feature
  (boolean, fillet...) after a sheet feature produces a plain solid: downstream code that needs
  a flat pattern must point at a sheet feature or convert (SM-D7). This keeps one owner per fact.
- Edge references (which wall edge a flange starts from) are persistent references to the
  generated `wall:<id>:edge:...` faces, resolved through the existing `TopologyResolver`, so
  the editor picks faces as for any other feature.

**SM-D6. Flat pattern is data first, files second.**
- `FlatPattern`: outer and inner loops (lines and arcs, splines refused), bend lines with id,
  direction, angle, inner radius and allowance, form markers, grain direction, part bounding
  box.
- DXF writer: ASCII R12 (AC1009) with `LINE`, `ARC`, `CIRCLE` and `TEXT` entities only
  (`LWPOLYLINE` is not R12), layers `OUTLINE`, `BEND_UP`, `BEND_DOWN`, `ETCH`, `TEXT`. SVG writer
  with the same layers. Both pure Haxe in `cadkit.drawing`.
- `manufacturingkit` gets a `SheetPartRequirement` from a flat pattern: blank = bounding box
  (rectangular stock only, until nesting exists), grain constraint from the bend lines (bends
  across the grain preferred), thickness from the sheet.

**SM-D7. Converting a solid is a one-time import into a `Sheet`.**
- Algorithm (ideas from FreeCAD's new unfolder, our own code): faces that share an edge and meet
  tangentially form a graph; the component containing a chosen root face is one skin; planar
  faces become walls, cylindrical faces become bends (axis, radius, angle from `cad_face_axis`
  and the face's boundary); thickness is the most common radius difference of coaxial cylinder
  pairs, cross-checked by the distance between parallel planar pairs.
- Limits, reported not guessed: only planar and cylindrical faces, straight bend edges, constant
  thickness. Anything else fails with a named diagnosis listing the offending faces.
- Needs one native query: edge → face ancestry (`cad_shape_ancestors` over
  `TopExp::MapShapesAndAncestors`). Concave/convex comes from comparing `cad_face_normal` with
  the radial direction from `cad_face_axis`; add `cad_face_normal_at(u, v)` only if the
  mid-parameter normal proves unreliable.

**SM-D8. A bend process model, air bending first.**
- One interface, `BendProcess`, answers for (material, thickness, punch, die, target angle,
  length): inner radius, bend allowance, ram depth below die top, springback and overbend
  angle, tonnage, minimum flange, and which tooling is valid.
- `AirBending` implements it first. `Bottoming` and `Coining` are later implementations of the
  same interface, not branches in callers.
- Starting rules, each marked assumed in code and calibratable:
  - natural inner radius `r ≈ 0.16·V·(UTS / 410 MPa)`; if the punch radius is larger, the punch
    radius governs and the result is flagged "radius bend";
  - tonnage `F[kN] ≈ 650·T²·L / V · (UTS / 450 MPa)` with `T`, `V` in mm and `L` in m;
  - minimum flange `≈ 0.7·V` for 90° bends;
  - recommended `V` between 6·T and 12·T, default 8·T for `T ≤ 3 mm`;
  - springback from the elastic–plastic bend formula
    `Ri/Rf = 4·(Ri·Y/(E·T))³ − 3·(Ri·Y/(E·T)) + 1`, with neutral length conserved
    `(Ri + T/2)·αi = (Rf + T/2)·αf`, solved for the loaded angle `αi` that unloads to target;
  - ram depth from the three-circle construction: punch tip circle (radius `rp`) tangent to the
    inner surface, die shoulder circles (radius `rd`, centres `±V/2` apart) tangent to the outer
    surface, flanges straight and tangent to both. Derive the closed form; test it against a
    numeric construction.
- A `CalibrationTable` keyed by (material lot, thickness, punch, die, angle band) stores
  measured corrections (depth offset, radius) that override the analytic values.
- With a `Process` bend rule, the flat pattern is computed for the chosen tooling.

**SM-D9. The press brake is an ordinary MachineKit assembly.**
- Ram on prismatic joints driven by two hydraulic cylinders. The torsion bar of the reference
  machine is a coupling Y2 = Y1 (X5 coupling). A closed-loop Y1/Y2 machine is a variant without
  the coupling and with a linear scale per side.
- Hydraulic cylinders are actuators like MT5's pneumatic ones: force `p·A`, speed limited per
  phase (approach, working, return) by pump flow, the phase change at the speed-change point.
- The mechanical depth stop is its own axis (motor + screw) on stop-controlled machines.
- Backgauge X (and optional R) are axes with servo drives and X7 transmissions; fingers are parts
  with contact switches (MT5 `addSwitch`).
- Tools are catalogue parts (Promecam-style punches and dies in segments, SM7). A
  `ToolClamp` capability is manual or powered (lock/unlock output + state sensor, shaped like
  `ChangerLock`).
- Machine profile: `StopControlled` (WC67Y + E21-class mechanics) or `ClosedLoopY1Y2`. The
  profile declares what the controller can know and do; the planner and players read it.

**SM-D10. Bend order is a search over sheet states.**
- State: completed bends, part orientation relative to the press (which side up, which end
  toward the backgauge), current grip, current station.
- Action: bend `b` at station `s` with orientation `o` and gauge edge `g`, or regrip (at a regrip
  station, or in the press while the ram holds the part at the clamping point), or flip.
- Feasibility per action:
  - tooling valid for the bend (SM-D8), flange at least the minimum;
  - a gauge edge exists: a part point that touches a finger at reachable X (and R, if the machine
    has R) in the state before the stroke. X and R come from the evaluated `SheetState` in the
    press frame, not from a formula; the "first bend = mould-line distance, later = b + flange
    difference" rule is a test oracle;
  - no collision between the part and punch, die, ram, frame and backgauge, sampled through the
    whole stroke (angle 0 → loaded angle);
  - the grip stays valid: the gripped wall is not in the bend zone, the gripper clears the tooling
    through the swing, the robot can reach the grip path (IK with margin);
  - station segments leave gaps where earlier flanges must pass (box bending).
- Cost: a time model (handling, flips, regrips, station moves, backgauge moves, bend time).
- Search: best-first with an admissible heuristic (remaining bends × minimum bend time). If no
  sequence exists, return the deepest failing states with named reasons ("flange B3 shorter than
  minimum 8.4 mm at V12", "flange W2 hits punch at 62°").
- Collision goes through a narrow interface `SheetCollisionQuery` implemented on KinematicsKit's
  coal `CollisionWorld`; walls are decomposed into convex prisms (outline → convex pieces), bend
  zones into a few convex segments. Switch the implementation to collisionkit when CL2–CL4 land.

**SM-D11. In simulation the sheet is welded wall bodies whose relative poses are the sheet's FK.**
- Each wall (and each bend zone, as a few segments) is a MuJoCo body with convex geoms. Neighbours
  are joined by weld equalities whose relative pose is the current `SheetState`.
- Outside the press the sheet is free (falls, rests, is gripped by contact). During a stroke the
  bend angle is process-driven: angle from ram depth through `BendProcess`, and the weld relative
  poses are updated each tick. On release, springback sets the final angle.
- In the press frame, the part's bend line stays at the punch tip and both sides rotate up by half
  the current bend angle; the back flange slides along the fingers (which retract, as the real
  retract function does).
- **To verify first (SM12):** that SimKit can update weld equality relative poses at runtime
  (`eq_data`). Fallback: rebuild the sheet's bodies once per bend (a geometry swap like the
  welder's bead), with the bend itself animated kinematically.
- **Rejected:** FEM or flexible bodies. Accuracy comes from the process model and calibration,
  not from simulating the metal.

**SM-D12. The robot follows the measured ram, through compliance it does not need to control.**
- Registration: approach the fingers, creep until both finger switches make, then hold. Position
  control plus contact sensing; no robot force control.
- Follow: the gripped wall's press-frame pose is a function of the measured ram position
  (`SheetState` with the angle from `BendProcess`). The robot runs it as a D5 streamed servo task
  keyed to the ram position input, never to time. The same code drives a real press whose speeds
  drift with oil temperature.
- The gripper deviation from the gripped wall during the follow is measured in simulation and
  reported. The acceptance limit stands for what a real gripper's passive compliance must absorb.
- Ownership during a stroke: from the clamping point until the ram opens, the press owns the
  sheet (SM-D11). In simulation the vacuum gripper's hold is released at the clamping point and
  taken again when the press opens; the vacuum stays on as a channel, as it would physically.
  The robot follows the gripped wall's predicted pose and the deviation is measured between the
  TCP and the wall's grip frame.
- Grippers: a vacuum gripper (the existing suction capability and `holdObjectOnLink`, unless a
  contact-based vacuum model exists by then) for loading, following and unloading; MT8's parallel
  gripper later for small parts and edge grips. Tool choice per part is the planner's.

**SM-D13. Two controllers, signals as wires (reuse MT-D1..D3).**
- The press controller owns ram, depth stop, backgauge and tool clamps; the robot controller owns
  the arm and gripper.
- `PressBrakeInterface` is a pure state machine on the press controller, the press brake's
  counterpart of MT7's `RobotInterface`:
  - inputs: `robot.request_stroke`, `robot.request_clamp_only`, `robot.request_open`,
    `robot.program_step(n)`, `robot.clear`, `robot.fault_reset`, finger switches;
  - outputs: `press.ready`, `press.backgauge_in_position`, `press.at_top`, `press.clamping`
    (pinch point reached), `press.at_bottom`, `press.decompressed`, `press.alarm(code)`, and the
    ram position as an analogue input to the robot controller;
  - interlocks: a stroke starts only with backgauge in position, part registered (fingers made)
    and the robot holding; open only after bottom or on request at clamping; every request times
    out into an alarm.
- Simulated safety (e-stop chain, fence, mode selector, light curtain muted in automatic) is
  modelled as signals for behaviour tests only. On hardware, safety is hardwired and certified
  (SM-D14).

**SM-D14. Hardware: replace the positioning controller, never the ram's safety.**
- Level 1 (bring-up): keep the machine untouched; add a start relay wired through the machine's
  safety circuit, a ram linear scale and the top-of-stroke switch.
- Level 2 (target): replace the E21-class controller: our controller drives backgauge X (+R) and
  the depth stop with servo drives in step/dir mode, reads the ram scale, the top / speed-change /
  bottom switches and finger switches, and starts strokes through the same relay.
- Level 3 (our software switching ram valves) is **rejected**. EN 12622 requires the closing
  stroke's safety functions at the highest reliability class; a certified safety controller
  keeps final authority over valves, e-stops, curtain and guards, and our controller only
  requests.

## Reference numbers (fixed in SM7, SM8, SM13; assumed until checked against a spec sheet)

| Item | Value |
|---|---|
| Press class | WC67Y-style, 40 t, 1600 mm bed, torsion bar, mechanical depth stop |
| Stroke / open height / throat | 100 / 320 / 230 mm |
| Speeds | approach 80 mm/s, working 8 mm/s, return 70 mm/s; speed change 10 mm above sheet |
| Backgauge | X 500 mm servo + ball screw, ±0.05 mm; R 120 mm optional; 2 fingers with contact switches |
| Tooling | Promecam style; punches straight 88° R0.6, gooseneck 88° R0.6, acute 30°; dies V8/V12/V16/V22 (single) and a 4V block; segments 10, 15, 20, 40, 50, 100, 200, 300 mm plus horns |
| Material A | 1.5 mm mild steel, UTS 400 MPa, yield 250 MPa, E 205 GPa, density 7850 kg/m³ |
| Material B | 1.0 mm 5052-H32 aluminium, UTS 228 MPa, yield 193 MPa, E 70 GPa (springback contrast) |
| Cell | cobot class picked by the SM13 reach study (start `Reach1300`), vacuum gripper 200 × 100 mm with 4 cups, blank stack table, centring (squaring) table, regrip table, outfeed table, fence |

Demo parts (all in material A unless noted):
- **P1** L-bracket 100 × 60 × 40, one 90° bend.
- **P2** U-channel 120 long, flanges 40 / 60 / 40, two bends the same way.
- **P3** Z-bracket, two bends in opposite directions (needs a flip).
- **P4** Hat section with short lips, four bends (the last needs the gooseneck).
- **P5** Tray 150 × 100 × 25, four flanges with corner reliefs (segmented punch with gaps; box
  bending).
- **P6** P2 in material B (springback and calibration contrast).

## Steps

### Phase A: sheet metal design (CadKit; can start now)

**SM0. Base and setup.**
- Worktree `/home/joao/dev/materia-worktrees/sheet-metal`, branch `sheet-metal` from local main
  1048bf768. Commit on this branch; do not create other branches.
- Phase A needs only the `haxeon` submodule. The `mobile-base` worktree is at the same commit
  (1048bf768) with populated submodules: `git clone --no-checkout
  /home/joao/dev/materia-worktrees/mobile-base/haxeon <wt>/haxeon`, check out the pinned sha
  (`git submodule status`), and copy `haxeon/.tools/` and `haxeon/out/haxeon_runtime.hdll` from
  that worktree. Write guarded paths in destructive commands (`"${WT:?}"/...`).
- CadKit native: `cadkit/core` is unchanged between origin/main 3fd620e4 and 1048bf768, so use
  `CADKIT_BUILD_ROOT=/home/joao/dev/materia-worktrees/topological-naming/cadkit/build/debug`
  until SM5 changes the native core. The shared checkout's build lacks `cad_shape_stamp_names`.
- When SM5 needs a native build: configure CadKit in the worktree with
  `CADKIT_OCCT_DIR=/home/joao/dev/materia-cache/occt-f1dc4efb-release` set **before** the first
  configure (`-DCADKIT_BUILD_OCCT=OFF`). Never build OCCT.
- Tests: `cadkit/scripts/test-haxeon` with `CADKIT_SMOKE_ENTRY=<entry>`. Add a
  `SheetMetalSmoke.hx` entry in `cadkit/haxe/tests/` and keep `HaxeonSmoke` passing.
- Disk: check `df -h /` first (9.8 GB free on 2026-10-03). Populate other submodules only when a
  later step needs them.
- Read `cadkit/ARCHITECTURE.md`, `cadkit/haxe/MODELING.md`, `cadkit/plans/TOPOLOGICAL_NAMING.md`
  before writing code; match their conventions.

**SM1. The sheet core (pure Haxe).**
- `cadkit.sheetmetal`: `Sheet`, `Wall`, `WallOutline` (loops of `Segment` = line or arc, 2D),
  `Bend`, `Rip`, `SheetSide`, `BendRule`, `KFactorTable` (ANSI/DIN input), `SheetMaterial`
  (thickness, bend rule, optional mechanical data for SM6), `SheetState`, `SheetEvaluation`
  (wall and bend-zone frames), `SheetDiagnosis` with codes (`sheet.no-bend-rule`,
  `sheet.closure-gap`, `sheet.bend-edge-mismatch` (bend edges of unequal length or not straight),
  `sheet.self-overlap`, `sheet.radius-too-small`, ...).
- Ids are permanent: `Sheet` allocates wall/bend ids monotonically and never reuses them.
- Operations: `addWall`, `addBend(wallA, edgeA, wallB, edgeB, angle, radius)`, `rip`, `cutHole`,
  `evaluate(state)`, `flat()`, `folded()`, `bendAllowance(bend)`.
- Length-spec helpers for features: `Leg` (straight length), `OuterSharp` (offset `(R+T)·tan(θ/2)`),
  `InnerSharp` (`R·tan(θ/2)`), `Tangential` (differs from `OuterSharp` above 90°: offset `R + T`).
  Refuse `OuterSharp`/`InnerSharp` above 179°.
- Tests (pure, no native):
  - FreeCAD's known answer: T 2, K 0.38, r 1.64, 90°, leg 48.12 → flange 51.76, mould-line
    distance 50 (±0.01);
  - BA, OSSB, BD against hand-computed values for 30°, 90°, 135°;
  - FK invariants: `evaluate(flat)` puts every wall in one plane; a wall's frame after folding and
    unfolding returns to the start within 1e-9; the neutral-line length across each bend equals
    `BA` in every state;
  - P1–P5 sheets built by code: folded bounding boxes match the drawn dimensions;
  - a closed box without rips reports `sheet.closure-gap` when a flange length is changed;
  - DIN K input equals twice the ANSI value.

**SM2. Solid generation and naming.**
- `SheetSolid.build(sheet, state):Shape` per SM-D4, using `cad_shape_extrude`, `cad_shape_revolve`,
  `cad_fuse`, then `cad_shape_seed_names` for faces (and edges where faces cannot name them).
- Any state can be built, not just folded and flat (planning previews and debugging use it).
- Tests:
  - `cad_shape_valid` for P1–P5 folded and flat;
  - folded volume equals `Σ wall area·T + Σ θ·L·T·(R + T/2)` within 1e-6 relative; flat volume
    equals `Σ wall area·T + Σ BA·L·T` (they differ unless K = 0.5; assert both);
  - every face name is present and unique; names are identical after changing a flange length,
    an angle or a radius;
  - planar/cylindrical face kinds where expected (`cad_face_surface_kind`), bend axes from
    `cad_face_axis` match the evaluation's bend frames within 1e-6.

**SM3. Sheet features in the document.**
- `SheetFeature extends Feature` (SM-D5) with `sheet()` and the folded shape as `shape`.
- Features (each with `serializationType`, parameters through `registerParameter`, codec branches
  in `DocumentCodec`, builder helpers in `DocumentBuilder`):
  - `sheet-base`: from a closed constrained-sketch profile, thickness, material reference, side
    (profile is side A or side B);
  - `sheet-edge-flange`: on one or more wall edges (persistent references to
    `wall:<id>:edge:...` faces); parameters `angle`, `length`, `lengthSpec` (Leg, OuterSharp,
    InnerSharp, Tangential), `position` (MaterialInside, MaterialOutside, BendOutside, Offset +
    value), `radius` (defaults to the material's), `gap1`, `gap2`, `extend1`, `extend2`, relief
    `type` (None, Rectangular, Obround) with `width`/`depth` or a factor of T;
  - `sheet-hem`: closed, open (gap) or teardrop, length;
  - `sheet-cut`: a sketch on a wall face (side A or B), through the wall; holes may cross a bend
    only if the bend zone is cut too (the cut is mapped to the flat pattern and back exactly);
  - `sheet-rip`: along an edge between two walls that would otherwise form a loop;
  - `sheet-corner` (closing two adjacent flanges: gap, overlap, corner relief round/square) can
    land after the rest; record it as a sub-step.
- Edits must keep wall and bend ids stable (a flange edit updates its wall and bend in place).
- Tests (`SheetMetalSmoke`): build P1–P5 through the document API; codec round trip
  (encode → decode → identical sheets and names); undo/redo of a flange edit; a flange on a
  renamed-but-surviving edge still resolves; a flange whose edge disappeared reports an
  unresolved reference, not a guess.

**SM4. Flat pattern, DXF/SVG, design checks, blank.**
- `FlatPattern.of(sheet)` per SM-D6, `cadkit.drawing.Dxf` (R12 ASCII) and `cadkit.drawing.Svg`.
- Bend table: id, direction, angle, inner radius, allowance, length, sorted by id.
- Design checks (`SheetChecks`), each a named finding with the geometry involved:
  - flat pattern self-overlap (two walls overlapping when unfolded);
  - flange shorter than the minimum for the default tooling (from SM6 when available, else
    `4·T` placeholder marked assumed);
  - hole or cut closer to a bend than `2.5·T + R` (assumed rule);
  - a bend ending inside an outline without relief;
  - inner radius below the material minimum;
  - bend line within 15° of the grain direction, when the material has grain.
- `SheetBlank.requirement(flat, material)` → `manufacturingkit` `SheetPartRequirement`.
- Tests: DXF parses back (write a minimal reader in the test) to the same loops and bend lines;
  P5 flat pattern matches a hand-computed outline; each check fires on a crafted bad sheet and
  stays silent on P1–P5.

**SM4b. Editor integration.**
- Follow how `ExtrudeFeature`/`FilletFeature` are exposed in `app/src/EditorScene.hx` and
  `app/src/editor/ScenePropertyProvider.hx`.
- Create sheet features from the editor (base from a sketch, flange from a picked edge face),
  edit their parameters, toggle the viewport between folded and flat, show bend lines and the
  bend table, export the flat pattern as DXF/SVG.
- Tests: an app test in the style of `app/tests/src/tests/CadPlateWorkflowTests.hx` builds P2
  through the editor commands and exports a DXF.

**SM5. Convert a solid into a sheet.**
- Native: `cad_shape_ancestors(shape, sub, kind, ...)` (edge → faces) with an indexed
  Haxe-friendly variant, a C smoke test in `cadkit/tests/core_smoke.cpp`, regenerate
  `CadKit.hxi` with `cadkit/scripts/generate-haxeon-hxi`.
- `SheetConvert.fromShape(shape, rootFace):Sheet` per SM-D7, plus a `sheet-convert` feature
  that takes any solid feature or imported STEP.
- Tests: round trip P1–P5 (`Sheet → SheetSolid → STEP export → import → convert`) gives equal
  graphs (walls, bends, angles, radii, T within 1e-6); a filleted box (non-constant thickness)
  and a part with a conical face fail with the named diagnosis.

**SM6. Bend process model.**
- `BendProcess`, `AirBending`, `CalibrationTable` per SM-D8, in `cadkit.sheetmetal.process`
  (pure; tooling geometry enters as plain numbers so CadKit does not depend on MachineKit).
- `MaterialDef` gains optional `mechanical {uts, yield, youngs}` and optional `sheet {kFactor
  table, minRadiusRatio, grain}` in `projectkit`. Optional fields keep old projects loading; add a
  codec test that a v-current project without them still loads.
- `BendRule.Process(tooling)`: flat pattern for a chosen punch/die.
- Tests:
  - natural radius, tonnage and minimum flange against the formulas for material A at V8/V12/V16;
  - springback: material B overbends more than material A for the same radius ratio; the solved
    loaded angle unloads to target within 0.01°;
  - depth: closed form equals the numeric three-circle construction within 1 µm over 20°–150°;
  - a calibration entry overrides the analytic depth and radius.

### Phase B: the press brake and bend planning (MachineKit; only SM8 waits for X7)

**SM7. Tooling catalogue.**
- No prerequisite: tooling is plain parts, with no transmissions or drives.
- `machinekit.pressbrake`: `PunchProfile` (straight, gooseneck, acute; tip radius, included angle,
  height, relief shape as a 2D polygon), `DieProfile` (V width, angle, shoulder radius, height;
  single V and multi-V blocks), `ToolSegment` (length from the standard set, horns), `ToolStation`
  (segments along the bed with gaps, x offset), `ToolLayout` (stations on a bed),
  `ToolClamp` capability (manual or powered).
- Geometry: segments are extrusions of the profile, one part each (honest convex hulls; concave
  gooseneck profiles split into convex parts like MT-D10).
- Tests: catalogue rows; a station's length equals the sum of segments; a layout rejects overlapping
  stations and stations past the bed.

**SM8. The press brake assembly.**
- Prerequisite: X7 done, meaning a commit whose message contains `X7 T5` is on
  `x7-transmissions` or on local `main` (`git log --oneline main x7-transmissions | grep 'X7 T5'`).
  Then merge whichever has it into `sheet-metal` and write against `Transmission`, never `Drive`.
  If it is not there yet, skip SM8 and continue with SM9 (see Working rules).
- `machinekit/examples/press-brake/PressBrake extends MachineAssembly`: side frames, bed, ram,
  hydraulic cylinders Y1/Y2 (new `Hydraulic` actuator kind: bore, rod, stroke, phase speeds,
  speed-change point; projectkit format bump), torsion-bar coupling (profile `StopControlled`),
  depth-stop axis, backgauge X (+R) with servo drives and transmissions, fingers with contact
  switches, front sheet supports, light curtain part with a `Signal` port, tool clamps, a
  `ToolLayout`.
- `PressBrakeChecks` in the MachineKit smoke suite: tonnage capacity vs the SM6 formula for the
  bed length; open height and throat clear the tallest tooling and P1–P5 at every stroke
  position; backgauge reach covers the demo parts' gauge distances; BOM.
- `press-brake/materia.project.json` and a Start-page entry showing the machine alone with a
  scripted stroke.

**SM9. Single-bend setup.**
- Independent of SM8: it takes a `PressGeometry` (bed, die-top height, punch line, ram envelope,
  backgauge travel and finger shape, as plain numbers in `machinekit.pressbrake.planning`) filled
  from the reference numbers. SM8 later derives the same `PressGeometry` from the assembly, and a
  check asserts they agree.
- `BendSetup.plan(sheet, state, bend, station, orientation, gaugeEdge, process)`: backgauge X and
  R from the evaluated state in the press frame (SM-D10), depth and overbend from SM6, tonnage
  check, minimum flange check, finger contact points, the part's swing envelope.
- Tests: first-bend X equals the mould-line distance and later bends equal `b + flange difference`
  (FreeCAD's rule) on P2; tonnage refusal on an over-long bend; a short flange refused with the
  named reason.

**SM10. Bend sequence search.**
- `BendSequencer` per SM-D10: states, actions, feasibility, cost, best-first search, diagnostics.
  `SheetCollisionQuery` on KinematicsKit's `CollisionWorld`. If `CollisionWorld` is not on
  `sheet-metal` yet, merge the `collision` branch (CL1 is commit 335ef1b98); if the merge does not
  apply cleanly, abort it, mark SM10 blocked and continue.
- Output `BendProgram`: ordered steps (station, orientation, gauge edge, X, R, depth, overbend,
  grip, regrip/flip markers, expected time).
- Robot grip feasibility is a pluggable check (`GripCheck`): Phase B uses a geometric stand-in
  (gripper box on the chosen wall, clearance against tooling through the swing); Phase C plugs
  in IK.
- Tests: P1–P4 find sequences; P3 contains a flip; P4 uses the gooseneck for its last bend;
  P5 needs a station with gaps; a crafted part with a flange that hits the punch in every order
  returns the collision diagnosis; the search is deterministic.

### Phase C: simulated press and bending cell (after machine-tending MT3, MT5, MT6, MT8, MT10 and the welder merge)

**SM11. Simulated press and its interface.**
- Prerequisites: check that `machine-tending` has landed MT5 (actuators, switches), MT6
  (controllers, `CellSignals`) and MT7's pattern; if not, stop and report.
- Hydraulic ram as a process-driven joint (MT5 style) with phase speeds and the speed-change point;
  depth stop and backgauge on the press controller; finger switches; retract of the fingers as
  the ram passes the clamping point.
- `PressBrakeInterface` (SM-D13) as a pure state machine with pure tests for every interlock and
  timeout.
- Scripted test without a robot: a rigid plate placed against the fingers by script, stroke to
  the clamping point and to depth, the interface's signal sequence and timings match the phase
  speeds within 5 %.

**SM12. The sheet in simulation.**
- First verify weld equality relative-pose updates at runtime in SimKit/MuJoCo (SM-D11); record
  the result here and choose the main path or the fallback.
- `SimulatedSheet`: bodies per wall and bend segment, welds from `SheetState`, free outside the
  press, process-driven bend angle from ram depth during a stroke, springback on release.
- Tests: angle-vs-depth matches `BendProcess` within 0.1°; the clamped wall does not slip on the
  die (< 0.05 mm); a free bent part rests stably on a table; material B ends further from the
  loaded angle than material A.

**SM13. The bending cell, one bend.**
- `machinekit/examples/press-brake/BendingCell`: press, cobot (`CobotArm` class from the reach
  study), vacuum gripper `ArmTool`, blank stack table, centring table, regrip table, outfeed table,
  fence, controller cabinets and wiring.
- Reach study (`BendingReach`): grip poses on P1–P4 at the press through their whole swing, the
  stack, the centring and regrip tables, with 10 % joint margin; start with `Reach1300`, record the
  class and riser here.
- Mission steps: reuse MT10's `signal`, `waitFor`, `moveVia`, oriented `pick`/`place`; add `bend`
  (register to fingers by creeping until both switches make → `request_stroke` → follow streamed
  from the ram position → release or keep → retract) and `regrip`. RobotKit `BendFollow` skill, a
  ProcessKit device for the press handshake, app `MissionPlayer` mapping, scene validation.
- `BendingMission.generate(cell, part, program)` from the SM10 program.
- Acceptance (`ProjectSourceTests.checkPressBrakeCell`, MuJoCo): P1 from the stack to the outfeed;
  angle within ±0.5° after one calibration pass; follow deviation below 1 mm; 0 collisions;
  0 interlock alarms; cycle time reported.

**SM14. Multi-bend parts, regrip and flip.**
- Regrip on the regrip table and in the press at the clamping point; flips through the wrist or
  the regrip table, as the program says.
- Acceptance: P2, P3, P4 unattended, in sequence; each part's bent angles within ±0.5°; the
  program's predicted time within 15 % of the simulated time. P5 once segmented stations work in
  simulation.

**SM15. Measurement and calibration loop.**
- A simulated angle sensor station (or a laser angle sensor on the press) measures each bend;
  results update the `CalibrationTable`; inject UTS variation ±10 % between blanks.
- Acceptance: with calibration, P6 (material B) converges to ±0.5° by the second part; without,
  report the error.

**SM16. Faults and recovery.**
- Double blank (thickness check at the centring table), registration timeout (fingers never
  make), sheet slip in the gripper, angle out of tolerance (part to a scrap position), press alarm,
  e-stop. Each ends in the safe state with a named mission failure; resume at the failed step after
  `robot.fault_reset`.

**SM17. Robot tool changing (later).**
- Powered tool clamps, safety-tang segments that lift out vertically, a tool gripper on the robot
  tool changer, a tool rack, `changeTooling(layout)` mission step, layout verification, and batch
  grouping in the planner to minimise changes.

**SM18. Hardware, Level 2 (later).**
- RKD6 board I/O (step/dir, GPIO in/out), quadrature input for the ram scale, input-triggered
  events (retract and follow keyed to ram position), a press brake deployment profile, homing for
  X, R and the depth stop.
- A retrofit wiring document: start relay through the safety circuit, ram scale mounting, finger
  switches, signal list. Never ram valves (SM-D14).

## Order

```
SM0 ── SM1 ── SM2 ── SM3 ─┬─ SM4 ── SM4b
                          ├─ SM5
                          └─ SM6 ── SM7 ── SM9 ── SM10   (collision CL1 merged)
                         (X7 T5 merged) ── SM8 ──┘ (agreement check with SM9's PressGeometry)
 (MT3, MT5–MT8, MT10 merged) ── SM11 ── SM12 ── SM13 ── SM14 ── SM15 ── SM16
                                                       SM17, SM18 later
```

- Unblocked now: SM0–SM7, SM9, and SM10 once the `collision` merge applies.
- SM8 waits for X7 T5. Phase C waits for the machine-tending steps listed.

## Working rules

- **Blocked steps.** Before each step, check its prerequisite exactly as written. If it is
  missing, do not work around it (no writing against `Drive`, no stand-in signal bus): mark the
  step `blocked: <what is missing>` in the Progress table, commit, and go on to the next unblocked
  step. When no unblocked step is left, re-check every blocked step once (a dependency may have
  landed meanwhile) and do any that are now unblocked. Then stop and end with a short report
  listing each blocked step and the commit or branch it waits for. When resumed, re-check blocked
  steps first.

- Commit after each step (or sub-step) with a message naming the step, e.g.
  `Sheet metal: fold sheets as forward kinematics (SM1)`, and update the Progress table below in
  the same commit.
- Keep every existing suite you touch passing; report pre-existing failures instead of fixing
  unrelated code.
- Diagnose before working around: when the compiler or a native call behaves unexpectedly, read
  its source to find the cause first.
- Each fact lives in one place: geometry in the sheet, conventions derived, numbers from catalogues
  marked assumed where not from a datasheet.
- No code from FreeCAD_SheetMetal (LGPL); formulas and published tables are fine.

## Progress

| Step | State | Commits |
|---|---|---|
| SM0 | worktree created from 1048bf768 | |
| SM1 | planned | |
| SM2 | planned | |
| SM3 | planned | |
| SM4 | planned | |
| SM4b | planned | |
| SM5 | planned | |
| SM6 | planned | |
| SM7 | planned | |
| SM8 | planned | |
| SM9 | planned | |
| SM10 | planned | |
| SM11 | planned | |
| SM12 | planned | |
| SM13 | planned | |
| SM14 | planned | |
| SM15 | planned | |
| SM16 | planned | |
| SM17 | later | |
| SM18 | later | |
