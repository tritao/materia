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

Surveyed 2026-10-03, updated 2026-10-04 on local main 38601ac61, plus the
`machine-tending` and `collision` branches. Since 1048bf768, main gained transmissions X7–X10 and
the robot welder (W0–W4); see "On main since the first survey" below.

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
  undo/redo), `DocumentCodec` (format `cadkit.document`, one supported `VERSION`, dispatches on
  `featureType` strings in its decode and encode paths), `recording/DocumentBuilder`.
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
  `LinearScale`, `PortKind.Signal`.

**On main since the first survey (X7–X10, welder)**
- `MachineAssembly.addTransmission(id, leader, follower, source:Transmission, ...)` with typed
  sources resolved by `TransmissionResolver` into one `TransmissionRelation` (allowances,
  efficiency, stiffness, provenance of assumed values); `supportScrew`, `addBeltPath`, belt
  elasticity and shared belt spans (X10).
- `addMotor(id, joint, motor, driver, margin, ?gearbox)`: motor, driver (`MotorDriver`), supply
  (`PowerSupply`, voltage from the power service graph) and gearbox members are separate parts;
  drivers own microsteps and pulse ceilings (X8). Effective joint limits are compiled from the
  physical assembly (X9a). `addEncoder`.
- Robot welder merged: `ArmTool`, `RobotArm(withCell, ?armTool)`, `robotkit.manipulation.ArmClearance`
  and `robotkit.tool.ConvexSolid` (swept clearance of arm and tool hulls), `ProgramPlanner.shutdown()`,
  weld mission steps, ProcessKit `WeldingPlanRunner`/`WelderProcessDevice`.
- `machinekit/TRANSMISSION_PLAN.md` records the conventions; read it before SM8a.
- **No backward compatibility** (X9b, and the rule in `robotkit/RESTRUCTURE_PLAN.md`): one
  schema version per saved format, older versions rejected with one generic message; bump versions
  when a format changes, regenerate fixtures, delete old paths. No migrations, shims or aliases.
- RobotKit restructuring R0–R6 is done on main (`robotkit/RESTRUCTURE_PLAN.md`; R7 review fixes
  planned, they may still touch skill and endpoint APIs):
  - process-specific skills and simulated process tools live in ProcessKit (`processkit.skill`:
    `WeldSeam`, `WeldRunner`, `Paint`, ...; `processkit.tool`: `WeldArcModel`, `WeldBead`, ...);
    RobotKit keeps generic skills (`GoTo`, `HandlePart`, ...);
  - a robot kind is a `RobotProfile` (`robotkit.profile`) held beside the `RobotModel` in a
    `RobotDefinition`, never a `RobotModel` field;
  - `TrajectoryChunk` is gone: execution plans only; streamed servoing is MotionKit's
    `ServoSession`/`ManipulatorServo` (`motionkit/robot`);
  - package boundaries are checked by `tools/check-robotkit-packages.py`; the maintained gate is
    `tools/robotkit-restructure-suite.sh` (its workspace stage is
    `haxeon workspace test --workspace materia.workspace.json`).
- Welder W5 (woven and multi-pass welds) is on main too.

**On the `machine-tending` branch (not on main yet) and needed here**
- Done there: `BallScrew`/`BallNut`/BK–BF supports and HGR rails (MT1), `BenchMill` (MT2),
  `CobotArm` size classes `Reach500/850/900/1300` (MT3; tending chose Reach850 on a 550 mm riser),
  enclosure and reach study (MT4).
- Also done there (2026-10-04): pneumatic actuators as process-driven joints, `addSwitch`
  (`joint_switch` sensors), `addPresence` (MT5). Planned there: two controllers per cell +
  `CellSignals` (MT6);
  the machine-side `RobotInterface` state machine pattern (MT7); a real parallel gripper and the
  `gripper` robot-tool kind (MT8); mission steps `signal`/`waitFor`/`moveVia` (MT10).
- Collision: `kinematicskit` `CollisionWorld` on coal exists only on branch `collision` (CL1,
  335ef1b98, unmerged). No collision-aware planner (CL6).
- SimKit: `hold_object`/`release_object`, kinematic actors, MuJoCo weld equalities (used for
  closures in `mujoco_backend.cpp`). No deformable bodies.
- RobotKit: no regrasp, no force control. Streamed Cartesian servoing exists (MotionKit Lane D5,
  now `ServoSession`).
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
- SM3k separates each bend's nominal straight attachment datum from the material contour.
  A cut can interrupt a seam without moving its fold axis. Authored developed-region tools
  derive bend patches from the current allowance on every rebuild; exact line/arc subtraction
  permits multiple patches joined through the walls. Analytic seam intervals must agree on
  both domains; missing attachment material and mismatched intervals are diagnosed. Every
  retained patch attaches to a wall, and each active bend retains a material bridge; detached
  islands and completely severed bend strips have distinct diagnoses.
  Cuts are developed blank profiles in the selected wall frame, then folded with the graph.
  This defines cross-bend cut semantics consistently with the manufactured flat pattern.
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
- SM1 clarification: at intermediate angle `a`, the neutral radius is `BA / |a|`,
  and the inner radius is `BA / |a| - n`, where `n = BA / |target| - R`.
  The side-A radius is that inner radius for up bends and inner radius + T for down bends.
  At the target this recovers R; at zero the strip is a rectangle of length BA.
  This replaces a fixed intermediate radius, which cannot conserve neutral length while
  changing angle. `BendZone` supplies both the strip geometry and child attachment frame.
- A rip cuts the child-side seam of its bend strip. It removes the spanning-tree/closure
  connection, while retaining the strip attached to wall A: a zero-gap cut must not delete
  the bend allowance's material. Rips that disconnect a wall from the root are diagnosed.

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
- SM1a validation: input R/T rows must be nonnegative and strictly increasing, and ANSI K
  must be in [0, 0.5] (DIN [0, 1]); malformed tables are refused rather than reordered or
  repaired. Table inputs are copied. Sheet lengths use mm, angles radians and mechanical
  strengths/modulus MPa, so SM6 will not carry implicit unit conversions.

**SM-D4. Solids are generated from the graph and named by it.**
- Each wall is its outline extruded by `T` along −Z of its frame. Each bend is the rectangle
  `T × width` revolved by θ about the bend axis. Everything is fused.
- Faces are named with `cad_shape_seed_names` from graph identity:
  `wall:<id>:a`, `wall:<id>:b`, `wall:<id>:edge:<loop>:<segment>`, `bend:<id>:inner`,
  `bend:<id>:outer`, `bend:<id>:end:0|1`. Wall and bend ids are permanent (created once, never
  renumbered), so names survive parameter edits by construction.
- SM3c distinguishes a segment's authored identity from its current loop/index. Pure graph
  outlines default to the numeric loop/index roles above; sketch-backed bases use
  `wall:<id>:edge:sketch:<entity-id>`. Contour rotation and insertion therefore cannot reuse a
  surviving edge's name for another face. Exact line/arc records and their identities reverse
  together when normalising winding. Native names stay opaque: after `TopologyResolver`
  resolves a persistent face, `SheetEdgeSelection` matches its geometry against analytically
  derived straight wall-side faces, refusing zero or multiple matches. Renamed surviving
  entities use the existing conservative geometric fallback; disappeared edges stay unresolved.
- Fusing many touching solids is acceptable to start. If it is slow or fragile, a native
  builder that assembles faces with known topology (planar and cylindrical faces sharing exact
  edges, then sewing) replaces it later, with the same names.
- SM2 seeds the logical identifiers above; the core's existing naming grammar percent-escapes
  colons in seed identifiers. They remain opaque in Haxe. Joining strip caps have logical roles
  `bend:<id>:join:a|b` (normally consumed by the fuse). Extrusion side faces are matched by
  native history and exact input-edge identity: history accessors return bare topology without
  names, so `IsSame` on the still-owned named profile recovers the authored segment.
- SM3n handles a trimmed closed hem's collapsed inner axis without inventing a radius or
  gap. Hole-free Y-monotone radial cut sectors derive analytically from authored tools and
  current strip dimensions. The builder unites wall prisms and the complete closed-hem strip
  before subtracting those sectors: the axis contact is interior to that union, whereas a
  standalone trimmed strip can be non-manifold there. This is solid generation from the graph,
  not editing a predecessor feature's solid. Partition boundaries use authored vertices and
  arc extrema; side surfaces reach the axis with valid degenerate edges. Grid partitioning
  does not approximate any profile curve.
- A rule that gives zero bend allowance for a nonzero bend cannot create a flat strip and
  is refused by the solid builder with `sheet.zero-bend-allowance`; no artificial gap is added.

**SM-D5. Sheet features form their own chain inside the document.**
- `SheetFeature extends Feature` and exposes `sheet():Sheet` alongside the folded `shape`.
- SM3a publication: graph and solid are staged in the same recompute; graph publication is an
  assignment-only `commitEvaluation`, and any failed recompute discards the pending graph.
  `sheet()` returns an isolated editable copy, sharing immutable records only. Downstream
  features use `sheetForEvaluation(context)` to consume the graph corresponding to the staged
  solid, rather than the previous committed graph. The context must belong to the same document.
- SM3b bases consume the constrained sketch's staged exact `ProfileGeometry`, shared with
  its native face builder: validation, nesting and unit conversion have one implementation.
  A base requires one connected region; holes and exact arcs are retained. Its root placement
  comes from the sketch workplane. For a side-B profile, the side-A frame is shifted +T along
  that same normal, preserving the sheet-wide material-toward-minus-Z convention.
  The thickness parameter owns T; a material snapshot owns its identity, explicit rule and
  optional mechanical data and explicit default/minimum radius ratios. Project material-library
  integration remains SM6. SM3e changes the base record to a typed wire payload and bumps the
  document format to version 12. SM3g adds typed relief sizing and dimensions, advancing the
  single supported format to version 13; no older sheet reader remains.
- SM3d fixes flange-position semantics explicitly: `MaterialInside` keeps the virtual outside
  sharp corner at the picked boundary (parent tangent shifts inward by OSSB), `MaterialOutside`
  keeps the virtual inside sharp corner there (shift inward by R·tan(|angle|/2)), `BendOutside`
  keeps the parent tangent itself there, and `Offset` moves that tangent by an explicit signed
  distance, positive outward. Sharp alignments are refused above 179 degrees. These are derived
  from the two intersecting sheet surfaces, for either bend direction, rather than copied code;
  see the [published flange-alignment vocabulary](https://cad.onshape.com/help/Content/PartStudio/sheet_metal_flange.htm).
  `WallOutlineEditor` intersects neighbouring lines/circles analytically and retains segment
  identities; parallel neighbours, absent circle intersections and tied branches are diagnosed.
  `SheetFlange` edits an isolated graph. Wall edits remap existing bend attachments by authored
  identity and restore wall and bend records together if validation fails.
- SM3e records sheet payloads with typed `@:wire` records and numbered enums. Geometry
  accesses nullable arc centres and bend radii through guarded typed functions; sheet code
  contains no unchecked casts. The generic document lookup boundary uses checked class narrowing.
- Authored output wall/bend ids belong to each feature; a root-owned monotonic identity ledger
  reserves them independently of rebuilt topology counts and saves its counters with the base.
  A restored feature claims its saved ids, diagnosing duplicate ownership or invalid counters.
  Upstream topology growth cannot renumber downstream walls, bends or native face names.
- Material-default radius is an explicit ratio multiplied by current thickness; a missing
  default diagnoses `sheet.no-default-radius`. Minimum radius is a separate optional property,
  never substituted for the preferred radius. Gaps trim the attachment width; extensions add
  child-wall material beyond it without widening the bend. Exact segment identities persist
  through attachment subdivision.
- SM3f subtracts exact planar regions by analytic line/circle intersections, split intervals,
  material-side classification and directed boundary assembly. It retains line and arc curves,
  handles existing holes and coincident boundaries, and diagnoses empty or disconnected walls.
  Untouched segment identities survive; multiple surviving fragments use authored intersection
  roles and infinite-curve branches, never contour indices or coordinates. This is shared
  geometry for reliefs and wall cuts; cuts through bend zones still require exact mapped trims.
- SM3g specifies bend-end relief placement: each slot occupies `width` along the attachment
  after the explicit end gap and cuts `depth` inward into the parent. The bend attachment is
  shortened by that width at each end; the child extends by the same width to preserve the
  requested flange width. Rectangular slots have square ends; obround slots have semicircular
  inward caps of radius width/2 and require depth >= width/2. Dimensions are either explicit
  millimetres or explicit thickness ratios, derived on every rebuild. Existing bend attachments
  still validate after parent edits; a relief cannot silently remove another bend's edge.
- SM3h specifies hem dimensions as a straight return length and fold side. Closed hems
  derive R = 0 and a half turn; open hems derive R = gap/2 and a half turn. Teardrops
  use an explicit bulb radius or the material default, and solve
  `R*(1-cos(theta)) + length*sin(theta) = gap` for the return above 180 degrees.
  The gap is measured between the concave sheet surfaces at the return tip; a return tip
  whose other surface penetrates the parent is diagnosed. No clearance gap or minimum
  radius is invented. Bend validation now accepts finite magnitudes below one full turn;
  the 179-degree sharp-dimension limit remains. Native closed contact can consume faces
  where layers touch, so only surviving faces retain their names across that contact event.
  `SheetEdgeFeature` owns shared persistent picks, graph attachment matching and output-id
  reservations for both flanges and hems. `HemSpec` saves only each kind's authored values:
  closed has no gap/radius parameter, open owns a gap, and teardrop owns a gap and optional
  explicit radius. The new hem feature record is additive to document version 13.
- SM3i rip selection is an authored permanent bend id, independent of contour order or native
  face names. The feature consumes the staged source graph, cuts that bend's child-side seam,
  retains its allowance strip and delegates identity ownership to the same sheet root. Missing,
  already-ripped and disconnected selections retain the core's precise diagnoses. The new
  typed rip record is additive to document version 13. A zero-gap seam changes graph
  connectivity; a coincident folded seam need not create a positive-width void in the native
  volumetric union. Flattening uses the cut graph rather than that union's face adjacency.
- SM3j wall cuts name a permanent wall id and sheet side and consume a staged constrained
  sketch on that side. The sketch may use its existing persistent support-face reference; its
  resolved plane must be parallel and at the selected side's exact offset. `SheetProfile` maps
  solved lines/circles into the wall frame and normalises winding with authored identities,
  shared with sheet bases. Subtraction edits the graph, never the native solid. A cut reaching
  an attached bend's developed allowance rectangle reports `sheet.cut-bend-zone` until the
  exact bend-region mapping sub-step lands; refusing the cut prevents an uncut strip or dangling
  attachment from being published. This is a partial SM3 sub-step, not completion of bend cuts.
  The new typed cut record is additive to document version 13.
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
- SM4a derives exact unions of mapped wall/bend material domains with `OutlineBoolean`
  (the renamed generalisation of `OutlineDifference`, without an alias). Boundary subdivisions
  retain authored identities; acceptance compares material geometry with an independent outline,
  rather than counting collinear subdivisions. Domain owners remain typed graph identities.
  Fold lines mark only material-covered intervals; effective bend length is their total length,
  while nominal attachment width is separately available. Bend records are sorted by permanent id.
- Grain is an authored undirected angle in the root wall frame, normalised modulo pi. It is
  introduced on `SheetMaterial` in SM4a because flat checks need it before SM6's library work.
  The exported plane's X axis follows grain, so an arbitrary rolling angle can map exactly to
  ManufacturingKit's width-aligned rectangular-stock constraint. Ungrained parts retain the
  root plane. Document version 14 saves the typed grain snapshot; version 13 is rejected,
  runtime fixtures use current code, and the named-thickness builder preserves grain.
- DXF writer: ASCII R12 (AC1009) with `LINE`, `ARC`, `CIRCLE` and `TEXT` entities only
  (`LWPOLYLINE` is not R12), layers `OUTLINE`, `BEND_UP`, `BEND_DOWN`, `ETCH`, `TEXT`. SVG writer
  with the same layers. Both pure Haxe in `cadkit.drawing`.
- SM4e evaluates the union of all material domains together, rather than constructing
  temporary wall-only contours that can branch at point contacts before bend strips are added.
  The same exact line/arc membership assembly retains authored boundary identities and origins.
- `SheetChecks.of(sheet, ?tooling)` returns a `SheetCheckReport` with one flat snapshot and typed findings,
  measured units and exact offending geometry. Positive domain intersections above 1e-8 mm²
  diagnose flat overlap. The assumed `0.7V` minimum is a straight leg from the tangent attachment,
  measured as the minimum first outer-profile exit over material-bearing seam intervals; holes
  are covered by their separate clearance rule. Vertex/cardinal events split the exact contour
  into monotone branches, including tapered legs and circular notches. A ripped strip has no
  straight return: its detached wall cannot supply one. An unmeasurable seam is diagnosed,
  never treated as infinite depth. SM6 replaces the former 4T placeholder with V=8T for stock
  up to 3 mm, the Process rule's selected tooling, or an explicit tooling argument; thicker
  stock without tooling is diagnosed.
  Hole and surviving cut-feature boundaries use exact minimum distance to the full developed
  fold line and the assumed `2.5T + R` threshold; explicit relief origins are exempt. A fold-line
  endpoint strictly inside the united blank, more than 1e-8 mm from its boundary, needs an opening.
  Material radius minima are explicit; grain warnings include the 15-degree boundary. These
  findings are separate from fatal graph validation and retain their measured geometry.
- SM4d carries typed `BoundaryOrigin` values alongside segment identities: authored,
  cut(owner), or relief(owner). Booleans retain surviving source purposes and give newly
  introduced boundaries the operation's explicit purpose. Relief cuts supply `Relief`, and
  ordinary through cuts supply `Cut`; no purpose is inferred by parsing an opaque identity.
  Mapping/reversal, supporting-edge edits, gap splitting, radial partitioning and flat union
  preserve origins. Origins are derived anew from the feature chain on recompute and decode,
  so they add no saved payload or schema version. Design checks can inspect actual surviving
  flat boundaries and exempt reliefs without guessing from shape or edge-name text.
- SM4c supplies one typed `Drawing2D` entity model to both writers: exact curves, explicit
  manufacturing layers and upright annotations. Grain arrows lie on exported X. R12 uses
  negative-Z OCS for clockwise arcs/circles, preserving hole winding rather than reversing
  the contour. Its reflected OCS X and unchanged Y follow Autodesk's
  [arbitrary-axis rule](https://help.autodesk.com/cloudhelp/2015/ENU/AutoCAD-DXF/files/GUID-E19E5B42-0CC7-4EBA-B29F-5E1D595149EE.htm);
  centre, radius, angle and extrusion groups follow the
  [ARC reference](https://help.autodesk.com/cloudhelp/2018/ENU/AutoCAD-DXF/files/GUID-0B14D8F1-0EBA-44BF-9108-57D8CE614BC8.htm).
  R12 text accepts printable ASCII only so authored newlines cannot corrupt group records.
  SVG reflects Y and changes arc sweep flags accordingly; circles remain circles, and XML
  annotation text is escaped. Annotations have explicit dimensions for drawing bounds.
- `manufacturingkit` gets a `SheetPartRequirement` from a flat pattern: blank = bounding box
  (rectangular stock only, until nesting exists), grain constraint from the authored rolling
  direction with export X aligned to grain, thickness and material identity from the sheet.
  Bend lines check the authored grain; they cannot determine a stock's rolling direction.
  Caller-owned order identities, revisions, quantity and explicit dimensional tolerances enter
  through `SheetBlankOrder`; neither the geometry nor the material invents production context.

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
- SM6a distinguishes final inner radius `Rf` from loaded radius `Ri`. The assumed natural
  radius is `Rf`; invert the physical springback branch to obtain `Ri`. If that would lie below
  the punch tip, the punch governs `Ri` and its released radius is derived from the same relation.
  The neutral-axis assumption for this process is explicitly `T/2`, as in the springback formula;
  it is distinct from an authored K-factor rule. Signed angles retain their bend direction.
- With `beta = abs(loaded angle)/2`, `a = V/2`, shoulder centre height `-rd`, and loaded
  inner radius `Ri`, the common tangent gives `h*cos(beta) + a*sin(beta) = Ri + T + rd`.
  Punch-bottom depth below the die top is `rd + Ri - h`. For a tip smaller than `Ri`, internal
  tangency at the bottom shifts the punch centre by `Ri-rp`; its bottom position is unchanged.
  Tests independently recover the tangent angle from the centre distance and numerically solve
  depth at every degree from 20 to 150 for nine thickness/radius combinations (1 micrometre
  tolerance). Unsupported tool contact and included-angle limits have named diagnoses.
- A `CalibrationTable` keyed by (material lot, thickness, punch, die, angle band) stores
  measured corrections (depth offset, final inner radius) that override the analytic values.
  SM6b uses exact thickness and tool identities and half-open magnitude bands `[first, last)`;
  overlapping bands for one key are rejected. A prediction applies calibration only with an
  explicit lot. Its measured final radius derives a fresh loaded radius and analytic depth,
  then the measured depth offset is added. A measured radius incompatible with the punch is
  diagnosed rather than clamped. Tables and entries are immutable; no nearest-key fallback.
- With a `Process` bend rule, the flat pattern is computed for the chosen tooling.

**SM-D9. The press brake is an ordinary MachineKit assembly.**
- Ram on prismatic joints driven by two hydraulic cylinders. The torsion bar of the reference
  machine is a coupling Y2 = Y1 (X5 coupling). A closed-loop Y1/Y2 machine is a variant without
  the coupling and with a linear scale per side.
- Hydraulic cylinders are actuators on the same process-driven-joint mechanism as MT5's
  pneumatic ones (one mechanism, a second actuator kind; never a parallel implementation): force
  `p·A`, speed limited per phase (approach, working, return) by pump flow, the phase change at the
  speed-change point.
- The mechanical depth stop is its own axis (motor + screw) on stop-controlled machines.
- Backgauge X (and optional R) are axes with servo drives and X7 transmissions; fingers are parts
  with contact switches (MT5 `addSwitch`).
- The reference gravity-return finger has a passive hinge and an explicit critically damped
  mechanical stop. Its assumed 1 mrad compression stores the full nominal gravity-drop energy:
  `k = 2 m g Δz / δ²`, `τ = sqrt(Ihinge/k)`, using the box inertia about its hinge and
  standard gravity 9.80665 m/s². Stock contact uses the same assumed elastic response. These
  are reference simulation assumptions, not manufacturer measurements. Shape contacts and
  joint stops derive physics substeps so each positive time constant spans at least two physics
  steps. A stock-contact test advances to the contact datum, and verifies fault-free runtime
  observations throughout gravity opening, contact closure and release.
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
- Phase-B search combines the required flip/regrip preparation and a checked bend into one
  transition. It charges the preparation and retains its markers in `BendProgram`; handling
  between setups is assumed free of motion constraints at this stage. Phase C must validate
  the actual handling paths and regrip support through its robot/process adapters.
- The finite setup domain uses installed segment/material-seam endpoint and horn-relief events, finger datums
  and the midpoints between adjacent events. Search exhaustion proves no sequence in that
  explicit domain, not infeasibility over continuous placements. Node/setup budget exhaustion
  is a separate result and never a physical collision diagnosis.
- Search: best-first with an admissible heuristic: sum each remaining bend's minimum hydraulic
  cycle, mandatory handling and checked withdrawal across installed stations, plus the minimum
  required stock-face flip. Registration, station moves and regrips are omitted from this bound.
  If no
  sequence exists, return the deepest failing states with named reasons ("flange B3 shorter than
  minimum 8.4 mm at V12", "flange W2 hits punch at 62°").
- Collision goes through a narrow interface `SheetCollisionQuery`. Every piece involved is a
  convex prism: walls decomposed into convex pieces of their outline, bend zones into a few
  segments, tool segments (gooseneck profiles split into convex parts), and the machine as a few
  boxes. The first implementation is an exact separating-axis test for convex prisms inside the
  planner package, recorded as interim. When coal's collision world (collisionkit, CL2–CL4) is on
  main, switch the implementation and delete the interim one.
- Curved approximations retain their geometric error. Prism overlap alone cannot diagnose a
  physical collision at intended rounded contact. Refine those pairs against the exact authored
  line/arc tool sections and evaluated bend sectors where their extrusion axes admit an exact
  section query. Clip to the real bed interval and preserve developed cuts. Unsupported axes or
  curved wall approximations request the general error-aware query; never exempt a whole active
  bend, wall or tool body. This refinement remains when the interim prism SAT is replaced.
- MachineKit must not depend on RobotKit, so the planner does not use `robotkit.tool.ConvexSolid`.
- An uncut bend also admits an exact query against any convex prism: transform into its own
  extrusion frame, clip the obstacle to the finite bend width, then intersect its projection
  with the exact flat/annular material section. This works for arbitrary bend-axis directions.
  Do not use that projection for a cut bend whose material varies along the axis, or treat an
  approximate tool prism as exact material.
- Exact directional support intervals can also certify bend/tool separation for arbitrary
  extrusion axes. The bend's radial and along-axis endpoint coordinates and stationary angular
  events give its exact support; the original line/arc tool section gives the tool support.
  Partition the tool cap into exact regions before checking separating directions, preserving
  their union. A cut bend uses the uncut strip as a containing bound. A successful separating
  direction proves clearance; failure to separate still requests the material query and is
  never a collision verdict. This removes isotropic chord-error ambiguity at planar contact
  without exempting any rounded material or tool body.
- General curved-body clearance uses unit-axis support gaps: a gap larger than the sum of
  the boundary errors proves separation. Collision requires overlap of physical material,
  proved by exact section refinement or face-inset prisms. The error contract includes a
  boundary correspondence preserving material interior, not just a Hausdorff distance.
  Insetting every piece face by its error keeps material inside the authored body; artificial
  decomposition faces only shrink that certificate. Cases between the two certificates
  remain an explicit unresolved result requiring refinement, never an assumed collision or
  clearance. Named stroke failures retain typed owners, sample index and active bend angle.
- Backgauge withdrawal is an explicit `BackgaugeStroke`: register at the setup X/R, hold the
  flat stock at first punch contact, sample withdrawal of X, then bend at the withdrawn X.
  Check fingers and carriers through both phases, within axis travel. The caller supplies
  withdrawal distance; no universal retract distance or finger-contact exemption is inferred.
- Exact wall/prism refinement transforms the obstacle into the wall's extrusion frame, clips
  it to the finite stock thickness, and intersects its projection with the unchanged exact
  line/arc outline, including holes. This supports arbitrary wall orientations and surface
  tangency without exempting the gripped wall. Phase-B `GripCheck` uses an explicit rectangular
  suction patch and outward gripper box; the complete patch must lie on wall material. Its
  wall-local grip frame follows FK and points out of the selected stock face. Check every
  tool, machine volume and sheet body through withdrawal and the sampled stroke. Phase C
  adds robot reach/IK through the same function-only interface.

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
  (`SheetState` with the angle from `BendProcess`). The robot runs it as a streamed servo task (`ServoSession`)
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

## Reference numbers (fixed in SM7, SM8a, SM13; assumed until checked against a spec sheet)

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

SM1 fixes the unspecified demo details in `SheetDemoParts`: reference R = 1 mm and ANSI
K = 0.38 are assumed until process-selected tooling in SM6. P3 is 120 long with a 60 outside
web and opposed 40 outside legs (78.5 overall height). P4 is 120 long with a 60 outside crown,
40 height and 20 outside lips (97 overall width, accounting for shared thickness). P5 has
2 mm square corner reliefs. All straight leg lengths derive from the drawing dimensions and
setbacks; these define new fixtures, with no existing baseline-number changes.

## Steps

### Phase A: sheet metal design (CadKit; can start now)

**SM0. Base and setup.**
- Worktree `/home/joao/dev/materia-worktrees/sheet-metal`, branch `sheet-metal`, rebased on local
  main 38601ac61 on 2026-10-04. Commit on this branch; do not create other branches.
- Populated at the pins: `haxeon` 8521d494 (with `.tools/` and `out/haxeon_runtime.hdll` copied
  from the `machine-tending` worktree, same pin) and `nativekit` 88cb8f88. The other submodules are
  empty; `/home/joao/dev/materia-cache/claude-scratch/sm-populate.sh` fills them from local checkouts
  that have the pinned commits (needed for the full gate). After merging a newer `main`, re-check
  `git submodule status` and move any changed pin the same way (fetch the sha from a checkout that
  has it, never `--shared`). Write guarded paths in destructive commands (`"${WT:?}"/...`).
- CadKit native: `cadkit/core` is unchanged between origin/main 3fd620e4 and 38601ac61, so use
  `CADKIT_BUILD_ROOT=/home/joao/dev/materia-worktrees/topological-naming/cadkit/build/debug`
  until native work is needed. SM3k prepares the local prebuilt-OCCT build for trimmed bend
  regions before SM5; the shared checkout's build lacks `cad_shape_stamp_names`.
- When native work needs a build (SM3 mapped bend regions, then SM5): configure CadKit in the worktree with
  `CADKIT_OCCT_DIR=/home/joao/dev/materia-cache/occt-f1dc4efb-release` set **before** the first
  configure (`-DCADKIT_BUILD_OCCT=OFF`). Never build OCCT.
- Tests: `cadkit/scripts/test-haxeon` with `CADKIT_SMOKE_ENTRY=<entry>`. Add a
  `SheetMetalSmoke.hx` entry in `cadkit/haxe/tests/` and keep `HaxeonSmoke` passing.
- Gates: `/home/joao/dev/materia-cache/claude-scratch/sm-suite.sh <tag>` (quick: cadkit,
  SheetMetalSmoke, projectkit, manufacturingkit, machinekit) and `sm-suite.sh <tag> full` (adds
  `haxeon workspace test` over `materia.workspace.json`, the app build and the project-source
  tests). Phase C steps that touch RobotKit or ProcessKit also run
  `tools/robotkit-restructure-suite.sh <build-root>` (all stages) at their final commit.
- Disk: check `df -h /` first (12 GB free on 2026-10-04). Populate other submodules only when a
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
  in `DocumentCodec`, builder helpers in `DocumentBuilder`). Saved feature additions extend the format: bump `DocumentCodec.VERSION` and regenerate
  fixtures and examples with the current serializers, with no reader for the old version:
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
    land after the rest; record it as a sub-step. Gap is the perpendicular stand-off of a
    non-overlapping flange end from its mate's full thickness slab. An overlap names the flange
    whose end extends beyond its mate's far face by the stated perpendicular overlap; the mate
    retains the stand-off. Oblique corners account for both thickness slabs rather than intersecting
    zero-thickness surfaces. SM3o is scheduled after SM6 and before phase B,
    so the required feature chain can proceed to flat patterns while corner closure remains explicit.
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
- `SheetBlank.requirement(flat, order)` → `manufacturingkit` `SheetPartRequirement`.
  Material and thickness come only from the flat snapshot; typed `SheetBlankOrder` supplies
  caller-owned requirement/part identities, revisions, quantity and explicit dimensional tolerances.
  This completes the original signature, which omitted the production context required by the DTO.
  Grain-aligned export permits rotation 0; ungrained stock permits 0/90 degrees. The pure DTO
  dependency from CadKit to ManufacturingKit is acyclic and introduces no inventory state.
- Tests: DXF parses back (write a minimal reader in the test) to the same loops and bend lines;
  P5 flat pattern matches a hand-computed outline; each check fires on a crafted bad sheet and
  stays silent on P1–P5.

**SM4b. Editor integration.**
- Follow how `ExtrudeFeature`/`FilletFeature` are exposed in `app/src/EditorScene.hx` and
  `app/src/editor/ScenePropertyProvider.hx`.
- Create sheet features from the editor (base from a sketch, flange from a picked edge face),
  edit their parameters, toggle the viewport between folded and flat, show bend lines and the
  bend table, export the flat pattern as DXF/SVG.
- Before SM6 supplies the material library, the inspector requires an explicit material identity
  and ANSI K factor for a custom stock. No silent rule is created. Stock/flange form input is
  transient; the typed feature snapshot remains the saved source of truth. Flat view is session
  state and derives a fresh solid on publication, without modifying the document or its history.
  Export-plane fold intervals are mapped back to the sheet plane for the viewport overlay.
  Inspector/table/overlay share an immutable flat snapshot per publication and selected feature;
  publication invalidates it. P2 outside dimensions are verified against analytic graph bounds: the
  existing native bounds query includes triangulation deflection after viewport tessellation.
- Tests: an app test in the style of `app/tests/src/tests/CadPlateWorkflowTests.hx` builds P2
  through the editor commands and exports a DXF.

**SM5. Convert a solid into a sheet.**

Deferred by user direction on 2026-10-05: prioritize authored sheets, the process model and
the press brake. SM5a remains available; recognition and STEP conversion remain incomplete.
This changes the implementation order, not the required correctness of a future conversion.
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
- `MaterialDef` gains `mechanical {uts, yield, youngs}` (optional: not every material is
  structural) and `sheet {kFactorTable, defaultRadiusRatio, minRadiusRatio, grain}`
  (optional: only sheet materials)
  in `projectkit`. Extend the current schema, bump its version if the codec requires it, and
  regenerate fixtures and example projects; no compatibility path (repo rule).
- `BendRule.Process(tooling)`: flat pattern for a chosen punch/die.
- Tests:
  - natural radius, tonnage and minimum flange against the formulas for material A at V8/V12/V16;
  - springback: material B overbends more than material A for the same radius ratio; the solved
    loaded angle unloads to target within 0.01°;
  - depth: closed form equals the numeric three-circle construction within 1 µm over 20°–150°;
  - a calibration entry overrides the analytic depth and radius.

### Phase B: the press brake and bend planning (MachineKit; required dependencies are on main)

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

**SM8a. The press brake assembly (unblocked: X7–X10 are on main).**
- `machinekit/examples/press-brake/PressBrake extends MachineAssembly`, in the cnc-router idiom:
  side frames, bed, ram on prismatic joints, torsion-bar coupling Y2 = Y1 (profile
  `StopControlled`), depth-stop axis (motor + screw), backgauge X (+R), fingers, front sheet
  supports, light curtain part with a `Signal` port, tool clamps, a `ToolLayout`.
- Drives through main's APIs: `addTransmission` with typed `Transmission` sources, `supportScrew`,
  `addMotor(id, joint, motor, driver, margin, ?gearbox)` with `ServoMotor` + `MotorDriver` for the
  backgauge and depth stop, `addEncoder`, `LinearScale` on the ram. Backgauge X is a ball screw if
  MT1's `BallScrew` is on main by then; otherwise a timing belt from main. Record which, and
  switch to the ball screw when it lands.
- The hydraulic cylinders are parts (bore, rod, stroke) mated to the ram; their actuation is SM8b.
- `PressGeometry.of(pressBrake)` derives SM9's plain-number geometry from the assembly, and a
  check asserts it equals the reference `PressGeometry`.
- `PressBrakeChecks` in the MachineKit smoke suite: tonnage capacity vs the SM6 formula for the
  bed length; the structural mount opening accepts the tallest tooling throughout the nominal stroke;
  conservative P1–P5 swing envelopes clear the ram beam and rear throat throughout that stroke;
  backgauge reach covers the demo envelopes; BOM. Installed tool contact/collision feasibility
  belongs to SM9/SM10, since nominal mechanical travel includes positions beyond a safe tool stroke.
- `press-brake/materia.project.json` and a Start-page entry showing the machine alone (joints
  moved by the editor, no actuation yet).

**SM8b. Hydraulic actuation.**
- Prerequisite: MT5's pneumatic actuators and process-driven joints are on local `main`
  (`git log --oneline main | grep 'machine tending MT5'` shows the MT5 completion commit, and the
  `Pneumatic` actuator kind exists in projectkit). Until then this step is blocked.
- Add the `Hydraulic` actuator kind beside `Pneumatic` on the same mechanism (bore, rod, stroke,
  supply pressure, phase speeds, speed-change point), `addCylinder` for the ram cylinders with a
  valve block part, and the switches (top, speed change, bottom, finger contacts) through
  `addSwitch`. Bump the projectkit format and regenerate fixtures.
- `PressBrakeChecks` in the MachineKit smoke suite: tonnage capacity vs the SM6 formula for the
  bed length; the structural mount opening accepts the tallest tooling throughout the nominal stroke;
  conservative P1–P5 swing envelopes clear the ram beam and rear throat throughout that stroke;
  backgauge reach covers the demo envelopes; BOM. Installed tool contact/collision feasibility
  belongs to SM9/SM10, since nominal mechanical travel includes positions beyond a safe tool stroke.
- `press-brake/materia.project.json` and a Start-page entry showing the machine alone with a
  scripted stroke.

**SM9. Single-bend setup.**
- It takes a `PressGeometry` (bed, die-top height, punch line, ram envelope, backgauge travel and
  finger shape/contact datums, as plain numbers in `machinekit.pressbrake.planning`) rebuilt
  from the installed SM8 assembly. An independent reference fixture checks those dimensions.
- Manufacturing acceptance parts take explicit stock and derive each signed bend radius and
  setback from it, retaining the default reference parts. Manual lateral finger spacing is an
  authored assembly setup; planning contact datums come from the physical connectors.
- Loaded stroke evaluation uses `Sheet.evaluateStroke`: only the active bend may exceed its
  design angle, bounded by a compatible typed process result. The authored target, radius and
  neutral length remain unchanged; incompatible process radii or allowances are diagnosed.
- `BendSetup.plan(sheet, state, bend, station, orientation, gaugeEdge, process)`: backgauge X and
  R from the evaluated state in the press frame (SM-D10), depth and overbend from SM6, tonnage
  check, minimum flange check, finger contact points and press-frame stroke poses/bounds.
  The sampled bounds serve broad-phase rejection; SM10 checks actual convex geometry through
  those stroke poses before declaring a collision-free setup.
- Tests: first-bend X equals the mould-line distance and later bends equal `b + flange difference`
  (FreeCAD's rule) on P2; tonnage refusal on an over-long bend; a short flange refused with the
  named reason.

**SM10. Bend sequence search.**
- `BendSequencer` per SM-D10: states, actions, feasibility, cost, best-first search, diagnostics.
  `SheetCollisionQuery` with the interim exact convex-prism test (SM-D10); do not merge the
  `collision` branch.
- Output `BendProgram`: ordered steps (station, orientation, gauge edge, X, R, depth, overbend,
  checked backgauge withdrawal, grip, regrip/flip markers, expected time).
- Robot grip feasibility is a pluggable check (`GripCheck`): Phase B uses a geometric stand-in
  (gripper box on the chosen wall, clearance against tooling through the swing); Phase C plugs
  in IK.
- Tests: P1–P4 find sequences; P3 contains a flip; P4 uses the gooseneck for its last bend;
  P5 uses gapped punch/die rows without end-relief horns; a crafted part with a flange that hits
  the punch in every order returns the collision diagnosis; the search is deterministic.

### Phase C: simulated press and bending cell (after machine-tending MT3, MT5–MT8 and MT10 are on main)

**SM11. Simulated press and its interface.**
- Prerequisites on local `main`: SM8b done, and machine-tending MT6 (controllers, `CellSignals`)
  and MT7 (`RobotInterface` pattern) merged. Otherwise blocked.
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
- Prerequisites on local `main`: SM11, SM12, and machine-tending MT3 (`CobotArm`), MT8 (gripper
  kind) and MT10 (mission steps) merged.
- Reach study (`BendingReach`, like MT4's `TendingReach`): grip poses on P1–P4 at the press through
  their whole swing, the stack, the centring and regrip tables, with 10 % joint margin; start with
  `Reach1300` (the bed and the tables spread wider than the tending cell), try `Reach850` on a
  riser, record the class and riser here.
- Mission steps: reuse MT10's `signal`, `waitFor`, `moveVia`, oriented `pick`/`place`; add `bend`
  (register to fingers by creeping until both switches make → `request_stroke` → follow streamed
  from the ram position → release or keep → retract) and `regrip`. The `BendFollow` skill and the
  press-handshake device live in ProcessKit (`processkit.skill`, `processkit.tool`, beside the
  welder's), per the restructured layout; RobotKit only gains generic pieces, and only if the
  package checker allows them. Follow streams through MotionKit's `ServoSession` on execution plans.
  App `MissionPlayer` mapping and scene validation as for the weld step.
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
                          └─ SM6 ── SM7 ─┬─ SM9 ── SM10
                                         └─ SM8a (agrees with SM9's PressGeometry)
                              (MT5 on main) ── SM8b
   (SM8b + MT6, MT7 on main) ── SM11 ── SM12 ── SM13 (MT3, MT8, MT10 on main) ── SM14 ── SM15 ── SM16
                                                       SM17, SM18 later
```

- Unblocked now (2026-10-04): SM0–SM7, SM8a, SM9, SM10.
- SM8b waits for MT5 on main; phase C waits for MT6–MT8 and MT10 on main.

## Working rules

- **Dependencies count only when they are on local `main`.** To pick one up, merge local `main`
  into `sheet-metal` (`git merge main`), rerun the affected suites, and commit. Never merge another
  session's live branch (`machine-tending`, `collision`, ...).
- **Blocked steps.** Before each step, check its prerequisite exactly as written. If it is
  missing, do not work around it (no stand-in actuator mechanism or signal bus): mark the
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
| SM0 | done: clean branch on 38601ac61, pins verified, prerequisite documents read; quick gate `sm0-setup` passes (ProjectKit 138, ManufacturingKit 54) | c70c89745 |
| SM1 | done: exact line/arc regions, permanent graph ids, edits/cuts/rips, bidirectional FK, neutral-length conservation, closure and P1–P5 bounds; quick gate `sm1-core` passes (2102 sheet assertions) | 0713c4e0c; 54b5cab15 |
| SM2 | done: named wall extrusions and bend revolutions fused for flat/folded/partial states; P1–P5 validity, single bodies, volumes, axes and stable names; quick gate `sm2-solid` passes (3757 sheet assertions) | 7a3836d8c |
| SM3 | done (required feature chain); optional corner closure tracked separately as SM3o: SM3a atomic graph publication; SM3b exact sketch-backed base, side A/B, stock codec and named thickness builder; quick gates `sm3a-staging` (3773) and `sm3b-base` (3813 sheet assertions); SM3c authored segment names and exact face-to-graph picks; quick gate `sm3c-edges` (3843 sheet assertions); SM3d exact flange geometry and alignment; quick gate `sm3d-flange-geometry` (3880 sheet assertions); SM3e typed flange features, authored shared identities, gaps/extensions, stock radius defaults and P1–P5 document acceptance; quick gate `sm3e-flanges-typed` (4044 sheet assertions); SM3f exact planar subtraction foundation; quick gate `sm3f-outline-cuts` (4068 sheet assertions); SM3g typed rectangular/obround reliefs and sizing, exact gap/bend-width semantics, atomic edits and codec; quick gate `sm3g-reliefs` (4132 sheet assertions); SM3h closed/open/teardrop hems, shared edge selection, typed codec and builder; quick gate `sm3h-hems` (4303 sheet assertions); SM3i permanent bend-id rip feature, typed codec and builder; quick gate `sm3i-rips` (4315 sheet assertions); SM3j exact through-wall cuts on staged attached sketches, shared region mapping, typed codec and builder; quick gate `sm3j-wall-cuts` (4348 sheet assertions); SM3k nominal seam datums, exact bend material regions and seam validation; quick gate `sm3k-bend-regions` (4368 sheet assertions); SM3l native mapped positive-radius regions and adaptive volume integration; quick gate `sm3l-native-regions` (4368 sheet assertions); SM3m developed cuts across wall/bend/flange domains, named native patches, typed codec and edits, certified material bounds; quick gate `sm3m-cross-bend-cuts` (4633 sheet assertions); SM3n graph-derived collapsed-axis sectors, exact closed-hem cuts, material connectivity and production save/edit checks; quick gate `sm3n-closed-hem-cuts` (5200 sheet assertions) | 535ebc547; 08afd7a32; bb3405a64; 84a51f00d; 3ed9b3971; 31a696ed7; a02214101; f3ef57fa0; e5d66c947; 3d2d330dd; 6033a2f60; ee0d9e834; f83853060; db7e8be88; ab52986ef |
| SM3o | done: SM3o-a derives exact gap/overlap supporting edges from folded planes and both stock thicknesses; quick gate `sm3oa-corner-geometry` passes (7282 sheet assertions, including 34 corner checks); SM3o-b implements exact round/square relief tools and developed cuts; quick gate `sm3ob-corner-relief` passes all five suites (7321 sheet assertions, including 39 relief checks); SM3o-c implements atomic gap/overlap closure, graph-owned picks, named builder dimensions and saved corner features; geometry/feature checks pass 75/108 assertions, including partial folds and insufficient-relief refusal; document 16 fixtures regenerated and app regressions pass; full gate `sm3o-corners-full` passes every stage including project-source runtime (7504 sheet assertions); original MachineKit and router engineering baselines unchanged | 2ef1f5455; 09d95720d |
| SM4 | done: SM4a exact developed blank regions, typed domain owners, sorted bend data and interrupted fold markings, authored grain frame and typed version-14 snapshot; quick gate `sm4a-flat-verified` (5259 sheet assertions); SM4c typed shared drawing data, exact R12 DXF/SVG, bend annotations and grain arrow; quick gate `sm4c-drawing-exports` (5888 sheet assertions); SM4d typed cut/relief boundary provenance and propagation; quick gate `sm4d-boundary-provenance` (5911 sheet assertions); SM4e six typed manufacturability findings with exact geometry and minimum flange depth; quick gate `sm4e-checks-verified` (5944 sheet assertions); typed blank requirement with explicit order context; quick gate `sm4-flat-pattern-complete` (5963 sheet assertions) | a5508e566; 83a7a8c18; 723b0bac4; 27f66bb9e; a7156977e |
| SM4b | done: typed editor base/flange commands, explicit stock input, parameter inspector, folded/flat publication, bend table, exact fold-line overlay and DXF/SVG commands; P2 editor workflow passes; full gate `sm4b-editor-full` passes | be21d1579 |
| SM5 | in progress: SM5a unique native ancestry count/index/bulk queries and typed owning Haxe wrappers; three native suites and focused ancestry checks pass; quick gate `sm5a-ancestry` (5974 sheet assertions); recognition and convert feature deferred by user direction on 2026-10-05 | 51c4f2d5c |
| SM6 | done: SM6a typed air-bending predictions, physical springback inversion and exact contact depth; focused process smoke passes 1235 assertions; quick gate `sm6a-air-bending` passes (7209 sheet assertions); SM6b explicit lot/tool/angle-band calibration with measured radius and additive depth correction, focused smoke passes 1257 assertions, quick gate `sm6b-calibration` passes (7231 sheet assertions); SM6 complete: typed grade defaults, editor selection, Process-rule persistence and tooling minimums; full gate `sm6-process-material-full` passes all stages (7248 sheet assertions, ProjectKit 155); final material integration and CAD workflow checks pass | 7148a8bb8; 6458df23b; d664caa22 |
| SM7 | done: typed exact punch and single-/four-V die catalogue; standard segments and physical horns; independently segmented paired rows and validated bed layouts; derived process contact data; registered segment/clamp recipes and manual/powered clamp capabilities; strong unique tooling face names. Quick gates `sm7a-profile-foundation`, `sm7b-punch-profiles`, `sm7c-die-profiles` and full gate `sm7-tooling-verified-full` pass; original MachineKit and router engineering summaries unchanged | c4e418ba5; 310a41894; 67da9cae8; final tooling integration |
| SM8a | done: cylinder parts and coupled mechanical assembly; typed planning geometry derived from physical datums, independent reference comparisons, full-bed force/depth and conservative demo/frame checks; Start-page project with editable ram, X/R and depth-stop axes. Quick gates `sm8a-cylinder-parts`, `sm8a-mechanical-assembly` and full gate `sm8a-press-assembly-full` pass every stage; original MachineKit and router engineering summaries unchanged | c4517c795; ed8faa29b; final assembly integration |
| SM8b | done: MT5 is on local main 1484b924d and merged; shared typed fluid cylinder/valve/pressure-source interfaces replace pneumatic-only interfaces, and pressure resolution no longer casts components; explicit immutable phase-speed model added, its quick gate `sm8b-phase-speeds` passes all five suites; assembly format 7 hydraulic data passes focused tests (187 ProjectKit assertions), full gate `sm8b-hydraulic-format-full` passes every stage; quick gate `sm8b-fluid-interfaces` passes all five suites with unchanged original MachineKit summaries; typed shared cylinder runtime and current RobotModel 12 fixtures pass focused model (30), both-backend hydraulic (14), world (5004), maintained MuJoCo and four-platform ABI checks; full gate `sm8b-cylinder-runtime-verified-full` passes every stage including project-source runtime; actual press hydraulic service, distribution blocks, bore-area frame-pressure limit and both guide bindings pass quick gate `sm8b-fluid-bindings-verified`; authored passive-joint support reaches assembly/model/runtime, passes 36 model, 192 ProjectKit and both-backend ownership tests (MuJoCo gravity verified); full gate `sm8b-passive-joints-full` passes every stage with unchanged five engineering summaries and app engineering baselines; typed convex-hull contact settings and maintained primitive/hull finger-contact tests pass 6 MuJoCo assertions and the four-platform ABI audit; full gate `sm8b-hull-contact-full` passes every stage with unchanged original MachineKit and app engineering baselines; the later health audit strengthens contact proof to 12 fault-free outcomes; typed assembly 9, machine recipe 16 and scene 17 transport stops, contacts and native timed outputs; quick gate `sm8b-assembled-press-verified` passes all five suites with unchanged original MachineKit summaries; assembled press contacts, stroke, home and lifecycle pass in the rebuilt app; full gate `sm8b-assembled-press-full` passes every stage, including workspace, app build and complete project-source runtime (215 ProjectKit assertions and 12 fault-free finger-contact outcomes) | edf913c0f; 68768fcae; 22ce57f1c; 439ca8690; b15e594c6; 19fc1938c; bf3d5cd31; 63357537d |
| SM9 | done: typed loaded-stroke evaluation, explicit process-compatible P1–P5 stock, physical manual finger datums, symmetric press placement and exact straight/arc backgauge registration; complete setup checks process compatibility, tonnage, both flange minima, material-bearing tool support and sampled nominal ram travel. Maintained checks pass independent P2 first/later oracles, folded R lift, physical contacts, loaded depth/force, sampled swing and named refusals. Quick gates `sm9a-loaded-stroke`, `sm9b-manufacturing-setup`, `sm9c-press-placement`, `sm9d-backgauge-registration` and final `sm9-single-bend-complete` pass all five suites (7594 sheet assertions); original engineering results unchanged | ce5b1e66b; d1e0876b8; 5a6e8663a; 3d51a5b2c; 5b63f5c9c |
| SM10 | done: typed convex-prism data and narrow collision interface; interim exact SAT checks face normals and every edge-cross-product axis. Focused collision checks pass 24 assertions; quick gate `sm10a-prism-collision` passes all five suites with unchanged engineering results. SM10b wall decomposition preserves holes/concavity and explicit arc chord error; focused checks pass 19 assertions, quick gate `sm10b-sheet-prisms` passes all five suites. SM10c bend decomposition preserves developed holes through flat, partial, loaded and zero-radius hem states with explicit mapping error; focused checks pass 66 assertions; quick gate `sm10c-bend-prisms` passes all five suites with unchanged engineering results. SM10d press-frame collision assembly derives all installed tool rows, moving machine boxes and exact open frame volumes, retaining typed owners and approximation errors; focused checks pass 73 assertions; quick gate `sm10d-collision-geometry` passes all five suites with unchanged engineering results. SM10e analytic tool-contact refinement distinguishes material overlap from tip/shoulder tangency; focused checks pass 786 assertions; serial full gate `sm10e-exact-contact-full` passes every stage, including all five quick suites, workspace, app build and project-source runtime. SM10f exact uncut bend/prism refinement supports arbitrary axes, finite widths and hollow interiors; maintained contact checks pass 1082 assertions; quick gate `sm10f-bend-obstacles` passes all five suites with unchanged original engineering results. SM10g typed material query combines exact refinement, support-gap clearance and face-inset collision certificates; sampled stroke checks every installed tool and machine volume, including clamps and explicit backgauge withdrawal; maintained checks pass 50 prism, 1748 contact and 76 geometry assertions plus checked P2 withdrawal/collision/travel outcomes. Quick gate `sm10g-stroke-material` passes all five suites with unchanged engineering results. SM10h exact wall/prism refinement retains curved boundaries, holes and finite thickness at arbitrary orientation; typed GripCheck with an explicit gripper box checks full patch support and physical clearance through withdrawal/stroke, without exempting the gripped wall; 160 maintained grip assertions pass. Quick gate `sm10h-grip-geometry` passes all five suites with unchanged engineering results (52 prism, 1748 contact, 76 geometry assertions). SM10i: finite event candidates include seam and tool ends, horn transitions and finger datums; typed best-first search uses admissible bend/flip time bounds, explicit node/setup budgets and retained physical failure witnesses. Immutable `BendPreparation` derives each program step, checked withdrawal/regrip/flip cost and fresh final state. `GripCheck` returns ordered typed candidate outcomes, and the geometric checker shares per-setup stroke geometry across grips. Maintained checks plan P1-P5: P3 flips, P4 ends on gooseneck, P5 uses gapped V12 rows without horns in order 2,4,1,3 (22.0915 s); both orders of the negative fixture report physical collision witnesses. Final quick gate `sm10i-sequence-search-own-cadkit` passes all five suites: 7602 sheet assertions, ProjectKit 215, ManufacturingKit 54; press-brake checks include 101355 tool-support, 1751 contact, 76 collision-geometry and 164 grip assertions. All 11 original MachineKit engineering/diagnostic rows match baseline. No casts or saved-format changes. | 168867937; 6884dbacf; 4043105d5; e1d55a1a4; a1bd4f33b; 9a5629588; 8515ae696 |
| SM11 | blocked: MT6 (two controllers and `CellSignals`) and MT7 (`RobotInterface` state-machine pattern) are not on local main 3bc5f73aa | |
| SM12 | planned | |
| SM13 | planned | |
| SM14 | planned | |
| SM15 | planned | |
| SM16 | planned | |
| SM17 | later | |
| SM18 | later | |

### Implementation notes

- SM0: `sm0-setup` passed; MachineKit's robot-arm and CNC-router printed summaries exactly
  match `sm-s-machinekit-baseline.log`. No baseline numbers changed.
- SM1a: SheetMetalSmoke covers table validation/ownership, ANSI/DIN conversion, published
  flange and mould-line dimensions, BA/OSSB/BD at 30°/90°/135° for either bend sign,
  length conventions, stock units and named invalid-input diagnoses (41 assertions).
  Haxeon rejects direct field access on `Null<SheetMechanical>` (`E1005`):
  `haxeon/src/compiler/types/typing/BodyTyper.hx`, `typedMember`, explicitly rejects
  `TNullable`. The test uses a typed nonnullable local for its known-present data; this is
  compiler policy, not a compiler patch or an unresolved blocker.
  Quick gate `sm1a-dimensions` passes all five suites; existing MachineKit summaries and
  ProjectKit/ManufacturingKit assertion counts are unchanged.
- SM1: `sm1-core` passes all five quick suites, including 2102 sheet assertions. Outline
  validation uses analytic intersections and winding (including circular holes); evaluations
  retain immutable wall snapshots, so later graph edits cannot corrupt derived bounds.
  P1–P5 cover both fold signs, 0/near-zero/partial/target angles, inverse traversal from another
  root, tangent contact on both sheet sides, restored edits and a changed closed-ring flange.
  No new haxeon issue or baseline-number change; existing MachineKit summaries match exactly.
- SM2: `sm2-solid` passes all five quick suites, including 3757 sheet assertions. P1–P5
  are valid single solids in flat/folded states, volumes match the separate graph formulas,
  partial-state cylinders match evaluated axes/radii, and every face name is strong and unique.
  Flat/folded name sets agree; angle, radius and flange-length edits preserve names. Circular
  outlines/holes, partial arcs and K = 0.5 equal volumes are also checked. The native source-name
  issue and logical seed encoding are recorded in SM-D4; no native or compiler patch was needed.
  Existing MachineKit summaries and ProjectKit/ManufacturingKit counts remain unchanged.
- SM3a: `sm3a-staging` passes all five quick suites (3773 sheet assertions). A test-only
  two-feature chain verifies consumption of staged graphs, isolated caller edits, preserved
  allocation counters, undo/redo, atomic graph/solid retention after downstream failure, and
  successful recovery. No saved feature type or compatibility path is introduced by this
  foundation. Existing baseline counts and MachineKit summaries remain unchanged; no new
  haxeon issue was found. Concrete features and codec acceptance remain required for SM3.

- SM3e: `sm3e-flanges-typed` checks production P1–P5 document chains, typed codec round trips,
  malformed numeric wire payload rejection and version-11 rejection, shared parameter binding,
  gap/extension native validity, thickness-derived default radius, undo/redo, conservative edge
  resolution and stable output identities after upstream topology growth (4044 sheet assertions).
  Root ledger counters and feature-owned ids are authored state, not reconstructed from wall counts.
  Runtime fixtures generate the current format; no static version-11 sheet fixtures exist.
  Sheet code and its tests have no unchecked casts. MachineKit's schema-version assertion
  now expects version 12; its original geometry summaries are unchanged. No new haxeon issue was found.

- SM3f: `sm3f-outline-cuts` includes 24 planar subtraction assertions, raising sheet checks
  to 4068. Rectangular holes, open/circular notches, circle-circle lens area, overlapping voids,
  coincident boundaries and stable fragment identities are checked. Curved and holed results
  build valid named native walls whose volumes match exact areas. Empty/disconnected cuts have
  named diagnoses. This foundation introduces no saved feature record or schema change; relief,
  hem, cut and rip features remain in SM3. No new compiler issue was found.

- SM3g: `sm3g-reliefs` checks both relief types with absolute dimensions and thickness ratios,
  exact removed areas and explicit bend widths, valid flat/folded solids, builder arguments,
  typed codec round trips, persistent face names after thickness edits, failed-edit atomicity and
  undo recovery (63 relief assertions; 4132 sheet assertions total). Boundary assembly now uses
  the wall's area tolerance rather than the length tolerance, with a small-region regression check.
  The flange record changes shape, so document version 13 replaces version 12 and MachineKit's
  version check follows it. Runtime fixtures use current code; no compatibility reader is added.
  Existing ProjectKit/ManufacturingKit counts and MachineKit summaries remain unchanged.
  No new haxeon issue was found. Hem, cut and rip features remain.

- SM3h: `sm3h-hems` checks closed, open and teardrop hems on either sheet side, including
  exact contact, explicit tip gaps, valid single flat/partial/folded solids, volumes from an
  independent neutral-length integral, partial-state conservation and named invalid geometry
  diagnoses. Production document tests cover codec identities and names, shared builder length,
  edit/undo/redo and thickness-derived default bulb radii (171 hem assertions; 4303 total).
  The shared edge-feature layer also reruns every flange and relief check. No native or compiler
  change is needed, and no unchecked casts are introduced. Existing suite counts and original
  MachineKit summaries remain unchanged. Cut and rip features remain in SM3.

- SM3i: `sm3i-rips` checks a production rip on a closed four-wall ring, source isolation,
  flattening into one plane, retained bend allowance and native validity. A second seam that
  disconnects walls is diagnosed and leaves the committed graph unchanged. Codec/builder
  checks preserve source and bend ids in an inactive tree-seam record; active ring-source
  codec coverage will use the convert feature in SM5 (the ring builder is test-only, not a
  saved feature). There are 12 new assertions, 4315 total. Existing suite counts and original
  MachineKit summaries remain unchanged. No compiler issue or unchecked cast was introduced.
  Wall cuts and exact cuts across bend zones remain in SM3.

- SM3j: `sm3j-wall-cuts` includes 33 production cut assertions, 4348 total. Circular cuts
  on persistent attached sketches work on side A/B of a rotated sheet; radius and thickness
  edits use staged exact geometry. Codec preserves authored wall/side and native names; undo/redo,
  consumed-wall rollback, builder integration, off-wall plane refusal and explicit bend-zone
  crossing refusal are checked. Base-region mapping now has the same implementation as cuts,
  and every earlier base/flange/hem/rip test reruns. Generic edge-selection diagnostics now
  describe both flanges and hems. Existing suite counts and original MachineKit summaries are
  unchanged; no new compiler issue or unchecked cast was introduced. Exact cuts across bend
  zones remain required before SM3 completes; this guard is temporary, not a compatibility path.

- SM3k: `sm3k-bend-regions` checks circular developed bend holes, split strip patches,
  interrupted wall seams, fixed fold transforms, copy isolation and precise disconnected/mismatch
  diagnoses (20 new assertions, 4368 total). Nominal attachments are typed immutable records,
  independent of contour indices. The native builder explicitly refuses trimmed bend regions
  until its mapped-region operation lands; SM3 remains in progress. A local CadKit build was
  configured early because this native work is needed before SM5. Its configure log explicitly
  says “CadKit uses prebuilt OCCT”; both OCCT build options are OFF, and all three native
  baseline tests pass. Existing suite numbers and original MachineKit summaries are unchanged.
  No new compiler issue or unchecked cast was introduced.

  Haxeon note (SM3k): a secondary typedef used inside its own module needs an explicit
  `import cadkit.sheetmetal.BendSeam.SeamInterval`. Without it, dependency scanning records
  `cadkit.sheetmetal.SeamInterval` and E2001 rejects a nonexistent module. The cause is
  `haxeon/src/compiler/semantic/ModuleAnalyzer.hx` dependency qualification (the class-name
  removal handles classes only, then unresolved names fall back to the package at lines 183–189)
  despite `indexDeclarations` registering the alias owner; `sourceModuleForDependency` at
  lines 473–484 probes source paths without consulting that owner map.
  `modules/ModuleReachability.hx:63` reports it. The local explicit import preserves typed
  data; haxeon is unchanged. The first quick gate exposed this after the import was removed;
  it was restored before the passing gate.

- SM3l: the native mapped-region operation preserves exact rational pcurves on analytic
  cylindrical skins, builds radial ruled cut sides, and sews a checked solid at OCCT tolerance.
  No sampled polygon profile is introduced. Standard 3D edge representations use OCCT's 1e-8
  approximation tolerance, with 1e-7 sewing tolerance; the flat curves and cylinder pcurves
  remain exact. Native tests verify circular-hole volume for up/down and partial bends and
  prove every generated history face belongs to the result. Positive radii are supported;
  collapsed zero-radius hem skins require separate handling before cross-bend integration
  is complete. All three native suites pass. OCCT's default nonadaptive volume integration
  differed by 0.009059 mm³ on a curved trimmed test face; adaptive integration matches its
  independent Jacobian integral within 1.3e-8 mm³. `Shape.volume(relativeTolerance)` exposes
  typed adaptive integration explicitly, retaining existing measurement behavior when omitted.
  The `sm3l-native-regions` quick gate retains 4368 sheet assertions, ProjectKit 143,
  ManufacturingKit 54 and unchanged original MachineKit summaries. No unchecked cast or new
  compiler issue is introduced. SheetSolid and document cut integration remain required.

- SM3m: `SheetCutGeometry` replaces the temporary wall-only guard; an authored developed
  profile trims every affected wall and bend domain in the flat atlas, then the sheet folds.
  Material contours are distinct from nominal datums, and each rebuild derives cut tools once.
  Named native patches preserve input-edge history; multiple patch skins use authored boundary
  identities instead of contour indices. Trimmed bounds use analytic second-derivative bounds
  and adaptive interval subdivision to certify coordinate enclosures within 1e-8 mm; this is
  numerical extrema evaluation of exact curves, not polygonal profile approximation. Uncut
  bounds retain their analytic path. Tests compare circular band areas with an independent
  integral, volumes for flat/partial/folded up/down states, native bounds, surviving end trims,
  native face identities, production codec, staged radius/thickness edits, undo/redo and atomic
  failed cuts (4633 sheet assertions total). The quick gate is `sm3m-cross-bend-cuts`; existing
  ProjectKit/ManufacturingKit counts and original MachineKit summaries remain unchanged.
  SM3 remains in progress for collapsed-axis topology at a zero-radius closed hem; such a
  trimmed folded skin reports `sheet.collapsed-bend-cut`, with its bend identity, while positive
  radii and partial states are supported. No unchecked cast or new compiler issue is introduced.

- SM3n: exact zero-radius radial sectors permit closed-hem cuts after the wall/hem union,
  including circular holes entirely inside a bend and profiles crossing both seams, on either
  sheet side. Open and teardrop cuts also pass flat/partial/target volume and identity checks.
  Production closed-hem codec and thickness/return-length/hole-radius edits preserve tested
  names and derive tools once. Material islands and wholly severed active bends are refused.
  Annular radial-tool decomposition preserves exact area and emits hole-free sectors; every
  partition is named by authored endpoints/extrema, never mutable contour indices or coordinates.
  Line/circle intersections now use closest-point projection with a relative roundoff bound:
  the previous quadratic discriminant produced spurious crossing pairs at translated small-circle
  tangencies. A translated 0.3 mm circle has one tangent contact in the regression test.
  No chord profile approximation, artificial hem gap, schema change or unchecked cast is added.
  All three native suites pass, including zero-radius up/down sector Jacobian volumes.
  The `sm3n-closed-hem-cuts` quick gate passes 5200 sheet assertions (567 more than SM3m),
  ProjectKit 143, ManufacturingKit 54 and unchanged original MachineKit summaries.
  No new haxeon issue was found. SM3's required features are complete; optional corner closure
  is explicitly SM3o after SM6, before phase B, as permitted by the SM3 step text.

- SM4a: exact mapped material domains produce connected P1–P5 blanks and preserve circular
  holes across wall/bend seams. P5 matches an independently constructed twelve-corner polygon
  by empty differences in both directions, exact area and extrema; authored contour subdivisions
  are preserved rather than coalesced away. Bend lines stop at removed material and the table
  retains nominal width separately from effective fold length. Nested disconnected rings retain
  the nearest containing hole owner. Grain normalisation, diagonal/right-angle stock bounds,
  isolated returned arrays, typed codec and named-thickness material copying are covered.
  The `sm4a-flat-verified` quick gate passes 5259 sheet assertions (59 new), ProjectKit 143,
  ManufacturingKit 54 and unchanged original MachineKit summaries. Schema version 14 replaces
  version 13; runtime fixtures use current code, with no compatibility reader or unchecked cast.
  No new haxeon issue was found. Exports, design checks and the blank requirement remain SM4.

- SM4c: the shared typed drawing model feeds pure DXF and SVG writers with all five specified
  layers. An independent minimal R12 reader reconstructs P1–P5 curves and bend markings,
  small/large arcs crossing zero in either direction, complete circles and upright text.
  A grained circular-hole blank reconstructs valid outer/hole loops with unchanged exact area;
  its etch arrow follows exported X. SVG layers, millimetre dimensions, circular holes and XML
  escaping are checked; DXF rejects embedded newlines instead of corrupting group records.
  The focused drawing suite passes 629 assertions. Quick gate `sm4c-drawing-exports` passes
  5888 sheet assertions, ProjectKit 143 and ManufacturingKit 54; original MachineKit summaries
  remain unchanged. No schema bump, unchecked cast or new haxeon issue is introduced.
  Design checks and the rectangular blank requirement remain before SM4 is complete.

- SM4d: typed boundary origins distinguish cuts from reliefs without interpreting edge-name
  strings. Focused tests cover notch/relief purpose, defensive copies, supporting-edge edits,
  gap fragments, reflected frames and unions. Production cut tests verify purpose after typed
  codec rebuilds and radius/thickness edits on both sheet sides. No origin is persisted separately:
  the current feature graph derives it again on every rebuild. The quick gate is
  `sm4d-boundary-provenance`, with 5911 sheet assertions (23 new), ProjectKit 143,
  ManufacturingKit 54 and unchanged original MachineKit summaries. No schema or compiler change
  is introduced. Manufacturability findings and the blank requirement remain in SM4.

- SM4e: every specified finding fires on a crafted bad sheet and stays silent on P1–P5.
  Tests cover tapered/circular flange minima, ripped free strips, exact hole and outer-notch
  clearance, explicit relief exemptions, internal crease endings, overlap, material minimum
  equality and inclusive 15-degree grain limits. Typed measurements carry units and findings
  keep exact geometry. A missing material interval has a named diagnosis, caught with a typed
  catch. N-ary union resolves valid final material whose wall-only intermediate contours touch
  at a point; curves are never polygonised. Quick gate `sm4e-checks-verified` passes all five
  suites: 5944 sheet assertions (33 new), ProjectKit 143, ManufacturingKit 54 and unchanged
  original MachineKit summaries. No unchecked cast, saved schema change or new haxeon issue.
  The blank requirement remains before SM4 is complete.

- SM4 final: exact flat bounds and stock identity derive the rectangular manufacturing blank.
  Caller context provides identities, revisions, quantity and asymmetric mm tolerances; invalid
  orders and tolerances have typed diagnoses. Tests cover P5 dimensions, arbitrary grain angle,
  allowed rotations and the current ManufacturingKit codec round trip. Quick gate
  `sm4-flat-pattern-complete` passes all five suites: 5963 sheet assertions (19 new),
  ProjectKit 143, ManufacturingKit 54 and unchanged original MachineKit summaries.
  No saved schema change, unchecked cast or new haxeon issue is introduced.

- SM4b: the editor creates sheet bases from solved sketches and flanges from persistent picked
  wall-side faces, edits dimensions with document history, displays the derived flat solid and
  exact bend intervals, lists sorted bend data and exports DXF/SVG. Stock identity and ANSI K
  remain explicit until SM6 supplies the library; numeric form input rejects trailing text.
  A publication-scoped flat snapshot prevents exact unions on every repaint. View state does
  not change the saved graph. Touched publication/refinement paths use typed shape locals.
  P2 editor acceptance verifies 60/120/40 mm outside dimensions, 1.5 mm flat thickness, parameter
  edits, explicit-rule form defaults, unchanged authored graph across view toggles and DXF output.
  Existing CAD workflow fixtures now write inside this worktree. The final focused CAD workflow
  runtime passes, and full gate `sm4b-editor-full` passes all five quick suites, the maintained
  workspace selection (peer tests excluded), app build and complete project-source suite.
  Sheet assertions remain 5963, ProjectKit 143 and ManufacturingKit 54; original MachineKit
  summaries match baseline exactly. The suite uses a local copy of the supplied gate script
  with HashLink's library directory added to its runtime path; the supplied script is unchanged.
  Population ran once; cloned nested Haxeon/NativeKit repositories needed local submodule
  registration to stop repeated cloning. Approved coal/proxsuite symlinks are unstaged.
  Every CadKit configure uses prebuilt OCCT; no vendor pin, saved schema or haxeon source changes.

- SM5a: OCCT's shape-to-ancestor map supplies unique containing shapes, preserving source
  names on returned owning handles. Identical geometry from another shape is not membership;
  invalid kind, index and buffer size are diagnosed. The typed `Shape` wrapper exposes count,
  indexed access and a cleanup-safe array query. Tests verify box edge adjacency, foreign-edge
  refusal, indexed/bulk agreement, retained opaque sheet face names after source closure and
  deduplication of the cylinder's twice-used periodic seam. All three native suites pass;
  the regenerated Haxe binding and focused ancestry smoke pass 11 new assertions.
  Quick gate `sm5a-ancestry` passes all five suites: 5974 sheet assertions, ProjectKit 143,
  ManufacturingKit 54 and unchanged original MachineKit summaries. No saved format, vendor pin
  or haxeon source change is introduced. Solid recognition and the convert feature remain SM5.

- SM6a: typed process/tooling inputs predict final/loaded radius, conserved neutral length,
  signed overbend, punch-bottom depth, kN force and assumed 90-degree flange minimum. The
  physical springback branch is inverted with a bracketed solve. Radius bends respect the punch
  under load and derive the released radius; tool angle/contact limits are diagnosed. Independent
  circle-centre tangent construction verifies depth over 20–150 degrees for nine stock/radius
  combinations. Focused process smoke passes 1235 assertions; quick gate `sm6a-air-bending`
  passes all five suites (7209 sheet assertions, ProjectKit 143, ManufacturingKit 54), with
  unchanged original MachineKit summaries. No schema, cast, native or compiler change is made.
  Calibration, saved Process rules and material-library integration remain required for SM6.

- SM6b: immutable calibration entries match an explicit lot, exact thickness/tool identities
  and half-open angle-magnitude bands. Overlap is refused; caller and returned arrays are
  isolated. Measured final radius derives fresh loaded radius, allowance, overbend and analytic
  depth, then the measured depth offset is added with explicit result provenance. Missing keys
  retain analytic predictions; incompatible measured/punch radii are diagnosed. Springback uses
  the factored cubic to avoid cancellation and the inverse brackets only the loaded physical
  branch, including large released radii. Focused smoke passes 1257 assertions; quick gate
  `sm6b-calibration` passes all five suites (7231 sheet assertions, ProjectKit 143,
  ManufacturingKit 54), with unchanged original MachineKit summaries. No cast, schema, native
  or compiler change is made. Saved Process rules, tooling-based sheet checks and material-library
  integration remain required before SM6's full gate and final commit.

- SM6 integration (complete): ProjectKit owns the canonical wire mechanical
  properties and ANSI K-factor rows; CadKit uses those types directly. Haxeon's wire generator
  deliberately rejects duplicate structural wire types (`WireCodecGenerator.hx:268`, E1024),
  so the duplicate CadKit declarations are removed rather than aliased. Mechanical records remain
  immutable. Specific mild-steel and 5052-H32 grades carry the plan's assumed strengths and
  assumed development defaults; generic visual materials retain no inferred grade strength.
  Demo parts obtain strengths from the library while keeping their explicit K=.38 reference rule,
  authored radius and unconstrained grain, preserving their acceptance geometry.
  Stock creation copies the library data and requires explicit root-frame rolling direction.
  The editor selects built-in or custom sheet grades and permits an explicit K override.
  Scene format 2 saves the material section through the typed wire codec; format 1 is rejected.
  The current fixture and worker examples are regenerated with current serializers, including
  their empty BIM sections formerly saved at CadKit version 11. The negative retired-human
  fixture uses the current outer scene format so it still exercises its intended diagnosis.
- Process rules save authored tool dimensions, optional explicit lot and immutable calibration
  entries, never derived predictions. Document version 15 replaces version 14. Allowance and
  deduction calculations consume stock directly, giving material properties and thickness one
  owner. Default flange radii derive from tooling on each rebuild; explicit incompatible radii
  receive `sheet.process-radius-mismatch`. The current air-bending rule refuses hems and other
  angles beyond its physical/tool limits instead of substituting a K rule.
  Sheet checks replace the temporary 4T minimum with the assumed 0.7V process minimum: default
  V=8T for T<=3 mm, selected tooling for Process stock, or an explicit tooling argument. Thicker
  stock without selected tooling is diagnosed. The 1.5 mm default minimum changes from 6 to
  8.4 mm by this planned replacement; demo geometry and original machine summaries must not move.
- Haxeon note (SM6): statement-switch definite assignment treats nullable enums as nonexhaustive
  even with a catch-all (`BodyTyper.exhaustiveEnum`, lines 2217–2219); `StatementTyper.typeSwitch`
  includes the unchanged incoming scope at lines 837–838, so a later read gets E1023. SheetChecks
  uses a directly initialized switch expression instead. No compiler source change is made.

  Focused SheetMetalSmoke passes 7248 assertions and the Process-rule production smoke passes
  14 assertions. The app regression entry passes material integration, scene documents and worker
  examples, then exposed a CAD test artifact path that assumed root cwd; a single worktree-aware
  artifact helper fixes it. The affected CAD workflow passes from app/tests after that fix.
  Scene format 2/current CadKit format 15 fixtures are generated by current serializers, and
  `make-worker-gallery.py --check` passes. Full gate `sm6-process-material-full` passes every
  kit stage, the app build and project-source runtime suite (all exit 0). The final material
  integration binary also passes, including explicit K stock retaining known strengths without
  sheet defaults. ProjectKit rises from 143 to 155 assertions for the canonical material data and
  wire validation. SheetMetalSmoke rises from 7231 to 7248 for Process-rule document acceptance
  and tooling-derived minimums. The original five MachineKit summaries and the SM4b router
  engineering summaries are unchanged; tick timings and collection counts are observations.

- SM3o-a: `CornerGeometry` derives two supporting-edge offsets from the current folded planes,
  selected authored outer segments and stock thickness. The closed `CornerJoin` enum names gap
  or which flange overlaps. Segment directions must match the planes' intersection; parallel
  planes, non-corner boundaries and invalid gap/overlap dimensions have named diagnoses. Both
  depth endpoints are included in the stand-off and overlap calculation at oblique corners.
  This is the geometry foundation only: document features, corner reliefs and their native/flat
  acceptance checks remain required before SM3o is complete.
  Quick gate `sm3oa-corner-geometry` passes all five suites with 7282 sheet assertions,
  including 34 slab-clearance, overlap, selection-order and diagnosis checks. Original five
  MachineKit summaries remain identical to baseline. No new haxeon issue or cast was introduced.

- SM3o-b: `CornerReliefGeometry` intersects the two nominal parent bend supporting lines and
  authors one exact developed cut there. Round size is diameter; square size is side length,
  with its first axis along the first bend seam. Both shapes retain their exact line/arc geometry.
  The shared developed-cut path carries an explicit `BoundaryOrigin.Relief` through parent and
  flange contours and `BendCut` records, so bend material rebuilt later retains its purpose.
  This avoids reporting a corner relief as a nearby ordinary hole. Folded and flat native solids
  pass the focused smoke (35 assertions); the gate adds exact connected-blank area checks.
  The gap/overlap closure operation, saved feature and builder, edits and codec acceptance are
  still required before SM3o is complete.
  Quick gate `sm3ob-corner-relief` passes all five suites with 7321 sheet assertions. The
  39 relief checks cover exact tool areas, root quarter cuts, permanent graph datums, purpose
  through both bend contours, valid native folded/flat solids and one connected developed blank
  whose union area equals its domains. Original five MachineKit summaries are unchanged.
  No saved format changes or haxeon issue occur in this geometry-only sub-step.

- SM3o-c: `SheetCorner` stages both supporting-edge contours and the exact corner relief in
  one material transaction. `SheetCutGeometry.applyEdits` uses the validated source's developed
  nominal frames, applies all contour changes and cuts, and verifies the complete result once;
  no invalid intermediate material graph is published. Real slab-position tests cover gap and
  both overlap directions with both relief shapes, including a larger gap that trims both ends.
  Picks own permanent wall and authored boundary identities; `fromFace` resolves a live side
  face through the existing graph matcher without parsing native names. Missing picks diagnose
  the failed selection. `sheet-corner` saves only these picks and its arrangement, gap, overlap
  and relief dimensions. The builder binds shared length dimensions. Its source allocator is
  retained; corners allocate no replacement wall or bend ids.
  The 108 production checks cover all six saved arrangements, byte-identical current codec
  round trips, exact blank area, native face names under gap and upstream leg edits, undo/redo,
  and committed graph/solid retention after an invalid gap. Pure corner checks pass 75 assertions, including partial folds and insufficient-relief refusal.
  Document format 16 replaces 15 for the new saved feature variant, following the one-schema
  policy even though existing record shapes are unchanged; no earlier format reader is added.
  Current empty BIM fixtures and worker examples are regenerated through current serializers;
  the gallery generator check passes. The complete app regression entry passes (`sm3oc-app-tests.log`,
  ending with `CAD part workflow tests passed`). Full gate `sm3o-corners-full` passes all five kit
  suites, the workspace suite, the app build and the project-source runtime, including 7504 sheet
  assertions. Original MachineKit summaries and router engineering baselines are unchanged. No new
  compiler issue or unchecked cast was introduced.

- SM7 reference punch body assumptions: straight and acute profiles are 100 mm high, with
  contact flanks to z=15 mm, a transition to a 16 mm wide upper section at z=30 mm, and the
  mounting datum at z=100 mm. The gooseneck is 120 mm high; its positive-Y relief has a
  boundary at y=-5 mm from z=35 to 65 mm, with transitions at z=25 and 80 mm. These are
  demonstration sections, not manufacturer drawings or strength ratings. R0.6 and 88/30-degree
  contact geometry come from the reference table. The exact nose is circular; collision sections
  use circumscribed tangent facets with an explicit maximum outward error.

- SM7b verification: native solids are valid and exact circular-segment volume matches an
  independent formula. Collision covers contain sampled exact noses at 30, 88 and 90 degrees
  with 0.1, 0.01 and 0.001 mm maximum outward errors; convex pieces preserve section area.
  The gooseneck relief stays empty. Quick gate `sm7b-punch-profiles` passes all five suites,
  retaining 7504 sheet assertions and all five original MachineKit engineering summaries.

- SM7 die body assumptions: single V8/V12/V16/V22 profiles are 40 mm wide and 30 mm high,
  with an 88-degree groove and R1 shoulders. The four-V block is a 60 mm square with the same
  grooves on consecutive faces. Opening measures between shoulder centres, as in SM6; exact
  shoulder tangents and groove depth are derived. All four grooves stay in both physical and
  collision geometry when a face is selected. No manufacturer dimensions or strength rating
  are inferred from these demonstration profiles.

- SM7c verification: sharp and rounded single-V native solids are valid; independent sharp
  volume and contact tangent/depth checks pass. All four-V face selections retain the same
  physical volume, including every groove, and leave the active opening clear. Conservative
  shoulder covers contain sampled exact contacts; convex pieces conserve section area.
  Quick gate `sm7c-die-profiles` passes all five suites with unchanged original MachineKit
  summaries and 7504 sheet assertions. No unchecked cast or new compiler workaround is needed.

- SM7 station representation: each station pairs independently segmented homogeneous punch
  and die rows at one bed offset. Row material length is the sum of segment lengths; row span
  includes every gap. Station length reports punch material length, while the die row exposes
  its own; the station reserves the larger row span, preventing another station from intruding
  into its tooling setup. Touching station ends are allowed. Process contact dimensions are
  derived from the installed profiles on demand, including the selected four-V face.
  Punch horns remove upper end material while retaining the contact nose; the mounting connector
  lies in the unrelieved length. Convex collision prisms are clipped by the same authored end
  interval and remaining height. Horn shape dimensions are explicit, with no inferred rating.

- SM7 clamp assumptions: a 50 x 40 x 60 mm steel envelope with no inferred holding rating.
  Manual clamps declare a manual tooling-lock capability. Powered reference clamps use an
  assumed pneumatic service (6 mm tube) with separate lock/unlock signal inputs and locked-state
  feedback output. This is the declared hardware interface; runtime actuation belongs to the
  later simulation steps. Segment and clamp recipes reconstruct catalogue selections, material,
  horn end/dimensions and powered/manual interfaces. A custom profile reusing a catalogue id
  does not acquire that recipe unless its complete authored specification matches.

- SM7 naming verification: the first full gate (`sm7-tooling-full`) exposed unlabelled section
  edges on the new default segment during MachineKitNamingAudit and was stopped. Tool contact,
  authored section edges and section faces now own names before extrusion; four-V groove cuts
  carry per-face provenance. The existing naming audit passes, and focused checks confirm strong,
  unique faces for every catalogue profile and both horn ends, including recipe reconstruction.
  Row homogeneity compares complete authored specifications, not ids alone.

- SM7 final verification: `sm7-tooling-verified-full` passes CadKit, SheetMetalSmoke
  (7504 assertions), ProjectKit (155), ManufacturingKit (54), MachineKit, all workspace
  suites (MotionKit 67533), the app build and the complete project-source runtime suite.
  The original five MachineKit summaries and router engineering baselines remain unchanged;
  only performance timings vary. No new cast or Haxeon compiler workaround was introduced.

- SM8a cylinder assumptions: two 160 mm bore / 100 mm rod cylinders, each with 100 mm
  stroke, 20 mm barrel wall, 40 mm end caps and 50 mm initial stem projection. Bodies and
  moving rods are separate parts on the existing guided ram; there is no extra cylinder joint.
  Pressure-area force is evaluated from authored dimensions, not inferred from the 40 t frame
  rating. These are explicit reference envelopes, pending a manufacturer drawing. The focused
  cylinder checks pass native validity, independent material volumes, full-bore/annular forces
  and invalid-dimension/pressure diagnoses. Runtime actuation and hydraulic service wiring
  remain SM8b on MT5's existing process-driven-joint mechanism.

- SM8a assembly assumptions: the bed top is 800 mm above the floor. Two 50 mm side
  plates provide the 230 mm throat and 320 mm open mount separation. The continuous
  ram beam attaches to its left guide; Y2 = Y1 is the torsion-bar coupling that
  maintains right-guide alignment. No redundant rigid loop repeats that coordinate relation. The guides and torsion bar sit in front of the side plates.
  The initial tooling station has a 150 mm straight punch row and 200 mm V12 die row.
  Upper clamp height derives from mount separation minus punch and die heights minus
  an explicitly assumed 80 mm open tool gap; this leaves 20 mm penetration within the
  100 mm stroke, instead of treating the 320 mm mount opening as tool clearance.
  X uses a 650 mm SFU1605, depth stop and optional R use 300 mm SFU1605 screws, all
  with preloaded SFU1605 nuts, physical BK12/BF12 supports and critical-speed limits
  derived over their bearing-centre spans. Servo classes are 200 W X, 50 W stop and
  100 W R, with existing generic bus amplifiers wired to a 48 V supply. These ratings
  and guide/frame envelopes are assumptions, not manufacturer specifications.
  Focused checks pass native validity and positive volume for every part, BOM coverage
  and coupled ram, finger and depth-stop poses with and without R at 0/37/100 mm travel.
  Integrating the moving rod exposed that its retained length must include the rear
  cap: rod length is stroke + cap length + initial stem, keeping its inner end in the
  bore at both stroke limits. Independent endpoint checks now verify this.

- SM8a planning geometry: `PressGeometrySource` declares member/joint identities through
  function-only typed behaviour. `PressGeometry.of` rebuilds bed and station contact datums,
  structural opening/throat, ram/finger bounds and finite travel intervals from the assembly.
  Optional R is a fixed zero interval when absent. `reference(layout)` is an independent
  numeric fixture; default and tallest gooseneck/four-V installations agree with it. The
  example accepts typed layouts, and the lowest installed die sets the gauge base height.
  Forces are converted explicitly from SM6's kN to the frame's declared N capacity.
  Demo/frame clearance uses an outer-arc plus straight-leg bound along each hinge chain,
  valid at every partial fold and ram position; checks require rectangular descendants
  and parallel nested hinges rather than silently applying the bound to other graph shapes.
  This is a structural frame check; die/punch/sheet contact and setup/order feasibility
  remain SM9/SM10. Nominal stroke is not an installed-tool safety limit.

- SM8a editor verification: the focused app test compiles the project, loads 56 physical
  members with material-derived masses, and moves Y1, X, R and the independent depth stop
  through editor operations. Its first run exposed a redundant rigid ram-guide closure:
  `AssemblyState.dependentJoints` treated the unmarked Y1 on that loop as solved. The
  torsion-bar coupling already owns Y2 = Y1, so the duplicate closure was removed instead
  of adding an editor exception or another saved input choice. Coupled guide/ram alignment
  remains verified by FK at 0/37/100 mm travel; the editor test now passes. No new cast,
  schema compatibility path or Haxeon compiler workaround was needed.

- SM8a final gate `sm8a-press-assembly-full`: CadKit, SheetMetalSmoke (7504),
  ProjectKit (155), ManufacturingKit (54), MachineKit, workspace, app build and
  project-source runtime all return zero. Original five MachineKit engineering
  summaries and router cutting/drive results are unchanged; no new compiler issue.

- SM8b shared fluid interfaces: `machinekit.fluid.CylinderSpec`, `ValveSpec` and
  `PressureSource` replace the pneumatic-only interface names, with no aliases.
  `MachineComponent` exposes typed optional capabilities, including pressure sources;
  air-pressure resolution uses that capability rather than a runtime type test and cast.
  The existing MT5 binding, saved data and runtime behavior are unchanged at this checkpoint.

- SM8b interface checkpoint gate `sm8b-fluid-interfaces` passes all five suites
  (7504 sheet, 155 project, 54 manufacturing assertions); original five MachineKit
  summaries match exactly. Hydraulic parts, saved drive data, phase speeds and switches
  remain for the subsequent SM8b checkpoints. No new cast or compiler workaround.

- SM8b phase-speed model: `CylinderPhaseSpeeds` owns positive finite approach,
  working and return rates, a nominal stroke and an explicit extension change point
  within that stroke. Extension selects working speed at the boundary; finite
  end-stop overtravel keeps the working phase. Ideal timing splits intervals at
  the change point and rejects reversed or out-of-travel timing intervals.
  Reference tests use an 80 mm tip gap, 1.5 mm sheet and 10 mm clearance, giving
  68.5 mm change travel, 4.79375 s extension and 1.42857 s return. The class does
  not infer a sheet thickness or supply pressure; setup supplies the change point.
  Quick gate `sm8b-phase-speeds` passes all five suites with unchanged original MachineKit summaries; saved/runtime integration remains in progress.

- SM8b saved drive checkpoint: assembly definitions and states now use only version 8. The rebased main already used v7 for sensor and process-velocity fields, so adding the hydraulic drive is a new format revision.
  Feature-dependent v3/v4/v5/v6 constants, codec acceptance and model upgrades are
  removed. `AssemblyHydraulicDrive` adds explicit bore/rod/stroke, pressure, three
  phase speeds, extension change travel and distinct A/B valve channels with a
  signed guide direction. Its declared valve is closed-centre when both coils
  agree; runtime actuation remains the next checkpoint. The bridge diagnoses the
  missing hydraulic simulation drive instead of treating it as an electric motor.
  Validation requires a prismatic guide of matching stroke and sufficient rate
  ceiling; nested assembly copying scopes both channels and owns the copied DTO.
  The new hydraulic fixture is written by the current codec using
  `PROJECTKIT_WRITE_FIXTURES=1`; existing assembly examples generate current v8
  records from code. Focused ProjectKit checks pass 187 assertions (+32 for format,
  nesting and invalid hydraulic inputs); full gate `sm8b-hydraulic-format-full` passes every stage.
  No new cast, old-version alias or compiler workaround was introduced.

- SM8b format gate `sm8b-hydraulic-format-full` passes CadKit, SheetMetalSmoke
  (7504), ProjectKit (187), ManufacturingKit (54), MachineKit, workspace, app build
  and project-source runtime. Original five MachineKit summaries and router
  engineering results match baseline; serialized assembly payloads change for v8
  and the new optional hydraulic field. Hydraulic runtime and press actuation
  remain in progress; no new Haxeon issue was found.

- SM8b runtime work in progress: the shared cylinder blueprint distinguishes pressure-effort
  pneumatic valves from flow-limited hydraulic valves. Hydraulic phase selection reads measured
  extension; agreeing coil states close the centre, and zero pressure applies zero effort rather
  than inheriting a joint force rating. RobotModel version 12 replaces all older accepted versions.
  Its hydraulic parameters use one typed wire payload within the existing JSON model boundary.
  The focused hydraulic model test passes 30 assertions, and the core-only package test passes
  (`sm8b-hydraulic-model-focus.log`, `sm8b-core-package.log`). Native binding regeneration passes
  the four-platform ABI audit (`sm8b-cylinder-hxi.log`). Hydraulic service wiring and physical press switches remain outstanding.

- The focused native hydraulic runtime passes 14 assertions on deterministic and MuJoCo
  backends (`sm8b-hydraulic-runtime-focus.log`): approach, working and return rates, closed-centre
  neutral, safe-coil emergency stop and zero-pressure loss of drive. The broader RobotKit world
  suite passes (`sm8b-robot-world-runtime.log`). Seven current-model fixtures are authored with
  typed model constructors and the current codec; comparison with their original facts preserves
  every existing field, including explicit collision policies. The MJCF importer writes version 12.
  The bridge derives the retracted coordinate from the signed guide limit after placement conversion.
  Full gate `sm8b-cylinder-runtime-full` passes the five quick stages with unchanged engineering
  baselines, then runs out of filesystem space during workspace compilation. The interrupted gate
  is retained as evidence; an obsolete isolated build is removed and the complete gate is retried
  serially as `sm8b-cylinder-runtime-serial-full`, without changing the original suite script.

- Haxeon issue encountered during SM8b: a null comparison on an inherited cylinder capability
  fails IR verification, including after a clean build. `IrInliner.splice` maps callee arguments
  directly to the actual receiver and substitutes a straight-line return without preserving its
  declared base type (lines 485–486 and 510–513); `IrVerifier` requires identical nominal types
  in equality (lines 309–316). The test checks the returned capability through a helper with an
  explicit `Null<CylinderDrive>` parameter instead. It compiles and preserves the force assertion
  without a cast. Haxeon sources are unchanged.

- SM8b runtime verification continues in `sm8b-cylinder-runtime-verified-full`. The serial
  retry first passed workspace tests but exposed one app test diagnostic still naming the
  removed pneumatic-only blueprint field; that diagnostic now uses `cylinderDrives`.
  The current complete retry passes the quick stages, workspace (including 5004 world assertions),
  app build and project-source runtime; every stage returns zero. Original five MachineKit engineering
  summaries remain identical to baseline. The maintained MuJoCo entry additionally passes seven
  hydraulic assertions (`sm8b-maintained-hydraulic-mujoco.log`) alongside the existing 64-assertion
  scenario. The current MJCF importer builds successfully (`sm8b-current-mjcf-build.log`);
  its existing complete-JSON fixture check also passes (`sm8b-current-mjcf-fixture.log`).

- SM8b runtime checkpoint is verified by full gate `sm8b-cylinder-runtime-verified-full`.
  SheetMetalSmoke remains 7504, ProjectKit 187 and ManufacturingKit 54. RobotKit world adds
  seven hydraulic assertions (5004 total); current model tests add 30 assertions, and the
  maintained MuJoCo entry adds seven while preserving its existing 64-assertion scenario.
  Both native backends demonstrate unloaded phase rates, closed-centre stopping, safe-coil
  stopping and an initially unpressurized cylinder supplying no effort; these checks do not
  assert a live pressure transition or loaded bend physics. Original five MachineKit and router
  engineering summaries are unchanged; model artifacts now carry the single current version 12.
  The shared bridge's edited-placement hydraulic home is derived from the signed nominal limit.
  The actual press cylinder/valve service bindings, physical switches and scripted stroke remain SM8b.

- SM8b fluid assembly checkpoint in progress: cylinder and valve capabilities now expose
  typed pressure-effort versus flow-limited/closed-centre variants. Hydraulic phase records
  save only authored speeds and change travel; cylinder geometry owns stroke. Every rebuild
  resolves pressure, forces, valve channels and guide sign from installed parts and hoses.
  Normal and reversed A/B hoses, current-codec rebuilding, authored phase edits/copy isolation,
  nested channel scoping and invalid pressure/phase/valve inputs pass the focused hydraulic
  tests (`sm8b-hydraulic-fluid-focus.log`). No cast or new compiler workaround is introduced.
  MachineAssembly has one current schema 14, removing the feature-dependent 9–12 readers.
  Version 13 is already a distinct hierarchical format on local main; 14 avoids reusing it.
  Hydraulic port id 7, cylinder/rod/source/closed-centre-valve recipes and modeled tank return
  are registered. The reference press wires both cylinders through one valve, two physical distribution blocks and eight hose
  assemblies, and derives regulated pressure as frame capacity divided by both bore areas.
  Joint connector datums move to the actual rod tips while guide placement and axis stay unchanged.
  Quick gate `sm8b-fluid-bindings-verified` passes all five suites; the physical contact switches and scripted stroke
  remain outstanding. Reference phase setup explicitly assumes 80 mm gap, 1.5 mm stock and
  10 mm clearance; other setups supply their profile rather than inferring stock thickness.

- The first fluid-binding quick gate exposed reused valve A/B outlets when wiring both cylinders.
  The service graph correctly requires a physical distribution fitting. Two hydraulic manifold
  parts now own distinct work outlets; the shared upstream tracer can stop at the bound valve,
  so cylinder compilation follows actual fittings before mapping A/B coil direction. Direct and
  distributed, normal and reversed work lines are covered by the focused hydraulic tests.
  `sm8b-fluid-bindings` is retained as the failed attempt; a fresh complete quick gate follows the fix.

- SM8b fluid assembly checkpoint passes `sm8b-fluid-bindings-verified`: CadKit,
  SheetMetalSmoke (7504), ProjectKit (187), ManufacturingKit (54) and MachineKit all return zero.
  Original five MachineKit engineering summaries match baseline exactly. Press checks cover both
  ram bindings, shared valve channels, geometry-preserving rod-tip joint datums and installed force
  equal to the assumed 392266 N frame capacity. Regulated pressure is approximately 9.75485 MPa;
  the independent 10 MPa cylinder area fixtures remain unchanged. The service includes a feed,
  tank return, A/B distribution blocks and eight explicitly assumed hose assemblies. Every saved
  MachineAssembly uses schema 14 and all tested recipe fixtures are reconstructed by current code;
  old feature-dependent format paths and pneumatic-only capability forwarding methods are deleted.
  The real finger contact mechanism and scripted empty-machine stroke remain the next checkpoints.

- SM8b passive-contact support is in progress. An authored passive joint carries no host
  position target or hold-at-rest servo; cylinders and motors cannot bind to it. Assembly
  definitions use current version 10 (hydraulic drive introduced v8, passive ownership adds v9, mechanical stop response adds v10), MachineAssembly uses 16 and RobotModel uses 13.
  Seven robot fixtures are regenerated through typed authors and the current codec, preserving
  every existing model fact. ProjectKit passes 192 assertions and the core-only package passes
  36 hydraulic/model assertions. The maintained MuJoCo entry passes three passive checks:
  host targets are rejected, a passive hinge falls under gravity with hold-at-rest enabled,
  and the equivalent ordinary hinge stays held. Existing hydraulic seven and wall-finishing
  64 assertions still pass (`sm8b-passive-maintained-mujoco.log`). The deterministic backend
  is deliberately kinematic for articulated bodies (`simkit/sim_core/src/physics_backend.cpp`);
  its ownership checks cannot establish gravity or contact response. Physical finger contact,
  switches and the scripted stroke remain outstanding; the final full gate is not yet run.

- SM8b passive-joint checkpoint full gate `sm8b-passive-joints-full` passes every stage,
  including workspace, app build and complete project-source runtime. ProjectKit increases
  from187 to192 assertions (current-version rejection and passive ownership/invalid bindings),
  core model checks from30 to36 (version rejection and passive save/load/actuator ownership),
  and RobotKit world from5004 to5007 (native command ownership and hold-at-rest behavior).
  MuJoCo adds three physical passive/controlled hinge checks alongside unchanged seven hydraulic
  and64 wall-finishing assertions. The original five MachineKit summaries, router cutting/drive
  results, pneumatic timings/force and welding engineering summaries are unchanged.
  AssemblyDefinition10, MachineAssembly16 and RobotModel13 each have one supported format;
  fixtures are written by current code. No new cast or Haxeon workaround is introduced.
  The physical press finger contact, ram switches and scripted stroke remain outstanding.

- The current MJCF writer also builds and reproduces its complete version13 walker fixture
  (`sm8b-passive-mjcf-build.log`, `sm8b-passive-mjcf-fixture.log`). The regenerated runtime
  binding retains the four-platform ABI audit (`sm8b-passive-runtime-hxi.log`).

- SM8b physical-contact checkpoint in progress: CAD-style per-link convex hulls now carry
  the same typed `ContactSurface` and `ShapeContact` settings as authored primitives.
  Unspecified settings keep backend defaults; the four-platform SimKit ABI audit passes.
  Maintained MuJoCo `PressFingerContactTests` checks gravity opening, sheet-contact closing
  and gravity reset for both primitive and convex-hull geometry (6 assertions at this checkpoint;
  the later assembled-contact audit below found a missing runtime-health assertion).
  The probe uses an explicitly assumed 2 ms contact time constant and damping ratio 1,
  with `PairsAndEnvironment` so environment defaults do not soften the contact by blending.
  These are simulation contact assumptions, not inferred material or manufacturer properties.
  Identical mass, geometry and switch windows fail with default compliance and pass with
  explicit settings; no mass or threshold tuning substitutes for contact physics.
  Finger assembly authoring and scripted stroke remain outstanding.

- SM8b hull-contact checkpoint full gate `sm8b-hull-contact-full` passes CadKit,
  SheetMetalSmoke (7504), ProjectKit (192), ManufacturingKit (54), MachineKit,
  every workspace suite, the app build and the complete project-source suite.
  The maintained MuJoCo runner passes all 6 finger-contact assertions alongside
  passive (3), hydraulic (7) and wall-finishing (64) assertions. All five original
  MachineKit summaries and 49 app engineering summaries match the previous
  passing gate; only planning/tick wall time and allocation statistics vary.
  Four-platform SimKit ABI audit `sm8b-hull-surface-hxi.log` passes. No new cast
  or compiler workaround. Assembled press switches and the scripted stroke remain.

- SM8b assembled-contact audit (2026-10-05, integration still in progress): focused
  ProjectKit scene-metadata and end-stop checks pass 215 assertions and `PressBrakeChecks` passes.
  The assembled MuJoCo test exposed a first-tick limit fault. The native debugger identified
  a reference slide at -1.151185936e-10 m with explicitly zero observed end-stop room.
  The reference slides now retain the bridge's established 1 mm default observation margin.
  The audit also found that the earlier 1 ms contact test faulted at tick 9 and could accept
  its last published position; its historical green result did not prove healthy operation.
  The maintained test now checks runtime fault and safety on every tick, and covers primitive
  and hull contacts at 1 ms and the normal app 10 ms tick. Explicit stops and contact response
  are derived from the gravity-drop energy under SM-D9; stock ends at its authored contact
  datum instead of being pushed 0.2 mm beyond it. Focused gate
  `sm8b-contact-resolution-focus.log` passes all 12 contact outcomes with no runtime fault.
  Native debugger evidence `sm8b-contact-native-substeps.log` confirms actual MuJoCo substeps.
  Assembly end-stop response is carried as typed current-schema data (assembly 9,
  MachineAssembly 16), including frozen descriptions and the regenerated hydraulic fixture;
  assembled press proof remains to be completed before
  the required quick/full gates and integration commit. The app event-time conversion now
  rounds in Float before Int64 truncation, avoiding Math.floor's 32-bit intermediate.

- The assembled-contact diagnostic confirms the authored finger mass (0.19782 kg), exact
  eight-corner CAD hull and 0.0004968594779560878 s contact/stop response survive lowering.
  An isolated test with the CAD-style offset child hinge frame passes all 12 outcomes
  (`sm8b-offset-finger-focus.log`), ruling out that frame convention as the cause of the
  assembled switch remaining open. The full assembly still needs physical contact proof.

- The assembled stock check must follow the measured backgauge carrier while contact settles;
  fixing the plate at an earlier carrier position accidentally overpushes the finger as the
  carrier settles. The typed timed-output player also accepts the runtime's documented cleared
  active plan id at terminal idle only after checking every submitted event receipt. Contact
  and stroke checks reset between scenarios so a looping second stroke cannot masquerade as
  the first stroke's home measurement. Stroke endpoint validation is still in progress.

- Empty-stroke timing audit: the first measured peak is 71.759993 mm and the same ideal
  70 mm return leaves 1.760022 mm above home. MuJoCo's velocity target uses a finite
  50/s acceleration response, and phase selection occurs on measured tick positions, so
  ideal flow durations are not exact position control. The demo now reserves the full nominal
  cylinder travel for return and checks actual positive installed-tool clearance plus the
  measured speed-change switch; the unsupported 70 ±1 mm test is removed. The 10 mm
  allowance is an ideal-timing assumption, not a claimed measured clearance. Exact depth
  control belongs to the press controller; physical home and safe end-stop behavior still
  need verification before the SM8b integration gate.

- The full-travel return reaches home at -0.000177 mm without a runtime fault, but a
  switch window starting exactly at zero excludes that seated observation. Sensor validation
  now permits windows within explicitly declared overtravel; commands and planning retain
  nominal limits. Reference slides explicitly declare the same existing 1 mm observation room,
  and top/bottom switches cover ±0.1 mm around their physical datums. Focused checks cover an
  accepted window within that room and rejection beyond it. This resolves the prior mismatch
  between end-stop observations and the nominal-travel-only switch validator.

- Focused assembled MuJoCo proof now passes (`sm8b-press-app-focus.log`): both CAD-hull
  fingers open by gravity, close against stock at the measured carrier datum and reopen on
  removal; the first native timed hydraulic cycle completes with a 71.759993 mm peak below
  the 80 mm installed-tool contact gap, and returns to its measured top switch. Reset/restart,
  native emergency-stop diagnosis and reset recovery also pass. Reference sensor validation
  checks pass `sm8b-press-switches-focus.log`, including the saved in-overtravel switch and
  rejection outside declared observation room. Required quick/full integration gates remain.

- SM8b final integration: quick gate `sm8b-assembled-press-verified` and full gate
  `sm8b-assembled-press-full` pass every stage. All five original MachineKit engineering
  summaries and the existing app engineering results remain unchanged; wall time and
  allocation statistics vary. The reference fingers now weigh 0.19782 kg each because their
  contact faces reach the stock datum (+0.01884 kg total). The new physical empty-stroke
  measurement is 71.759993 mm; it is not a depth-control guarantee. Saved formats are
  assembly 9, MachineAssembly 16 and scene 17, with typed contacts, stops and timed outputs.

- SM9a: typed manufacturing-stroke evaluation conserves the authored bend allowance and
  released radius, permits only the active bend to reach its process-loaded angle, and
  diagnoses incompatible predictions. Both signs, sampled strokes, loaded radius and
  ordinary/inactive-bend bounds pass 46 new assertions. Quick gate `sm9a-loaded-stroke`
  passes all five suites; original MachineKit engineering summaries are unchanged.

- SM9b focused checks: explicit process stock rebuilds P1–P5 with compatible radii and
  allowances (7594 SheetMetalSmoke assertions). The manual ±30 mm finger setup passes
  `PressBrakeChecks`, including connector-derived X/R datums and physical envelopes;
  default ±300 mm setup remains the reference. Quick gate `sm9b-manufacturing-setup` passes all five suites with unchanged original engineering summaries.

- SM9c placement foundation: typed orientation includes face, insertion direction and lateral
  bed centre. Each stroke sample aligns the evaluated bend bisector with the press, keeping
  its concave midpoint under the punch while both flanges rise symmetrically. The stable
  three-circle contact expression also handles the large-radius near-flat strip; the straight
  die-contact legs must remain positive. Calibrated depth offsets interpolate linearly from
  flat contact to the loaded endpoint, an explicit intermediate-motion assumption. Focused
  `sm9-placement-focus.log` passes both bend signs, insertion directions, near-flat and loaded
  samples, midpoint alignment, symmetric rotation and wrong-face rejection. Backgauge
  registration, setup refusals and the complete single-bend result remain to implement.

- SM9c quick gate `sm9c-press-placement` passes all five suites. Original MachineKit
  engineering summaries and diagnostics remain unchanged.

- SM9d registration foundation: exact wall-plane intersections derive contacts on straight
  and arc boundaries at each physical finger's lateral position. All contacts must share
  nominal X/R settings. Diagnoses distinguish a missed boundary, unreachable travel and
  incompatible contacts. Focused `sm9-registration-focus.log` passes P2 first-bend mould-line
  distance, later-bend `b + flange difference` (+20 mm), the folded flange's 40 mm R setting,
  both actual ±30 mm contacts, exact circular contacts and the three refusal cases. Quick
  gate `sm9d-backgauge-registration` passes all five suites with unchanged engineering results; complete setup feasibility/result remain.

- SM9 complete-API focused proof: `BendSetup` owns typed press geometry and takes the
  installed `ToolStation`, explicit orientation/gauge boundary and a `BendProcess`. Its
  derived result owns registration, process predictions, physical ram travel and sampled
  press-frame sheet poses/envelope. The sampled envelope is a broad-phase bound, not a
  collision-clearance certificate; SM10 consumes the actual poses. Stock/state compatibility,
  tonnage, both straight flange depths, material-bearing punch/die support and every sampled
  nominal ram position are checked with named diagnoses. `sm9-plan-focus.log` passes whole
  first/later setups, the independent flange oracles, depth/force, exact loaded endpoint,
  caller-state isolation, sampled bend containment and tonnage/short-flange/support/ram-travel
  refusals. Final quick gate `sm9-single-bend-complete` passes every suite with unchanged original engineering results.

- SM10a: local main `9e82c686b` has no CollisionKit, so SM-D10's interim exact prism
  SAT is used. Typed immutable prisms validate planar convex boundaries and positive
  transverse extrusion. The query tests both face-normal sets and all nonparallel edge
  cross-products, with an explicit 1e-8 mm penetration tolerance; mere contact is permitted.
  Focused `sm10-prism-focus.log` passes 24 checks: containment, contact/penetration,
  declared tolerance, skew rods whose faces all overlap but an edge axis separates,
  rigid transforms, nearly parallel directions, oblique extrusion/clockwise winding,
  array ownership and named invalid geometry. No RobotKit dependency or cast.

- SM10a quick gate `sm10a-prism-collision` passes all five suites; original MachineKit
  engineering summaries and diagnostics remain unchanged.

- SM10b: wall collision material is decomposed by vertical slabs across all authored loops.
  Straight boundaries are exact; arcs use chords with an explicit maximum sagitta and the
  actual maximum chord error retained in `SheetPrismProfile`. A one-million-chord guard
  diagnoses impossible requested resolution before the reduced Math.ceil Int conversion.
  Holes and concave reliefs are never bridged into a filled hull. The implementation is
  native-free and typed end to end. Focused `sm10-sheet-prisms-focus.log` passes 19 checks
  for exact line area/volume, concavity, square/circular holes, material/boundary probes,
  rigid placement, independently bounded annulus area error and array ownership. Quick
  gate `sm10b-sheet-prisms` passes all five suites with unchanged engineering results. These pieces carry curve approximation explicitly;
  their exact prism SAT result alone does not certify the original curved CAD geometry.

- SM10c: evaluated bend collision pieces reuse canonical developed material regions, so
  authored cuts remain holes through the loaded stroke. Uncut zones use annular sectors;
  cut zones clip developed regions into angular strips, triangulate material only, and map
  each triangle with a bounded constant thickness normal. Zero inner-radius hems use the
  outer face to avoid collapsed bases. Curvature, source chord and thickness-normal mapping
  errors are retained explicitly; SAT of these pieces alone is not an original-CAD clearance
  certificate. Focused `sm10-bend-prisms-focus.log` passes 66 checks for both bend signs,
  near-flat/partial/full/loaded states, independent sector volume, material probes, rectangular
  and circular cuts, and cut/uncut zero-radius hems. No native mesh dependency or cast.

- SM10c quick gate `sm10c-bend-prisms` passes all five suites, including the maintained
  66 bend-piece checks. The original 11 MachineKit engineering/diagnostic rows are unchanged.

- SM10d: `PressGeometry` retains the immutable installed `ToolLayout` from its assembly
  source. Single-bend setup rejects a foreign station specification reusing an installed ID.
  Collision bodies retain typed physical owners separately from piece names. Each evaluated
  stroke pose rebuilds wall/bend pieces with their existing material holes and error bounds.
  All installed punch/die segments participate at their actual bed/contact datums, including
  adjacent stations, independent gaps and end-horn/gooseneck reliefs. Punches follow the ram;
  dies stay fixed. Assembly-derived boxes cover the bed, front supports, ram/guide/bracket/
  torsion-bar members, backgauge carriage/lift and fingers with explicit motion ownership.
  Side-frame local boxes share their CAD dimensions and preserve the open throat, rather than
  filling its opening with a bounding box. The reference fixture supplies independent fixed
  frame/carrier dimensions. Focused `sm10-collision-geometry-focus.log` passes 73 checks for
  material/void probes, row ownership, ram/X/R movement, optional R absence/travel, frame
  volume against independent arithmetic and actual CAD, caller array ownership, foreign
  station refusal, wall holes and both bend signs through the loaded stroke. Geometry
  assembly does not yet classify intended contact or certify a collision-free setup; those
  checks, gripper feasibility and the complete sequence search remain in SM10.

- SM10d quick gate `sm10d-collision-geometry` passes all five suites, including the maintained
  73 press-frame geometry checks. All 11 original MachineKit engineering/diagnostic rows
  remain unchanged; sheet assertions remain 7594, ProjectKit 215 and ManufacturingKit 54.

- SM10e: exact line/arc contact sections reuse the authored punch/die profile data, including
  every four-V groove and end-horn clipping. This is analytic refinement of the interim
  prism query, not a whole-body contact exception. Positive-area overlap is found by exact
  curve events and alternating material intervals; tangency alone remains clear. Convex
  sheet prisms are clipped to the actual tool bed interval before projecting into Y/Z.
  Bed-parallel bend axes refine against exact annular/flat sections. Developed cut regions
  project only material present within that bed interval, preserving a hole that spans it.
  Other bend axes and curved wall approximations explicitly request the general error-aware
  query; they are never assumed clear. Focused `sm10-press-contact-focus.log` passes 786 checks:
  line/circle/circle contacts and penetration, holes, clipped skew prisms, exact catalogue
  and left/right horn volume against native CAD, conservative-tip/shoulder false-overlap
  witnesses, both bend signs through all 37 loaded-stroke samples, intentionally overclosed
  punch/raised die collisions, and developed-hole projection across narrow/wide tool spans.
- That cut test exposed a CadKit Boolean defect: joined retained fragments could partly
  duplicate an already retained coincident boundary, leaving a dangling open contour. The
  collector now subtracts already-covered coincident curve intervals before naming/assembly,
  preserving unaffected edge identities. Eight maintained regression assertions cover a
  strip ending exactly on a rectangular hole boundary in both operand orders; the sheet
  smoke count increases from 7594 to 7602 for these checks. Bend-strip clipping failures also
  retain the precise bend/sector and developed interval in their diagnosis. Because this
  fixes shared region Boolean code, SM10e runs the serial full gate, including its five
  quick suites, workspace, app build and project-source runtime. Full sequence feasibility,
  general curved-body clearance and gripper/search integration remain in SM10.

- SM10e serial full gate `sm10e-exact-contact-full` passes every stage. All 786 contact
  assertions are maintained in MachineKit; sheet assertions are 7602 (+8 Boolean regressions),
  ProjectKit remains 215 and ManufacturingKit 54. All 11 original MachineKit engineering/
  diagnostic rows and 58 selected app engineering/diagnostic rows match their earlier
  baselines after excluding planning wall time and allocation/profiling statistics.

- SM10f: `PressBendPrismQuery` transforms an exact convex obstacle into the uncut bend's
  own extrusion frame, clips it to the finite width and intersects the projected section
  with exact flat/annular material. Unlike tool-bed section refinement this supports any
  bend-axis direction, including previously folded flanges perpendicular to the bed. Cuts
  explicitly return an unsupported result because projecting them across the whole width
  would conflate material and obstacle at different along-axis positions. Approximate tool
  prisms still require their error-aware query. The maintained contact count increases
  786 to 1082 (+296 assertions): interior/exterior probes, rotated placement and hollow
  interior checks across both signed 37-sample strokes, and explicit cut refusal. Focused
  `sm10-bend-prism-focus.log` and quick gate `sm10f-bend-obstacles` pass; all 11 original
  MachineKit engineering/diagnostic rows match SM10e. Sheet remains 7602, ProjectKit 215,
  ManufacturingKit 54. No schema changes, casts or new compiler issues. This checkpoint
  supplies the exact bend-versus-machine primitive; full stroke feasibility/search remain.

- SM10g: `SheetCollisionQuery.separation` returns the largest signed unit-axis support
  gap, a certified lower distance bound when positive. `ConvexPrism.inset` clips each side
  face and shifts the caps by Euclidean face distance, including oblique extrusion. The
  material query distinguishes exact material overlap, gap clearance exceeding both errors,
  inset material collision, and unresolved geometry. Its explicit boundary-deformation
  contract preserves interior ownership; mere surface-distance proximity is insufficient.
  `PressStrokeCollision` checks every setup sample against all installed tool rows and
  assembly machine volumes, retaining typed physical owners, sample and active angle.
  It prefers a proved collision over an earlier unresolved pair and never labels uncertainty
  as clearance or physical collision. Installed upper clamps now participate and retain native
  volume and ram motion; returned frame-volume arrays cannot mutate the authored source.
- The P2 first-bend stroke exposed a real static-gauge interference: wall 2 hits the left
  finger at sample 1 (2.508145468378716 degrees). `BackgaugeStroke` makes withdrawal an
  explicit action: register, hold flat stock at first punch contact, sample X withdrawal,
  then bend at the withdrawn X/R. No finger or carrier is exempted. The maintained P2
  fixture's explicit 20 mm withdrawal clears every sample; zero withdrawal diagnoses the
  physical finger, and excessive/negative withdrawal is refused with named travel/input
  failures. Twenty mm is a fixture input, not a universal machine default. Sequence output
  must retain the checked withdrawal action and charge it to the time model.
- Focused `sm10-error-prism-focus.log`, `sm10-material-query-focus.log`,
  `sm10-clamp-geometry-focus.log` and `sm10-stroke-collision-focus.log` pass. The quick gate
  `sm10g-stroke-material` passes all five suites. Counts change only for new assertions:
  prism 24 to 50 (+6 metric gap and +20 inset), contact 1082 to 1748 (+666 material
  certainty), collision geometry 73 to 76 (+3 clamp/ownership). Sheet remains 7602,
  ProjectKit 215, ManufacturingKit 54; all 11 original MachineKit engineering/diagnostic
  rows match SM10f. No casts, schema changes or new compiler issues. Grip checks,
  refinement integration and search/BendProgram acceptance remain within SM10.

- SM10h: `PressWallPrismQuery` puts the stock normal on the canonical extrusion axis,
  clips the convex obstacle to the physical stock thickness and intersects its projected
  section with the original exact wall outline. It supports curved boundaries, all holes
  and arbitrary wall/obstacle orientation; tangency at either stock face is clear. The
  material query uses this exact refinement for all walls against exact machine/gripper
  prisms, even when their decomposed wall outlines retain a nonzero chord error. Tool
  contact refinement exposes its exact-prism path for grippers without inventing sheet
  ownership or exempting a tool.
- `GripCheck` is a function-only interface. `GripperBox` owns explicit rectangular patch
  and envelope dimensions; whole-patch Boolean subtraction checks material support.
  `SheetGrip` owns wall-local center and selected stock face and derives its outward frame
  from wall FK. `GeometricGripCheck` follows that box through every withdrawal and bend
  sample, checking all installed tools, machine volumes and sheet owners, including its
  own wall. It names physical interference separately from unresolved material geometry.
  The geometric candidate set is deliberately finite and deterministic: both stock faces
  at inset bounding-box endpoints and quarter positions, filtered by exact patch support.
  Sequence search will enumerate this set; it is not a completeness claim over continuous
  grip locations. Phase-C reach/IK remains an adapter through `GripCheck`.
- Maintained `GripChecks` passes 160 assertions: curved annular wall material and holes,
  boundary tangency/penetration, finite stock thickness and arbitrary rigid placement;
  a valid grip on either stock face through the 37-sample stroke and withdrawal; independent
  underside frame position/normal checks; full patch boundary refusal; an oversized box
  colliding with the actual ram; deterministic candidates; a valid grip after rebuilding a
  curved wall cut; and an off-center hole invalidating the patch although its center remains
  solid. The stand-in dimensions and withdrawal are explicit fixture inputs, not ratings.
- Face insets now reject subresolution remaining cap thickness and merge clipped vertices
  at the prism constructor's edge tolerance. Removing a near-duplicate convex vertex only
  shrinks the certificate. Two maintained regressions cover near-collapse cap thickness
  and a very long polygon whose area could otherwise conceal invalid tiny side edges;
  prism checks increase 50 to 52. Focused `sm10-grip-focus.log`,
  `sm10-grip-prism-focus.log`, `sm10-grip-contact-focus.log` and quick gate
  `sm10h-grip-geometry` pass. Contact remains 1748, collision geometry 76, sheet 7602,
  ProjectKit 215 and ManufacturingKit 54; all 11 original MachineKit engineering/diagnostic
  rows match SM10g. No casts, schema changes or new compiler issues. Refinement integration,
  search/time model, BendProgram and P1–P5 sequence acceptance remain within SM10.
