# Reusable panel tooling and mineral-casting molds

Status: phase 1 implemented. Panel generation and physical qualification remain pending.

## Implementation progress

- [x] Phase 1: neutral input/setup/stock records, strict versioned codec, stable
  fingerprints, typed input review, and the MachineKit foundation adapter.
  ManufacturingKit retains its ProjectKit-only dependency. CAD equivalence checks
  cover box/chamfered/flared envelopes, rotated inserts and existing hobber layouts.
  The initial contract supports mm inputs, convex additive prisms and box cuts;
  unsupported units must be converted before entering the contract.
- [ ] Phase 2: panel geometry, joints and fit coupon.
  In progress: phase 1 closed vocabularies converted to enum abstracts; tooling
  codec version 2 rejects version 1. Original insert role labels remain source
  descriptions alongside the typed role. The uncommitted panel prototype is being
  reconciled to exact line/arc operations, CadKit drawings and swept blocking.
- [ ] Phase 3: single-pocket box mold, end to end.
- [ ] Phase 4: mill base mold review assembly.
- [ ] Phase 5: assembly validation and fabrication package.
- [ ] Phase 6: mill column and independent pedestal example.
- [ ] Phase 7: physical calibration and release.

See [README.md](README.md#casting-tooling-inputs) for the implemented API and checks.

## Objective and first deliverable

Generate an inexpensive, self-locating mold kit from a machine's casting definition:
assembled and exploded CAD, numbered sheet panels, EPS cores, insert fixtures,
hardware, fabrication drawings, and an ordered assembly/demolding procedure.
Reuse the same tooling library for other machine beds, columns and pedestals.
Reuse the panel-joint library for jigs and fixtures that do not involve casting.

A single-pocket box mold first exercises the whole pipeline end to end. The
first full milestone is the bench-mill base mold, including its six
underside pockets, leveling-foot openings and cast-in steel interfaces. Follow
with the column mold and an independent pedestal example. Automatic assembly
means generated CAD placement and a self-locating physical kit; robotic physical
assembly is outside this release.

The initial construction recipe is melamine-faced particleboard contact panels,
external plywood/timber reinforcement, sacrificial EPS cavity cores, detachable
rigid detail formers, and independently located steel inserts. Start with nominal
18 mm panels as a configurable study value; measure stock before generating
joinery. Thickness, brace spacing, foam strength, resin compatibility and cure
behavior require evidence before fabrication approval.

## Existing code and constraints

Baseline at the start of implementation; the progress list above records changes.

- `machinekit.structural.MineralCasting` owns the casting envelope and box voids;
  its recipe also supports chamfered corners and a widened, tapered foot.
- `CastingFeatures.CastingBox` has stable feature IDs, dimensions and rigid poses.
  Features describe design cuts; they contain no draft or removal semantics.
- `MineralFoundation` adds insert recesses to those voids and exposes
  `insertLocations()`. Insert recesses must be classified separately from cores.
- `machinekit.milling.MillFrame` builds separate base and column foundations,
  underside pockets, service openings, rail/bearing inserts and foot details.
- `MillFrameLayout`, `MillXYStageLayout` and `MillZAxisLayout` supply datums.
  Derive mold dimensions from the instantiated foundations, not README numbers
  or a second copy of the frame layout calculations.
- `FoundationIntegrity` checks steel interface placement. It does not establish
  mold stiffness, insert anchorage or manufacturing accuracy.
- `MachineAssembly` supports connectors, mates, component recipes, preview and
  portable descriptions. Tooling must support recipe reconstruction as well.
- ManufacturingKit currently depends only on ProjectKit and owns `materia.sheet`.
  Its sheet planner handles rectangular blanks and straight guillotine cuts.
  Its SVG is a planning drawing, not a CNC toolpath or arbitrary contour nest.
- MachineKit depended on ProjectKit, CadKit and CollisionKit native; phase 1
  added the explicit ManufacturingKit dependency.
- ProjectKit has plywood and mineral-cast material entries. EPS and coated board
  need appropriate material representation; supplier process limits belong in
  tooling specifications rather than being inferred from display materials.
- `cadkit.drawing` already provides `Drawing2D` with exact lines/arcs and layers,
  an `Svg` writer and an ASCII R12 `Dxf` writer used by sheet-metal flat patterns.
  Tooling drawings use it; no new SVG or DXF writer is added.
- CamKit already depends on ManufacturingKit and turns contours into 2.5D outside/
  inside profiles, pockets with depth steps and drilled holes, with stock/fixture
  checks and LinuxCNC export. `CamSheetProfiles` is the precedent for bridging
  ManufacturingKit records into CAM. CamKit and ToolpathKit work in metres and
  `CamContour` is a polyline.
- The hot-wire foam cutter (`machinekit/examples/foam-cutter/PLAN.md`) plans a
  `camkit.foam.FoamJob` that accepts a CadKit solid or two paired profiles and
  checks ruledness between foam faces. It is not on main yet.
- CollisionKit has no shape-cast or continuous query, and CadKit has no Minkowski
  sum. CadKit `Shape.extrude(delta)` and booleans are available. Removal checks
  therefore need the swept-volume construction defined under validation.

## Package ownership

```text
machinekit/examples/bench-mill/       machine-specific setups and preview/export
                 |
                 v
machinekit.tooling                   CAD bridge, components, assembly, CAD checks,
          |                |          swept removal checks, drawings
          v                v
materia.tooling          CadKit (modeling, booleans, cadkit.drawing)
          |   ^
          v   |
  ProjectKit  camkit.CamToolingPanels   panel operations -> ToolpathProgram
```

CamKit already depends on ManufacturingKit and CadKit, so the panel CAM bridge
lives in CamKit beside `CamSheetProfiles`; neither ManufacturingKit nor MachineKit
gains a CAM dependency.

Proposed source locations:

- `manufacturingkit/haxe/src/materia/tooling/`: shared records, codecs,
  diagnostics, deterministic planning and export manifests.
- `.../tooling/panel/`: profiles, joints, stock faces, operations and panel plans.
- `.../tooling/casting/`: casting input, setup, cores, fixtures and mold plans.
- `.../tooling/export/`: cut lists, labels, manifests and the sheet blank bridge.
  It emits neutral profile/operation records, never SVG/DXF text.
- `machinekit/haxe/src/machinekit/tooling/`: foundation adapter, CAD realization,
  rebuildable components, assemblies, geometric validation, swept removal checks
  and `cadkit.drawing` rendering of panel profiles to SVG and DXF.
- `camkit/haxe/camkit/CamToolingPanels.hx`: panel operations to CamKit jobs.
- `machinekit/examples/bench-mill/MillMoldSetup.hx`: base/column configuration.
- `machinekit/examples/bench-mill/MillMoldPreview.hx`: generation entry point.
- `manufacturingkit/tests/tooling/` and `machinekit/tests/tooling/`: focused tests.
- `machinekit/examples/casting-tooling/`: independent pedestal reuse example.

ManufacturingKit must not import MachineKit, CadKit, milling types or OCC handles.
Use ProjectKit rigid frames and unit conversion where suitable. MachineKit adapts
the casting into neutral input, asks ManufacturingKit to plan the kit, and builds
the resulting geometry. Machine-specific setup code may choose splits and
overrides but must not implement its own joint or panel generator.

Do not add UI controls to the app in the first release. Use existing scene
artifacts and command-line entry points to make plans reviewable first.

## Data contracts

Names below are proposed implementation types. Add each contract in the phase
that first consumes it; phase 1 added the input, setup and stock records.
Prefer small typed records/enums over unvalidated dynamic maps. Closed
vocabularies are enum abstracts; strings are only for external references such
as supplier grades and evidence. Phase 2 starts by converting the phase 1
string fields `PanelStockSpec.coatedFace`, `PanelStockSpec.directionConstraint`,
`CastingToolingInput.lengthUnit`, `CastingToolingInsert.role` and
`CastingToolingInsert.finish` to enum abstracts, with a codec version bump and no
compatibility path for the old encoding.

| Contract | Required contents |
| --- | --- |
| `ToolingRecipe` | ID/revision, panel/core stock, joint policy, process limits, fit allowances, seal/liner thickness, hardware choices |
| `PanelStockSpec` | Material reference, nominal and measured thickness, variation, coated face, stock dimensions, permitted rotations/direction |
| `CoreStockSpec` | EPS grade reference, available block sizes, density, declared compression/temperature limits, layer adhesive, evidence references |
| `CastingToolingInput` | Source ID/revision/hash, supported envelope, named void features, insert geometry/poses, precision datums |
| `CastingSetup` | Source-to-mold transform, gravity/fill direction, open face, parting choices, feature overrides, support and finishing datums, compaction |
| `CompactionSpec` | Method (vibration table, external vibrators, internal poker, none), mold-bed interface, declared frequency/amplitude/duration or pending, evidence |
| `InsertAnchorage` | Insert reference, anchorage kind (studs, undercut, bonded keys), geometry, declared pull-out/shear capacity or pending, evidence |
| `PanelPlan` | Stable ID, exact 2D profile, stock/thickness, mold pose, contact face, joints, operations, label zone, removal motion |
| `PanelOperation` | Stable ID, kind (outside/inside profile, pocket, rebate, drill), exact line/arc contour or hole centres, machining face, depth, through/blind, setup |
| `JointPlan` | Paired member/edge references, joint kind, datum role, clearances, reliefs, fastening and assembly direction |
| `CorePlan` | Source feature IDs, actual cavity intersection, material, layers, mold pose, covering allowance, restraint, removal strategy, cutting definition |
| `InsertFixturePlan` | Insert references, independent locating datums, fixing hardware, insert-sealed penetrations, threaded-hole protection, withdrawal and finishing allowance |
| `PenetrationPlan` | Hole/slot reference, class (outside casting, insert-sealed, plugged, fill/vent opening), sealing face or plug, seal material |
| `DesignChangeRequest` | Source feature/insert IDs, requested change (draft, lifting insert, split, radius, access), reason, blocking checks |
| `MoldPlan` | Panels, formers, cores, fixtures, braces, hardware, penetrations, steps, change requests, assumptions and source provenance |
| `ToolingValidation` | Rule ID, affected IDs, severity, evidence, and pass/fail/pending/unsupported status |
| `ToolingManifest` | Format version, units, revisions/hashes, generated files, validation status, quantities and replacement groups |

`PanelOperation` maps one-to-one onto a CamKit operation: outside/inside profile,
pocket (rebates are pockets with an open side) or drill. The kind is stored, not
inferred from depth or shape. Parametric cut shapes (relieved rectangle, circle,
obround) may be the stored form, provided each yields one exact line/arc contour
through a single function shared by CAD, drawings and CAM. Contours keep exact arcs
in the record; `CamToolingPanels` samples arcs to a declared chord tolerance
while `CamContour` remains a polyline, and stops sampling once CamKit contours
carry arcs. Profiles are finished geometry: CamKit applies cutter compensation.

`DesignChangeRequest` is how tooling asks for a casting change it may not make
itself. Requests are reported against source IDs and block fabrication readiness
for the checks they name until the casting revision that satisfies them is used.
The machine design owns accepting or rejecting them.

All generation coordinates are millimetres with documented right-handed local
frames. Panel profiles use a documented lower-left blank origin, local X/Y in the
sheet and +Z through thickness. Define the coated/contact face explicitly.
Convert supported input units at the boundary; reject non-finite dimensions,
invalid rotations and mixed implicit units. Preserve source feature IDs through
generation. Stable IDs must not depend on array iteration order.

Expose a validated, defensive tooling snapshot from `MineralFoundation` or a
shared foundation adapter. Do not expose mutable internal feature arrays or
parse private geometry. Include insert-recess classification and the supported
envelope parameters. Existing casting recipes must reconstruct unchanged.

The planning input must describe how the casting envelope is built, including
the flared foot when present. A bounding box alone is insufficient. Support
explicit box and extruded-profile envelopes first; reject unsupported shapes.
Clip design-cut tools to the casting envelope: existing cuts deliberately extend
beyond casting faces, and those extensions are not literal finished cavity sizes.

Store plan/setup records in a versioned tooling JSON format separate from sheet
inventory. Validate codec input, bounded record counts and referential integrity.
Use deterministic serialization and source/recipe fingerprints. Geometry or
recipe changes invalidate generated reports and physical qualification tied to
the previous version. Do not serialize native CAD resources.

## Generation pipeline

1. **Adapt:** obtain casting envelope, voids and steel interfaces from the source.
2. **Orient:** apply the selected setup; classify open faces and fill height.
3. **Classify:** resolve every feature as EPS core, reusable former, cast-in insert
   recess or explicitly unsupported. Never silently omit a feature. A feature that
   needs a casting change to be moldable yields a `DesignChangeRequest`.
4. **Split:** build the bed, independently detachable walls and special split
   panels from an explicit supported mold template.
5. **Allocate:** create EPS blocks/layers and rigid detail formers; resolve
   intersecting cavities into an ordered, non-overlapping tooling arrangement.
6. **Fixture:** locate inserts and cores; reserve fixture/tool access and prevent
   unintended connections to embedded hardware after cure. Classify every hole
   through a contact panel in a `PenetrationPlan`.
7. **Join:** add external alignment features, closure hardware, reinforcement and
   the compaction interface (table clamping or vibrator mounts) from `CompactionSpec`.
8. **Realize:** build CAD from the resolved plan using the same profiles and poses
   consumed by drawings and lists.
9. **Validate:** check geometry, sequence, access, material/process assumptions and
   supported load calculations.
10. **Export:** emit a versioned review package, including design-change requests;
    expose fabrication readiness separately from geometric validity.

Version 1 uses explicit setups and rule-based templates. It does not infer a
general mold split from arbitrary BREP geometry. Draft is an explicit parameter
or design change; never modify the machine casting silently to improve demolding.
Request draft, lifting points or splits through `DesignChangeRequest` instead.

## Panel joints and assembly behavior

Implement a small useful set first:

- **Rebated removable corners:** continuous sealing lands, with external closure
  bolts or clamps. Model the remaining stock thickness at rebates.
- **Tab-and-slot external ribs/flanges:** locate reinforcement and wall positions
  outside the casting boundary. Keep resin-contact panels continuous.
- **Locating pins plus clearance bolts:** repeatable core-bridge and precision
  fixture placement. A primary round pin and a secondary relieved/slot location
  can locate without overconstraining stock variation.

Calculate mating dimensions together from one joint record. Separate locating
clearance from closure clearance; bolts supply clamp force. Include measured sheet
thickness, lining thickness and machining allowance. Calibrate fit with a coupon.
Use asymmetric layouts and paired labels to prevent reversed assembly where
practical; also validate the permitted installation orientation geometrically.

For router-cut slots, support corner reliefs or compatible rounded tabs with an
explicit cutter diameter and minimum ligament checks. Keep rebates and pockets
as `PanelOperation` records, not just final outlines. Record machining side, depth,
through/blind status and setups. The cutter diameter used for reliefs must match
the tool CamKit is given for that operation. Avoid hidden captive hardware that
becomes inaccessible when adjacent panels are fitted.

Finger joints, wedge tenons, captive-nut joints, toggle-clamp variants and
dovetails are later extensions. Avoid permanent interlocks between walls that
must detach separately. The dependency graph must reject cyclic assembly or
demolding orders instead of exporting an attractive but trapped kit.

## EPS cores and precision fixtures

Use cut construction-grade EPS for large nonprecision cavities. Specify supplier
grade and declared strength; density alone is not a strength qualification.
Represent sacrificial cores separately from reusable tooling and track their
per-casting consumption. Reusable formers need an extraction direction; EPS
needs accessible breakout space and a removal step.

Support rectangular blocks and laminated layers first. Export block dimensions,
layer thicknesses, orientation, compatible adhesive assumptions and labels.
Account for covering thickness in finished cavity size. Stepped shapes need
explicit cuts or layers.

Each core's cutting definition is a block in stock coordinates, a wire direction
and the core solid, which must be ruled between the two foam faces the wire
enters and leaves. Prismatic blocks and layers satisfy this trivially. This is
the input `camkit.foam.FoamJob` accepts, so the in-repo hot-wire cutter can cut
cores once foam-cutter lands, without changing `CorePlan`. FoamJob owns kerf;
`CorePlan` holds finished core geometry and never pre-compensates. Until then,
export the same definition as drawings. Thermal process tuning is FoamJob's.

Model smooth release covering and sealed seams as a chosen process. Record
compatibility and cure-temperature evidence for the actual resin/EPS/adhesive/
release combination. Do not infer compatibility from the generic material name.
Two combinations fail outright rather than waiting for evidence: a resin system
declared as styrene-bearing (polyester, vinyl ester) with an EPS core or EPS-based
adhesive, and a declared cure peak temperature at or above the core stock's
declared maximum temperature. The resin record therefore declares solvent
family and expected peak temperature for the section thickness, or pending.

Restrain cores with broad plates or mechanical bridges connected to the outer
tooling. Calculate displaced volume once for overlapping features. Show uplift
and pressure assumptions, attachment loads and supported bearing areas. Do not
treat a screw embedded in EPS as a qualified restraint.

Use rigid formers for small access pockets, bores and dimensions that foam cannot
hold reliably. Steel rail and bearing inserts use independent fixture datums;
cheap panel flatness does not establish final machine alignment. Include actual
fixture attachment interfaces, release access, thread protection, finishing stock
and physical insert anchorage as explicit inputs or pending decisions. Existing
box-shaped inserts alone do not establish an anchorage design; anchorage is an
`InsertAnchorage` record per insert, and its geometry joins the void and overlap
checks once supplied.

Precision inserts are normally fixed by bolting through the bed or a datum bar
into their own threaded holes. Those holes penetrate a contact panel but are
closed by the insert's mounting face: record them as insert-sealed penetrations
with the sealing face, seal material and the thread protection that keeps resin
out. Plug any threaded hole not used for fixing.

## Validation and readiness

Run pure record checks before CAD. Then check the following against the resolved
assembly, using CadKit modeling and booleans and closing native resources.

- Profiles are valid; joints match; holes have edge distance; reliefs and rebates
  preserve required material; labels avoid contact and machining areas.
- Casting boundary coverage is continuous except the intended fill opening.
  Every contact-panel hole or slot inside the casting boundary has a
  `PenetrationPlan`; insert-sealed penetrations lie fully within the insert's
  mounting face less a seal margin; unclassified penetrations fail.
- No unintended positive-volume overlap exists among panels, cores, inserts and
  the intended casting. Classify shared contact faces and intentional overlaps
  explicitly rather than using broad collision exclusions.
- The union of cores, formers and inserts reproduces the intended casting voids.
  Compare the reconstructed casting to the source using symmetric difference
  and tolerances; detect missing voids and excess cuts.
- Each assembly step has its prerequisites and hardware/tool access. Each reusable
  part can leave along its declared motion after prior removals. Validate the
  complete translation sweep, not only endpoint poses. Treat unimplemented
  rotational or complex removal paths as pending/unsupported.
- Swept removal is exact for straight translations. The volume swept by solid A
  moving by vector v is A ∪ (A + v) ∪ the prisms of A's boundary faces extruded by
  v (`Shape.extrude(v)`), skipping faces parallel to v: any swept point lies on a
  segment from A's boundary along v. Choose |v| to clear the bounds of all parts
  still present. A is blocked by B in direction v when B's interior meets that
  swept volume beyond declared contact tolerance. These relations form a
  directional blocking graph; the assembly and demolding orders must be
  topological orders of it, and any cycle is reported with its member IDs.
  Sampled poses are not used.
- Sacrificial-core breakup openings and handling access exist. Separation of the
  mold from the cured casting, including bed release, has a feasible sequence.
- Insert datum positions and finishing allowances survive setup transforms.
- Calculate liquid-head pressure with documented density and fill height, and
  core buoyancy with documented displaced volume. Treat these as baseline load
  estimates; vibration, tamping and aggregate bridging require explicit process
  loads from `CompactionSpec` or physical evidence. Vibration is usually the
  governing lateral load, so a mold without a declared compaction method keeps
  its structural checks pending.
- Panel/brace/fastener checks use applicable material properties, support geometry,
  load factors and deflection limits. Panel spans between braces use closed-form
  rectangular-plate deflection and stress under the hydrostatic or declared
  pressure, taking simply supported edges as the bound unless edge fixity is
  evidenced; braces are beams carrying tributary panel load; fasteners and core
  bridges carry their reaction shares. Missing data yields pending status. Do not
  label the mold structurally qualified from a hydrostatic calculation alone.
- Handling: the cured casting's mass, centre of mass and every flip or lift in
  the demolding sequence have declared lifting points. Lifting points that need
  cast-in hardware are `DesignChangeRequest`s until the casting provides them.

Export review drawings even with pending process checks, clearly labeled. Block
fabrication-ready status for failed geometry, unsupported features or unresolved
critical checks. Store per-check evidence and limitations; avoid one ambiguous
overall `valid` flag. Physical coupon results must be attributable to a particular
recipe and setup revision.

## Mill setups

### Base first

- Trial orientation: base upside down, Y rail faces toward the bed and underside
  cavities toward the open fill side.
- Locate Y rail and bearing inserts with independent datum bars, bolted through
  into the inserts' threaded holes as insert-sealed penetrations; support other
  insert faces with explicit fixtures rather than assuming one common plane.
- Six EPS pocket cores retained by removable bridges. Use actual pocket sizes
  from the frame, preserving longitudinal beams, transverse rib and front bridge.
- Side-withdrawing rigid foot-access formers and separate stem formers; account
  for their intersection and steel seat bores.
- Reproduce shallow drive-seat and rail-shoulder recesses and column mounting
  pads. Inventory every feature ID from the foundation adapter.
- Define detachable walls, sealing lands, closure points and a handling/demolding
  sequence for the actual casting mass; do not rely on an unspecified manual flip.
  The upside-down setup needs a flip after cure, so expect lifting-point
  `DesignChangeRequest`s against the base foundation.
- Declare the compaction method; the bed carries its table or vibrator interface.

### Column second

- Trial orientation: rail face toward the bed, rear-open chambers upward.
- Locate Z rail and bearing inserts independently; maintain access to sole pads
  and rear hatch anchors in their different planes.
- EPS rear-chamber cores with accessible breakout. Resolve the Z-drive channel,
  lower motor bay and intersecting openings with removable rigid details as needed.
- Support the actual column envelope, including a flared/tapered foot and bolt
  access wells when the selected layout enables them.
- Demonstrate removal of the motor-bay former through the actual service opening;
  use a split or sacrificial alternative if a continuous former cannot withdraw.

Both orientations are proposals to validate, not frozen process decisions. Build
from the same complete frame instance used by the preview so optional XY/Z
features cannot disappear through a simplified adapter.

## Exports and reproducibility

Write generated mill output under `machinekit/examples/bench-mill/build/tooling/`,
with separate `base/` and `column/` subdirectories. Use a dedicated tooling build
configuration/output directory to avoid collisions with existing frame builds.

Each review package contains:

- `manifest.json`, normalized tooling plan and machine-readable validation report.
- Assembled/exploded `.mtrg` scenes; reusable part/assembly recipe descriptions.
- STEP geometry for CAD review, with explicit local frames and units.
- Per-panel SVG and DXF drawings through `cadkit.drawing` at declared scale:
  outline, drill centers, pocket/rebate depths, machining face, labels and datum
  dimensions, one layer per operation kind.
- Per-panel LinuxCNC programs from `CamToolingPanels` and CamKit, with the tool,
  setup, stock and fixture data they were checked against.
- EPS core cutting definitions (block, wire direction, core solid as STEP).
- `DesignChangeRequest` list with the checks each one blocks.
- CSV panel blanks, EPS layers/blocks, fixtures, hardware and reusable/consumable
  quantities; optional cost estimates only from supplied prices.
- Human-readable assembly, sealing, fixture release, cure prerequisites and
  demolding instructions tied to the ordered steps.
- Joint fit coupon drawings and a physical trial record template.

Drawings and CAM programs derive from the same `PanelOperation` records as the
CAD. Profiles are finished geometry; CamKit applies cutter compensation, so it is
never applied twice. A generated program is a review output checked by CamKit's
stock, fixture and travel rules; running it on a particular router is a shop
decision recorded with the machine and tool used.

Bridge panel bounding blanks into `SheetPartRequirement`, retaining finished
profile IDs and revisions. Keep detailed routing/drilling separate from guillotine
blank cutting. Do not claim automatic contour nesting or consume sheet inventory
as a side effect of generation. Preserve existing sheet codecs and execution
semantics.

## Implementation phases and acceptance criteria

### 1. Contracts and package boundaries

Add neutral tooling records, typed validation, versioned codec and unit/frame
conventions. Add the foundation snapshot/adapter and explicit MachineKit dependency
on ManufacturingKit. Define material/process records without changing existing
casting or sheet formats.

Acceptance: round-trip a small box foundation with a void and insert; reject
invalid references, dimensions, frames and versions. Existing casting and sheet
tests pass. ManufacturingKit still builds without CAD or MachineKit dependencies.

### 2. Panel system and joint coupon

First convert the phase 1 closed-vocabulary strings to enum abstracts (see data
contracts). Implement panel profiles and `PanelOperation`s, rebates, external
tab-and-slot joints and pin/bolt positioning. Add CAD components with registered
rebuild recipes and a small generic panel fixture example. Render panel drawings
through `cadkit.drawing`. Implement the swept-volume construction and blocking
graph in `machinekit.tooling`, since panel kits already need it. Generate the
joint calibration coupon.

Acceptance: measured-thickness changes update both mating sides consistently;
router reliefs are manufacturable; reversed or trapped assembly cases are found
by the blocking graph, including a case that endpoint-only checks pass;
CAD reconstruction preserves dimensions, IDs, materials and connector frames.
SVG and DXF profiles come from the same operation records as the CAD.
Panel logic has no casting-specific dependency.

### 3. Single-pocket box mold, end to end

Before the full base, mold a box casting with one EPS pocket core, one steel
insert bolted through the bed and a five-panel box. Run the whole pipeline:
adapter, setup transform, classification, core and fixture plans, penetrations,
CAD realization, rebuild recipes, assembled/exploded scene, casting
reconstruction and the blocking graph for assembly and demolding.

Acceptance: the reconstructed casting matches the source by symmetric
difference; the insert-sealed penetration passes and an unclassified hole fails;
the core's cutting definition is a valid ruled block; changing panel thickness or
setup orientation regenerates a consistent kit; a trapped variant is rejected.

### 4. Base mold review milestone

Implement the box-mold planner, feature classification, EPS blocks, detail
formers, core bridges and insert fixtures. Add `MillMoldSetup`, `MillMoldPreview`
and a dedicated `haxeon.tooling.json` entry point for base-only generation first.

Acceptance: all current base features and inserts have an explicit tooling
strategy; assembled/exploded views are inspectable; reconstructed casting matches
the source; core restraints and panel removal steps are represented. Unresolved
physical assumptions and design-change requests (lifting points at least) are
visible. No fabrication-ready claim is made yet.

### 5. Assembly validation and fabrication package

Implement full sequence validation over the blocking graph, seam and penetration
coverage, load-estimate reports (hydrostatic, buoyancy, plate/brace/fastener,
compaction when declared, handling) and export manifests. Add per-part SVG/DXF/
CSV, the blank planning bridge, `camkit.CamToolingPanels` with CamKit-checked
LinuxCNC programs, core cutting definitions and instructions. Preserve recipe
restoration throughout.

Acceptance: negative fixtures catch trapped walls, missing pockets, displaced
inserts, leaking slots, unclassified penetrations, styrene resin with EPS, cure
peak above the core limit, and inadequate/pending restraint information.
Drawings, CAM programs, CAD and cut lists derive from the same records; repeated
generation is stable. Missing material/process evidence, an undeclared
compaction method or an open design-change request cannot become a passing
fabrication check.

### 6. Column and independent structure

Add the column setup, stepped/flared envelope support, intersecting channels and
side-removal details. Add a small mineral-cast pedestal example with different
dimensions, insert layout and opening direction using the shared planner.

Acceptance: both selected mill foundations generate complete review packages;
the pedestal uses shared records/joints/planner with configuration only. Changing
stock thickness, casting size, opening direction and core strategy requires no
mill-specific conditionals in the libraries. Existing frame geometry is unchanged.

### 7. Physical calibration and release

Fabricate a joint coupon and a representative small mold with EPS core, steel
insert, covering, seam and restraint, using the phase 3 box where practical.
Record actual dimensions, release behavior, core deformation, peak cure
temperature, compaction method and its measured settings, and insert pull-out if
tested. Adjust recipe values from evidence. Review full mold loads, handling,
insert anchorage and finishing.

Acceptance: evidence is attached to the relevant recipe revision; full mill
fabrication readiness is decided against its actual dimensions and loads. A small
successful coupon alone does not qualify a much larger casting. Manufacture full
molds only after the outstanding process and structural checks are resolved.

## Test and delivery strategy

Deliver each phase as a focused commit or small series with documentation and
meaningful checks. Do not mix unrelated machine or app work into these changes.

Pure tests: unit/frame conversion, deterministic codec, joint fit across sheet
thickness variation, sequence cycles, invalid/unsupported inputs and rectangular
blank conversion, penetration classes and the hard resin/EPS rules. Geometric
tests: cavity equivalence, insert fixture placement, rebate/slot material, swept
volumes and the blocking graph, and recipe reconstruction. Include negative
geometries that would pass endpoint-only or bounding-box-only checks. CAM tests
cover `PanelOperation` to CamKit mapping, arc sampling tolerance and CamKit's
stock/fixture rejection of a bad setup.

Add dedicated configurations:

```sh
./haxeon/scripts/haxeon run --project=manufacturingkit/tests/tooling/haxeon.json
./haxeon/scripts/haxeon run --project=machinekit/tests/tooling/haxeon.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.tooling.json
```

The first two configurations are implemented in phase 1. The mill tooling entry
point is planned for phase 4. `CamToolingPanels` tests join the existing CamKit
suite in phase 5. Existing regression commands are:

```sh
./haxeon/scripts/haxeon run --project=manufacturingkit/tests/haxeon.json
./haxeon/scripts/haxeon run --project=machinekit/tests/casting/haxeon.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.frame-tests.json
```

Run the existing bench-mill configurations sequentially because some share build
outputs. Use the current Haxeon/native dependency setup; do not introduce a new
build mechanism. Visually inspect the assembled/exploded artifacts as well as
running automated checks. Keep generated full-size geometry under ignored build
directories; commit source setups, small test fixtures and documentation.

## Deferred scope

General BREP parting-surface discovery, arbitrary contour nesting, robotic mold
assembly, wiring core cutting definitions into `FoamJob` (after foam-cutter lands
on main), CNC posts beyond CamKit's LinuxCNC writer, arc contours in CamKit,
rotational removal paths, automatic hardware optimization, full cure/flow/
compaction simulation, FEM of the mold, full structural qualification and app
editing UI are follow-on work. Preserve extension points for them without implementing
placeholder subsystems. The initial release succeeds when the shared libraries
produce complete, reproducible and honestly validated mold kits for the two mill
castings and the independent pedestal.
