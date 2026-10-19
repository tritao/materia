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
- [ ] Phase 3: mill base mold review assembly.
- [ ] Phase 4: assembly validation and fabrication package.
- [ ] Phase 5: mill column and independent pedestal example.
- [ ] Phase 6: physical calibration and release.

See [README.md](README.md#casting-tooling-inputs) for the implemented API and checks.

## Objective and first deliverable

Generate an inexpensive, self-locating mold kit from a machine's casting definition:
assembled and exploded CAD, numbered sheet panels, EPS cores, insert fixtures,
hardware, fabrication drawings, and an ordered assembly/demolding procedure.
Reuse the same tooling library for other machine beds, columns and pedestals.
Reuse the panel-joint library for jigs and fixtures that do not involve casting.

The first reviewable milestone is the bench-mill base mold, including its six
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
- MachineKit currently depends on ProjectKit, CadKit and CollisionKit native.
  Adding ManufacturingKit is an explicit dependency change, not an existing edge.
- ProjectKit has plywood and mineral-cast material entries. EPS and coated board
  need appropriate material representation; supplier process limits belong in
  tooling specifications rather than being inferred from display materials.

## Package ownership

```text
machinekit/examples/bench-mill/       machine-specific setups and preview/export
                 |
                 v
machinekit.tooling                   CAD bridge, components, assembly, CAD checks
          |                |
          v                v
materia.tooling          CadKit / existing collision facilities
          |
          v
      ProjectKit
```

Proposed source locations:

- `manufacturingkit/haxe/src/materia/tooling/`: shared records, codecs,
  diagnostics, deterministic planning and export manifests.
- `.../tooling/panel/`: profiles, joints, stock faces, operations and panel plans.
- `.../tooling/casting/`: casting input, setup, cores, fixtures and mold plans.
- `.../tooling/export/`: profile drawings, cut lists, labels and sheet bridge.
- `machinekit/haxe/src/machinekit/tooling/`: foundation adapter, CAD realization,
  rebuildable components, assemblies and geometric validation.
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

Names below are proposed implementation types; finalize them during phase 1.
Prefer small typed records/enums over unvalidated dynamic maps.

| Contract | Required contents |
| --- | --- |
| `ToolingRecipe` | ID/revision, panel/core stock, joint policy, process limits, fit allowances, seal/liner thickness, hardware choices |
| `PanelStockSpec` | Material reference, nominal and measured thickness, variation, coated face, stock dimensions, permitted rotations/direction |
| `CoreStockSpec` | EPS grade reference, available block sizes, density, declared compression/temperature limits, layer adhesive, evidence references |
| `CastingToolingInput` | Source ID/revision/hash, supported envelope, named void features, insert geometry/poses, precision datums |
| `CastingSetup` | Source-to-mold transform, gravity/fill direction, open face, parting choices, feature overrides, support and finishing datums |
| `PanelPlan` | Stable ID, local 2D profile, stock/thickness, mold pose, contact face, joints, holes/pockets, label zone, removal motion |
| `JointPlan` | Paired member/edge references, joint kind, datum role, clearances, reliefs, fastening and assembly direction |
| `CorePlan` | Source feature IDs, actual cavity intersection, material, layers, mold pose, covering allowance, restraint and removal strategy |
| `InsertFixturePlan` | Insert references, independent locating datums, fixing hardware, threaded-hole protection, withdrawal and finishing allowance |
| `MoldPlan` | Panels, formers, cores, fixtures, braces, hardware, steps, assumptions and source provenance |
| `ToolingValidation` | Rule ID, affected IDs, severity, evidence, and pass/fail/pending/unsupported status |
| `ToolingManifest` | Format version, units, revisions/hashes, generated files, validation status, quantities and replacement groups |

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
   recess or explicitly unsupported. Never silently omit a feature.
4. **Split:** build the bed, independently detachable walls and special split
   panels from an explicit supported mold template.
5. **Allocate:** create EPS blocks/layers and rigid detail formers; resolve
   intersecting cavities into an ordered, non-overlapping tooling arrangement.
6. **Fixture:** locate inserts and cores; reserve fixture/tool access and prevent
   unintended connections to embedded hardware after cure.
7. **Join:** add external alignment features, closure hardware and reinforcement.
8. **Realize:** build CAD from the resolved plan using the same profiles and poses
   consumed by drawings and lists.
9. **Validate:** check geometry, sequence, access, material/process assumptions and
   supported load calculations.
10. **Export:** emit a versioned review package; expose fabrication readiness
    separately from geometric validity.

Version 1 uses explicit setups and rule-based templates. It does not infer a
general mold split from arbitrary BREP geometry. Draft is an explicit parameter
or design change; never modify the machine casting silently to improve demolding.

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
as machining operations, not just final outlines. Record machining side, depth,
through/blind status and setups. Avoid hidden captive hardware that becomes
inaccessible when adjacent panels are fitted.

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
Account for covering thickness in finished cavity size. Hot-wire templates must
state kerf allowance, and stepped shapes need explicit cuts or layers. Hot-wire
controller output, robot motion and thermal process tuning are deferred.

Model smooth release covering and sealed seams as a chosen process. Record
compatibility and cure-temperature evidence for the actual resin/EPS/adhesive/
release combination. Do not infer compatibility from the generic material name.

Restrain cores with broad plates or mechanical bridges connected to the outer
tooling. Calculate displaced volume once for overlapping features. Show uplift
and pressure assumptions, attachment loads and supported bearing areas. Do not
treat a screw embedded in EPS as a qualified restraint.

Use rigid formers for small access pockets, bores and dimensions that foam cannot
hold reliably. Steel rail and bearing inserts use independent fixture datums;
cheap panel flatness does not establish final machine alignment. Include actual
fixture attachment interfaces, release access, thread protection, finishing stock
and physical insert anchorage as explicit inputs or pending decisions. Existing
box-shaped inserts alone do not establish an anchorage design.

## Validation and readiness

Run pure record checks before CAD. Then check the following against the resolved
assembly, using existing CAD/collision facilities and closing native resources.

- Profiles are valid; joints match; holes have edge distance; reliefs and rebates
  preserve required material; labels avoid contact and machining areas.
- Casting boundary coverage is continuous except the intended fill opening.
  Locating slots and closure holes do not create unsealed cavity penetrations.
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
- Sacrificial-core breakup openings and handling access exist. Separation of the
  mold from the cured casting, including bed release, has a feasible sequence.
- Insert datum positions and finishing allowances survive setup transforms.
- Calculate liquid-head pressure with documented density and fill height, and
  core buoyancy with documented displaced volume. Treat these as baseline load
  estimates; vibration, tamping and aggregate bridging require explicit process
  loads or physical evidence.
- Panel/brace/fastener checks use applicable material properties, support geometry,
  load factors and deflection limits. Missing data yields pending status. Do not
  label the mold structurally qualified from a hydrostatic calculation alone.

Export review drawings even with pending process checks, clearly labeled. Block
fabrication-ready status for failed geometry, unsupported features or unresolved
critical checks. Store per-check evidence and limitations; avoid one ambiguous
overall `valid` flag. Physical coupon results must be attributable to a particular
recipe and setup revision.

## Mill setups

### Base first

- Trial orientation: base upside down, Y rail faces toward the bed and underside
  cavities toward the open fill side.
- Locate Y rail and bearing inserts with independent datum bars; support other
  insert faces with explicit fixtures rather than assuming one common plane.
- Six EPS pocket cores retained by removable bridges. Use actual pocket sizes
  from the frame, preserving longitudinal beams, transverse rib and front bridge.
- Side-withdrawing rigid foot-access formers and separate stem formers; account
  for their intersection and steel seat bores.
- Reproduce shallow drive-seat and rail-shoulder recesses and column mounting
  pads. Inventory every feature ID from the foundation adapter.
- Define detachable walls, sealing lands, closure points and a handling/demolding
  sequence for the actual casting mass; do not rely on an unspecified manual flip.

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
- Per-panel SVG drawings at declared scale: outline, drill centers, pocket/rebate
  depths, machining face, labels and datum dimensions.
- CSV panel blanks, EPS layers/blocks, fixtures, hardware and reusable/consumable
  quantities; optional cost estimates only from supplied prices.
- Human-readable assembly, sealing, fixture release, cure prerequisites and
  demolding instructions tied to the ordered steps.
- Joint fit coupon drawings and a physical trial record template.

SVG profiles are fabrication references, not machine-ready G-code. Add DXF only
after confirming an existing suitable exporter or implementing a tested limited
exporter. Kerf/tool compensation belongs to a later machining process; never
apply it twice across profile generation and CAM.

Bridge panel bounding blanks into `SheetPartRequirement`, retaining finished
profile IDs and revisions. Keep detailed routing/drilling separate from guillotine
blank cutting. Do not claim automatic contour nesting or consume sheet inventory
as a side effect of generation. Preserve existing sheet codecs and execution
semantics. Integrate toolpath/CAM packages in a later phase after the design
exports and process requirements are stable.

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

Implement panel profiles/operations, rebates, external tab-and-slot joints and
pin/bolt positioning. Add CAD components with registered rebuild recipes and a
small generic panel fixture example. Generate the joint calibration coupon.

Acceptance: measured-thickness changes update both mating sides consistently;
router reliefs are manufacturable; reversed or trapped assembly cases are found;
CAD reconstruction preserves dimensions, IDs, materials and connector frames.
Panel logic has no casting-specific dependency.

### 3. Base mold review milestone

Implement the box-mold planner, feature classification, EPS blocks, detail
formers, core bridges and insert fixtures. Add `MillMoldSetup`, `MillMoldPreview`
and a dedicated `haxeon.tooling.json` entry point for base-only generation first.

Acceptance: all current base features and inserts have an explicit tooling
strategy; assembled/exploded views are inspectable; reconstructed casting matches
the source; core restraints and panel removal steps are represented. Unresolved
physical assumptions are visible. No fabrication-ready claim is made yet.

### 4. Assembly validation and fabrication package

Implement sequence validation, supported translation sweeps, seam coverage,
load-estimate reports and export manifests. Add per-part SVG/CSV, blank planning
bridge and instructions. Preserve recipe restoration throughout.

Acceptance: negative fixtures catch trapped walls, missing pockets, displaced
inserts, leaking slots and inadequate/pending restraint information. Profiles,
CAD and cut lists derive from the same records; repeated generation is stable.
Missing material/process evidence cannot become a passing fabrication check.

### 5. Column and independent structure

Add the column setup, stepped/flared envelope support, intersecting channels and
side-removal details. Add a small mineral-cast pedestal example with different
dimensions, insert layout and opening direction using the shared planner.

Acceptance: both selected mill foundations generate complete review packages;
the pedestal uses shared records/joints/planner with configuration only. Changing
stock thickness, casting size, opening direction and core strategy requires no
mill-specific conditionals in the libraries. Existing frame geometry is unchanged.

### 6. Physical calibration and release

Fabricate a joint coupon and a representative small mold with EPS core, steel
insert, covering, seam and restraint. Record actual dimensions, release behavior,
core deformation, cure temperature and compaction method. Adjust recipe values
from evidence. Review full mold loads, handling, insert anchorage and finishing.

Acceptance: evidence is attached to the relevant recipe revision; full mill
fabrication readiness is decided against its actual dimensions and loads. A small
successful coupon alone does not qualify a much larger casting. Manufacture full
molds only after the outstanding process and structural checks are resolved.

## Test and delivery strategy

Deliver each phase as a focused commit or small series with documentation and
meaningful checks. Do not mix unrelated machine or app work into these changes.

Pure tests: unit/frame conversion, deterministic codec, joint fit across sheet
thickness variation, sequence cycles, invalid/unsupported inputs and rectangular
blank conversion. Geometric tests: cavity equivalence, insert fixture placement,
rebate/slot material, continuous removal sweeps and recipe reconstruction. Include
negative geometries that would pass endpoint-only or bounding-box-only checks.

Add dedicated configurations:

```sh
./haxeon/scripts/haxeon run --project=manufacturingkit/tests/tooling/haxeon.json
./haxeon/scripts/haxeon run --project=machinekit/tests/tooling/haxeon.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.tooling.json
```

The first two configurations are implemented in phase 1. The mill tooling entry
point is planned for phase 3. Existing regression commands are:

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
assembly, hot-wire controller output, CNC posts, automatic hardware optimization,
full cure/flow/compaction simulation, full structural qualification and app editing
UI are follow-on work. Preserve extension points for them without implementing
placeholder subsystems. The initial release succeeds when the shared libraries
produce complete, reproducible and honestly validated mold kits for the two mill
castings and the independent pedestal.
