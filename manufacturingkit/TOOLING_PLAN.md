# Reusable panel tooling and mineral-casting molds

Status: phases 1–5 and the phase 6 process-analysis foundation are implemented as review tooling. The fabrication package reports unresolved process, structural, fixture and finishing requirements; physical qualification remains pending.

## Implementation progress

- [x] Phase 1: neutral input/setup/stock records, strict versioned codec, stable
  fingerprints, typed input review, and the MachineKit foundation adapter.
  ManufacturingKit retains its ProjectKit-only dependency. CAD equivalence checks
  cover box/chamfered/flared envelopes, rotated inserts and existing hobber layouts.
  The initial contract supports mm inputs, convex additive prisms and box cuts;
  unsupported units must be converted before entering the contract.
- [x] Phase 2: exact panel geometry, joint pairs, generic preview and fit coupon.
  Phase 1 closed vocabularies are enum abstracts; tooling codec version 2 rejects
  version 1. Original insert role labels remain source descriptions alongside the
  typed role. Each panel operation owns one exact line/arc contour and explicit
  operation kind, side, setup, depth and tool diameter. CAD and CadKit SVG/DXF
  drawings consume `PanelContours.operation`; ManufacturingKit emits no drawings.
  Rebuildable panel/pin recipes are registered. Straight swept volumes, including
  cylindrical face subdivisions at tangent generators, supply the blocking graph
  and its assembly/removal orders. Tests cover clear endpoints with an obstructed
  path, cyclic blocking, reversed keyed panels and open-bore clearance.
  `machinekit/examples/panel-tooling/` generates assembled/exploded scenes,
  STEP/SVG/DXF/recipe records and a numbered fit coupon. Reports explicitly cover
  the panels after loose closure hardware is removed. Physical fit and loads
  remain unqualified.
- [x] Phase 3: single-pocket box mold, end to end.
  `BoxMoldPlanner` consumes the adapter/setup/recipe document and emits five
  thickness-driven panels, a covered EPS block with ruled cutting definition,
  a bed-facing insert fixture and nine classified contact cuts. Outer wall
  tongues locate in blind bed sockets, behind continuous casting-contact lands.
  `BoxMoldCodec` version 2 verifies complete deterministic regeneration; panel
  codec version 2 requires named contact openings (unclassified mold cuts fail).
  Registered block recipes, nominal EPS/coated-board display materials and
  `machinekit/examples/box-mold/` provide assembled/exploded scenes, STEP,
  shared-operation SVG/DXF drawings, reconstruction and ordered procedures.
  Exact boundary coverage, sealing margins and assembly/demolding sweeps are
  checked. Tests cover both 18.0/17.8 mm stock and two setup orientations,
  source-CAD symmetric difference, persistence, bad holes/seals and trapped walls.
  The template explicitly supports one rectangular envelope, one open-top core
  and recipe-selected insert fixing hardware; unsupported geometry fails. Physical core restraint,
  independent insert-datum accuracy, thread/seal qualification, anchorage,
  finishing allowance, process loads and handling remain pending.
  Follow-up: core support selection now covers bed-open sockets and suspended
  carriers/bridges. The bed variant uses an inset EPS foot and explicit glue
  lines in a blind socket within the cavity footprint. The bridge variant uses
  registered top pads, round/slot locators and separate bolts; its permanently
  bonded carrier/bridge moves as one fixture. Both produce rebuildable CAD and
  cutting/drawing records, participate in full swept assembly/demolding graphs,
  and preserve source casting geometry. Forcing either support through a solid
  casting face fails. Geometry is provided; adhesive strength, EPS compression,
  pad clamps and uplift/bridge capacity remain pending. `--bed-core` generates
  a separate source variant under `build/review-bed/` for inspection.
- [x] Phase 4: mill base mold review assembly.
  `CastingMoldPlanner` classifies all 31 features and 12 inserts from the full
  `MillFramePreview` foundation instance. The single-pocket and full-base planners
  share `MoldPanels`. Exact orthogonal volume allocation splits overlapping
  details into 14 connected formers, with six laminated EPS cores on two shared
  bridges and twelve bed-registered nominal steel supports at their actual heights.
  `CastingMoldCodec` and `MoldVolumeCodec` persist rebuildable neutral records;
  `ToolingVolume`, `CastingMoldCad` and `CastingMoldExport` realize/export them.
  The dedicated `haxeon.tooling.json` emits review artifacts under
  `machinekit/examples/bench-mill/build/tooling/base/`. Actual foundation CAD
  equivalence, boundary coverage and separate continuous assembly/demolding
  graphs pass. Tests reject lost pockets, displaced datums, unclassified cuts
  and an inward-trapped foot former, and cover stock variation and optional XY
  interfaces. Process records include replication/grouting, compaction and
  per-insert anchorage. Source requests cover draft/split decisions, finish intent,
  machining/replication/grout allowances and lifting/flip provisions. Fixture
  attachments and insert XY clamping remain tooling validation work. Those interfaces,
  clamps, materials/process evidence, loads and handling remain unqualified;
  no source geometry is altered and fabrication readiness is false.
- [x] Review hardening of phases 1–4: explicit recipe hardware and fit allowances;
  required contact openings; neutral seal-margin checks and typed sealing faces,
  plugs, datums and precision routes; fill height/freeboard and selectable core
  cutting faces/direction; structured validation and separate readiness. Exact
  internal cutter-fit and rebate ligament checks apply without a relief flag.
  Joint records validate mating dimensions, not only frames. Shared bounded IDs,
  R12/NCName layer checks, SCC-only cycle diagnostics and sorted blocking edges
  are enforced. Native throw paths close owned resources. Regression fixtures
  include physical pins, sliding contact and member permutations. Codecs reject
  old encodings: tooling 3, box mold 3, casting mold 2, mold volume 2; panel 2
  remains the required-contact-opening format. Side former sealing profiles derive
  from the same panel operations so routed reliefs neither leak nor trap formers.
  Validation: 160 pure tooling, 336 native tooling, 653 drawing, 51 full-base,
  49 casting, 54 sheet-cutting and 215 mill-frame assertions pass.
- [x] Phase 4 review: explicit feature allow-list, draft and overlap priorities;
  unsupported features retain blocking records/requests. Finishing routes match
  source intent or request a revision, with allowances required for machining and
  replication. Unimplemented dam/pour fields are removed. Service orientation
  derives reorientation, with resin/bored-steel mass/CoM and typed lifting evidence.
  Nominal supports add round/slot locators, recipe fixings and sealed bed cuts;
  through-bed pins and bore masks join the exact graphs rather than an assumed release.
  CoreSupports owns shared bridge registration/hardware. Volume pieces are
  disjoint/canonically ordered, grout IDs unique, and all change requests reference
  real validation codes bridged into `machinekit.assembly.Diagnostics`.
  `machinekit/tests/tooling/haxeon.mill.json` registers the mill acceptance suite.
  Current formats: tooling 4, panel 3, box mold 4, casting mold 3, mold volume 3;
  older formats have no compatibility readers. Physical qualification stays pending.
  Validation: 183 pure tooling, 339 native tooling, 151 full-mill, 49 casting
  and 27 design-interface assertions pass. The box example and full mill review
  package regenerate; the complete hardware assembly restores exactly using one
  shared bed datum, within the assembly connector limit.
- [x] Phase 5: assembly validation and fabrication review package.
  `tooling.export.ToolingFabrication` adds versioned process/load inputs, typed
  material/temperature checks, peak-head pressure and gross core uplift, declared
  rectangular plate/brace/fastener capacity estimates, and per-core restraint
  checks. Missing data stays pending; cutout plates and localized support loads
  need qualified models. Styrene/EPS and assessed cure peaks at the EPS limit fail.
  Exact sequence/coverage/reconstruction checks remain the CAD gate.
  Neutral sheet requirements and deterministic shelf layouts retain part IDs and
  revisions without inventory mutation or contour-nesting claims. The package
  includes per-panel operation CSV, EPS/steel/hardware lists, reusable/consumable
  groups, fit and round/slot locator coupons, and a fingerprinted physical trial
  template. ManufacturingKit still imports only ProjectKit and writes no drawings.
  `CamToolingPanels` maps operations 1:1 onto CamJob profile/pocket/drill, uses
  declared tool/feed/setup/stock/fixtures/travel and exports checked LinuxCNC
  programs through the existing CncWriter. Underside reflection, pocket-access
  prerequisites and last outside cuts are explicit. Exact arcs are retained as
  optional CamContour boundary metadata: CamKit's exact native offset is needed
  because polygon offsets reject cutter-radius reliefs. Isolated matched-radius
  relief centres get disc-checked vertical entries within the same operation;
  output centrelines retain the declared chord tolerance. No compensation is
  applied to neutral profiles. This extension replaces the original polyline-only
  offset assumption for tooling; general arc-native IR is deferred.
  `haxeon.fabrication.json -- --checks` generates a fresh full package under
  `build/tooling/base-fabrication/`, with hashed manifest/verification and a
  normalized STEP timestamp for repeatable output. Shop feeds/workholding,
  non-panel machining, precision masters/dams/plugs, source change requests and
  physical qualification remain explicit blockers rather than fabricated data.
  Validation: 216 pure tooling, 105 panel-CAM, 12,456 existing CamKit, 339 native
  tooling and 151 full-mill assertions pass. The sheet bridge passes the existing
  sheet-plan validator. All 19 panel/coupon parts produce 25 checked programs;
  the assembly-derived hardware BOM contains 76 pieces. Two full generations
  have identical manifests and all 650 artifact hashes; tampering is rejected.
  Source/reconstructed CAD symmetric difference is zero.
- [ ] Phase 6: design-stage moldability feedback and casting process analysis.
  Process-analysis foundation implemented: independent strict `CastingProcessSpec`
  version 3 binds resin/thermal/fill/screening inputs to the exact recipe fingerprint.
  `FillAnalysis` computes exact orthogonal event-plane connectivity, limiting
  trapped volumes, source-owned ceilings and candidate vent points, ideal
  level/volume/mass/head/time curves and per-core submerged uplift. It uses the
  existing disjoint box algebra and symbolic right-hand limits, rather than
  sampled heights or native offsets. Independent CadKit Boolean fixtures verify
  the volumes and component topology; the full package cross-checks its resin
  volume against the fingerprint-bound source CAD review. Non-orthogonal fill
  geometry remains unsupported. Connected coplanar ceilings use a conservative
  bounding-width rule; an unknown air-escape limit stays pending.
  `CureAnalysis` adds an environment-aware adiabatic bound and coupled implicit
  1D conduction/first-order Arrhenius reaction for explicitly declared slabs.
  Insulated heating, nonlinear energy balance and inert-slab Fourier cooling
  fixtures pass. Residual reaction heat prevents early-stop qualification;
  missing data and actual-section numerical convergence/coverage/calibration
  remain blocking, and nonconvergent solves report unsupported.
  Process checks join the existing fabrication manifest and Diagnostics without
  clearing planner requests. The mill's supplier properties, temperatures,
  margin, ceiling-width limit and section definitions are intentionally unknown.
  Further phase 6 work adds `MoldOrientationStudy`: explicit candidate documents
  run the real planner, fabrication checks and source fill/cure analyses, with
  fingerprint-bound CAD assessments supplied by MachineKit. Deterministic ranking
  prioritizes completed planning/CAD checks, then failures, unsupported rules,
  source requests and pending checks. Effort reports production panels/unique
  geometry, production-only blank sheet utilization, EPS consumption, formers,
  logical hardware stacks and handling mass/flip; failed planning has no invented
  effort values. The mill compares inverted/upright/side-on setups, preserving
  feature withdrawal directions under rotation. Alternatives retain the current
  support policy: failure requests support redesign, not an inherent impossibility.
  `OrthogonalSections` extracts complete opposing-face material spans along all
  three axes, with exact volume coverage and source face ownership. Partition
  sizes are not wall thickness. A span becomes a `CureSection` only with explicit
  thermal boundaries; nonrectangular inscribed thickness now has the bounded
  exposed-face search described below.
  `MoldPlayback` emits static scenes from the exact checked assembly/demolding
  orders and expands grouped fixtures/hardware explicitly. It retains the cured
  casting at withdrawal clearance and shows the service-orientation endpoint;
  pending loose fixings, release and lift/flip qualifications are not cleared.
  Castability screening now reuses exact fill/ceiling and planner draft records,
  with typed source-owned review proposals, aggregate-based minimum passage and
  rectangular-solid maximum section checks. Recipe-bound process spec version 2
  requires explicit nullable screening limits/evidence and rejects version 1.
  Widening proposals carry measured/target separations, axes and representative
  regions; vent proposals retain exact ceiling points and require routing/sealing.
  Junctions require explicit thermal-model coverage, and EPS breakout
  stays pending physical access/release trials. The source is never edited and
  suggestions cannot clear checks. Each orientation includes the same screening;
  `castability-analysis.json` and `castability-review.md` join the hashed fabrication package.
  Remaining phase 6 work: broader radius/insert-cover/access rules, accepted
  source-parameter proposals with replan demonstrations, qualified 3D junction
  thermal coverage, and column orientation/design-study integration. No phase 6 completion or physical release is claimed.
  Process-foundation validation: 492 pure tooling assertions (276 process), 361 native tooling
  assertions (22 independent process/CAD) and 151 full-mill assertions pass.
  Two full generations have byte-identical manifests and all 654 artifact hashes,
  including the external `--process-spec` input path; 25 programs pass CAM checks. The current
  inverted base has no topologically trapped air; its 70,107,544 mm3 mix region
  agrees with CAD. Readiness remains false: 22 passing and 147 pending records,
  including undeclared broad-ceiling/cure inputs and the existing qualification
  and source-change requirements.
  Orientation/section/playback validation: 524 pure, 367 native and 239 full-mill
  assertions pass. The package contains 156 checked-order scene snapshots and
  815 hashed artifacts, with 25 checked CNC programs. Two final generations have
  byte-identical manifests and all 815 hashes, and the assembled/released scenes
  open successfully in Materia. Inverted is the preferred
  review setup under the existing support policy; upright/side-on fail the
  fill-open shared bridge requirement while still producing source fill/cure
  reports. All three have no topological air trap in this model. Planar spans
  reconstruct native BREP fixtures by symmetric difference on each axis. Static
  visual-only scenes retain pending release/flip states and do not substitute
  viewer collision boxes for the exact saved review. Actual final readiness is
  false: 23 passing, 147 pending and one unsupported junction-thickness rule.
  Castability screening validation: 571 pure tooling assertions (47 castability),
  28 independent native process/CAD and 239 full-mill assertions pass. The full
  package has 23 passing, 155 pending and one unsupported check; the 9 mm minimum
  opposing-face span cannot be approved against an undeclared aggregate size.
  The feedback contains 23 review actions: six EPS breakout trials, thirteen
  rigid-former release reviews, three process-limit declarations and junction
  analysis. No source change, automatic vent or physical release is claimed.
  Junction geometry and coverage follow-up: `JunctionThickness` computes largest
  Euclidean inscribed-sphere intervals from exact exposed rectangles, with
  source-owned faces, a contained witness and Lipschitz branch-and-bound upper
  bounds. Work exhaustion retains bounds and cannot claim convergence. The mill
  converges to 216.99–217.47 mm at 0.5 mm diameter tolerance; native checks verify
  the witness against actual foundation CAD and compare exposed surface area.
  `ThermalCoverage` accounts for explicit slab-to-span bindings and exact uncovered
  volume; stale geometry, wrong thickness/ownership, mixed axes and duplicate
  assignments fail. Process spec 3 requires span IDs and a geometry fingerprint
  (empty/null for an unbound example), rejects prior formats, and shares bindings
  across same-geometry plan/document report stamps. Section report 2, process and
  orientation reports 3 and castability report 2 carry the new results.
  Geometry coverage does not qualify junction heat flow. Nonrectangular junction
  modelling stays pending; a safe adiabatic tier can bypass missing cooling
  coverage but never invalid authored bindings. The current mill has no authored
  slabs, so all 70,107,544 mm3 is explicitly unbound. No material data, 3D thermal
  result, source change or physical release is invented.
  Validation: 616 pure tooling, 32 independent native process/CAD and 243 full-mill
  assertions pass. The 819-artifact package has 24 passing and 157 pending checks,
  with no failed/unsupported records; junction thermal modelling and the missing
  material/physical evidence remain pending, so fabrication readiness is false.
- [ ] Phase 7: mill column and independent pedestal example.
- [ ] Phase 8: physical calibration and release.

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
  `CamContour` retains polyline vertices; phase 5 adds exact boundary metadata
  for native cutter compensation of matched-radius reliefs.
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
- `machinekit/examples/panel-tooling/`: implemented phase 2 generic joinery preview
  and fit coupon, independent of casting and mill layouts.

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
| `ToolingRecipe` | ID/revision, panel/core stock, joint policy, process limits, fit allowances, seal/liner thickness, hardware choices, resin cure spec, mix properties |
| `PanelStockSpec` | Material reference, nominal and measured thickness, variation, coated face, stock dimensions, permitted rotations/direction |
| `CoreStockSpec` | EPS grade reference, available block sizes, density, declared compression/temperature limits, layer adhesive, evidence references |
| `CastingToolingInput` | Source ID/revision/hash, supported envelope, named void features, insert geometry/poses, precision datums |
| `CastingSetup` | Source-to-mold transform, gravity/fill direction, open face, parting choices, feature overrides, support and finishing datums, compaction, fill height, pour rate and pour points, initial/ambient temperature |
| `CompactionSpec` | Method (vibration table, external vibrators, internal poker, none), mold-bed interface, declared frequency/amplitude/duration or pending, evidence |
| `InsertAnchorage` | Insert reference, anchorage kind (studs, undercut, bonded keys), geometry, declared pull-out/shear capacity or pending, evidence |
| `PanelPlan` | Stable ID, exact 2D profile, stock/thickness, mold pose, contact face, joints, operations, label zone, removal motion |
| `PanelOperation` | Stable ID, kind (outside/inside profile, pocket, rebate, drill), exact line/arc contour or hole centres, machining face, depth, through/blind, setup |
| `JointPlan` | Paired member/edge references, joint kind, datum role, clearances, reliefs, fastening and assembly direction |
| `CorePlan` | Source feature IDs, actual cavity intersection, material, layers, mold pose, covering allowance, restraint, removal strategy, cutting definition |
| `InsertFixturePlan` | Insert references, independent locating datums, fixing hardware, insert-sealed penetrations, threaded-hole protection, withdrawal and finishing allowance |
| `PenetrationPlan` | Hole/slot reference, class (outside casting, insert-sealed, plugged, fill/vent opening), sealing face or plug, seal material |
| `DesignChangeRequest` | Source feature/insert IDs, requested change (draft, lifting insert, split, radius, access), reason, blocking checks |
| `ResinCureSpec` | Supplier/grade reference, solvent family, heat of reaction, kinetic model and parameters (or an adiabatic temperature-rise curve), resin mass fraction, mix density/specific heat/conductivity, gel and limit temperatures, evidence; each value or pending |
| `FillReport` | Setup/source fingerprints, critical levels, level-volume curve, resin mass, trapped-air pockets (volume, ceiling faces, source/tooling IDs, proposed vent point), head and buoyancy per level, fill time when a pour rate is declared |
| `CureReport` | Recipe/setup fingerprints, analysed sections (source IDs, thickness, boundary conditions), adiabatic bound, 1D peak temperature and time, margin to each limit, 3D result reference when run, validation evidence |
| `MoldPlan` | Panels, formers, cores, fixtures, braces, hardware, penetrations, steps, change requests, assumptions and source provenance |
| `ToolingValidation` | Rule ID, affected IDs, severity, evidence, and pass/fail/pending/unsupported status |
| `ToolingManifest` | Format version, units, revisions/hashes, generated files, validation status, quantities and replacement groups |

`PanelOperation` maps one-to-one onto a CamKit operation: outside/inside profile,
pocket (rebates are pockets with an open side) or drill. The kind is stored, not
inferred from depth or shape. Parametric cut shapes (relieved rectangle, circle,
obround) may be the stored form, provided each yields one exact line/arc contour
through a single function shared by CAD, drawings and CAM. Contours keep exact arcs
in the record; `CamToolingPanels` samples arcs to a declared chord tolerance
while `CamContour` retains a sampled representation and optional exact line/arc
boundary metadata for CamKit compensation (added in phase 5 for matched-radius
reliefs). Profiles are finished geometry: CamKit applies cutter compensation.

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
adhesive, and a cure peak temperature, from the cure analysis, at or above the
core stock's declared maximum temperature. The recipe therefore declares the solvent family and a
`ResinCureSpec`; peak temperatures come from the cure analysis below, and are
pending when its inputs are.

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

**Finishing route.** Each precision interface declares how it reaches its final
plane: `machined` (proud finishing stock, as before), `replicated`, or
`as-cast`. The bench mill's first build uses `replicated` for rail and
screw-bearing seats, and grouts the column joint (see
`machinekit/examples/bench-mill/README.md`, "Precision interfaces"). For
replicated seats, the insert fixture only needs to place the insert within
the replication allowance. The plan must not credit it with final alignment.
Record per seat:

- the allowance below the final plane, and how it was derived (compound
  minimum layer + fixture height error + cure distortion);
- the master (part, flatness evidence) and the release agent;
- the dam geometry that contains the compound;
- plugs or the drill-after rule for threaded holes through the layer;
- which seats share one pour.

Grouted joints record the gap, jacking/squaring method, bolt sleeves and cure
before torque. Replication masters, dams and plugs are tooling items in the
fabrication package. Phase 8 coupons cover replication thickness, flatness
transfer and bond to the steel seat.

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

## Design-stage moldability feedback

The planner and its checks also inform the casting design, so problems are
fixed in the design rather than worked around in the mold. Feedback never edits
the casting: it reports against source feature and insert IDs, and the machine
design accepts a change explicitly, producing a new casting revision whose
fingerprint triggers re-planning.

Four levels, cheapest first:

1. **Castability rules.** Pure checks over `CastingToolingInput`, a setup and a
   recipe, with no mold plan, fast enough to run on every design rebuild:
   - draft or a split on walls of features assigned to rigid formers, and a
     straight removal direction for every non-EPS feature in the setup;
   - minimum wall/rib thickness against the declared maximum aggregate size;
   - maximum section thickness against the cure analysis's peak temperature for
     that section and the core stock and resin limits;
   - air traps found by the fill analysis need a vent or another orientation;
     flat ceilings wider than the recipe's air-escape width are flagged even when
     connected, since a vibrated paste does not drain air sideways reliably;
   - internal corner radii at least half the router diameter where formers or
     panels follow them, and at least the recipe's flow radius elsewhere;
   - insert cover depth, edge spacing and through-bed fixing access;
   - lifting points for every handling step, from casting mass and centre of mass.
   Thresholds come from the recipe, never from constants in the rules.
2. **Moldability report.** Run the real planner and its validation on the
   current design with a declared default setup and recipe, and report its
   `ToolingValidation` records and `DesignChangeRequest`s. This finds what the
   fast rules cannot: unsupported cores, trapped walls in the blocking graph,
   unsealable inserts. It reuses the mold checks, so there is no second rule set.
3. **Proposed changes.** A `DesignChangeRequest` may carry a concrete proposal:
   a parameter ID with a new value (draft angle, corner radius, pocket depth
   snapped to EPS layer thickness, wall height snapped to the sheet module) or a
   new feature (lifting insert positions symmetric about the centre of mass).
   The proposal is data; the machine design decides whether to apply it.
4. **Effort metrics.** The planner reports panel count and unique panels, sheet
   yield, rigid formers versus EPS cores, sacrificial EPS volume per casting,
   assembly steps, hardware count, resin mass and open change requests. Design
   studies compare variants on these beside stiffness. A setup comparison
   evaluates an explicit list of candidate orientations (axis-aligned, rule-based)
   and ranks them by failures and change requests; it does not search parting
   surfaces.

Rules, reports, proposals and metrics live in ManufacturingKit. MachineKit maps
them into the `Diagnostics` used by `FoundationIntegrity.check`, keyed by source
feature IDs so a later editor can highlight them; this release adds no app UI.
`MillDesignStudy` writes the metrics alongside its existing outputs.

Recipe limits start as declared assumptions. When phase 8 evidence revises a
recipe (aggregate size, achievable insert tolerance, minimum ligament, cure peak
by section), the design rules use the revised values; feedback reports which
recipe revision and evidence each threshold came from.

## Casting process analysis

Two analyses answer questions the mold and the design actually face: where air
is trapped while filling, and how hot the casting gets while curing. Both feed
the validation records and the castability rules; neither claims to predict mix
flow, compaction or cure distortion.

### Fill analysis

Model filling as a rising flat free surface in the setup's fill orientation; the
mix is not simulated as a fluid. The resin region is the reconstructed casting
solid in mold coordinates, already checked against the source.

- For a level h, the air region is the resin region above h. A connected
  component of it that does not reach the open fill face is a trapped pocket,
  first sealed at that level. Its volume, ceiling faces, the tooling or source
  feature IDs owning those faces and its highest point (the proposed vent) are
  reported.
- Component topology changes only at critical heights: the z values of
  vertices, horizontal faces and curved-face extrema. For the implemented
  orthogonal mold contract, exact disjoint-box algebra evaluates the symbolic
  right-hand limit at each critical plane: ending cells disappear and surviving
  cells retain their limiting volume. CadKit Boolean/component fixtures verify
  this calculation independently. This replaces the initial native-only proposal
  for orthogonal fill, avoids epsilon-height volume loss and needs no sampling.
  General curved/sloped fill regions require a future CadKit event/extrema path;
  they must not be approximated by bounding boxes.
- Report the level-volume curve, resin mass, liquid head on each panel and the
  submerged volume, and so buoyancy, of each core at each level. With a declared
  pour rate, add a time axis. The load checks use the worst level, not only the
  full one: cores reach full uplift once submerged, and panels see full head at
  the end of the fill.
- A vent becomes a `PenetrationPlan` of class fill/vent through tooling, or a
  `DesignChangeRequest` when it has to pass through the casting design.

The flat free surface is an idealisation: a vibrated paste traps at least these
pockets, and may trap more under broad flat ceilings. The air-escape-width rule
covers that case, and phase 8 compares predicted pockets with demolded voids.

### Cure temperature

The exotherm matters because EPS softens near its declared limit, resin systems
have gel and limit temperatures, and temperature differences move inserts.
Three tiers, each run only when the previous one cannot pass:

1. **Adiabatic bound.** Peak rise is at most w·H/c, from the resin mass fraction,
   heat of reaction and mix specific heat, for any geometry. If the maximum initial/ambient/boundary
   temperature plus the remaining-reaction bound is below every limit by the recipe margin, the
   check passes without kinetics.
2. **Section model.** Transient 1D conduction through each section, with heat
   generation from the `ResinCureSpec` kinetics and boundary conditions from the
   mold: panel (board conductivity, thickness and convection), EPS (near
   insulating), open face (convection) and steel inserts (heat sinks). Section
   thicknesses come from the casting: distances between opposing planar faces for
   walls and ribs, and the largest inscribed thickness at junctions. Other shapes
   are unsupported until a medial-axis method exists. Implicit time stepping in
   ManufacturingKit, verified against the adiabatic bound and the analytical
   inert-slab cooling series before use. The 1D result is conservative for wall
   interiors, not for junctions.
3. **3D transient.** Only when a junction or thick block is within the recipe
   margin after tier 2. Use the existing Gmsh/CalculiX pipeline and its
   solver-fixture gate. Cure heat generation uses either a CalculiX DFLUX user
   routine or a staggered external kinetics update, whichever passes the
   adiabatic, 1D-slab and mesh-convergence fixtures; record the choice. Move the
   shared deck writer out of the bench-mill example when this second consumer
   needs it.

Missing kinetics or mix properties leave tiers 2 and 3 pending; the adiabatic
bound alone can still pass a check. Phase 8 thermocouples (thickest section
centre, core surface, a panel face) validate peak temperature and time to peak
within a declared tolerance, attached to the recipe revision.

### Sequence playback

The review package adds one scene per assembly and demolding step, taken from the
validated orders, so trapped parts and awkward steps can be seen as well as read.
This is presentation of existing results, not simulation.

### Not modelled

Flow simulation of the mix, particle simulation of compaction, cure shrinkage and
residual-stress distortion, and demolding adhesion are not modelled. The mix is a
vibrated, highly filled paste outside the validity of fluid filling models;
compaction and adhesion need physical evidence; precision seats are replicated
after cure, which absorbs most distortion. Mold structural FEM is used only for
braced panels or large cutouts the closed-form checks cannot represent, and
vibration pressure still comes from the compaction spec or measurement.

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

### 6. Design-stage moldability feedback and process analysis

Implement the castability rules, the moldability report, proposals on
`DesignChangeRequest`, effort metrics and the candidate-orientation comparison
described under design-stage feedback. Add the fill analysis, the adiabatic and
1D cure tiers with `ResinCureSpec`, `FillReport` and `CureReport`, and sequence
playback scenes. Expose them as foundation diagnostics and in `MillDesignStudy`.
The 3D cure tier is built only if a mill section fails tier 2's margin.

Acceptance: each castability rule has a passing and a failing fixture with the
offending feature ID reported; the moldability report for the mill base equals
the phase 4/5 validation for the same setup and recipe; accepting a proposed
lifting-insert or draft change yields a new casting revision that re-plans and
clears the request; metrics are deterministic and change when a pocket is added;
the orientation comparison ranks the base's inverted setup against at least two
alternatives with reasons. No rule reads a threshold that is not in the recipe.
Fill analysis finds the pocket in a fixture with a downward-facing ceiling, its
volume matching the analytic value, and none in its vented variant; it is
unchanged when evaluated at extra non-critical levels; core uplift peaks at
submersion. The 1D cure model matches the adiabatic bound with insulated faces
and the analytical inert-slab series with no reaction; missing kinetics yields
pending, never pass. The base and column orientation comparison includes trapped
air and cure margin.

### 7. Column and independent structure

Add the column setup, stepped/flared envelope support, intersecting channels and
side-removal details. Add a small mineral-cast pedestal example with different
dimensions, insert layout and opening direction using the shared planner.

Acceptance: both selected mill foundations generate complete review packages;
the pedestal uses shared records/joints/planner with configuration only. Changing
stock thickness, casting size, opening direction and core strategy requires no
mill-specific conditionals in the libraries. Existing frame geometry is unchanged.

### 8. Physical calibration and release

Fabricate a joint coupon and a representative small mold with EPS core, steel
insert, covering, seam and restraint, using the phase 3 box where practical.
Record actual dimensions, release behavior, core deformation, thermocouple
traces (thickest section centre, core surface, a panel face), voids found at
demold against the fill analysis's predicted pockets, compaction method and its
measured settings, and insert pull-out if tested. Fit or confirm `ResinCureSpec`
values from the traces and supplier data. Adjust recipe values from evidence.
Review full mold loads, handling, insert anchorage and finishing.

Acceptance: evidence is attached to the relevant recipe revision; full mill
fabrication readiness is decided against its actual dimensions and loads. A small
successful coupon alone does not qualify a much larger casting. Manufacture full
molds only after the outstanding process and structural checks are resolved.

## Test and delivery strategy

Deliver each phase as a focused commit or small series with documentation and
meaningful checks. Do not mix unrelated machine or app work into these changes.

Pure tests: unit/frame conversion, deterministic codec, joint fit across sheet
thickness variation, sequence cycles, invalid/unsupported inputs and rectangular
blank conversion, penetration classes, the hard resin/EPS rules, and the cure
tiers against the adiabatic and inert-slab fixtures. Geometric
tests: cavity equivalence, insert fixture placement, rebate/slot material, swept
volumes and the blocking graph, fill-analysis pockets at critical levels, and
recipe reconstruction. Include negative
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
point is implemented in phase 4; add `-- --checks` to run its focused acceptance cases. `CamToolingPanels` tests join the existing CamKit
suite in phase 5. Existing regression commands are:

```sh
./haxeon/scripts/haxeon run --project=manufacturingkit/tests/haxeon.json
./haxeon/scripts/haxeon run --project=machinekit/tests/casting/haxeon.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.frame-tests.json
```

Phase 2 also adds a generic preview and a focused regression for CadKit's existing
drawing writers and their named process layers:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/panel-tooling/haxeon.json
./haxeon/scripts/haxeon run --project=cadkit/haxe/tests/haxeon-drawing.json
```

Run the existing bench-mill configurations sequentially because some share build
outputs. Use the current Haxeon/native dependency setup; do not introduce a new
build mechanism. Visually inspect the assembled/exploded artifacts as well as
running automated checks. Keep generated full-size geometry under ignored build
directories; commit source setups, small test fixtures and documentation.

## Deferred scope

General BREP parting-surface discovery, arbitrary contour nesting, robotic mold
assembly, wiring core cutting definitions into `FoamJob` (after foam-cutter lands
on main), CNC posts beyond CamKit's LinuxCNC writer,
general arc-native CAM IR, rotational removal paths, automatic hardware optimization, automatic application
of design-change proposals, parting-surface search beyond listed orientations,
the 3D cure tier unless a section needs it, the items under "Not modelled", mold
FEM beyond what closed-form checks cannot cover, full structural qualification and app
editing UI are follow-on work. Preserve extension points for them without implementing
placeholder subsystems. The initial release succeeds when the shared libraries
produce complete, reproducible and honestly validated mold kits for the two mill
castings and the independent pedestal.


Phase 3 review entry point (independent of the mill):

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/box-mold/haxeon.json
python3 machinekit/examples/box-mold/render_review.py
```

Its generated review package stays under `machinekit/examples/box-mold/build/review/`.
The optional renderer consumes exported CAD triangles; NumPy and Matplotlib are
only preview dependencies. The neutral planner and codec retain the ProjectKit-only
package boundary. Assembly installs the bed first, using its downward approach;
demolding allows bed withdrawal downward after fixing release and EPS breakup.
These are separate swept blocking graphs, not an assumed reversal of one physical
process. Exploded offsets are for visual inspection and do not establish removal.


Phase 4 full-base review and acceptance entry point:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.tooling.json -- --checks
python3 machinekit/examples/bench-mill/render_tooling.py
```

See `machinekit/examples/bench-mill/TOOLING_README.md` for supported geometry,
artifact frames, protected insert envelopes and unresolved physical interfaces.
The generator uses a separate build directory from frame/FEM exports. Installation
checks approach bed-facing tooling from above before casting; demolding checks
withdraw it below the released bed after cure. Prebonded EPS/carrier/bridge rows
are installed as groups, then EPS and reusable fixtures separate after bond release.
The directional blocking graphs come from exact sweeps, not reversing one assumed
physical process. Exact AABB rejection only removes pairs that cannot have a
positive-volume intersection; the remaining pairs use native booleans.


Phase 5 fabrication review and checks:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.fabrication.json -- --checks
./haxeon/scripts/haxeon run --project=camkit/tests/haxeon.json
```

The full package includes the existing CAD/sequence checks, declared load/process
reports, per-panel checked LinuxCNC, lists, layouts and physical coupon templates.
`-- --records-only --checks` writes an independent records/CAM package with CAD
validation pending. Neither entry manufactures the coupon or releases the mold.
