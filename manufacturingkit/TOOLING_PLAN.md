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
  need qualified models. Styrene/EPS incompatibility fails; cure qualification now comes only from CureReport (review 23).
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
  `CureAnalysis` adds a lumped estimate and coupled implicit 1D conduction/reaction
  for declared slabs; review 24/29 adds autocatalytic kinetics and completion gates.
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
  uses planning/strategy/CAD feasibility gates, then the declared trapped-volume,
  worst-cure-margin, flip, panel-count and sheet-count tuple (review 26). Effort reports production panels/unique
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
  modelling stays pending. A lumped thermal estimate never bypasses missing
  coverage, junction modelling or invalid authored bindings (review 24). The current mill has no authored
  slabs, so all 70,107,544 mm3 is explicitly unbound. No material data, 3D thermal
  result, source change or physical release is invented.
  Validation: 616 pure tooling, 32 independent native process/CAD and 243 full-mill
  assertions pass. The 819-artifact package has 24 passing and 157 pending checks,
  with no failed/unsupported records; junction thermal modelling and the missing
  material/physical evidence remain pending, so fabrication readiness is false.
- [ ] Phase 7: foundation design model: typed casting features with openings and
  owners, insert holes from `DesignInterfaces`, design-owned interface
  requirements, datums, lifting inserts and foundation joints, then drafted,
  radiused and profiled features. Precedes the column.
  Compilation recovery checkpoint (step 1 remains in progress): source features,
  insert roles/finishes and hole patterns now use neutral `materia.casting` records;
  old JSON boxes and boreDiameter recipes are rejected. Tooling schema 6 and
  box/casting-mold schema 5 have no prior-format path. The shared box algebra is
  used by fill, collision and section screening; planar cut/body intersection is
  centralized and non-box cavities are rejected by feature ID by box consumers.
  The adapter copies source facts and `DesignInterfaces` holes. The old hobber
  receives only required type/import and recipe-key changes.
  The remaining step-1 rules, core-fragment grouping, hole-pattern fixture
  expansion and unified levelling-foot family are not claimed complete; steps 2
  and 3 remain pending. Current fixtures still declare explicit setup strategies.
  Validation: 1,712 neutral tooling/process assertions, 17 independent foundation
  record/algebra assertions, 49 casting assertions and 375 native tooling
  assertions pass. Full mill review passes 248 assertions and both package modes
  pass manifest tamper checks (40 programs, 837 full-package artifacts). Frozen
  pre-migration mill CAD and the sloped-foot cut/body intersection pass four
  independent native assertions. Readiness remains false: 14 Pass, 250 Pending, 15 Unsupported;
  no physical coupon trial is recorded.
- [ ] Phase 8: mill column and independent pedestal example.
- [ ] Phase 9: physical calibration and release.

See [README.md](README.md#casting-tooling-inputs) for the implemented API and checks.

## Manufacturing review follow-up

- [x] Review 1–9, panel CAM: measured-length multi-tool programs (G43/G49),
  dependency-preserving grouping, explicit single-tool review fallback, declared
  spoilboard/breakthrough, qualified-vacuum retention route, Z-only start, spindle
  dwell, open-side square-rebate extension, arc/blend tolerance, cut direction,
  nominal/measured diameter matching and machine-coordinate qualification gates.
  Panel tests include outside/relief compensation, full contour mirror, square
  rebate cutter coverage, tool offset sequencing, direction and CAM-depth rejection.
  The 276 panel assertions and 12,457 existing CamKit assertions pass. The records export yields 44 review programs, with five ordered bed files;
  all 19 per-panel CAM records stay pending under the study setup. The mill
  remains not fabrication-ready: tools/fixtures/vacuum and a recorded coupon trial
  are not qualified. Tabs/onion skins are not claimed by the vacuum route.
- [x] Review 10–13: gross displaced-volume uplift and X/Y restraint bounds
  for every rigid former and datum, with separate evidenced capacities. Real
  machined/support panels explicitly report an Unsupported structural model.
  Manifest v2 hashes readiness/validation and verifies recomputed readiness;
  unknown hardware is included in the BOM and blocks qualification.
  The neutral suites pass 630 assertions, including lateral restraint failure;
  the records export passes artifact and manifest metadata tamper checks.
- [x] Review 14–21: vertical-only flip detection; maximum-depth row bridges
  with a real minimum (mill study minimum explicitly 40 mm, not qualified);
  unapplied finishing allowances remain change requests, replication requires
  master/release data, equal-priority former overlaps reject. Release requests
  include bed backing and explicitly exclude unimplemented draft/split routes.
  CastInsert roles are choices with separate labels; foundations expose recess
  links and finish routes follow source intent. Neutral validation rejects datum
  intrusion into resin; datum profiles remain explicitly unsupported. Removed
  the codec-sorted permutation test. Neutral and native tooling suites pass;
  the mill records export passes manifest integrity checks.
- [x] Review 22–30: subtract tooling projections from fill openings and reject
  zero open area; curved openings needing an exact projection remain Unsupported.
  Trapped air is removed from the level curve and each level reports panel heads.
  CureReport is the sole cure authority (fabrication spec 3 / process spec 4),
  including EPS-containing adhesive, resin identity and gel limits. Autocatalytic
  kinetics, remaining-heat completion and energy accounting are implemented;
  first-order kinetics are analytical fixtures and cannot qualify a reactive mix.
  The lumped estimate requires margin and never clears coverage/junction rules.
  Planned orientation candidates call CastingProcessAnalysis directly, emit the
  declared metric tuple and deciding reason, and retain actual review validation.
  Full export compares symmetric difference against foundation.body.geometry().
  Validation: 1,675 neutral assertions, 34 native process assertions and 243 mill
  assertions pass. The 840-artifact full export has zero foundation symmetric
  difference, 13 Pass / 175 Pending / 15 Unsupported, and passes manifest tamper
  checks. There is no recorded coupon trial and the mold is not fabrication-ready.

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
| `CastingToolingInput` | Source ID/revision/hash, supported envelope, typed `materia.casting` features, inserts with holes and interface requirements, datum scheme, foundation joints (phase 7) |
| `CastingSetup` | Source-to-mold transform, gravity/fill direction, open face, parting choices, feature overrides, support and finishing datums, compaction, fill height, pour rate and pour points, initial/ambient temperature |
| `CompactionSpec` | Method (vibration table, external vibrators, internal poker, none), mold-bed interface, declared frequency/amplitude/duration or pending, evidence |
| `InsertAnchorage` | Insert reference, anchorage kind (bolted anchors, welded studs, undercut, bonded keys), anchor holes and hardware, embedment, bearing area, spacing and cover, declared pull-out/shear capacity or pending, evidence |
| `InsertManufacturing` | Insert reference, steel grade and blank stock, pre-cast operations (cut, square, drill and tap, anchors, deburr, surface preparation), post-cast operations with their stock, inspection points before casting, thread protection, procurement `Make(process)` |
| `PanelPlan` | Stable ID, exact 2D profile, stock/thickness, mold pose, contact face, joints, operations, label zone, removal motion |
| `PanelOperation` | Stable ID, kind (outside/inside profile, pocket, rebate, drill), exact line/arc contour or hole centres, machining face, depth, through/blind, setup |
| `JointPlan` | Paired member/edge references, joint kind, datum role, clearances, reliefs, fastening and assembly direction |
| `CorePlan` | Source feature IDs, actual cavity intersection, material, layers, mold pose, covering allowance, restraint, removal strategy, cutting definition |
| `InsertFixturePlan` | Insert references, independent locating datums, fixing hardware, insert-sealed penetrations, threaded-hole protection, withdrawal and finishing allowance |
| `PenetrationPlan` | Hole/slot reference, class (outside casting, insert-sealed, plugged, fill/vent opening), sealing face or plug, seal material |
| `DesignChangeRequest` | Source feature/insert IDs, requested change (draft, lifting insert, split, radius, access), reason, blocking checks |
| `ResinCureSpec` | Supplier/grade reference, solvent family, heat of reaction, kinetic model and parameters (or an adiabatic temperature-rise curve), resin mass fraction, mix density/specific heat/conductivity, gel and limit temperatures, evidence; each value or pending |
| `FillReport` | Setup/source fingerprints, critical levels, level-volume curve, resin mass, trapped-air pockets (volume, ceiling faces, source/tooling IDs, proposed vent point), head and buoyancy per level, fill time when a pour rate is declared |
| `CureReport` | Recipe/setup fingerprints, analysed sections (source IDs, thickness, boundary conditions), lumped estimate (not a spatial upper bound), 1D peak temperature and time, margin to each limit, 3D result reference when run, validation evidence |
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
Until phase 7, clip design-cut tools to the casting envelope: existing cuts
deliberately extend beyond casting faces, and those extensions are not literal
finished cavity sizes. Phase 7 replaces this clipping with declared openings and
one shared finished-cavity function.

Store plan/setup records in a versioned tooling JSON format separate from sheet
inventory. Validate codec input, bounded record counts and referential integrity.
Use deterministic serialization and source/recipe fingerprints. Geometry or
recipe changes invalidate generated reports and physical qualification tied to
the previous version. Do not serialize native CAD resources.

## Foundation design model

The mold pipeline is sound, but the casting description it starts from is too
thin, and several tooling workarounds come from that: a feature-ID strategy table
in the mill setup, clipping of deliberately oversize design cuts, hardware written
into insert labels, precision requirements held in mold options, and box-only
geometry that blocks drafted formers, corner-radius rules and non-orthogonal fill
analysis. Phase 7 replaces the source model before the column and pedestal are
built on it. Each step is one schema change with no compatibility path: casting
recipes, tooling inputs, codecs and fingerprints move together, and generated
packages are regenerated.

### Ownership

- `materia.casting`, a new ManufacturingKit package with no CadKit or MachineKit
  imports, holds the neutral casting design records and closed vocabularies that
  the design and the tooling share: feature kinds, openings, feature geometry,
  insert role and finish, interface requirements, datums and foundation joints.
  `InsertRole` and `InsertFinish` move here from `materia.tooling.ToolingEnums`.
- `machinekit.structural` (`MineralCasting`, `MineralFoundation`, `CastInsert`)
  stores these records as recipe parameters and realizes them as CAD. It defines
  no second feature type.
- `machinekit.design.DesignInterfaces` stays the hole and datum model for steel
  inserts; it uses CadKit thread specifications, so it remains in MachineKit.
- `FoundationToolingAdapter` becomes a copy: neutral records, plus hole interfaces
  converted to neutral hole records, fingerprinted. It infers nothing.
- One exact disjoint-box algebra lives in `materia.casting`. It replaces both
  ManufacturingKit's `BoxVolumes` (fill analysis) and the hobber's
  `machinekit.structural.CastingCells` (collision hulls, section stiffness), so
  collision, stiffness screening and fill analysis share one implementation.

This is the single casting tooling plan. The hobber's
`machinekit/CASTING_TOOLING_PLAN.md` (CT0–CT10, unimplemented) is superseded and
is deleted in phase 7 step 1; its
replication-relationship, fixture-constraint, batch and printed-piece decisions
are folded in here (interface requirements, insert fixtures, fill analysis and
deferred scope).

### Typed casting features

`CastingFeature` replaces `CastingFeatures.CastingBox`, which is an ID, size and
pose stored in a JSON text parameter.

| Field | Meaning |
| --- | --- |
| `id` | Stable source ID |
| `kind` | `Lightening`, `ServiceOpening`, `InterfaceRecess`, `InsertRecess` or `Clearance` |
| `geometry` | Design cut: box, or straight prism from a closed line/arc profile; it may cross the body boundary only through declared openings |
| `openings` | Body faces, or other feature IDs, the feature opens through; empty means enclosed |
| `draft`, `cornerRadius` | Wall draft along a declared withdrawal axis and internal corner radius; zero is explicit |
| `owner` | The interface or insert the feature serves, or null |
| `adjustable` | Dimensions the design allows proposals to change (lightening cores yes, interface recesses no) |
| `derivedFrom` | The design feature this one was split from when retained webs, backing or bosses are subtracted from it, or null |

- Retained material (support webs, backing under precision seats, foot bosses) is
  subtracted from a design feature by the shared box algebra. Each resulting
  fragment keeps a stable ID and `derivedFrom`, so tooling can group the
  fragments of one bay into one core unit and castability can report against the
  design feature that the designer edits.

- A feature stores design intent: its cut and the body faces or features it
  opens through. The finished cavity (cut ∩ body) is derived by one function in
  `materia.casting`, exact for the planar envelopes this model supports, and
  every consumer uses it; no consumer clips cuts itself. Where a cut crosses a
  non-orthogonal body face, such as the column's sloped foot, the finished
  cavity is a line-profile prism even though the cut is a box. A cut may extend
  beyond the body only across declared openings, by no fixed margin; a declared
  opening the cut does not reach, or a breakout through an undeclared face, is
  an error. CAD realization still extends tools internally for robust booleans.
  Openings are declared by the source; computing reached faces is a diagnostic,
  never how a feature gets its openings. Body faces have stable IDs per envelope
  prism and face, so coplanar-normal faces (the column's foot side and body side)
  never share an ID.
- Consumers that support only boxes report a non-box finished cavity as
  Unsupported with its feature ID until step 3, never approximating it.
- `MineralFoundation` generates insert recesses from its inserts with
  `kind = InsertRecess` and the insert as `owner`; recess links derive from that.
- Default mold strategies follow from kind and openings through recipe rules
  (for example, a lightening pocket open to the fill face becomes an EPS core and
  an interface recess a rigid former). A setup overrides a strategy by naming the
  feature ID. Unknown kinds fail. The mill's ID-keyed strategy table is deleted.
- Castability proposals change only `adjustable` dimensions; other features get
  change requests.

### Feature geometry beyond boxes

- Draft and corner radius are realized in CAD, in the planner (formers get drafted
  walls; EPS cores ignore draft unless declared), in drawings and in CAM, where a
  corner radius below the router radius fails with the feature ID.
- Prism features from line/arc profiles cover chamfered and stepped recesses.
  Other shapes stay unsupported.
- Box algebra remains the exact fast path for features that are axis-aligned boxes
  in the setup. Other regions use CadKit booleans at the same critical levels, as
  the fill analysis originally specified. `OrthogonalSections` and
  `JunctionThickness` report non-box regions as unsupported until they gain an
  exact method. Drafted rigid formers become supported, and their release request
  clears through draft rather than only through EPS or a source change.

### Steel inserts

- Insert holes come from a `DesignInterfaces` facet on `CastInsert`: patterns of
  `HoleInterface` with thread via `ThreadSpec`, tap drill, blind or through and
  counterbore. They replace `boreDiameter` and hardware text in labels such as
  "nominal M12 tap drill". Bore masks, plugs, insert-sealed penetrations and
  fixture fixings come from these holes.
- Hole limits (minimum engagement, tip gap, back wall, wall and clearance) are
  design rules, not evidence, so they are always numbers. A named
  `ThreadedHolePolicy` in `machinekit.design` resolves them from the thread
  specification and the insert material (for example, engagement as a multiple
  of nominal diameter for steel). A hole may override a value explicitly. The
  neutral hole record stores the resolved numbers and the policy ID, both in the
  fingerprint. No placeholder or unknown limit enters `DesignInterfaces`. Hand-set
  limits on existing parts (`MillYNutMount`, `MillCrossSaddle`) move to the policy
  when those parts are next touched.
- Anchorage is insert geometry and feeds `InsertAnchorage`; its capacity stays
  pending evidence until a pull-out coupon is recorded. The default kind is
  **bolted anchors**: blind tapped holes in the insert's back face, from
  `InsertHoles` and the `ThreadedHolePolicy`, with standard bolts (or threaded rod
  with nut) and washers screwed in before the pour. The heads and washers are the
  bearing anchors, and no welding heat reaches the precision faces. The fasteners
  are real standard components placed in the insert, so they appear in the BOM,
  in overlap checks against cores, formers and fixtures, and in the fill analysis
  (air under heads and washers). Design rules check back wall to the mounting
  face, clearance to the front mounting holes, embedment, head bearing area,
  spacing and cover to the casting surface. Each bolt is torqued to a shoulder or
  set with retaining threadlocker so the thread has no play. Welded studs,
  undercuts and bonded keys remain other kinds; none is assumed.
- Each insert is a made part with an `InsertManufacturing` record derived from the
  insert, its holes, anchorage and interface requirement. It holds the steel
  grade and blank, the pre-cast operations (cut and square, drill and tap, fit
  anchors, deburr, degrease and bond-ready surface preparation), and the post-cast
  operations. An insert finished after casting is cast oversize by its finishing
  stock, so the stock is in the insert geometry as well as the casting allowance.
  Holes whose positions relative to other inserts are tighter than the insert
  placement tolerance are drilled after casting, as the interface requirement
  decides. Flatness and hole positions are inspected before the insert enters the
  mold. The package gets insert drawings through `cadkit.drawing`, an operation
  sheet, and `Make(process)` BOM lines in the `BOM_PLAN.md` vocabulary.
- Holes that bolt a bought part to an insert (rail screws, bearing housings)
  derive from that part's mount connectors, never from hand-entered positions;
  `machinekit/RAIL_MOUNTING.md` RM-D9 and RM-D10 apply this to rails. Each such
  hole's route is chosen by a clearance budget: the mating part's bolt clearance
  against the insert's placement tolerance, the adjustment the part needs at
  assembly, cure distortion and machining tolerance. When the budget closes the
  hole is drilled and tapped before casting and masked during the pour; when it
  does not, the route is transfer drilling through the aligned part after
  casting, with its guide bushing, tap guide and swarf protection on the
  operation sheet; when no permitted route closes, a design change request. Insert
  drawings dimension these holes from the insert datums its fixture locates on.
- `finish` becomes typed tokens; `label` is display text only.

### Interface requirements

Precision intent belongs to the design. Each precision insert or foundation
interface carries an `InterfaceRequirement`: route (as cast, machined, ground,
replicated), allowance (machining stock or replication/grout gap), the finishes
the interface permits, and position tolerance. The casting geometry includes the
allowance. A route that the geometry cannot achieve is a design change request.

Accuracy requirements are **relationships**, not single faces, and they come from
the machine design rather than the casting:

- `InterfaceRelationship`: coplanar, parallel, perpendicular or a distance between
  datums, with a tolerance. Members are interface IDs or **machine axes** (for
  example, the hobber column's Z-rail seats parallel to the C axis and its table
  seat square to it). The machine assembly declares them; each insert's
  requirement references the relationships it takes part in.
- A replicated relationship is produced by a **replication jig**: release-coated
  masters (ground bars, plates, granite squares, or an axis master such as a
  ground arbor on the C-axis line) held in the design relationship on one rigid
  reference frame, each on three-point jack supports with locks. Simpler finishes
  stay available where the interface allows them: the real rail as master,
  self-levelling (one gravity plane only) or face-down replication on a surface
  plate. The jig planner chooses only among permitted finishes.
- Each relationship names its metrology: instrument and method (electronic level,
  indicators against the axis master, autocollimator or laser) and the stated
  uncertainty. The measurement chain must fit inside the tolerance, or the
  relationship stays pending.
- The mold options' `PrecisionInterface` keeps only process choices: master and
  jig references, release, hole protection and evidence. Replication masters,
  jigs and their metrology are fabrication-package items; nothing is shimmed
  later.
- *Amendment 2026-10-20 (`machinekit/RAIL_MOUNTING.md`).* Rail relationships
  also have **in-plane** kinds: straightness of a master rail edge, parallelism
  of a follower rail, and squareness between axes within the rail plane. Their
  route is "adjusted at assembly": a cast-in steel side bar with push screws,
  aligned against a straightedge, the master rail or a granite square, with the
  measured profile attached as evidence. Replication still owns the plane. The
  rail-mount family that declares these lives in MachineKit (RM2); step 2 of
  phase 7 owns the relationship records (RM4).

### Foundation-level declarations

- **Datum scheme:** primary, secondary and tertiary datum faces or features for
  replication, machining and fixtures. Masters and datum fixtures reference them.
- **Lifting and handling:** `LiftingMount` inserts placed by the design. Handling
  checks use them, and the lifting-point castability rule proposes positions from
  casting mass and centre of mass.
- **Foundation joints:** the base–column interface becomes an explicit joint
  between two foundations, with grout or replication gap, jacking screws, anchor
  pattern and cure-before-torque. It replaces the standing change request.

### Layout parameters

`MillFrame` builds features from named layout parameters instead of inline numbers
(shoulder clearance 12, recess depth 21, foot access 90 × 100 at 40, stem
20 × 20 × 42, seat width 106, bores 10.2 and 18). Tap drills derive from thread
specifications. Shared feature patterns (open underside pocket grids, foot access
with stem, recesses beside guide seats, retained webs and backing) live in one
builder in `machinekit.structural`.

- The foot pattern starts from the hobber's `CastingFootPocket`
  (side-access pocket, removable plate bearing up against a registered roof,
  stem clearance, reserved boss) with its constants as parameters, reconciled
  with the mill's foot access and stem. The plate register is an
  `InterfaceRecess` owned by the removable plate.
- One levelling-foot component family replaces the overlapping `LevellingFoot`,
  `MillLevelingFoot` and the hobber's `LevelingFoot`/`LevelingSeat`, with variants
  for screw, pad, removable plate and nuts.
- The hobber is on main (merged at 28c395bd8) without conversion, so phase 7
  step 1 converts the mill and the hobber together: one feature type and one box
  algebra hold on main, not after a later rebase. `FoundationLayout` builds typed
  features through the shared builder with declared kinds, openings, owners,
  adjustable dimensions and `derivedFrom` fragments; its `legacy` flags and
  branches go. `CastingFeatures`, `CastingCells`, `CastingFootPocket`,
  `LegacyFoundationRecipes` (with `decodeLegacyBaseline` and the HB5 recipe
  tests) are deleted, and baseline reports are regenerated rather than read in
  an old format. `hobbing.MineralFoundation` and `hobbing.FoundationIntegrity`
  fold into the structural classes: bed and column are configured structural
  foundations, and rail coplanarity waits for step 2 as a declared
  `InterfaceRelationship` instead of hobber code. Inserts get typed roles and
  finishes directly, threaded holes through `InsertHoles`, and labels without
  hardware data.
- Cast-in fixing sockets (`CastInFixingSocket`, round catalogue sockets) are an
  insert body variant in `materia.casting.InsertSpec` (box or cylindrical body,
  plus the same hole records), not a second insert component. A given anchor is
  modelled one way: the hobber tray anchors are sockets, not also box seats. The
  socket fixture becomes an input to the shared insert-fixture planner beside
  `DatumFixtures`: plate size, sites and clamps derive from the foundation's
  inserts, the demould order comes from the planner's removal sequence, and
  `SocketFixturePlanner` keeps only socket-specific geometry.

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

Every insert fixture is checked for full constraint: its pins, slots, faces and
clamps must remove all six degrees of freedom of the insert, with no redundant
locator fighting another. The check reports the free or over-constrained
directions per insert. The fixture's stated accuracy budget is the insert
tolerance minus its finishing allowance; fixtures must not block the pour where
the mix must flow, and each must come off after cure.

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
fabrication package. Phase 9 coupons cover replication thickness, flatness
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

Recipe limits start as declared assumptions. When phase 9 evidence revises a
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
- **Pour plan.** From the resin mass and the recipe's mix (binder and aggregate
  fractions by weight), report material quantities, the batch count for the
  declared mixer capacity and working time, and the pour sequence with each
  batch's level range. Undeclared mixer or working-time data leave it pending.

The flat free surface is an idealisation: a vibrated paste traps at least these
pockets, and may trap more under broad flat ceilings. The air-escape-width rule
covers that case, and phase 9 compares predicted pockets with demolded voids.

### Cure temperature

The exotherm matters because EPS softens near its declared limit, resin systems
have gel and limit temperatures, and temperature differences move inserts.
Three levels of analysis; a screening result never clears coverage or junction checks:

1. **Lumped estimate.** The remaining reaction rise w·H/c is a uniform-mix
   estimate. Conduction can preheat unreacted material, so adding it to the
   maximum initial/environment temperature is not a spatial upper bound. A
   screening Pass requires a positive declared margin to every material/gel
   limit; it cannot qualify a casting without the separate thermal-model and
   physical-evidence checks.
2. **Section model.** Transient 1D conduction through each section, with heat
   generation from the `ResinCureSpec` kinetics and boundary conditions from the
   mold: panel (board conductivity, thickness and convection), EPS (near
   insulating), open face (convection) and steel inserts (heat sinks). Section
   thicknesses come from the casting: distances between opposing planar faces for
   walls and ribs, and the largest inscribed thickness at junctions. Other shapes
   are unsupported until a medial-axis method exists. Implicit time stepping in
   ManufacturingKit, verified against the insulated energy balance, convective reactive refinement
   and analytical inert-slab cooling series. No slab Pass is allowed until the
   remaining reaction heat is below its declared tolerance. A slab result does
   not establish a junction result.
3. **3D transient.** Only when a junction or thick block is within the recipe
   margin after tier 2. Use the existing Gmsh/CalculiX pipeline and its
   solver-fixture gate. Cure heat generation uses either a CalculiX DFLUX user
   routine or a staggered external kinetics update, whichever passes the
   adiabatic, 1D-slab and mesh-convergence fixtures; record the choice. Move the
   shared deck writer out of the bench-mill example when this second consumer
   needs it.

Missing kinetics or mix properties leave thermal qualification pending. A lumped
screening Pass never removes model coverage blockers. Physical-trial thermocouples (thickest section
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

### 7. Foundation design model

Implement `materia.casting` and the source model described under foundation
design model, in three steps:

1. **Records.** Typed features with openings, finished extents, owners,
   adjustable dimensions and `derivedFrom`; shared vocabulary and the one box
   algebra in `materia.casting`; insert holes through `DesignInterfaces`; typed
   finish; named mill layout parameters, the shared feature builder and one
   levelling-foot family; the adapter reduced to a copy; the hobber foundations
   and fixing sockets converted and their legacy types deleted. Geometry stays
   box-only.
2. **Requirements.** Interface requirements with the allowance in the casting
   geometry; interface relationships with axis members, replication jigs and
   metrology; insert-fixture constraint checks; bolted insert anchorage and the
   insert manufacturing records with drawings and operation sheets; the datum
   scheme, lifting inserts and the base–column joint.
3. **Geometry.** Draft, corner radius and prism features through CAD, planner,
   drawings, CAM and the phase 6 analyses.

Acceptance: after step 1 the mill base and column rebuild with zero symmetric
difference against their previous CAD, and so do the hobber bed and column;
fill, collision hulls and section stiffness give identical results through the
shared box algebra, and fragments group by `derivedFrom`. A body the box hull
cannot represent reports its collision hull Unsupported rather than omitting it.
`CastingFeatures`, `CastingCells`, `CastingFootPocket`, `LegacyFoundationRecipes`
and the hobber `MineralFoundation`/`FoundationIntegrity` no longer exist. `MillMoldSetup` has no
feature-ID strategy table, and an override names its feature explicitly. No code
outside the shared finished-cavity function clips design cuts; an unreached
declared opening, or an undeclared breakout, is rejected. A column cut crossing
the sloped foot yields an exact prism finished cavity that box-only consumers
report as Unsupported by feature ID. Every penetration record traces
to a `HoleInterface`, and no insert label carries hardware data. A replicated
seat's allowance appears in the casting geometry and clears its allowance request,
while master and release stay pending evidence. A relationship whose metrology
uncertainty exceeds its tolerance stays pending; an under- or over-constrained
fixture reports the affected directions. Every mill insert has bolted anchors
whose holes pass the hole policy and clear its mounting holes, whose hardware is
in the BOM and the overlap checks, and whose capacity stays pending; an anchor
too close to the mounting face, another hole or the casting surface fails with
the insert ID. Every insert has a drawing, an operation sheet and a pre-cast
inspection list, and an insert finished after casting carries its stock in its
own geometry. Mill rail-seat holes derive from the placed rails, every rail screw is
coaxial with a seat hole, and each hole reports its clearance budget and route. The lifting-point and column-joint
requests clear when the design declares them and return when removed. A drafted
former fixture passes release and clears its release request; a prism feature with
a corner radius below the router radius fails with its ID. Fill analysis agrees
with CAD booleans on a sloped-ceiling fixture, and box-feature results are
unchanged. Castability proposals change only adjustable dimensions.

### 8. Column and independent structure

Add the column setup, stepped/flared envelope support, intersecting channels and
side-removal details on the phase 7 model. Add an independent structure using
the shared planner with different dimensions, insert layout and opening direction.
The independent structure is the hobber's monolithic plinth (930 × 800 × 800 mm)
and column on the phase 7 model, including the C-axis replication relationships.

Acceptance: both selected mill foundations generate complete review packages;
the pedestal uses shared records/joints/planner with configuration only. Changing
stock thickness, casting size, opening direction and core strategy requires no
mill-specific conditionals in the libraries. Existing frame geometry is unchanged.

### 9. Physical calibration and release

Fabricate a joint coupon and a representative small mold with EPS core, steel
insert, covering, seam and restraint, using the phase 3 box where practical.
Record actual dimensions, release behavior, core deformation, thermocouple
traces (thickest section centre, core surface, a panel face), voids found at
demold against the fill analysis's predicted pockets, compaction method and its
measured settings, and insert pull-out if tested. Include the rail edge trial
from `machinekit/RAIL_MOUNTING.md` (RM6): a replicated pad about 1 m long, a
real rail aligned with push screws, straightness before and after torque and
after one week. Fit or confirm `ResinCureSpec`
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
on main), CNC posts beyond CamKit's LinuxCNC writer, CAM for the steel inserts
(CamKit has no tapping or facing yet; inserts are drawn and made or bought),
general arc-native CAM IR, rotational removal paths, automatic hardware optimization, automatic application
of design-change proposals, parting-surface search beyond listed orientations,
the 3D cure tier unless a section needs it, printed mold pieces (a former
strategy for shapes panels and foam cannot make, segmented to the printer's build
volume and emitted as `Slicer` jobs), sand-cast metals, the items under "Not modelled", mold
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


## Follow-up review: 51dbe29ee..fc0b267b7

- [x] CAM 1–7: honest single-tool G49 touch-off instructions and per-tool diameter/
  shape comments; dependency-aware minimum-load ordering; onion skin plus final
  removal pass or force-qualified vacuum; declared cutter reach, drill pecking,
  through-only breakthrough and machine spoilboard plane; directed raster/on
  loops, proven local pocket links and near-surface rapid approaches. Shipped
  settings and real CAM reach/depth rejection have acceptance tests. The bed's
  access graph requires four loads, rather than the suggested three.
- [x] Fabrication/planner 8–14: actual foam/bridge bond overlap; reject unsupported
  precision allowance; required typed insert roles and separate labels; reject
  unsupported plate-support submissions; hashed hardware validation cannot be
  dropped by callers; manifest status/severity consistency; fill-clipped datum
  intersection; renamed release-alternative request.
- [x] Process 15–18: conservative unknown EPS/adhesive handling; residual-heat cure
  extension with declared cap and Pending at cap; separate heat-removing-face
  thickness interval; thermal boundary ownership and cross-axis coverage checks.
- [x] Orientation/export 19–20: CAD emits its own four evidence records; candidates
  share analysis, catch process failures independently and mark unplanned cases
  Unsupported; rank failures/open requests first after feasibility; actual
  foundation CAD volume must agree with the fill report independently.
- [x] Castability/materials 21–23: conditional release proposals, recipe-driven
  source-only level-1 rules and design metrics, explicit proposed parameter/value,
  accepted insert-width/recess change followed by replan test; FoundationIntegrity
  Diagnostics and MillDesignStudy integration; raw cure/fabrication coverage and
  EPS consistency; obsolete first-order/upper-bound claims removed.

The coupon trial, measured tools, machine work-origin/fixtures and physical
material/process evidence remain outstanding. These changes do not release
fabrication. See the mill TOOLING_README for current study settings and commands.

Validation for this follow-up: neutral tooling/process suites pass 1,712 assertions;
CAM panel tests 293 and general CamKit 12,386; CncKit 436; native tooling 375;
casting 49; full mill acceptance 248. Both records-only and full fabrication
exports pass manifest integrity/tamper checks, including dropped hardware checks
and invalid status/severity. The full package contains 837 hashed artifacts and
40 review programs; readiness remains false (14 Pass, 250 Pending, 15 Unsupported).
The actual foundation symmetric difference is zero. Review and coupon archives
are regenerated from the verified parent manifest; no physical trial is recorded.
The optional broad recipe-contract run still needs the external HGH15CA reference
asset; the default casting suite and native/full tooling checks pass without it.

## Follow-up review: 1bf30dc87 and the hobber merge

Phase 7 step 1 continues from here. First restore main: the MachineKit tooling
suite fails at `FoundationToolingTests.hx:278` ("Array index out of bounds")
after the hobber merge, most likely in the hobber bed/column equivalence checks.

- [x] Stage 0: restore the MachineKit tooling suite after the hobber merge.
  Root cause: the test still indexed the removed static bed `columnPlate`;
  the merged moving-column design instead declares carriage guide seats and a
  column `basePlate`. The test now verifies those exact replacement interfaces
  and roles, with no adapter or casting-geometry changes. Prebuilt OCCT and the
  shared heavy runner: FoundationToolingTests **62**, PanelNativeTests **132**,
  BoxMoldTests **153**, ProcessNativeTests **34** assertions pass.

- [ ] Source contracts 1–7: callers declare openings, and `reachedOpenings` is
  diagnostic only; unique body-face IDs per prism and face; flared or chamfered
  bodies report collision hull Unsupported, and material residue names the
  feature, not the envelope prism; delete `ToolingRecipe.hardware.insertBoreDiameter`
  and derive bores from holes; consumers read full `InsertHole` records with
  frames, not `InsertHoleGeometry.diameter()`; one stored hole encoding (no
  `depth:null` sentinel, frames not overwritten, real minimum wall, clearance and
  engagement from the thread spec, not 0.001 placeholders) and multi-hole patterns;
  finish tokens from `InsertFinish`, as roles already are.
- [ ] Mill layout 8–12: the remaining MillFrame literals (90/100/40, 20/20/42,
  160, 80, 22/11, 100, the `-1`/`+2` overshoot margins, `rearBayDesignClearance`,
  the "M12" literal) become named details or declared openings; thread sizes are
  typed specs, not strings; callers use the shared foot/underside/retain patterns
  or the patterns are deleted; one envelope from `MineralCasting.castingEnvelope()`
  (no rebuild in MillFrame, MillPedestal or MineralFoundation); delete dead
  `recessLinks`, `ToolingCodec.readFeature` and the `ToolingPrism` alias.
- [ ] Setup 13–15: delete the MillMoldSetup ID-keyed strategy table in favour of
  recipe rules on kind and openings, with overrides naming their feature; draft
  comes only from `feature.draft` (remove setup `draftDegrees`), and corner radius
  from `feature.cornerRadius`; compare kinds as `InsertRecess`, not strings; typed
  adjustable dimensions; retained webs and backing subtracted through the algebra
  with fragments grouped by `derivedFrom` in cores and castability.
- [ ] Hobber conversion 16–19: as under layout parameters above; regenerate
  `BASELINE_FOUNDATIONS.json`; delete `machinekit/CASTING_TOOLING_PLAN.md`; fix
  `machinekit/README.md` (CastingFeatures description, legacy read path).
- [ ] Tests 20–22: verify the frozen STEP SHA from `reference.json` and decompress
  natively, without python3; run the sloped-foot cavity through each box-only
  consumer (fill, sections, planner) and assert Unsupported by feature ID; fragment
  ID stability under unrelated edits and change under geometry edits; reject old
  feature and tooling document versions.

