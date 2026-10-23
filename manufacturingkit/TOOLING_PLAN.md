# Reusable panel tooling and mineral-casting molds

Status: phases 1–5 and the phase 6 process-analysis foundation are implemented as review tooling. The fabrication package reports unresolved process, structural, fixture and finishing requirements; physical qualification remains pending.

## Scope freeze and definition of done (2026-10-21)

- **Scope is frozen.** Phases 6–8 contain what this plan states today and
  nothing more. A new requirement found in review or implementation goes under
  "Deferred scope" with a one-line reason, unless it is a defect in work already
  planned. Reviews fix defects; they do not add features.
- **Code complete** means phases 6–8 produce complete, reproducible and honestly
  validated mold kits for the mill base, the mill column and the hobber plinth.
  Readiness remaining Pending on inputs that software cannot supply does not
  block code completion. Those inputs are accuracy budgets (RM0), production
  compound and layer, the base–column joint gap and jacking, workshop equipment
  and phase 9 evidence. Phase 9 is the physical release gate.
- **Delivery.** Land work on main in slices that build and pass their suites,
  each with at most one format version bump per affected codec. Do not hold
  long-lived isolated changes. While iterating, run the focused suites only.
  Before each commit, run the full native suites and regenerate the
  packages once.

## Implementation progress

- [x] Phase 1: neutral source/setup/recipe contracts, strict source formats and copy-only adapters.
- [x] Phase 2: exact line/arc panels, fit coupons and continuous swept assembly/removal graphs.
- [x] Phase 3: single-pocket box mold with selectable core supports and source-thread insert fixtures.
- [x] Phase 4: mill-base allocation, exact reconstruction/coverage and separate assembly/demolding graphs.
- [x] Phase 5: checked drawing/CAM/shop packages; fabrication readiness remains evidence-dependent.
- [ ] Phase 6: exact fill, castability and compound-datasheet reporting landed; phase-8 setup integration remains.
- [x] Phase 7: records, requirements, exact geometry, checked former CAM and cleanup landed; EPS selected and protected square contacts retained.
- [ ] Phase 8: exact envelope tooling landed; release-system slice, column/hobber fixture integration and complete setups remain.
- [ ] Phase 9: workshop calibration and physical release remain Pending.

See [README.md](README.md#casting-tooling-inputs) for the implemented API and checks.

## Retained manufacturing review decisions

- CAM uses measured tool offsets, dependency-preserving grouping, declared
  spoilboard/breakthrough and retention, Z-only starts, spindle dwell, open-side
  rebate extension, direction and machine-coordinate qualification. Single-tool
  review programs state their touch-off requirement. Coupon evidence is Pending.
- Rigid formers and datums carry uplift and lateral-restraint checks. Machined
  panels without a structural model are Unsupported; unknown hardware blocks
  qualification and stays in the BOM.
- Flip depends on a changed vertical, bridges use the maximum row depth and an
  actual minimum width, overlaps require declared ownership, replication requires
  master/release evidence, and source interfaces own finishing allowances.
- Fill uses exposed openings, reports trapped volume and level-dependent panel
  head. Full export compares the actual foundation geometry by symmetric difference.
- Package integrity is one inventory/hash check; casting checks run at generation.

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
| `InsertAnchorage` | Insert reference, anchorage kind (bolted anchors, welded studs, undercut, bonded keys), anchor holes and hardware, resolved preload/lifting demand and recipe factor, embedment, flange bearing and projected-cone areas, spacing and cover, material capacity or pending, evidence |
| `InsertManufacturing` | Insert reference, steel grade and blank stock, placed-part connector lineage and per-hole clearance budgets/routes, pre-cast operations (cut, square, drill and tap, anchors, deburr, surface preparation), post-cast operations with their stock, inspection points before casting, thread protection, procurement `Make(process)` |
| `PanelPlan` | Stable ID, exact 2D profile, stock/thickness, mold pose, contact face, joints, operations, label zone, removal motion |
| `PanelOperation` | Stable ID, kind (outside/inside profile, pocket, rebate, drill), exact line/arc contour or hole centres, machining face, depth, through/blind, setup |
| `JointPlan` | Paired member/edge references, joint kind, datum role, clearances, reliefs, fastening and assembly direction |
| `CorePlan` | Source feature IDs, actual cavity intersection, material, layers, mold pose, covering allowance, restraint, removal strategy, cutting definition |
| `InsertFixturePlan` | Insert references, independent locating datums, fixing hardware, insert-sealed penetrations, threaded-hole protection, withdrawal and finishing allowance |
| `PenetrationPlan` | Hole/slot reference, class (outside casting, insert-sealed, plugged, fill/vent opening), sealing face or plug, seal material |
| `DesignChangeRequest` | Source feature/insert IDs, requested change (draft, lifting insert, split, radius, access), reason, blocking checks |
| `CompoundDatasheet` | Declared supplier/product reference, maximum section OR layer thickness, maximum mass per continuous lift; typed known/unresolved values |
| `FillReport` | Setup/source fingerprints, critical levels, level-volume curve, resin mass, trapped-air pockets (volume, ceiling faces, source/tooling IDs, proposed vent point), head and buoyancy per level, fill time when a pour rate is declared |
| `CastingProcessReport` | Source/process fingerprints, exact fill and pour records, one compound datasheet rule, geometric sections, castability and detailed validation; no independent format version |
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

- Draft preserves the authored section dimensions at the declared withdrawal
  opening and tapers inward. The withdrawal axis is expressed in source
  coordinates. This is the confirmed nominal-section convention.
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
- A tapped hole is through only when its far end opens into a declared opening
  or feature, where its closure stays accessible. A through hole whose exit would
  be buried in casting material is a design error: the hole is declared blind,
  with its back wall from the policy, so no plug is lost in the casting and no
  resin path into the thread exists. If the blind hole does not fit the insert
  thickness, the insert gets thicker (a design change request), never a shorter
  engagement or a made-to-length plug.
- Policy basis: default insert steel is C45 (EN 1.0503) bright or ground flat
  bar, with the grade a typed insert material carrying its tensile strength;
  service and fixture screws are ISO 4762 property class 8.8. Engagement for that
  pairing comes from VDI 2230 Part 1, clearance diameters from ISO 273 (medium),
  each recorded with its reference. Back wall, radial wall and tip gap are named
  in-house rules with their reasoning stated, not presented as standards. Fixture
  screws use the same limits as service screws.
- Anchorage is insert geometry and feeds `InsertAnchorage`. Use **ISO 4162
  Type F, non-serrated, full-form (non-indented) hex flange bolts, class 8.8**, in blind back-face taps
  from `InsertHoles` and `ThreadedHolePolicy`. A continuous flange replaces the
  washer and an external hex replaces the socket, so neither recess is present
  in the resin domain. The standard components appear in the BOM, overlap and
  exact fill checks. Keep the unchanged trap check and the vent/pre-pour-seal
  proposal for any remaining real trap; no volume threshold or assumed seal.
- One named recipe rule sizes anchor patterns from the source service screws'
  assembly preloads plus a lifting share, multiplied by a named design factor.
  The default mill rule uses a **1.5** reserve and **two** design lifting load
  paths; its lifting share includes the final as-cast hardware mass. Admit
  patterns against the class-8.8 assembly-preload limits and the hole policy,
  footprint, mounting-hole clearance, embedment, cover and tooling geometry.
  Prefer the fewest admitted anchors, then the smallest standard thread. No
  first-smallest-thread fallback; an unsatisfied insert fails with its ID.
  VDI 2230 supplies the C45 engagement basis; the existing published Optimas
  preload rows supply the assembly-preload numbers at mu=0.125. Keep both
  references explicit rather than mislabelling the preload table as a VDI joint
  calculation.
- The lean material screen specified by the anchor review (2026-10-22) is the
  source of two in-house formulas only: **bearing = guaranteed flat flange area
  x allowable cured-mix bearing stress**, and **breakout = projected cone area
  x cured-mix tensile strength**. The cone half-angle is **45 degrees**. Exact
  projected-circle union removes double-counted overlap; each anchor receives
  no more than its own projection or an equal share of that union. No further
  edge/group coefficient or full EN 1992-4 method is implied. Both cured-mix
  strengths are explicit fabrication inputs, Pending until supplied. The
  numerical checks pass only with adequate supplied values; physical coupons
  remain the phase 9 release evidence, not a new qualification record family.
  Back wall, mounting-hole clearance, embedment, flange bearing, spacing and
  exposed casting cover still have their existing geometric checks. Retaining
  compound or a realized shoulder controls thread play; no clamp against the
  insert is assumed.
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
- Cast-in fixing sockets use `InsertSpec.InsertBody` and the common `CastInsert`
  component. `CastingSolid` is a pure Box/Cylinder primitive. Insert bodies are
  `Primitive(solid)` or `CrimpedSocket(mouth, tail)`; retention geometry cannot be
  attached to a design cut. Source-owned insert recesses preserve that complete
  exterior through `CastingCavities.derive` and the shared `CastingCavityCad` backend.
- `InsertOrigin` is required: `Fabricated` or `Catalogue(product)`. A catalogue
  product identifies its manufacturer, reference, document revision and hole
  policy, with explicit nominal/drawing/measured geometry evidence. Geometry
  evidence binds product identity, revision, body and holes. All of this enters
  the source fingerprint. Nominal exteriors remain Pending until verified.
- The PFEIFER 05.250.064.040 source variant uses its published 6 mm mouth thread
  and 25 mm blind bore with a separate enlarged relief. Its document revision is
  the downloaded PDF's SHA-256. Engagement follows the explicit catalogue policy;
  C45 service/fixture engagement rules are unchanged. OD, relief and crimp dimensions
  are declared nominal study choices, not supplier CAD or anchorage qualification.
- The hobber tray's twelve anchors are sockets, with no duplicate rectangular steel
  seats. This is an explicit casting design correction: frozen pre-socket CAD is
  independently checked after restoring its old box voids and subtracting only the
  declared socket cavities. The unchanged mill and hobber-column controls retain
  zero symmetric difference. The retired socket's nominal exterior is separately
  frozen and fingerprinted to prove the body migration has zero symmetric difference.
- `InsertFixturePlanner` owns source-hole selection, protection and plate layout
  beside `DatumFixtures`. It derives the carrier blank from the casting envelope,
  sites from source holes, and locator/clamp locations from declared layout options.
  Stock screw selection checks engagement, relief, tip clearance and far-end protection.
  There are no threaded tooling receivers. Missing source threads produce a design
  change request; unused holes retain protection, and fixing holes carry no mask.
- The coupon uses this same planner and `InsertFixtureCad`, with a neutral source
  casting component. Its release sequence is a topological order of the native
  blocking graph, with explicit unscrewing before translation. The old duplicate
  socket component, coupon source encoding and `SocketFixturePlanner` are deleted;
  product-only retention geometry is realized by the common insert CAD builder.
  Sealing, fixture accuracy, wall closure, cure and mineral-cast anchorage still
  require physical evidence. No fabrication-ready claim follows from CAD checks.
- Exact round cavities are brought forward for this socket stage. Box-only fill,
  hull and section consumers report the actual unsupported source feature IDs;
  they never substitute bounding boxes. The box algebra continues to validate
  the exact intermediate before round cuts. Draft/radius and general prism/CAM
  realization remain step 3. The source/mold/feature/hole formats for this stage
  are 9/8/3/4, with no prior-format reader.


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
Styrene-bearing resin (polyester/vinyl ester) with EPS cores or potentially
EPS-containing adhesive fails outright. Actual chemical and observed cure-temperature
compatibility with EPS/adhesive/release remains physical evidence, including the
stock's declared temperature limit. The process inputs declare compound datasheet
section/lift limits; no thermal prediction is inferred from nominal material names.

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

The fixing screw runs from under the bed through clearance holes in the bed and
the datum block or post into the insert's steel thread, so one screw clamps
insert, datum and bed together and the pins locate the datum on the bed. Fixture
screws never thread into panel stock, and the tooling needs no nuts, threaded
plates or wood inserts. The fixing hole carries its screw instead of a bore mask.
A fixing hole is a source hole on the insert (a mounting hole facing the bed, or
a dedicated fixture hole declared through `InsertHoles` and the hole policy).
Tooling never invents one: an insert with no usable hole gets a design change
request, and its fixture stays blocked until the design declares the hole.

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

### Compound datasheet limits

Use one rule for the declared compound's maximum section or layer thickness and
maximum mass per lift. Values remain typed unresolved inputs until the supplier
datasheet is declared. Check geometric section bounds or the continuous pour's
layer range, and its full mass. Sequential mixer batches are one continuous lift;
no cooling/cure pause is inferred. Missing mixer data leaves scheduling Pending.
Compound, weighed-mix and fabrication identities agree. Physical trials record
actual temperatures and qualify the chosen process. Kinetic, slab and thermal
coverage solvers are outside this release, as decided in cleanup item 8.

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

Implement the castability rules, typed proposals, effort metrics, exact fill,
compound datasheet limits, pour planning and sequence playback. Expose the source
checks as foundation diagnostics and in `MillDesignStudy`. Cleanup items 8/10
replace the earlier thermal tiers and orientation-study machinery with the
supplier-limit rule and the recorded setup decision.

Acceptance: each retained castability rule has passing/failing fixtures with
source IDs; the moldability report agrees with the actual setup/recipe checks;
accepting a mapped lifting/draft proposal creates a new revision that replans
and clears its request; effort metrics are deterministic. Design thresholds come
from the recipe; material limits come from the declared compound datasheet.
Fill catches the analytic downward-ceiling pocket, clears its genuinely vented
variant, is unchanged by extra levels, and reports peak core uplift. The compound
rule rejects exceeded section/layer or full-lift mass limits, retains a straddling
section interval as Pending and leaves undeclared supplier/equipment inputs
Pending. The chosen orientation and reasons from the completed comparison remain
recorded; the study and thermal solvers are deleted per the cleanup decisions.

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

**Core strategy comparison (after the cleanup slice, before phase 8).** Run
the mill base in two variants from the same source:
- the current recipe, with lightening pockets as sacrificial EPS;
- a recipe variant whose rule makes lightening pockets that open to a body
  face into drafted melamine formers, with their draft as an adjustable source
  dimension proposed by the castability rules.

Enclosed voids stay EPS in both. Compare release and withdrawal checks, effort
metrics, panel and core counts, router time, core uplift and restraint, panel
head pressure, and the source changes the draft needs. Report both kits
side by side. This adds no new core type and no library code beyond the recipe
rule and fixtures. The user picks the production recipe from the results.

Comparison status: [the EPS baseline and missing drafted route](../machinekit/examples/bench-mill/CORE_STRATEGY_COMPARISON.md)
are recorded. Closed by user decision (2026-10-23): pockets that open to a body
face and are deeper than one board, including the four foot-access pockets, use
sacrificial EPS like the lightening pockets. Rigid formers stay for features
within one board thickness (seats, shoulders, stems, bed backing). The drafted
route is not built. The panel-box construction stays as the rule for any former
taller than the thickest declared board.

**Cleanup slice (after step 3, before the core comparison and phase 8).**
These are defects or excess weight in work already landed, so the scope freeze
allows them. Phase 8 carries this machinery to the column and the hobber, so it
is trimmed first. Items 8–14 are deletions and merges with no new capability:
kits stay identical except where an item says otherwise, and the slice report
lists each deleted module with the check or record that still covers it.

1. [x] Unresolved inputs are a typed value (known, or unresolved with a reason), not
   `Null<Float>` found by reflecting over field-name strings
   (`HoleClearanceBudget` and the other nullable inputs in `materia.casting`).
   Missing-input lists come from the type, not from field names.
2. [x] Enum tokens decode through a checked function that rejects unknown tokens;
   no `cast parameterToken(...)`.
3. [x] Each codec `snapshot` takes raw records only. It has no
   `Std.isOfType(raw, …)` branch that also accepts an already-typed value, and
   callers copy typed values explicitly.
4. [x] A test checks that every vocabulary `values()` list matches its codec tokens.
5. [x] This plan keeps design, decisions and status. Per-slice validation counts
   and progress logs move to commit messages, and the progress sections shrink
   to one status line per phase.
6. [x] Simplify the verification layers. Keep the checks that prevent a bad casting:
   fill and traps, release and withdrawal, insert location, hole and anchor
   sizing, and exact reconstruction. Review the package machinery (manifest
   recomputation, omission and tamper checks, native-check caches, per-artifact
   report recomputation, duplicate verification paths) and keep one integrity
   check per package. Remove layers that re-verify what another layer already
   proves. List each removal with the check that still covers it. Delete
   unneeded layers rather than disabling them behind flags.
7. [x] Make anchor selection fast. `InsertAnchorPattern` now takes about 92 s per
   mill build because it runs native OCCT cover and containment checks on
   every enumerated candidate. Inserts, holes, anchors and the cavity faces
   they meet are boxes, cylinders and discs. Screen candidates with exact
   analytic distances on those primitives, in the selector's existing order,
   and run the native check only on the chosen pattern as confirmation. A
   native rejection of a screened pattern is a defect in the screen, not a
   fallback to the next candidate. The selected patterns, and the kits, stay
   identical. Report selection time before and after.
8. [x] Delete the cure and thermal solvers (`CureAnalysis`, `ThermalCoverage`,
   `ThermalFaces`). Their kinetic inputs do not exist for any compound we can
   buy, so they can only report Pending. Replace them with one datasheet rule
   on the declared compound: maximum section or layer thickness and maximum
   mass per lift, checked against the pour plan. The rule's values stay
   Pending until the compound is declared.
9. [x] One mold planner. `BoxMoldPlanner` becomes a case of `CastingMoldPlanner`
   (a casting whose features are one box pocket), sharing its allocation,
   panels and records. Delete the separate planner, its codec and its options.
   Box-template kits stay identical.
10. [x] One castability pass. Fold the source screening in `CastingLevelOne` into
    `CastabilityAnalysis` and delete what they duplicate. Record the result of
    `MoldOrientationStudy` as a decision in this plan (the chosen orientation
    and why) and delete the study code and its configuration.
11. [x] Strict formats only at the boundaries: source casting features in, and the
    delivered package out (drawings, CAM, cut lists, shop and fixture records,
    the validation report, one manifest). Intermediate plans (mold plan, mold
    volumes, process, fabrication spec, castability and similar) stay in memory
    and lose their codecs and schema versions, unless the package delivers them
    as a shop document; then they are part of the package format. Exact
    reconstruction compares the native CAD with the in-memory plan and the
    frozen CAD fixtures. Do this together with item 6.
12. [x] Report Pending by missing input. Each unresolved input appears once, with
    the checks and artifacts it blocks. The per-instance records stay in the
    detailed report, and readiness is unchanged. The kit status (README and
    START-HERE) leads with this list.
13. [x] Mill-specific casting data belongs to the example, not the library.
    `MillCastingDetails`, `MillLiftingPattern` and the casting parts of
    `MillFrame` move out of `machinekit.milling` into the bench-mill example's
    tooling setup. `machinekit.milling` keeps machine parts and their source
    geometry. In the bench-mill directory, merge build configs that only differ
    by entry point into one project with named entries where haxeon supports
    it, and delete configs no suite or document uses.
14. [x] Consolidate anchors and insert stages. Anchor demand, pattern selection,
    hardware and checks are spread over about eight files in three packages
    (`InsertAnchorage`, `EmbeddedAnchors`, `InsertAnchorPattern`,
    `InsertAnchorHardware`, `InsertAnchorGeometry`, `InsertAnchorageChecks`,
    and others). Leave one neutral module for rules and selection and one native
    module for geometry. Do the same review for the insert stage model
    (`InsertManufacturing`, `InsertManufacturingPlan`, `InsertStages`,
    `InsertFinishing`, `InsertBlankFit`): keep what the fixtures and shop records
    consume, and delete the rest.

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
after one week. Confirm compound datasheet limits and the chosen process from
supplier data and trial observations. Adjust recipe values from evidence.
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
blank conversion, penetration classes, the hard resin/EPS rules, compound limits
and continuous-pour quantity/working-time checks. Geometric
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

- Drafted melamine formers (source Draft adjustability, its mapped proposal/application, leaning hollow-former walls): not built; the comparison closed on EPS (2026-10-23).

- Extreme integral JSON round-trips (observed 1e14 becoming 276447232) need a separate toolchain investigation; do not add another JSON parser to tooling.

- Interrupted/multi-lift pours with cooling or cure pauses are outside this release; current mixer batches form one continuous lift.

- Collapsible (knock-down) melamine cores for undrafted pockets that open to a face: a segmented box released by removing a key panel first, with joinery, bracing and take-apart sequence. This is a new core strategy, considered after phase 8 only if several castings of one design are planned, since reusable formers pay off over repeated pours. It builds on the screwed panel-box formers and the existing removal-sequence checks.
- Structured recipe parameters: MachineKit components store structured records as JSON strings inside recipe tokens (`MineralCasting` `embeddedAnchors`, `datumScheme`, `interfaces`; `CastInsert` `origin`). This affects every component, so it needs its own MachineKit plan rather than a casting slice.
- Generic catalogue recipe solid-count qualification: the bought Vallder BK12 assembly has 32 solids; the broad recipe test assumes one, outside casting phases 6–8.

General BREP parting-surface discovery, arbitrary contour nesting, robotic mold
assembly, wiring core cutting definitions into `FoamJob` (after foam-cutter lands
on main), CNC posts beyond CamKit's LinuxCNC writer, CAM for the steel inserts
(CamKit has no tapping or facing yet; inserts are drawn and made or bought),
general arc-native CAM IR, rotational removal paths, automatic hardware optimization, automatic application
of design-change proposals, parting-surface search beyond listed orientations,
printed mold pieces (a former
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


## Retained implementation decisions

- The base review orientation is inverted. Upright and side-on alternatives do
  not satisfy the current fill-open shared-bridge support policy; that finding
  concerns these supports, not every possible mold. Fill, release and exact CAD
  checks remain independent of the selected orientation.
- Base column-joint insert thickness changed from 20 to 30 mm, preserving the
  service mounting plane, to fit the C45/class-8.8 blind-thread policy and runout.
  The approved base change removed 96,000 mm³ of resin and added none; its frozen
  STEP was deliberately replaced in the separate source-design commit.
- Source lifting/hatch plate footprints follow the admitted anchor pattern,
  steel wall, cover and neighbour constraints, with service holes fixed. Current
  sizes and per-insert cover are recorded in the mill TOOLING_README.
- Generic strategy rules have no mill-specific ownership priority. Equal withdrawal
  normals need no face-ID tie-break; tied distinct directions require an explicit
  source-feature override. Derived fragments retain their authored parent.
- Missing source fixture threads require a casting design request. Obstructed
  access to an existing source thread remains tooling validation work; tooling
  never invents a thread or threaded board receiver.
- Real air traps remain failures without volume filtering. The former M3
  head/washer family produced a 1.5–1.6 mm-radius washer-bore annulus, accessible
  to resin below and capped above. The diagnosed component was
  `region-537a567194a8f2ac264c83b1` at Z 37.6–38.1 mm, bounded by
  `columnLeftRear/back-anchor-1-0`. ISO 4162 external-hex flange bolts remove the
  washer and socket recess geometrically; the unchanged fill check then finds
  zero anchor traps. Attribution intersects actual boundary faces and identifies
  the insert, local anchor and hardware; a pre-pour fill/seal remains a proposal
  for any real residual trap.
- CAM uses existing neutral contours and exact offsets; matched-radius relief
  centres use disc-checked entries. Source profiles never receive cutter
  compensation. Single-tool G49 programs require tool touch-off instructions;
  dependencies and outside-last rules govern minimum-load ordering.
- Unknown EPS/adhesive composition is treated conservatively for compatibility.
  Physical coupons, board grip, measured tools, machine origin/fixtures, compound,
  accuracy budgets and workshop evidence remain unresolved inputs, not defaults.

### Former manufacturing decisions

Former construction decision (2026-10-22): hollow formers use internal pocket screws, with screw heads and access entirely in the dry cavity; board/grip trial remains Pending. No through-bolts or resin-side counterbores. Neck and flange sheets remain removable board parts, with no glued blank route.
Former hardware study decision (2026-10-22): recipe-declared 4 × 30 mm pocket screws, 15° jig, 20 mm end distance and at most 100 mm pitch; these are shop-study choices, with board/grip and jig trial Pending.
Former shop decision (2026-10-22): dry-side pocket drilling uses a manual angled drilling jig after contour machining; jig setup and board grip stay Pending, and the shop record lists joint IDs, screw quantities and sites.
Former spacing decision (2026-10-22): a short rebate gets one centred pocket screw when two screws cannot retain the declared end distance and two-diameter separation; longer rebates use evenly spaced screws within the declared maximum pitch.
Former assembly decision (2026-10-22): the portable assembly uses one deterministically selected parent joint per board; all rebate contacts and closure sites remain in the validated panel plan and shop records.
Former tolerance decision (2026-10-22): on drafted faces, radial contour and reference-mesh deflection are limited to one twentieth of the declared tolerance multiplied by tan(draft), so shallow walls do not amplify discretisation beyond the vertical tolerance.
Former machining findings (2026-10-22): the preceding kit rejected sharp inner corners in bed backing, the bearing/drive seat tools and four foot-access neck/flange pairs. The contact-preserving disposition below resolves them without changing source geometry or weakening cutter-fit checks.
Former corner decision (user, 2026-10-23): the source declares a corner radius on every casting edge formed by a concave former corner, at least the declared profile cutter's radius plus the recipe margin, using the existing source radius intent. Before applying it, check against the interface records that no mating part or datum touches a new radius; a conflict is reported, not rounded. Frozen-CAD comparisons record the radii as an approved source correction, as with the insert blanks.

Corner contact decision (2026-10-23): the remaining three sharp inner loops coincide with temporary datum blocks or proud source steel. Rounding would change those mating contacts, so no source radius is applied there. Their exact disjoint box pieces are separate single-board formers with sealed contact joints; the original rectangular cavity and contact planes remain unchanged. The four deep foot-access tools use EPS, preserving the access flange and the independent datum clearance. No casting geometry correction is needed for these seven findings.

**Release system slice (after the corner and EPS slice, before the rest of phase 8).**
Replace the `pending release and seam system` placeholder with a declared process:
1. The recipe declares one release system: sealer for exposed board cores, the
   release agent, coat count, buff and drying times, and an optional PVA film.
   EPS cores declare their covering (tape or film) and the covering thickness
   already allowed for. The release system declares whether it is EPS-safe; an
   EPS-unsafe agent on a face that touches EPS fails.
2. Surfaces come from the tooling geometry, not from lists. Every resin-wetted
   face of panels, formers and fixtures gets the release system. Faces in the
   sealing list are sealed first. Insert and anchor surfaces that must bond are
   listed as must-not-coat, with masking where they sit next to coated tooling.
3. The shop record orders the steps (seal, cure, wax coats, PVA, assembly) per
   part, and the mold assembly sequence applies release before inserts are
   fitted.
4. Chemical compatibility of resin, EPS, adhesive and release stays Pending
   until a coupon trial records it. Readiness is unchanged.
Acceptance: every resin-wetted tooling face is covered exactly once, no bonding
face is coated, and an EPS-unsafe agent next to EPS fails with its face IDs.
Former CAM datum decision (2026-10-22): all former stock heights use the declared machine spoilboard plane and fixed work datum, including thicker boards. Only through bands receive breakthrough; tapered finishing offsets its contact height by that breakthrough. Program headers state the spoilboard and stock-top Z.
Former load decision (2026-10-22): hollow-former boards enter the existing peak-pressure and plate-support checks as wetted walls; finished clipping and coupled corner loads retain the existing Unsupported plate-model gap per board, without credit from nominal rectangular blanks. Gross former restraint checks remain unchanged.
Former registration decision (2026-10-22): neck and flange sheets register against the mold wall and use its external closure; the hollow body uses internal pocket screws. Capture, grip and the complete restraint remain pending shop qualification.


### Cleanup decisions

- Input states use concrete numeric/text enums for the assembly wire profile,
  encoded as `{kind:"known",value}` or `{kind:"unresolved",reason}`. Numeric
  budgets, metrology, shop-process and bought-part clearance references use them;
  anchor retaining references and mix-strength inputs use the same explicit states.
- Typed model copies explicitly write raw records before parsing. Raw snapshots
  accept one representation, with existing canonical ordering and geometry limits.
- The package manifest references the single `fabrication-validation.json` report.
  Its hash inventory protects delivered artifacts; generation collects critical
  checks once. The in-memory Diagnostics conversion remains available.
- Deleted manifest source/report/drawing recomputation is covered by generation's
  source validation, exact CAD reconstruction/coverage/withdrawal, anchor review,
  drawing construction and the package hash check. The native anchor-report cache,
  per-artifact omission/tamper loops and duplicate hardware/diagnostics reports are
  removed; one artifact-hash regression exercises the shared integrity path.
- Separately versioned mold and volume documents are removed. Exact in-memory
  plans, frozen CAD and portable fixture/component records retain reconstruction
  coverage. Shop inputs remain plain package records, without a separate format.

- Orientation decision (cleanup): retain `base-inverted` as the mill review setup.
  The last recorded study planned only that candidate; upright and side candidates
  rejected the current recipe's shared bridges because the underside pockets did
  not open onto their fill face. No candidate was qualified for fabrication.
  The study code/configuration is removed; existing effort metrics move into
  `ToolingFabrication.effort` for shop reporting and the core comparison.

- Compound decision (cleanup): the existing sequential mixer batches form one
  continuous lift. Datasheet layer thickness and mass limits apply to the complete
  continuous pour; no curing or cooling pause is inferred from a batch boundary.
  Compound identity and supplier limits remain typed unresolved inputs until declared.

- Format decision (cleanup): process, fabrication and castability plans have no
  independent format versions. Compound/pour shop records use the package format.
  Exact panel and core-cutting readers remain because delivered panel/fixture and
  cutting records use them for CAD restoration and manufacturing geometry checks.

- Anchor cleanup decision: `InsertAnchorage` owns neutral demand records,
  candidate ordering, screening and checks. `InsertAnchorGeometry` owns catalogue
  resolution, exact boundary extraction and native confirmation. The former
  pattern/hardware/embedded/check modules are deleted; these two modules retain
  their checks. The screen copies actual trimmed planar edges from the shared
  cavity realization, including openings; it never approximates them by a mesh.
  A rejected native confirmation is an error, never another candidate or plate.
- Retain the insert manufacturing model, shop-operation planner, finishing,
  blank-fit and stage helpers: drawings and operation sheets consume them;
  fixtures require before-casting holes and the assembly requires service holes.
  These are two physical stages, separate from before/after-casting shop operations.
- Anchor locking is a shoulder stop or a retaining process with a typed reference.
  No retaining product or cured-mix strength is chosen. Evidence remains Pending.

- Mill source decision (cleanup): the bench-mill example constructs its
  foundations in `BenchMillFrame`, using its own casting details and lifting
  pattern. Library `MillFrame` retains the functional datums and the supplied
  assembly; no library default or callback imports example casting data.
- Haxeon currently has one package entry, so keep the used entry/test configs.
  Delete the unreferenced base-only and motor-study preview configs; preserve
  their source parts and all check suites. Do not add a multi-entry build feature.
- Frame-check decision (cleanup): compare the actual steel interface connectors
  and selected screw lengths. The earlier 20 mm/35 mm assumptions predated the
  approved blind-tap stock and bolt-stack changes; unchanged main reproduced the
  stale assertion. No casting dimensions change to satisfy it.
- Corner-seat decision (cleanup): the non-pocketed base variant explicitly
  declares the bottom and two side openings of its existing corner steel seats.
  The strict breakout check stays unchanged; this repairs missing intent metadata
  without changing the source cut or the production pocketed-base kit.
- Build-input decision (cleanup): legacy explicit-source configs list the three
  moved example source files. Lookup roots alone did not invalidate a cached
  executable when one of those files changed; the declared inputs now do.
- Mechanical-check decision (cleanup): keep unknown motor mounting covered by
  an unconfigured motor fixture, separately from the named preset's explicit
  assumed flange. Supplier CAD checks read the existing pinned shared libraries
  through supported environment overrides; no geometry is substituted.

- Missing-input presentation decision (cleanup): README and START-HERE group
  unresolved typed inputs and named physical evidence once, with existing blocked
  rule selectors and delivered artifacts. Instance IDs stay in the detailed report;
  readiness, source-change findings and the single integrity check are unchanged.

- Common-planner decision (cleanup): the single-pocket construction shares
  `CastingMoldPlanner`, core allocation, panels, fixture/opening records and
  validation with the shared-bridge construction. Delete `BoxMoldPlanner`,
  `MoldPlan` and support-mode options; choose support from the real opening and
  declared recipe. Keep the existing north/south versus east/west shell layout
  and delivered shop projections so kits do not change.
- Retain `BoxMoldCad` as the single-pocket native construction case: its exact
  individual-bridge and bed-foot reconstruction/removal checks are not proved
  by the shared-rail builder. The common CAD API dispatches to those checks
  once; this is not a second planner or a second verification pass.

- Phase-8 shell decision: keep a reusable rectangular panel shell around stock
  bounds and derive its removable shaping fillers from the exact source-envelope
  complement. Bounds never replace casting material in resin, mass or fill.
  Separate prism-section islands are physical fillers; exterior side fillers
  withdraw outward, bottom backing downward and a fill-open collar upward.
  Native complete sweeps confirm each declared direction without sampled poses.

- Column recipe decision: put the front on the bed, use mold-Z foam wire, and
  prefer source-Y service withdrawals. The rear motor bay owns its junction
  before the selected shaft passage and then the Z channel; these explicit
  feature overrides resolve shared negative space without changing the casting.

- Panel-fixture record decision: use the source mounting frame and certified
  contact witnesses for datum geometry. Remove the unused nominal quarter-point
  `InsertDatums` copy; strict contact recomputation and the six-DOF report retain
  the actual location check. Circular witnesses are certified inside the disk,
  including thin annular mouths, rather than credited from its bounding square.

- Panel-fixture hardware decision: all direct receivers use the shared source-hole
  selector on the actual support frame, clearance-only panel bores and explicit
  screw/mask/end-plug records. Plain masks facing a panel get clearance access
  bores; resin-side receiver threads or counterbores are not introduced.

- EPS support decision: a deep access pocket retains the exact registered neck,
  flange and independent insert-datum clearance of the former template. The EPS
  blank is wire-cut in exact section bands and held by external flange clamps;
  clamp capacity stays Pending. Routed sockets remain one shared construction
  for clear rectangular bed and side faces, with the actual receiving panel.
