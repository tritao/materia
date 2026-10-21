# Base casting mold review

This is phase 4 of `manufacturingkit/TOOLING_PLAN.md`. It generates the mold for
the same complete frame instantiated by `MillFramePreview`: both XY and Z
interfaces are present. `MillMoldSetup` adapts `frame.base`; it does not repeat
casting dimensions or generate its own joints.

```sh
./haxeon/scripts/haxeon run --project=machinekit/tests/tooling/haxeon.mill.json
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.tooling.json -- --checks
python3 machinekit/examples/bench-mill/render_tooling.py
./app/run-built.sh --project="$PWD/machinekit/examples/bench-mill/build/tooling/base/mold.mtrg" --perspective
```

The entry point uses `build/tooling-runner/`, independently of the existing
frame/FEM configurations. Review artifacts stay under ignored
`build/tooling/base/`. No inventory is consumed and no CAM/G-code is emitted.
The renderer uses NumPy and Matplotlib; it plots actual exported CAD triangles.
`mold-cutaway.png` explicitly hides the south and west walls for inspection.
The `.mtrg` scenes retain all components.

The current source has 31 features and 12 inserts. Every feature retains its
source ID in `feature-inventory.json`. The kit includes:

- Five measured-thickness panels from the shared `MoldPanels` generator.
- Six underside EPS cores on two shared transverse bridges. Each EPS core is
  laminated from two ruled stock blocks that fit the declared 100 mm stock.
  Covering, interlayer adhesive and foam/carrier bond thicknesses are included
  in the finished dimensions. Broad carriers are permanently bonded to their
  bridge; prebond all three cores in a row before installing that hanging unit.
- Two longitudinal support rails with blind wall-top locating sockets, round/slot
  positioning and separate M6 clamp bolts. The rails themselves need qualified
  external clamps to the walls; their attachment capacity is not established.
- Fourteen connected detail/backing formers for shallow guide/drive recesses,
  side-withdrawing foot-access openings and separate fill-side stem pieces.
  Overlapping cuts are allocated deterministically into disjoint tooling solids;
  insert envelopes and datum fixtures are subtracted, preserving the intended
  intersection between foot-access and stem openings around the steel seats.
- Twelve nominal steel datum supports at the actual insert heights. Bed-facing
  supports seat in blind board sockets; elevated foot supports have external
  columns and withdraw sideways. Each has round/slot locating receivers and a
  clearance-only path for screws entering declared steel insert threads.
  Screw lengths include the bed, datum and the source hole policy's engagement.
  Fixing holes carry screws; other source holes carry framed bore masks.
  Missing usable source threads raise design change requests. Existing threads
  obstructed by the current support/locator layout raise tooling validation work.
  Both block readiness. These supports do not establish final insert alignment;
  fixture constraint, accuracy and release/access qualification remain pending.
- Recipe rules choose ordinary feature strategies from typed kind and declared
  openings; setup overrides name only specific features. Draft belongs to the
  source feature. Unknown kinds or override IDs fail. Zero-draft rigid features
  need a release alternative; unsupported strategies block readiness.
  Positive draft cannot silently taper the current box-only source geometry.
- Retained beams and backing split authored features through the shared box
  algebra. Stable fragments retain their parent through `derivedFrom`;
  `core-units.json` groups the foam pieces by their authored core intent.


The insert stock specification is C45 EN 1.0503 bright or ground bar, with a
580 MPa minimum tensile strength; service and fixture screws are class 8.8.
Buried M12 base anchors and M4 column hatch anchors have blind, flat-bottom
machined bores followed by bottoming taps. Usable thread excludes two pitches of
runout. The base anchors use named 30 mm stock and derived 40 mm service screws.
Their mounting plane is unchanged; the casting refreeze records 96,000 mm³ of
additional steel recess. Insert manufacturing and physical qualification remain
part of the pending work.

Tapped bore masks are threaded set screws carrying the source thread. Accessible
through holes have a separately budgeted end closure; source checks reject a
thread exit disk that meets casting material. Fixing holes carry their fixture
screw and preserve the blind steel back wall. Threaded hardware has explicit
release steps before the checked linear withdrawal sequence.

Draft and radius intent is preserved in the source. The box tooling template
reports unsupported geometry with its source ID and a linked request. CAD,
fill and section assessments produce no substitute geometry or numeric report.
The sloped-foot fixture exercises the actual fill, section and planner entry
points against its independent CAD cut. Unsupported command-line assessments
are written under `build/tooling/base-source-assessment/<source fingerprint>/`.

The inverted setup puts rail faces toward the bed, underside cavities toward the
fill face and the nominal rail mounting plane at mold Z=0. The casting envelope
starts 4 mm above that datum in this source instance. Backing formers fill that
space without changing the casting or moving its steel inserts. Precision rail
and screw-bearing seats follow the reviewed **replicated** route; their actual
allowances, masters, flatness evidence and hole protection are pending. Source
finish mismatches and missing machining/replication stock produce change requests.
Precision masters, dams and hole plugs require resolved source allowances and
process choices. The fabrication package reports these missing tooling routes;
no unused geometry fields imply that they have been implemented. The column joint has a separate pending grout-gap, jacking, sleeve and
cure-before-torque record. Requests flag any source-interface changes needed;
this generator does not invent replication or grout clearance in the machine.

The study shares the measured sheet-thickness parameter between the contact
panels and nominal plywood support displays. Separate stock lots and material
properties need qualification before the fabrication package is approved.

The review checks source casting symmetric difference, closed boundary coverage,
positive-volume interference and exact continuous straight-translation sweeps.
There are separate assembly and demolding graphs: before pouring, bed-facing
parts approach from the fill side; after cure they leave below the released bed.
The assembly graph treats each prebonded row as one loaded fixture. Demolding
separates EPS from the reusable bridge/carriers after adhesive release.

Steel seat bores are actual holes. Explicit temporary masks and through-bed fixture
pins join both exact swept graphs, with distinct installation and withdrawal
motions. Fixture pins enter from below after lateral supports are seated, and
withdraw below before those supports slide out. Only already-accessible loose fixing stacks are prerequisites; a bore
mask or trapped pin is not assumed to disappear. The ordered demolding steps
include the source-to-service flip/reorientation after withdrawal. Resin and
bored-steel mass and CoM come from declared recipe densities and are checked
against CAD. Lifting qualification requires typed equipment, capacity, verified
mass/CoM, attachment IDs and verification references.

`diagnostics.json` uses the same code/subject/severity vocabulary as
`machinekit.design`. Every change request names a real validation rule. Finished
profiles retain exact router reliefs with matched former sealing sections, and
explicit pocket-floor drill operations reference their access pocket. CamKit
owns cutter compensation.


The package contains a single mold scene with an Explode slider, portable assembly recipes,
STEP solids, panel SVG/DXF and operation records, local EPS layer STEP/SVG/DXF
with ruled cutting definitions, normalized source/plan/options, a geometric
review, typed diagnostics, handling/demolding-step records, change requests and
ordered review instructions. Native checks compare
against actual `frame.base.body` CAD, cover stock changes and the optional-XY
inventory, and reject missing pockets, displaced datums, unclassified holes and
an inward-trapped foot former.

**This is not fabrication-ready.** Outstanding items include fixture attachment,
EPS/adhesive/release/cure evidence, insert anchorage, replication/grout allowances,
compaction, closure/brace/bridge loads and qualified lifting/flip provisions.
The review always reports fabrication readiness as false; a correct geometric
mold is not a structural or process qualification. Phase 5 adds the fabrication
review and load/access checks below; phase 9 supplies physical evidence.


## Fabrication review package (phase 5)

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.fabrication.json -- --checks
./app/run-built.sh --project="$PWD/machinekit/examples/bench-mill/build/tooling/base-fabrication/mold.mtrg" --perspective
```

`MillFabricationSetup` declares process/load inputs and study CAM settings. Its
resin/adhesive compatibility, computed cure inputs, compaction pressure, material
properties, support/connection capacities and shop workholding are deliberately
unqualified. The entry uses `build/fabrication-runner/` and generates into a fresh
staging directory. It publishes `build/tooling/base-fabrication/` only after the
export and manifest verification succeed. No stock inventory or machine is changed.

The package contains the full phase 4 CAD/drawings/assembly and these additions:

- Nineteen panel/coupon blank requirements, with source IDs/revisions, measured
  thickness, original bounds, margin and permitted rotations. The sheet bridge
  retains a registered material ID and an explicit `sheet-stock.json`; the supplier
  stock description stays in the panel recipe. Shelf layouts are
  rectangular blank planning aids, not contour nests or executable sheet plans.
- Per-panel and combined operation CSV; EPS layers, embedded steel inserts,
  former/fixture pieces and hardware lists, with reusable/per-casting groups.
  Hardware CSV and aggregated BOM use actual assembly component lengths and
  washer/nut counts; the raw grip/engagement plan is retained separately.
  Unspecified process consumable quantities are not guessed.
- Fit-coupon receiver/plug and round/slot locator-coupon pairs, with STEP,
  SVG/DXF, panel recipes, CAM and a recipe/package-linked `coupon-trial.csv`.
  `coupon-instructions.md` explains the measurements and representative core
  retention trials. Cutting and measuring these pieces requires the shop.
- Twenty-five LinuxCNC programs, one per declared panel face/setup. Exact
  operation IDs remain in `cam-programs.json` and CAM provenance. Tool diameter,
  stock, fixtures, work origin, XY reflection and machine travel are retained.
  `cam-settings.json` explicitly labels the feeds/RPM/tool library and travel as
  study values. An empty fixture list is not qualified workholding. The final
  outside cut needs a qualified retention/release method.
- `load-estimates.json`: peak-head panel loads, gross displaced-mix core uplift
  and local carrier/bridge demands. Additional compaction pressure is a supplied
  input, not inferred from vibration frequency. Uncut isotropic rectangles can
  use the declared simply-supported plate model; tabs/cutouts/rebates and local
  support loads require a qualified model. Missing brace, fastener and core
  capacities are per-rule pending results. No EPS-weight credit is used.
- `fabrication-validation.json` and Diagnostics vocabulary in
  `fabrication-diagnostics.json`. Styrene/EPS and a cure peak at the declared
  core limit fail outright; missing evidence, compaction or restraints stay
  pending. Source design requests and fixture/finishing qualification remain
  blockers. The new report does not overwrite the phase 4 review status.
- A versioned `manifest.json` containing source/fabrication/package fingerprints,
  every generated file's byte count and SHA-256, validation and separate readiness.
  Only STEP header timestamps are normalized for reproducibility. Verification
  catches modified artifacts; `--checks` includes a tamper/restore regression.

CAM uses the same finished contours as CAD and drawings. CamKit retains the exact
boundary for cutter compensation, then samples its centreline to the declared
20 micrometre chord tolerance. Cutter-radius reliefs need separate vertical
entries when their centre disappears from a normal offset; CamKit checks the
complete cutter disc against the exact source face before adding those entries
under the same operation. The actual tool must be suitable for axial entry.
ManufacturingKit never compensates profiles or emits SVG/DXF.

For a quicker list/CAM check, use `-- --records-only --checks`. It writes a
separate `build/tooling/base-fabrication-records/` directory and explicitly keeps
its CAD/sequence check pending. Use the full command for the complete review.
The existing phase 4 entry remains a geometry-only export without G-code.


## Fill and cure checks (phase 6 foundation)

The fabrication command also emits `casting-process-spec.json`,
`fill-analysis.json`, `cure-analysis.json` and `process-diagnostics.json`.
`MillFabricationSetup.process` binds these inputs to the exact mold recipe.
Its supplier properties, mix/ambient temperatures, thermal margin, pour rate,
air-escape width and slab definitions remain explicitly unknown.

The current inverted base has no disconnected air component under the exact
orthogonal flat-free-surface model. This does not qualify paste flow: the
broad-ceiling escape-width check remains pending until its limit is declared.
The fill volume is checked against the exact source CAD review in a full export;
a records-only run leaves that agreement pending. Candidate vents on a failing
fixture are suggestions, not automatically drilled or sealed tooling holes.

Cure reports do not invent resin properties or claim that EPS is safe. The
lumped adiabatic estimate requires a positive declared margin and is not a
spatial upper bound: conduction can preheat unreacted material. It never clears
thermal coverage or junction checks. The implicit 1D solver supports declared
autocatalytic kinetics, checks remaining reaction heat before allowing a section
Pass, and reports a global energy balance. First-order kinetics are kept for
analytic fixtures only. Resin identity must match the fabrication solvent
record, and gel/material limits include EPS-containing adhesive even without
foam cores. `fabrication.materials.cure` derives only from CureReport; there is
no second assessed-peak field. Material data, numerical convergence, geometry
coverage and thermocouple evidence remain blocking.

The process analysis now includes extracted opposing-face planar spans, the
orientation comparison and checked-order playback described below. Complex
qualified junction heat flow, broader design rules, accepted source edits and column studies remain outstanding.
The mold is still a review package.


To supply declared process properties, copy `casting-process-spec.json` outside
`build/`, preserve its recipe fingerprint and edit its explicit unknown values.
Use supplier/trial references for the actual mix rather than generic resin data:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.fabrication.json -- --process-spec /path/to/casting-process-spec.json --checks
```

An old recipe fingerprint, unknown field/model or invalid physical input is
rejected. Review exports may contain failing checks; always read final manifest
readiness. No command executes a CNC program or accepts physical qualification.


## Orientation comparison, sections and sequence scenes

The full fabrication package now contains `orientation-comparison.json` and
`.md`, `section-geometry.json`, `sequence-playback.json` and a `sequence/` folder
of `.mtrg` snapshots. `sequence/README.md` lists scenes in order. The snapshots
show states after each checked installation/removal, including prebonded groups,
loose hardware, the cured casting and the final service-orientation endpoint.
They are visual-only (no collision proxies or dynamics); exact collision and
removal evidence remains in `review.json`. They do not simulate adhesion, EPS
breakup, fastening or a flip trajectory.

The comparison can also run independently:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.mold-study.json
```

This compares the same source and recipe in inverted, upright and side-on
orientations under the existing feature strategies and support policy. The
current inversion has a complete mold and exact CAD/swept-graph check. Upright
avoids a flip, but upright/side-on cannot use the current fill-open shared core
bridges. They need alternative support/fixture designs before a complete mold
can be assessed. Source fill/cure reports remain available even if planning fails;
failed candidates have no fabricated effort or source-change counts. Comparison
never changes the selected setup or grants fabrication approval.

`OrthogonalSections` measures continuous material between opposing planar faces
on all three axes. It merges through Boolean partition boundaries, preserves
source face IDs and verifies each axis covers the full mix volume. The output
can supply `CureSection` thickness/source IDs when actual thermal boundary data
are declared. Chord lengths do not establish a complex junction's inscribed
thickness; the exposed-face search supplies bounded Euclidean thickness instead.

Authored thermal section/boundary assignments apply to the selected orientation.
Alternative candidates reuse material properties but require their own boundary
assignments; the comparison does not silently rotate heat sinks or convection.
Records-only exports contain no sequence scenes and leave every candidate's CAD
assessment pending.


## Castability feedback

`castability-analysis.json` and `castability-review.md` contain source-owned
findings and suggested actions. The same screening results are included in each
orientation candidate. The report retains existing fill topology/ceiling and
planner draft checks, adds aggregate-based passage and section-size screens,
and keeps EPS breakout trials explicitly pending. Nothing changes the casting.

The strict process input is now version 3 (versions 1 and 2 are rejected). Its required
`castability` object declares `maximumAggregateSizeMm`,
`minimumPassageAggregateRatio`, `maximumSectionThicknessMm` and `evidence`.
The mill leaves these limits null until the actual mix/process is selected.
Supply them in the existing `--process-spec` input, preserving the recipe
fingerprint. Declared limits are screening assumptions, not physical qualification.

Suggestions identify source IDs and the blocking rule. Where supported they give
measured/target dimensions, an axis and a representative region, or a candidate
vent point, all in mold coordinates (mm). A design reviewer must map a proposed
face separation to actual source parameters and replan a new source revision.
Vents need a continuous route and sealing plan; no penetration is added by the
report. Junction inscribed diameter now has explicit numerical bounds; junction heat flow, cure, structural,
release and handling evidence remain independent requirements.


## Junction bounds and thermal coverage

The full and records-only packages now include `junction-thickness.json` and
`thermal-coverage.json`. The orientation comparison and `castability-review.md`
show the same geometric bounds and unbound thermal volume. The initial mill search
finds a largest-inscribed-diameter interval of 216.99–217.47 mm within a declared
0.5 mm numerical tolerance. The witness and nearby source faces are recorded;
full-mill acceptance checks its containment against the actual foundation CAD,
checks that enlarging it crosses the boundary, and compares exposed surface area.

This is a geometric thickness result. It does not claim that a 217 mm slab models
the junction's heat flow. The present process setup declares no thermal sections,
so all 70,107,544 mm3 remains unbound. Supplier data, actual boundary conditions,
a junction thermal model and physical calibration are still needed.

In a version-5 process file every authored section needs `spanIds` and
`geometryFingerprint`. Use empty IDs and null for an unbound numerical example.
For real coverage, take span IDs and the geometry fingerprint from
`section-geometry.json` and match its thickness/source faces, while explicitly
specifying the actual left/right thermal conditions. Unknown/stale references,
wrong dimensions, mismatched face boundary kinds, repeated span assignments or
overlapping coverage across section axes fail. EPS faces are insulated; the open
face removes heat only when declared. `faceContacts` declares reviewed overrides
by stable thermal-face ID; numerical heat-transfer data belongs to the section. Even complete slab volume
coverage leaves nonrectangular junction modelling pending. Review exports may
contain failures; no suggestion changes the source casting or releases fabrication.


## Manufacturing review corrections

**Not fabrication-ready.** No coupon trial has been recorded. The panel CAM route
now declares nominal versus measured cutter diameters and matching tolerance,
measured tool lengths, G43 after measured tool changes and G49 at program end.
Unknown lengths are allowed only in single-tool review programs. Operations group
by tool while preserving access prerequisites and outside-last. Run split files
in `cam-programs.json` order, including earlier faces and access cuts.

Programs begin with a Z-only safe retract and use a declared spindle spin-up dwell,
arc fitting, blending tolerance and climb/conventional direction. Through cuts
include the declared breakthrough allowance into a stock-bottom spoilboard plane.
Square rebates extend their open contour sides outside the finished outline in
CAM; the neutral profile and CAD remain finished geometry.

The study route leaves a 0.5 mm onion skin, then makes a separate final-depth
outside-profile pass. A vacuum alternative requires contact area, holding force,
cutting force, safety factor and evidence; a reference string cannot qualify it.
Missing fixture geometry, measured lengths or a qualified machine-coordinate work
origin keeps the per-panel CAM check pending. Each single-tool G49 file says to
touch off Z with that tool, and comments its nominal/measured diameter and shape
after M6. Loads follow the regenerated operation records, preserving declared
access dependencies and the final outside profile.

Study tool reach is explicitly 25 mm flute length / 30 mm stick-out, awaiting
measurement. Cuts beyond reach fail. Drilling deeper than three diameters pecks
in 3 mm increments. Blind operations get no breakthrough allowance; through
operations use the separately declared machine spoilboard plane at Z = -18 mm.
Raster passes follow the declared cutting direction; proven links within an
already cleared pocket use local clearance. Approaches rapid to surface plus
clearance before feeding. Shop feeds, machine travel, setup and the physical
coupon trial remain required.

The reviewed fill opening subtracts all tooling at or above fill height before
checking air connectivity. Curved openings requiring an unimplemented exact
projection are blocked conservatively and marked Unsupported. The level curve
subtracts sealed air and reports head for each panel at each level. A full export
writes `fill-foundation-cad-check.json`, with symmetric difference against the
actual foundation CAD; the current result is zero.

Orientation candidates that plan successfully use the same process analysis as
the selected mold. Candidates without a plan report Unsupported; a process error
fails that candidate without aborting the comparison. Only the CAD review's own
four reconstruction, coverage and graph records establish geometry evidence.
After feasibility gates the ranking compares failures and open change requests
before trapped volume, worst cure margin, flip, panels and sheets, and records
its deciding reason.

The section screen also computes a thickness interval against heat-removing
faces: an EPS-backed, one-side-cooled slab has twice its geometric thickness.
Cure continues beyond the nominal duration until residual heat meets the declared
tolerance or maximum extension is reached; the latter remains Pending. Unknown
adhesive composition is treated as potentially containing EPS. The raw cure report
and fabrication material check share coverage records and EPS assumptions.

Source-level recipe thresholds cover sharp corners, insert cover/edge/access,
lifting points and removal direction. They feed FoundationIntegrity Diagnostics
and MillDesignStudy metrics. Proposals state a source parameter and value when
supported; an explicitly accepted insert-width proposal can update the linked
recess and replan. Unknown mappings remain explicit. Non-null precision finishing
allowance is rejected until supported. Insert roles are required; labels are
separate from their shared typed role vocabulary.

Real panels with rebates/holes retain an explicit Unsupported structural model;
supplying a plate support for one throws. Manifest v3 hashes readiness and
validation, folds in hardware checks independently of caller forwarding, and
verifies enum values, severity consistency and recomputed readiness. Current
strict formats are Tooling v5, CastingMold v4 and Process v5; old encodings have
no compatibility path.

The latest full rerun passes 248 mill acceptance assertions and manifest tamper
checks. Its 837 hashed artifacts include 40 review programs. Validation reports
14 Pass, 250 Pending and 15 Unsupported, with zero foundation symmetric difference.

**Not fabrication-ready; no coupon trial recorded.**
