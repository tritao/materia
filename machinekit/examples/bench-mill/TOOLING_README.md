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
  recipe-selected tool-only threaded fixing. Bed cuts carry insert-sealed
  penetration records. These bed-registered supports do not establish final
  insert alignment: insert XY clamps, reference accuracy, thread engagement and
  release/access qualification are tooling validation work, not casting changes.
- Every ordinary feature has an explicit strategy, priority and draft value.
  Unknown mill IDs fail. Zero-draft rigid features request draft/split/sacrificial
  decisions; unsupported strategies remain in the inventory and block readiness.
  Positive draft cannot silently taper the current box-only source geometry.


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


The package contains assembled/exploded scenes, portable assembly recipes,
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
review and load/access checks below; phase 8 supplies physical evidence.


## Fabrication review package (phase 5)

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.fabrication.json -- --checks
./app/run-built.sh --project="$PWD/machinekit/examples/bench-mill/build/tooling/base-fabrication/mold.mtrg" --perspective
```

`MillFabricationSetup` declares process/load inputs and study CAM settings. Its
resin/adhesive compatibility, assessed cure peak, compaction pressure, material
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

Cure reports do not invent a resin heat output or claim that EPS is safe. They
provide an environment-aware adiabatic bound when inputs exist, and a tested
implicit 1D reaction/conduction solver for declared planar sections. Missing mix,
kinetic and boundary data, actual-section numerical convergence, section/junction
coverage and thermocouple calibration remain blocking. The existing assessed
cure-peak/material check is retained. All new checks enter the final hashed
manifest and fabrication Diagnostics alongside the existing design requests.

This implements the process-analysis foundation, not all of phase 6. Automatic
section extraction, design-rule/proposal and orientation studies, and sequence
playback remain outstanding. The mold is still a review package.


To supply declared process properties, copy `casting-process-spec.json` outside
`build/`, preserve its recipe fingerprint and edit its explicit unknown values.
Use supplier/trial references for the actual mix rather than generic resin data:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.fabrication.json -- --process-spec /path/to/casting-process-spec.json --checks
```

An old recipe fingerprint, unknown field/model or invalid physical input is
rejected. Review exports may contain failing checks; always read final manifest
readiness. No command executes a CNC program or accepts physical qualification.
