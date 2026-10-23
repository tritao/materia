# Base casting mold review

**Not fabrication-ready; no coupon trial recorded.** Deep face-open access
pockets use sacrificial EPS. Shallow rigid tooling uses single-board parts;
square datum and steel contacts are preserved by separate board pieces.

This follows `manufacturingkit/TOOLING_PLAN.md` and generates the mold for
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

Every source feature and insert retains its ID in `feature-inventory.json`.
The kit includes:

- Five measured-thickness panels from the shared `MoldPanels` generator.
- Six underside EPS cores on two shared transverse bridges. Each EPS core is
  laminated from two ruled stock blocks that fit the declared 100 mm stock.
  Covering, interlayer adhesive and foam/carrier bond thicknesses are included
  in the finished dimensions. Broad carriers are permanently bonded to their
  bridge; prebond all three cores in a row before installing that hanging unit.
- Two longitudinal support rails with blind wall-top locating sockets, round/slot
  positioning and separate M6 clamp bolts. The rails themselves need qualified
  external clamps to the walls; their attachment capacity is not established.
- Single-board detail/backing formers for shallow guide/drive recesses and
  fill-side stems, plus four side-withdrawing EPS foot-access tools.
  Overlapping cuts are allocated deterministically into disjoint tooling solids;
  insert envelopes and datum fixtures are subtracted, preserving the intended
  intersection between foot-access and stem openings around the steel seats.
- Nominal steel datum supports at the actual insert heights. Bed-facing
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
  Draft is never added implicitly by the mold setup.
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
- `fabrication-validation.json` uses the shared Diagnostics vocabulary and keeps
  every instance finding. Styrene/EPS incompatibility and exceeded declared
  compound limits fail; missing evidence, compaction or restraints stay Pending.
  Source design requests and fixture/finishing qualification remain blockers.
- The versioned `manifest.json` inventories each artifact's byte count and
  SHA-256, the detailed validation report and readiness. Only STEP header
  timestamps are normalized. One integrity check covers the delivered files;
  its shared hash path has a tamper/restore regression in the test suite.
- `README.md` and `START-HERE.md` lead with each missing input or physical
  evidence once, listing the existing checks and artifact releases it blocks.
  Source actions remain separate. The summary changes no detailed check or
  readiness result.

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


## Fill, compound and pour checks

The fabrication command emits `casting-process-spec.json`, `fill-analysis.json`,
`compound-datasheet-check.json`, pour quantities/batches and the single detailed
`fabrication-validation.json`. Process inputs are plain shop records under the
package manifest format; there is no separate intermediate process encoding.
`MillFabricationSetup.process` binds them to the recipe. Production compound,
section/layer and mass-per-lift limits, mix fractions, equipment, pour rate and
air-escape width remain explicitly unresolved until supplied.

The fill model checks critical-level air connectivity and reports boundary
source/hardware IDs, trapped volume, per-level panel head and core uplift.
The current base has zero anchor traps; this does not qualify paste flow or
compaction. A full export compares against actual foundation CAD; a records-only
run leaves that agreement Pending. Vent/sealing suggestions do not edit geometry.

The compound check uses the supplier's section OR layer limit and mass per lift.
The existing sequential mixer batches are one continuous lift: a smaller mixer
does not create a cooling or cure pause or reduce the total lift mass. There is
no thermal solver and no invented kinetics. Missing compound and workshop data
remain Pending. Shop compatibility and physical coupon evidence remain required.

To supply process inputs, copy `casting-process-spec.json` outside `build/`,
preserve its recipe fingerprint and declare the actual supplier/workshop values:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.fabrication.json -- --process-spec /path/to/casting-process-spec.json --checks
```

## Sections and sequence scenes

A full package includes `section-geometry.json`, `sequence-playback.json` and
ordered `.mtrg` snapshots under `sequence/`. Their order comes from the checked
assembly and demolding graphs. The scenes do not simulate adhesion, EPS breakup,
fastening or the actual flip trajectory.

The retained review setup is inverted. The removed orientation study planned
only that candidate: upright and side candidates rejected the current recipe's
shared bridges because the underside pockets were not fill-open. No candidate
was fabrication-qualified. The decision is in TOOLING_PLAN.md; there is no study
command or automatic setup switch.

## Castability feedback

`castability-analysis.json` and `castability-review.md` contain source-owned
findings and suggested actions for the recorded setup. The report retains
existing fill topology/ceiling and
planner draft checks, adds aggregate-based passage and section-size screens,
and keeps EPS breakout trials explicitly pending. Nothing changes the casting.

Process inputs are the plain shop records described above. Their `castability`
object declares `maximumAggregateSizeMm`, `minimumPassageAggregateRatio` and
`evidence`; the `compound` object owns the supplier's section or layer limit and
maximum mass per lift. No intermediate process version or thermal model is
required. The mill leaves production limits unresolved until the actual compound
and process are selected. Declared limits screen the design; physical
qualification remains separate.

Suggestions identify source IDs and the blocking rule. Where supported they give
measured/target dimensions, an axis and a representative region, or a candidate
vent point, all in mold coordinates (mm). A design reviewer must map a proposed
face separation to actual source parameters and replan a new source revision.
Vents need a continuous route and sealing plan; no penetration is added by the
report. Junction inscribed diameter has explicit numerical bounds. Compound,
structural, release and handling qualification remain independent requirements.


## Geometric section bounds

`junction-thickness.json` and `section-geometry.json` retain exact exposed faces,
source ownership and an inscribed-diameter interval. A safe upper bound passes a
declared compound section limit; a failing lower bound rejects it; a straddling
interval stays Pending. No midpoint acceptance or box approximation is used.
Non-box sections remain Unsupported with their feature IDs. These are geometric
screens, with no thermal coverage or cure prediction.

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

Unknown adhesive composition is treated as potentially containing EPS. Styrene
compatibility is checked independently of the compound datasheet limits. The
single validation report retains every detailed source/process finding.

## Demand-sized insert anchors

The source uses ISO 4162 Type F full-head flange bolts, class 8.8, without separate washers or socket recesses. One recipe rule sizes each pattern from service-screw assembly preload and the lifting share. Mix bearing and tensile strengths, locking evidence and physical qualification remain pending.

Lifting blanks are selected against the existing casting cavities, full thread-envelope steel walls and anchor cover. The base and rear column use 71 × 45 × 30 mm; the front column uses 40 × 100 × 30 mm where its rail recesses limit width. The 70 mm compact blank fails the demand/wall regression; 71 mm passes. Hatch blanks are 44 × 20 × 12 mm. Every service-hole position is unchanged. Clearance-bore pads retain the full central-bore wall beside the back taps: 57 mm square at base feet and 48 mm wide at the column joint.

Cover below is the minimum exposed-anchor cover to the casting boundaries for that insert, in millimetres. It is a geometry check, not an anchorage-capacity qualification.

### Base

| Insert | Raw steel, mm | Anchors | Minimum cover, mm |
|---|---:|---:|---:|
| `railYLeft` | 40 × 410 × 20 | 4 × M8 | 15.582 |
| `railYRight` | 40 × 410 × 20 | 4 × M8 | 15.582 |
| `driveYFixed` | 100 × 25 × 20 | 2 × M5 | 20.000 |
| `driveYFloating` | 100 × 20 × 20 | 2 × M5 | 19.948 |
| `columnLeftFront` | 48 × 82.4 × 30 | 4 × M8 | 26.000 |
| `footLeftFront` | 57 × 57 × 20 | 2 × M5 | 9.751 |
| `columnLeftRear` | 48 × 82.4 × 30 | 4 × M8 | 26.000 |
| `footLeftRear` | 57 × 57 × 20 | 2 × M5 | 9.751 |
| `columnRightFront` | 48 × 82.4 × 30 | 4 × M8 | 26.000 |
| `footRightFront` | 57 × 57 × 20 | 2 × M5 | 9.751 |
| `columnRightRear` | 48 × 82.4 × 30 | 4 × M8 | 26.000 |
| `footRightRear` | 57 × 57 × 20 | 2 × M5 | 9.751 |
| `liftLeftFront` | 71 × 45 × 30 | 4 × M8 | 8.000 |
| `liftLeftRear` | 71 × 45 × 30 | 4 × M8 | 8.000 |
| `liftRightFront` | 71 × 45 × 30 | 4 × M8 | 8.000 |
| `liftRightRear` | 71 × 45 × 30 | 4 × M8 | 8.000 |

### Column

| Insert | Raw steel, mm | Anchors | Minimum cover, mm |
|---|---:|---:|---:|
| `hatchRearLeftLower` | 44 × 20 × 12 | 2 × M5 | 6.800 |
| `hatchRearLeftUpper` | 44 × 20 × 12 | 2 × M5 | 10.000 |
| `hatchRearRightLower` | 44 × 20 × 12 | 2 × M5 | 6.800 |
| `hatchRearRightUpper` | 44 × 20 × 12 | 2 × M5 | 10.000 |
| `railZLeft` | 40 × 833 × 20 | 8 × M6 | 6.000 |
| `railZRight` | 40 × 833 × 20 | 8 × M6 | 6.000 |
| `footLeftFront` | 48 × 82.4 × 20 | 2 × M5 | 5.000 |
| `footLeftRear` | 48 × 82.4 × 20 | 2 × M5 | 5.000 |
| `footRightFront` | 48 × 82.4 × 20 | 2 × M5 | 5.000 |
| `footRightRear` | 48 × 82.4 × 20 | 2 × M5 | 5.000 |
| `liftLeftFront` | 40 × 100 × 30 | 7 × M6 | 9.600 |
| `liftLeftRear` | 71 × 45 × 30 | 4 × M8 | 8.000 |
| `liftRightFront` | 40 × 100 × 30 | 7 × M6 | 9.600 |
| `liftRightRear` | 71 × 45 × 30 | 4 × M8 | 8.000 |

The base has 52 anchors and the column 54. The focused native run measured 92.1 seconds across the selector calls for both foundations, including unsuccessful calls; complete frame construction took 105.4 seconds. These are observed wall times on the shared development machine. Temporary selector timing instrumentation was removed.

The anchor-revision baseline has zero trapped pockets (previously 74), with the fill check unchanged. Its 265 artifacts include 40 panel review programs; validation was 626 Pass, 568 Pending, 119 Unsupported and 0 Fail. Fabrication readiness remains false: the recorded material strengths, workshop inputs and physical evidence are still unresolved.


## Release and sealing

The recipe declares two sealer coats, five wax coats and one optional PVA coat.
Actual product references, manufacturer timing and EPS safety remain explicit
unresolved workshop inputs. `release-system.json` derives wetted faces from CAD
contact with the resin; its face IDs also identify exposed board surfaces for
sealing and protected insert/anchor steel. Adjacent bonding faces are masked.

Follow `release-operations.csv` and `release-system.md`: seal, dry/cure, cover the
wetted EPS portions, wax/dry/buff, then PVA/drying before mold installation.
Keep dry support/foot bond contacts bare and cure those bonds before release.
The EPS tape/film uses the existing lining thickness; no extra geometry allowance
is added. An EPS-unsafe agent fails with the actual affected face IDs.

All preparation precedes fitting source steel. Never coat insert or anchor
bonding faces. The existing fabrication `compatibilityEvidence` field holds
passing physical resin/EPS/adhesive/release coupon references; product declarations
alone do not qualify the system. Readiness remains false.

## Former board construction and CAM

The recipe selects sacrificial EPS for a body-open pocket whose smallest blank
section exceeds its thickest declared board. This converts the four foot-access
tools. Their registered neck/flange and clearance around independent insert
datums remain exact; EPS is wire-cut in proved prismatic bands, with the declared
covering and adhesive gaps. External flange clamp capacity remains Pending.

Shallow tools are cut from one declared 18 or 25 mm board, faced to height where
needed. The former panel-box construction remains available for tall tools:
internal pocket screws and dry-side access, no glue stack or drafted box route.

The three remaining sharp inner loops met source steel or temporary datum
supports. Rounding those contacts would change their fit. Instead the exact box
algebra supplies separate rectangular board parts, with sealed contact joints.
No source casting radius or mating position is changed. Bed backing uses 33
single-board pieces; each bearing/drive seat uses four. Every machined edge and
faced contact surface is listed for sealing.

| Former feature | Applied source radius | Disposition |
| --- | --- | --- |
| `front-y-bearing-seat` | 0 mm | Preserve `driveYFloating` datum contact; four board pieces |
| `rear-y-drive-seat` | 0 mm | Preserve `driveYFixed` datum contact; four board pieces |
| `foot-access-LeftFront` | 0 mm | EPS; no routed rigid inside corner |
| `foot-access-LeftRear` | 0 mm | EPS; no routed rigid inside corner |
| `foot-access-RightFront` | 0 mm | EPS; no routed rigid inside corner |
| `foot-access-RightRear` | 0 mm | EPS; no routed rigid inside corner |
| Bed backing | Not a source feature | Preserve proud steel and datum contacts; separate board pieces |

Use `eps-supports.json` for each core's receiver, registration profiles and
restraint construction. Use `former-board-stock.csv` and `former-manufacturing.json` for stock, sealing
faces and program order. Programs retain the fixed G54 datum and spoilboard;
only through bands receive breakthrough. Machine, workholding, board/grip and
chemical compatibility remain unqualified.

The earlier totals changed from 889/402/123/74 to 626/568/119/0 (Pass/Pending/
Unsupported/Fail) because fewer anchors and no separate washers removed repeated
geometry checks, unqualified mix capacities became Pending, hardware gaps were
resolved, and the 74 real washer-annulus traps were removed geometrically.

The preceding rigid-access baseline had 625 Pass, 556 Pending, 136 Unsupported
and 7 Fail, with 370 artifacts and 67 review programs. Its seven former machining
findings are the regression baseline for the corner/EPS revision. The corner/EPS kit has 560 artifacts and 88 programs (40 panel, 48 former),
with **767 Pass / 615 Pending / 116 Unsupported / 0 Fail** and zero traps. All
48 former machining checks pass. The increase includes 62 drawing checks, 48
machining checks and 32 restraint checks for the separate parts; release
qualification remains Pending. Removing the four hollow bodies removes their
20 unsupported wall-model checks. Fabrication readiness remains false.

The release-system revision has 563 artifacts and the same 88 programs, with
**770 Pass / 620 Pending / 116 Unsupported / 0 Fail** and zero traps. The three additional Pass records cover release
coverage, protected bonding faces and preparation before assembly. Five Pending
records cover EPS safety and the four product/timing inputs; chemical coupon
evidence remains in the existing compatibility check. All 475 prior CAD, drawing
and CAM artifacts across the base, box and socket kits remain unchanged.
