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
Dam geometry and pour groups are deferred to their actual phase 5 CAD/checks;
no unused fields imply they have been implemented. The column joint has a separate pending grout-gap, jacking, sleeve and
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
and load/access checks; phase 8 supplies physical evidence.
