# Single-pocket box mold

Phase 3 geometry review, independent of the mill. Run from the repository root:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/box-mold/haxeon.json
# regenerate from measured stock, with the source turned 90 degrees in the bed:
./haxeon/scripts/haxeon run --project=machinekit/examples/box-mold/haxeon.json -- --thickness=17.8 --quarter-turn
python3 machinekit/examples/box-mold/render_review.py
# Separate source variant with a cavity opening onto the bed:
./haxeon/scripts/haxeon run --project=machinekit/examples/box-mold/haxeon.json -- --bed-core
python3 machinekit/examples/box-mold/render_review.py --bed-core
```

Outputs are under `build/review/` or `build/review-bed/` (ignored): a single `mold.mtrg` with an Explode slider, portable
assembly recipes, the versioned mold record and geometric review, local panel
STEP/SVG/DXF/operation records, local EPS STEP/ruled cutting definition/drawings,
reconstructed casting STEP and an ordered procedure. The optional PNG renderer
uses NumPy and Matplotlib and renders exported CAD triangles.

`SinglePocketSetup` constructs a 240 x 180 x 100 mm foundation with a nominal
100 x 80 x 60 mm open pocket and one 40 x 35 x 15 mm steel insert. The adapter
supplies every casting dimension and source feature ID. The inverted setup puts
the insert mounting face on the bed and the pocket at the open fill face.

The shared ManufacturingKit template creates a bed and four independently
removable walls. A rebate on each wall's lower inner face leaves an outer tongue;
blind bed sockets locate those tongues. Socket fit clearance and exact router
reliefs stay behind the continuous inner contact lands. External closure clamps
remain a required process input; no clamp load qualification is implied. The
2 mm minimum ligament is a geometry study value, not an approved board design.

Core support is selected from the actual cavity opening. A bed opening selects
`bed-socket`; otherwise a fill-face opening selects `bridge`. A closed internal
cavity is rejected. `BoxMoldPlanner.plan(document, options)` also accepts explicit
`CoreSupportOptions` from `CoreSupports.defaults(recipe)`. Forcing a bed socket
through a solid casting floor or a bridge through a solid top fails.

For a suspended core, four reusable panels form two registered wall-top pads,
a broad carrier and a spanning bridge. Long walls have a half-stock-height
extension above the fill plane for the pad sockets; the other walls stop at the
fill plane. The pads register over the long wall tops, so X/Y and height come
from physical seating surfaces. One round pin and one slot locate the bridge;
separate M6 bolts, washers and nuts clamp it to the pads outside the resin region.
The pads still require external clamps to the walls; those clamps are a declared
closure requirement, not a modeled or qualified clamp product. The carrier is
permanently bonded to the bridge and the pair moves as one rigid group during
demolding. Before installation, bond the EPS to this fixture and install the
complete hanging unit together; the assembly graph checks this larger group. Bond the EPS top to this carrier across its full area. The declared
0.2 mm foam/carrier adhesive line is deducted from the foam height. Release this
bond before removing the fixture; do not split the permanent wooden fixture.

For a bed-open core, route a blind socket entirely inside the cavity footprint.
A smaller EPS locating foot is bonded underneath the main foam block and glued
into this socket. Its inset keeps fit clearance and router reliefs behind a
continuous sealing footprint. The 6 mm default embed depth includes two explicit
0.2 mm adhesive lines (foot-to-bed and foot-to-main-core), leaving a 5.6 mm EPS
foot at 18 mm stock. The main core starts at the bed plane: no solid casting floor
is removed to make room for it. After release of the bed bond, the bed withdraws
before the foot/core is withdrawn or broken out through the bed opening.

A 0.1 mm covering allowance on submerged EPS faces is included in cavity size.
Main blocks and separate locating feet have ruled +Y stock cutting definitions,
STEP and SVG/DXF drawings, without kerf compensation. Adhesive references, bond
areas, clearances, embed depth, pad closure and support members are persisted in
the mold record. `BoxMoldCodec` version 2 includes support options and rejects
version 1; changes invalidate the package fingerprint. Geometry is supplied, but
adhesive compatibility/strength, EPS bearing/compression, uplift resistance,
bridge/pad strength and external clamp capacity remain unqualified.

The template explicitly selects an M6 fixing screw, 7 mm bed clearance hole,
5 mm nominal tap-drill envelope and resin-facing bore plug. Thread tapping,
engagement, plug/seal performance and independent insert datum accuracy require
physical/design evidence. Finishing allowance is `null` pending a decision;
source insert geometry is unchanged. The bolt secures the insert from below;
board flatness does not qualify its alignment. EPS and coated board CAD materials
carry nominal display/BOM densities only; supplier limits remain in stock records.

Validation checks all named contact cuts, exact seal-margin containment, closed
boundary coverage, unintended solid overlaps and casting symmetric difference.
Assembly sweeps include the covered core, supports and insert; demolding sweeps include the
cured casting with steel, after fixing/closure/bond release. The EPS core remains in the graph with the appropriate withdrawal direction; its whole-block clearance also establishes an available breakup opening.
Each order comes from full straight-translation swept volumes. Hardware thread/bond release and core breakup are explicit manual operations, not simulated motions.

The initial template requires one rectangular envelope, one bed-open or fill-open EPS pocket,
one bed-facing insert and an orthogonal setup. Unsupported configurations fail.
Library code contains no mill dimensions or feature-name cases. Tests cover two
stock thicknesses, two orientations, recipe restoration, source CAD equivalence,
both support modes, invalid support choices/cutting definitions, unclassified holes, insufficient seal lands and
trapped walls. Resin compatibility, cure temperature, compaction, core restraint,
insert anchorage, panel/closure strength and handling remain pending. This package
is for review and is not fabrication-ready; those checks belong to later phases.
