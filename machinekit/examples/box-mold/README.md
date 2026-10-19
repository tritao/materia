# Single-pocket box mold

Phase 3 geometry review, independent of the mill. Run from the repository root:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/box-mold/haxeon.json
# regenerate from measured stock, with the source turned 90 degrees in the bed:
./haxeon/scripts/haxeon run --project=machinekit/examples/box-mold/haxeon.json -- --thickness=17.8 --quarter-turn
python3 machinekit/examples/box-mold/render_review.py
```

Outputs are under `build/review/` (ignored): assembled/exploded `.mtrg`, portable
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

A 0.1 mm covering allowance on each submerged EPS face is included in cavity
size. The stock cutting definition is a rectangular block with a +Y wire spanning
its depth and the finished foam solid in local stock coordinates. It is ruled by
construction; no kerf compensation is added. The upper surface remains accessible
for breakout. A qualified broad-area restraint and its attachment loads remain
pending; the scene does not claim to provide that restraint.

The template explicitly selects an M6 fixing screw, 7 mm bed clearance hole,
5 mm nominal tap-drill envelope and resin-facing bore plug. Thread tapping,
engagement, plug/seal performance and independent insert datum accuracy require
physical/design evidence. Finishing allowance is `null` pending a decision;
source insert geometry is unchanged. The bolt secures the insert from below;
board flatness does not qualify its alignment. EPS and coated board CAD materials
carry nominal display/BOM densities only; supplier limits remain in stock records.

Validation checks all named contact cuts, exact seal-margin containment, closed
boundary coverage, unintended solid overlaps and casting symmetric difference.
Assembly sweeps include the covered core and insert; demolding sweeps include the
cured casting with steel, after fixing/closure release and sacrificial EPS breakup.
Each order comes from full straight-translation swept volumes. Hardware thread
release and core breakup are explicit prerequisites, not simulated motions.

The initial template requires one rectangular envelope, one open-top EPS pocket,
one bed-facing insert and an orthogonal setup. Unsupported configurations fail.
Library code contains no mill dimensions or feature-name cases. Tests cover two
stock thicknesses, two orientations, recipe restoration, source CAD equivalence,
invalid cutting definitions, unclassified holes, insufficient seal lands and
trapped walls. Resin compatibility, cure temperature, compaction, core restraint,
insert anchorage, panel/closure strength and handling remain pending. This package
is for review and is not fabrication-ready; those checks belong to later phases.
