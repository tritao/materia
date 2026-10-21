# SR12 true-shape nesting

Run `machinekit/examples/sheet-router/nesting/check.sh` from the repository root.
It compiles Haxe only and reuses the existing CAD provider/OCCT build. The final
run passes **37 nesting assertions** and **54 legacy sheet-cutting assertions**.
The machine-readable results, layouts and tested source hashes are in
[validation.json](validation.json).

| Bed | Cabinet parts | Sheets | Guillotine candidate | Material utilisation | Nesting time |
|---|---:|---:|---:|---:|---:|
| 1600×2600 mm | 6 panels + 6 small parts | 1 | 1 | 76.08% | 40.73 ms |
| 1600×3150 mm | 8 panels + 6 small parts | 1 | 1 | 83.63% | 53.53 ms |

Panels are 700×750 mm and small parts 50×50 mm. The tool diameter is 6 mm, extra
gap 1 mm and edge margin 10 mm. These are geometric planning inputs; SR13 selects
actual tooling/forces. Both cabinet results use the no-fit solver. Times are local
`CamSheetNester.nest` wall times, including validation and remnant extraction,
using the prebuilt provider on this host. They are observations, not a throughput
guarantee. A second run with the same seed produces identical complete plans.

Review the actual exported layouts:

- [2600 mm cabinet sheet](cabinet-2600.svg)
- [3150 mm cabinet sheet](cabinet-3150.svg)
- [Two interlocking L shapes](interlocking-l.svg): their 80×80 mm bounding boxes
  overlap on a 105×105 mm sheet, while their material polygons remain disjoint
  with 5 mm clearance.
- [Part inside a real hole](hole-nest.svg): an 80×80 mm plug nests inside the
  120×120 mm void of a 200×200 mm frame, with 7 mm clearance. Green rectangles
  show conservative reusable remnant regions.

## Geometry and search

`CamSheetNester.nest(requirements, stock, options)` accepts several stock
specifications with available quantities. Requirements carry an optional polygon
outline (outer boundary then holes); absent outlines use their declared rectangular
blank. Inputs are captured as normalized mm requirements in the result, which must
be kept with the generated plans for validation and serialization.

Material is decomposed into convex pieces. Convex parts use their boundary directly;
concave parts and parts with holes use the existing planar-face triangulator, with
a material-area conservation check. For placed material A and a moving part B, the
forbidden translations are `A ⊕ (−B)`. Distributing that Minkowski sum over the
convex pieces represents the no-fit set as a union of convex components. Each
component receives a rounded offset of `toolDiameter + gap`, using analytic lines
and circular arcs. Component endpoints, arc extrema, component intersections and
sheet-boundary intersections supply contact candidates, including cavities caused
by holes.

The deterministic search places larger material areas first and chooses a
bottom-left feasible contact for each part. A seed resolves equal-area instance
ordering. Rotations default to 0/90/180/270 degrees; `rotationStepDegrees` or an
explicit `allowedAngles` list permits other angles. Positive free-form angles are
**counterclockwise**, unlike the legacy rectangular placement's clockwise quarter
turn field. `width-aligned` locks retain angle zero. Explicit angle lists are the
free-form policy; the legacy `allowedRotations` field is retained for version-1
compatibility.

A guillotine partition of rectangular inputs is also evaluated. The lower-source-
area complete candidate is retained, so rectangular batches cannot have worse
material utilisation than that baseline. This is a deterministic placement
heuristic, not an optimal-packing proof. Failure means no complete layout was found
for the supplied sheets, rotation policy and search order. It never returns a
silently incomplete quantity. Limits are 1000 part instances, 1000 source sheets,
1..100 copies per stock entry and 4096 polygon edges per shape.

## Independent checks and precision

The final placement check does not use no-fit membership or bounding-box overlap
as its acceptance test. It verifies source/requirement fingerprints, material and
thickness, captured poses, allowed angles/grain, edge margin, polygon-edge
intersections, material containment including holes, and analytic segment distance
for the full tool/gap allowance. Batch validation checks exact quantities and
unique identities across sheets. Deliberately overlapping, insufficiently spaced,
out-of-margin and stale layouts are rejected by this path.

Geometry is on a **0.001 mm grid** within a finite ±10 m coordinate domain.
Orientation/intersection predicates use integer-coordinate products represented
exactly within that domain; clearance comparisons use a stated **1e−7 mm** numeric
tolerance. Arbitrary-angle vertices are quantized once on rotation, then translated
on the same grid. Candidate grid neighbours around analytic contacts handle
round-off at a contact. This is polygonal nesting: DXF arcs/splines must first be
converted to polygon outlines at a declared chord tolerance, and that conversion
allowance belongs in the user's gap/geometry budget. The 0.001 mm grid is separate
from the machine's 0.05 mm motion-error allowance.

Remaining material is screened for conservative rectangular remnants using at most
64 coordinate lines per axis, then adjacent empty cells are merged. Returned
rectangles meet the requested minimum side and independently clear all parts and
cutting allowances. They are a reusable subset of the remainder, not a claim to
recover every concave offcut. Layout area accounting separates parts, retained
rectangles and all remaining unretained area. Kerf versus other discarded material
is assigned only when the actual cut sequence exists.

## Plan and CAM integration

`SheetCutPlan.schemaVersion=2`, `layout="freeform"` stores polygon placements,
angles, a batch/sheet identity, gap and remnant rectangles. Legacy guillotine
placements/cuts/leaf-region selections are empty. The version-1 format and its
fingerprints remain compatible: the old implementation at `b38630144` and the
current one both produce the recorded golden fingerprint. New polygon and angle
snapshots participate in staleness detection. The project codec round-trips the
new data; SVG/CSV exporters preserve the polygon geometry.

`CamSheetProfiles.freePart` maps a placed outer boundary and all holes into SI
contours for SR11. Nesting is a layout stage: SR11's actual native cutting sequence
and SR8 admission checks still decide which cuts/releases are acceptable. Close
neighbouring tool sweeps can affect retained links, so tab retention must be
checked on that resulting sequence, rather than inferred from layout spacing.
The legacy rectangular inventory executor explicitly refuses a free-form layout
until a qualified CNC execution/output mapping exists; allocation and preview work
without consuming stock. SR13/commissioning, physical tool reach and MotionKit
OM-R1/SR16 acceptance gates stay in force. No router structure was changed.

Retained packet: `build/sheet-router-evidence/sr12-nesting.tar.gz`, SHA-256
`a808f053601f7b93854bf6e97014f412f9dd95ae726296c3164c9b6b2114e57e`; regular-file member hashes verified.
