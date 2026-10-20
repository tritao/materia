# Rail mounting plan

How profile rails are mounted accurately on our own castings and frames without a machine large
enough to finish them. It applies to every machine with rails: the bench mill, the sheet router, the
flexspline hobber and the lathe. Purchasing options for the instruments and compounds are in
[METROLOGY_SOURCING.md](METROLOGY_SOURCING.md).

Status: planned 2026-10-20. Nothing below is implemented.

## What exists

- **Plane by replication (decided).** The bench mill README ("Precision interfaces: DIY replication
  route", 2026-10-19) sets rail and screw-bearing seats by replication: steel seats stop below the
  final plane by an allowance, and a thin layer of replication compound is cast against a precision
  master. Both rails of one axis share one pour. The sheet router (SR-D2 in
  `examples/sheet-router/PLAN.md`) follows the same route on a welded frame and names a wire or laser
  for straightness.
- **Tooling model (planned).** `manufacturingkit/TOOLING_PLAN.md` phase 7 plans
  `InterfaceRequirement` (route, allowance, permitted finishes), `InterfaceRelationship` (coplanar,
  parallel, perpendicular, distance, with axis members), replication jigs and metrology with a stated
  uncertainty. `CastInsert` already offers the finish `replicated-after-casting`, and
  `CastingMoldPlanner` keeps a replicated seat pending until a master and release are named.
- **Rail data.** `LinearRailProfileSpec` (`motion/LinearRailProfile.hx`) carries envelopes, hole
  pitches, screws, an assumed static capacity and an optional preload target.

## What is missing

- **The edge.** Replication fixes height, flatness and twist. Nothing sets or holds a rail's lateral
  straightness, the parallelism of the second rail or the squareness between axes. Rails are modelled
  as bolted down vertically only: no reference side, no reference shoulder or bar, no lateral clamp.
- **Rail tolerances.** The spec has no reference side, no permitted parallelism or height difference
  between paired rails (which depend on preload class), no screw torques and no thread depths.
- **Lateral metrology.** `FoundationIntegrity` compares face normals only, so a seat rotated about its
  normal (exactly a straightness or parallelism error) is invisible.
- **Procedure and evidence.** There is no assembly sequence, no acceptance check for the instruments,
  and nowhere to record a measured profile against the relationship it satisfies.

## Decisions

- **RM-D1. Replicated plane, adjusted edge.** Each error is set by one feature:

  | Error | Set by | Method |
  | --- | --- | --- |
  | Flatness along a rail, twist, coplanarity of a pair | Mounting plane | Replication against one master per axis |
  | Straightness of the master rail | Lateral reference | Aligned at assembly to a straight reference, then locked |
  | Parallelism of the follower rail | Master rail | Indicator on a block riding the master, or a gauge bar |
  | Squareness between axes (in the rail plane) | Lateral adjustment of one rail | Indicated against a granite square while the other axis moves |
  | Column squareness out of the rail plane | Column joint | Jack screws and grout (bench mill README) |
  | Coplanarity of the block pads on a moving member | Saddle, carriage, table | Machined in one setup; these parts are small |

  We do not cast, replicate or machine a precision shoulder as the primary reference. A replicated
  shoulder would fix squareness at pour time, would need a vertical master inside a dam, and would put
  clamp load into a thin epoxy lip. Adjusting at assembly takes insert placement error and cure
  distortion out of the result and can be repeated if a rail is ever re-seated.
- **RM-D2. Steel side bar with push screws.** Each rail gets a steel side bar cast in beside its seat,
  on the rail's non-reference side, from the same insert family as the seat. Push screws in the bar,
  at the rail-bolt pitch, press the rail edge (HIWIN's "retaining screw" method). The bar needs only
  coarse placement from the mold fixture. A tapered-gib slot is an alternative clamp where speed of
  assembly matters. Clamp loads go into steel, never into casting or compound.
- **RM-D3. Optional permanent edge.** After alignment, the gap between rail and side bar may be filled
  with replication compound against the release-coated rail, so the rail is the master for its own
  full-length shoulder. TOOLING_PLAN already permits "the real rail as master". This is a lock, not
  the reference.
- **RM-D4. Assembly sequence.** Snug the rail bolts; push the master rail straight with the push
  screws while indicating against the reference, working from the middle outwards; set the follower
  parallel to the master; torque the rail bolts in sequence to the catalogue value; witness-mark every
  screw. Squareness to the other axis is set on the master rail's push screws before torque.
- **RM-D5. References by length.** Up to about 1 m: a granite straightedge, with reversal. Longer
  (sheet router): a tensioned wire or a laser line read by a camera or position-sensing detector, as
  SR-D2 states. A wire sags vertically, so it only serves lateral straightness; the replicated plane
  covers vertical.
- **RM-D6. Reversal before trust.** A straightedge, square or level is accepted by reversal, not by its
  certificate alone: indicate, turn the reference end-for-end or through 180°, indicate again, and
  separate the reference's error from the measured part. This is mandatory for instruments bought
  without a traceable calibration (most Chinese metrology) and good practice for all.
- **RM-D7. Measurements are records.** Every alignment produces a measured profile with the instrument,
  method and uncertainty, attached as evidence to the `InterfaceRelationship` it satisfies. A
  relationship whose measurement uncertainty exceeds its tolerance stays pending (TOOLING_PLAN rule).
- **RM-D8. Compound with a datasheet.** The replication compound must publish minimum and maximum
  layer thickness, modulus, cure shrinkage and creep under sustained load, because the replication
  allowance is derived from them. First choice is Moglice (Diamant Polymer, Germany); SKC (CKB,
  Japan) is the alternative. China uses the same technique, but no documented Chinese equivalent was
  found (2026-10-20); a Chinese compound may be trialled only beside Moglice in the coupon. Rail seats
  are static, so a sliding (MoS2) grade is not required; ask Diamant for their static load-bearing
  replication grade.
- **RM-D9. Rail bolt holes come from the rail.** The rail owns its hole pattern (`LinearRail`
  `holePositions` and its `mount1…mountN` connectors, from pitch and end margin). The rail mount
  derives the seat insert's tapped holes from the placed rail: each hole frame is the rail's mount
  connector expressed in insert coordinates, with the thread from `railMountScrew` and depth and
  limits from `ThreadedHolePolicy`. The side bar's push-screw holes derive from the same positions.
  No hole position is entered by hand, and an assembly check requires every rail screw to be coaxial
  with a seat hole. On the insert drawing the holes are dimensioned from the insert's own datum edge
  and end face, the faces its mold fixture locates on.
- **RM-D10. Drill before casting only when the clearance budget closes.** Under RM-D1 the rail is
  moved laterally at assembly, so its aligned line departs from the line of its seat holes by the
  insert's in-plane placement error over the rail length (rotation and offset relative to the master
  rail or the other axis), its bow from cure distortion, and the hole machining tolerance. Along the
  rail only pitch accumulation counts, since the rail slides to the holes. If that departure fits
  inside the rail's bolt clearance with margin, the holes are drilled and tapped before casting and
  protected by bore masks during the pour and replication. Otherwise the hole route is **transfer
  drilling after alignment**: align and clamp the rail with the bolts out, drill through the rail's
  own counterbored holes with a guide bushing, tap with a tap guide, and protect the casting and
  compound from swarf; the operation sheet lists bushing, tap guide and protection. A tighter insert
  fixture is the other remedy. The model computes the budget per hole and chooses only routes the
  interface requirement permits; when none closes, it raises a design change request.

## Accuracy budgets (inputs)

Budgets are per machine and stay explicit inputs until set. Each row becomes an
`InterfaceRelationship` tolerance.

| Machine | Master rail straightness | Pair parallelism | Pair height difference | Axis squareness |
| --- | --- | --- | --- | --- |
| Bench mill | input (µm per 300 mm) | catalogue limit for the preload class, or tighter | catalogue limit | input (µm per 300 mm) |
| Sheet router | input (µm per m) | catalogue limit | catalogue limit | input |
| Flexspline hobber | from its PLAN error budget | catalogue limit | catalogue limit | from its PLAN |
| Lathe | input | catalogue limit | catalogue limit | input |

Rail catalogue limits (permitted parallelism and height difference between paired rails by preload
class) are taken from the supplier for the rail actually bought and recorded with their source.

## Metrology

| Job | Instrument | Notes |
| --- | --- | --- |
| Lateral straightness, parallelism | Two logging digital indicators (1 µm) on a bracket bolted to a rail block, read against a straightedge or the master rail | The core kit; data output feeds the alignment assistant |
| Squareness | Granite square, class 00, with the indicator on the moving axis | Also the reference for grouting the column |
| Twist, flatness of the replicated plane | Electronic level (1 µm/m class), stepped one block length at a time | Gravity only; cannot see lateral straightness |
| Long axes | Tensioned wire or laser with a camera or position-sensing detector on a block | Air movement limits a laser to several µm over metres |
| Quick manual checks | Lever test indicator | |
| Acceptance of the assembled machine | Ballbar | Squareness, backlash, servo mismatch; measures the result, does not align |
| Certification, once | Laser interferometer with straightness optics | Rent or hire a calibration service |

Once one axis moves under its own drive, it carries the indicator: logging against axis position
measures the other rails' straightness and squareness without a separate carriage.

### Alignment assistant

A small tool in our stack that:

- reads the indicator or level stream (USB HID, Bluetooth or RS232, depending on the instrument);
- knows the rail-mount geometry from the machine model: push-screw positions, reference side, rail
  length;
- applies reversal and a best-fit line, and shows the deviation and correction at each push screw;
- repeats measure, adjust, re-measure until the relationship's tolerance is met;
- saves the final profile as evidence on the relationship (RM-D7).

The same reversal routines accept the instruments on arrival (RM-D6), so they are built first.

## Phases

### RM0 — Inputs and procurement

Set the accuracy budgets above. Choose the replication compound and obtain its datasheet. Order the
core metrology kit (METROLOGY_SOURCING.md, "first order").

Acceptance: budgets recorded in this file; compound datasheet values recorded with source.

### RM1 — Rail specification

`LinearRailProfileSpec` gains: reference side, permitted pair parallelism and height difference per
preload class, rail and block screw torques, block thread depth, the rail's bolt-hole and counterbore
diameters (the clearance RM-D10 budgets against), and its hole pitch tolerance. Values are labelled as catalogue
data with source or as assumptions.

Acceptance: every profile in `LinearRailSystem` either carries sourced values or is labelled
assumed; the bench mill's 16 M4 × 16 block screws are checked against the recorded thread depth.

### RM2 — Shared rail-mount family

A rail mount built once and used by every machine: a seat insert with a replication allowance
(derived from compound minimum layer + fixture height error + cure distortion), a steel side bar on
the non-reference side, and an edge clamp chosen from push screws, tapered gib or replicated edge.
Rail and side-bar holes go through `InsertHoles`/`DesignInterfaces`, derived from the rail's mount
connectors (RM-D9), with the drilling route chosen by the clearance budget (RM-D10). The mount
declares its `InterfaceRequirement` and the relationships it takes part in.

Acceptance: a pair of rails on a test foundation builds with seats, side bars and clamps; the
allowance appears in the casting geometry; clamp screws engage steel only; a missing side bar or a
clamp bearing on casting is rejected. Changing rail length, pitch or end margin moves the seat and
push-screw holes with no other edit; a seat hole offset from its rail screw fails the coaxial check;
a placement tolerance that exceeds the bolt clearance switches the route to transfer drilling, and
with transfer drilling not permitted it raises a design change request.

### RM3 — Bench mill adoption

`MillFrame` uses RM2 for the Y rails and the column's Z rails; the saddle carries the X rails the same
way. Seats get a real replication allowance (today they are flush with the casting: review finding
B8). `FoundationIntegrity` detects rotation about the face normal, with a negative control that rotates
a seat in plane.

Acceptance: the mill rebuilds with the new mounts; the in-plane rotation control fails as expected;
FEM packet includes replication layer solids.

### RM4 — Lateral relationships

In-plane `InterfaceRelationship` kinds (straightness of a master edge, parallelism of a follower,
squareness between axes in the rail plane) with route "adjusted at assembly", instrument, method and
uncertainty. Coordinated with TOOLING_PLAN phase 7 step 2, which owns the relationship records.

Acceptance: the bench mill declares all its rail relationships; one with uncertainty above tolerance
stays pending.

### RM5 — Alignment assistant

The tool described above, starting with instrument acceptance by reversal, then guided rail
alignment.

Acceptance: a recorded straightedge reversal separates a synthetic straightedge error from a
synthetic rail error within the stated uncertainty; a live session against one real instrument
produces an evidence record.

### RM6 — Coupon trial and procedures

On a scrap casting or steel coupon about 1 m long: replicate a pad, mount a real rail, align it with
push screws, measure straightness, torque, measure again, and measure after one week for creep.
Record layer thickness achieved, flatness transfer, bond to the steel seat and edge chipping. Write the
procedures (replication, rail alignment, torque sequence) into the fabrication package. The sheet
router then adopts RM2.

Acceptance: measured values replace the assumptions in the replication allowance and the procedures
reference the evidence. TOOLING_PLAN phase 9 carries the same trial.

## Open questions for the compound supplier

Ask Diamant (and SKC if compared):

1. Which grade for static, load-bearing replication under rail bolt preload (not a sliding way)?
2. Minimum and maximum layer thickness, and the gap tolerance for injection.
3. Compressive modulus and strength; creep under sustained compressive load.
4. Cure shrinkage and the recommended cure temperature and time.
5. Bond preparation on steel; recommended release agent for granite and steel masters.
6. Edge support and dam materials; hole plugging.
7. Smallest trial kit and lead time within the EU.
