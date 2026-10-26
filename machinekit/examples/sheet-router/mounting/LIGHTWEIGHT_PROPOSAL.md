# Lightweight rack/pinion mount proposal — 2026-10-26

Review before construction: no frozen CAD or driveRack API edit is authorized by
this note. Retain the frame, braces, module-1.5 transmission and existing axis
datums. This supersedes the heavy continuous carrier as the design direction,
not the qualified local fixture evidence in PROPOSAL.md.

## Layout and conflict removal

Keep the sourced rack side-bolted through its own mount connectors to a continuous
C45 receiver, initially 20 mm thick × 22 mm high. Use short discrete steel cleats
instead of the continuous 44.9 mm foot and 140 mm pad/replication strip. Reuse the
existing relieved host pads where bearing and bolt access permit; replicate only
the supported narrow datum. Adjustment is by a removable side bar/push screws,
then clamp and lock, with section pitch set by a joining gauge. No fixed locating
lip, invented rack holes or new rack-specific mount family is proposed.

Five host anchors per 500 mm section retain the three original stations and
end stations at ±230 mm. A provisional cleat envelope is 60 × 40 × 6 mm, subject
to bolt engagement, bearing and wrench checks. Section joints must permit end
anchors on both sides without an unsupported tooth span. Retain real washers,
steel clamp threads, captive nuts and frame access openings; do not drill a tube
wall until its local bearing/net-section check passes. The X beam needs its own
host/cleat layout; Y dimensions are not automatically valid there.

Replace the drawn Y preload hinge: with world Y as travel and Z radial, use a
hinge axis parallel to X and a hinge-to-pinion arm parallel to Y, so axis × arm
is radial Z. Put the pivot and spring above/outboard of the receiver, with a
forked carrier straddling the pinion axial centre rather than running through
the receiver foot. Both Y racks lie below their pinions; mirror the axial layout,
not the mesh-side datum. Preserve the existing pitch height. Spring force, arm
ratio, radial travel and backlash must be checked together; do not assume a ×4
spring improvement from a stiffer plate. Initial sizing retains original radial
compliance and reducer properties, with no stiffness-upgrade credit.

The 16 blocking pairs per bed comprise four preload/profile pairs and twelve
switch/pad-or-compound pairs. Removing the continuous foot and wide pad clears
their proposed occupied volumes by design intent. The new fork/pivot must sit
outside the narrow receiver/cleat swept envelope. Move all six home/limit switch
brackets to the opposite accessible host face, using offset flags on the moving
carriage; preserve sensing positions, overtravel and independent limit function.
Do not simply omit supports at a switch. Existing chain-bracket pairs were native
false alarms; preserve their clearance and service route.

These are explicit redesign actions, not a native clearance pass. Before landing,
check all 16 former pairs, new cleats/fasteners/forks/flags, both beds and full
axis/overtravel sweeps. Require positive clearance under tolerance and preload
travel, plus nut capture, tool approach, cassette removal and chain articulation.
The final minimum clearance must include a documented tolerance budget; the old
local 1.10 mm clearance is not a full-machine allowance.

## Preliminary mass and model prediction

Gross steel density is assumed 7850 kg/m³. Receiver stock is 3.454 kg/m;
ten provisional cleats/m add 1.1304 kg/m. A 22 × 4 mm replication strip at assumed
1800 kg/m³ adds 0.1584 kg/m: **4.7428 kg/m** before holes, joints and hardware.
Using the old rack envelopes solely for estimation gives **9.72 kg moving Y**
(2.05 m X support) and **31.78/36.99 kg fixed** (two 3.35/3.90 m Y supports).
Allow a provisional additional **2 kg moving Y / 2 kg fixed** for net fork,
fastener and switch changes: design budgets **11.72 kg moving**, **33.78/38.99 kg
fixed**. Existing pad/rack/drive mass is retained; no removal credit is taken.
These are stock budgets, not a BOM. Actual catalogue section coverage, supports,
net removed material and both axis inertias must replace them before acceptance.

This cuts the moving-stock budget to about one third of the studied +34.76 kg
penalty. It does not establish an error by interpolating between modal results.
The old end-anchor stiffness of 20.49 MN/m per pinion belongs to a different
continuous-foot section; discrete cleat bending, bolt uplift, receiver torsion,
compound compression and host flexibility must be recalculated. The prior
original-drive/end-anchor screen was 0.08573/0.11919 mm braking error; even the
mass-neutral strongest drive bound missed 0.05 mm. Mechanical changes alone are
therefore not the acceptance route. The mass-loaded offline k=0.6 screen reached
0.02717/0.03811 mm at governing poses, supporting a reduced-stop policy study,
not a numerical prediction for this new mount. New executed histories and the
actual mount model must demonstrate the target together.

## Approval and implementation boundary

Request approval of this layout direction before building it. After approval,
build/validate the real shared mount, then land it with the required-mount
driveRack/rackPlacement contract and every caller in one change, as RM2 records.
No optional unmounted overload. Recheck contact load paths and orientation from
mesh plus declared host face; remove the bounding-box side heuristic. Keep frame
and braces unchanged. Fabrication release still requires native clearances,
actual mass/compliance, structural replay, weld checks and commissioning.


## Added compliance screen and outstanding prediction — 2026-10-26

The [explicit component calculation](lightweight-compliance-estimate.json) uses
this proposed receiver, 60 × 40 × 6 mm cleats, a 20 mm unsupported cleat arm,
two M6 host and two M6 receiver bolts per cleat, assumed 18/20 mm bolt grips and
125 mm maximum anchor span. With steel E=210 GPa and Poisson ratio 0.3, the
projected component compliance is **0.02819 µm/N**, equivalent to **35.48 MN/m**.
It sums receiver axial extension, cleat shear/bending, bolt shear and radial
bolt uplift under pressure-angle/helical force components. Values are derived
from the new dimensions; the previous 20.49 MN/m is not reused.

**This is an incomplete component sum, not the mount stiffness.** Receiver
bending/torsion, drilled net section, bolt bearing/slip, cleat prying, contact
opening, host wall/pad deformation, replication contact, section joints and
fork/pivot compliance remain to be resolved. Do not insert 35.48 MN/m in the
structural model. In particular, a 6 mm cleat and two M6 fasteners are proposed
geometry, not a demonstrated adequate connection. The moving-mass budget must
also include the newly explicit four fasteners per cleat before it becomes a
BOM; the 2 kg hardware allowance is still a budget, not verified coverage.

| Required prediction for this mount | Current evidence |
|---|---|
| Nominal executed stop, both beds | Not yet calculated on this mount; prior-model numbers are not transferable |
| Stop A/J fractions 0.64/0.512 | New executed stop-only histories pending; uniform k=0.8 is not an equivalent history |
| Stop A/J fractions 0.36/0.216 | New executed stop-only histories pending; uniform k=0.6 is not an equivalent history |

Consequently the present note does **not** establish that this mount alone meets
0.05 mm, or that the mount plus either requested stop policy meets it. The design
intent is a lightweight mount plus reduced stop; approval of that direction is
not a measured or simulated pass. Complete the load path/compliance and explicit
mass model before presenting numerical predictions for this geometry. Until the
new exports arrive, any policy calculation must be labelled a counterfactual
screen and cannot close acceptance. No fabricated interpolation from old mass
or stiffness results is offered as a prediction.

## One final structural verification, after implementation

After approval and landing the real mount plus all-caller driveRack migration,
and after the motion session supplies the reduced-stop executed histories,
perform **one final verification campaign on that final geometry**, closing:

1. **SR5:** braking-following ≤0.05 mm on both beds, reporting the 0.04 mm reserve,
   geometric contour and physical-rest residual separately.
2. **SR6:** recompute the dynamic stress envelope from the executed J7 histories,
   including the changed mount load path, and requalify all affected weld groups
   at 100 million full-load reversals. Explicitly replace the old J4 W066 result
   of 13.03 MPa against 12.67 MPa; no reduced-load or improved-detail credit without
   checking the actual final connection and applicable fatigue category.

Do not run a separate SR6 requalification on the frozen frame. Retain both beds,
mesh/pose coverage and modal-history provenance, with the approved serial router
worker/resource limits. SR13/SR16 remain held on OM-R1 plus SR6-C; other hardware
and workshop gates remain in force. Stop at this note; nothing else is unblocked.
