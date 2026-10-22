# Sourced gear-rack sections

`GearRack` is the catalogue recipe; `GearRackCatalog` holds reviewed ATLANTA HPR
quality-6 heat-treatable-steel rows. Existing `Rack`/`HelicalRack` remain generic
engineering drawings, with explicitly assumed drive compliance. The frozen
router still uses its engineering rack and is not changed by this addition.

[ATLANTA's catalogue](https://www.motiontech.com.au/wp-content/uploads/2026/01/Atlanta-Servo-Drive-Systems.pdf)
C-16 supplies section geometry, holes, pin pilots and mass. C-44/C-45 supply
feed-force ratings by pinion tooth count. The retained PDF hash and field/page
mapping are in [provenance.json](provenance.json).

| Order code | Module | Length mm | Face width mm | Mass kg | Published force with 20-tooth pinion N |
|---|---:|---:|---:|---:|---:|
| 29-15-055 | 1.5 | 500 | 19 | 1.30 | 3000 |
| 29-15-105 | 1.5 | 1000 | 19 | 2.60 | 6000 |
| 29-20-055 | 2 | 500 | 24 | 2.10 | 5750 |
| 29-20-105 | 2 | 1000 | 24 | 4.10 | 11500 |

The 500 mm rows include the supplier's 50% screw-connection derating. Module-2
with 15 teeth is 8000 N on a 1 m section, from the standard heat-treatable HPR
column, not the higher case-hardened column. The force is a published maximum
under the stated supplier material, alignment, lubrication, speed and unity
factor conditions. Apply application/lifetime/load-distribution/safety factors
and separately check mounting screws. It is not an automatic rating for our
unverified pinion. Unreviewed tooth-count ratings throw rather than interpolate.

Face widths above are nominal; C-16 specifies +0.4 mm tolerance, retained in
the catalogue row for the mounting clearance budget.

The supplier's 19 mm face does not justify assigning its rating or mass to the
frozen router's assumed 20 mm rack. Selecting it for the router changes geometry
and remains subject to the user's approval and the shared RM2 mount.

The reviewed racks are right-handed and need a left-handed pinion. The generic
pinion default is right-handed; the relation rejects that incompatible pairing.
Published capacity remains separate from assumed mesh stiffness.

CAD axes match the generic helical rack. `back` and `referenceSide` identify
datums. `mount1…N` expose the eight/four supplier bores (7 mm through, 11 mm
counterbore, 7 mm deep) at 62.5 mm end margin and 125 mm pitch. `pinPilot1/2`
identify the published 5.7 mm pilots at 31.7 mm from the ends; these are not
invented finished 6 mm dowel bores. Final reaming/locking follows the assembly
procedure. Host seat holes derive from these connectors using the shared
`InsertMountPattern` route; this recipe creates no support, lip or epoxy layer.

The tooth drawing is nominal, including square-ended layout geometry. Mass is
the supplier's mass; the centroid/inertia use a labelled box approximation.
Helix angle derives from the published module, length and tooth count. Generic
mesh stiffness, backlash and efficiency remain assumptions in the engineering
relation. No catalogue claim is made for them.

Validation: `GearRackChecks` covers all four sections, saved-recipe reload,
declared mass, hole/pin connectors, datum normals, valid/transformed drilled
solids, capacity-column selection, short-section derating and refusal of an
unreviewed rating. Run with `haxeon run --project=machinekit/examples/supplier-racks/haxeon.json`
under the heavy-run gate and existing OCCT provider. Connector-derived seat
holes are RM2/phase-7 work, not duplicated here.
