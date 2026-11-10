# Flexspline cup: axial datum options (closed-end datum vs sacrificial collar)

Study only; the model and the cup recipe are unchanged. Reproduce with
`python3 datum_study.py <supplier-package.json> [keepout.svg]` (stdlib; the JSON comes from
`supplier-packages/SupplierPackageExport.hx`, see its README; the script imports `keepout()` from
`supplier-packages/mandrel_package.py`, so the keep-out is the one the mandrel supplier is given: swept
hob Ø32 at full depth, swivel +-15 deg, 0.3 mm clearance; the run takes about a minute). Figure: `keepout.svg`.
Evidence for the mandrel loads and clamping: `../loads/README.md`, `../fem/README.md`.

## Today (reference)

- The cup sits open end down: the open end face is the seat (z = 0), the tooth band is z 0..8, the 2 mm closed
  end plate is at z 38..40 (`FlexsplineCupBlank`: height 40, endThickness 2). A 2 mm high stop land of radial
  width 0.3 (Ø58.65) carries the open end face; the sleeve starts at the seat.
- Hob: Ø32, blank Ø62 (R 31), root R 29.625, so the cut is 1.375 mm deep. The CAM engagement overrun is the
  chord of the hob circle at that depth, sqrt(16^2 - (16 - 1.375)^2) = **6.489 mm**, plus 1.0 mm clearance:
  the hob centre starts **7.49 mm below the seat** (`hobCentreZMin`) and ends 15.49 above it.
- Keep-out: the narrowest point of the swept hob is the root circle R 29.625 at the lowest hob centre. With the
  0.3 mm clearance the mandrel and fixture must stay within **R 29.325 from z = -7.49 up to the seat**, and
  the land is exactly at that limit (R 29.325): zero margin. Below -7.49 the limit opens quickly
  (R 29.49 at -10, 30.84 at -15). The mandrel stem under the land is R 28.03 (1.3 mm inside the limit).

## Option A: closed-end datum (nose stop on the inside face of the end plate)

The cup hangs on a ring or three pads on the mandrel nose, touching the inside face at z = 38; no land at the
open end; the sleeve starts below the open edge.

**Datum face.** In `FlexsplineCupBlank` the inside bottom face is a flat annulus of R 29.025 (z = 38) with no hub,
boss or inner fillet modelled. That is only a placeholder: a real cup has an inside corner radius (assume 1 mm),
so the contact must stay inside about R 27.5, and if the output hub carries a boss or a central bore the pads
sit on the plate between them. Use 3 pads (kinematic, 120 deg) or a ring, not the whole plate: the plate is only
2 mm thick and dishes after hardening, but 3 contact points do not care about its flatness.
Achievable: the bore and the inside face are produced in the same setup from the open end (finish bore, then
face the plate with the same boring bar, L/D < 1), so perpendicularity bore to face of **3 um** is a machine property
(the same figure R8 asks of the open end land), local flatness at the pads 5 um. Tilt this adds: 3 um over the
~55 mm pad circle = 0.055 mrad = **0.44 um over the 8 mm band** (budget 2 um), better than cocking in the
clearance (1.75 mrad = 14 um over b) if nothing pulls the cup down, because the weight and the clamp's
friction hold the plate on the pads and the bore contact gives the rest.

**Axial stack-up, datum face to tooth band** (band bottom 38 mm below the datum, centre 34, top 30):

| Item | Value | Note |
|---|---|---|
| Inside face to open edge (cup depth, ground/turned in one setup) | +-0.020 mm | dominant; set by the cup drawing |
| Mandrel: pad height above its own work seat | +-0.005 mm | ground |
| Pad contact on the plate (local roughness, debris) | +-0.003 mm | inside the cup, no chips |
| Elastic, 100 N pull-down (fem/README case 4: top shortens 0.21 um) | 0.0002 mm | |
| Thermal, 2 K steel on steel over 38 mm | 0.0009 mm | |
| RSS | **+-0.021 mm** | |

What changes is *which* edge carries the tolerance: with A the teeth are located to the functional feature
(the plate and hub that the wave generator and output flange see) to about +-0.006 mm, and the open edge
floats by +-0.020 mm relative to the teeth. The hob has 6.49 + 1.0 mm of run-out below the face, so +-0.02 mm
on the open end does not touch tooth quality; it moves the face-width end by 0.25% of 8 mm. Today it is the other
way round: open end to teeth is exact, but teeth to closed end carry the full +-0.02..0.05 mm of cup depth.

**Hob keep-out.** The sleeve is no longer cut off at the seat, so it extends below the open edge. Its collapsed
OD is R 29.015 (R 29.025 clamped) against the limit R 29.325: margin **0.31 mm** at the tightest point (z
-7.49..0), 0.48 mm at 10 mm below, 1.8 mm at 15 mm below (table in the script output). A sleeve starting 3 mm below the
open edge (about 1/beta of the thin wall, so the sleeve's end land is clear of the contact start) is inside the
limit with 0.31 mm; the free part of the sleeve below the edge swells about 19 um at 7 MPa (P/ks, 2.67 um/MPa
for a 1.5 mm wall), which uses 0.019 of the 0.31. This replaces the zero-margin land with a 0.31 mm margin and
removes the land (nothing at R 29.325 any more); everything below the sleeve is the R 28.03 stem.
FE case 1 pressure profile is unchanged: pressure still starts at the cup's open edge, the sleeve simply continues
under it, which also puts the sleeve's end land (the V1/V2 problem in fem/README case 1b) outside the band.

**Seat check.** The nose is the natural place and it is solid, dry and away from the hob: either three air-gauge
nozzles (Ø0.8 mm orifice flush with the pads, back pressure rises when the plate sits down, about 1 um
resolution) or three eddy sensors in the pads. This is loads/README seat-check option A without the extra 4 mm of
plate-to-post tolerance, since the plate is now the datum, and it needs no holes in any land. Add the booster
stroke/pressure check as plausibility. Chips and oil run down the outside of the cup; the pads are inside it.

**Costs and risks.**
- Mandrel nose: 38 mm above the seat instead of 18; `FRONT_LIFT` is 35 mm and wants protrusion + 5 =
  **43 mm (+8 mm)**, the robot lift, `LOADING_CLEARANCE.json` and the 30 mm "maximum protrusion" text change.
  The robot lowers the cup 38 mm over a Ø56 post (1 mm radial clearance) before the sleeve, so the lead-in chamfer
  sits on the sleeve top edge.
- The cup plate inner face needs a finished flat and perpendicular face (one more drawing feature, same setup as
  the bore); the final cup may have a fillet or boss that changes the pad layout.
- Axial hold when the hob pulls up: friction only (28 N required), same as today.
- Model change would be: `workSeat` stays the band reference, `sleeveStart = workSeat - 3`, `sleeveLength = 19`,
  `stopDiameter = 0`, mandrel height 38, `pressesBand` still true.

## Option B: sacrificial collar (blank longer at the open end, parted off after hobbing)

**Run-out and collar length.** The hob is fully engaged over a chord of 2 x 6.489 mm. For the hob to stay in
material for the whole band, so that the end of the band is cut like its middle and the unsupported thin edge does
not deflect at the exit, the collar must be at least the overrun, **6.49 mm**; with the 1.0 mm CAM clearance
and a part-off kerf and clean-up (about 1.5 mm to be parted from the band edge, finishing 0.2) the length is **8 mm**
(5 mm is not enough: the last 1.5 mm of the band would be cut with the hob leaving material). The collar is blank
stock (Ø62, wall 1.975, 23 g) and gets partial teeth; it is thrown away.

**Keep-out gained.** The seat moves down by the collar length, the hob does not (it still starts 7.49 mm below the
band), so relative to the seat the lowest hob centre rises to Lc - 7.49. Allowed radius (mandrel and fixture) at
the seat plane and below:

| Lc (mm) | lowest hob centre rel. seat | z = 0 | -2 | -5 | -8 | -12 |
|---|---|---|---|---|---|---|
| 0 (today) | -7.49 | 29.325 | 29.325 | 29.325 | 29.333 | 29.860 |
| 5 | -2.49 | 29.325 | 29.325 | 29.493 | 30.140 | 31.812 |
| 8 | +0.51 | 29.333 | 29.493 | 30.140 | 31.290 | 33.794 |
| 10 | +2.51 | 29.493 | 29.860 | 30.841 | 32.397 | 35.558 |
| 12.6 | +5.11 | 30.021 | 30.679 | 32.142 | 34.284 | 38.482 |
| 15.4 | +7.91 | 31.011 | 32.028 | 34.122 | 37.029 | 43.326 |

With an 8 mm collar the gain is 0.01 mm at the seat plane (nothing), 0.17 mm at the bottom of the land, 0.8 mm
5 mm below the seat and 2 mm 8 mm below. That helps a flange or union below the seat, which is not what limits
this mandrel (the stem is already 1.3 mm inside the limit).

**Can the land become a normal wide face?** The land top is at the seat plane, so its outer radius needs the
allowed radius at z = 0:

| Land width (radial) | outer R | collar length needed |
|---|---|---|
| 0.3 (today) | 29.325 | 0 |
| 0.6 | 29.625 | 10.9 mm |
| 1.0 (room for a Ø0.6 air hole) | 30.025 | **12.6 mm** |
| 1.5 | 30.525 | 14.2 mm |
| 1.975 (the whole end face, blank OD) | 31.000 | 15.4 mm |

So an 8 mm collar does not widen the land at all, and air seat-check holes need a collar of about 12.6 mm.
That collar means a 52.6 mm blank (+37 g of steel), a 28.6 mm sleeve (band + collar + 8 mm), and a thick-walled
collar (1.975 mm, 3x stiffer in hoop than the 0.6 mm wall) next to the band bottom: its stiffness step is
6.5 mm below the band bottom at best, so about exp(-0.31 x 6.5) = 13% of the end-restraint disturbance remains at the
band edge (about 1 um at 1.5 MPa, a third of the 3 um band variation limit) unless the collar OD is turned thin.

**Extra process steps.** (1) Blank turned with the collar and the collar bottom face ground flat and square
(replaces today's open-end grind, same 2 um / 3 um). (2) Hob. (3) Part off the collar from a hardened toothed
thin ring on a mandrel (second clamp; parting tool, wheel or wire EDM) at z = 0 +- 0.05 relative to a datum, and
this plane is again referenced to the open end or to the plate, so B does not remove the need for a
closed-end reference, it only moves it to the part-off. (4) Deburr the parted edge and the tooth ends, break the
edge (the 0.1 mm edge break spec). (5) Inspect the parted face (flatness/perp are now functional only for the
wave generator side). Net: +3 operations per part (collar grind is the old open-end grind, so net: part-off, deburr, second clamp), +23..37 g of scrap, plus the part-off position error
(+-0.05 mm) lands directly on the face width.

## Comparison

| | Today | A closed-end datum | B collar 8 mm | B collar 12.6 mm |
|---|---|---|---|---|
| Axial datum | open end face (0.3 mm land) | inside face of the plate | collar bottom face (0.3 mm land) | collar bottom (1.0 mm face) |
| Teeth to closed end | +-0.02..0.05 (cup depth) | **+-0.006** | cup depth + part-off +-0.05 | same |
| Teeth to open edge | exact | +-0.020 | +-0.05 (part-off) | +-0.05 |
| Keep-out margin at the mandrel | 0 (land) | **0.31 mm** (sleeve) | 0.01 (land) | 0 (1.0 mm land at the limit) |
| Air seat-check in the datum face | no (0.3 mm land) | yes (nose nozzles, dry side) | no | yes (about 1 mm face) |
| Sleeve | 16 mm from the seat | 19 mm, starts 3 mm below the edge | 24 mm | 28.6 mm |
| Mandrel height / robot lift | 18 / 35 mm | 38 / 43 mm (+8) | 26 / 35 | 30.6 / 36 (+1) |
| Extra cup process | none | inside face finished | collar, part-off, deburr | same, longer |
| Extra scrap per cup | 0 | 0 | 23 g | 37 g |
| Clamp uniformity at the band | free edge, pressure starts at the edge | free edge, sleeve land outside the band | thick collar stiffens the band edge | same, a little more |

## Recommendation

**Choose A.** It is the only option that adds keep-out margin where the mandrel is tight (0 to 0.31 mm), gives a
dry, chip-free place for the seat check without a hole in a 0.3 mm land, ties the teeth to the functional closed-end
feature, and costs one finished inside face that comes from the bore setup. B only wins on the hob exit (support
for the thin edge), and to get the same seat-check benefit it needs a 12.6 mm collar, an extra hardened-ring
part-off and a stiffness step beside the band. The price of A is mandrel/robot height: nose at 38 mm, lift 43 mm
(+8 mm); the sleeve stays inside the keep-out with 0.31 mm.

If the hob-exit edge deflection turns out to matter (not modelled; an unsupported rim deflects 0.10 um/N, fem/README
case 3), an A cup can still be hobbed with a short collar of 8 mm in addition, since A removes the land and the collar is then
only a hob support, with no keep-out cost; decide that on the first cuts.

Open items for A before changing the model: the real inner corner radius and any boss, the pad ring radius
(about 20..27 mm), and the robot lift (`FRONT_LIFT`, `LOADING_CLEARANCE.json`).
