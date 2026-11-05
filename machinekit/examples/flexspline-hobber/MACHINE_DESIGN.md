# GE-class moving-column hobber

This is a pure vertical hobber. The direct-drive C table is fixed; X moves the
column towards the table on twin bed guides, Z moves the head on the column,
A swivels directly on the Z saddle, and Y shifts the hob on twin rails.
Turning belongs to the separate lathe plan. This checked design is not a released
manufacturing drawing set: stiffness, guide preload, fits and drive friction
remain assumed inputs.

## Current frames, drives and head (FRAME_PLAN.md, 2026-11-04)

This section is the current state; the sections below keep their history and are corrected where a figure changed. All of it is
model, not released manufacturing, and every assumption is listed in [FRAME_PLAN.md](FRAME_PLAN.md) (Decisions, Open).

- **Frames, no welding:** the X carriage is one milled steel part (20 mm deck, 25 mm ribs, open U); the Z saddle one milled plate
  with the eRob pocket and the nut block bolts; one ground steel bar pad per axis (X and Z) carries both screw supports, ground together so the seats are coplanar; the Y drive stands on two milled
  L brackets and a nut seat bar; the swivel plate is ground steel plate (the Y rails and the spindle seat are steel: aluminium would be about a third as stiff and would bow against them as it warms), the adapter ring turned aluminium; the bought Ø80 BT30 belt spindle is held
  by one milled split-clamp seat (axial envelope assumed, `SpindleHousing.CLAMP_FROM..CLAMP_TO`); its standard-taper hob arbor is a separate made part. Each has a `ManufacturingRoute`;
  `supplier-packages/machined/` holds their STEP, drawings and RFQ table.
- **Drives:** SFU1605 screws (X 260, Y 170, Z 420 mm), HZMotion M60D-BK12-C mount with BF12, Leadshine ELM2H-0400 servo on each axis, Z with its built-in
  brake (holding torque assumed 2 × rated), steel nut blocks.
- **Scales:** Yuheng JFT-40 sealed absolute BiSS-C scales, X ML 120, Y ML 70, Z ML 220, with the Abbe offset of each printed by `HobberFrameChecks.scales`.
- **A axis:** ZeroErr eRob110I (ratio 100, brake, ±15°) on the Z saddle; output bearing moment rating and brake holding torque are unknown or assumed.
- **Electrical:** one 48 V supply with a regeneration resistor and a BiSS master on the controller; the A axis has its own amplifier.
- **Masses (this model, `layout-metrics`, regenerated into [LAYOUT_REPORT.json](LAYOUT_REPORT.json)):** moving X 184.4 kg, Y 24.7 kg, Z 47.8 kg; whole machine 1521 kg, 255 members (the bed casting package is 1170.9 kg; 835.6 kg is the figure of an older revision kept in the sections below). The table below and the HB-D11 sections keep the 2026-10 figures (135 / 14 / 41 kg, 1133 kg) as history.

## Coordinates and load path

`MACHINE_FROM_WORK` is an explicit +90° rotation about C and a 637 mm floor-height translation. Work-frame radial X
therefore becomes global Y; the untilted hob axis becomes global X. All cutting,
synchronisation and force calculations retain their work-frame conventions.
The frame regression compares actual assembled poses, cut boundaries and loads
with the analytic reference chain, rather than changing the cutting definition.

The base is one 930 × 800 × 800 mm monolithic mineral casting, supported on
three rigid levelling feet with 47 mm nominal floor clearance. Its top is 847 mm above the floor and the tooth
band is at 1000 mm. The initial 650 mm plinth height would leave the band at
850 mm with this low fixture; the taller casting raises the work without adding
a stand, long legs or a taller fixture. The casting retains an 85 mm sole,
upper ribs/pockets, a work well and drainage passages below the table.
The bed has two continuous longitudinal casting beams under the
50 mm rail-seat inserts. Bed guides are spaced 440 mm apart (centres ±220 mm),
and the carriage is widened to suit. Lightening cores stay outside 110 mm beam
bands; the chip well leaves at least 50 mm beam width at the guide seats.
Each complete bed rail-seat footprint is backed by solid casting down to the sole;
the work well is 350 mm wide throughout for top-down chip-tray installation.
The steel top faces deliberately stand 10 mm above the casting for rail mounting.
Full-footprint solid checks verify the bed load beams and at least 10 mm of backing
behind both column rail inserts. No rail mounting face overhangs a casting cavity.

Its table seat, two column rail seats, drive seat and lifting points use shared
`machinekit.structural` casting recipes and separately listed steel inserts.
Precision mounts land on inserts, never on mineral cast. The recessed column is
400 × 200 × 500 mm, on a steel carriage and twin preloaded HGR25 bed rails.
Z uses twin 480 mm HGR15 rails with four blocks. Its vertical saddle carries
a ZeroErr eRob110I actuator, its housing in a pocket cut into the saddle from behind. The rotating output and vertical swivel plate
carry the Y rails on their front face with four blocks. The spindle (in its split-clamp seat) and floating-sleeve outboard housing share the vertical
Y saddle; there is no overhead roof. These compact motor and bearing dimensions and ratings require
supplier confirmation; geometry checks do not establish a thermal duty rating.
All twelve carriages use pinned HGH25CA (X) or HGH15CA (Z/Y) supplier CAD,
including wraparound sides and lubrication fittings. X uses matching supplier-derived
HGR25R rails; Z/Y retain the verified analytical HGR15R running profile. These
portable recipes require explicit local supplier-asset preparation. The source
CAD does not qualify load ratings or mixed-material carriage mass.

The X screw runs centrally between the bed rails, at floor Z=817 mm (work-frame Z=180 mm), in a 110 mm wide
recessed casting channel. Its nut carrier drops from the carriage into the channel;
a steel deck mounts to a separate ground-after-casting steel insert. The fixed
support and motor sit at the rear; the screw axis points towards the table.
The screw transmission sign follows that reversed axis. The insert starts beyond
the chip guard, with 45.5 mm nominal radial clearance. A way cover over the screw does not fit as built
(see "Chip protection and service" below: the front one fits dimensionally but has no host for its wall, the rear one is 31 mm short);
the exposed mechanism is an inspection model with wipers on the guide blocks.

Ground screws and linear scales serve
all three translations; screw manufacturing grades and preload need procurement
confirmation.

| Quantity | Previous overhead bridge | Moving column |
|---|---:|---:|
| X guide-plane to hob centre | 347 mm | 121 mm |
| Z guide-plane to hob centre | 325 mm | 161 mm |
| Y guide-plane to hob centre | 137 mm | 70 mm |
| X moving mass | 81.71 kg | 135.26 kg |
| Z moving mass | 106.79 kg | 41.30 kg |
| Y moving mass | 18.01 kg | 14.03 kg |
| Machine mass | 351.61 kg | 1132.83 kg |
| Bed footprint (radial × tangential) | 650 × 750 mm | 930 × 800 mm |
| Installed-pose envelope, including separate rack | 1225.63 × 750 mm | 1510 × 800 mm |
| X continuous thrust margin, same assumed demand basis | 911.15 N | 884.37 N |
| Y continuous thrust margin, same assumed demand basis | 897.29 N | 909.38 N |
| Z continuous thrust margin, same assumed demand basis | 859.36 N | 1534.35 N |

The initial moving-column revision had a 125 mm Z guide offset but retained the
old underhung head. The front-mounted head needs 36 mm more radial space: its
Z guide offset is now 161 mm, slightly beyond the approximately 150 mm target.
The swivel-bearing front face is 114 mm behind the hob, its rotating output face
100 mm, and the Z saddle/swivel mounting face 135 mm. The bearing is physically
in front of the saddle. X remains 121 mm.

[LAYOUT_SECTION.svg](LAYOUT_SECTION.svg) is a dimensioned radial section schematic;
[LAYOUT_REPORT.json](LAYOUT_REPORT.json) records measured masses, datums, demand
basis and validation. Y-offset guides and spindle housing are projected into the
section. It is not a manufacturing drawing.

Offsets are distances to the nearest guide plane, not predictions of angular
stiffness. Masses include generated bodies at their stated material densities, with the
separate electrical rack included and the enclosure omitted;
`layout-metrics` reports the current masses and continuous-drive thrust margins.
Drive demand includes 0.5 m/s² acceleration, Z gravity, assumed 100 N process load
and 100 N friction, with 20% continuous-thrust reserve. Y includes its gravity
component at the worst declared ±15° tilt; Z includes full gravity. These assumptions do not
replace the HB10 measured-load report or modal analysis.

## Travel, fixture and support

The target envelope is flexspline outside diameter 25–140 mm, face width up to
20 mm. The X mechanism spans 25..110 mm with fixture-dependent cutting soft limits;
front loading retracts to 110 mm. Z spans -10..190 mm, Y ±10 mm,
and A ±15°. Guide checks include block length, end margins and overtravel.
An integral low-profile flange and mandrel base replaces the stacked riser assembly;
it preserves the work-frame tooth-band datum (Z=363 mm), now 1000 mm above the floor, and tilt clearance. The mandrel sleeve continues
to support the toothed rim.

The spindle locates the arbor axially. The outboard uses two SKF 7002 CDGA/P4A
15 × 32 × 9 mm universally matchable bearings back-to-back in one ground sliding
sleeve. SKF states 20 N class-A unmounted preload and 17 N/µm axial stiffness for
the DB/DF pair ([maker sheet](https://docs.rs-online.com/4857/A700000013821392.pdf)).
The sleeve shoulder and M36x1 gland clamp the outer rings; the arbor's Ø20
shoulder, ground inner spacer and M15x1 AF22 retaining nut clamp the inner rings.
The Ø15 journal is 29 mm long (arbor 151..180): pair 151..169, spacer 169..174,
retaining thread 174..180. Shoulder fillets are at most 0.2 mm. Shaft fit, installed
preload and retaining torque require shop qualification; no vendor runout or
installed radial stiffness is inferred from catalogue class alone.

The entire preloaded sleeve floats ±1 mm between the housing's front stop and
the rear cap's spigot. The cartridge remains the only axial locator. With assumed
steel expansion 12e-6/K and 30 K rise over 180 mm, growth is 0.0648 mm;
0.2 mm assembly allowance leaves 0.7352 mm of travel. The Ø21 front throat
forms a dry labyrinth with 0.5 mm radial clearance to the Ø20 arbor shoulder.
The fixed cap bore Ø30 clears the retaining nut corners by 2.298 mm radially;
a thin 22 AF socket with OD ≤28 fits through it.

Indicate the housing bore against the spindle axis before tightening its four
derived M4 screws; shim the carrier face, then ream and fit two derived Ø4 H7
pins. The worst-case **radial** coaxiality budget is 5 µm: 2 indication residual,
1 sleeve/seats, 1.5 bearing, 0.5 thermal (all **assumed project requirements**).
Indication residual 2 µm corresponds to 4 µm TIR, not 2 µm TIR. `b-align-outboard`
declares assembly, pinning, free-slide verification and service. Remove the nut
and inner spacer first, then pull the complete carrier/housing/pair off the journal.

## Chips and enclosure

A sloped annular table guard drains into the tray below the work. Its recipe now
uses the installed table face, housing and height rather than the old fixed
cone. The stationary lip clears the rotating face radially by 5 mm and sits
3 mm below it; the 2 mm skirt clears the housing by at least 15 mm and extends
to 2 mm above the table seat. For the default table this gives a Ø210 mm
opening and Ø294 mm lower outside diameter, covering the full 210 mm housing.
Removal/fastening details remain manufacturing-design work. A drilled
coolant-manifold placeholder reserves flushing provision. The stationary rail-end
shields have been removed at the user’s request. Rails, blocks and the central
screw remain exposed; bellows with suitable chip protection and attached end
frames are an open design item, including compressed-length checks at both
travel limits. The enclosure recipe reserves
the machine envelope, front loading access and optional top opening; its panels are currently omitted
from the assembly at the user’s request for unobstructed inspection. Door interlocks,
fluid containment and a complete conveyor are not implemented.

## Acceptance and remaining work

Mechanical checks cover mounted-solid contact, insert-only precision mounts,
rail margins, connected fabricated saddles and persistence. Exact-solid audits
check the initial/cutting poses and the declared X/Y/Z/A grid, permitting only
the intentional hob/blank cutting intersection. A grid is sampled clearance
evidence, not a continuous swept-volume proof.

HB6 now concerns this moving-column assembly, integrated fixture, twin Y rails,
floating outboard support and chip/loading provisions. HB9's app smoke check is
a coarse physical cut; it cannot establish reference ISO grade or HB12's full
reference cycle-time acceptance. Frozen HB4/HB11 cuts use their original layout
and remain separate evidence. Structural/modal analysis, spindle/mandrel sag,
thermal behaviour and detailed manufacturing release remain open plan items.

## Accuracy budget (2026-11-06)

`HobberAccuracyBudget` (`machinekit/hobbing`) predicts the reference gear's ISO 1328 class from the machine model; `tests/hobber/src/HobberAccuracyChecks.hx`
reads `FlexsplineHobber` (read-only), prints the budget and gates on it. Run: `haxeon run --project machinekit/tests/hobber/haxeon.accuracy.json`
(about 1 min; argument `report` prints without failing, `uncompensated` adds the budget before the tilt-compensation step).

- **Target.** PLAN.md HB12: "the reference flexspline grades ISO 5-6 on every parameter" (prototype target class 5-6). The gate is **class 6**; class 5 is the stretch.
  Class limits come from `gearkit.Iso1328` (ISO 1328-1:2013 tolerance formulas, class 5 base from module and reference diameter, times 2^(1/2) per class; fp = 0.001 d + 0.4 m + 5,
  Fp = 0.002 d + 0.55 sqrt(d) + 0.7 m + 12, Fr = 0.9 Fp, profile and helix terms from the standard). m 0.3 is below the standard's range (m >= 0.5), so every class here is extrapolated.
  Class 6 limits at d 60, b 8: fp 7.5, Fp 23, Fr 21, F-alpha 9.5, F-beta 11 um (rounded as the standard rounds; unrounded fp 7.3, Fp 23.5).
- **Rows.** Each error source has a value, a basis (Catalog, Model, Stated, Derived, Assumed), a sensitivity and a bound on fp, Fp, Fr, F-alpha, F-beta (and tooth thickness, which ISO 1328 does not
  grade; it is set by the first-article trim). A bound is what that source alone can do to the parameter. Totals: RSS (expected, gates) and the plain sum (all at bound). Mapping:
  radial error to thickness 2 tan(alpha) = 0.728 per um; a hob radial runout to profile sin(alpha) = 0.342 per um and axial runout cos(alpha) = 0.94 per um (a single-start hob makes one
  revolution per work tooth, so its runout repeats on every tooth and gives no pitch error); C angle to pitch 2 r per rad = 0.291 um per arcsecond; A and guide tilt to helix b per rad = 0.039 um per arcsecond;
  B encoder and timing reach C divided by z/starts, so 1 us of unknown delay is 0.09 um (1 ms is the 47 um case); work eccentricity is Fr = Fp = TIR.
  Values from the model: scale accuracies and Abbe offsets (`HobberScales` poses), table fields (`RotaryAxisSpec`), part runouts (`PrecisionErrors` facets of hob, arbor, outboard support and spindle bearings),
  eRob output accuracy, mandrel eccentricity, roundness and band expansion, outboard stiffness. Assumed (each labelled in the printout): guide angular error 5" (the `HobberFrameChecks` value; no rail class in
  the catalogue), tilt-table residual 20 %, rail straightness 3 um per 200 mm, thermal dT 1 K (frame loops of 262 mm steel and 253 mm mineral cast from the poses), cartridge nose 50 N/um, unlisted structure equal to the listed,
  A torsional stiffness 2e4 N m/rad with 10 mm of engagement lever, B encoder accuracy 40", sync delay 1 us, cycle 250 us, B speed ripple 0.5 %. Forces are the HB10 values already quoted in the repository (radial 17.0 N, tangential 46.7 N, table torque 1.49 N m).
- **Result (this model).** With the tilt table loaded: fp 1.1, Fp **60.1**, Fr 3.9, F-alpha 4.2, F-beta 1.2 um RSS, i.e. extrapolated classes 1, **9**, 2, 4, 1; worst-case sums 2.1, 68, 6.8, 11.9, 2.4 (classes 3, 10, 3, 7, 2).
  The check **fails the gate on Fp only**, and the cause is a single modelled number: the table's once-per-turn speed ripple (assumed 0.1 %, 60 um of Fp; the same finding as the table-only screen above). Every other
  row together is 4 um of Fp (RSS). The ripple that keeps Fp at class 6 with the rest unchanged is 0.038 % (RSS; the 0.0317 % above sums the amplitudes instead). The tilt-compensation step changes only tooth thickness
  here (RSS 6.3 to 5.0 um): the Abbe offsets are 220 mm on X and Z, 72 mm on Y (5" gives 5.3 um uncompensated, 1.1 um after), and none of it reaches a class parameter for a spur gear.
- **Top three limiting causes** (share of the class 6 limit on the worst parameter): 1. C table speed ripple, 261 % of Fp (measure it on the bench at 10-20 rpm; position-loop gain or torque-ripple feedforward; ask the vendor);
  2. hob chain axial runout (hob, arbor, two spindle bearings, 2 um TIR each, assumed) 3.8 um on F-alpha, 40 % (square clamping faces, indicate the hob face, lower-runout cartridge);
  3. hob chain radial runout (five 2 um sources, assumed) 1.5 um on F-alpha, 16 % (arbor and outboard coaxiality). Outside the class: X thermal growth 6.1 um per K is the largest tooth-thickness term (thermal control or a trim per part).
  Also close: mandrel eccentricity 3 um on Fr and Fp (13 % of Fp). Not budgeted: hob lead and pitch errors, hob wear, flexspline wall compliance (HB10/HB11 simulate it), forced response at the tooth-pass frequency (the spindle drive modes keep a 1.3x margin from the band, so the quasi-static stiffness is used).
- **Limits of the budget.** It is analytic and untrimmed; static terms (thickness, a constant helix error from A or deflection) can be trimmed from a first article. Most rows are assumptions until a bench measurement; the ranking
  is robust, the absolute Fp number is not (it is one assumed ripple). The full-cut error budget (`stockkit.hobbing.errors`) remains the simulation; this one ranks the machine's parts.

## Feedback and drive requirements (open, 2026-11-01)

Procurement requirements for position feedback. None is selected or commissioned.

| Axis | Feedback | Requirement | Status |
|---|---|---|---|
| X | Sealed absolute linear scale, JFT-40 ML 120 | ±3 µm/m, ≤ 0.1 µm, BiSS-C, slim, air purge | RFQ drafted (Changchun Yuheng JFT series) |
| Z | Same, JFT-40 ML 220 (stroke 200 + 2 × 5 mm overtravel) | ±5 µm/m; spur parts, so scale error barely reaches the gear | same RFQ |
| Y | Same, JFT-40 ML 70 | ±5–10 µm/m; same model as Z for spares | same RFQ |
| C | Table's own angle encoder | Purchased table, ±5″ positioning / ±2″ repeatability; protocol must match the C drive | uncommissioned |
| B | Spindle servo's own absolute encoder (master) | ≥ 2¹⁶ counts/rev (model 2¹⁷), up to 4000 rpm, deterministic latching and low, fixed latency | with drive selection |
| X/Y/Z/A motors | Servo motor encoders | Standard | with drive selection |

Why X matters most: radial error changes tooth thickness by ≈ 2·tan 20°·ΔX ≈ 0.73·ΔX.
Its repeatability and thermal stability matter more than absolute accuracy.

Open decisions:

1. **Drive selection, X/Y/Z.** Each drive needs a second encoder input for full closed loop
   that accepts BiSS-C. The motor encoder closes the velocity loop and the scale closes position.
   Avoid EnDat-only or vendor-locked protocols. Without this input, the scales cannot be used.
2. **Drive selection, B.** The B drive must report the master position with a deterministic,
   EtherCAT distributed-clock timestamp every cycle. C then extrapolates it with feedforward.
   B encoder *accuracy* is attenuated at C by `starts/teeth` (1/200 for the reference part:
   a 40″ B error is 0.2″, ≈ 0.03 µm at r 30 mm). Timing is not attenuated: at 3000 rpm,
   1 µs of unknown delay is ≈ 65″ at B (≈ 0.3″ at C). An uncompensated 1 ms delay is the
   47 µm case in PLAN.md. Spindle accuracy limits are hob/arbor runout, bearing preload and
   thermal growth (HB14), not the encoder.
3. **C drive and protocol.** The table encoder protocol must match the C drive. The design's own
   screen is limited by assumed 0.1 % once-per-revolution table speed ripple, which must be
   ≤ 0.0317 %. Raise this with the table vendor; better scales do not fix it.
4. **Scale purge air.** Branch from the 6 bar clamp supply: regulator (≈ 1 bar), coalescing
   and fine filters, and a dryer, to ISO 8573-1 class 1:4:1 (oil-free). Then a distribution block
   to the X/Y/Z scales. Optionally add a pressure switch as a cut-permission interlock.
   Not yet a service port.
5. **Hob drive (decided 2026-11-01).** The hob has a Ø16 H5 plain bore with one end
   drive slot (face key, DIN 138, supplier-standard dimensions) and is axially
   clamped on the arbor. The arbor collar therefore needs a matching drive dog. The
   model only has the optional longitudinal keyway (`Hob.fromDesign(.., keyed)`), so
   the dog is not modelled yet. The plain bore keeps the arbor runout budget (2 µm);
   the slot gives positive, repeatable drive, so a slipping hob cannot silently
   break the B/C gearing. A shank-type hob, which removes the arbor stack, is the
   later route for class 3. The supplier packages are built by `supplier-packages/`.
6. **Mandrel actuation (decided 2026-11-01).** The expanding mandrel is actuated
   by 6 bar air through a rotary union in the C bore, driving an air-to-oil
   booster in the fixture that supplies the 10 MPa (15 MPa maximum) sleeve
   pressure. Pressure is regulated so the 0.6 mm cup wall is not distorted
   (≤ 2 µm). The union carries 2-3 air passages: clamp, unclamp or spring return,
   and possibly a seat check. The seat check cannot use air holes in the 0.3 mm
   stop land; the method (air through the sleeve clearance before clamping, or a
   sensor at the closed end) is left to the mandrel maker. Low drag
   torque is a requirement, because seal drag is a table ripple source. A manual
   screw-piston mandrel is ruled out by robot loading; a 100 bar hydraulic union
   would add drag. The sleeve is sized from the cup bore (Ø58.05 H5, 0.020 mm
   diametral collapsed clearance = `FlexsplineHobber.LOADING_CLEARANCE`: OD 58.040 -0.005/0 against
   58.050/+0.013 gives 0.010 to 0.028 mm), as `FlexsplineHobber` builds it; "OD 59"
   in PARTS.md is only the generic recipe default. The earlier 0.050 mm clearance could not hold
   the cup (about 0.2 MPa contact at 10 MPa oil). Rated values now in `ExpandingMandrel`
   (evidence: `workholding/loads/README.md`, `workholding/fem/README.md`):
   - `oilPressure` 10 MPa / `maxOilPressure` 15 MPa are oil pressures inside the sleeve. The
     contact pressure on the cup is a separate window, `contactPressureMin/Max` 0.5 to 1.5 MPa
     (oil 5 to 9 MPa for the 0.010-0.028 mm clearance range).
   - Holding: 3.0 N·m torque (SF 2 on the 1.49 N·m `maxTableTorqueBound`, mu 0.08) and 28 N
     pull-off, replacing the underived "below 5 N·m".
   - Expansion: the cup wall grows 6.8 µm per MPa of contact pressure, so the old 2 µm limit
     (0.29 MPa) could not hold the part. Uniform expansion up to 10 µm radial is a size offset
     compensated by hob depth (`uniformExpansionLimit`); non-uniform (lobed) part ≤ 1.5 µm
     (`lobedDistortion`); expansion variation over the 8 mm tooth band ≤ 3 µm
     (`bandExpansionVariation`, flank-line effect 0.34 x that); clamped sleeve roundness ≤ 1.5 µm
     (`clampedRoundness`, the cup copies the sleeve at about 0.95 µm/µm).
   - Accuracy: clamped runout ≤ 3 µm TIR (`eccentricity` 0.0015 mm amplitude; the old 0.004 mm
     was 8 µm TIR, over the 5.1 µm class 5 workholding Fr allowance), re-clamp repeatability
     1.5 µm, tilt ≤ 2 µm over 8 mm.
   - Pressed length: pressure starts at the cup's open edge (`sleeveStart` = seat) and runs the
     tooth band plus 8 mm (`pressedMargin`; FE case 1 gives band taper under 4% of the mean). The
     reference sleeve max(16, face + 8) = 16 mm satisfies this (`ExpandingMandrel.pressesBand`, checked
     in the loading test).
   Bore specification for the flexspline drawing (2026-11-01, updated 2026-11-02):
   Ø58.05 H5 honed after hardening, roundness 0.002 mm peak-to-peak (cup copies the sleeve and
   springs back; with the 1.5 µm sleeve it stays inside the 1.5 µm lobing limit), cylindricity 0.003 mm
   over the clamped length, wall thickness constant around the circumference to 0.010 mm
   peak-to-peak (expansion varies as dt/t: under 0.1 µm of lobing at 1.5 MPa; EMAG flags wall
   variation as changing expansion behaviour), Ra 0.8 (0.4 preferred), only an edge break
   (≤ 0.1 mm) at the open-end bore edge because the cup seats there on the 0.3 mm land (the loading
   lead-in is on the mandrel nose), seating face flatness 0.002 mm and
   perpendicularity to the bore 0.003 mm. The teeth are cut while the cup is
   clamped, so clamping expansion springs back as tooth error: contact pressure
   is limited to the 0.5 to 1.5 MPa window, which holds 3.0 N·m with margin.
   The mandrel request does not need the table drawing:
   the maker uses its standard flange and we make the adapter. Below the seat the
   mandrel and fixture must stay inside the swept-hob keep-out (0.3 mm clearance),
   which `supplier-packages/mandrel_package.py` computes from the CAM job's hob
   travel (centre 7.49 mm below the seat); a supplier's STEP must still be checked
   in the model for the arbor and outboard support. The union
   choice waits for the table's bore drawing (to ask Demat).
7. **Built-up table option.** If `RotaryTable` replaces the purchase, it needs a modular ring
   encoder, about 100 mm bore, two heads, absolute, BiSS-C. One set is quoted in the same RFQ (see `../../STRAIN_WAVE_RESEARCH.md`).

The RFQ was sent to sales@yu-heng.cn on 2026-11-01. It requests a quote for the three
scales and one ring-encoder set; the text is kept outside the repository, in
`materia-cache/claude-scratch/emails/yuheng-quote-request.txt`.

## Heavy plinth acceptance (HB-D11)

The casting is 835.58 kg; the base assembly is 859.22 kg and the complete machine
is 1132.83 kg. The casting-to-moving-mass ratios are 6.18:1 (X), 59.56:1 (Y)
and 20.23:1 (Z), meeting the assumed ≥5:1 X target. The central Z-drive spine raises column casting mass from the preceding
31.75 kg to 43.34 kg; its insert-equipped assembly becomes 62.00 kg (previously
49.97 kg). The pre-lightening reference was 37.28 / 55.50 kg. This reinforcement
retains the base-to-X mass-ratio target while providing a direct drive load path.
X moves 135.26 kg including the complete column/head. The three support points
span 690 mm radially and 660 mm across the front. Positive reactions are checked
through X travel against an assumed 15 kN target per foot; no vibration pads
are fitted. Floor, pad and screw compliance remain unqualified.

The three mounts now use side-access cast pockets, following the mill frame's
access-pocket/stem-passage technique. The mill's existing seat is cast in; the
hobber instead uses a removable 110 × 110 × 15 mm ground steel plate registered
under the pocket roof. Two separate nominal M24×3 nuts below the plate carry and
lock the adjustment. The screw pushes the plate upward against the casting roof;
there are no embedded steel foot anchors or threads in mineral cast.

The pocket floor is 40 mm above the casting bottom, its plate bearing roof is
at 160 mm, and a solid local boss retains 70 mm of roof material. A 28 mm cast
stem passage clears the nominal 24 mm screw. The registered plate recess has
0.5 mm nominal clearance per side. The two front mounts open outward through
the side walls; the rear mount opens through the rear wall. Exact-solid checks
cover an 80 mm-wide wrench corridor for both nuts and plate lowering/extraction.
Replacement requires unloading and lifting the base, withdrawing the stem,
lowering the plate 20 mm and sliding it out sideways; supports are never removed
under machine weight. Floor clearance, support centres and the 1000 mm tooth-band
height are unchanged. Nominal foot/stem envelope height is now 232 mm.

Maximum static reaction is 5.15 kN across 21 X positions; mean roof bearing stress
is about 0.455 MPa assuming full plate contact outside the stem passage. Full-area
contact needs a replicated/bedded roof bearing face, using a removable released
master; no machining of the mineral casting is specified. Bedding, roof bending,
creep, thread strength, lateral restraint, floor compliance and dynamic tipping
remain unqualified. See [FOOT_MOUNTS_REPORT.json](FOOT_MOUNTS_REPORT.json).

The default C table is a bought 200 mm-face, 210 mm-height, 40 mm-bore DD
unit seeded from the Demat RTV210-050-class purchase data in
[STRAIN_WAVE_RESEARCH.md](../../STRAIN_WAVE_RESEARCH.md#suppliers-and-prices-collected-2026-10-19).
Its 52 kg catalogue mass is split into 44 kg fixed and 8 kg rotating envelopes
for simulation only; the split and 0.04 kg·m² rotor inertia are assumed. The
rotor is included in the single purchased-unit BOM line. Its internal bearing
construction is not inferred from the face diameter. `PrecisionRotaryAxis`
(RA-D8) also accepts the built-up `RotaryTable`; a ground steel adapter fills
its smaller height so both forms retain the same work datum. Persistence and
interface parity are checked.

The larger mounting/fixture envelope needs a deeper table well. The seat top is
655 mm above the floor (work Z=18), table face 865 mm and toothed rim 1000 mm.
The integral fixture shoulder is 135 mm above the face. Its 25 mm-thick six-lobed
flange has a 140 mm hub and six 30 mm lugs on provisional PCD160, clocked 30°
away from the Y guides; M8 clearance/counterbore holes sit inside the lugs. A
100 × 3 mm pilot and the mating face pocket are provisional. These features
await the vendor drawing; they are not published mounting dimensions. The
40 mm bore remains clear through the table/insert/base access and into a 20 mm
fixture recess for a rotary union; actual plumbing/union design is not fitted.

Catalogue entries are 54.9/223 Nm, 3.43/17.9 A rms, 200/234 rpm, 50 kg payload,
±5″ positioning, ±2″ repeatability, 6 bar air clamp, natural cooling, IP67,
52 kg and 220/380 VAC driver input. Additional envelope/current/mass details
are the user's 2026-10-19 catalogue transcription; source confirmation and the
outline drawing remain required. A conservative Ø260 flange/Ø200 barrel is
assumed. The six M10 mount holes on PCD230 are provisional and externally
accessible above the flange. Required air, clamp-release/feedback, encoder,
driver-input and union services are exposed as ports. Port/fitting/cable details,
real clamp interlocking/failure behaviour and encoder protocol are uncommissioned;
the simulation assumes the table clamp released for hobbing. The existing generic
400 VAC supply/driver uses an assumed ±10% allowance on the 380 VAC nameplate.
The model's simultaneous peak regeneration check needs 12.33 kW; its declared
braking allowance is 15 kW, with pulse energy/duty/resistor selection still open.

Every numeric table value has an individual `ValueBasis` and reason, including
unpublished runout/NRRO, interpolation, cogging/ripple, seal drag, stiffness,
inertia, electrical constants, thermal values and mounting interfaces. See
[PURCHASED_TABLE_REPORT.json](PURCHASED_TABLE_REPORT.json) for the complete list.
Positioning/repeatability are never substituted for runout. Total runout is
split between periodic and non-repeatable shares without adding NRRO twice.
HB11 adds explicit interpolation, speed ripple, cogging, seal drag, NRRO and
radial compliance sources, using the installed description for either form.
Seal drag also consumes continuous torque reserve.

The **conditional analytic table-only pitch screen is extrapolated grade 9**
for the reference m0.3 part, dominated by the assumed 0.1% once-per-revolution
speed ripple: Fp bound about 64.0 µm versus the extrapolated class-6 limit 23 µm.
Keeping the other assumed bounds would require ripple ≤0.0317% for that pitch
allocation. This is not a simulated full cut, an overall gear grade or a vendor
accuracy claim. Full source/combined HB11 grading remains open; frozen earlier
batches describe the preceding built-up table. New budget reports identify the
installed table fields/signature and retain the independent assumed-plant sync
basis. Bench measurements at 15 rpm are needed before qualification.

[BASELINE_FOUNDATIONS.json](BASELINE_FOUNDATIONS.json) preserves the previous
210 mm bed's CAD geometry/masses and the preceding full-stroke heavy plinth.
`base-report` uses a simply supported beam, minimum net section and E=35 GPa
(assumed). Across the new full mechanism X25..110 range, the old bed retains
its original 860 mm support span and changes by 0.132 µm; the shortened plinth
uses its 690 mm span and changes by 0.00309 µm (0.00619 µm at half modulus).
The previous 1100 mm plinth changed by 0.00628 µm across X45.2..250; that
historical result is retained separately, not compared as equal travel.
The new bound is below the explicitly allocated 1.9 µm profile share.
Both beds pass this simplified allocation: this calculation alone does not
prove the old bed unacceptable. The old mass ratio and work height motivate
the redesign. Local insert/contact flexibility, joints, torsion, foot/floor
compliance and dynamics are excluded. Phase D FEA must check these and modes
against approximately 50/600 Hz excitation.

The casting reserves 7.92 L for a separate coolant tank below and forward of
the guide seats, with a connecting drain and front service opening. Cable
routing has its own upper passage. Tank lining, plumbing, thermal isolation
and coolant temperature control are design provisions, not solved thermal
performance. Lifting hardware remains deferred; existing insert placeholders
are not qualified lifting points.

## Foundation assembly sequence

Cast the bed with the steel inserts located as child parts, then grind or replicate
the steel mounting faces after cure; do not machine datum surfaces into mineral cast.
Inspect the rail-seat planes and backing before fitting the guides. Lower the chip
tray into the open well before the moving column/head are fitted; its insertion
path is checked with the bed guides, screw supports and table present. Fit and align
the bed rails with the column carriage removed, preserving access to every rail
mounting bore. Align the master guide first, then indicate the second guide parallel
before tightening. Install blocks, carriage and column afterwards; fit and align the
column guides before adding the Z saddle and head. The central screw deck remains
separate and accessible through its channel. Mounting threads, retention details,
fastener selections and tightening procedures still need manufacturing drawings;
this layout check is not structural or mould-process qualification.


### BT30 hob spindle (Phase P1, 2026-11-05)

The spindle is the bought RobotDigg 2049 option A **Ø80 BT30 belt-drive cartridge**.
The public listing confirms D80, BT30, 6000 rpm, HTD5M and the 7008+7006 bearing
family. Its linked drawing is labelled NT30 and differs from the text; it is not
accepted as BT30 axial-interface evidence. See FRAME_PLAN Decisions for links and
provenance. The model keeps a deliberate 4000 rpm operating cap. It replaces the
supplied 36/45T pulley with the existing 40T cartridge pulley for the 1:1 drive.

Load path: hob → plain Ø16 arbor seat and end-face key → BT30 taper → bought rotor
and bearings → bought housing → split clamp → Y saddle. The cartridge's rotor is
an included physical envelope for B kinematics, not a separately ordered shaft.
There is no made gland, separately purchased internal bearing pair or custom TF20
interface. Cartridge internals and drawbar retention are the supplier's assembly.

The **axial packaging remains assumed**, pending the selected option's outline:
rear-to-gauge 180 mm, front register Ø110 at 170..180, clamp land 10..168, Ø20 rear
stub 36 mm long, rotor/stationary mass partition. The split clamp grips 18..168
(150 mm), stopping 2 mm short of the front register. A Ø116 relief (minimum 2 mm clearance at the reader spine)
through the Y saddle lets that register leave toward the tool. Its outboard
carrier is a separate structural interface, fixed by four derived M4 with
washers and two Ø4 H7 pins, with a declared load/slip check. The carrier, support,
bearing and Y reading head come off before cartridge withdrawal; the reader
is clamped to a 2.3 mm spine behind the channel. These assumptions permit
layout checks; they are not manufacturing-release dimensions.

The made arbor has the standard MAS BT30 shank (gauge Ø31.75, 7/24 diametral taper,
48.4 mm long, Ø46 flange, M12 pull stud). It is taper-located; no dual face contact
is claimed. From its gauge plane: hob shoulder 40, hob seat 40..80, M16x1 retaining
nut at 80..88 (24 AF x 8, design inputs), Ø15 x 29 outboard journal at 151..180, with a Ø20 shoulder at 146..151.
An integral end-face dog 5 wide x 3 radial x 2 axial drives the plain-bore hob.
Its matching slot is 5.2 wide x 2.2 deep, **assumed** pending the hob maker's drawing.
The retaining nut is secured with medium-strength thread-locking compound and witness-marked; 2000 N assembly preload is an assumed target to establish by shop test. `b-fit-bt30-arbor` declares fitting, runout inspection and forward cartridge withdrawal with the outboard support off.
The 45-degree pull-stud head must match the bought gripper; its outline is assumed.
The arbor is turned in two setups, the dog milled at the second, then functional
surfaces ground. The machined and spindle supplier packages both describe it.

The Leadshine ELM2H-1000LA80F remains above the cartridge, 125 mm belt centres,
shaft rearward on its slotted cradle with jack-screw tension adjustment. A 450-5M
15 mm belt and 40T/40T pulleys transmit 3.18 N m rated / 9.5 peak; the Ø32 hob at
300 m/min runs at 2984 rpm, below the servo's 3000 rpm rated speed. The B master
remains the 19-bit spindle-side encoder on the rear stub; the motor encoder
serves the drive loop. Encoder geometry, belt properties and winding remain
assumed as recorded in FRAME_PLAN.

The analytic drive check reports about 450 N radial belt pull and requires at
least 700 N allowable pull at the cartridge pulley. The bought cartridge's
allowable pull and runout are unpublished: those are **assumed acceptance
requirements**, including the existing 2 µm model error parameters. Haozhi's
separately published accuracy/balance ratings belong to its Ø100 DGZZ-10020;
they do not qualify this D80 purchase. No bearing-life result is calculated
from invented internal bearing locations. Cradle modal estimates retain their
stated joint assumptions and need FE/impact-test confirmation.


### Cup and front loading

The workpiece is now a one-piece cup blank rather than an annular band: the
open/toothed end is the unchanged Z=0 work datum, the closed end is Z=40 mm,
with a 2 mm end plate and a 0.6 mm non-toothed wall below the tooth-root radius.
These body dimensions are assumed; no production flexspline drawing is claimed.
Only the tooth band is subtracted/inspected by RingStock; the unmachined body/end
are retained in the display and full CAD mass/collision model.

The integral mandrel has a 2 mm-high axial stop immediately below the work seat,
with a 0.3 mm radial rim-support land. The sleeve is a separate child part;
the body is recessed for it and its assumed 0.5 mm oil cavity. Sleeve start is
now the work seat, so the stop is below the active sleeve and the complete
4/8 mm tooth band remains supported. Collapsed radial bore clearance is 0.010 mm (0.020 mm diametral).
The head retracts to X=110 mm, parks Y=Z=0 and stops B. The robot approaches
from work −X (global −Y, away from the column), transfers 250 mm horizontally
with the cup 35 mm above its seat, then lowers it over the collapsed sleeve.
This clears the maximum 30 mm mandrel protrusion by 5 mm. Unloading reverses
that path after cutting and measured X/Y/Z withdrawal. The transfer distance
belongs to the robot, not the X screw.

[LOADING_CLEARANCE.json](LOADING_CLEARANCE.json) records the exact-solid sizing.
For cup outside radius R, the assumed front palm occupies x=[−R−80,−R−25],
y=±(R+25), z=40..80 mm above the cup base. Two side fingers occupy
x=[−R−30,15], y=[R−3,R+25] and its opposite, z=30..55 mm. These are assumed
occupied envelopes, including finger engagement; robot reach/wrist design remains
open. The front transfer sweeps them through 250 mm and then 35 mm down.
The head corridor adds 10 mm clearance and encloses all A ±15° using 1°
samples padded by Rmax·0.5°=2.735 mm. B is stopped; Y=Z=0.

| Cup OD, face 20 mm | Front required / rounded | Top required / rounded |
|---|---:|---:|
| 25 mm | 61.37 / 65 mm | 57.66 / 60 mm |
| 140 mm | 108.54 / 110 mm | 142.33 / 145 mm |

Front loading is selected. Top loading remains an evaluated alternative with
R+25 mm gripper reach and 120 mm vertical lift; it needs longer mechanics.
The chosen HGR25 rails are 420 mm, including 84 mm blocks, 20 mm rail end
margins and 5 mm overtravel. The X screw is 260 mm, including journal lengths,
nut/flange length, support/carrier clearance and 10 mm end allowance. Its fixed
motor ends 610 mm behind the work datum; 20 mm rear clearance sets the base
rear at 630 mm (front −300 mm), hence 930 mm base length. The top alternative
would need 460 mm rails, 290 mm screw and 960 mm base. See LAYOUT_REPORT.json
for the exact loading and cutting audits.

The app handling timeline is explicitly presentational: no gripper, pressure
solver or loose-body collision integration. The cup is held in the fixture's
simulation model, and the handling view offsets its displayed mesh while the
spindle is stopped. The cut is gated until seating/clamping; the release/lift
is gated until measured X reaches loading clearance after spindle stop. Blank,
Clamped and Finished views preserve native cut stock and authored assembly state.
Sleeve motion shows the nominal radial fit take-up without exaggeration; it
must not be used to qualify pressure, strain, clamping distortion or support stiffness.


### Handling display performance

The cup is a cached child mesh moved by a scene transform; handling does not
rebuild vertices, triangle filters or GPU geometry. Material-removal previews
still replace geometry when the cut changes it. Child transforms are composed
with the live C pose for rendering and picking, while paused views use the
saved assembly pose. The sleeve uses one cached mesh with an illustrative
radial affine scale that matches its outside-radius take-up; this is not a
constant-wall-thickness strain/contact model.

Realtime load/unload trajectories use active playback wall time (four seconds
each), so slow physics does not stretch a display-only transfer. Pause freezes
that time. Explicit deterministic steps advance handling by the fixed timestep.
Cutting and head withdrawal retain the physical clock and 4 kHz controller;
illustrative transfers remain excluded from the measured machining cycle.


## Central X-drive casting load path

The former lightening cores left the central X-drive insert unsupported: the
entire required 50 mm backing slab (1,437,500 mm³) was void. Contact between the
steel frame and its insert did not prove that the insert itself had casting
support. The existing rail checks missed this separate mount.

The corrected channel has a 60 mm cast floor/cap, 300 mm wide, at casting
Z=660..720 mm. The insert recess leaves 50 mm of continuous casting immediately
below its full footprint. Two 35 mm longitudinal webs at transverse Y=±112.5 mm
carry the floor down to the sole. Two 30 mm transverse ribs at work X=202.5 and
450 mm connect the drive structure into both existing rail beams. These supports
run from the table-side structure to the rear wall; hollow bays remain between
webs. The motor body is flange-mounted, so its overhang behind the flange is
intentional; its flange/frame now has a continuous casting load path.

The updated casting is 835.58 kg, within the assumed 600–900 kg range; the whole
machine is 1132.83 kg. X's casting-to-moving-mass ratio is 6.18. Local drive-floor
bending, screw thrust, motor torque and structural/modal adequacy still need FEA.
The simplified global beam bound remains 0.00309 µm over X travel and does not
qualify this local load path.

See the dimensioned transverse [section below the motor](DRIVE_SUPPORT_SECTION.svg),
the [flange section](DRIVE_SUPPORT_FLANGE.svg), the [previous section](DRIVE_SUPPORT_BEFORE.svg),
and [exact-volume evidence](DRIVE_SUPPORT_REPORT.json). Normal assembly checks now
verify both the backing slab and supporting ribs. Collision boxes are compacted
by exact repartition, preserving every cavity and the CAD volume; no geometry or
metrology tolerance is relaxed.


## Offset Z drive

The ground SFU1605 screw lies 19 mm behind the Z guides' radial plane (screw X=176 mm after the 25 mm BK12 shaft height
in front of the 40 mm ground pads at the column's seat) but is no longer at Y=0: the eRob housing of the A axis (Ø110, rear boss and
connectors, 88 mm deep) stands behind the Z saddle there. The offset is derived, not typed (`HobberZOffset`): the least |Y| at which the
BK12 mount, the BF12 support, the ball nut and its block keep 2 mm from the housing swept over the Z travel (BF12 governs: 81 mm), on the
side of the Z moving mass's centre of mass (+Y: the head and its spindle seat lie at Y 70..222), within the casting's limit (the drive seat insert
must keep 10 mm of casting between it and the lifting lands: |Y| + half the insert width at most 120 mm; the seat holes need 37.5 mm
each side, so at most 82.5 mm). Fails with the numbers if no Y satisfies both. The floating pad is cut by the housing's swept
envelope grown by the 2 mm gap; the column's head recess on that side, the insert width and position follow from the same offset
(`FoundationLayout`), and the Z scale stands on the other channel wall. `HobberFrameChecks.zOffsetMoments` prints the yaw moment the offset
thrust and the weight put on the four Z blocks against the block row's rating. See [FRAME_PLAN.md](FRAME_PLAN.md).
Head frames, guide-to-hob offsets and Z=-10..190 mm travel stay unchanged.

The 420 mm screw points down from work Z=730 to 310 mm. Its fixed bearing is
at the top (691.5 mm), floating bearing below (317 mm, 7 mm from the screw tip: BF12 6000ZZ bearing 8 mm wide against the shoulder of the standard 11 mm floating end, ring on the tip side; `HzMotionDrawingScrew.BF12_SUPPORT_CENTRE`), and 750 W motor above
the column (mount face Z=765 mm, body top Z=910 mm). All three axes use the standard 51 mm BK12 screw end (Ø10 x 15 journal, M12x1 x 12,
Ø12 x 24 seat to the shoulder): the 25 mm BK12 housing seats on the shoulder, so
its centre is 38.5 mm from the screw end (`HzMotionDrawingScrew.BK12_SUPPORT_CENTRE`)
and the race starts at 51 mm. The Z bearing pad is centred on that bearing and
the Z clamp coupling is 40 mm long, so its screw half clears the housing. The head hangs below the
fixed bearing, loading that screw segment in tension. The normal positive-Z
command still raises the head; an actual-pose/right-hand-thread regression
verifies the transmission sign.

The nut is a steel nut block bolted to the Z saddle's back face. The BK12 mount (M60D-BK12-C, 400 W braked servo) and the BF12 stand on
one ground steel bar pad (40 mm) under both, against one ground steel insert in the column's drive seat, whose width and position are derived
from the offset. The rear casting spine keeps its full backing behind that insert (`z-drive-checks` asserts 125 mm). The Z scale
stands on the channel wall opposite the drive. The motor platform's 40 × 40 mm R3 coupling window is no longer part of the model;
the coupling clears the pad by at least 2 mm. Motor mounting/centering and nut fastener details remain assumed pending the
selected hardware drawings; this is a layout, not a manufacturing release.

Checks cover 5 mm of reserve beyond both soft stops: minimum nut/race margin
39.5 mm and minimum moving saddle/drive clearance 11 mm. The reinforced column
adds mass, included in the updated X/Z moving masses and thrust margins above.
The work height remains 1000 mm. Local spine/adapter stiffness and a Z holding
brake specification remain open; the simulation models powered holding.

See [Z_DRIVE_SECTION.svg](Z_DRIVE_SECTION.svg) and [Z_DRIVE_REPORT.json](Z_DRIVE_REPORT.json).

## Chip protection and service

Block wipers: an end scraper at each end of every guide block (`WIPER-HG25-SCRAPER` on the four X blocks, `WIPER-HG15-SCRAPER` on the four Z and four Y blocks, 24 in all;
`WayWiperCatalog`, added lengths assumed; BOM lines only, the block models do not carry them).

Way covers (`machinekit.cover`, box covers of the lamella bellows row, 14 mm end walls and carriers, 2 mm gaps, opening 40 x 50 round the bare screw) measured on the
posed machine by `HobberFrameChecks.coverFit`: X front (BF12 to the nut, stroke 64.8 mm) 28.7 mm free, 22 mm needed, flange 106 mm of the 110 mm channel: fits, but its wall
has no host (the nut groove of the pad bar starts at x 205, 2 mm behind the BF12, the insert is 100 wide, the casting wall in front is a 12.5 mm strip); X rear (nut to BK12) -9 mm free,
31 mm short; Z bottom 21.5 mm and Z top 23 mm free against 38 mm (4 folds for 200 mm), 16.5 and 15 mm short. Y has no room for a skin: the 28 mm between the saddle's plate and the
swivel plate holds the rails, blocks and screw. The geometry that would make them fit: 31 mm more screw behind the nut's far stop (the bed 40 mm longer), 16.5 and 15 mm more at the two Z
ends (a 20 mm taller column), and a purpose-made wall for the X front (an insert extended 14 mm forward with the channel front, or the pad bar milled with an upstand). The channel floor
is the ground drive-seat insert over 100 of 110 mm, so no casting fall or drain was cut; gutters need a wider channel (at least 150 mm) and the deck ribs' inner faces, now on the channel walls, would move.

Service removals (declared, `FlexsplineHobber.serviceRemovals`, swept by `ServiceChecks`): the X, Y, Z and B servos come off along their own axes (shaft out of the coupling, 45 mm);
the B belt slides off along its pulleys (40 mm); the B cartridge with the hob and arbor leaves forward out of its split clamp (240 mm, X at retract, belt, outboard carrier/support/bearing and Y reading head off); the X screw and nut leave
upward after 25 mm rearward, once the motor, the BK12, the BF12's ring and the carriage stack are off. Blocked (reported by `HobberFrameChecks.service`, `serviceTries`): the X screw
rearward with the nut (the nut's flange is on the block's front face), with the block (the pad bar's full-height rear end is 5 mm above the block's lower end), and the eRob out of the Z
saddle rearward (Z pad bar after 4 mm, column casting after 38 mm; about 90 mm needed) or forward with the saddle (the nut block hangs on the nut).
