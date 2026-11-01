# GE-class moving-column hobber

This is a pure vertical hobber. The direct-drive C table is fixed; X moves the
column towards the table on twin bed guides, Z moves the head on the column,
A swivels directly on the Z saddle, and Y shifts the hob on twin rails.
Turning belongs to the separate lathe plan. This checked design is not a released
manufacturing drawing set: stiffness, guide preload, fits and drive friction
remain assumed inputs.

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
a 25 mm deep swivel housing on its front face. A 180 × 150 × 16 mm crossed-roller
bearing surrounds an assumed 150 × 80 × 20 mm torque motor; a 200 × 180 × 10 mm
clamp shares the outer radial bay. The rotating output and vertical swivel plate
carry twin 220 mm Y rails on their front face, with four blocks and 120 mm
vertical separation. The spindle and floating outboard bracket share the vertical
Y saddle. An open C bracket supports the rear spindle housing; there is no
overhead roof. These compact motor and bearing dimensions and ratings require
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
the chip guard, with 45.5 mm nominal radial clearance. A channel cover/bellows remains
a design provision; the exposed mechanism is an inspection model.

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

The spindle locates the arbor axially. The outboard 15 × 32 × 9 mm radial bearing
is explicitly axially floating (±1 mm assumed allowance); its 22 mm journal
provides the required float margin. It is not a second locating bearing pair.
Fits, retention hardware, lubrication and demonstrated radial stiffness remain
manufacturing-design work.

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

## Feedback and drive requirements (open, 2026-11-01)

Procurement requirements for position feedback. None is selected or commissioned.

| Axis | Feedback | Requirement | Status |
|---|---|---|---|
| X | Sealed absolute linear scale, ML ≈ 140 mm | ±3 µm/m, ≤ 0.1 µm, BiSS-C, slim, air purge | RFQ drafted (Changchun Yuheng JFT series) |
| Z | Same, ML ≈ 240 mm | ±5 µm/m; spur parts, so scale error barely reaches the gear | same RFQ |
| Y | Same, ML ≈ 70 mm | ±5–10 µm/m; same model as Z for spares | same RFQ |
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
   would add drag. The sleeve is sized from the cup bore (Ø58.05, 0.025 mm
   collapsed radial clearance), as `FlexsplineHobber` already builds it; "OD 59"
   in PARTS.md is only the generic recipe default. **Open model issue:** the
   `ExpandingMandrel` default expansion (0.05 mm diametral) equals the nominal
   loading gap, so it cannot also absorb bore tolerance or provide grip; the
   supplier sizes sleeve and expansion for the real bore, and the model should take
   their values. Bore specification for the flexspline drawing (2026-11-01):
   Ø58.05 H5 honed after hardening, roundness and cylindricity 0.003 mm over the
   clamped length, Ra 0.8 (0.4 preferred), only an edge break (≤ 0.1 mm) at the
   open-end bore edge because the cup seats there on the 0.3 mm land (the loading
   lead-in is on the mandrel nose), seating face flatness 0.002 mm and
   perpendicularity to the bore 0.003 mm. The teeth are cut while the cup is
   clamped, so clamping expansion springs back as tooth error: contact pressure
   is limited to what the cutting torque needs (below 5 N·m).
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


### Hob spindle cartridge (revised 2026-11-01: coupled servo)

Load path: hob → removable arbor → taper and face → spindle shaft → opposed
angular-contact pair → stepped steel housing → Y saddle. B is driven by a standard
80-frame servo through a coupling behind the housing, replacing the earlier assumed
hollow direct motor. The model uses the assumed `GENERIC-HV-SERVO-1000W` (1 kW,
3.18 N m, 3000/5000 rpm) in the envelope and hole pattern of the Leadshine
ELM2H-1000LA80F being quoted; the catalogue Leadshine row has no published winding
constants, which the drive check requires. The CAM job's reference speed is 2984 rpm (300 m/min on Ø32), inside
the servo's rated speed. The outboard bearing remains radially supporting and
axially floating. This is an **assumed custom TF20** interface, not an HSK-E25
claim. Dimensions are assumed, in millimetres, along the spindle from the housing
rear datum; the tool face stays at 180, so the cutting geometry and work-frame
kinematics are unchanged.

| Feature | Axial interval / dimensions |
| --- | --- |
| Housing | Steel, Ø80 g6 outside (a standard spindle-clamp size), 0–180, held by the Y saddle's front ring and rear open C bracket |
| Servo | Face at −70 on a bell adapter: Ø80 front flange (−8–0, 4 × M4 to the housing), Ø64 tube with two coupling windows, 90 mm square rear flange (4 × M6 on Ø90), Ø70 pilot recess |
| Coupling | −55 to −10, motor Ø19 to shaft stub Ø20; envelope for a balanced bellows/disc coupling |
| Shaft stub | Ø20, −30–0 |
| Floating rear bearing | 6205 (25 × 52 × 15) at 0–15 in a Ø52 bore |
| Opposed bearing pair | 129–146 and 146–163; Ø25 journals, Ø62 housing seats |
| Outer-ring retention | Housing shoulder at 129; bolted front gland: Ø62 spigot 163–180, Ø80 flange 180–186, 4 × M4 on Ø72 |
| Inner-ring retention | Shaft shoulder at 163; catalogue precision locknut M25 × 1.5 (KM5 size, Ø38 × 7) at 122–129, in a Ø44 housing shoulder bore |
| Spindle nose | Ø40, 170–180; gland bore Ø42 (1 mm labyrinth gap) |
| Taper engagement | 160–180; Ø16 small end, Ø20 large end (1:5) |
| Arbor face flange | Ø40 × 4, starting at unchanged gauge face 180 |
| Arbor retention | Four M4 × 10 socket screws on Ø32 PCD, at 45° to the pins |
| Torque transmission | Two Ø4 axial pins on Ø28 PCD; 2 mm engagement in flange |

The taper locates radially, the face locates axially, and the four flange screws
hold both contacts closed; there is no drawbar, so the arbor changes from the
front. Assembly sequence: slide the matched pair onto the shaft from its rear
against the shaft shoulder and tighten the rear locknut; fit the shaft and pair
from the front against the housing shoulder and bolt the front gland, its spigot
ground at assembly to clamp the outer rings; fit the
rear 6205 over the journal into its floating bore; mount the
bell adapter, coupling and servo; fit the arbor and its four screws.

A bought cartridge to this interface is being quoted as an alternative to making
it (2026-11-01); `supplier-packages/spindle/` generates the drawing package (per-part
STEP, exact sections and the specified tolerances) from this model. Bearing envelopes do not resolve individual races. Thread
helices, fits, preload, screw torque, pin strength, balance, sealing and grinding
tolerances remain manufacturing design requirements; the CAD does not qualify them.
Simultaneous taper/face contact must be specified and verified during manufacture.
The existing runout assumptions are retained, not requalified. The B master
(electronic-gearbox) encoder is the servo's own absolute encoder through the
torsionally stiff coupling: coupling wind-up and encoder error reach the table
divided by the tooth count, so the requirement is the drive's position latency, not
an extra ring encoder. The cartridge is rated 4000 rpm, which limits B below the
servo's 5000 rpm.

Before HB-D11, the original direct-motor cartridge added 1.086 kg: total mass 425.297 kg; moving X/Y/Z masses
132.064 / 14.574 / 41.603 kg. Continuous thrust margins under the existing assumed
demand are X 885.968 N, Y 907.722 N, Z 1531.207 N. Guide-to-hob offsets and bed
footprint were unchanged by that cartridge change; current masses are in the HB-D11 table above.


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
4/8 mm tooth band remains supported. Collapsed radial bore clearance is 0.025 mm.
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


## Centered recessed Z drive

The former Z screw at transverse Y=-250 mm and its high side nut arm are removed.
The ground SFU1605 screw now lies at Y=0 between the Z guides at Y=±145 mm,
19 mm behind their radial plane (screw X=180 mm, guides X=161 mm). Head frames,
guide-to-hob offsets and Z=-10..190 mm travel stay unchanged.

The 420 mm screw points down from work Z=730 to 310 mm. Its fixed bearing is
at the top (682.5 mm), floating bearing below (322.5 mm), and 750 W motor above
the column (mount face Z=765 mm, body top Z=910 mm). The head hangs below the
fixed bearing, loading that screw segment in tension. The normal positive-Z
command still raises the head; an actual-pose/right-hand-thread regression
verifies the transmission sign.

A short 20 mm-thick nut shelf on the Z saddle replaces the long side arm. The
bearing pads and motor platform connect to a vertical backplate, bolted to one
central ground steel casting insert. A retained rear casting spine provides
125 mm of full backing behind that insert, continuous to the column sole. The
linear scale sits beside the screw, at Y=70 mm. The motor platform has a milled
40 × 40 mm R3 coupling service window, with 2.9 mm minimum coupling clearance.
Motor mounting/centering and nut fastener details remain assumed pending the
selected hardware drawings; this is a layout, not a manufacturing release.

Checks cover 5 mm of reserve beyond both soft stops: minimum nut/race margin
39.5 mm and minimum moving saddle/drive clearance 11 mm. The reinforced column
adds mass, included in the updated X/Z moving masses and thrust margins above.
The work height remains 1000 mm. Local spine/adapter stiffness and a Z holding
brake specification remain open; the simulation models powered holding.

See [Z_DRIVE_SECTION.svg](Z_DRIVE_SECTION.svg) and [Z_DRIVE_REPORT.json](Z_DRIVE_REPORT.json).
