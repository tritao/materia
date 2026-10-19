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
three 47 mm rigid levelling feet. Its top is 847 mm above the floor and the tooth
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
| X moving mass | 81.71 kg | 127.98 kg |
| Z moving mass | 106.79 kg | 41.10 kg |
| Y moving mass | 18.01 kg | 14.03 kg |
| Machine mass | 351.61 kg | 1034.40 kg |
| Bed footprint (radial × tangential) | 650 × 750 mm | 930 × 800 mm |
| Installed-pose envelope, including separate rack | 1225.63 × 750 mm | 1510 × 800 mm |
| X continuous thrust margin, same assumed demand basis | 911.15 N | 888.01 N |
| Y continuous thrust margin, same assumed demand basis | 897.29 N | 909.38 N |
| Z continuous thrust margin, same assumed demand basis | 859.36 N | 1536.43 N |

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

A sloped annular table guard drains into the tray below the work. A drilled
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

## Heavy plinth acceptance (HB-D11)

The casting is 748.12 kg; the base assembly is 776.28 kg and the complete machine
is 1034.40 kg. The casting-to-moving-mass ratios are 5.85:1 (X), 53.33:1 (Y)
and 18.20:1 (Z), meeting the assumed ≥5:1 X target. Column casting mass falls
from 37.28 to 31.75 kg; its insert-equipped assembly falls from 55.50 to 49.97 kg.
X moves 127.98 kg including the complete column/head. The three support points
span 690 mm radially and 660 mm across the front. Positive reactions are checked
through X travel against an assumed 15 kN target per foot; no vibration pads
are fitted. Floor, pad and screw compliance remain unqualified.

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
uses its 690 mm span and changes by 0.00240 µm (0.00479 µm at half modulus).
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


### Hob spindle cartridge (2026-10-19)

The former arbor-at-an-empty-gauge-line model is replaced with a cartridge load
path: hob → removable arbor → taper and face → hollow spindle shaft → opposed
angular-contact pair → stepped housing → Y saddle. The outboard bearing remains
radially supporting and axially floating. This is an **assumed custom TF20**
interface, not an HSK-E25 claim. All dimensions below are assumed, in millimetres,
measured along the spindle from the housing rear datum; the tool face stays at
180, so the cutting geometry and work-frame kinematics are unchanged.

| Feature | Axial interval / dimensions |
| --- | --- |
| Motor envelope | 20–120; Ø25 bore on spindle shaft |
| Opposed bearing pair | 129–146 and 146–163; Ø25 journals, Ø62 housing seats |
| Outer-ring retention | Housing shoulder at 129; removable threaded gland at 163–180 |
| Inner-ring retention | Shaft shoulder at 163; shaft locknut at 123–129 |
| Spindle nose | Ø40, 170–180; removable gland clearance bore Ø42 |
| Taper engagement | 160–180; Ø16 small end, Ø20 large end |
| Arbor face flange | Ø40 × 4, starting at unchanged gauge face 180 |
| Torque transmission | Two Ø4 axial pins on Ø28 PCD; 2 mm engagement in flange |
| Drawbar | Ø6 shank through hollow shaft, head on rear face; 12 mm engagement into arbor |

The taper locates radially, face contact locates axially, the drawbar holds both
contacts closed, and the pins transmit torque. Assembly sequence: fit the motor in the rear housing bore; slide the matched
bearing pair onto the shaft from its rear against the shaft shoulder, then fit
and tighten the rear locknut. Fit the shaft/pair from the front against the housing shoulder, slide the Ø42-bore front gland over the Ø40 nose and secure it, then insert
the arbor and tighten the drawbar from the rear. Access to the rear drawbar must
remain available. The BOM lists shaft, drawbar, gland and locknut separately.

Bearing envelopes do not resolve individual races. Thread helices, fits, preload,
drawbar tightening force, pin strength, balance, sealing and grinding tolerances
remain manufacturing design requirements; the CAD does not qualify them. In
particular, simultaneous taper/face contact must be specified and verified during
manufacture. The existing runout assumptions are retained, not requalified.

Before HB-D11, the cartridge added 1.086 kg: total mass 425.297 kg; moving X/Y/Z masses
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
